// OGC 2026: 2D Bounding Box 기반 MILP 모델
int nbBays = ...;
int nbBlocks = ...;
range Bays = 0..nbBays-1;
range Blocks = 0..nbBlocks-1;

int W[Bays] = ...;
int H[Bays] = ...;

int R[Blocks] = ...;
int D[Blocks] = ...;
int P[Blocks] = ...;
int L[Blocks] = ...;
int S[Blocks][Bays] = ...;

float w1 = ...;
float w2 = ...;
float w3 = ...;

float u[Bays] = ...;
int Smax[Blocks] = ...;

int nbOrients[Blocks] = ...;
int maxOrients = ...;

// 각 블록의 방향(Orientation)별 Bounding Box의 최소/최대 상대 좌표
float minX[Blocks][0..maxOrients-1] = ...;
float maxX[Blocks][0..maxOrients-1] = ...;
float minY[Blocks][0..maxOrients-1] = ...;
float maxY[Blocks][0..maxOrients-1] = ...;

// Big-M 상수
float M_space = 10000;
int M_time = 10000;

// === 결정 변수 (Decision Variables) ===
dvar boolean a[Blocks][Bays];                       // 블록 i가 베이 j에 할당되면 1
dvar boolean o[i in Blocks][0..maxOrients-1];       // 블록 i의 방향이 or이면 1

dvar int+ x[Blocks];                                // 블록 i의 기준점 X 좌표
dvar int+ y[Blocks];                                // 블록 i의 기준점 Y 좌표
dvar int+ entry_time[Blocks];                       // 반입 시간 (ENTRY)
dvar int+ exit_time[Blocks];                        // 반출 시간 (EXIT)
dvar int+ T[Blocks];                                // 지연 시간 (Tardiness)

// 2D 공간/시간 충돌 방지를 위한 보조 변수
dvar boolean left[Blocks][Blocks];
dvar boolean right[Blocks][Blocks];
dvar boolean below[Blocks][Blocks];
dvar boolean above[Blocks][Blocks];
dvar boolean before[Blocks][Blocks];
dvar boolean after[Blocks][Blocks];

dvar float+ Z2;                                     // 작업량 불균형 (Workload imbalance)

// === 중간 수식 (Expressions) ===
// 선택된 방향에 따른 실제 Bounding box 범위 계산
dexpr float b_minX[i in Blocks] = sum(or in 0..nbOrients[i]-1) o[i][or] * minX[i][or];
dexpr float b_maxX[i in Blocks] = sum(or in 0..nbOrients[i]-1) o[i][or] * maxX[i][or];
dexpr float b_minY[i in Blocks] = sum(or in 0..nbOrients[i]-1) o[i][or] * minY[i][or];
dexpr float b_maxY[i in Blocks] = sum(or in 0..nbOrients[i]-1) o[i][or] * maxY[i][or];

// 목적 함수 요소
dexpr float Z1 = sum(i in Blocks) T[i];
dexpr float Z3 = sum(i in Blocks, j in Bays) a[i][j] * (Smax[i] - S[i][j]);

// === 목적 함수 (Objective) ===
minimize w1 * Z1 + w2 * Z2 + w3 * Z3;

// === 제약 조건 (Constraints) ===
subject to {
  // 1. 할당 및 방향 제약
  forall(i in Blocks) {
    sum(j in Bays) a[i][j] == 1;                     // 정확히 하나의 베이에 할당
    sum(or in 0..nbOrients[i]-1) o[i][or] == 1;      // 정확히 하나의 방향 선택
    forall(or in nbOrients[i]..maxOrients-1) o[i][or] == 0; // 유효하지 않은 방향은 0
  }

  // 2. 시간 제약
  forall(i in Blocks) {
    entry_time[i] >= R[i];                           // 반입은 출시일 이후
    exit_time[i] - entry_time[i] >= P[i];            // 반입-반출 사이 시간은 가공시간 이상
    T[i] >= exit_time[i] - D[i];                     // 지연 시간 계산 (T_i >= EXIT - D_i)
  }

  // 3. 베이 이탈 방지 (Bay Containment)
  forall(i in Blocks, j in Bays) {
    // a[i][j]가 1일 때만 제약이 활성화됨 (1-a = 0)
    x[i] + b_minX[i] >= -M_space * (1 - a[i][j]);
    x[i] + b_maxX[i] <= W[j] + M_space * (1 - a[i][j]);
    y[i] + b_minY[i] >= -M_space * (1 - a[i][j]);
    y[i] + b_maxY[i] <= H[j] + M_space * (1 - a[i][j]);
  }

  // 4. 충돌 방지 (Collision-free)
  forall(i1 in Blocks, i2 in Blocks : i1 < i2) {
    // i1과 i2가 시공간적으로 겹치지 않아야 함 (Big-M 사용)
    exit_time[i1] <= entry_time[i2] + M_time * (1 - before[i1][i2]);
    exit_time[i2] <= entry_time[i1] + M_time * (1 - after[i1][i2]);

    x[i1] + b_maxX[i1] <= x[i2] + b_minX[i2] + M_space * (1 - left[i1][i2]);
    x[i2] + b_maxX[i2] <= x[i1] + b_minX[i1] + M_space * (1 - right[i1][i2]);
    y[i1] + b_maxY[i1] <= y[i2] + b_minY[i2] + M_space * (1 - below[i1][i2]);
    y[i2] + b_maxY[i2] <= y[i1] + b_minY[i1] + M_space * (1 - above[i1][i2]);

    // 두 블록이 같은 베이에 있다면, 위 6개 조건 중 최소 1개는 반드시 True(1)여야 함
    forall(j in Bays) {
       before[i1][i2] + after[i1][i2] + left[i1][i2] + right[i1][i2] + below[i1][i2] + above[i1][i2] 
       >= a[i1][j] + a[i2][j] - 1;
    }
  }

  // 5. 작업량 불균형 (Workload Imbalance)
  forall(j1 in Bays, j2 in Bays : j1 != j2) {
    Z2 >= u[j1] * sum(i in Blocks) (a[i][j1] * L[i]) - u[j2] * sum(i in Blocks) (a[i][j2] * L[i]);
  }
}
