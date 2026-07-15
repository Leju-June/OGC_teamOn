# OGC 2026 baseline_gurobi Variables

이 문서는 알고리즘을 구성하는 주요 함수와 최적화 변수들의 역할을 설명함.

## 주요 함수
- **`generate_candidates`**: 하나의 블록에 대해 도면 내 배치 가능한 여러 유효 후보군(시간적 오프셋 포함)을 탐색하여 반환함.
- **`spatiotemporal_shaw_removal`**: ALNS의 파괴(Destroy) 단계에서 임의의 기준 블록을 선택하고 시공간적 밀접도가 높은 관련 블록들을 찾아 함께 제거함.
- **`repair_gurobi`**: 파괴된 블록들을 Gurobi 솔버를 활용하여 재배치하며, 정수/이진 변수를 혼합한 수리 모델을 구성하여 최적화함.

## Gurobi 최적화 변수
- **`E_i`** (`vtype=GRB.INTEGER`): 블록 $i$의 진입 시간(Entry time)을 나타내는 정수 변수. $E_i \ge r_i$ 제약을 가짐.
- **`Ex_i`** (`vtype=GRB.INTEGER`): 블록 $i$의 진출 시간(Exit time)을 나타내는 정수 변수. $Ex_i = E_i + p_i$ 로 정의됨.
- **`X_{i,c}`** (`vtype=GRB.BINARY`): 블록 $i$가 공간적 후보군 $c$를 선택했는지 여부를 나타내는 이진 변수. 선택 시 1, 미선택 시 0. $\sum_c X_{i,c} = 1$.
- **`y_order_{b1}_{b2}`** (`vtype=GRB.BINARY`): 블록 $b1$과 $b2$의 후보군 간에 공간적 충돌이 있을 때, 어떤 블록이 먼저 진입/진출할지를 결정하는 이진 변수. (Big-M 제약과 결합하여 시간적 비겹침을 보장함)
- **`y_fixed`** (`vtype=GRB.BINARY`): 복구되지 않고 고정(Fixed)되어 있는 기존 블록과 새로 배치될 블록 간의 순서를 결정하는 보조 이진 변수.
- **`Tard_i`** (`vtype=GRB.CONTINUOUS`): 블록 $i$의 납기 지연 페널티(Tardiness). $Ex_i - d_i$ 보다 크거나 같은 비음수 변수로 설정됨.
