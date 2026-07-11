# OGC 2026 알고리즘 변수명 명세서 (`myalgorithm.py`)

이 문서는 `baseline_cpsat/myalgorithm.py` 파일 내에서 사용되는 주요 변수명과 그 의미를 정리한 가이드입니다. 

## 1. 전역 / 시스템 관련 변수
* **`prob_info`**: 입력 `.json` 파일에서 파싱된 문제의 모든 정보가 담긴 딕셔너리 (베이 정보, 블록 정보, 가중치 등 포함)
* **`timelimit`**: 알고리즘에 허용된 최대 실행 시간 (초 단위). 대회 서버나 `test_all.py`에서 전달받습니다.
* **`start_time`**: 메인 `algorithm` 함수가 시작된 시각 기록 (`time.time()`)
* **`bays`**: `prob_info`의 데이터를 바탕으로 생성된 `Bay` 객체들의 리스트 (각 베이의 물리적 크기 및 ID 관리)
* **`HAS_ORTOOLS`**: 구글 `ortools` 모듈의 설치 여부를 나타내는 불리언(Boolean) 변수. (현재 버전에서는 기본값으로 사용됨)

## 2. 상태(State) 및 스케줄 관련 변수
* **`state`, `current_state`, `best_state`, `best_global_state`**: 블록들의 배치를 나타내는 딕셔너리.
  - `Key`: `b_id` (블록 ID)
  - `Value`: 튜플 `(bay_id, x, y, o_idx, entry, exit_t)`
* **`fixed_state`**: ALNS 알고리즘의 Repair 단계에서, 제거되지 않고 위치가 고정된(유지된) 블록들의 상태.
* **`bay_placed`**: 각 베이(`bay_id`)별로 배치된 `Block` 객체들의 리스트. (공간 기하학 충돌 검사를 위해 존재)
* **`bay_schedule`**: `bay_placed`에 매핑되는 리스트로, 해당 블록들의 `(entry, exit_t)` 시간 구간 튜플을 저장.

## 3. 블록 속성 및 배치 관련 변수
* **`b_id`**: 블록의 고유 인덱스/ID (정수)
* **`b_info`**: 특정 블록의 요구사항 딕셔너리 (`release_time`, `due_date`, `processing_time`, `shape`, `bay_preferences` 등)
* **`r_time`**: 블록이 공장에 도착해 배치가 가능해지는 가장 빠른 시간 (`release_time`)
* **`d_time` / `due_date`**: 블록 처리를 완료해야 하는 마감 기한
* **`p_time`**: 블록이 베이에 머물러야 하는 작업 소요 시간 (`processing_time`)
* **`bay_id` / `bay_idx`**: 블록이 배치될 베이의 번호
* **`x`, `y`**: 블록이 배치되는 베이 내의 기준점(좌측 하단) 좌표
* **`o_idx`**: 블록의 회전/방향(Orientation)을 나타내는 인덱스 (`shape` 리스트의 인덱스)
* **`entry`**: 크레인을 통해 블록이 베이에 들어오는 시각
* **`exit_t`**: 작업이 완료되어 블록이 베이에서 나가는 시각 (`exit_t = entry + p_time`)

## 4. 충돌(Collision) 및 공간 검사 관련 변수
* **`dummy_blk`**: 블록의 크기(Bounding Box)를 임시로 계산하기 위해 `(x=0, y=0)`에 생성한 블록 객체
* **`new_blk`**: 현재 배치를 시도하고 있는 새로운 후보 `Block` 객체
* **`p_blk`**: 베이에 이미 배치되어 있는 기존 `Block` 객체
* **`p_entry`, `p_exit`**: 기존에 배치된 블록(`p_blk`)의 진입/진출 시간
* **`present_at_entry`, `present_at_exit`**: 새로운 블록이 들어오거나 나갈 때(entry/exit_t), 동일한 시각에 베이 안에 존재하고 있는 기존 블록들의 리스트. (크레인 간섭 검사용)

## 5. 최적화(ALNS & CP-SAT) 탐색 관련 변수
* **`U` / `removed`**: Destroy(파괴) 연산자에 의해 현재 스케줄에서 잠시 제거된 블록 ID들의 집합
* **`num_remove`**: 한 번의 파괴 사이클에서 제거할 블록의 개수 (전체 블록의 약 20%)
* **`cands`**: 수리(Repair) 과정에서 임의의 휴리스틱(코너 앵커링 등)으로 생성해 둔 여러 개의 배치 후보군
* **`conflicts`**: 서로 기하학적/시간적으로 겹쳐서 동시에 선택될 수 없는 두 후보 배치의 쌍 리스트 `(b1, c1_idx, b2, c2_idx)`
* **`model`**: CP-SAT 제약 만족 모델 객체
* **`X`**: CP-SAT에서 쓰이는 결정 변수(Boolean Variable). 
  - `X[(b_id, c_idx)]`가 `True`이면 블록 `b_id`에 대해 `c_idx`번째 후보를 최종 선택함을 뜻함.
* **`alns_weights`**: 파괴 연산자(Random, Workload, Tardiness) 각각이 룰렛 휠에서 선택될 가중치 배열. 각 연산자가 성공적인 결과를 냈을 때 `score`에 의해 업데이트 됨.

## 6. 멀티프로세싱 및 동적 쿨링 (Multiprocessing & Time-based SA) 관련 변수
* **`workers_count`**: 병렬로 실행할 워커(프로세스)의 개수. 현재 시스템 자원을 최대한 활용하기 위해 `4`로 설정되어 있음.
* **`pool`**: 병렬 처리를 관리하는 `multiprocessing.Pool` 객체.
* **`async_results`**: 각 독립된 워커가 비동기적(`apply_async`)으로 실행된 결과(`best_obj`, `best_state`)를 담고 있는 리스트.
* **`seed`**: 각 워커가 다른 탐색 경로를 가지도록 보장하기 위해 부여하는 난수 시드값 (시간 기반으로 부여).
* **`alns_duration`**: ALNS가 수행되도록 할당된 시간(`timelimit`의 90%). 나머지는 후처리와 결과 병합에 쓰임.
* **`T_start` / `T_end`**: 모의 담금질(Simulated Annealing) 기법의 시작 온도와 종료 온도.
* **`elapsed`**: 탐색이 시작된 시점부터 흘러간 누적 시간.
* **`progress`**: 전체 할당된 탐색 시간 대비 현재 진행률 (0.0 ~ 1.0).
* **`T`**: 현재 진행률(`progress`)에 따라 수식 $T(t) = T_{start} \times (\frac{T_{end}}{T_{start}})^{progress}$ 를 통해 동적으로 계산된 현재 온도. 이 온도를 이용해 Metropolis 해 수용 확률이 결정됨.
