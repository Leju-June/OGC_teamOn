# OGC 2026 알고리즘 변수명 명세서 (`myalgorithm.py`)

이 문서는 `baseline_cpsat/myalgorithm.py` 파일 내에서 사용되는 주요 변수명과 그 의미를 정리한 가이드입니다. 

## 1. 전역 / 시스템 관련 변수
* **`prob_info`**: 입력 `.json` 파일에서 파싱된 문제의 모든 정보가 담긴 딕셔너리 (베이 정보, 블록 정보, 가중치 등 포함)
* **`timelimit`**: 알고리즘에 허용된 최대 실행 시간 (초 단위)
* **`start_time`**: 알고리즘(또는 개별 단계)이 시작된 시각 기록 (`time.time()`)
* **`bays`**: `prob_info`의 데이터를 바탕으로 생성된 `Bay` 객체들의 리스트 (각 베이의 물리적 크기 및 ID 관리)
* **`HAS_ORTOOLS`**: 구글 `ortools` 모듈의 설치 여부를 나타내는 불리언(Boolean) 변수. (설치되어 있으면 `True`, 없으면 `False`)

## 2. 상태(State) 및 스케줄 관련 변수
* **`state`, `current_state`, `best_state`**: 블록들의 현재 배치 상태를 나타내는 딕셔너리. 
  - `Key`: `b_id` (블록 ID)
  - `Value`: 튜플 `(bay_id, x, y, o_idx, entry, exit_t)`
* **`fixed_state`**: ALNS 알고리즘의 Repair 단계에서, 제거되지 않고 위치가 고정된(유지된) 블록들의 상태.
* **`bay_placed`**: 각 베이(`bay_id`)별로 배치된 `Block` 객체들의 리스트. (공간 충돌 검사를 위해 존재)
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
* **`dummy_blk`**: 블록의 크기(Bounding Box)를 계산하기 위해 `(x=0, y=0)`에 임시로 생성한 블록 객체
* **`new_blk`**: 현재 배치를 시도하고 있는 새로운 후보 `Block` 객체
* **`p_blk`**: 베이에 이미 배치되어 있는 기존 `Block` 객체
* **`p_entry`, `p_exit`**: 기존에 배치된 블록(`p_blk`)의 진입/진출 시간
* **`present_at_entry`, `present_at_exit`**: 새로운 블록이 들어오거나 나갈 때(entry/exit_t), 동일한 시각에 베이 안에 존재하고 있는 기존 블록들의 리스트. (크레인 Sweep 충돌을 검사할 때 사용됨)

## 5. 최적화(ALNS & CP-SAT) 탐색 관련 변수
* **`U` / `removed`**: Destroy(파괴) 연산자에 의해 현재 스케줄에서 잠시 제거된 블록 ID들의 집합
* **`num_remove`**: 한 번의 사이클에서 제거(Destroy)할 블록의 개수 (전체 블록의 약 15%)
* **`cands`**: 제거된 블록을 다시 배치(Repair)하기 위해 임의로 생성해 둔 여러 개의 배치 후보군 리스트
* **`cand`**: `cands` 리스트 내의 단일 후보 딕셔너리 (`bay, x, y, o_idx, entry, exit` 포함)
* **`conflicts`**: 서로 공간/시간이 겹쳐서 동시에 선택될 수 없는 두 후보 배치의 쌍을 저장한 리스트 `(b1, c1_idx, b2, c2_idx)`
* **`model`**: CP-SAT 제약 만족(Constraint Programming) 모델 객체
* **`X`**: CP-SAT 수리 로직에서 쓰이는 결정 변수(Boolean Variable). 
  - `X[(b_id, c_idx)]`가 `True`이면 블록 `b_id`에 대해 `c_idx`번째 후보 배치를 최종 선택한다는 의미입니다.
* **`solver`**: 구성된 제약 모델(`model`)을 풀어내는 엔진 객체
* **`best_tardiness`, `best_obj`**: 여태까지 발견한 가장 적은 지연 시간 또는 가장 낮은 목적함수 점수 (값이 낮을수록 좋음)
