OGC 2026 Baseline Simple Algorithm
==================================

이 알고리즘은 OGC 2026 선박 블록 배치 문제를 해결하기 위한 가장 단순한 형태의 베이스라인 알고리즘입니다.

1. 알고리즘 기본 개념
------------------
- 각 블록은 복잡한 다각형 형태 대신, 모든 레이어의 최소/최대 x, y 좌표를 기준으로 가장 바깥쪽 경계를 감싸는 직육면체(Bounding Box) 형태로 가정합니다.
- 이 직육면체 블록들을 2차원 평면(Bay)에 겹치지 않게 배치하는 직관적인 채우기(Packing) 문제로 단순화하여 접근합니다.
- 블록의 방향(Orientation)은 항상 초기에 주어진 0번 방향을 사용합니다.

2. 블록 진입(Entry) 및 배치 규칙
-----------------------------
- 순서 정렬: 전체 대기 블록들을 출시일(release_time)이 빠른 순서대로 정렬합니다. 만약 출시일이 동률일 경우에는 고유 ID(block_id)가 작은 순서대로 우선권을 가집니다.
- 선호 Bay 선택: 각 블록은 무조건 선호도(bay_preferences) 점수가 가장 높은 Bay로만 진입을 시도합니다.
- 배치 위치 : 선호 Bay에 블록을 배치할 때, 배치 가능한 가장 북서쪽 위치를 찾습니다.
  - 이 알고리즘에서 북서쪽의 의미는 좌표계상 Y값을 최대로, X값을 최소로 만드는 위치입니다.
  - 따라서 가장 큰 Y 좌표부터 우선적으로 탐색하고, 그 다음으로 가장 작은 X 좌표부터 탐색하여 빈 공간이 발견되면 즉시 해당 위치에 배치합니다.

3. 시간 흐름 및 반출(Exit) 규칙
----------------------------
- 알고리즘은 가상의 현재 시간(current_time)을 0부터 추적합니다.
- 현재 시간에 들어갈 수 있는 모든 블록을 겹치지 않게 Bay에 채워 넣습니다 (Entry).
- 한 번의 진입(Entry) 라운드가 끝나면, 현재 Bay에 있는 블록들이 모두 작업을 마칠 때까지(즉, entry_time + processing_time이 모두 지날 때까지) 기다립니다.
- Bay 내 모든 블록의 작업이 완료되는 가장 늦은 시간에 다 같이 반출(Exit) 작업을 수행합니다.
- 반출 후, 공간이 빈 Bay에 대해 남은 대기 블록들을 대상으로 다시 진입(Entry) 과정을 반복합니다.
- 모든 블록이 처리될 때까지 이 과정이 반복됩니다.

---

OGC 2026 Baseline Simple Algorithm (English)
============================================

This algorithm is the simplest form of a baseline algorithm for solving the OGC 2026 ship block placement problem.

1. Basic Algorithm Concept
--------------------------
- Instead of complex polygonal shapes, each block is assumed to be a rectangular cuboid (Bounding Box) that wraps the outermost boundaries based on the minimum and maximum x, y coordinates across all layers.
- It simplifies the problem into an intuitive 2D packing problem, placing these rectangular blocks into a 2D plane (Bay) without overlapping.
- The block's orientation always uses the initially given orientation 0.

2. Block Entry and Placement Rules
----------------------------------
- Sorting Order: All waiting blocks are sorted by their release_time in ascending order. If the release_time is the same, priority is given to the smaller unique ID (block_id).
- Preferred Bay Selection: Each block unconditionally attempts to enter only the Bay with the highest preference score (bay_preferences).
- Placement Location: When placing a block in the preferred Bay, it searches for the most North-West position possible.
  - In this algorithm, North-West means the position that maximizes the Y coordinate and minimizes the X coordinate.
  - Therefore, it searches the highest Y coordinate first, and then the smallest X coordinate. If an empty space is found, it is placed at that location immediately.

3. Time Flow and Exit Rules
---------------------------
- The algorithm tracks a virtual current_time starting from 0.
- All blocks that can enter at the current time are packed into the Bay without overlapping (Entry).
- After one Entry round finishes, it waits until all blocks currently in the Bay complete their processing (i.e., until entry_time + processing_time has passed for all of them).
- At the latest completion time when all blocks in the Bay are done, they are all exited together (Exit).
- After the exit, the Entry process is repeated for the remaining waiting blocks into the now-empty Bay.
- This process is repeated until all blocks are processed.
