import time
import random
import math
from collections import defaultdict
from ortools.sat.python import cp_model

try:
    from utils import Bay, Block, check_entry, check_exit, check_collisions, _bounding_box, _anchor_verts, _resolve_layers
except ImportError:
    try:
        from baseline.utils import Bay, Block, check_entry, check_exit, check_collisions, _bounding_box, _anchor_verts, _resolve_layers
    except ImportError:
        from baseline_cpsat.utils import Bay, Block, check_entry, check_exit, check_collisions, _bounding_box, _anchor_verts, _resolve_layers

from shapely.geometry import Polygon as ShapelyPolygon
from shapely import affinity

def _time_overlaps(a_entry, a_exit, b_entry, b_exit):
    return a_entry < b_exit and b_entry < a_exit

def _empty_bay_entry(schedule_in_bay, r_time, proc):
    entry = int(r_time)
    changed = True
    while changed:
        changed = False
        exit_t = entry + proc
        for a, e in schedule_in_bay:
            if _time_overlaps(entry, exit_t, a, e):
                entry = max(entry, e)
                changed = True
    return entry

def check_obstruction(new_blk, entry_time, exit_time, bay, bay_placed, bay_schedule):
    for p_blk, (p_entry, p_exit) in zip(bay_placed, bay_schedule):
        if entry_time <= p_entry < exit_time:
            if check_entry(bay, [new_blk], p_blk, fast=True):
                return True
        if entry_time < p_exit <= exit_time:
            if check_exit(bay, [new_blk], p_blk, fast=True):
                return True
    return False

def find_latest_slot(new_blk, bay, bay_placed, bay_schedule, r_time, p_time, due_date):
    exit_t = due_date
    entry = exit_t - p_time
    while entry >= r_time:
        has_collision = False
        for p_blk, (p_entry, p_exit) in zip(bay_placed, bay_schedule):
            if _time_overlaps(entry, exit_t, p_entry, p_exit):
                if check_collisions(bay, [new_blk, p_blk]):
                    has_collision = True
                    break
                    
        if not has_collision:
            present_at_entry = [b for b, (a, e) in zip(bay_placed, bay_schedule) if a <= entry < e]
            if not check_entry(bay, present_at_entry, new_blk, fast=True):
                present_at_exit = [new_blk] + [b for b, (a, e) in zip(bay_placed, bay_schedule) if a <= exit_t < e]
                if not check_exit(bay, present_at_exit, new_blk, fast=True):
                    if not check_obstruction(new_blk, entry, exit_t, bay, bay_placed, bay_schedule):
                        return entry, exit_t
        entry -= 1
        exit_t -= 1
    return None, None

def find_earliest_tardy_slot(new_blk, bay, bay_placed, bay_schedule, r_time, p_time, due_date):
    entry = max(r_time, due_date - p_time + 1)
    limit = max([e for a, e in bay_schedule] + [0]) + p_time if bay_schedule else r_time + p_time
    limit = max(entry + 100, limit + 50)
    
    while entry <= limit:
        exit_t = entry + p_time
        has_collision = False
        for p_blk, (p_entry, p_exit) in zip(bay_placed, bay_schedule):
            if _time_overlaps(entry, exit_t, p_entry, p_exit):
                if check_collisions(bay, [new_blk, p_blk]):
                    has_collision = True
                    break
                    
        if not has_collision:
            present_at_entry = [b for b, (a, e) in zip(bay_placed, bay_schedule) if a <= entry < e]
            if not check_entry(bay, present_at_entry, new_blk, fast=True):
                present_at_exit = [new_blk] + [b for b, (a, e) in zip(bay_placed, bay_schedule) if a <= exit_t < e]
                if not check_exit(bay, present_at_exit, new_blk, fast=True):
                    if not check_obstruction(new_blk, entry, exit_t, bay, bay_placed, bay_schedule):
                        return entry, exit_t
        entry += 1
    return None, None

def format_solution(state):
    ops_by_time = defaultdict(list)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        ops_by_time[exit_t].append({
            "type": "EXIT",
            "block_id": b_id,
            "bay_id": bay_id
        })
        ops_by_time[entry].append({
            "type": "ENTRY",
            "block_id": b_id,
            "bay_id": bay_id,
            "x": x,
            "y": y,
            "orient_idx": o_idx
        })
        
    solution = {"operations": {}}
    for t_val in sorted(ops_by_time.keys()):
        # EXIT first, then ENTRY
        exits = [op for op in ops_by_time[t_val] if op["type"] == "EXIT"]
        entries = [op for op in ops_by_time[t_val] if op["type"] == "ENTRY"]
        solution["operations"][str(t_val)] = exits + entries
    return solution

def compute_objective_val(prob_info, bays, state):
    w = prob_info['weights']
    w1, w2, w3 = w['w1'], w['w2'], w['w3']
    total_area = sum(b.width * b.height for b in bays)
    avg_area = total_area / len(bays) if bays else 1.0
    u_factors = [avg_area / (b.width * b.height) for b in bays]
    
    obj1 = 0
    obj3 = 0
    bay_workloads = [0] * len(bays)
    
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        b_info = prob_info['blocks'][b_id]
        obj1 += max(0, exit_t - b_info['due_date'])
        obj3 += max(b_info['bay_preferences']) - b_info['bay_preferences'][bay_id]
        
        layers = _resolve_layers(b_info['shape'][o_idx]['layers'])
        if layers:
            bay_workloads[bay_id] += b_info.get('workload', 0)
            
    obj2 = 0
    for i in range(len(bays)):
        for j in range(i+1, len(bays)):
            diff = abs(u_factors[i] * bay_workloads[i] - u_factors[j] * bay_workloads[j])
            if diff > obj2:
                obj2 = diff
                
    return w1 * obj1 + w2 * obj2 + w3 * obj3

def initialization(prob_info, bays, timelimit, start_time):
    blocks_info = prob_info['blocks']
    sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['due_date'], reverse=True)
    
    state = {}
    bay_placed = {bay.id: [] for bay in bays}
    bay_schedule = {bay.id: [] for bay in bays}
    
    for idx, b_id in enumerate(sorted_bids):
        b_info = blocks_info[b_id]
        r_time = b_info['release_time']
        d_time = b_info['due_date']
        p_time = b_info['processing_time']
        
        best_cand = None
        best_tardiness = float('inf')
        
        prefs = b_info['bay_preferences']
        bay_order = sorted(range(len(bays)), key=lambda idx: prefs[idx], reverse=True)
        
        # If running out of init time, switch to fast fallback
        fast_fallback = (time.time() - start_time) > (timelimit * 0.2)
        
        if not fast_fallback:
            for bay_idx in bay_order:
                bay = bays[bay_idx]
                for o_idx in range(len(b_info['shape'])):
                    dummy_blk = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
                    lx0, ly0, lx1, ly1 = dummy_blk.bounding_rect()
                    min_x = max(0, int(math.ceil(-lx0)))
                    min_y = max(0, int(math.ceil(-ly0)))
                    max_x = int(math.floor(bay.width - lx1))
                    max_y = int(math.floor(bay.height - ly1))
                    
                    if min_x > max_x or min_y > max_y:
                        continue
                    
                    bw_approx = lx1 - lx0
                    bh_approx = ly1 - ly0
                    step = max(3, int(min(bw_approx, bh_approx)) // 2)
                    
                    for y in range(min_y, max_y + 1, step):
                        for x in range(min_x, max_x + 1, step):
                            new_blk = Block(block_id=b_id, block_data=b_info, x=x, y=y, orient_idx=o_idx)
                            
                            entry, exit_t = find_latest_slot(new_blk, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time)
                            if entry is None:
                                entry, exit_t = find_earliest_tardy_slot(new_blk, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time)
                                
                            if entry is not None:
                                tard = max(0, exit_t - d_time)
                                if tard < best_tardiness:
                                    best_tardiness = tard
                                    best_cand = (bay.id, x, y, o_idx, entry, exit_t)
                                if best_tardiness == 0:
                                    break
                        if best_tardiness == 0: break
                    if best_tardiness == 0: break
                if best_tardiness == 0: break
            
        if best_cand:
            bay_id, x, y, o_idx, entry, exit_t = best_cand
            new_blk = Block(block_id=b_id, block_data=b_info, x=x, y=y, orient_idx=o_idx)
            state[b_id] = best_cand
            bay_placed[bay_id].append(new_blk)
            bay_schedule[bay_id].append((entry, exit_t))
        else:
            valid_found = False
            for bay_idx in bay_order:
                bay = bays[bay_idx]
                for o_idx in range(len(b_info['shape'])):
                    dummy_blk = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
                    lx0, ly0, lx1, ly1 = dummy_blk.bounding_rect()
                    min_x = max(0, int(math.ceil(-lx0)))
                    min_y = max(0, int(math.ceil(-ly0)))
                    max_x = int(math.floor(bay.width - lx1))
                    max_y = int(math.floor(bay.height - ly1))
                    
                    if min_x <= max_x and min_y <= max_y:
                        bay_id = bay.id
                        valid_o_idx = o_idx
                        valid_x = min_x
                        valid_y = min_y
                        valid_found = True
                        break
                if valid_found: break
                
            if not valid_found:
                bay_id = bay_order[0]
                bay = bays[bay_id]
                valid_o_idx = 0
                valid_x, valid_y = 0, 0
                
            entry = _empty_bay_entry(bay_schedule[bay_id], r_time, p_time)
            exit_t = entry + p_time
            new_blk = Block(block_id=b_id, block_data=b_info, x=valid_x, y=valid_y, orient_idx=valid_o_idx)
            state[b_id] = (bay_id, valid_x, valid_y, valid_o_idx, entry, exit_t)
            bay_placed[bay_id].append(new_blk)
            bay_schedule[bay_id].append((entry, exit_t))
            
    return state

def destroy_random(state, num_remove):
    return set(random.sample(list(state.keys()), min(num_remove, len(state))))

def destroy_workload(state, prob_info, bays, num_remove):
    # Find bay with highest workload penalty
    bay_workloads = [0] * len(bays)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        b_info = prob_info['blocks'][b_id]
        bay_workloads[bay_id] += b_info.get('workload', 0)
    
    total_area = sum(b.width * b.height for b in bays)
    avg_area = total_area / len(bays) if bays else 1.0
    u_factors = [avg_area / (b.width * b.height) for b in bays]
    
    penalties = [bay_workloads[i] * u_factors[i] for i in range(len(bays))]
    worst_bay = penalties.index(max(penalties))
    
    candidates = [b_id for b_id, v in state.items() if v[0] == worst_bay]
    if not candidates:
        return destroy_random(state, num_remove)
        
    return set(random.sample(candidates, min(num_remove, len(candidates))))

def destroy_tardiness(state, prob_info, num_remove):
    # 2. 지연 파괴 (Tardiness Destroy) 연산자
    tardiness_scores = []
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        tard = max(0, exit_t - prob_info['blocks'][b_id]['due_date'])
        tardiness_scores.append((b_id, tard))
        
    tardiness_scores.sort(key=lambda item: item[1], reverse=True)
    
    candidates = [b_id for b_id, tard in tardiness_scores if tard > 0]
    
    if not candidates:
        return destroy_random(state, num_remove)
        
    selected = set()
    for b_id, tard in tardiness_scores:
        if tard > 0:
            selected.add(b_id)
            if len(selected) >= num_remove:
                break
    
    if len(selected) < num_remove:
        remaining = set(state.keys()) - selected
        selected.update(random.sample(list(remaining), min(num_remove - len(selected), len(remaining))))
        
    return selected

def is_conflict(c1, c2, prob_info, bays):
    if c1['bay'] != c2['bay']: return False
    if not _time_overlaps(c1['entry'], c1['exit'], c2['entry'], c2['exit']):
        return False
        
    bay = bays[c1['bay']]
    blk1 = Block(block_id=c1['id'], block_data=prob_info['blocks'][c1['id']], x=c1['x'], y=c1['y'], orient_idx=c1['o_idx'])
    blk2 = Block(block_id=c2['id'], block_data=prob_info['blocks'][c2['id']], x=c2['x'], y=c2['y'], orient_idx=c2['o_idx'])
    
    if check_collisions(bay, [blk1, blk2]): return True
    
    if c2['entry'] <= c1['entry'] < c2['exit']:
        if check_entry(bay, [blk2], blk1, fast=True): return True
    if c1['entry'] <= c2['entry'] < c1['exit']:
        if check_entry(bay, [blk1], blk2, fast=True): return True
        
    if c2['entry'] < c1['exit'] <= c2['exit']:
        if check_exit(bay, [blk2], blk1, fast=True): return True
    if c1['entry'] < c2['exit'] <= c1['exit']:
        if check_exit(bay, [blk1], blk2, fast=True): return True
        
    return False

def generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, num=10):
    cands = []
    r_time = b_info['release_time']
    d_time = b_info['due_date']
    p_time = b_info['processing_time']
    
    attempts = 0
    prefs = b_info['bay_preferences']
    weights = [p + 0.1 for p in prefs]
    
    while len(cands) < num and attempts < 50:
        attempts += 1
        bay_idx = random.choices(range(len(bays)), weights=weights)[0]
        bay = bays[bay_idx]
        o_idx = random.randint(0, len(b_info['shape'])-1)
        
        dummy_blk = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
        lx0, ly0, lx1, ly1 = dummy_blk.bounding_rect()
        min_x = max(0, int(math.ceil(-lx0)))
        min_y = max(0, int(math.ceil(-ly0)))
        max_x = int(math.floor(bay.width - lx1))
        max_y = int(math.floor(bay.height - ly1))
        
        if min_x > max_x or min_y > max_y:
            continue
        
        # 2. 기하학적 코너 앵커링 휴리스틱 (Corner Anchoring)
        corners = [
            (min_x, min_y),
            (max_x, min_y),
            (min_x, max_y),
            (max_x, max_y)
        ]
        
        if attempts <= 4:
            x, y = corners[(attempts - 1) % 4]
        else:
            x = random.randint(min_x, max_x)
            y = random.randint(min_y, max_y)
        
        new_blk = Block(block_id=b_id, block_data=b_info, x=x, y=y, orient_idx=o_idx)
        
        entry, exit_t = find_latest_slot(new_blk, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time)
        if entry is None:
            entry, exit_t = find_earliest_tardy_slot(new_blk, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time)
            
        if entry is not None:
            cands.append({'id': b_id, 'bay': bay.id, 'x': x, 'y': y, 'o_idx': o_idx, 'entry': entry, 'exit': exit_t})
            
    return cands

def repair_cpsat(U, fixed_state, prob_info, bays, time_limit):
    bay_placed = {bay.id: [] for bay in bays}
    bay_schedule = {bay.id: [] for bay in bays}
    bay_workloads = [0] * len(bays)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in fixed_state.items():
        blk = Block(block_id=b_id, block_data=prob_info['blocks'][b_id], x=x, y=y, orient_idx=o_idx)
        bay_placed[bay_id].append(blk)
        bay_schedule[bay_id].append((entry, exit_t))
        bay_workloads[bay_id] += prob_info['blocks'][b_id].get('workload', 0)
        
    total_area = sum(b.width * b.height for b in bays)
    avg_area = total_area / len(bays) if bays else 1.0
    u_factors = [avg_area / (b.width * b.height) for b in bays]
        
    candidates_by_block = {}
    for b_id in U:
        b_info = prob_info['blocks'][b_id]
        cands = generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, num=15)
        if not cands: return None
        candidates_by_block[b_id] = cands
        
    conflicts = []
    U_list = list(U)
    for i in range(len(U_list)):
        b1 = U_list[i]
        for j in range(i+1, len(U_list)):
            b2 = U_list[j]
            if b1 not in candidates_by_block or b2 not in candidates_by_block: continue
            for c1_idx, c1 in enumerate(candidates_by_block[b1]):
                for c2_idx, c2 in enumerate(candidates_by_block[b2]):
                    if is_conflict(c1, c2, prob_info, bays):
                        conflicts.append((b1, c1_idx, b2, c2_idx))
                        
    model = cp_model.CpModel()
    X = {}
    for b_id, cands in candidates_by_block.items():
        for c_idx in range(len(cands)):
            X[(b_id, c_idx)] = model.NewBoolVar(f'X_{b_id}_{c_idx}')
        model.AddExactlyOne([X[(b_id, c_idx)] for c_idx in range(len(cands))])
        
    for b1, c1_idx, b2, c2_idx in conflicts:
        model.AddImplication(X[(b1, c1_idx)], X[(b2, c2_idx)].Not())
        
    w = prob_info['weights']
    w1, w2, w3 = w['w1'], w['w2'], w['w3']
    obj_vars = []
    for b_id, cands in candidates_by_block.items():
        for c_idx, cand in enumerate(cands):
            tard = max(0, cand['exit'] - prob_info['blocks'][b_id]['due_date'])
            pref = max(prob_info['blocks'][b_id]['bay_preferences']) - prob_info['blocks'][b_id]['bay_preferences'][cand['bay']]
            
            # 4. CP-SAT 목적함수에 Obj2(Workload 불균형) Proxy 점수 반영
            bay_id = cand['bay']
            proxy_obj2_penalty = u_factors[bay_id] * bay_workloads[bay_id]
            
            score = int((w1 * tard + w3 * pref + w2 * proxy_obj2_penalty * 0.001) * 100)
            obj_vars.append(X[(b_id, c_idx)] * score)
            
    model.Minimize(sum(obj_vars))
    
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    status = solver.Solve(model)
    
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        new_state = dict(fixed_state)
        for b_id, cands in candidates_by_block.items():
            for c_idx, cand in enumerate(cands):
                if solver.BooleanValue(X[(b_id, c_idx)]):
                    new_state[b_id] = (cand['bay'], cand['x'], cand['y'], cand['o_idx'], cand['entry'], cand['exit'])
        return new_state
    return None

def post_optimize_time(state, prob_info, bays):
    # 4. 후처리 시간 당기기 최적화 (Post-Optimization Local Search)
    new_state = dict(state)
    changed = True
    while changed:
        changed = False
        for b_id in sorted(new_state.keys(), key=lambda k: new_state[k][5]):
            bay_id, x, y, o_idx, entry, exit_t = new_state[b_id]
            b_info = prob_info['blocks'][b_id]
            r_time = b_info['release_time']
            if entry > r_time:
                other_blocks = {}
                for ob_id, ob_data in new_state.items():
                    if ob_id != b_id and ob_data[0] == bay_id:
                        other_blocks[ob_id] = {
                            'id': ob_id, 'bay': ob_data[0], 'x': ob_data[1], 'y': ob_data[2], 'o_idx': ob_data[3], 'entry': ob_data[4], 'exit': ob_data[5]
                        }
                        
                best_entry = entry
                best_exit = exit_t
                curr_entry = entry - 1
                curr_exit = exit_t - 1
                
                while curr_entry >= r_time:
                    cand = {'id': b_id, 'bay': bay_id, 'x': x, 'y': y, 'o_idx': o_idx, 'entry': curr_entry, 'exit': curr_exit}
                    has_conflict = False
                    for ob_id, ob_data in other_blocks.items():
                        if is_conflict(cand, ob_data, prob_info, bays):
                            has_conflict = True
                            break
                    if not has_conflict:
                        best_entry = curr_entry
                        best_exit = curr_exit
                        curr_entry -= 1
                        curr_exit -= 1
                    else:
                        break
                        
                if best_entry < entry:
                    new_state[b_id] = (bay_id, x, y, o_idx, best_entry, best_exit)
                    changed = True
    return new_state

def algorithm(prob_info, timelimit=60):
    start_time = time.time()
    
    bays_info = prob_info['bays']
    bays = [Bay(width=b['width'], height=b['height'], id=i) for i, b in enumerate(bays_info)]
    
    current_state = initialization(prob_info, bays, timelimit, start_time)
    best_state = dict(current_state)
    best_obj = compute_objective_val(prob_info, bays, best_state)
    
    num_blocks = len(prob_info['blocks'])
    num_remove = max(1, int(num_blocks * 0.20))
    
    # 3. Simulated Annealing 시작 온도 설정
    T = 1000.0
    
    # 3. ALNS 가중치 초기화
    alns_weights = [1.0, 1.0, 1.0]
    
    while time.time() - start_time < 55.0:
        # ALNS 룰렛 휠 선택
        operator = random.choices([0, 1, 2], weights=alns_weights)[0]
        if operator == 0:
            removed = destroy_random(current_state, num_remove)
        elif operator == 1:
            removed = destroy_workload(current_state, prob_info, bays, num_remove)
        else:
            removed = destroy_tardiness(current_state, prob_info, num_remove)
            
        fixed_state = {k: v for k, v in current_state.items() if k not in removed}
        
        new_state = repair_cpsat(removed, fixed_state, prob_info, bays, time_limit=3.0)
        
        score = 0
        if new_state:
            new_obj = compute_objective_val(prob_info, bays, new_state)
            if new_obj < best_obj:
                best_state = dict(new_state)
                best_obj = new_obj
                current_state = dict(new_state)
                score = 10
            else:
                # 3. Simulated Annealing 해 수용 확률 적용 (Metropolis)
                current_obj = compute_objective_val(prob_info, bays, current_state)
                if new_obj < current_obj:
                    current_state = dict(new_state)
                    score = 5
                elif random.random() < math.exp((current_obj - new_obj) / T):
                    current_state = dict(new_state)
                    score = 2
                else:
                    current_state = dict(best_state)
            
            # 온도 감쇠 (Cooling schedule)
            T *= 0.95
            
        # ALNS 가중치 업데이트
        alns_weights[operator] = alns_weights[operator] * 0.9 + score * 0.1
                    
    best_state = post_optimize_time(best_state, prob_info, bays)
    return format_solution(best_state)
