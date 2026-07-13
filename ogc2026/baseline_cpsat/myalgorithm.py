import time
import random
import math
import multiprocessing
from collections import defaultdict
from ortools.sat.python import cp_model

from shapely.geometry import Polygon as ShapelyPolygon, Point
from shapely.ops import unary_union
from shapely import affinity

try:
    from utils import Bay, Block, check_entry, check_exit, check_collisions
except ImportError:
    try:
        from baseline.utils import Bay, Block, check_entry, check_exit, check_collisions
    except ImportError:
        from baseline_cpsat.utils import Bay, Block, check_entry, check_exit, check_collisions

def _time_overlaps(a_entry, a_exit, b_entry, b_exit):
    # 두 스케줄의 시간적 겹침 여부를 확인
    return a_entry < b_exit and b_entry < a_exit

def _empty_bay_entry(schedule_in_bay, r_time, proc):
    # 해당 도면에서 다른 스케줄과 겹치지 않는 가장 빠른 진입 시간을 계산
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

class RasterCache:
    def __init__(self, prob_info, bays):
        # 블록과 도면의 비트마스킹 연산을 위한 캐시 객체를 초기화
        self.prob_info = prob_info
        self.bays = bays
        self.block_masks = {}
        
        max_w = max((b.width for b in bays), default=0)
        max_h = max((b.height for b in bays), default=0)
        
        self.scale = 1
        if max_w > 200 or max_h > 200:
            self.scale = max(1, int(math.ceil(max(max_w, max_h) / 150.0)))

    def get_block_mask(self, b_id, o_idx):
        # 블록의 다각형 형태를 이진 비트마스크 배열로 변환하여 캐싱 및 반환
        key = (b_id, o_idx)
        if key in self.block_masks:
            return self.block_masks[key]
            
        b_info = self.prob_info['blocks'][b_id]
        layers = b_info['shape'][o_idx]['layers']
        
        polys = [ShapelyPolygon(l) for l in layers if len(l) >= 3]
        if not polys:
            self.block_masks[key] = ([], 0, 0, 0.0, 0.0)
            return self.block_masks[key]
            
        footprint = unary_union(polys)
        lx0, ly0, lx1, ly1 = footprint.bounds
        footprint = affinity.translate(footprint, xoff=-lx0, yoff=-ly0)
        
        gw = int(math.ceil((lx1 - lx0) / self.scale))
        gh = int(math.ceil((ly1 - ly0) / self.scale))
        if gw == 0: gw = 1
        if gh == 0: gh = 1
        
        mask = [0] * gh
        for y in range(gh):
            for x in range(gw):
                cx = x * self.scale + self.scale/2.0
                cy = y * self.scale + self.scale/2.0
                if footprint.intersects(Point(cx, cy)):
                    mask[y] |= (1 << x)
        self.block_masks[key] = (mask, gw, gh, lx0, ly0)
        return self.block_masks[key]

    def build_bay_grid(self, bay_id, bay_placed, bay_schedule, current_entry, current_exit):
        # 특정 시점에 도면에 배치된 모든 블록들의 위치를 2D 비트마스크 그리드로 변환
        bay = self.bays[bay_id]
        gw = int(math.ceil(bay.width / self.scale))
        gh = int(math.ceil(bay.height / self.scale))
        grid = [0] * gh
        
        for p_blk, (p_entry, p_exit) in zip(bay_placed, bay_schedule):
            if _time_overlaps(current_entry, current_exit, p_entry, p_exit):
                b_mask, bw, bh, blx0, bly0 = self.get_block_mask(p_blk.block_id, p_blk.orient_idx)
                gx0 = int(math.floor((p_blk.x + blx0) / self.scale))
                gy0 = int(math.floor((p_blk.y + bly0) / self.scale))
                
                for r in range(bh):
                    grid_y = gy0 + r
                    if 0 <= grid_y < gh:
                        shift = max(0, gx0)
                        if gx0 < 0: # Should not happen usually
                            grid[grid_y] |= (b_mask[r] >> (-gx0))
                        else:
                            grid[grid_y] |= (b_mask[r] << shift)
        return grid, gw, gh

    def find_valid_spots(self, grid, gw, gh, b_mask, bw, bh):
        # 비트 연산을 활용하여 도면 그리드 내에서 블록이 들어갈 수 있는 유효 좌표들을 고속 탐색
        valid = []
        if bh > gh or bw > gw:
            return valid
            
        for y in range(gh - bh + 1):
            for x in range(gw - bw + 1):
                conflict = False
                for r in range(bh):
                    if (grid[y + r] & (b_mask[r] << x)) != 0:
                        conflict = True
                        break
                if not conflict:
                    touch_score = 0
                    if y == 0 or y + bh == gh: touch_score += 1
                    if x == 0 or x + bw == gw: touch_score += 1
                    for r in range(bh):
                        if x > 0 and (grid[y+r] & (1 << (x-1))): touch_score += 1
                        if x + bw < gw and (grid[y+r] & (1 << (x+bw))): touch_score += 1
                    if y > 0 and (grid[y-1] & (b_mask[0] << x)): touch_score += 1
                    if y + bh < gh and (grid[y+bh] & (b_mask[-1] << x)): touch_score += 1
                    valid.append((x * self.scale, y * self.scale, touch_score))
        return valid

def search_placement(b_id, b_info, bay, bay_placed, bay_schedule, r_time, p_time, due_date, o_idx, raster_cache, mode='backward'):
        # 주어진 블록이 특정 도면에 배치될 수 있는 최적의 좌표와 시간을 양방향(정방향/역방향)으로 탐색
    if mode == 'backward':
        entry = max(r_time, due_date - p_time)
        exit_t = entry + p_time
        step = -1
        limit = r_time
    else: # forward
        entry = max(r_time, due_date - p_time + 1)
        exit_t = entry + p_time
        step = 1
        limit = max([e for a, e in bay_schedule] + [0]) + p_time if bay_schedule else r_time + p_time
        limit = max(entry + 100, limit + 50)

    while (entry >= limit if mode == 'backward' else entry <= limit):
        grid, gw, gh = raster_cache.build_bay_grid(bay.id, bay_placed, bay_schedule, entry, exit_t)
        b_mask, bw, bh, blx0, bly0 = raster_cache.get_block_mask(b_id, o_idx)
        spots = raster_cache.find_valid_spots(grid, gw, gh, b_mask, bw, bh)
        spots.sort(key=lambda v: v[2], reverse=True)
        
        for gx, gy, score in spots:
            cx = int(math.ceil(gx - blx0))
            cy = int(math.ceil(gy - bly0))
            new_blk = Block(block_id=b_id, block_data=b_info, x=cx, y=cy, orient_idx=o_idx)
            # Full collision check
            has_col = False
            for p_blk, (p_e, p_ex) in zip(bay_placed, bay_schedule):
                if _time_overlaps(entry, exit_t, p_e, p_ex):
                    if check_collisions(bay, [new_blk, p_blk]):
                        has_col = True; break
            if not has_col:
                p_in = []
                p_out = [new_blk]
                obs_failed = False
                for p_blk, (p_e, p_ex) in zip(bay_placed, bay_schedule):
                    if p_e <= entry < p_ex: p_in.append(p_blk)
                    if p_e <= exit_t < p_ex: p_out.append(p_blk)
                    if entry <= p_e < exit_t and check_entry(bay, [new_blk], p_blk, fast=True):
                        obs_failed = True; break
                    if entry < p_ex <= exit_t and check_exit(bay, [new_blk], p_blk, fast=True):
                        obs_failed = True; break
                
                if not obs_failed and not check_entry(bay, p_in, new_blk, fast=True) and not check_exit(bay, p_out, new_blk, fast=True):
                    return cx, cy, entry, exit_t
        entry += step
        exit_t += step
    return None, None, None, None

def format_solution(state, prob_info, bays):
    # 내부의 딕셔너리 배치 상태를 제출 규격에 맞는 JSON 포맷으로 변환 (동시 퇴출 시 간섭을 방지하기 위한 위상 정렬 포함)
    ops_by_time = defaultdict(list)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        ops_by_time[exit_t].append({"type": "EXIT", "block_id": b_id, "bay_id": bay_id, "x": x, "y": y, "orient_idx": o_idx})
        ops_by_time[entry].append({"type": "ENTRY", "block_id": b_id, "bay_id": bay_id, "x": x, "y": y, "orient_idx": o_idx})
    
    solution = {"operations": {}}
    for t_val in sorted(ops_by_time.keys()):
        exits = [op for op in ops_by_time[t_val] if op["type"] == "EXIT"]
        entries = [op for op in ops_by_time[t_val] if op["type"] == "ENTRY"]
        
        if len(exits) > 1:
            bay_groups = defaultdict(list)
            for op in exits:
                bay_groups[op["bay_id"]].append(op)
            
            sorted_exits = []
            for bay_id, ops in bay_groups.items():
                if len(ops) <= 1:
                    sorted_exits.extend(ops)
                    continue
                
                bay = bays[bay_id]
                blocks_dict = {}
                for op in ops:
                    blocks_dict[op["block_id"]] = Block(block_id=op["block_id"], block_data=prob_info['blocks'][op["block_id"]], x=op["x"], y=op["y"], orient_idx=op["orient_idx"])
                
                adj = {op["block_id"]: [] for op in ops}
                in_degree = {op["block_id"]: 0 for op in ops}
                blks_list = list(blocks_dict.values())
                
                for target_blk in blks_list:
                    obs = check_exit(bay, blks_list, target_blk, fast=False)
                    if obs:
                        for o in obs:
                            A = o.existing_block.block_id
                            B = target_blk.block_id
                            if A in adj:
                                adj[A].append(B)
                                in_degree[B] += 1
                                
                queue = [b_id for b_id in in_degree if in_degree[b_id] == 0]
                topo_order = []
                while queue:
                    curr = queue.pop(0)
                    topo_order.append(curr)
                    for nxt in adj[curr]:
                        in_degree[nxt] -= 1
                        if in_degree[nxt] == 0:
                            queue.append(nxt)
                            
                for b_id in in_degree:
                    if in_degree[b_id] > 0 and b_id not in topo_order:
                        topo_order.append(b_id)
                        
                op_map = {op["block_id"]: op for op in ops}
                for b_id in topo_order:
                    sorted_exits.append(op_map[b_id])
            exits = sorted_exits
            
        for op in exits:
            op.pop("x", None); op.pop("y", None); op.pop("orient_idx", None)
            
        solution["operations"][str(t_val)] = exits + entries
    return solution

def compute_objective_val(prob_info, bays, state):
    # 지연, 불균형, 선호도 페널티를 모두 합산하여 현재 스케줄 상태의 목적 함수 점수를 계산
    w = prob_info['weights']
    w1, w2, w3 = w['w1'], w['w2'], w['w3']
    total_area = sum(b.width * b.height for b in bays)
    avg_area = total_area / len(bays) if bays else 1.0
    u_factors = [avg_area / (b.width * b.height) for b in bays]
    
    obj1 = obj3 = 0
    bay_workloads = [0] * len(bays)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        b_info = prob_info['blocks'][b_id]
        obj1 += max(0, exit_t - b_info['due_date'])
        obj3 += max(b_info['bay_preferences']) - b_info['bay_preferences'][bay_id]
        bay_workloads[bay_id] += b_info.get('workload', 0)
            
    obj2 = 0
    for i in range(len(bays)):
        for j in range(i+1, len(bays)):
            diff = abs(u_factors[i] * bay_workloads[i] - u_factors[j] * bay_workloads[j])
            if diff > obj2: obj2 = diff
    return w1 * obj1 + w2 * obj2 + w3 * obj3

def initialization_strategy(prob_info, bays, timelimit, start_time, sort_strategy, raster_cache):
    # 지정된 정렬 기준에 따라 블록들을 순차적으로 배치하여 초기해 생성
    blocks_info = prob_info['blocks']
    if sort_strategy == 'rev_edd':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['due_date'], reverse=True)
    elif sort_strategy == 'ptime':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['processing_time'], reverse=True)
    elif sort_strategy == 'edd':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['due_date'])
    else: # area
        def get_area(i):
            b_info = blocks_info[i]
            l = b_info['shape'][0]['layers']
            if not l or len(l[0]) < 3: return 0
            return ShapelyPolygon(l[0]).area
        sorted_bids = sorted(range(len(blocks_info)), key=get_area, reverse=True)
        
    state = {}
    bay_placed = {bay.id: [] for bay in bays}
    bay_schedule = {bay.id: [] for bay in bays}
    
    for b_id in sorted_bids:
        b_info = blocks_info[b_id]
        r_time = b_info['release_time']
        d_time = b_info['due_date']
        p_time = b_info['processing_time']
        
        bay_order = sorted(range(len(bays)), key=lambda idx: b_info['bay_preferences'][idx], reverse=True)
        
        best_cand = None
        best_tard = float('inf')
        
        for bay_idx in bay_order:
            bay = bays[bay_idx]
            for o_idx in range(len(b_info['shape'])):
                x, y, entry, exit_t = search_placement(b_id, b_info, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time, o_idx, raster_cache, 'backward')
                if entry is None:
                    x, y, entry, exit_t = search_placement(b_id, b_info, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time, o_idx, raster_cache, 'forward')
                    
                if entry is not None:
                    tard = max(0, exit_t - d_time)
                    if tard < best_tard:
                        best_tard = tard
                        best_cand = (bay.id, x, y, o_idx, entry, exit_t)
                    if best_tard == 0: break
            if best_tard == 0: break
            
        if best_cand:
            bay_id, x, y, o_idx, entry, exit_t = best_cand
            new_blk = Block(block_id=b_id, block_data=b_info, x=x, y=y, orient_idx=o_idx)
            state[b_id] = best_cand
            bay_placed[bay_id].append(new_blk)
            bay_schedule[bay_id].append((entry, exit_t))
        else:
            # Fallback
            bay_id = bay_order[0]
            bay = bays[bay_id]
            entry = _empty_bay_entry(bay_schedule[bay_id], r_time, p_time)
            exit_t = entry + p_time
            _, _, _, blx0, bly0 = raster_cache.get_block_mask(b_id, 0)
            cx = int(math.ceil(-blx0))
            cy = int(math.ceil(-bly0))
            new_blk = Block(block_id=b_id, block_data=b_info, x=cx, y=cy, orient_idx=0)
            state[b_id] = (bay_id, cx, cy, 0, entry, exit_t)
            bay_placed[bay_id].append(new_blk)
            bay_schedule[bay_id].append((entry, exit_t))
            
    return state, compute_objective_val(prob_info, bays, state)

def initialization(prob_info, bays, timelimit, start_time, raster_cache):
    # 여러 정렬 전략을 시도하여 제한시간 내에 도출된 가장 좋은 초기해를 선택
    best_state = None
    best_obj = float('inf')
    
    strategies = ['rev_edd', 'edd', 'area']
    for strat in strategies:
        if time.time() - start_time > timelimit * 0.15: break
        state, obj = initialization_strategy(prob_info, bays, timelimit, start_time, strat, raster_cache)
        if obj < best_obj:
            best_obj = obj
            best_state = state
    return best_state

def destroy_random(state, num_remove):
    # 현재 배치 상태에서 지정된 개수만큼의 블록을 무작위로 선택하여 제거
    return set(random.sample(list(state.keys()), min(num_remove, len(state))))

def destroy_workload(state, prob_info, bays, num_remove):
    # 작업량이 가장 많이 몰린 도면에 배치된 블록들을 집중적으로 선택하여 제거
    bay_workloads = [0] * len(bays)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        bay_workloads[bay_id] += prob_info['blocks'][b_id].get('workload', 0)
    
    u_factors = [1.0 / (b.width * b.height) for b in bays]
    penalties = [bay_workloads[i] * u_factors[i] for i in range(len(bays))]
    worst_bay = penalties.index(max(penalties))
    candidates = [b_id for b_id, v in state.items() if v[0] == worst_bay]
    if not candidates: return destroy_random(state, num_remove)
    return set(random.sample(candidates, min(num_remove, len(candidates))))

def destroy_tardiness(state, prob_info, num_remove):
    # 납기를 맞추지 못해 지연 페널티가 발생하는 블록들을 우선적으로 선택하여 제거
    tardiness_scores = [(b_id, max(0, v[5] - prob_info['blocks'][b_id]['due_date'])) for b_id, v in state.items()]
    tardiness_scores.sort(key=lambda x: x[1], reverse=True)
    candidates = [b_id for b_id, tard in tardiness_scores if tard > 0]
    if not candidates: return destroy_random(state, num_remove)
    sel = set([b_id for b_id, tard in tardiness_scores if tard > 0][:num_remove])
    if len(sel) < num_remove:
        rem = set(state.keys()) - sel
        sel.update(random.sample(list(rem), min(num_remove - len(sel), len(rem))))
    return sel

def is_conflict(c1, c2, prob_info, bays):
    # 두 후보 배치 상태 간의 시간적 겹침 및 물리적 충돌 여부를 검사
    if c1['bay'] != c2['bay']: return False
    if not _time_overlaps(c1['entry'], c1['exit'], c2['entry'], c2['exit']): return False
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

def generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, raster_cache, num=20):
    # 특정 블록에 대하여 도면 내 배치 가능한 여러 유효 후보군을 다수 탐색하여 반환
    cands = []
    r_time = b_info['release_time']
    d_time = b_info['due_date']
    p_time = b_info['processing_time']
    
    prefs = b_info['bay_preferences']
    bay_order = sorted(range(len(bays)), key=lambda idx: prefs[idx], reverse=True)
    
    for bay_idx in bay_order:
        bay = bays[bay_idx]
        for o_idx in range(len(b_info['shape'])):
            exit_t = d_time
            entry = exit_t - p_time
            grid, gw, gh = raster_cache.build_bay_grid(bay.id, bay_placed[bay.id], bay_schedule[bay.id], entry, exit_t)
            b_mask, bw, bh, blx0, bly0 = raster_cache.get_block_mask(b_id, o_idx)
            spots = raster_cache.find_valid_spots(grid, gw, gh, b_mask, bw, bh)
            spots.sort(key=lambda v: v[2], reverse=True)
            
            for gx, gy, score in spots:
                cx = int(math.ceil(gx - blx0))
                cy = int(math.ceil(gy - bly0))
                new_blk = Block(block_id=b_id, block_data=b_info, x=cx, y=cy, orient_idx=o_idx)
                has_col = False
                for p_blk, (p_e, p_ex) in zip(bay_placed[bay.id], bay_schedule[bay.id]):
                    if _time_overlaps(entry, exit_t, p_e, p_ex) and check_collisions(bay, [new_blk, p_blk]):
                        has_col = True; break
                if not has_col:
                    p_in = []
                    p_out = [new_blk]
                    obs_failed = False
                    for p_blk, (p_e, p_ex) in zip(bay_placed[bay.id], bay_schedule[bay.id]):
                        if p_e <= entry < p_ex: p_in.append(p_blk)
                        if p_e <= exit_t < p_ex: p_out.append(p_blk)
                        if entry <= p_e < exit_t and check_entry(bay, [new_blk], p_blk, fast=True):
                            obs_failed = True; break
                        if entry < p_ex <= exit_t and check_exit(bay, [new_blk], p_blk, fast=True):
                            obs_failed = True; break
                    
                    if not obs_failed and not check_entry(bay, p_in, new_blk, fast=True) and not check_exit(bay, p_out, new_blk, fast=True):
                        cands.append({'id': b_id, 'bay': bay.id, 'x': cx, 'y': cy, 'o_idx': o_idx, 'entry': entry, 'exit': exit_t})
                        if len(cands) >= num: return cands
    
    if not cands: # fallback to earliest tardy
        for bay_idx in bay_order:
            bay = bays[bay_idx]
            for o_idx in range(len(b_info['shape'])):
                x, y, entry, exit_t = search_placement(b_id, b_info, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time, o_idx, raster_cache, 'forward')
                if entry is not None:
                    cands.append({'id': b_id, 'bay': bay.id, 'x': x, 'y': y, 'o_idx': o_idx, 'entry': entry, 'exit': exit_t})
                    break
            if cands: break
    return cands

def repair_cpsat(U, fixed_state, prob_info, bays, raster_cache, time_limit):
    # 제거된 블록들을 제약 만족(CP-SAT) 기법과 후보군 조합을 활용하여 최적의 위치로 재배치
    bay_placed = {bay.id: [] for bay in bays}
    bay_schedule = {bay.id: [] for bay in bays}
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in fixed_state.items():
        blk = Block(block_id=b_id, block_data=prob_info['blocks'][b_id], x=x, y=y, orient_idx=o_idx)
        bay_placed[bay_id].append(blk)
        bay_schedule[bay_id].append((entry, exit_t))
        
    candidates_by_block = {}
    for b_id in U:
        b_info = prob_info['blocks'][b_id]
        cands = generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, raster_cache, num=20)
        if not cands: return None
        candidates_by_block[b_id] = cands
        
    conflicts = []
    U_list = list(U)
    for i in range(len(U_list)):
        b1 = U_list[i]
        for j in range(i+1, len(U_list)):
            b2 = U_list[j]
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
            score = int((w1 * tard + w3 * pref) * 100)
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

def post_optimize_time(state, prob_info, bays, start_time, timelimit):
    # 배치가 완료된 상태에서 각 블록의 진입 시간을 최대한 앞당겨 지연을 최소화하는 후처리 수행
    new_state = dict(state)
    changed = True
    while changed:
        changed = False
        for b_id in sorted(new_state.keys(), key=lambda k: new_state[k][5]):
            if time.time() - start_time > timelimit - 0.5:
                return new_state
            bay_id, x, y, o_idx, entry, exit_t = new_state[b_id]
            b_info = prob_info['blocks'][b_id]
            r_time = b_info['release_time']
            if entry > r_time:
                other_blocks = {ob_id: {'id': ob_id, 'bay': ob_data[0], 'x': ob_data[1], 'y': ob_data[2], 'o_idx': ob_data[3], 'entry': ob_data[4], 'exit': ob_data[5]} for ob_id, ob_data in new_state.items() if ob_id != b_id and ob_data[0] == bay_id}
                best_entry, best_exit = entry, exit_t
                curr_entry, curr_exit = entry - 1, exit_t - 1
                
                while curr_entry >= r_time:
                    cand = {'id': b_id, 'bay': bay_id, 'x': x, 'y': y, 'o_idx': o_idx, 'entry': curr_entry, 'exit': curr_exit}
                    if any(is_conflict(cand, ob_data, prob_info, bays) for ob_data in other_blocks.values()):
                        break
                    best_entry, best_exit = curr_entry, curr_exit
                    curr_entry -= 1
                    curr_exit -= 1
                        
                if best_entry < entry:
                    new_state[b_id] = (bay_id, x, y, o_idx, best_entry, best_exit)
                    changed = True
    return new_state

def alns_worker(prob_info, bays, initial_state, initial_obj, timelimit, start_time, seed, alns_duration):
    # 단일 프로세스 환경에서 파괴(Destroy)와 복구(Repair) 루프를 반복하며 점진적으로 해를 개선
    random.seed(seed)
    raster_cache = RasterCache(prob_info, bays)
    current_state = dict(initial_state)
    best_state = dict(initial_state)
    best_obj = initial_obj
    
    num_blocks = len(prob_info['blocks'])
    num_remove = max(1, int(num_blocks * 0.20))
    T_start, T_end = 1000.0, 0.01
    alns_weights = [1.0, 1.0, 1.0]
    
    stagnation = 0
    
    while True:
        elapsed = time.time() - start_time
        if elapsed >= alns_duration: break
            
        progress = max(0.0, min(1.0, elapsed / alns_duration))
        T = T_start * ((T_end / T_start) ** progress)
        
        # Adaptive removal size on stagnation
        curr_remove = num_remove
        if stagnation > 20:
            curr_remove = max(1, int(num_blocks * 0.40)) # Group B/D escape
            
        operator = random.choices([0, 1, 2], weights=alns_weights)[0]
        if operator == 0: removed = destroy_random(current_state, curr_remove)
        elif operator == 1: removed = destroy_workload(current_state, prob_info, bays, curr_remove)
        else: removed = destroy_tardiness(current_state, prob_info, curr_remove)
            
        fixed_state = {k: v for k, v in current_state.items() if k not in removed}
        new_state = repair_cpsat(removed, fixed_state, prob_info, bays, raster_cache, time_limit=3.0)
        
        score = 0
        if new_state:
            new_obj = compute_objective_val(prob_info, bays, new_state)
            if new_obj < best_obj:
                best_state, best_obj, current_state = dict(new_state), new_obj, dict(new_state)
                score = 10
                stagnation = 0
            else:
                current_obj = compute_objective_val(prob_info, bays, current_state)
                if new_obj < current_obj:
                    current_state = dict(new_state)
                    score = 5
                    stagnation = 0
                elif random.random() < math.exp((current_obj - new_obj) / T):
                    current_state = dict(new_state)
                    score = 2
                    stagnation += 1
                else:
                    stagnation += 1
        else:
            stagnation += 1
            
        alns_weights[operator] = alns_weights[operator] * 0.9 + score * 0.1
        
    return best_obj, best_state

def algorithm(prob_info, timelimit=60):
    # 멀티프로세싱을 통해 다중 워커를 실행하고 수집된 결과 중 최적의 스케줄링 솔루션을 도출
    start_time = time.time()
    bays = [Bay(width=b['width'], height=b['height'], id=i) for i, b in enumerate(prob_info['bays'])]
    raster_cache = RasterCache(prob_info, bays)
    
    initial_state = initialization(prob_info, bays, timelimit, start_time, raster_cache)
    if not initial_state: return {"operations": {}} # Failsafe
    initial_obj = compute_objective_val(prob_info, bays, initial_state)
    
    alns_duration = timelimit * 0.90
    workers_count = 4
    
    try:
        pool = multiprocessing.Pool(processes=workers_count)
        async_results = []
        for i in range(workers_count):
            seed = int(time.time() * 1000) + i
            res = pool.apply_async(alns_worker, (prob_info, bays, initial_state, initial_obj, timelimit, start_time, seed, alns_duration))
            async_results.append(res)
        pool.close()
        
        best_global_obj = initial_obj
        best_global_state = dict(initial_state)
        
        # Deadlock prevention loop
        while True:
            if time.time() - start_time > timelimit - 1.0:
                pool.terminate() # forcefully stop
                break
            all_ready = all(r.ready() for r in async_results)
            if all_ready:
                break
            time.sleep(0.5)
        
        for res in async_results:
            if res.ready():
                try:
                    worker_obj, worker_state = res.get()
                    if worker_obj < best_global_obj:
                        best_global_obj = worker_obj
                        best_global_state = worker_state
                except Exception: pass
    except Exception:
        best_global_obj, best_global_state = alns_worker(prob_info, bays, initial_state, initial_obj, timelimit, start_time, 0, alns_duration)
                    
    best_state = post_optimize_time(best_global_state, prob_info, bays, start_time, timelimit)
    return format_solution(best_state, prob_info, bays)
