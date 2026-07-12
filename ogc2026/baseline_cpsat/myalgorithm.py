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

class RasterCache:
    def __init__(self, prob_info, bays, scale=1.0, is_coarse=False):
        self.prob_info = prob_info
        self.bays = bays
        self.block_masks = {}
        self.scale = scale
        self.is_coarse = is_coarse

    def get_block_mask(self, b_id, o_idx):
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
        
        if self.is_coarse:
            shrunk = footprint.buffer(-0.4)
            if not shrunk.is_empty:
                footprint = shrunk
        
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

    def find_valid_spots(self, grid, gw, gh, b_mask, bw, bh, x_range=None, y_range=None):
        valid = []
        if bh > gh or bw > gw:
            return valid
            
        start_y, end_y = 0, gh - bh
        start_x, end_x = 0, gw - bw
        
        if y_range:
            start_y = max(0, min(end_y, y_range[0]))
            end_y = max(0, min(end_y, y_range[1]))
        if x_range:
            start_x = max(0, min(end_x, x_range[0]))
            end_x = max(0, min(end_x, x_range[1]))
            
        for y in range(start_y, end_y + 1):
            for x in range(start_x, end_x + 1):
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

def check_pos_valid(x, y, entry, exit_t, b_id, b_info, bay, bay_placed, bay_schedule, o_idx):
    new_blk = Block(block_id=b_id, block_data=b_info, x=x, y=y, orient_idx=o_idx)
    bb = new_blk.bounding_rect()
    if bb[0] < 0 or bb[1] < 0 or bb[2] > bay.width or bb[3] > bay.height:
        return False
        
    p_in = []
    p_out = []
    for p_blk, (p_e, p_ex) in zip(bay_placed, bay_schedule):
        if _time_overlaps(entry, exit_t, p_e, p_ex):
            if check_collisions(bay, [new_blk, p_blk]):
                return False
        if p_e < entry < p_ex:
            p_in.append(p_blk)
        if p_e < exit_t < p_ex:
            p_out.append(p_blk)
            
    if p_in and check_entry(bay, p_in, new_blk, fast=True): return False
    if p_out and check_exit(bay, p_out, new_blk, fast=True): return False
    
    # Check if new_blk obstructs others
    for p_blk, (p_e, p_ex) in zip(bay_placed, bay_schedule):
        if entry <= p_e < exit_t:
            if check_entry(bay, [new_blk], p_blk, fast=True): return False
        if entry < p_ex <= exit_t:
            if check_exit(bay, [new_blk], p_blk, fast=True): return False
            
    return True

def search_placement(b_id, b_info, bay, bay_placed, bay_schedule, r_time, p_time, due_date, o_idx, coarse_cache, fine_cache, mode='backward'):
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

    dummy_blk = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
    lx0, ly0, lx1, ly1 = dummy_blk.bounding_rect()
    min_x = max(0, int(math.ceil(-lx0)))
    max_x = int(math.floor(bay.width - lx1))
    min_y = max(0, int(math.ceil(-ly0)))
    max_y = int(math.floor(bay.height - ly1))
    
    while (entry >= limit if mode == 'backward' else entry <= limit):
        grid, gw, gh = coarse_cache.build_bay_grid(bay.id, bay_placed, bay_schedule, entry, exit_t)
        b_mask, bw, bh, blx0, bly0 = coarse_cache.get_block_mask(b_id, o_idx)
        spots = coarse_cache.find_valid_spots(grid, gw, gh, b_mask, bw, bh)
        
        fine_spots = []
        if spots:
            f_grid, f_gw, f_gh = fine_cache.build_bay_grid(bay.id, bay_placed, bay_schedule, entry, exit_t)
            fb_mask, fbw, fbh, fblx0, fbly0 = fine_cache.get_block_mask(b_id, o_idx)
            for cx, cy, c_score in spots:
                start_fx = int(math.floor((cx - 1.0) / fine_cache.scale))
                end_fx = int(math.ceil((cx + 1.0) / fine_cache.scale))
                start_fy = int(math.floor((cy - 1.0) / fine_cache.scale))
                end_fy = int(math.ceil((cy + 1.0) / fine_cache.scale))
                f_spots = fine_cache.find_valid_spots(f_grid, f_gw, f_gh, fb_mask, fbw, fbh, x_range=(start_fx, end_fx), y_range=(start_fy, end_fy))
                for fx, fy, f_score in f_spots:
                    fine_spots.append((fx, fy, c_score * 10 + f_score))
        
        if fine_spots: spots = fine_spots
        spots.sort(key=lambda v: v[2], reverse=True)
        
        for gx, gy, score in spots[:5]:
            actual_x = max(min_x, min(max_x, round(gx - (fblx0 if fine_spots else blx0), 3)))
            actual_y = max(min_y, min(max_y, round(gy - (fbly0 if fine_spots else bly0), 3)))
            
            if check_pos_valid(actual_x, actual_y, entry, exit_t, b_id, b_info, bay, bay_placed, bay_schedule, o_idx):
                return actual_x, actual_y, entry, exit_t
            
        entry += step
        exit_t += step
        
    return None, None, None, None

def format_solution(state):
    ops_by_time = defaultdict(list)
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in state.items():
        ops_by_time[exit_t].append({"type": "EXIT", "block_id": b_id, "bay_id": bay_id})
        ops_by_time[entry].append({"type": "ENTRY", "block_id": b_id, "bay_id": bay_id, "x": x, "y": y, "orient_idx": o_idx})
    solution = {"operations": {}}
    for t_val in sorted(ops_by_time.keys()):
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

def initialization_strategy(prob_info, bays, timelimit, start_time, sort_strategy, coarse_cache, fine_cache):
    blocks_info = prob_info['blocks']
    if sort_strategy == 'rev_edd':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['due_date'], reverse=True)
    elif sort_strategy == 'edd':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['due_date'])
    elif sort_strategy == 'ptime':
        sorted_bids = sorted(range(len(blocks_info)), key=lambda i: blocks_info[i]['duration'], reverse=True)
    elif sort_strategy == 'random':
        sorted_bids = list(range(len(blocks_info)))
        random.shuffle(sorted_bids)
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
        
        if time.time() - start_time > timelimit * 0.15:
            # Fallback immediately
            best_bay, best_o, best_x, best_y = bay_order[0], 0, 0, 0
            found_fit = False
            for b_idx in bay_order:
                bay = bays[b_idx]
                for o_idx in range(len(b_info['shape'])):
                    dummy = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
                    lx0, ly0, lx1, ly1 = dummy.bounding_rect()
                    min_x = max(0, int(math.ceil(-lx0)))
                    max_x = int(math.floor(bay.width - lx1))
                    min_y = max(0, int(math.ceil(-ly0)))
                    max_y = int(math.floor(bay.height - ly1))
                    if min_x <= max_x and min_y <= max_y:
                        best_bay, best_o, best_x, best_y = b_idx, o_idx, min_x, min_y
                        found_fit = True
                        break
                if found_fit: break
            
            if not found_fit:
                bay = bays[bay_order[0]]
                dummy = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=0)
                lx0, ly0, lx1, ly1 = dummy.bounding_rect()
                best_x, best_y = max(0, int(math.ceil(-lx0))), max(0, int(math.ceil(-ly0)))

            entry = _empty_bay_entry(bay_schedule[best_bay], r_time, p_time)
            exit_t = entry + p_time
            new_blk = Block(block_id=b_id, block_data=b_info, x=best_x, y=best_y, orient_idx=best_o)
            state[b_id] = (best_bay, best_x, best_y, best_o, entry, exit_t)
            bay_placed[best_bay].append(new_blk)
            bay_schedule[best_bay].append((entry, exit_t))
            continue
            
        best_cand = None
        best_tard = float('inf')
        
        for bay_idx in bay_order:
            bay = bays[bay_idx]
            for o_idx in range(len(b_info['shape'])):
                gx, gy, entry, exit_t = search_placement(b_id, b_info, bay, bay_placed[bay_idx], bay_schedule[bay_idx], r_time, p_time, d_time, o_idx, coarse_cache, fine_cache, 'forward')
                if entry is not None:
                    tard = max(0, exit_t - d_time)
                    if tard < best_tard:
                        best_tard = tard
                        best_cand = (bay_idx, gx, gy, o_idx, entry, exit_t)
        
        if best_cand:
            bay_id, gx, gy, o_idx, entry, exit_t = best_cand
            new_blk = Block(block_id=b_id, block_data=b_info, x=gx, y=gy, orient_idx=o_idx)
            state[b_id] = best_cand
            bay_placed[bay_id].append(new_blk)
            bay_schedule[bay_id].append((entry, exit_t))
        else:
            # Fallback
            best_bay, best_o, best_x, best_y = bay_order[0], 0, 0, 0
            found_fit = False
            for b_idx in bay_order:
                bay = bays[b_idx]
                for o_idx in range(len(b_info['shape'])):
                    dummy = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
                    lx0, ly0, lx1, ly1 = dummy.bounding_rect()
                    min_x = max(0, int(math.ceil(-lx0)))
                    max_x = int(math.floor(bay.width - lx1))
                    min_y = max(0, int(math.ceil(-ly0)))
                    max_y = int(math.floor(bay.height - ly1))
                    if min_x <= max_x and min_y <= max_y:
                        best_bay, best_o, best_x, best_y = b_idx, o_idx, min_x, min_y
                        found_fit = True
                        break
                if found_fit: break
            
            if not found_fit:
                bay = bays[bay_order[0]]
                dummy = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=0)
                lx0, ly0, lx1, ly1 = dummy.bounding_rect()
                best_x, best_y = max(0, int(math.ceil(-lx0))), max(0, int(math.ceil(-ly0)))

            entry = _empty_bay_entry(bay_schedule[best_bay], r_time, p_time)
            exit_t = entry + p_time
            new_blk = Block(block_id=b_id, block_data=b_info, x=best_x, y=best_y, orient_idx=best_o)
            state[b_id] = (best_bay, best_x, best_y, best_o, entry, exit_t)
            bay_placed[best_bay].append(new_blk)
            bay_schedule[best_bay].append((entry, exit_t))
            
    return state, compute_objective_val(prob_info, bays, state)

def initialization(prob_info, bays, timelimit, start_time, raster_cache):
    # Backward compatibility if needed, but not used by main algorithm anymore.
    return initialization_strategy(prob_info, bays, timelimit, start_time, 'area', raster_cache)
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
    return set(random.sample(list(state.keys()), min(num_remove, len(state))))

def destroy_workload(state, prob_info, bays, num_remove):
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

def generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, coarse_cache, fine_cache, num=20):
    cands = []
    r_time = b_info['release_time']
    d_time = b_info['due_date']
    p_time = b_info['processing_time']
    
    prefs = b_info['bay_preferences']
    bay_order = sorted(range(len(bays)), key=lambda idx: prefs[idx], reverse=True)
    
    for bay_idx in bay_order:
        bay = bays[bay_idx]
        for o_idx in range(len(b_info['shape'])):
            dummy_blk = Block(block_id=b_id, block_data=b_info, x=0, y=0, orient_idx=o_idx)
            lx0, ly0, lx1, ly1 = dummy_blk.bounding_rect()
            min_x = max(0, int(math.ceil(-lx0)))
            max_x = int(math.floor(bay.width - lx1))
            min_y = max(0, int(math.ceil(-ly0)))
            max_y = int(math.floor(bay.height - ly1))
            
            exit_t = d_time
            entry = exit_t - p_time
            grid, gw, gh = coarse_cache.build_bay_grid(bay.id, bay_placed[bay.id], bay_schedule[bay.id], entry, exit_t)
            b_mask, bw, bh, blx0, bly0 = coarse_cache.get_block_mask(b_id, o_idx)
            spots = coarse_cache.find_valid_spots(grid, gw, gh, b_mask, bw, bh)
            if not spots: continue
            
            fine_spots = []
            f_grid, f_gw, f_gh = fine_cache.build_bay_grid(bay.id, bay_placed[bay.id], bay_schedule[bay.id], entry, exit_t)
            fb_mask, fbw, fbh, fblx0, fbly0 = fine_cache.get_block_mask(b_id, o_idx)
            for cx, cy, c_score in spots:
                start_fx = int(math.floor((cx - 1.0) / fine_cache.scale))
                end_fx = int(math.ceil((cx + 1.0) / fine_cache.scale))
                start_fy = int(math.floor((cy - 1.0) / fine_cache.scale))
                end_fy = int(math.ceil((cy + 1.0) / fine_cache.scale))
                f_spots = fine_cache.find_valid_spots(f_grid, f_gw, f_gh, fb_mask, fbw, fbh, x_range=(start_fx, end_fx), y_range=(start_fy, end_fy))
                for fx, fy, f_score in f_spots:
                    fine_spots.append((fx, fy, c_score * 10 + f_score))
            
            if fine_spots: spots = fine_spots
            
            spots.sort(key=lambda v: v[2], reverse=True)
            chosen_spots = spots[:3]
            if len(spots) > 3:
                chosen_spots.extend(random.sample(spots[3:], min(2, len(spots) - 3)))
                
            for gx, gy, score in chosen_spots:
                actual_x = max(min_x, min(max_x, round(gx - (fblx0 if fine_spots else blx0), 3)))
                actual_y = max(min_y, min(max_y, round(gy - (fbly0 if fine_spots else bly0), 3)))
                
                if check_pos_valid(actual_x, actual_y, entry, exit_t, b_id, b_info, bay, bay_placed[bay.id], bay_schedule[bay.id], o_idx):
                    cands.append({'id': b_id, 'bay': bay.id, 'x': actual_x, 'y': actual_y, 'o_idx': o_idx, 'entry': entry, 'exit': exit_t})
                    if len(cands) >= num: return cands
    
    if not cands: # fallback to earliest tardy
        for bay_idx in bay_order:
            bay = bays[bay_idx]
            gx, gy, entry, exit_t = search_placement(b_id, b_info, bay, bay_placed[bay.id], bay_schedule[bay.id], r_time, p_time, d_time, 0, coarse_cache, fine_cache, 'forward')
            if entry is not None:
                cands.append({'id': b_id, 'bay': bay.id, 'x': gx, 'y': gy, 'o_idx': 0, 'entry': entry, 'exit': exit_t})
                break
    return cands

def repair_cpsat(U, fixed_state, prob_info, bays, coarse_cache, fine_cache, time_limit):
    bay_placed = {bay.id: [] for bay in bays}
    bay_schedule = {bay.id: [] for bay in bays}
    for b_id, (bay_id, x, y, o_idx, entry, exit_t) in fixed_state.items():
        blk = Block(block_id=b_id, block_data=prob_info['blocks'][b_id], x=x, y=y, orient_idx=o_idx)
        bay_placed[bay_id].append(blk)
        bay_schedule[bay_id].append((entry, exit_t))
        
    candidates_by_block = {}
    for b_id in U:
        b_info = prob_info['blocks'][b_id]
        cands = generate_candidates(b_id, b_info, bays, bay_placed, bay_schedule, coarse_cache, fine_cache, num=20)
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


def alns_worker(prob_info, bays, timelimit, start_time, seed, alns_duration, worker_id):
    random.seed(seed)
    coarse_cache = RasterCache(prob_info, bays, scale=1.0, is_coarse=True)
    fine_cache = RasterCache(prob_info, bays, scale=0.1)
    
    strategies = ['area', 'ptime', 'edd', 'random']
    strategy = strategies[worker_id % len(strategies)]
    
    init_res = initialization_strategy(prob_info, bays, timelimit, start_time, strategy, coarse_cache, fine_cache)
    initial_state = init_res[0] if init_res else None
    
    if not initial_state:
        init_res = initialization_strategy(prob_info, bays, timelimit, start_time, 'area', coarse_cache, fine_cache)
        initial_state = init_res[0] if init_res else None
        if not initial_state: return float('inf'), None
        
    initial_obj = compute_objective_val(prob_info, bays, initial_state)
    
    current_state = dict(initial_state)
    best_state = dict(initial_state)
    best_obj = initial_obj
    
    num_blocks = len(prob_info['blocks'])
    num_remove = max(1, int(num_blocks * 0.20))
    T_start, T_end = max(10000.0, initial_obj * 0.05), 0.01
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
        new_state = repair_cpsat(removed, fixed_state, prob_info, bays, coarse_cache, fine_cache, time_limit=3.0)
        
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
    start_time = time.time()
    bays = [Bay(width=b['width'], height=b['height'], id=i) for i, b in enumerate(prob_info['bays'])]
    
    alns_duration = timelimit - 1.0
    workers_count = 4
    
    try:
        pool = multiprocessing.Pool(processes=workers_count)
        async_results = []
        for i in range(workers_count):
            seed = int(time.time() * 1000) + i
            res = pool.apply_async(alns_worker, (prob_info, bays, timelimit, start_time, seed, alns_duration, i))
            async_results.append(res)
        pool.close()
        
        best_global_obj = float('inf')
        best_global_state = None
        
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
                    if worker_state and worker_obj < best_global_obj:
                        best_global_obj = worker_obj
                        best_global_state = worker_state
                except Exception: pass
                
        if not best_global_state:
            # Absolute fallback
            rc_c = RasterCache(prob_info, bays, scale=1.0, is_coarse=True)
            rc_f = RasterCache(prob_info, bays, scale=0.1)
            init_res = initialization_strategy(prob_info, bays, timelimit, start_time, 'area', rc_c, rc_f)
            best_global_state = init_res[0] if init_res else None
            if not best_global_state: return {"operations": {}}
            
    except Exception:
        best_global_obj, best_global_state = alns_worker(prob_info, bays, timelimit, start_time, 0, alns_duration, 0)
        if not best_global_state: return {"operations": {}}
                    
    return format_solution(best_global_state)
