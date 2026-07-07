import math

def algorithm(prob_info: dict, timelimit: float) -> dict:
    bays = prob_info['bays']
    blocks = prob_info['blocks']
    
    # We need an ID for each block. In the JSON, blocks might not have 'id', but their index is the id.
    # Let's attach the original index as block_id.
    for i, b in enumerate(blocks):
        b['block_id'] = i
        
    # Sort blocks by release_time ascending, then block_id ascending
    remaining_blocks = sorted(blocks, key=lambda x: (x['release_time'], x['block_id']))
    
    current_time = 0
    operations = {}
    
    def add_operation(time_val, op):
        t_str = str(int(time_val))
        if t_str not in operations:
            operations[t_str] = []
        # Exits should be prepended or we can just let utils handle it (usually exits before entries).
        # We will separate ENTRY and EXIT in the dict, or just append since we only do one type per time per block group.
        # Actually, since exits happen at exit_time and entries at current_time, if exit_time == next current_time,
        # we might have exits and entries at the same time. Let's make sure EXIT comes before ENTRY in the list.
        if op['type'] == 'EXIT':
            operations[t_str].insert(0, op)
        else:
            operations[t_str].append(op)

    placed_in_this_round = []
    
    while remaining_blocks:
        block = remaining_blocks[0]
        
        # Advance time if the earliest available block is in the future
        if block['release_time'] > current_time:
            current_time = block['release_time']
            
        # Find preferred bay
        prefs = block['bay_preferences']
        best_bay_id = max(range(len(prefs)), key=lambda i: prefs[i])
        
        # Calculate bounding box W, H
        layers = block['shape'][0]['layers']
        min_x = min(v[0] for l in layers for v in l)
        max_x = max(v[0] for l in layers for v in l)
        min_y = min(v[1] for l in layers for v in l)
        max_y = max(v[1] for l in layers for v in l)
        
        bay_w = bays[best_bay_id]['width']
        bay_h = bays[best_bay_id]['height']
        
        min_xw = math.ceil(-min_x)
        max_xw = math.floor(bay_w - max_x)
        min_yw = math.ceil(-min_y)
        max_yw = math.floor(bay_h - max_y)
        
        placed = False
        # Max Y first, Min X next
        for y_w in range(max_yw, min_yw - 1, -1):
            for x_w in range(min_xw, max_xw + 1):
                bb_min_x = x_w + min_x
                bb_max_x = x_w + max_x
                bb_min_y = y_w + min_y
                bb_max_y = y_w + max_y
                
                overlap = False
                for pb in placed_in_this_round:
                    if pb['bay_id'] == best_bay_id:
                        # AABB overlap check
                        if not (bb_max_x <= pb['bb_min_x'] + 1e-5 or 
                                bb_min_x >= pb['bb_max_x'] - 1e-5 or 
                                bb_max_y <= pb['bb_min_y'] + 1e-5 or 
                                bb_min_y >= pb['bb_max_y'] - 1e-5):
                            overlap = True
                            break
                if not overlap:
                    pb_dict = {
                        'block_id': block['block_id'],
                        'bay_id': best_bay_id,
                        'x_w': x_w,
                        'y_w': y_w,
                        'bb_min_x': bb_min_x,
                        'bb_max_x': bb_max_x,
                        'bb_min_y': bb_min_y,
                        'bb_max_y': bb_max_y,
                        'processing_time': block['processing_time'],
                        'entry_time': current_time
                    }
                    placed_in_this_round.append(pb_dict)
                    add_operation(current_time, {
                        "type": "ENTRY",
                        "block_id": pb_dict['block_id'],
                        "bay_id": pb_dict['bay_id'],
                        "x": pb_dict['x_w'],
                        "y": pb_dict['y_w'],
                        "orient_idx": 0
                    })
                    remaining_blocks.pop(0)
                    placed = True
                    break
            if placed:
                break
        
        if not placed:
            # We COULD NOT place this block because there was an overlap in its preferred bay.
            # Thus, we trigger the EXIT phase.
            if not placed_in_this_round:
                print(f"Warning: Could not place block {block['block_id']} even in empty bay. Skipping.")
                remaining_blocks.pop(0)
                continue
                
            # Exit all currently placed blocks at the maximum required completion time
            max_processing_end_time = max(pb['entry_time'] + pb['processing_time'] for pb in placed_in_this_round)
            exit_time = max(current_time, max_processing_end_time)
            
            for pb in placed_in_this_round:
                add_operation(exit_time, {
                    "type": "EXIT",
                    "block_id": pb['block_id'],
                    "bay_id": pb['bay_id']
                })
                
            placed_in_this_round = []
            current_time = exit_time
            # Do not pop remaining_blocks[0]; it will be tried again in the next iteration.

    # After processing all remaining_blocks, check if any are still in the bay
    if placed_in_this_round:
        max_processing_end_time = max(pb['entry_time'] + pb['processing_time'] for pb in placed_in_this_round)
        exit_time = max(current_time, max_processing_end_time)
        for pb in placed_in_this_round:
            add_operation(exit_time, {
                "type": "EXIT",
                "block_id": pb['block_id'],
                "bay_id": pb['bay_id']
            })

    return {"operations": operations}
