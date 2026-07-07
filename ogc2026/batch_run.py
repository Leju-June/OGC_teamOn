import os
import glob
import json
import time
from baseline_cpsat.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

def main():
    train_dir = 'train'
    prob_files = sorted(glob.glob(os.path.join(train_dir, 'prob_*.json')), key=lambda x: int(os.path.basename(x).split('_')[1].split('.')[0]))
    
    results = []
    
    print("| Problem | Status | Feasible | Total Score | Obj1 (Tardiness) | Obj2 (Balance) | Obj3 (Pref) | Time (s) |")
    print("|---------|--------|----------|-------------|------------------|----------------|-------------|----------|")
    
    for prob_file in prob_files:
        prob_name = os.path.basename(prob_file)
        
        try:
            with open(prob_file, 'r', encoding='utf-8') as f:
                prob_info = json.load(f)
                
            start_t = time.time()
            solution = algorithm(prob_info, timelimit=8)
            elapsed = time.time() - start_t
            
            feas_result = check_feasibility(prob_info, solution)
            feasible = feas_result['feasible']
            
            if feasible:
                obj = feas_result['objective']
                obj1 = feas_result['obj1']
                obj2 = feas_result['obj2']
                obj3 = feas_result['obj3']
                print(f"| {prob_name} | OK | {'Yes' if feasible else 'No'} | {obj:,.1f} | {obj1:,.1f} | {obj2:,.1f} | {obj3:,.1f} | {elapsed:.1f} |", flush=True)
                results.append((prob_name, "OK", feasible, obj, obj1, obj2, obj3, elapsed))
            else:
                stage = feas_result.get('stage', 'Unknown')
                violations = feas_result.get('violations', [])
                print(f"| {prob_name} | FAIL | No (Stage {stage}) | - | - | - | - | {elapsed:.1f} |", flush=True)
                results.append((prob_name, f"FAIL (Stage {stage})", False, None, None, None, None, elapsed))
                
        except Exception as e:
            print(f"| {prob_name} | ERROR | - | - | - | - | - | - |", flush=True)
            results.append((prob_name, f"ERROR: {str(e)}", False, None, None, None, None, 0))

if __name__ == '__main__':
    main()
