import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import json
import time
from baseline_gurobi.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

TIME_LIMIT = 20

def main():
    train_dir = '../../train'
    test_probs = ['prob_2.json', 'prob_5.json', 'prob_8.json']
    
    print(f"Starting Simple Local Test ({TIME_LIMIT}s limit)")
    print("-" * 80)
    for prob_name in test_probs:
        prob_file = os.path.join(train_dir, prob_name)
        if not os.path.exists(prob_file):
            print(f"File {prob_file} not found.")
            continue
            
        with open(prob_file, 'r', encoding='utf-8') as f:
            prob_info = json.load(f)
            
        print(f"Testing {prob_name}...")
        start_t = time.time()
        solution = algorithm(prob_info, timelimit=TIME_LIMIT)
        elapsed = time.time() - start_t
        
        feas_result = check_feasibility(prob_info, solution)
        feasible = feas_result['feasible']
        
        if feasible:
            obj = feas_result['objective']
            print(f"  -> SUCCESS! Feasible: Yes | Objective: {obj:,.1f} | Time: {elapsed:.1f}s")
        else:
            stage = feas_result.get('stage', 'Unknown')
            print(f"  -> FAIL! Feasible: No (Stage: {stage}) | Time: {elapsed:.1f}s")
    print("-" * 80)

if __name__ == '__main__':
    main()
