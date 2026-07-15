import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import glob
import json
import time
import csv
from baseline_gurobi.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

TIME_LIMIT = 60
NUM_RUNS = 1
CSV_FILENAME = 'test_results_60s_gurobi.csv'

def main():
    train_dir = '../../train'
    prob_files = sorted(glob.glob(os.path.join(train_dir, 'prob_*.json')), key=lambda x: int(os.path.basename(x).split('_')[1].split('.')[0]))
    
    with open(CSV_FILENAME, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Run', 'Problem', 'Status', 'Feasible', 'Total Score', 'Obj1(Tard)', 'Obj2(Bal)', 'Obj3(Pref)', 'Time(s)'])
        
        for run_idx in range(1, NUM_RUNS + 1):
            print("\n" + "="*105)
            print(f" [Run {run_idx}/{NUM_RUNS}] Starting Full {TIME_LIMIT}s Test Run for All Problems (ALNS + Gurobi)")
            print("="*105)
            print(f"| {'Problem':<12} | {'Status':<6} | {'Feasible':<8} | {'Total Score':>14} | {'Obj1(Tard)':>14} | {'Obj2(Bal)':>10} | {'Obj3(Pref)':>10} | {'Time(s)':>8} |")
            print("|" + "-"*14 + "|" + "-"*8 + "|" + "-"*10 + "|" + "-"*16 + "|" + "-"*16 + "|" + "-"*12 + "|" + "-"*12 + "|" + "-"*10 + "|")
            
            for prob_file in prob_files:
                prob_name = os.path.basename(prob_file)
                
                try:
                    with open(prob_file, 'r', encoding='utf-8') as f:
                        prob_info = json.load(f)
                        
                    start_t = time.time()
                    solution = algorithm(prob_info, timelimit=TIME_LIMIT)
                    elapsed = time.time() - start_t
                    
                    feas_result = check_feasibility(prob_info, solution)
                    feasible = feas_result['feasible']
                    
                    if feasible:
                        obj = feas_result['objective']
                        obj1 = feas_result['obj1']
                        obj2 = feas_result['obj2']
                        obj3 = feas_result['obj3']
                        print(f"| {prob_name:<12} | {'OK':<6} | {'Yes':<8} | {obj:>14,.1f} | {obj1:>14,.1f} | {obj2:>10,.1f} | {obj3:>10,.1f} | {elapsed:>8.1f} |", flush=True)
                        writer.writerow([run_idx, prob_name, 'OK', 'Yes', obj, obj1, obj2, obj3, round(elapsed, 1)])
                    else:
                        stage = feas_result.get('stage', 'Unknown')
                        print(f"| {prob_name:<12} | {'FAIL':<6} | {f'No(S{stage})':<8} | {'-':>14} | {'-':>14} | {'-':>10} | {'-':>10} | {elapsed:>8.1f} |", flush=True)
                        writer.writerow([run_idx, prob_name, 'FAIL', f'No(S{stage})', '', '', '', '', round(elapsed, 1)])
                        
                except Exception as e:
                    print(f"| {prob_name:<12} | {'ERROR':<6} | {'-':<8} | {'-':>14} | {'-':>14} | {'-':>10} | {'-':>10} | {'-':>8} |", flush=True)
                    writer.writerow([run_idx, prob_name, 'ERROR', str(e), '', '', '', '', ''])
                    
            csvfile.flush()
            
    print(f"\n✅ 총 {NUM_RUNS}번의 테스트가 완료되었으며, 결과가 '{CSV_FILENAME}'에 저장되었습니다.")

if __name__ == '__main__':
    main()
