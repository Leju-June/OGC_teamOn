import os
import glob
import json
import time
from baseline_cpsat.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

# ==============================================================================
# 실행 시간 제한 (Time Limit) 설정 
# ==============================================================================
# 서버에서 주어지는 제한 시간에 맞춰 아래 값을 자유롭게 변경하며 테스트해 보세요.
# 기본값: 60 (초)
TIME_LIMIT = 60
# ==============================================================================

def main():
    train_dir = 'train'
    prob_files = sorted(glob.glob(os.path.join(train_dir, 'prob_*.json')), key=lambda x: int(os.path.basename(x).split('_')[1].split('.')[0]))
    
    results = []
    
    print("=========================================================================================================")
    print(f" Starting Full {TIME_LIMIT}s Test Run for All Problems (ALNS + CP-SAT, Multiprocessing)")
    print("=========================================================================================================")
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
                results.append((prob_name, "OK", feasible, obj, obj1, obj2, obj3, elapsed))
            else:
                stage = feas_result.get('stage', 'Unknown')
                print(f"| {prob_name:<12} | {'FAIL':<6} | {f'No(S{stage})':<8} | {'-':>14} | {'-':>14} | {'-':>10} | {'-':>10} | {elapsed:>8.1f} |", flush=True)
                results.append((prob_name, f"FAIL (Stage {stage})", False, None, None, None, None, elapsed))
                
        except Exception as e:
            print(f"| {prob_name:<12} | {'ERROR':<6} | {'-':<8} | {'-':>14} | {'-':>14} | {'-':>10} | {'-':>10} | {'-':>8} |", flush=True)
            results.append((prob_name, f"ERROR: {str(e)}", False, None, None, None, None, 0))

if __name__ == '__main__':
    main()
