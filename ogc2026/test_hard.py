import os
import glob
import json
import time
import csv
from baseline_cpsat.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

# ==============================================================================
# 실행 설정 (Configuration)
# ==============================================================================
# Analysis_60300.md 에서 분석한 고난이도 문제들입니다.
HARD_PROBLEMS = [13, 23, 24, 25, 30, 35]        # Group B (좁은 공간 테트리스 형)
VERY_HARD_PROBLEMS = [26, 27, 28, 31, 32, 33, 37] # Group D (거대 스케일 구조적 재앙 형)

# 난이도별 시간 제한을 자유롭게 조절하세요 (단위: 초)
HARD_TIME_LIMIT = 300        # 어려움 난이도 제한 시간
VERY_HARD_TIME_LIMIT = 540  # 매우 어려움 난이도 제한 시간

NUM_RUNS = 1                            # 각 문제별 실행 반복 횟수
CSV_FILENAME = 'test_results_hard_run1_patched.csv'  # 결과 저장 CSV
# ==============================================================================

def main():
    train_dir = '../train'
    
    # 지정된 문제 번호에 해당하는 JSON 파일만 필터링
    target_numbers = set(HARD_PROBLEMS + VERY_HARD_PROBLEMS)
    
    all_prob_files = glob.glob(os.path.join(train_dir, 'prob_*.json'))
    prob_files = []
    for f in all_prob_files:
        prob_num = int(os.path.basename(f).split('_')[1].split('.')[0])
        if prob_num in target_numbers:
            prob_files.append(f)
            
    # 문제 번호 오름차순으로 정렬
    prob_files = sorted(prob_files, key=lambda x: int(os.path.basename(x).split('_')[1].split('.')[0]))
    
    with open(CSV_FILENAME, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Run', 'Problem', 'Difficulty', 'TimeLimit', 'Status', 'Feasible', 'Total Score', 'Obj1(Tard)', 'Obj2(Bal)', 'Obj3(Pref)', 'Time(s)'])
        
        for run_idx in range(1, NUM_RUNS + 1):
            print("\n=====================================================================================================================")
            print(f" [Run {run_idx}/{NUM_RUNS}] Hard & Very Hard Problems Test Run (Rasterization Version)")
            print("=====================================================================================================================")
            print(f"| {'Problem':<12} | {'Diff':<9} | {'T.Limit':<7} | {'Status':<6} | {'Feasible':<8} | {'Total Score':>14} | {'Obj1(Tard)':>14} | {'Obj2':>8} | {'Obj3':>8} | {'Time(s)':>8} |")
            print("|" + "-"*14 + "|" + "-"*11 + "|" + "-"*9 + "|" + "-"*8 + "|" + "-"*10 + "|" + "-"*16 + "|" + "-"*16 + "|" + "-"*10 + "|" + "-"*10 + "|" + "-"*10 + "|")
            
            for prob_file in prob_files:
                prob_name = os.path.basename(prob_file)
                prob_num = int(prob_name.split('_')[1].split('.')[0])
                
                # 난이도 및 시간 제한 판별
                if prob_num in VERY_HARD_PROBLEMS:
                    difficulty = "Very Hard"
                    time_limit = VERY_HARD_TIME_LIMIT
                else:
                    difficulty = "Hard"
                    time_limit = HARD_TIME_LIMIT
                
                try:
                    with open(prob_file, 'r', encoding='utf-8') as f:
                        prob_info = json.load(f)
                        
                    start_t = time.time()
                    solution = algorithm(prob_info, timelimit=time_limit)
                    elapsed = time.time() - start_t
                    
                    feas_result = check_feasibility(prob_info, solution)
                    feasible = feas_result['feasible']
                    
                    if feasible:
                        obj = feas_result['objective']
                        obj1 = feas_result['obj1']
                        obj2 = feas_result['obj2']
                        obj3 = feas_result['obj3']
                        print(f"| {prob_name:<12} | {difficulty:<9} | {time_limit:>7} | {'OK':<6} | {'Yes':<8} | {obj:>14,.1f} | {obj1:>14,.1f} | {obj2:>8,.1f} | {obj3:>8,.1f} | {elapsed:>8.1f} |", flush=True)
                        writer.writerow([run_idx, prob_name, difficulty, time_limit, 'OK', 'Yes', obj, obj1, obj2, obj3, round(elapsed, 1)])
                    else:
                        stage = feas_result.get('stage', 'Unknown')
                        print(f"| {prob_name:<12} | {difficulty:<9} | {time_limit:>7} | {'FAIL':<6} | {f'No(S{stage})':<8} | {'-':>14} | {'-':>14} | {'-':>8} | {'-':>8} | {elapsed:>8.1f} |", flush=True)
                        writer.writerow([run_idx, prob_name, difficulty, time_limit, 'FAIL', f'No(S{stage})', '', '', '', '', round(elapsed, 1)])
                        
                except Exception as e:
                    print(f"| {prob_name:<12} | {difficulty:<9} | {time_limit:>7} | {'ERROR':<6} | {'-':<8} | {'-':>14} | {'-':>14} | {'-':>8} | {'-':>8} | {'-':>8} |", flush=True)
                    writer.writerow([run_idx, prob_name, difficulty, time_limit, 'ERROR', str(e), '', '', '', '', ''])
                    
            csvfile.flush()
            
    print(f"\n✅ 고난이도 문제 테스트가 완료되었습니다. 결과가 '{CSV_FILENAME}'에 저장되었습니다.")

if __name__ == '__main__':
    main()
