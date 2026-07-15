import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import json
import time
import csv
from baseline_gurobi.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

# ==============================================================================
# 실행 설정 (Configuration)
# ==============================================================================
# 서버 문제(P1~P6)와 유사한 난이도의 Train 예제 매칭 및 시간 배분 설정 (총 1시간)
MOCK_CONFIG = [
    {"p_idx": "P1", "filename": "prob_2.json",  "time_limit": 60},
    {"p_idx": "P2", "filename": "prob_8.json",  "time_limit": 120},
    {"p_idx": "P3", "filename": "prob_5.json",  "time_limit": 300},
    {"p_idx": "P4", "filename": "prob_22.json", "time_limit": 720},
    {"p_idx": "P5", "filename": "prob_20.json", "time_limit": 900},
    {"p_idx": "P6", "filename": "prob_33.json", "time_limit": 1500},
]

CSV_FILENAME = 'test_results_server_mock.csv'  # 결과 저장 CSV
# ==============================================================================

def main():
    train_dir = '../../train'
    
    with open(CSV_FILENAME, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Mock', 'Problem', 'TimeLimit', 'Status', 'Feasible', 'Total Score', 'Obj1(Tard)', 'Obj2(Bal)', 'Obj3(Pref)', 'Time(s)'])
        
        print("\n=====================================================================================================================")
        print(" [Server Mock Test] 1시간(3600초) 시뮬레이션 - P1~P6 모의고사 (Gurobi Version)")
        print("=====================================================================================================================")
        print(f"| {'Mock':<4} | {'Problem':<12} | {'T.Limit':<7} | {'Status':<6} | {'Feasible':<8} | {'Total Score':>14} | {'Obj1(Tard)':>14} | {'Obj2':>8} | {'Obj3':>8} | {'Time(s)':>8} |")
        print("|" + "-"*6 + "|" + "-"*14 + "|" + "-"*9 + "|" + "-"*8 + "|" + "-"*10 + "|" + "-"*16 + "|" + "-"*16 + "|" + "-"*10 + "|" + "-"*10 + "|" + "-"*10 + "|")
        
        total_time_used = 0
        total_score = 0
        
        for config in MOCK_CONFIG:
            p_idx = config["p_idx"]
            prob_name = config["filename"]
            time_limit = config["time_limit"]
            
            prob_file = os.path.join(train_dir, prob_name)
            if not os.path.exists(prob_file):
                print(f"| {p_idx:<4} | {prob_name:<12} | {time_limit:>7} | {'ERROR':<6} | {'-':<8} | {'File Not Found':>14} | {'-':>14} | {'-':>8} | {'-':>8} | {'-':>8} |", flush=True)
                continue
                
            try:
                with open(prob_file, 'r', encoding='utf-8') as f:
                    prob_info = json.load(f)
                    
                start_t = time.time()
                solution = algorithm(prob_info, timelimit=time_limit)
                elapsed = time.time() - start_t
                total_time_used += elapsed
                
                feas_result = check_feasibility(prob_info, solution)
                feasible = feas_result['feasible']
                
                if feasible:
                    obj = feas_result['objective']
                    obj1 = feas_result['obj1']
                    obj2 = feas_result['obj2']
                    obj3 = feas_result['obj3']
                    total_score += obj
                    print(f"| {p_idx:<4} | {prob_name:<12} | {time_limit:>7} | {'OK':<6} | {'Yes':<8} | {obj:>14,.1f} | {obj1:>14,.1f} | {obj2:>8,.1f} | {obj3:>8,.1f} | {elapsed:>8.1f} |", flush=True)
                    writer.writerow([p_idx, prob_name, time_limit, 'OK', 'Yes', obj, obj1, obj2, obj3, round(elapsed, 1)])
                else:
                    stage = feas_result.get('stage', 'Unknown')
                    print(f"| {p_idx:<4} | {prob_name:<12} | {time_limit:>7} | {'FAIL':<6} | {f'No(S{stage})':<8} | {'-':>14} | {'-':>14} | {'-':>8} | {'-':>8} | {elapsed:>8.1f} |", flush=True)
                    writer.writerow([p_idx, prob_name, time_limit, 'FAIL', f'No(S{stage})', '', '', '', '', round(elapsed, 1)])
                    
            except Exception as e:
                print(f"| {p_idx:<4} | {prob_name:<12} | {time_limit:>7} | {'ERROR':<6} | {'-':<8} | {'-':>14} | {'-':>14} | {'-':>8} | {'-':>8} | {'-':>8} |", flush=True)
                writer.writerow([p_idx, prob_name, time_limit, 'ERROR', str(e), '', '', '', '', ''])
                
            csvfile.flush()
            
    print("=====================================================================================================================")
    print(f"✅ 서버 모의고사가 완료되었습니다! (총 소요 시간: {total_time_used:.1f}초 / 총점: {total_score:,.1f})")
    print(f"결과가 '{CSV_FILENAME}'에 저장되었습니다.")

if __name__ == '__main__':
    main()
