import json, time
from baseline_cpsat.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility
if __name__ == '__main__':
    p = json.load(open('train/prob_1.json', 'r', encoding='utf-8'))
    t0 = time.time()
    sol = algorithm(p, timelimit=15.0)
    t1 = time.time()
    feas = check_feasibility(p, sol)
    print(f'Feasible: {feas.get("feasible")}, Score: {feas.get("objective")}, Time: {t1-t0:.2f}s')
