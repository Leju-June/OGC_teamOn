import json
import time
from baseline_cpsat.myalgorithm import algorithm
from baseline_cpsat.utils import check_feasibility

def main():
    with open('train/prob_1.json', 'r') as f:
        prob_info = json.load(f)
        
    print("Starting ALNS CP-SAT Algorithm...")
    start = time.time()
    # Set a short timelimit for verification
    solution = algorithm(prob_info, timelimit=15)
    elapsed = time.time() - start
    
    print(f"Algorithm finished in {elapsed:.2f} seconds.")
    
    feasibility_res = check_feasibility(prob_info, solution)
    print("Feasibility Check Result:", feasibility_res)
    
if __name__ == '__main__':
    main()
