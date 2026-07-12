import json
import time
from baseline_cpsat.myalgorithm import algorithm

with open('cpsat_tester/example/example_B2_b10.json', 'r', encoding='utf-8') as f:
    prob_info = json.load(f)

start = time.time()
sol = algorithm(prob_info, timelimit=15.0)
print("Finished in:", time.time() - start)
print("Solution found:", len(sol.get('operations', {})))
