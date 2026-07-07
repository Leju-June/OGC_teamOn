import sys
import json
import os
sys.path.append(os.path.abspath(r'C:\Users\user\Desktop\OGC\ogc2026\baseline'))

from utils import check_feasibility
import baseline_simple

with open(r'C:\Users\user\Desktop\OGC\ogc2026\alg_tester\example\example_B2_b10.json', 'r') as f:
    prob_info = json.load(f)

print("Running baseline_simple algorithm...")
solution = baseline_simple.algorithm(prob_info, 60)

print("Checking feasibility...")
result = check_feasibility(prob_info, solution)
print("Result:", result)
