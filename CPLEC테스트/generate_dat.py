import json
import os

def generate_opl_dat(json_path, dat_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    bays = data['bays']
    blocks = data['blocks']
    weights = data['weights']

    nbBays = len(bays)
    nbBlocks = len(blocks)

    # 1. Bay 정보
    W = [b['width'] for b in bays]
    H = [b['height'] for b in bays]
    
    # 2. Block 기본 정보
    R = [b['release_time'] for b in blocks]
    D = [b['due_date'] for b in blocks]
    P = [b['processing_time'] for b in blocks]
    L = [b['workload'] for b in blocks]
    S = [b['bay_preferences'] for b in blocks]
    Smax = [max(pref) for pref in S]

    # 가중치 (w1, w2, w3)
    w1 = weights['w1']
    w2 = weights['w2']
    w3 = weights['w3']

    # Bay 무게 u 계산
    total_area = sum(w * h for w, h in zip(W, H))
    avg_area = total_area / nbBays
    u = [avg_area / (w * h) for w, h in zip(W, H)]

    # 3. Shape Bounding Box 계산
    nbOrients = [len(b['shape']) for b in blocks]
    maxOrients = max(nbOrients)

    minX = [[0.0] * maxOrients for _ in range(nbBlocks)]
    maxX = [[0.0] * maxOrients for _ in range(nbBlocks)]
    minY = [[0.0] * maxOrients for _ in range(nbBlocks)]
    maxY = [[0.0] * maxOrients for _ in range(nbBlocks)]

    for i, block in enumerate(blocks):
        for shape in block['shape']:
            ori_idx = shape['orientation']
            all_x = []
            all_y = []
            for layer in shape['layers']:
                for point in layer:
                    all_x.append(point[0])
                    all_y.append(point[1])
            if all_x and all_y:
                minX[i][ori_idx] = min(all_x)
                maxX[i][ori_idx] = max(all_x)
                minY[i][ori_idx] = min(all_y)
                maxY[i][ori_idx] = max(all_y)

    # 4. .dat 파일로 작성 (OPL 문법)
    with open(dat_path, 'w') as f:
        f.write(f"nbBays = {nbBays};\n")
        f.write(f"nbBlocks = {nbBlocks};\n\n")

        f.write(f"W = {W};\n")
        f.write(f"H = {H};\n\n")

        f.write(f"R = {R};\n")
        f.write(f"D = {D};\n")
        f.write(f"P = {P};\n")
        f.write(f"L = {L};\n")
        
        # 2D Array formatting
        def fmt_2d(arr):
            return "[\n" + ",\n".join(["  [" + ", ".join(map(str, row)) + "]" for row in arr]) + "\n]"
            
        f.write(f"S = {fmt_2d(S)};\n\n")

        f.write(f"w1 = {w1};\n")
        f.write(f"w2 = {w2};\n")
        f.write(f"w3 = {w3};\n\n")

        f.write(f"u = [{', '.join([f'{val:.6f}' for val in u])}];\n")
        f.write(f"Smax = {Smax};\n\n")

        f.write(f"maxOrients = {maxOrients};\n")
        f.write(f"nbOrients = {nbOrients};\n\n")

        f.write(f"minX = {fmt_2d(minX)};\n")
        f.write(f"maxX = {fmt_2d(maxX)};\n")
        f.write(f"minY = {fmt_2d(minY)};\n")
        f.write(f"maxY = {fmt_2d(maxY)};\n")

    print(f"OPL 데이터 파일이 성공적으로 생성되었습니다: {dat_path}")

if __name__ == "__main__":
    # 실행
    json_input = "train/prob_1.json"
    dat_output = "prob_1.dat"
    if os.path.exists(json_input):
        generate_opl_dat(json_input, dat_output)
    else:
        print(f"파일을 찾을 수 없습니다: {json_input}")
