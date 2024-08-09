import numpy as np
import matplotlib.pyplot as plt

# for i in range(300):
for i in range(299,-1,-1):
    npy_file = f'grid_files/grid_{i}.npy'
    txt_file = f'inflated_txt_files/output_{i}.txt'

    raw_data = np.load(npy_file)
    raw_data = np.array(raw_data, dtype=np.float64)
    bracket = [1] + [0 for _ in range(28)] + [1]
    bracket = np.array(bracket, dtype=np.float64)
    for _ in range(10):
        raw_data = np.insert(raw_data, 0, bracket, axis = 1)
    for _ in range(10):
        raw_data = np.insert(raw_data, raw_data.shape[1], bracket, axis = 1)

    # plt.imshow(raw_data, cmap = 'Greys')

    rows = len(raw_data)
    cols = len(raw_data[0])

    BLOCK = 10
    raw_data[raw_data==1] = BLOCK

    data = raw_data
    for row in range(rows):
        for col in range(cols):
            if raw_data[row][col] == BLOCK:
                for nr in range(max(0,row-1), min(row+1,rows)):
                    for nc in range(max(0,col-1), min(col+1,cols)):
                        data[nr][nc] = BLOCK
                        
    rmax = 5
    inflated_data = np.full_like(data, rmax)
    for row in range(rows):
        for col in range(cols):
            if data[row][col] == BLOCK:
                for nr in range(max(0,row-rmax), min(row+rmax,rows)):
                    for nc in range(max(0,col-rmax), min(col+rmax,cols)):
                        dr = abs(nr - row)
                        dc = abs(nc - col)
                        in_rectangle = 0
                        if dr==0 and dc==0:
                            pass
                        elif dr!=0 and dc!=0:
                            mul = 0.5/max(dr, dc)
                            in_rectangle = ((dr)**2 + (dc)**2)**0.5*mul
                        else:
                            in_rectangle = 0.5
                        distance = ((nr - row)**2 + (nc - col)**2)**0.5 - in_rectangle
                        if distance > rmax:
                            continue
                        if distance == 0:
                            continue
                        # print(distance)
                        if inflated_data[nr][nc] < distance:
                            # print(inflated_data[nr][nc], distance)
                            continue
                        inflated_data[nr][nc] = distance
    
    inflated_data[data == BLOCK] = BLOCK
    new_data = np.full_like(data, rmax)

    for row in range(rows):
        for col in range(cols):
            if inflated_data[row][col] != BLOCK:
                min_distance = 10
                for nr in range(max(0,row-3), min(row+3,rows)):
                    for nc in range(max(0,col-3), min(col+3,cols)):
                        min_distance = min(min_distance, inflated_data[nr][nc])
                new_data[row][col] = min_distance
    new_data[data == BLOCK] = BLOCK
                        
    # print(inflated_data)
    # plt.figure()
    # plt.imshow(inflated_data, cmap = 'Greys')

    # plt.figure(figsize=(25,25))
    # plt.imshow(new_data, cmap = 'Greys')
    # plt.show()
    
    np.savetxt(txt_file, inflated_data, fmt='%.5e')

