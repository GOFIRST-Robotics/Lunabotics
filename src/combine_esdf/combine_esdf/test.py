import numpy as np

def main():
    x1 = np.array([1, 2, 3, 4])
    y1 = np.array([6, 2, 3, 4])

    x2 = np.array([2, 3, 4, 5, 1])
    y2 = np.array([3, 3, 4, 5, 1])



    coords1 = np.array([x1, y1])
    coords2 = np.array([x2, y2])

    # print(coords1.T)

    data1 = np.array([10, 20, 30, 40])
    data2 = np.array([50, 60, 70, 80])

    same_rows_mask = np.all(coords1.T[:,None] == coords2.T[None, :], axis=-1)

    row_indices = np.where(same_rows_mask)
    print(row_indices[0], row_indices[1])

    print(coords1.T)
    print(coords2.T)
    print(np.where(coords1.T.flatten() == coords2.T[:,0]))


    # print(matching_indices)
    # # Find matching indices based on x and y coordinates
    # matching_indices = np.where((coords))

    # # Modify data arrays where x and y match
    # data1[matching_indices[0]] += data2[matching_indices[1]]

    # print("Modified data1 array:", data1)