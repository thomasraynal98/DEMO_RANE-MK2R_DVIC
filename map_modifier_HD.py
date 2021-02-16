# Here we will modify map_nbmap_HD.txt to add a security range and the size of the robot
# to each walls and obstacles already here in the map
import cv2
import numpy as np
# from pre_compute_path_FASTER import input_from_txt, array_map_to_ZeroOne

# map_number = 2

# name_minimap = "result/map/map_" + str(map_number) + "_L.txt"

# print("NUMPY ARRAY")
# low_map = input_from_txt(name_minimap)
# print("low map:\n", low_map)
# map_zero_one = array_map_to_ZeroOne(low_map)
# print("map_zero_one:\n", map_zero_one)

# print()

def DOWN(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] + i, position[1]])

    return position_extended

def UP(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] - i, position[1]])

    return position_extended

def LEFT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0], position[1] - i])

    return position_extended

def RIGHT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0], position[1] + i])

    return position_extended

def DOWN_LEFT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] + i, position[1] - i])

    return position_extended

def UP_LEFT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] - i, position[1] - i])

    return position_extended

def DOWN_RIGHT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] + i, position[1] + i])

    return position_extended

def UP_RIGHT(wanted_range, position):
    position_extended = []

    for i in range(wanted_range):
        position_extended.append([position[0] - i, position[1] + i])

    return position_extended


# low_map_array[6:,8:] = 1
# low_map_array[5, 5] = 1
# print("############ TEST #############")
# print(low_map_array)
# print()

# print()
# print("Copied_bis_matrix\n", Copied_bis_matrix)
# print()

def matrix_change(low_map_array, extend_coeff: int):
    
    low_map_array = np.array(low_map_array)
    
    COPIED_MATRIX = low_map_array.copy()
    Copied_bis_matrix = np.zeros(low_map_array.shape, dtype=int)

    # print("COPIED_MATRIX:\n", COPIED_MATRIX)

    for i in range(len(low_map_array)):
        for j in range(len(low_map_array[0])):
            if low_map_array[i, j] == 1:
                position = [i, j]
                # DOWN i ne doit pas dépasser la longeure max
                if i < len(low_map_array) - 1:
                    if low_map_array[i + 1, j] != 1:
                        position_exte = DOWN(extend_coeff, position)
                        position_copied = DOWN(extend_coeff, position)
                        for val in position_copied:
                            if val[0] > len(low_map_array) - 1:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # UP i ne doit pas être inférieur à 1
                if i > 0:
                    if low_map_array[i - 1, j] != 1:
                        position_exte = UP(extend_coeff, position)
                        position_copied = UP(extend_coeff, position)
                        for val in position_copied:
                            if val[0] < 0:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # LEFT j ne doit pas être inférieur à la hauteur
                if j > 0:
                    if low_map_array[i, j - 1] != 1:
                        position_exte = LEFT(extend_coeff, position)
                        position_copied = LEFT(extend_coeff, position)
                        for val in position_copied:
                            if val[1] < 0:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # RIGHT j ne doit dépasser la longueure max
                if j < len(low_map_array[0]) - 1:
                    if low_map_array[i, j + 1] != 1:
                        position_exte = RIGHT(extend_coeff, position)
                        pos_copied = RIGHT(extend_coeff, position)
                        for val in pos_copied:
                            if val[1] > len(low_map_array[0]) - 1:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # DOWN LEFT i ne doit pas dépasser la longeure max && j doit pas être inférieur à la hauteur
                if i < len(low_map_array) - 1 and j > 0:
                    if low_map_array[i + 1, j - 1] != 1:
                        position_exte = DOWN_LEFT(extend_coeff, position)
                        position_copied = DOWN_LEFT(extend_coeff, position)
                        for val in position_copied[:]:
                            if val[0] > len(low_map_array) - 1 or val[1] < 0:
                                position_exte.remove(val)
                            # print(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # UP LEFT i ne doit pas être inférieur à 1 && j doit pas être inférieur à la hauteur
                if i > 0 and j > 0:
                    if low_map_array[i - 1, j - 1] != 1:
                        position_exte = UP_LEFT(extend_coeff, position)
                        position_copied = UP_LEFT(extend_coeff, position)
                        for val in position_copied[:]:
                            if val[0] < 0 or val[1] < 0:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # DOWN RIGHT i ne doit pas dépasser la longeure max && j ne doit dépasser la longueure max
                if i < len(low_map_array) - 1 and j < len(low_map_array[0]) - 1:
                    if low_map_array[i + 1, j + 1] != 1:
                        position_exte = DOWN_RIGHT(extend_coeff, position)
                        position_copied = DOWN_RIGHT(extend_coeff, position)
                        for val in position_copied[:]:
                            if val[0] > len(low_map_array) - 1 or val[1] > len(low_map_array[0]) - 1:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1
                # UP RIGHT i ne doit pas être inférieur à 1 && j ne doit dépasser la longueure max
                if i > 0 and j < len(low_map_array[0]) - 1:
                    if low_map_array[i - 1, j + 1] != 1:
                        position_exte = UP_RIGHT(extend_coeff, position)
                        position_copied = UP_RIGHT(extend_coeff, position)
                        for val in position_copied[:]:
                            if val[0] < 0 or val[1] > len(low_map_array[0]) - 1:
                                position_exte.remove(val)
                        for val in position_exte:
                            Copied_bis_matrix[val[0], val[1]] = 1

    # print("FIRST TRANFORMATION Copied_bis_matrix :\n", Copied_bis_matrix)
    # print()

    for index, valu in enumerate(Copied_bis_matrix):
        for indexj, valo in enumerate(Copied_bis_matrix[0]):
            if Copied_bis_matrix[index, indexj] == 1:
                COPIED_MATRIX[index, indexj] = 1

    # print("COPIED_MATRIX:\n", COPIED_MATRIX)

    return COPIED_MATRIX
# print("MODIFIED LIST :")
# print(matrix_change(map_zero_one, 2))

def matrix_change_2(low_map_array, extend_coeff: int):
    """
        DESCRIPTION: transform map and add security.
        INPUT:
            * low_map_array = numpy[nxm] get map with 0 (empty), 1 (full)
        OUTPUT:
            * safe_map = numpy[nxm] return map with 0 (empty), 1 (full), 2 (safe)
    """
    extend_coeff_i = extend_coeff
    extend_coeff_j = extend_coeff
    safe_map = np.zeros((low_map_array.shape[0],low_map_array.shape[1]))

    for i in list(range(low_map_array.shape[0])):
        for j in list(range(low_map_array.shape[1])):
            # each cell, draw a box all around.
            
            if low_map_array[i,j] == 1:
                # detect bordure.
                ii = -(extend_coeff//2)
                while ii < (extend_coeff//2):
                    jj = -(extend_coeff//2)
                    while jj < (extend_coeff//2):
                        if (i+ii >= 0 and i+ii < low_map_array.shape[0]) and (j+jj >= 0 and j+jj < low_map_array.shape[1]):
                            if (ii == 0) and (jj == 0):
                                # a now wall.
                                safe_map[i+ii,j+jj] = 1
                            elif (low_map_array[i+ii,j+jj] == 0):
                                # a safe border when case is empty
                                safe_map[i+ii,j+jj] = 2 # 2
                        jj += 1
                    ii += 1

    return safe_map
        