def distance(position_A, position_B):
    distance_twoPosition = np.sqrt(pow((position_A[0] - position_B[0]), 2) + pow((position_A[1] - position_B[1]), 2))
    return distance_twoPosition

def insert_open(xVal, yVal, parent_xVal, parent_yVal,hn,gn,fn):
    new_row = [1, xVal, yVal, parent_xVal, parent_yVal, hn, gn, fn]
    return new_row

def expand_array(node_x, node_y, hn, xTarget, yTarget, CLOSED, MAX_X, MAX_Y):
    expand_array = []
    CLOSED_size = np.shape(CLOSED)[0]
    for i in range(8):
        near_point = np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        s_x = node_x + near_point[i][0]
        s_y = node_y + near_point[i][1]
        if s_x >=0 and s_x<=MAX_X and s_y >= 0 and s_y<=MAX_Y:
            flag = 1
            for i in range(CLOSED_size):










def A_star_Algorithm(start_node, target_node, distance_array):
    OPEN = []
    CLOSED = []
    MAX_X = distance_array.shape[0]
    MAX_Y = distance_array.shape[1]
    x, y = np.where(distance_array == 0)
    for i in range(x.shape[0]):
        CLOSED.append([x[i], y[i]])
    xNode = start_node[0]
    yNode = start_node[1]
    xTarget = target_node[0]
    yTarget = target_node[1]
    path_cost = 0
    goal_distance = distance(start_node, target_node)
    OPEN.append(insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance))
    CLOSED.append([xNode, yNode])
    NoPath = 1
    while (xNode != xTarget or yNode != yTarget) and NoPath == 1:
        exp_array = expend_array()








