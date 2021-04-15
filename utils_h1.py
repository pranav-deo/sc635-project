def min_dist(v, v0, Gm, Gc, num_uav, nodes_at, nodes_used, time_taken):
    latest_nodes = nodes_at[:]
    # print(time_taken, nodes_at)
    while latest_nodes:
        latest_nodes_new = []
        len_nodes_at = len(latest_nodes)
        for i in range(len_nodes_at):
            if latest_nodes[i][1] < num_uav:
                for node in Gc[latest_nodes[i][0]]:
                    new_node = [node, latest_nodes[i][1] + 1]
                    if new_node not in nodes_used:
                        nodes_used.append(new_node)
                        nodes_at.append(new_node)
                        latest_nodes_new.append(new_node)
        latest_nodes = latest_nodes_new[:]
    print(time_taken, nodes_at)
    nodes_at_new = []
    for node in nodes_at:
        if node[0] == v:
            return time_taken
        for point in Gm[node[0]]:
            new_node = [point, node[1]]
            if new_node not in nodes_used:
                nodes_used.append(new_node)
                nodes_at_new.append(new_node)
    return min_dist(v, v0, Gm, Gc, num_uav, nodes_at_new[:], nodes_used[:], time_taken + 1)


def min_latency(Vs, v0, num_uav, Gm, Gc):
    """
    Input:  Vs -> list of SL node numbers
            v0 -> Base node number
            num_uav -> Number of uav available
            Gm -> Movement Graph
            Gc -> Communication Graph

    Output: dis_array -> list of minimum latencies from each vertex in Vs to v0
    """
    dis_array = []
    for v in Vs:
        nodes_at = []
        nodes_used = []
        for node in Gc[v0]:
            nodes_at.append([node, 1])
            nodes_used.append([node, 1])
        x = min_dist(v, v0, Gm, Gc, num_uav, nodes_at, nodes_used, 0)
        dis_array.append(x)
    return dis_array


def solve_tsp(Vs, Gm):
    """
    Input:  Vs -> list of SL node nmbers
            Gm -> Movement Graph

    Output: T -> Tour path (List of nodes in the path)
    """
    pass


def split_tour(T, k):
    """
    Input:  T -> Tour path (List of nodes in the path)
            k -> number of uav in the path

    Output: tours_array -> List of lists of tours (i.e. list of nodes in the path)
    """
    pass


def distGM(ev, sv):
    """
    Input:  ev -> end node
            sv -> start node
            Gm -> Movement Graph

    Output: len_path -> length of the shortest path between ev and sv in Gm 
    """
    pass


def minmax_matching(A):
    """
    Input:  A -> 2D matrix of PnC of all start and end nodes

    Output: M -> A list of UAVs and their start vertices
    """
    pass


def mlp(v, v0, gamma_v, Gm, Gc):
    """
    Input:  v -> node number
            v0 -> Base node number
            gamma_v -> Number of uav to use
            Gm -> Movement Graph
            Gc -> Communication Graph

    Output: sv -> list of start vertex for each UAV
            ev -> list of end vertex for each UAV
            st -> list of start time for each UAV
            et -> list of end time for each UAV
    """
    pass
