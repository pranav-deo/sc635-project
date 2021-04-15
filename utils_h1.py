from python_tsp.exact import solve_tsp_dynamic_programming
import numpy as np


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
    Input:  Vs -> list of SL node numbers
            Gm -> Movement Graph

    Output: T -> List of Tour path (List of nodes in the path) for each start point
    """

    # Gm_new = {}
    for v in Vs:
        #     Gm_new[v] = [v_ for v_ in Gm[v] if v_ in Vs]

    distance_matrix = [[float('inf') for _ in range(len(Gm_new.keys()))] for _ in range(len(Gm_new.keys()))]
    # for i in range()

    distance_matrix_np = np.array(distance_matrix)
    # distance_matrix_np[:, [V, 0]] = distance_matrix_np[:, [0, V]]
    # distance_matrix_np[[V, 0], :] = distance_matrix_np[[0, V], :]
    distance_matrix_np[:, 0] = 0

    permutation, distance = solve_tsp_dynamic_programming(distance_matrix)


def split_tour(T, k):
    """
    Input:  T -> Tour path (List of nodes in the path)
            k -> number of uav in the path

    Output: tours_array -> List of lists of tours (i.e. list of nodes in the path)
    """
    pass


def distGM(Gm, ev, sv):
    """
    Input:  ev -> end node
            sv -> start node
            Gm -> Movement Graph

    Output: len_path -> length of the shortest path between ev and sv in Gm
    """
    src = sv
    dest = ev
    V = len(Gm.keys())
    visited = [False] * (V)
    parent = [-1] * (V)

    # Create a queue for BFS
    queue = []

    # Mark the source node as visited and enqueue it
    queue.append(src)
    visited[src] = True

    while queue:

        # Dequeue a vertex from queue
        s = queue.pop(0)

        # if s = dest then print the path and return
        if s == dest:
            return printPath(Gm, parent, s)

        # Get all adjacent vertices of the dequeued vertex s
        # If a adjacent has not been visited, then mark it
        # visited and enqueue it
        for i in Gm[s]:
            if visited[i] is False:
                queue.append(i)
                visited[i] = True
                parent[i] = s


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


def printPath(Gm, parent, j):
    Path_len = 1
    V_org = len(Gm.keys())
    if parent[j] == -1 and j < V_org:  # Base Case : If j is source
        return 0  # when parent[-1] then path length = 0
    ll = printPath(Gm, parent, parent[j])

    Path_len = ll + Path_len

    return Path_len
