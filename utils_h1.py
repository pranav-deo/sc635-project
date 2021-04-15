def min_dist(v, v0, Gm, Gc, num_uavs, nodes_at, nodes_used, time_taken):
    len_nodes_at = len(nodes_at)
    for i in range(len_nodes_at):
        if num_uavs[i] > 1:
            for node in Gc[nodes_at[i]]:
                if node not in nodes_used:
                    nodes_used.append()


def min_latency(Vs, v0, num_uav, Gm, Gc):
    """
    Input:  Vs -> list of SL node nmbers
            v0 -> Initial node number
            num_uav -> Number of uav available
            Gm -> Movement Graph
            Gc -> Communication Graph

    Output: dis_array -> list of minimum latencies from each vertex in Vs to v0
    """
    dis_array = []
    for v in Vs:
        min_dist(v, v0, Gm, Gc, [num_uav], [v0], [[v0, 0]], 0)
    return dis_array


def solve_tsp(Vs, Gm):
    pass


def split_tour(T, k):
    pass


def distGM(ev, sv):
    pass


def minmax_matching(A):
    pass


def mlp(v, v0, gamma_v, Gm, Gc):
    pass
