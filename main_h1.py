"""
Algorithm 1: Heuristic for MILC(MILC - H1).

Input: movement and communication graphs GM, GC, sensing locations VS, number of UAVs r, latency bound Lc

Output: subtours t 1, . . ., t k, MLP and schedule (st(v), sv v, ev v, st v) ∀v ∈ V S
"""

# G = {0: {1, 3, 4}, 1: {0, 4, 2}, 2: {1, 4, 5}, 3: {0, 1, 4, 7, 6}, 4: {0, 1, 2, 5, 8, 7, 6, 3},
#      5:{1,2,4,7,8}, 6:{3,4,7,10,9}, 7:{3,4,5,,8,11,10,9,6}, 8:{5,4,7,10,11}, 9:{}}


# 8 -> S, 0 -> BS

from utils_h1 import min_latency, solve_tsp, split_tour, distGM, minmax_matching

Gm = {8: [5], 5: [4, 6], 6: [5], 4: [5, 3], 3: [4, 2], 2: [3, 1], 1: [2, 7, 0], 7: [1], 0: [1]}
Gc = {8: [5, 4, 6], 5: [4, 6], 6: [5, 8, 7], 4: [5, 3, 8], 3: [4, 2], 2: [3, 1], 1: [2, 7, 0], 7: [1, 6], 0: [1]}

Vs = [8]
r = 2
Lc = 4
V0 = 0


def give_m1(Gm, Gc, Vs, r, Lc, V0):
    gammas = [float('inf') for v in range(len(Vs))]

    for num_uav in range(r, 0, -1):
        # All MLPs from all v ∈ Vs to the BS with i UAVs:
        dist_array = min_latency(Vs, V0, num_uav, Gm, Gc)

        for ii in Vs:
            if dist_array[ii] <= Lc:
                gammas[ii] = num_uav

    if max(gammas) > r:
        print("Problem is infeasible!")
        return None

    new_Vs = [v for v in Vs if v not in Gc[V0]]
    gamma = max(new_Vs)
    k = r // gamma

    T = solve_tsp(Vs, Gm)
    tour_array = split_tour(T, k)

    for ii in range(1, k + 1):
        Ri = list(range((ii - 1) * gamma + 1, ii * gamma + 1))
        v_ = V0

        for v in tour_array[i]:
            (svv, evv, stv, etv) = mlp(v, V0, gammas[v], Gm, Gc)

            A = {}

            for l in range(1, gammas[v]):
                for m in Ri:
