import numpy as np
from numba import jit

l2n = lambda l: np.array(l)
n2l = lambda n: list(n)

@jit(nopython=True, fastmath= True)
def make_transducer(ROC, width, dx, Tcenter, Tnormal):

    Tdia = width / 1000
    Troc = ROC / 1000
    pi = 3.141592653589793
    TP = 1

    H = Troc - np.sqrt(Troc ** 2 - (0.5 * Tdia) ** 2)
    # print(H*1000, "Tcenter to plane center [mm]")

    TP_dis = np.floor(H / dx)
    
    ####
    TP_dis = 0
    ####
    
    move_Tcenter = (Tcenter + TP_dis * Tnormal)

    nH = 2000
    nD = 2000
    nA = int(np.floor(0.5 * Tdia / dx) + 5)
    A = np.zeros((2 * nA + 1, 2 * nA + 1, 2 * nA + 1))

    Vs = np.zeros(3)
    Vs[1] = -Tnormal[2]
    Vs[2] = Tnormal[1]

    if np.all(Vs == 0):
        Vs[1] = 0.000000000001
        Vs[2] = 0.000000000001

    Vs = Vs / np.linalg.norm(Vs)
    Vt = np.cross(Tnormal, Vs)

    for i in range(nH):
        R = np.sqrt(Troc ** 2 - (Troc - H + i * H / nH) ** 2)
        for j in range(nD):
            theta = 2 * pi / nD * j
            X = int((nA + 1) + np.round(
                (-(i) * H / nH * Tnormal[0] + R * np.cos(theta) * Vs[0] + R * np.sin(theta) * Vt[0]) / dx))
            Y = int((nA + 1) + np.round(
                (-(i) * H / nH * Tnormal[1] + R * np.cos(theta) * Vs[1] + R * np.sin(theta) * Vt[1]) / dx))
            Z = int((nA + 1) + np.round(
                (-(i) * H / nH * Tnormal[2] + R * np.cos(theta) * Vs[2] + R * np.sin(theta) * Vt[2]) / dx))
            A[X, Y, Z] = 1

    nS = int(np.sum(A))
    Spos = np.zeros((nS, 4))
    idx = np.where(A == 1)

    Spos[:, 3] = 1000*H*(TP_dis + np.floor((idx[0] - nA + 1) * Tnormal[0] + (idx[1] - nA + 1) * Tnormal[1] + (idx[2] - nA + 1) * Tnormal[2]))/TP_dis
    Spos[:, 2] = idx[0] - (nA + 1) + move_Tcenter[0]
    Spos[:, 1] = idx[1] - (nA + 1) + move_Tcenter[1]
    Spos[:, 0] = idx[2] - (nA + 1) + move_Tcenter[2]

    return Spos

