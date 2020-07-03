#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

dt = 0.1 # sec
t_res = 0.01
v_res = 0.05
lst_omega = np.arange(-3,3.1, v_res)
lst_v = np.arange(-3,3.1, v_res)

lst_dt = np.arange(0.0, dt, t_res)
for i in range(len(lst_v)):
    color_tup = np.random.rand(3) 
    for j in range(len(lst_omega)):
        r = lst_v[i] / lst_omega[j]
        dY = r - r * np.cos(lst_omega[j] * lst_dt)
        dX = r * np.sin(lst_omega[j] * lst_dt)
        plt.plot(dX, dY, color=color_tup)
        print("Progress:{0}/{1}".format(i*len(lst_v)+j, len(lst_v)**2))

plt.show()

