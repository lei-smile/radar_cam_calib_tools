import numpy as np
import matplotlib.pyplot as plt
import os  
import math
import yaml
import sys

# print(str(sys.argv[1]))

#file_name = "../dataset/"+str(sys.argv[1])+"/points_on_road.txt"
file_name_lai = "83_1_lai_5_27.txt"
lai_r = np.loadtxt(file_name_lai, delimiter=',')

file_name_qu = "83_1_qu_5_27.txt"
qu_r = np.loadtxt(file_name_qu, delimiter=',')

delay_t = 3     #83_1
delta_r = 1.9

# delay_t = 3     #83_2
# delta_r = 1.9

# delay_t = 4     #83_3
#delta_r = 1.9      


lai_radar = np.array(lai_r[delay_t:, 0] + delta_r)
qu_radar = np.array(qu_r[delay_t:, 0] + delta_r)

lai_gps = np.array(lai_r[:(len(lai_r)-delay_t), 1])
qu_gps = np.array(qu_r[:(len(qu_r)-delay_t), 1])


all_gps = np.append(lai_gps, qu_gps)
all_radar = np.append(lai_radar, qu_radar)

error_r =  all_radar - all_gps
error_r_abs = np.abs(all_radar - all_gps)

avg_err = error_r.sum() / len(error_r)
avg_err_abs = error_r_abs.sum()/ len(error_r)
print("avg_err:", avg_err)
print("avg_err_abs: ", avg_err_abs)

length = range(len(all_radar))
plt.plot(length, all_radar, 'r*-', label = 'radar_r')
plt.plot(length, all_gps, 'g+-', label = 'gps_r')
plt.legend()
plt.show()


# plt.plot(time_se, x_err_arr,'r+')
# plt.plot(time_se, y_err_arr,'g+')

# plt.plot(time_se, r_err_list,'b+')


# plt.xlabel('Time/s',fontsize=14)
# plt.ylabel('Error/m',fontsize=14)

# plt.title('X(r) Y(g) Distance(b) Error',fontsize=24)

# plt.show()

