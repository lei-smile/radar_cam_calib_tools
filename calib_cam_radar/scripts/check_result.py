import numpy as np
import matplotlib.pyplot as plt
import os  
import math
import yaml
import sys

print(str(sys.argv[1]))

file_name = "../dataset/"+str(sys.argv[1])+"/points_on_road.txt"
points = np.loadtxt(file_name, delimiter=',')

radar_x = points[:, 0]
radar_y = points[:, 1]

rtk_x = points[:, 2]
rtk_y = points[:, 3]

x_error = radar_x - rtk_x
y_error = radar_y -rtk_y

x_error_np = np.array(x_error)
x_error_avg = x_error_np.sum() / len(x_error_np)
print("x_error_avg:", x_error_avg)

y_error_np = np.array(y_error)
y_error_avg = y_error_np.sum() / len(y_error_np)
print("y_error_avg:", y_error_avg)

length = range(len(x_error))
plt.plot(length, x_error, 'r*-', label = 'x_error')
plt.plot(length, y_error, 'g+-', label = 'y_error')
plt.show()

plt.plot(radar_x, radar_y, 'r*-')
plt.plot(rtk_x, rtk_y, 'g+-')
plt.show()

# plt.plot(time_se, x_err_arr,'r+')
# plt.plot(time_se, y_err_arr,'g+')

# plt.plot(time_se, r_err_list,'b+')


# plt.xlabel('Time/s',fontsize=14)
# plt.ylabel('Error/m',fontsize=14)

# plt.title('X(r) Y(g) Distance(b) Error',fontsize=24)

# plt.show()

