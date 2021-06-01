import numpy as np
import matplotlib.pyplot as plt
import os  
import math
import yaml



file = open(".points_on_road.txt", "r")
rtk_list = file.readlines() 
times = []
lists_r_radar = []
lists_dis_gps = []

x_radar_list = []
y_radar_list = []
x_gps_list = []
y_gps_list = []

x_err_list = []
y_err_list = []

r_err_list = []

# tx_wr = 0.956176
# ty_wr = -0.199588
tx_wr = 0.0
ty_wr = 0.0
M_PI = 3.14159265359

result_file = open("../config/calib_result.txt", "r")  
lines = result_file.readlines()  
for line in lines[0:]:
  result = line.split(',',3)
  yaw_wr = float(result[0])
  delt_t = float(result[1])
  tz_wr = float(result[2])

for string in rtk_list[0:]:
  strs = string.split(',',9)
  t = float(strs[0])
  r = float(strs[1])
  angle = float(strs[2])
  v = float(strs[3])
  x_rtk = float(strs[4])
  y_rtk = float(strs[5])
  vx_rtk = float(strs[6])
  vy_rtk = float(strs[7])

  dis_gps = math.sqrt(tz_wr*tz_wr + x_rtk*x_rtk + y_rtk*y_rtk)

  times.append(t)
  lists_r_radar.append(r)
  lists_dis_gps.append(dis_gps)
  
  x_gps_list.append(x_rtk)
  y_gps_list.append(y_rtk)

  dis_hori = math.sqrt(r*r - tz_wr*tz_wr)
  x_radar_r = dis_hori*math.sin(angle*M_PI/180.0)
  y_radar_r = dis_hori*math.cos(angle*M_PI/180.0)
  x_radar_w = x_radar_r*math.cos(yaw_wr) - y_radar_r*math.sin(yaw_wr) + tx_wr 
  y_radar_w = x_radar_r*math.sin(yaw_wr) + y_radar_r*math.cos(yaw_wr) + ty_wr 

  #y_radar_w = math.sqrt(r*r - tz_wr*tz_wr - x_radar_w*x_radar_w)

  x_radar_list.append(x_radar_w)
  y_radar_list.append(y_radar_w)

  x_err_list.append(float(x_radar_w - x_rtk))
  y_err_list.append(float(y_radar_w - y_rtk+1.0))
  r_err_list.append(float(r - dis_gps+1.0))

sum_x_err = 0
for i in x_err_list:
  sum_x_err = sum_x_err + abs(i)
x_err_avg = sum_x_err/len(x_err_list)
print("x_err_avg: " , x_err_avg)

sum_y_err = 0
for i in y_err_list:
  sum_y_err = sum_y_err + abs(i)
y_err_avg = sum_y_err/len(y_err_list)
print("y_err_avg: " , y_err_avg)

sum_r_err = 0
for i in r_err_list:
  sum_r_err = sum_r_err + abs(i)
r_err_avg = sum_r_err/len(r_err_list)
print("r_err_avg: " , r_err_avg)

save_file = open("../config/final_result.yaml", "w") 
save_datas = {
                'theta': yaw_wr,
                'delta_t': delt_t,
                'h': tz_wr,
                'x_err_avg': x_err_avg,
                'y_err_avg': y_err_avg,
                'r_err_avg': r_err_avg
             }
yaml.dump(save_datas, save_file)

time_se = np.array(times)
radar_dis = np.array(lists_r_radar)
gps_dis = np.array(lists_dis_gps)
x_gps_arr = np.array(x_gps_list)
y_gps_arr = np.array(y_gps_list)
x_radar_arr = np.array(x_radar_list)
y_radar_arr = np.array(y_radar_list)

time_se = time_se.astype(np.float_)
radar_dis = radar_dis.astype(np.float_)
gps_dis = gps_dis.astype(np.float_)

# plt.plot(time_se,radar_dis,'bo')
# plt.plot(time_se,gps_dis,'go')
# plt.plot(y_gps_arr, x_gps_arr,'g+')
# plt.plot(y_radar_arr, x_radar_arr,'r+')

x_err_arr = np.array(x_err_list)
y_err_arr = np.array(y_err_list)

# plt.plot(time_se, x_radar_arr,'r*')
# plt.plot(time_se, x_gps_arr,'g*')

# plt.plot(time_se, y_radar_arr,'r^')
# plt.plot(time_se, y_gps_arr,'g^')

plt.plot(time_se, x_err_arr,'r+')
plt.plot(time_se, y_err_arr,'g+')

plt.plot(time_se, r_err_list,'b+')

# plt.text(0.5, 1, 'put some text')

plt.xlabel('Time/s',fontsize=14)
plt.ylabel('Error/m',fontsize=14)

plt.title('X(r) Y(g) Distance(b) Error',fontsize=24)

plt.show()

