# -*- coding: utf-8 -*-
"""
Created on Wed Sep  4 22:23:23 2019

@author: toothsmile
"""

import numpy as np

# matplotlib don't use Xwindows backend (must be before pyplot import)
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

#global varible
timestamp_innov=[]
vel_pos0=[]
vel_pos1=[]
vel_pos2=[]
vel_pos3=[]
vel_pos4=[]
vel_pos5=[]
vel_pos_var0=[]
vel_pos_var1=[]
vel_pos_var2=[]
vel_pos_var3=[]
vel_pos_var4=[]
vel_pos_var5=[]
innov_mag=[]
heading_innov=[]
innov_mag_var=[]
vel_pos_innov_var=[]
heading_innov_var=[]

output_tracking_error_0=[]
output_tracking_error_1=[]
output_tracking_error_2=[]
#read estimator_status,ekf2_innovations,gps_position,local_position file
def readfile(fname):
    with open(fname) as textdata:
        lines=textdata.readlines()
    return lines        
    
                        

innovatin_file="ekf_innovations.txt"#ecl_offline残差文件
estimate_file="estimate_status.txt"#ecl_offline滤波状态文件
lpos_file="vechile_local_position.txt"#滤波出的当地坐标系的坐标文件
gps_file="gps_position_0.csv"#原始解出的gps csv文件
vision_file="vision_position_0.csv"#解出的视觉文件
output_plot_filename="innov_analyse.pdf"#输出的分析文件
#读取新息数据
lines=readfile(innovatin_file)
for line in lines:
    strLine=line.split(',')
    timestamp_innov.append(float(strLine[0])*1e-6)
    vel_pos=[]#速度位置新息
    mag_inno=[]#mag新息
    #head_inno=[]#航向的新息
    vel_pos_var=[]#速度位置新息协方差
    mag_inno_var=[]#mag新息方差
    vel_pos0.append(float(strLine[1]));vel_pos1.append(float(strLine[2]));vel_pos2.append(float(strLine[3]));
    vel_pos3.append(float(strLine[4]));vel_pos4.append(float(strLine[5]));vel_pos5.append(float(strLine[6]));
   
    
    mag_inno.append(float(strLine[7]));mag_inno.append(float(strLine[8]));mag_inno.append(float(strLine[9]));
    innov_mag.append(mag_inno)
    heading_innov.append(float(strLine[10]))
    vel_pos_var0.append(float(strLine[16]));vel_pos_var1.append(float(strLine[17]));vel_pos_var2.append(float(strLine[18]));
    vel_pos_var3.append(float(strLine[19]));vel_pos_var4.append(float(strLine[20]));vel_pos_var5.append(float(strLine[21]));
    heading_innov_var.append(float(strLine[25]))
    output_tracking_error_0.append(1e3*float(strLine[31]));output_tracking_error_1.append(float(strLine[32]));
    output_tracking_error_2.append(float(strLine[33]));
pp = PdfPages(output_plot_filename)
# generate max, min and 1-std metadata
    plt.figure(1, figsize=(20, 13))
# generate metadata for velocity innovationsinnov_2_max_arg = np.argmax(vel_pos2)
innov_2_max_arg = np.argmax(vel_pos2)
innov_2_max_time = timestamp_innov[innov_2_max_arg]
innov_2_max = np.amax(vel_pos2)
innov_2_min_arg = np.argmin(vel_pos2)
innov_2_min_time = timestamp_innov[innov_2_min_arg]
innov_2_min = np.amin(vel_pos2)
s_innov_2_max = str(round(innov_2_max, 2))
s_innov_2_min = str(round(innov_2_min, 2))
# s_innov_2_std = str(round(np.std(vel_pos2),2))
# generate metadata for position innovations
innov_5_max_arg = np.argmax(vel_pos5)
innov_5_max_time = timestamp_innov[innov_5_max_arg]
innov_5_max = np.amax(vel_pos5)
innov_5_min_arg = np.argmin(vel_pos5)
innov_5_min_time = timestamp_innov[innov_5_min_arg]
innov_5_min = np.amin(vel_pos5)
s_innov_5_max = str(round(innov_5_max, 2))
s_innov_5_min = str(round(innov_5_min, 2))
# s_innov_5_std = str(round(np.std(vel_pos5),2))
# generate plot for vertical velocity innovations
plt.subplot(2, 1, 1)
plt.plot(timestamp_innov, vel_pos2, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos_var2), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos_var2), 'r')
plt.title('Vertical Innovations')
plt.ylabel('Down Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_2_max_time, innov_2_max, 'max=' + s_innov_2_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_2_min_time, innov_2_min, 'min=' + s_innov_2_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_2_std],loc='upper left',frameon=False)
# generate plot for vertical position innovations
plt.subplot(2, 1, 2)
plt.plot(timestamp_innov, vel_pos5, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos5), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos5), 'r')
plt.ylabel('Down Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_5_max_time, innov_5_max, 'max=' + s_innov_5_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_5_min_time, innov_5_min, 'min=' + s_innov_5_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_5_std],loc='upper left',frameon=False)
pp.savefig()
plt.show()
plt.close(1)
# horizontal velocity innovations
plt.figure(2, figsize=(20, 13))
# generate North axis metadata
innov_0_max_arg = np.argmax(vel_pos0)
innov_0_max_time = timestamp_innov[innov_0_max_arg]
innov_0_max = np.amax(vel_pos0)
innov_0_min_arg = np.argmin(vel_pos0)
innov_0_min_time = timestamp_innov[innov_0_min_arg]
innov_0_min = np.amin(vel_pos0)
s_innov_0_max = str(round(innov_0_max, 2))
s_innov_0_min = str(round(innov_0_min, 2))
# s_innov_0_std = str(round(np.std(vel_pos0),2))
# Generate East axis metadata
innov_1_max_arg = np.argmax(vel_pos1)
innov_1_max_time = timestamp_innov[innov_1_max_arg]
innov_1_max = np.amax(vel_pos1)
innov_1_min_arg = np.argmin(vel_pos1)
innov_1_min_time = timestamp_innov[innov_1_min_arg]
innov_1_min = np.amin(vel_pos1)
s_innov_1_max = str(round(innov_1_max, 2))
s_innov_1_min = str(round(innov_1_min, 2))
# s_innov_1_std = str(round(np.std(vel_pos1),2))
# draw plots
plt.subplot(2, 1, 1)
plt.plot(timestamp_innov, vel_pos0, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos_var0), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos_var0), 'r')
plt.title('Horizontal Velocity  Innovations')
plt.ylabel('North Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_0_max_time, innov_0_max, 'max=' + s_innov_0_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_0_min_time, innov_0_min, 'min=' + s_innov_0_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_0_std],loc='upper left',frameon=False)
plt.subplot(2, 1, 2)
plt.plot(timestamp_innov, vel_pos1, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos_var1), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos_var1), 'r')
plt.ylabel('East Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_1_max_time, innov_1_max, 'max=' + s_innov_1_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_1_min_time, innov_1_min, 'min=' + s_innov_1_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_1_std],loc='upper left',frameon=False)
pp.savefig()
plt.close(2)
# horizontal position innovations
plt.figure(3, figsize=(20, 13))
# generate North axis metadata
innov_3_max_arg = np.argmax(vel_pos3)
innov_3_max_time = timestamp_innov[innov_3_max_arg]
innov_3_max = np.amax(vel_pos3)
innov_3_min_arg = np.argmin(vel_pos3)
innov_3_min_time = timestamp_innov[innov_3_min_arg]
innov_3_min = np.amin(vel_pos3)
s_innov_3_max = str(round(innov_3_max, 2))
s_innov_3_min = str(round(innov_3_min, 2))
# s_innov_3_std = str(round(np.std(vel_pos3),2))
# generate East axis metadata
innov_4_max_arg = np.argmax(vel_pos4)
innov_4_max_time = timestamp_innov[innov_4_max_arg]
innov_4_max = np.amax(vel_pos4)
innov_4_min_arg = np.argmin(vel_pos4)
innov_4_min_time = timestamp_innov[innov_4_min_arg]
innov_4_min = np.amin(vel_pos4)
s_innov_4_max = str(round(innov_4_max, 2))
s_innov_4_min = str(round(innov_4_min, 2))
# s_innov_4_std = str(round(np.std(vel_pos4),2))
# generate plots
plt.subplot(2, 1, 1)
plt.plot(timestamp_innov, vel_pos3, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos_var3), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos_var3), 'r')
plt.title('Horizontal Position Innovations')
plt.ylabel('North Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_3_max_time, innov_3_max, 'max=' + s_innov_3_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_3_min_time, innov_3_min, 'min=' + s_innov_3_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_3_std],loc='upper left',frameon=False)
plt.subplot(2, 1, 2)
plt.plot(timestamp_innov, vel_pos4, 'b')
plt.plot(timestamp_innov, np.sqrt(vel_pos_var4), 'r')
plt.plot(timestamp_innov, -np.sqrt(vel_pos_var4), 'r')
plt.ylabel('East Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_4_max_time, innov_4_max, 'max=' + s_innov_4_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_4_min_time, innov_4_min, 'min=' + s_innov_4_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_4_std],loc='upper left',frameon=False)

pp.savefig()
plt.show()
plt.close(3)

 #magnetic heading innovations
plt.figure(5, figsize=(20, 13))
# generate metadata
innov_0_max_arg = np.argmax(heading_innov)
innov_0_max_time = timestamp_innov[innov_0_max_arg]
innov_0_max = np.amax(heading_innov)
innov_0_min_arg = np.argmin(heading_innov)
innov_0_min_time = timestamp_innov[innov_0_min_arg]
innov_0_min = np.amin(heading_innov)
s_innov_0_max = str(round(innov_0_max, 3))
s_innov_0_min = str(round(innov_0_min, 3))
# s_innov_0_std = str(round(np.std(heading_innov),3))
# draw plot
plt.plot(timestamp_innov, heading_innov, 'b')
plt.plot(timestamp_innov, np.sqrt(heading_innov_var), 'r')
plt.plot(timestamp_innov, -np.sqrt(heading_innov_var), 'r')
plt.title('Magnetic Heading Innovations')
plt.ylabel('Heaing (rad)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_0_max_time, innov_0_max, 'max=' + s_innov_0_max, fontsize=12, horizontalalignment='left',
         verticalalignment='bottom')
plt.text(innov_0_min_time, innov_0_min, 'min=' + s_innov_0_min, fontsize=12, horizontalalignment='left',
         verticalalignment='top')
# plt.legend(['std='+s_innov_0_std],loc='upper left',frameon=False)

pp.savefig()
plt.show()
plt.close(5)

#读取估计状态信息
mag_test_ratio=[]
vel_test_ratio=[]
pos_test_ratio=[]
hgt_test_ratio=[]
tas_test_ratio=[]
status_time=[]
hagl_test_ratio=[]
beta_test_ratio=[]
control_mode_flags=[]
innovation_check_flags=[]
gps_check_fail_flags=[]
filter_fault_flags=[]
solution_status_flags=[]
pos_horiz_accuracy=[]
pos_vert_accuracy=[]
state_10=[]
state_11=[]
state_12=[]
state_13=[]
state_14=[]
state_15=[]
state_16=[]
state_17=[]
state_18=[]
lines=readfile(estimate_file)
for line in lines:
    strLine=line.split(',')
    status_time.append(int(strLine[0])*1e-6)
    control_mode_flags.append(int(strLine[53]))
    mag_test_ratio.append(float(strLine[56]))
    vel_test_ratio.append(float(strLine[57]))
    pos_test_ratio.append(float(strLine[58]))
    hgt_test_ratio.append(float(strLine[59]))
    tas_test_ratio.append(float(strLine[60]))
    hagl_test_ratio.append(float(strLine[61]))
    beta_test_ratio.append(float(strLine[62]))
    innovation_check_flags.append(int(strLine[66]))
    filter_fault_flags.append(int(strLine[64]))
    gps_check_fail_flags.append(int(strLine[65]))    
    solution_status_flags.append(int(strLine[67]))
    pos_horiz_accuracy.append(float(strLine[54]))
    pos_vert_accuracy.append(float(strLine[55]))
    state_10.append(float(strLine[11]));state_11.append(float(strLine[12]));state_12.append(float(strLine[13]));
    state_13.append(float(strLine[14]));state_14.append(float(strLine[15]));state_15.append(float(strLine[16]));
    state_16.append(float(strLine[17]));state_17.append(float(strLine[18]));state_18.append(float(strLine[19]));
    
# generate metadata for the normalised innovation consistency test levels
# a value > 1.0 means the measurement data for that test has been rejected by the EKF
# magnetometer data
mag_test_max_arg = np.argmax(mag_test_ratio)
mag_test_max_time = status_time[mag_test_max_arg]
mag_test_max = np.amax(mag_test_ratio)
mag_test_mean = np.mean(mag_test_ratio)
# velocity data (GPS)
vel_test_max_arg = np.argmax(vel_test_ratio)
vel_test_max_time = status_time[vel_test_max_arg]
vel_test_max = np.amax(vel_test_ratio)
vel_test_mean = np.mean(vel_test_ratio)
# horizontal position data (GPS or external vision)
pos_test_max_arg = np.argmax(pos_test_ratio)
pos_test_max_time = status_time[pos_test_max_arg]
pos_test_max = np.amax(pos_test_ratio)
pos_test_mean = np.mean(pos_test_ratio)
# height data (Barometer, GPS or rangefinder)
hgt_test_max_arg = np.argmax(hgt_test_ratio)
hgt_test_max_time = status_time[hgt_test_max_arg]
hgt_test_max = np.amax(hgt_test_ratio)
hgt_test_mean = np.mean(hgt_test_ratio)
# airspeed data
tas_test_max_arg = np.argmax(tas_test_ratio)
tas_test_max_time = status_time[tas_test_max_arg]
tas_test_max = np.amax(tas_test_ratio)
tas_test_mean = np.mean(tas_test_ratio)
# height above ground data (rangefinder)
hagl_test_max_arg = np.argmax(hagl_test_ratio)
hagl_test_max_time = status_time[hagl_test_max_arg]
hagl_test_max = np.amax(hagl_test_ratio)
hagl_test_mean = np.mean(hagl_test_ratio)
# plot normalised innovation test levels
plt.figure(8, figsize=(20, 13))
if tas_test_max == 0.0:
    n_plots = 3
else:
    n_plots = 4
plt.subplot(n_plots, 1, 1)
plt.plot(status_time, mag_test_ratio, 'b')
plt.title('Normalised Innovation Test Levels')
plt.ylabel('mag')
plt.xlabel('time (sec)')
plt.grid()
plt.text(mag_test_max_time, mag_test_max,
         'max=' + str(round(mag_test_max, 2)) + ' , mean=' + str(round(mag_test_mean, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='bottom', color='b')
plt.subplot(n_plots, 1, 2)
plt.plot(status_time, vel_test_ratio, 'b')
plt.plot(status_time, pos_test_ratio, 'r')
plt.ylabel('vel,pos')
plt.xlabel('time (sec)')
plt.grid()
plt.text(vel_test_max_time, vel_test_max,
         'vel max=' + str(round(vel_test_max, 2)) + ' , mean=' + str(round(vel_test_mean, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='bottom', color='b')
plt.text(pos_test_max_time, pos_test_max,
         'pos max=' + str(round(pos_test_max, 2)) + ' , mean=' + str(round(pos_test_mean, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='bottom', color='r')
plt.subplot(n_plots, 1, 3)
plt.plot(status_time, hgt_test_ratio, 'b')
plt.ylabel('hgt')
plt.xlabel('time (sec)')
plt.grid()
plt.text(hgt_test_max_time, hgt_test_max,
         'hgt max=' + str(round(hgt_test_max, 2)) + ' , mean=' + str(round(hgt_test_mean, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='bottom', color='b')
if hagl_test_max > 0.0:
    plt.plot(status_time, hagl_test_ratio, 'r')
    plt.text(hagl_test_max_time, hagl_test_max,
             'hagl max=' + str(round(hagl_test_max, 2)) + ' , mean=' + str(round(hagl_test_mean, 2)), fontsize=12,
             horizontalalignment='left', verticalalignment='bottom', color='r')
    plt.ylabel('hgt,HAGL')
if n_plots == 4:
    plt.subplot(n_plots, 1, 4)
    plt.plot(status_time, tas_test_ratio, 'b')
    plt.ylabel('TAS')
    plt.xlabel('time (sec)')
    plt.grid()
    plt.text(tas_test_max_time, tas_test_max,
             'max=' + str(round(tas_test_max, 2)) + ' , mean=' + str(round(tas_test_mean, 2)), fontsize=12,
             horizontalalignment='left', verticalalignment='bottom', color='b')
pp.savefig()
plt.show()
plt.close(8)


#读取初始化时间


# extract control mode metadata from estimator_status.control_mode_flags
# 0 - true if the filter tilt alignment is complete
# 1 - true if the filter yaw alignment is complete
# 2 - true if GPS measurements are being fused
# 3 - true if optical flow measurements are being fused
# 4 - true if a simple magnetic yaw heading is being fused
# 5 - true if 3-axis magnetometer measurement are being fused
# 6 - true if synthetic magnetic declination measurements are being fused
# 7 - true when the vehicle is airborne
# 8 - true when wind velocity is being estimated
# 9 - true when baro height is being fused as a primary height reference
# 10 - true when range finder height is being fused as a primary height reference
# 11 - true when range finder height is being fused as a primary height reference
# 12 - true when local position data from external vision is being fused
# 13 - true when yaw data from external vision measurements is being fused
# 14 - true when height data from external vision measurements is being fused
i=0
tilt_aligned=[]
yaw_aligned=[]
using_gps=[]
using_optflow=[]
using_magyaw=[]
using_mag3d=[]
using_magdecl=[]
airborne=[]
estimating_wind=[]
using_barohgt=[]
using_rnghgt=[]
using_gpshgt=[]
using_evpos=[]
using_evyaw=[]
using_evhgt=[]

for flags in control_mode_flags:
    tilt_aligned.append((2 ** 0 & flags > 0) * 1)
    yaw_aligned.append((2 ** 1 & flags > 0) * 1)
    using_gps.append((2 ** 2 & flags > 0) * 1)
    using_optflow.append((2 ** 3 & flags > 0) * 1)
    using_magyaw.append((2 ** 4 & flags > 0) * 1)
    using_mag3d.append((2 ** 5 & flags > 0) * 1)
    using_magdecl.append((2 ** 6 & flags > 0) * 1)
    airborne.append((2 ** 7 & flags > 0) * 1)
    estimating_wind.append((2 ** 8 & flags > 0) * 1)
    using_barohgt.append((2 ** 9 & flags > 0) * 1)
    using_rnghgt.append((2 ** 10 & flags > 0) * 1)
    using_gpshgt.append( (2 ** 11 & flags > 0) * 1)
    using_evpos.append((2 ** 12 & flags > 0) * 1)
    using_evyaw.append((2 ** 13 & flags > 0) * 1)
    using_evhgt.append((2 ** 14 & flags > 0) * 1)
    i+=1

# calculate in-air transition time
if (np.amin(airborne) < 0.5) and (np.amax(airborne) > 0.5):
    in_air_transtion_time_arg = np.argmax(np.diff(airborne))
    in_air_transition_time = status_time[in_air_transtion_time_arg]
elif (np.amax(airborne) > 0.5):
    in_air_transition_time = np.amin(status_time)
    print('log starts while in-air at ' + str(round(in_air_transition_time, 1)) + ' sec')
    b_starts_in_air = True
else:
    in_air_transition_time = float('NaN')
    print('always on ground')
# calculate on-ground transition time
if (np.amin(np.diff(airborne)) < 0.0):
    on_ground_transition_time_arg = np.argmin(np.diff(airborne))
    on_ground_transition_time = status_time[on_ground_transition_time_arg]
elif (np.amax(airborne) > 0.5):
    on_ground_transition_time = np.amax(status_time)
    print('log finishes while in-air at ' + str(round(on_ground_transition_time, 1)) + ' sec')
    b_finishes_in_air = True
else:
    on_ground_transition_time = float('NaN')
    print('always on ground')
if (np.amax(np.diff(airborne)) > 0.5) and (np.amin(np.diff(airborne)) < -0.5):
    if ((on_ground_transition_time - in_air_transition_time) > 0.0):
        in_air_duration = on_ground_transition_time - in_air_transition_time;
    else:
        in_air_duration = float('NaN')
else:
    in_air_duration = float('NaN')
# calculate alignment completion times
tilt_align_time_arg = np.argmax(np.diff(tilt_aligned))
tilt_align_time = status_time[tilt_align_time_arg]
yaw_align_time_arg = np.argmax(np.diff(yaw_aligned))
yaw_align_time = status_time[yaw_align_time_arg]
# calculate position aiding start times
gps_aid_time_arg = np.argmax(np.diff(using_gps))
gps_aid_time = status_time[gps_aid_time_arg]
optflow_aid_time_arg = np.argmax(np.diff(using_optflow))
optflow_aid_time = status_time[optflow_aid_time_arg]
evpos_aid_time_arg = np.argmax(np.diff(using_evpos))
evpos_aid_time = status_time[evpos_aid_time_arg]
# calculate height aiding start times
barohgt_aid_time_arg = np.argmax(np.diff(using_barohgt))
barohgt_aid_time = status_time[barohgt_aid_time_arg]
gpshgt_aid_time_arg = np.argmax(np.diff(using_gpshgt))
gpshgt_aid_time = status_time[gpshgt_aid_time_arg]
rnghgt_aid_time_arg = np.argmax(np.diff(using_rnghgt))
rnghgt_aid_time = status_time[rnghgt_aid_time_arg]
evhgt_aid_time_arg = np.argmax(np.diff(using_evhgt))
evhgt_aid_time = status_time[evhgt_aid_time_arg]
# calculate magnetometer aiding start times
using_magyaw_time_arg = np.argmax(np.diff(using_magyaw))
using_magyaw_time = status_time[using_magyaw_time_arg]
using_mag3d_time_arg = np.argmax(np.diff(using_mag3d))
using_mag3d_time = status_time[using_mag3d_time_arg]
using_magdecl_time_arg = np.argmax(np.diff(using_magdecl))
using_magdecl_time = status_time[using_magdecl_time_arg]
    
# control mode summary plot A
plt.figure(9, figsize=(20, 13))
# subplot for alignment completion
plt.subplot(4, 1, 1)
plt.title('EKF Control Status - Figure A')
plt.plot(status_time, tilt_aligned, 'b')
plt.plot(status_time, yaw_aligned, 'r')
plt.ylim(-0.1, 1.1)
plt.ylabel('aligned')
plt.grid()
if np.amin(tilt_aligned) > 0:
    plt.text(tilt_align_time, 0.5, 'no pre-arm data - cannot calculate alignment completion times', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='black')
else:
    plt.text(tilt_align_time, 0.33, 'tilt alignment at ' + str(round(tilt_align_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
    plt.text(yaw_align_time, 0.67, 'yaw alignment at ' + str(round(tilt_align_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='r')
# subplot for position aiding
plt.subplot(4, 1, 2)
plt.plot(status_time, using_gps, 'b')
plt.plot(status_time, using_optflow, 'r')
plt.plot(status_time, using_evpos, 'g')
plt.ylim(-0.1, 1.1)
plt.ylabel('pos aiding')
plt.grid()
if np.amin(using_gps) > 0:
    plt.text(gps_aid_time, 0.25, 'no pre-arm data - cannot calculate GPS aiding start time', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
elif np.amax(using_gps) > 0:
    plt.text(gps_aid_time, 0.25, 'GPS aiding at ' + str(round(gps_aid_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
if np.amin(using_optflow) > 0:
    plt.text(optflow_aid_time, 0.50, 'no pre-arm data - cannot calculate optical flow aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='r')
elif np.amax(using_optflow) > 0:
    plt.text(optflow_aid_time, 0.50, 'optical flow aiding at ' + str(round(optflow_aid_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='r')
if np.amin(using_evpos) > 0:
    plt.text(evpos_aid_time, 0.75, 'no pre-arm data - cannot calculate external vision aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='g')
elif np.amax(using_evpos) > 0:
    plt.text(evpos_aid_time, 0.75, 'external vision aiding at ' + str(round(evpos_aid_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='g')
# subplot for height aiding
plt.subplot(4, 1, 3)
plt.plot(status_time, using_barohgt, 'b')
plt.plot(status_time, using_gpshgt, 'r')
plt.plot(status_time, using_rnghgt, 'g')
plt.plot(status_time, using_evhgt, 'c')
plt.ylim(-0.1, 1.1)
plt.ylabel('hgt aiding')
plt.grid()
if np.amin(using_barohgt) > 0:
    plt.text(barohgt_aid_time, 0.2, 'no pre-arm data - cannot calculate Baro aiding start time', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
elif np.amax(using_barohgt) > 0:
    plt.text(barohgt_aid_time, 0.2, 'Baro aiding at ' + str(round(gps_aid_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
if np.amin(using_gpshgt) > 0:
    plt.text(gpshgt_aid_time, 0.4, 'no pre-arm data - cannot calculate GPS aiding start time', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='r')
elif np.amax(using_gpshgt) > 0:
    plt.text(gpshgt_aid_time, 0.4, 'GPS aiding at ' + str(round(gpshgt_aid_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='r')
if np.amin(using_rnghgt) > 0:
    plt.text(rnghgt_aid_time, 0.6, 'no pre-arm data - cannot calculate rangfinder aiding start time', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='g')
elif np.amax(using_rnghgt) > 0:
    plt.text(rnghgt_aid_time, 0.6, 'rangefinder aiding at ' + str(round(rnghgt_aid_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='g')
if np.amin(using_evhgt) > 0:
    plt.text(evhgt_aid_time, 0.8, 'no pre-arm data - cannot calculate external vision aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='c')
elif np.amax(using_evhgt) > 0:
    plt.text(evhgt_aid_time, 0.8, 'external vision aiding at ' + str(round(evhgt_aid_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='c')
# subplot for magnetometer aiding
plt.subplot(4, 1, 4)
plt.plot(status_time, using_magyaw, 'b')
plt.plot(status_time, using_mag3d, 'r')
plt.plot(status_time, using_magdecl, 'g')
plt.ylim(-0.1, 1.1)
plt.ylabel('mag aiding')
plt.xlabel('time (sec)')
plt.grid()
if np.amin(using_magyaw) > 0:
    plt.text(using_magyaw_time, 0.25, 'no pre-arm data - cannot calculate magnetic yaw aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='b')
elif np.amax(using_magyaw) > 0:
    plt.text(using_magyaw_time, 0.25, 'magnetic yaw aiding at ' + str(round(using_magyaw_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='right', verticalalignment='center', color='b')
if np.amin(using_mag3d) > 0:
    plt.text(using_mag3d_time, 0.50, 'no pre-arm data - cannot calculate 3D magnetoemter aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='r')
elif np.amax(using_mag3d) > 0:
    plt.text(using_mag3d_time, 0.50, 'magnetometer 3D aiding at ' + str(round(using_mag3d_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='r')
if np.amin(using_magdecl) > 0:
    plt.text(using_magdecl_time, 0.75, 'no pre-arm data - cannot magnetic declination aiding start time',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='g')
elif np.amax(using_magdecl) > 0:
    plt.text(using_magdecl_time, 0.75,
             'magnetic declination aiding at ' + str(round(using_magdecl_time, 1)) + ' sec', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='g')
pp.savefig()
plt.show()
plt.close(9)
# control mode summary plot B
plt.figure(10, figsize=(20, 13))
# subplot for airborne status
plt.subplot(2, 1, 1)
plt.title('EKF Control Status - Figure B')
plt.plot(status_time, airborne, 'b')
plt.ylim(-0.1, 1.1)
plt.ylabel('airborne')
plt.grid()
if np.amax(np.diff(airborne)) < 0.5:
    plt.text(in_air_transition_time, 0.67, 'ground to air transition not detected', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
else:
    plt.text(in_air_transition_time, 0.67, 'in-air at ' + str(round(in_air_transition_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='left', verticalalignment='center', color='b')
if np.amin(np.diff(airborne)) > -0.5:
    plt.text(on_ground_transition_time, 0.33, 'air to ground transition not detected', fontsize=12,
             horizontalalignment='left', verticalalignment='center', color='b')
else:
    plt.text(on_ground_transition_time, 0.33, 'on-ground at ' + str(round(on_ground_transition_time, 1)) + ' sec',
             fontsize=12, horizontalalignment='right', verticalalignment='center', color='b')
if in_air_duration > 0.0:
    plt.text((in_air_transition_time + on_ground_transition_time) / 2, 0.5,
             'duration = ' + str(round(in_air_duration, 1)) + ' sec', fontsize=12, horizontalalignment='center',
             verticalalignment='center', color='b')
# subplot for wind estimation status
plt.subplot(2, 1, 2)
plt.plot(status_time, estimating_wind, 'b')
plt.ylim(-0.1, 1.1)
plt.ylabel('estimating wind')
plt.xlabel('time (sec)')
plt.grid()
pp.savefig()
plt.close(10)    
    
# filter reported accuracy
plt.figure(13, figsize=(20, 13))
plt.title('Reported Accuracy')
plt.plot(status_time, pos_horiz_accuracy, 'b', label='horizontal')
plt.plot(status_time, pos_horiz_accuracy, 'r', label='vertical')
plt.ylabel('accuracy (m)')
plt.xlabel('time (sec')
plt.legend(loc='upper right')
plt.grid()
pp.savefig()
plt.close(13)
## Plot the EKF IMU vibration metrics

# Plot the EKF output observer tracking errors
plt.figure(15, figsize=(20, 13))
ang_track_err_max_arg = np.argmax(output_tracking_error_0)
ang_track_err_max_time = timestamp_innov[ang_track_err_max_arg]
ang_track_err_max = np.amax(output_tracking_error_0)
vel_track_err_max_arg = np.argmax(output_tracking_error_1)
vel_track_err_max_time = timestamp_innov[vel_track_err_max_arg]
vel_track_err_max = np.amax(output_tracking_error_1)
pos_track_err_max_arg = np.argmax(output_tracking_error_2)
pos_track_err_max_time = timestamp_innov[pos_track_err_max_arg]
pos_track_err_max = np.amax(output_tracking_error_2)
plt.subplot(3, 1, 1)
plt.plot(timestamp_innov, output_tracking_error_0, 'b')
plt.title('Output Observer Tracking Error Magnitudes')
plt.ylabel('angles (mrad)')
plt.grid()
plt.text(ang_track_err_max_time, ang_track_err_max, 'max=' + str(round(ang_track_err_max, 2)),
         fontsize=12, horizontalalignment='left', verticalalignment='top')
plt.subplot(3, 1, 2)
plt.plot(timestamp_innov, output_tracking_error_1, 'b')
plt.ylabel('velocity (m/s)')
plt.grid()
plt.text(vel_track_err_max_time, vel_track_err_max, 'max=' + str(round(vel_track_err_max, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='top')
plt.subplot(3, 1, 3)
plt.plot(timestamp_innov, output_tracking_error_2, 'b')
plt.ylabel('position (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(pos_track_err_max_time, pos_track_err_max, 'max=' + str(round(pos_track_err_max, 2)), fontsize=12,
         horizontalalignment='left', verticalalignment='top')
pp.savefig()
plt.show()
plt.close(15)
# Plot the delta angle bias estimates
plt.figure(16, figsize=(20, 13))
plt.subplot(3, 1, 1)
plt.plot(status_time, state_10, 'b')
plt.title('Delta Angle Bias Estimates')
plt.ylabel('X (rad)')
plt.xlabel('time (sec)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(status_time, state_11, 'b')
plt.ylabel('Y (rad)')
plt.xlabel('time (sec)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(status_time, state_12, 'b')
plt.ylabel('Z (rad)')
plt.xlabel('time (sec)')
plt.grid()
pp.savefig()
plt.close(16)
# Plot the delta velocity bias estimates
plt.figure(17, figsize=(20, 13))
plt.subplot(3, 1, 1)
plt.plot(status_time, state_13, 'b')
plt.title('Delta Velocity Bias Estimates')
plt.ylabel('X (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(status_time, state_14, 'b')
plt.ylabel('Y (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(status_time, state_15, 'b')
plt.ylabel('Z (m/s)')
plt.xlabel('time (sec)')
plt.grid()
pp.savefig()
plt.close(17)
# Plot the earth frame magnetic field estimates
plt.figure(18, figsize=(20, 13))
plt.subplot(2, 1, 2)
strength=[]
inclination=[]
i=0
for state in state_16:
    strength.append((state_16[i] ** 2 + state_17[i] ** 2 + state_18[i] ** 2) ** 0.5)
    i+=1
plt.plot(status_time, strength, 'b')
plt.ylabel('strength (Gauss)')
plt.xlabel('time (sec)')
plt.grid()
plt.subplot(2, 1, 2)
rad2deg = 57.2958
declination = rad2deg * np.arctan2(state_17, state_16)
plt.plot(status_time, declination, 'b')
plt.title('Earth Magnetic Field Estimates')
plt.ylabel('declination (deg)')
plt.xlabel('time (sec)')
plt.grid()

pp.savefig()
plt.close(18)



#读取gps数据和local数据
# plot gps pos and local pos
lat=[]
lon=[]
lines=readfile(gps_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    lat.append(float(strLine[2]))
    lon.append(float(strLine[3]))
plt.figure(21,figsize=(20,13))
plt.plot(lon,lat,'.')
plt.title('gps_pos')
plt.ylabel('lon')
plt.xlabel('lat')
plt.grid()
pp.savefig()
plt.close(21)

x=[]
y=[]
lines=readfile(lpos_file)
for line in lines:
     strLine=line.split(',')
     x.append(float(strLine[4]))
     y.append(float(strLine[5]))
    

plt.figure(22,figsize=(20,13))
plt.plot(y,x,'.')
plt.title('fileted_pos')
plt.ylabel('x')
plt.xlabel('y')
plt.grid()
pp.savefig()
plt.close(22)

evx=[]
evy=[]
lines=readfile(vision_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    evx.append(float(strLine[4]))
    evy.append(float(strLine[5]))
plt.figure(23,figsize=(20,13))
plt.plot(evy,evx,'.')
plt.title('vision_pos')
plt.ylabel('x')
plt.xlabel('y')
plt.grid()
pp.savefig()
plt.close(22)
   

# close the pdf file
pp.close()
# don't display to screen
# plt.show()
# clase all figures
plt.close("all")

    
    
    

        
