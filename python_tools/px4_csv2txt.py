# -*- coding: utf-8 -*-
"""
Created on Thu Sep  5 23:33:06 2019

@author: toothsmile,CQU 
@email:  767200403@qq.com
"""

import sys,getopt
import os

def mkdir(path):
    # 去除首位空格
    path=path.strip()
    # 去除尾部 \ 符号
    path=path.rstrip("\\") 
    # 判断路径是否存在
    # 存在     True
    # 不存在   False
    isExists=os.path.exists(path) 
    # 判断结果
    if not isExists:
        # 如果不存在则创建目录
        # 创建目录操作函数
        os.makedirs(path)  
        print path+' 创建成功'
        return True
    else:
        # 如果目录存在则不创建，并提示目录已存在
        print path+' 目录已存在'
        return False
        
        
def readfile(fname):
    with open(fname) as textdata:
        lines=textdata.readlines()
    return lines 
def outfile(fname,line):
    with open(fname,"a")as wf:
        wf.write(line)
argv= sys.argv[1:]       
inputfile = ''
outputfile = ''
try:
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
except getopt.GetoptError:
    print 'csv2txt_file.py -i <inputfilehead> -o <outputfilehead>'
    sys.exit(2)
for opt, arg in opts:
    if opt == '-h':
        print 'csv2txt_file.py -i <inputfilehead> -o <outputfilehead>'
        sys.exit()
    elif opt in ("-i", "--ifile"):
        inputfile = arg
    elif opt in ("-o", "--ofile"):
        outputfile = arg
print '输入的文件夹+文件名前缀为：', inputfile
print '输出的文件夹为：', outputfile

mkdir(outputfile)
#处理ulog2csv生成的文件作为ecl_offline的输入
#主要转换gps，imu，mag，vision数据
#ecl的输入要求（1）是数据之间不能是，需要是空格（2）去掉首行(3)处理时间：必须处理成imu的时间最早（由于ecl_offline中的数据输入设计）
#file_head="03_19_48_vision_rtk_fuse"
file_head=inputfile
imu_file=file_head+"_sensor_combined_0.csv"
gps_file=file_head+"_vehicle_gps_position_0.csv"
air_file=file_head+"_vehicle_air_data_0.csv"
mag_file=file_head+"_vehicle_magnetometer_0.csv"
vision_pos_file=file_head+"_vehicle_vision_position_0.csv"
vision_att_file=file_head+"_vehicle_vision_attitude_0.csv"

#outfile_head=''
outfile_head=outputfile
out_imu=outfile_head+'imu.txt'
out_gps=outfile_head+'gps.txt'
out_gps2=outfile_head+'gps2.txt'
out_mag=outfile_head+'mag.txt'
out_vision_pos=outfile_head+'vision_pos.txt'
out_vision_att=outfile_head+"vision_att.txt"
out_air=outfile_head+"baro.txt"
#handle imu
lines=[]

lines=readfile(imu_file)
imu_time_first=float(lines[1].split(',')[0])
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_imu,line2str)

#handle gps data
lines=readfile(gps_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    if(float(strLine[0])<imu_time_first):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_gps,line2str)
#handle gps2 data
lines=readfile(gps_file)
#time_end=(lines[len(lines)-1].split(','))[0]
for line in lines:
    print(line)
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    if(int(strLine[0])>=153376210 and int(strLine[0])<=153376210+8*1e6):#从153376210开始搞10个历元无数据
        print(strLine[0])
        strLine[3]='0';strLine[4]='0';strLine[23]='1'        
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_gps2,line2str)

#handle mag data
lines=readfile(mag_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    #print(strLine[0])
    if(float(strLine[0])<imu_time_first):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_mag,line2str)
    
#handle vision data
lines=readfile(vision_pos_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    if(float(strLine[0])<imu_time_first):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_vision_pos,line2str)

lines=readfile(vision_att_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    if(float(strLine[0])<imu_time_first):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_vision_att,line2str)
    
#handle baro data
lines=readfile(air_file)
for line in lines:
    strLine=line.split(',')
    if(strLine[0]=="timestamp"):
        continue
    if(float(strLine[0])<imu_time_first):
        continue
    line2str=''
    line2str+=(strLine[0])
    for str_data in strLine:
        if(str_data==strLine[0]):
            continue
        line2str+=(' ')
        line2str+=(str_data)   
    outfile(out_air,line2str)   
