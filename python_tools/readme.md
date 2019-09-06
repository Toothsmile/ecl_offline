# ecl_offlie python tools
2019.9.6 更新，今天主要给ecl_offline添加了两个脚本，分别是px4_csv2txt.py和ecl_offline_plot.py
## px4_csv2txt
主要是将px4日志*.ulg转换为csv文件，通过该脚本可转换为ecl_offline可用的txt输入文件。
使用方法，以转换123.ulg文件,输出文件到当前目录的test文件夹下为例
```
ulog2csv 123.ulg
python px4_csv2txt.py -i 123 -o  ./test/
```
需要注意的是 -i 后面跟的是ulog文件的前缀，不但后缀名。
转换成功即可在test文件夹下见到转换成功的文件

## ecl_offline
将ecl_输出的文件做出分析
该函数正在开发中，这是个开发版，其中错误较多，如果使用，能帮改帮用，感谢
使用方法
```
innovatin_file="ekf_innovations.txt"#ecl_offline残差文件
estimate_file="estimate_status.txt"#ecl_offline滤波状态文件
lpos_file="vechile_local_position.txt"#滤波出的当地坐标系的坐标文件
gps_file="gps_position_0.csv"#原始解出的gps csv文件
vision_file="vision_position_0.csv"#解出的视觉文件
output_plot_filename="innov_analyse.pdf"#输出的分析文件

```
注意：其中需要用到的文件需要去python脚本里面改一下，但是大部分功能可用，主要是模仿着官方的py分析脚本来的，但是我的bug较多，希望大家多帮改
