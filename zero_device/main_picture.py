# 步骤： 1.找到T型的颜色，判断标记点标准位置
#       2.计算出T交点，并得到实际偏移
#       3.换算为球的绝对坐标
#       4.发串口消息

import math
import cv2
import numpy as np
import serial
import color_list
import judge_color
import T_rotate_angle
import point_oblique_straight_point
import mark_point_list
import judge_T
import calculate_orientation
import struct

# 设定串口
# ser = serial.Serial("COM8", 115200)

filename = '10-28-23-15-05-102.BMP'

###############################################################
# 球的标准,单位毫米mm
sphereRadius = 30

# 摄像头的分辨率
width = 560
height = 420

# 摄像头与实际坐标的换算系数
k = 26 / 480
###############################################################

frame = cv2.imread(filename)
# frame = frame[30:450, 50:610]  # 图像尺寸 560*420
frame = frame[0:480, 50:610]  # 图像尺寸 560*420
# 输出颜色
# color = judge_color.get_color(frame)
color = judge_T.judge_T(frame)
print(color)

mark_point_dict = mark_point_list.get_mark_point_list()

if color != [] and (color[0] + color[1]) in mark_point_dict.keys():

    # 找到颜色对应的标记点标准位置
    mark_point_standard_position = mark_point_dict[color[0] + color[1]]
    # 求解T型两线的方程
    index = T_rotate_angle.get_rotate_angle(frame, color[0],color[1])
    T_point1 = index[0]  # 粗线的中心点
    T_point2 = index[1]  # 细线的中心点
    distance = math.sqrt((T_point1[0] - T_point2[0])*(T_point1[0] - T_point2[0]) + (T_point1[1] - T_point2[1])*(T_point1[1] - T_point2[1]))
    if distance > 100:
        print('中心点距离不正确')
    print('两中心点距离: ',distance)
    k1 = index[2]  # 粗线的斜率
    k2 = index[3]  # 细线的斜率
    p1 = point_oblique_straight_point.Point(T_point1[0], T_point1[1])
    p2 = point_oblique_straight_point.Point(T_point2[0], T_point2[1])

    # 找到标记的中心点坐标（采用T型解方程计算）
    line1 = point_oblique_straight_point.Line(p1, k1)
    line2 = point_oblique_straight_point.Line(p2, k2)
    cross_point = point_oblique_straight_point.get_cross_point(line1, line2)
    # 右下
    angle1 = index[4]
    angle2 = index[5]
    # print('验证:',cross_point.x,cross_point.y,T_point1[0],T_point1[1],T_point2[0],T_point2[1])
    if cross_point.x >= T_point2[0] and cross_point.y >= T_point2[1]:
        # print('>>')
        rotate_angle = 180 - angle2 
    # 右上
    if cross_point.x >= T_point2[0] and cross_point.y <= T_point2[1]:
        # print('><')
        rotate_angle = 180 - angle2 
    # 左上
    if cross_point.x <= T_point2[0] and cross_point.y <= T_point2[1]:
        # print('<<')
        rotate_angle = 360 - angle2 
    # 左下
    if cross_point.x <= T_point2[0] and cross_point.y >= T_point2[1]:
        # print('<>')
        rotate_angle = 360 - angle2 


    # 计算颜色标记点在相机中的偏移
    x = cross_point.x
    y = cross_point.y

    # 对坐标进行变换，得到真实坐标
    dx = k * (x - width / 2)
    dy = k * (y - height / 2)

    # 绝对坐标用直角坐标系（X,Y,Z）【大写】表示
    X_ = dx
    Y_ = dy
    Z_ = np.sqrt(pow(sphereRadius, 2) - pow(dx, 2) - pow(dy, 2))

    X = -Y_
    Y = X_
    Z = Z_

    X0 = mark_point_standard_position[0][0]
    Y0 = mark_point_standard_position[0][1]
    Z0 = mark_point_standard_position[0][2]

    res = calculate_orientation.get_orientation(X0,Y0,Z0,X,Y,Z,rotate_angle,compensate = mark_point_standard_position[1][0])
    R = res[3]
    ThetaX = res[0]
    ThetaY = res[1]
    ThetaZ = res[2]

    # 串口发送RPY角
    # 注意Big-endian 和small-endian! 在ARM中是big-endian
    # https://www.cnblogs.com/gala/archive/2011/09/22/2184801.html
    data1 = struct.pack(">f",ThetaX)
    data2 = struct.pack(">f",ThetaY)
    data3 = struct.pack(">f",ThetaZ)
    orientation_data = bytes('^','ascii') + data1 + data2 + data3 + bytes('^\r\n','ascii')

    print(type(orientation_data),len(orientation_data),orientation_data)
    # ser.write(orientation_data)

    for byte in data1:
        print(hex(byte))
    for byte in data2:
        print(hex(byte))
    for byte in data3:
        print(hex(byte))

    print('姿态矩阵：\n',R)
    print('RPY角：',ThetaX,ThetaY,ThetaZ)
    print('RPY角度: ', ThetaX*180.0/math.pi,ThetaY*180.0/math.pi,ThetaZ*180.0/math.pi)


    # 输出坐标
    print('识别T字：',color[0],color[1])
    print('标记点标记坐标：',mark_point_standard_position[0])
    print('此时此刻坐标：{}, {}, {}'.format(X, Y, Z))
    print('旋转角度：',rotate_angle)


# ser.close()
