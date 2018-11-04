import cv2
import numpy as np
import color_list
import heapq
import math

def judge_T(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    color = []

    color_dict = color_list.get_color_list()

    # 颜色字典的颜色数量
    num_color = len(color_dict)
    sum = [0] * num_color
    sum_tuple_list = []
    search_color_list = []

    ellipses = []

    # 对每个颜色进行判断，d是颜色字符串（如：red）
    for (d, i) in zip(color_dict, range(num_color)):

        search_color_list.append(d)
        # 根据阈值构建掩膜
        mask = cv2.inRange(hsv, color_dict[d][0], color_dict[d][1])

        # 先膨胀再腐蚀
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=4)

        img, cnts, hiera = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
        # 有轮廓才进行后面的判断
        # 把后面的判断放到这里了
        # 找到所有轮廓中最大的
        index1 = 0
        max1 = 0
        if len(cnts) > 1:
            for (points,ii) in zip(cnts,range(len(cnts))):
                if isinstance(points,int):
                    length = 1
                else:
                    length = len(points)
                    
                # print(ii,length)
                
                if length > max1:
                    max1 = len(points)
                    index1 = ii

        # 找到最大的轮廓的中心点，并存储
        if len(cnts) > 0 and len(cnts[index1]) > 5:
            sum[i] = cv2.contourArea(cnts[index1])
            #fitEllipse返回一个列表，0——椭圆中心点，1——长短轴，2——旋转角度
            ellipses.append(cv2.fitEllipse(cnts[index1]))
        else:
            ellipses.append([])
            sum[i] = 0
        sum_tuple_list.append((i,sum[i]))

        #print(d, sum[i])

    # find_color_list = heapq.nlargest(2, sum)
    # print(type(sum_tuple_list))
    # sum_tuple_list = tuple(sum_tuple_list)
    # print(type(sum_tuple_list))
    # print(sum_tuple_list)
    sum_tuple_list = sorted(sum_tuple_list, key=lambda item: item[1], reverse = True)
    # print(sum_tuple_list)

    main_colors = []
    sub_colors = []
    '''
    for i,color_area in sum_tuple_list:
        if color_area > 50.0:
            if color_area > 2000.0:
                main_colors.append(i)
            else:
                sub_colors.append(i)
                '''
    for i,color_area in sum_tuple_list:
        if color_area > 50.0:
            if ellipses[i][1][1] < 120.0 and ellipses[i][1][1] > 40.0:
                sub_colors.append(i)
            #if color_area > 1200.0:
            if ellipses[i][1][1] > 120.0:
                main_colors.append(i)
                
    
    sub_color_used = [False] * num_color
    if len(main_colors) > 0:
        for main_color_idx in main_colors:
            for sub_color_idx in sub_colors:
                if not sub_color_used[sub_color_idx] and main_color_idx != sub_color_idx:
                    T_point_main = ellipses[main_color_idx][0]
                    T_point_sub = ellipses[sub_color_idx][0]
                    distance = math.sqrt((T_point_main[0] - T_point_sub[0])*(T_point_main[0] - T_point_sub[0]) + (T_point_main[1] - T_point_sub[1])*(T_point_main[1] - T_point_sub[1]))
                    # 实测真正T字的中心点距离在50 ~ 60 之间，极端情况下会有更小
                    print('main',search_color_list[main_color_idx],ellipses[main_color_idx][1])
                    print('sub',search_color_list[sub_color_idx],ellipses[sub_color_idx][1])
                    print(search_color_list[main_color_idx],search_color_list[sub_color_idx],distance)
                    if distance < 100.0 and distance > 25.0:
                        color.append(search_color_list[main_color_idx])
                        color.append(search_color_list[sub_color_idx])
                        sub_color_used[sub_color_idx] = True
                        sub_color_used[main_color_idx] = True
                        break
            else:
                continue
            continue

    return color


def color_filtering(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_dict = color_list.get_color_list()

    mask = cv2.inRange(hsv, color_dict[color][0], color_dict[color][1])

    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.erode(mask, None, iterations=4)

    cv2.imwrite(filename + color + '.jpg', mask)


if __name__ == '__main__':

    # 设定要检测的图片
    # filename = '10-21-11-39-47-660.BMP'
    filename = '11-03-18-36-21-898.BMP'
    # filename = '10-21-09-39-02-271.BMP'
    # filename = '10-21-12-00-39-273.BMP'
    frame = cv2.imread(filename)
    frame = frame[0:480, 50:610]

    # 调试找颜色
    color = judge_T(frame)
    if color != []:
        print('判断的2个颜色为：', ' '.join(color))
    else:
        print('没有判断出颜色')

    # 调试过滤
    color_want = 'green'
    color_filtering(frame, color_want)