# 该程序是为了判断出图片中 T 型的两条线的颜色，返回如：color = ['red', 'blue']
# 思路为：用不同颜色进行过滤，求出每种颜色滤波后的面积，找到最大面积的2个确定为颜色
# 因为黑色和灰色会可能是主要颜色，故注释掉了颜色字典中的灰色和黑色

# 改进：不是任何情况下都能检测出两种颜色，必须要有这两种颜色才行，所以要排除面积过小的情况

import cv2
import numpy as np
import color_list
import heapq


def get_color(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    color = []

    color_dict = color_list.get_color_list()

    # 颜色字典的颜色数量
    num_color = len(color_dict)
    sum = [0] * num_color
    search_color_list = []

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

        if len(cnts) > 0:
            for c in cnts:
                sum[i] += cv2.contourArea(c)

        print(d, sum[i])

    find_color_list = heapq.nlargest(2, sum)
    # 0 是主色
    if find_color_list[0] > 2000 and find_color_list[1] > 1000:
        for j in range(2):
            color.append(search_color_list[sum.index(find_color_list[j])])

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
    filename = '10-21-09-39-02-271.BMP'
    frame = cv2.imread(filename)

    # 调试找颜色
    color = get_color(frame)
    if color != []:
        print('判断的2个颜色为：', ' '.join(color))
    else:
        print('没有判断出颜色')

    # 调试过滤
    color_want = 'green'
    color_filtering(frame, color_want)