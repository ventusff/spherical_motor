# 该程序为了计算T型的旋转角度，根据的是T的上横（图像表现为粗线）
# 通过判断T的竖在横的上下来判断旋转小于180还是大于180

import cv2
import color_list
import math


#求T形旋转角，color1是粗线的颜色
def get_rotate_angle(frame, color1, color2):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_dict = color_list.get_color_list()
    img_color1_filtering = cv2.inRange(hsv, color_dict[color1][0],
                                       color_dict[color1][1])
    img_color2_filtering = cv2.inRange(hsv, color_dict[color2][0],
                                       color_dict[color2][1])

    img_color1_filtering = cv2.dilate(img_color1_filtering, None, iterations=2)
    img_color1_filtering = cv2.erode(img_color1_filtering, None, iterations=4)

    img_color2_filtering = cv2.dilate(img_color2_filtering, None, iterations=2)
    img_color2_filtering = cv2.erode(img_color2_filtering, None, iterations=4)   

    #求轮廓
    img1, cnts1, hiera1 = cv2.findContours(img_color1_filtering, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    img2, cnts2, hiera2 = cv2.findContours(img_color2_filtering, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    #拟合椭圆
    index1 = 0
    max1 = 0
    index2 = 0
    max2 = 0
    # print(len(cnts1),len(cnts2))
    if len(cnts1) > 1:
        for (points,i) in zip(cnts1,range(len(cnts1))):
            if isinstance(points,int):
                length = 1
            else:
                length = len(points)
                
            # print('1',i,length)
            
            if length > max1:
                max1 = len(points)
                index1 = i
    
    if len(cnts2) > 1:
        for (points,i) in zip(cnts2,range(len(cnts2))):
            if isinstance(points,int):
                length = 1
            else:
                length = len(points)

            # print('2: i = ',i,', length = ',length)

            if length > max2:
                max2 = len(points)
                index2 = i
                
    ellipse1 = cv2.fitEllipse(cnts1[index1])
    ellipse2 = cv2.fitEllipse(cnts2[index2])

    # print(ellipse1[2],ellipse2[2])

    #返回旋转角度,以及两条线的中点和斜率
    return ellipse1[0], ellipse2[0], 0.0 - math.tan(
            (90 - ellipse1[2]) / 180 * math.pi), 0.0 - math.tan(
            (90 - ellipse2[2]) / 180 * math.pi), ellipse1[2], ellipse2[2]


if __name__ == '__main__':
    filename = '111.jpg'
    frame = cv2.imread(filename)
    color1 = 'red'
    color2 = 'blue'
    print(get_rotate_angle(frame, color1, color2))
