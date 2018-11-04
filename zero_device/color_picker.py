# 拾色器
# 该程序是为了拾取颜色，可以输出BGR、HSV、GRAY等格式

import cv2


# 定义鼠标交互函数
def mouseColor(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        print('HSV:', hsv[y, x])  #输出图像坐标(x,y)处的HSV的值


# 主函数
def main():
    cv2.namedWindow("Color Picker")
    cv2.setMouseCallback("Color Picker", mouseColor)
    cv2.imshow("Color Picker", img)
    if cv2.waitKey(0):
        cv2.destroyAllWindows()


if __name__ == '__main__':
    img = cv2.imread('11-03-18-40-54-076.BMP')  #读进来是BGR格式
    # img = cv2.imread('10-21-11-39-47-660.BMP')
    # img = cv2.imread('./colortest/10-20-10-07-03-702.BMP') #蓝绿白
    # img = cv2.imread('./colortest/10-20-10-07-27-839.BMP') # 棕色 粉红色
    # img = cv2.imread('./colortest/10-20-10-07-51-480.BMP') # 红色 黄色
    # img = cv2.imread('./colortest/10-20-10-36-46-046.BMP') # 紫色
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #变成HSV格式
    main()