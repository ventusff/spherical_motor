import numpy as np
import collections
 
#定义字典存放颜色分量上下限
#例如：{颜色: [min分量, max分量]}
#{'red': [array([160,  43,  46]), array([179, 255, 255])]}
 
def get_color_list():
    dict = collections.defaultdict(list)
 
    # 黑色
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 46])
    color_list = []
    color_list.append(lower_black)
    color_list.append(upper_black)
    dict['black'] = color_list
 
    #灰色
    # lower_gray = np.array([0, 0, 46])
    # upper_gray = np.array([180, 43, 220])
    # color_list = []
    # color_list.append(lower_gray)
    # color_list.append(upper_gray)
    # dict['gray']=color_list
 
    # 白色
    # lower_white = np.array([0, 0, 221])
    # upper_white = np.array([180, 30, 255])
    # color_list = []
    # color_list.append(lower_white)
    # color_list.append(upper_white)
    # dict['white'] = color_list
 
    #红色
    lower_red = np.array([0, 150, 43])
    upper_red = np.array([17, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red']=color_list
    '''
    # 未采用
    lower_red = np.array([155, 131, 150])
    upper_red = np.array([162, 165, 180])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['unknown'] = color_list
    '''
 
    #粉色
    lower_pink = np.array([159, 46, 46])
    upper_pink = np.array([166, 255, 255])
    color_list = []
    color_list.append(lower_pink)
    color_list.append(upper_pink)
    dict['pink'] = color_list
 
    #黄色
    lower_yellow = np.array([24, 46, 46])
    upper_yellow = np.array([31, 255, 172])
    color_list = []
    color_list.append(lower_yellow)
    color_list.append(upper_yellow)
    dict['yellow'] = color_list
 
    #绿色
    lower_green = np.array([51, 64, 43])
    upper_green = np.array([96, 255, 130])
    color_list = []
    color_list.append(lower_green)
    color_list.append(upper_green)
    dict['green'] = color_list
 
    #棕色
    # lower_brown = np.array([0, 40, 43])
    # upper_brown = np.array([19, 139, 149])
    # color_list = []
    # color_list.append(lower_brown)
    # color_list.append(upper_brown)
    # dict['brown'] = color_list
 
    #蓝色
    lower_blue = np.array([104, 46, 46])
    upper_blue = np.array([117, 255, 255])
    color_list = []
    color_list.append(lower_blue)
    color_list.append(upper_blue)
    dict['blue'] = color_list
 
    # 紫色
    lower_purple = np.array([118, 46, 46])
    upper_purple = np.array([133, 255, 255])
    color_list = []
    color_list.append(lower_purple)
    color_list.append(upper_purple)
    dict['purple'] = color_list
 
    return dict
 
 
if __name__ == '__main__':
    color_dict = get_color_list()
    print(color_dict)
 
    num = len(color_dict)
    print('num=',num)
 
    for d in color_dict:
        print('key=',d)
        print('value=',color_dict[d][1])
