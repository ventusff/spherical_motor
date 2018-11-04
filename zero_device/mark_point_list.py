# 标记点的字典

import collections
import math

#定义标记点的字典
#例如：{颜色: 点坐标}
#{'redblue': [0,0,r]}


def get_mark_point_list():
    dict = collections.defaultdict(list)

    r = 30
    angle = math.asin(math.sqrt(2.0/3.0))
    sin = math.sqrt(2.0/3.0) * r
    cos = math.cos(angle) * r
    sin45 = math.sin(math.pi/4.0) * r


    mark_point = []
    mark_point.append([0, 0, r])  # 北极点
    mark_point.append([0.0])    #补充角度
    dict['redblue'] = mark_point

    mark_point = []
    mark_point.append([0, 0, -r])  # 南极点
    mark_point.append([180])
    dict['redpink'] = mark_point

    mark_point = []
    # (r, 0, theta)
    mark_point.append([sin, 0, cos])  # 上半球
    mark_point.append([0])
    dict['greenblue'] = mark_point

    mark_point = []
    # (r, pi/2, theta)
    mark_point.append([0, sin, cos])
    mark_point.append([0])
    dict['greenyellow'] = mark_point

    mark_point = []
    # (r, pi, theta)
    mark_point.append([-sin, 0, cos])
    mark_point.append([0])
    dict['greenpink'] = mark_point

    mark_point = []
    # (r, 3pi/2, theta)
    mark_point.append([0, -sin, cos])
    mark_point.append([0])
    dict['greenpurple'] = mark_point

    mark_point = []
    # (r, 0, pi-theta)
    mark_point.append([sin, 0, -cos])  # 下半球
    mark_point.append([180])
    dict['blackblue'] = mark_point

    mark_point = []
    mark_point.append([0, sin, -cos])
    mark_point.append([180])
    dict['blackyellow'] = mark_point

    mark_point = []
    mark_point.append([-sin, 0, -cos])
    mark_point.append([180])
    dict['blackpink'] = mark_point

    mark_point = []
    mark_point.append([0, -sin, -cos])
    mark_point.append([180])
    dict['blackpurple'] = mark_point

    mark_point = []
    # (r, pi/4, pi/2)
    mark_point.append([sin45, sin45, 0])  # 赤道
    mark_point.append([0])
    dict['greenred'] = mark_point

    mark_point = []
    mark_point.append([-sin45, sin45, 0])
    mark_point.append([0])
    dict['purplered'] = mark_point

    mark_point = []
    mark_point.append([-sin45, -sin45,0])
    mark_point.append([0])
    dict['blackred'] = mark_point

    mark_point = []
    mark_point.append([sin45, -sin45, 0])
    mark_point.append([0])
    dict['greenblack'] = mark_point

    return dict


if __name__ == '__main__':
    mark_point_dict = get_mark_point_list()
    print(mark_point_dict)
    num = len(mark_point_dict)
    print('num=',num)
