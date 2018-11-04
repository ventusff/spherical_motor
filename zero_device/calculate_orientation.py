import math
from numpy import *


def fake_cartesian_to_spherical(X, Y, Z):
    r = 30
    phi = math.atan2(Y, X)
    if abs(Z) > r:
        Z = Z / abs(Z) * r
    theta = math.acos(Z/r)
    return r, phi, theta

def axis_angle_To_M(axis,angle):
    axis = axis.tolist()
    rx = axis[0][0]
    ry = axis[1][0]
    rz = axis[2][0]
    c_ = math.cos(angle)
    s_ = math.sin(angle)
    # 由轴角推到旋转矩阵
    M = mat(zeros((3, 3)))
    M[0, 0] = rx * rx * (1 - c_) + c_
    M[0, 1] = rx * ry * (1 - c_) + rz * s_
    M[0, 2] = rx * rz * (1 - c_) - ry * s_
    M[1, 0] = ry * rx * (1 - c_) - rz * s_
    M[1, 1] = ry * ry * (1 - c_) + c_
    M[1, 2] = ry * rz * (1 - c_) + rx * s_
    M[2, 0] = rz * rx * (1 - c_) + ry * s_
    M[2, 1] = rz * ry * (1 - c_) - rx * s_
    M[2, 2] = rz * rz * (1 - c_) + c_
    return M

def get_orientation(X0, Y0, Z0, X, Y, Z, rotate_angle, compensate = 0.0):

    # 视野：-y 是正向0度；定义：x正向0度差距90度
    # 因此对于正常向上的T，应为w + 90 - phi
    compensate  = compensate / 180.0 * math.pi + math.pi/2

    r = 30
    rotate_angle = rotate_angle * math.pi / 180
    res0 = fake_cartesian_to_spherical(X0, Y0, Z0)
    phi0 = res0[1]
    theta0 = res0[2]

    res = fake_cartesian_to_spherical(X, Y, Z)
    phi = res[1]
    theta = res[2]

    print("phi,theta = ",phi*180.0/math.pi,theta*180.0/math.pi,' phi0,theta0 = ',phi0*180.0/math.pi,theta0*180.0/math.pi)

    R = mat(zeros((3, 3)))
    sintheta = math.sin(theta)
    sintheta0 = math.sin(theta0)
    sinphi = math.sin(phi)
    sinphi0 = math.sin(phi0)
    costheta = math.cos(theta)
    costheta0 = math.cos(theta0)
    cosphi = math.cos(phi)
    cosphi0 = math.cos(phi0)
    # 绕z旋转-phi0
    R[0, 0] = cosphi0
    R[0, 1] = sinphi0
    R[0, 2] = 0
    R[1, 0] = 0.0 - sinphi0
    R[1, 1] = cosphi0
    R[1, 2] = 0
    R[2, 0] = 0
    R[2, 1] = 0
    R[2, 2] = 1
    # 经过第一次旋转后的姿态矩阵
    #　R1 = R.T
    R1 = R #important! 这里确定就应该使用不转置的矩阵！
    # 因为这里确实相当于直接确定了坐标系的旋转！没有什么“向量旋转”“坐标旋转的区别”

    # 绕y旋转-theta0
    # 对于R1姿态矩阵，因为仅仅只是坐标系发生了旋转，所以这里需要的操作是求在新系下的坐标，求法为 新坐标 = 姿态矩阵 * 旧坐标
    # 而 x,y,z 轴的旧坐标可以直接合成单位阵（每一列即为一轴），故新坐标即为姿态矩阵每一列
    axis = R1[:,1]
    R2 = axis_angle_To_M(axis,0.0-theta0) * R1

    coor = mat(zeros((3,1)))
    coor[0,0] = X0
    coor[1,0] = Y0
    coor[2,0] = Z0
    # print('坐标：',R2.T*coor)

    # 绕y旋转theta
    axis = R2[:,1]
    R3 = axis_angle_To_M(axis,theta) * R2


    # 绕z旋转phi
    axis = R3[:,2]
    R4 = axis_angle_To_M(axis,phi) * R3

    # 绕 0-X,Y,Z 旋转 w - phi
    axis0 = mat(zeros((3,1)))
    axis0[0,0] = X/r
    axis0[1,0] = Y/r
    axis0[2,0] = Z/r
    axis = R4 * axis0
    # 最终的姿态矩阵
    R_final = axis_angle_To_M(axis,rotate_angle - phi + compensate) * R4
    # R_final = R4

    # 由旋转矩阵 到 RPY值
    ThetaZ = math.atan2(R_final[1,0],R_final[0,0])
    sinThetaZ = math.sin(ThetaZ)
    cosThetaZ = math.cos(ThetaZ)
    ThetaY = math.atan2(0.0 - R_final[2,0], cosThetaZ * R_final[0,0] + sinThetaZ * R_final[1,0])
    ThetaX = math.atan2(sinThetaZ * R_final[0,2] - cosThetaZ * R_final[1,2],cosThetaZ * R_final[1,1] - sinThetaZ * R_final[0,1])

    return ThetaX,ThetaY,ThetaZ, R_final.tolist()



if __name__ == '__main__':
    # R = get_orientation(0,0,30,0,0,30,90)
    # R = get_orientation(0,0,30,12.6221,-14.3828,23.1045,90)
    # R = get_orientation(0,0,30,12.6221,-14.3828,23.1045, -48.73036361456043)
    res = get_orientation(0,0,30,0,-25.2441,16.2091, 10)
    R = res[3]
    print(R)
    
    print(res[0],res[1],res[2])
