//
//  EulerAngles.cpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/25.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#include "EulerAngles.hpp"
#include "Quaternion.hpp"
#include "MathUtil.h"
#include "Matrix4x3.hpp"
#include "RotationMatrix.hpp"

#include <math.h>

/*
    注意：更多的设计决策参见第11章
    欧拉角参看10.3节
 */

// 全局单位欧拉角
const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

/*
    将欧拉角转换到限制集中
    就表示3D防伪的目的而言，它不会改标欧拉角的值
    单对于其他表示对象如角速度等，则会产生影响
    参看10.3
 */
void EulerAngles::canonize() {
    // 首先，将pitch变换到-pi到pi之间
    pitch = wrapPi(pitch);
    
    // 将pitch变换到-pi/2到pi/2之间
    if (pitch < -KPiOver2) {
        pitch = -kPi - pitch;
        heading += kPi;
        bank += kPi;
    } else if (pitch > KPiOver2) {
        pitch = kPi - pitch;
        heading += kPi;
        bank += kPi;
    }
    
    // 检查万向锁的情况，允许存在一定的误差
    if (fabs(pitch) > KPiOver2 - 1e-4) {
        // 在万向锁中，将所有绕垂直轴的旋转付给heading
        heading += bank;
        bank = 0;
    } else {
        // 非万向锁，将bank转换到限制集中
        bank = wrapPi(bank);
    }
    
    // 将heading转换到限制集中
    heading = wrapPi(heading);
}

/*
    物体-惯性四元数到欧拉角
    参看10.6.6
 */
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
    // 计算sin(pitch)
    float sp = -2.0f * (q.y * q.z - q.w * q.x);
    
    // 检查万向锁，允许存在一定误差
    if (fabs(sp) > 0.9999f) {
        // 向正上方或正下方看
        pitch = KPiOver2 * sp;
        // bank置零，计算heading
        heading = atan2(-q.x * q.z + q.w * q.y, 0.5f - q.y * q.y - q.z * q.z);
        bank = 0.0f;
    } else {
        // 计算角度，这里不必使用安全的asin函数，因为之前在检查万向锁问题是已经检查过范围错误
        pitch = asin(sp);
        heading = atan2(q.x * q.z + q.w * q.y, 0.5f - q.x * q.x - q.y * q.y);
        bank = atan2(q.x * q.y + q.w * q.z, 0.5f - q.x * q.x -q.z * q.z);
    }
}

/*
    从惯性-物体四元数到欧拉角
    参看10.6.6
 */
void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q) {
    // 计算sin(pitch)
    float sp = -2.0f * (q.y * q.z + q.w * q.x);
    // 检查方向锁，允许一定误差
    if (fabs(sp) > 0.9999f) {
        // 向正上方或正下方看
        pitch = KPiOver2 * sp;
        // bank置零，计算heading
        heading = atan2(-q.x * q.z - q.w * q.y, 0.5f - q.y * q.y - q.z * q.z);
        bank = 0.0f;
    } else {
        pitch = asin(sp);
        heading = atan2(q.x * q.z - q.w * q.y, 0.5f -q.x * q.x - q.y * q.y);
        bank = atan2(q.x * q.y - q.w * q.z, 0.5f - q.x * q.x - q.z * q.z);
    }
}

/*
    从物体-世界坐标系变换矩阵到欧拉角
    假设矩阵是正交的，忽略平移部分
    参看10.6.2
 */
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m) {
    // 通过m32计算sin(pitch)
    float sp = -m.m32;
    
    // 检查万向锁
    if (fabs(sp) > 0.9999f) {
        // 向正上方或正下方看
        pitch = KPiOver2 * sp;
        // bank置零，计算heading
        heading = atan2(-m.m23, m.m11);
        bank = 0.0f;
    } else {
        // 计算角度
        heading = atan2(m.m31, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m12, m.m22);
    }
}

/*
    从世界-物体坐标系变换矩阵到欧拉角
    假设矩阵是正交的，忽略平移部分
    参看10.6.2
 */
void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m) {
    // 根据m32计算sin(picth)
    float sp = -m.m23;
    
    // 检查万向锁
    if (fabs(sp) > 0.9999f) {
        // 向正上方或正下方看
        pitch = KPiOver2 * sp;
        // back置零，计算heading
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    } else {
        heading = atan2(m.m13, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m21, m.m22);
    }
}

/*
    根绝旋转矩阵构造欧拉角
    参看10.6.2
 */
void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
    // 根据m23计算sin(pitch)
    float sp = -m.m23;
    
    // 检查万向锁
    if (fabs(sp) > 0.9999f) {
        // 向正上方或正下方看
        pitch = KPiOver2 * sp;
        // bank置零，计算heading
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    } else {
        // 计算角度
        heading = atan2(m.m13, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m21, m.m22);
    }
}

