//
//  Matrix4x3.cpp
//  3dmath
//
//  Created by xiaoxiangzi-imac on 2020/1/19.
//  Copyright © 2020 xiaoxiangzi. All rights reserved.
//

#include "Matrix4x3.hpp"
#include "Vector3.hpp"
#include "EulerAngles.hpp"
#include "Quaternion.hpp"
#include "RotationMatrix.hpp"
#include "MathUtil.h"

#include <assert.h>
#include <math.h>

void Matrix4x3::identity() {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
    tx = 0.0f; ty = 0.0f; tz = 0.0f;
}

// 将包含平移的部分置为0
void Matrix4x3::zeroTranslation() {
    tx = ty = tz = 0.0f;
}

// 平移部分赋值
void Matrix4x3::setupTranslation(const Vector3 &d) {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
    tx = d.x; ty = d.y; tz = d.z;
}

/*
    构造进行局部->父空间变换的矩阵，局部空间的位置和方位在父空间中描述
    该方法最常见的用途是构造物体->世界的变换矩阵，这个变换是非常直接的
    首先从物体空间变换到惯性空间，接着变换到世界空间
    方位可以由欧拉角或者旋转矩阵指定
 */
void Matrix4x3::setupLocalToParent(const Vector3 &pos, const EulerAngles &oriant) {
    RotationMatrix oriantMatrix;
    oriantMatrix.setup(oriant);
    
    // 构造4x3矩阵。注意：如果速度十分重要，我们可以直接计算矩阵，不用RotationMatrix临时对象，节省一次函数调用和一些复制操作
    setupLocalToParent(pos, oriantMatrix);
}

void Matrix4x3::setupLocalToParent(const Vector3 &pos, const RotationMatrix &oriant) {
    /*
        复制矩阵的旋转部分
        根据RotationMatrix中的注释，旋转矩阵“一般“是惯性->物体矩阵，是父->局部关系
        我们求的是局部->父关系的矩阵，因此要做转置
     */
    m11 = oriant.m11; m12 = oriant.m21; m13 = oriant.m31;
    m21 = oriant.m12; m22 = oriant.m22; m23 = oriant.m32;
    m31 = oriant.m13; m12 = oriant.m23; m13 = oriant.m33;
    
    // 现在设置平移部分，平移在3x3部分之后，只需要简单复制即可
    tx = pos.x; ty = pos.y; tz = pos.z;
}

/*
    执行父->局部空间变换的矩阵，局部空间的位置和方位在父空间的描述
    该方法最常见的用途是构造世界->物体的变换矩阵
    通常这个变换首先从世界空间转换到惯性空间，接着转换到物体空间
    4x3矩阵可以完成后一个转换
    所以我们想构造两个矩阵T和R，再连接M=TR
    方位可以由欧拉角或旋转矩阵指定
 */
void Matrix4x3::setupParentToLocal(const Vector3 &pos, const EulerAngles &oriant) {
    RotationMatrix orientMatrix;
    orientMatrix.setup(oriant);
    
    setupParentToLocal(pos, orientMatrix);
}

void Matrix4x3::setupParentToLocal(const Vector3 &pos, const RotationMatrix &oriant) {
    // 复制矩阵的旋转部分。可以直接复制元素（不用转置），根据RotationMatrix中注释的排列方式即可
    m11 = oriant.m11; m12 = oriant.m12; m13 = oriant.m13;
    m21 = oriant.m21; m22 = oriant.m22; m23 = oriant.m23;
    m31 = oriant.m31; m32 = oriant.m32; m33 = oriant.m33;
    
    /*
        设置平一部分
        一般来说，从世界空间到惯性空间只需要平移负坐标的量
        但是旋转是先发生的，所以应该旋转平移的部分
        这和先创建平移-pos的矩阵T，在创建旋转矩阵R，再把它们连接成TR是一样的
     */
    tx = -(pos.x * m11 + pos.y * m21 + pos.z *m31);
    ty = -(pos.x * m12 + pos.y * m22 + pos.z * m32);
    tz = -(pos.x * m13 + pos.y * m23 + pos.z * m33);
}

/*
    构造绕坐标轴旋转的矩阵
    旋转轴由一个从1开始的索引指定：1：x轴，2：y轴，3：z轴
    theta是旋转量，以弧度表示，用左手法则定义正方向，平移部分置零
    参看8.2.2
 */
void Matrix4x3::setupRotate(int axis, float theta) {
    // 取得旋转角的sin和cos值
    float s, c;
    sinCos(&s, &c, theta);
    
    switch (axis) {
        case 1: // 绕x轴旋转
            m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = 0.0f; m22 = c; m23 = s;
            m31 = 0.0f; m32 = -s; m33 = c;
            break;
        case 2: // 绕y轴旋转
            m11 = c; m12 = 0.0f; m13 = -s;
            m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
            m31 = s; m32 = 0.0f; m33 = c;
            break;
        case 3: // 绕z轴旋转
        m11 = c; m12 = s; m13 = 0.0f;
        m21 = -s; m22 = c; m23 = 0.0f;
        m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
        break;
        default:
            // 非法索引
            assert(false);
            break;
    }
    
    tx = ty = tz = 0.0f;
}

/*
    构造绕任意轴的旋转，旋转轴通过原点
    旋转轴为单位向量
    theta是旋转的量，以弧度显示，用左手法则来定义正方向
    平移部分置零
    参看8.3.3
*/
void Matrix4x3::setupRotate(const Vector3 &axis, float theta) {
    // 单位向量检查
    assert(fabs(axis * axis - 1.0f) < .01f);
    
    // 取得旋转角的sin和cos值
    float s, c;
    sinCos(&s, &c, theta);
    
    // 计算1-cos(theta)和一些公用的子表达式
    float a = 1.0f - c;
    float ax = a * axis.x;
    float ay = a* axis.y;
    float az = a * axis.z;
    
    // 矩阵元素赋值，仍有优化机会，因为有许多相同的子表达式，我们把这个任务交给编译器
    m11 = ax * axis.x + c;
    m12 = ax * axis.y + axis.z * s;
    m13 = ax * axis.z - axis.y * s;
    
    m21 = ay * axis.x - axis.z * s;
    m22 = ay * axis.y + c;
    m23 = ay * axis.z + axis.x * s;
    
    m31 = az * axis.x + axis.y * s;
    m32 = az * axis.y - axis.x * s;
    m33 = az * axis.z + c;
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
    从四元数转换到矩阵
    平移部分置零
    参看10.6.3

 */
void Matrix4x3::fromQuaternion(const Quaternion &q) {
    float ww = 2.0f * q.w;
    float xx = 2.0f * q.x;
    float yy = 2.0f * q.y;
    float zz = 2.0f * q.z;
    
    // 矩阵元素分赋值
    m11 = 1.0f - yy * q.y - zz * q.z;
    m12 = xx * q.y + ww * q.z;
    m13 = xx * q.z - ww * q.z;
    
    m21 = xx * q.y - ww * q.z;
    m22 = 1.0f - xx * q.x - zz * q.z;
    m23 = yy * q.z + ww * q.x;
    
    m31 = xx * q.z + ww * q.y;
    m32 = yy * q.z - ww * q.x;
    m33 = 1.0f - xx * q.x - yy * q.y;
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
    构造沿各坐标轴缩放的矩阵
    对于缩放因子k，使用向量Vector3(k,k,k)表示
    平移部分置零
    参看8.3.1
 */
void Matrix4x3::setupScale(const Vector3 &s) {
    // 矩阵元素赋值
    m11 = s.x; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = s.y; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = s.z;
    
    tx = ty = tz = 0.0f;
}
