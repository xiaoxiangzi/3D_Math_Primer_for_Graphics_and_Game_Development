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
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
    构造任意轴缩放矩阵
    旋转轴为单位向量
    平移部分置零
    参看8.3.2
 */
void Matrix4x3::setupScaleAlongAxis(const Vector3 &axis, float k) {
    // 检查旋转轴是否为单位向量
    assert(fabs(axis * axis - 1.0f) < .01f);
    
    // 计算k-1和常用的字表达式
    float a = k -1.0f;
    float ax = a * axis.x;
    float ay = a * axis.y;
    float az = a * axis.z;
    
    // 矩阵元素赋值，这里我们完成自己的操作，因为其对角元素相等
    m11 = ax * axis.x + 1.0f;
    m22 = ay * axis.y + 1.0f;
    m33 = az * axis.z + 1.0f;
    
    m12 = m21 = ax * axis.y;
    m13 = m31 = ax * axis.z;
    m23 = m32 = ay * axis.z;
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
    构造切变矩阵
    切变类型由一个索引制定，变换效果如以下伪代码所示
    axis == 1 => y += s*x, z += t*x
    axis == 2 => x += s*y, z += t*y
    axis == 3 => x += s*y, y += t*z
 */
void Matrix4x3::setupShear(int axis, float s, float t) {
    // 判断切变类型
    switch (axis) {
        case 1:
            // 用x切变y和z
            m11 = 1.0f; m12 = s; m13 = t;
            m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
            m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
            break;
        case 2:
            // 用y切变x和z
            m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = s; m22 = 1.0f; m23 = t;
            m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
        case 3:
            // 用z切变x和y
            m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
            m31 = s; m32 = t; m33 = 1.0f;
        default:
            // 非法索引
            assert(false);
            break;
    }
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
 
    构造投影矩阵，投影平面过原点，且垂直于单位向量
    参看8.4.2
 
 */
void Matrix4x3::setupProject(const Vector3 &n) {
    // 检查旋转轴是否为单位向量
    assert(fabs(n * n - 1.0f) < .01f);
    
    m11 = 1.0f - n.x * n.x;
    m22 = 1.0f - n.y * n.y;
    m33 = 1.0f - n.z * n.z;
    
    m12 = m21 = -n.x * n.y;
    m13 = m31 = -n.x * n.z;
    m23 = m32 = -n.y * n.z;
    
    // 平移部分置零
    tx = ty = tz = 0.0f;
}

/*
    构造反射矩阵，发射平面平行于坐标平面
    反射平面由一个索引指定
    1 => 沿着x=k平面反射
    2 => 沿着y=k平面反射
    3 => 沿着z=k平面反射
    平移部分置为合适的值，因为k!=0时，平移是一定会发生的
    参看8.5
 */
void Matrix4x3::setupReflect(int axis, float k /*= 0.0f*/) {
    // 判断反射平面
    switch(axis) {
        case 1:
            // 沿着x=k平面反射
            m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
            m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
            
            tx = 2.0f * k;
            ty = 0.0f;
            tz = 0.0f;
            break;
        case 2:
            // 沿着y=k平面反射
            m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
            m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
            
            tx = 0.0f;
            ty = 2.0f * k;
            tz = 0.0f;
            break;
        case 3:
            // 沿着z=k平面反射
            m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
            m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
            m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;
            
            tx = 0.0f;
            ty = 0.0f;
            tz = 2.0f * k;
            break;
        default:
            // 非法索引
            assert(false);
            break;
    }
}

/*
    构造沿任意轴反射矩阵，反射平面为过原点的任意平面，且垂直于单位向量n
    平移部分置零
 
    参看8.5
 */
void Matrix4x3::setupReflect(const Vector3 &n) {
    // 检查旋转轴是否为单位向量
    assert(fabs(n * n - 1.0f) < .01f);
    
    // 计算公共子表达式
    float ax = -2.0f * n.x;
    float ay = -2.0f * n.y;
    float az = -2.0f * n.z;
    
    // 矩阵元素赋值，这里我们自己完成优化操作，因为其对角元素相等
    m11 = 1.0f + ax * n.x;
    m22 = 1.0f + ay * n.y;
    m33 = 1.0f + az * n.z;
    
    m12 = m21 = ax * n.y;
    m13 = m31 = ax * n.z;
    m23 = m32 = ay * n.z;
}

/*
    变换该点，
    使得使用向量类就像在纸上作线性代数一样直观
    参看7.1.7
 */
Vector3 operator* (const Vector3& p, const Matrix4x3& m) {
    return Vector3(
        p.x * m.m11 + p.y * m.m21 + p.z * m.m31 + m.tx,
        p.x * m.m12 + p.y * m.m22 + p.z * m.m32 + m.ty,
        p.x * m.m13 + p.y * m.m23 + p.z * m.m33 + m.tz);
}

Vector3& operator*= (Vector3& p, const Matrix4x3& m) {
    p = p * m;
    return p;
}

/*
    矩阵连接，使得使用矩阵类就像在纸上做线性代数一样直观
    提供*=运算符，以符合c语言的语法习惯
    参看7.1.6
 */
Matrix4x3 operator*(const Matrix4x3& a, const Matrix4x3& b) {
    Matrix4x3 r;
    
    // 计算左上的线形变换部分
    r.m11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
    r.m12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
    r.m13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;
    
    r.m21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
    r.m22 = a.m21 * b.m22 + a.m22 * b.m22 + a.m23 * b.m32;
    r.m23 = a.m21 * b.m23 + a.m22 * b.m23 + a.m23 * b.m33;
    
    r.m31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
    r.m32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
    r.m33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;
    
    r.tx = a.tx * b.m11 + a.ty * b.m21 + a.tz * b.m31 + b.tx;
    r.ty = a.tx * b.m12 + a.ty * b.m22 + a.tz * b.m32 + b.ty;
    r.tx = a.tx * b.m13 + a.ty * b.m23 + a.tz * b.m33 + b.tz;
    
    // 这种方法需要调用拷贝构造函数，如果速度非常重要，可能要用单独的函数在期望的地方给出返回值
    return r;
}

Matrix4x3& operator*=(Matrix4x3& a, const Matrix4x3& b) {
    a = a * b;
    return a;
}

/*
    计算矩阵左上3x3部分的行列式
    参看9.1.1
 */
float determinant(const Matrix4x3& m) {
    return
        m.m11 * (m.m22 * m.m33 - m.m23 * m.m32)
        + m.m12 * (m.m23 * m.m31 - m.m21 * m.m33)
        + m.m13 * (m.m21 * m.m32 - m.m22 * m.m31);
}

/*
    求矩阵的逆，使用经典的伴随矩阵除以行列式的方法
    参看9.2.1
 */
Matrix4x3 inverse(const Matrix4x3& m) {
    // 计算行列式
    float det = determinant(m);
    
    // 如果是奇异的，即行列式为0，没有逆矩阵
    assert(fabs(det) > .000001f);
    
    // 计算1/行列式
    float oneOverDet = 1.0f / det;
    
    Matrix4x3 r;
    
    // 计算3x3部分的逆
    r.m11 = (m.m22 * m.m33 - m.m23 * m.m32) * oneOverDet;
    r.m12 = (m.m13 * m.m32 - m.m12 * m.m33) * oneOverDet;
    r.m13 = (m.m12 * m.m23 - m.m13 * m.m22) * oneOverDet;
    
    r.m21 = (m.m23 * m.m31 - m.m21 * m.m33) * oneOverDet;
    r.m22 = (m.m11 * m.m31 - m.m13 * m.m31) * oneOverDet;
    r.m23 = (m.m13 * m.m21 - m.m11 * m.m23) * oneOverDet;
    
    r.m31 = (m.m21 * m.m32 - m.m22 * m.m31) * oneOverDet;
    r.m32 = (m.m12 * m.m31 - m.m11 * m.m32) * oneOverDet;
    r.m33 = (m.m11 * m.m22 - m.m12 * m.m21) * oneOverDet;
    
    // 计算平移部分的逆
    r.tx = -(m.tx * r.m11 + m.ty * r.m21 + m.tz * r.m31);
    r.ty = -(m.tx * r.m12 + m.ty * r.m22 + m.tz * r.m32);
    r.tz = -(m.tx * r.m13 + m.ty * r.m23 + m.tz * r.m33);
    
    // 这种方法需要调用拷贝构造函数，如果速度非常重要，可能要用单独的函数在期望的地方给出返回值
    return r;
}

// 以向量的形式返回平移部分
Vector3 getTranslation(const Matrix4x3& m) {
    return Vector3(m.tx, m.ty, m.tz);
}

/*
    从父->局部（如世界->物体）变换矩阵中提取物体的位置
    假设矩阵代表刚体变换
 */
Vector3 getPositionFromParentToLocal(const Matrix4x3& m) {
    // 负的平移值乘以3*3部分的转置
    // 假设矩阵是正交的（该方法不能应用于非刚体变换）
    return Vector3(-(m.tx * m.m11 + m.ty * m.m12 + m.tz * m.m13),
                   -(m.tx * m.m21 + m.ty * m.m22 + m.tz * m.m23),
                   -(m.tx * m.m31 + m.ty * m.m32 + m.tz * m.m33));
}

/*
    从局部->父（如物体->世界）变换矩阵中提取物体的位置
 */
Vector3 getPositionFromLocalToParent(const Matrix4x3& m) {
    // 所需的e位置就是平移部分
    return Vector3(m.tx, m.ty, m.tz);
}
