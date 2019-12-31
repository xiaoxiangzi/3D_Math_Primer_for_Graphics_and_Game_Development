//
//  RotationMatrix.cpp
//  3dmath
//
//  Created by xiaoxiangzi-imac on 2019/12/31.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#include "RotationMatrix.hpp"

#include "Vector3.hpp"
#include "MathUtil.h"
#include "Quaternion.hpp"
#include "EulerAngles.hpp"

// 置为单位矩阵
void RotationMatrix::identity() {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
}

// 用欧拉角参数构造矩阵，10.6.1节
void RotationMatrix::setup(const EulerAngles &orientation) {
    // 计算角度的sin和cos值head-pitch-bank
    float sh, ch, sp, cp, sb, cb;
    sinCos(&sh, &ch, orientation.heading);
    sinCos(&sp, &cp, orientation.pitch);
    sinCos(&sb, &cb, orientation.bank);
    
    // 填充矩阵
    m11 = ch * cb + sh * sp * sb;
    m12 = -ch * sb + sh * sp * cb;
    m13 = sh * cp;
    
    m21 = sb * cp;
    m22 = cb * cp;
    m23 = -sp;
    
    m31 = -sh * cb + ch * sp * sb;
    m32 = sb * sh + ch * sp * cb;
    m33 = ch * cp;
}


// 根据惯性-物体旋转四元数构造矩阵，10.6.3节
void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion &q) {
    // 优化空间：相同子表达式计算一遍即可
    m11 = 1.0f -2.0f * (q.y * q.y + q.z * q.z);
    m12 = 2.0f * (q.x * q.y + q.w * q.z);
    m13 = 2.0f * (q.x * q.z - q.w * q.y);
    
    m21 = 2.0f * (q.x * q.y - q.w * q.z);
    m22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
    m23 = 2.0f * (q.y * q.z + q.w * q.x);
    
    m31 = 2.0f * (q.x * q.z + q.w * q.y);
    m32 = 2.0f * (q.y * q.z - q.w * q.x);
    m33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
}

// 根据物体-惯性旋转的四元数构造矩阵，10.6.3节
void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion &q) {
    // 优化空间：相同子表达式计算一遍即可
    m11 = 1.0f -2.0f * (q.y * q.y + q.z * q.z);
    m12 = 2.0f * (q.x * q.y - q.w * q.z);
    m13 = 2.0f * (q.x * q.z + q.w * q.y);
    
    m21 = 2.0f * (q.x * q.y + q.w * q.z);
    m22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
    m23 = 2.0f * (q.y * q.z - q.w * q.x);
    
    m31 = 2.0f * (q.x * q.z - q.w * q.y);
    m32 = 2.0f * (q.y * q.z + q.w * q.x);
    m33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
}

/**
    
    对向量做惯性-物体变换
    列向量形式：
    | m11 m21 m31 ||ix|   |ox|
    | m12 m22 m32 ||iy| = |oy|
    | m13 m23 m33 ||iz|   |oz|
    行向量形式：
               [m11 m12 m13]
    [ix iy iz] [m21 m22 m23] = [ox oy oz]
               [m31 m32 m33]
    
 */
Vector3 RotationMatrix::inertialToObject(const Vector3 &v) const {
    return Vector3(m11 * v.x + m21 * v.y + m31 * v.z,
                   m12 * v.x + m22 * v.y + m32 * v.z,
                   m13 * v.x + m23 * v.y + m33 * v.z);
}

/**
    对向量做物体-惯性变换
    列向量形式：
    | m11 m12 m13 ||ox|   |ix|
    | m21 m22 m23 ||oy| = |iy|
    | m31 m32 m33 ||oz|   |iz|
    行向量形式：
               [m11 m21 m31]
    [ox oy oz] [m12 m22 m32] = [ix iy iz]
               [m13 m23 m33]
 */
Vector3 RotationMatrix::objectToInertial(const Vector3 &v) const {
    return Vector3(m11 * v.x + m12 * v.y + m13 * v.z,
                   m21 * v.x + m22 * v.y + m23 * v.z,
                   m31 * v.x + m32 * v.z + m33 * v.z);
}
