//
//  Quaternion.cpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/25.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#include "Quaternion.hpp"

#include <assert.h>
#include <math.h>

#include "MathUtil.h"
#include "Vector3.hpp"
#include "EulerAngles.hpp"

// 全剧数据

// 全局单位四元数
const Quaternion kQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};

void Quaternion::setToRotateAboutX(float theta) {
    // 计算半角
    float thetaOver2 = theta * .5f;
    
    // 赋值
    w = cos(thetaOver2);
    x = sin(thetaOver2);
    y = 0.0f;
    z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta) {
    // 计算半角
    float thetaOver2 = theta * .5f;
    
    // 赋值
    w = cos(thetaOver2);
    x = 0.0f;
    y = sin(thetaOver2);
    z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta) {
    // 计算半角
    float thetaOver2 = theta * .5f;
    
    // 赋值
    w = cos(thetaOver2);
    x = 0.0f;
    y = 0.0f;
    z = sin(thetaOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta) {
    // 旋转轴必须标准化
    assert(fabs(vectorMag(axis) - 1.0f) < 0.1f);
    
    // 计算半角和sin值
    float thetaOver2 = theta * 0.5f;
    float sinThetaOver2 = sin(thetaOver2);
    
    w = cos(thetaOver2);
    x = axis.x * sinThetaOver2;
    y = axis.y * sinThetaOver2;
    z = axis.z * sinThetaOver2;
}

// 10.6.5
void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation) {
    // 计算半角的sin和cos值
    float sp, sb, sh;
    float cp, cb, ch;
    sinCos(&sh, &ch, orientation.heading * .5f);
    sinCos(&sp, &cp, orientation.pitch * .5f);
    sinCos(&sb, &cb, orientation.bank * .5f);
    
    w = ch * cp * cb + sh * sp * sb;
    x = ch * sp * cb + sh * cp * sb;
    y = -ch * sp * sb + sh * cp * cb;
    z = -sh * sp * cb + ch * cp * sb;
}

// 10.6.5
void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation) {
    // 计算半角的sin和cos值
    float sp, sb, sh;
    float cp, cb, ch;
    sinCos(&sh, &ch, orientation.heading * .5f);
    sinCos(&sp, &cp, orientation.pitch * .5f);
    sinCos(&sb, &cb, orientation.bank * .5f);
    
    w = ch * cp * cb + sh * sp * sb;
    x = -ch * sp * cb - sh * cp * sb;
    y = ch * sp * sb - sh * cp * cb;
    z = sh * sp * cb - ch * cp * sb;
}

// 四元数叉乘（乘法）运算，用以连接多个角位移 10.4.8
// 乘的顺序从左到右
Quaternion Quaternion::operator*(const Quaternion &a) const {
    Quaternion result;
    result.w = w * a.w - x * a.x - y * a.y - z * a.z;
    result.x = w * a.x + x * a.w + z * a.y - y * a.z;
    result.y = w * a.y + y * a.w + x * a.z - z * a.x;
    result.z = w * a.z + z * a.w + y * a.x - x * a.y;
    return result;
}

// 叉乘并赋值
Quaternion& Quaternion::operator*=(const Quaternion &a) {
    *this = *this * a;
    return *this;
}

void Quaternion::normalize() {
    float mag = (float)sqrt(w * w + x * x + y * y + z * z);
    if (mag > 0.0f) {
        float oneOverMag = 1.0f / mag;
        w *= oneOverMag;
        x *= oneOverMag;
        y *= oneOverMag;
        z *= oneOverMag;
    } else {
        assert(false);
        // 在发布版中，返回单位四元数
        identity();
    }
}

float Quaternion::getRotationAngles() const {
    float thetaOver2 = safeAcos(w);
    
    return thetaOver2 * 2;
}

Vector3 Quaternion::getRotationAxis() const {
    // 计算sin^2(theta / 2)，w = cos(theta / 2), sin^2(x) + cos^2(x) = 1
    // x = axis.x * sin(theta / 2), y = axis.y * sin(theta / 2), z = axis.z * sin(theta / 2)
    float sinThetaOver2Sq = 1.0 - w * w;
    
    // 保证数值精度
    if (sinThetaOver2Sq <= 0.0f) {
        // 单位四元数或不精确的数值，只需要fan'hu 有效的向量即可
        return Vector3(1.0f, 0.0f, 0.0f);
    }
    
    // 计算1/sin(theta / 2)
    float oneOverSinThetaOver2 = 1.0 / sqrt(sinThetaOver2Sq);
    
    // 返回旋转轴
    return Vector3(x * oneOverSinThetaOver2, y * oneOverSinThetaOver2, z * oneOverSinThetaOver2);
}

// 四元数点乘 10.4.10
float dotProduct(const Quaternion& a, const Quaternion& b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

// slerp，球面线性插值，10.4.13
Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t) {
    // 检查参数边界
    if (t < 0.0f) {
        return q0;
    }
    
    if (t >= 1.0f) {
        return q1;
    }
    
    // 用点乘计算四元数夹角的cos值
    float cosOmega = dotProduct(q0, q1);
    
    // 如果点乘为负，使用-q1
    // 四元数q和-q代表相同的旋转，但可能产生不同的slerpc运算，我们要选择正确的一个以便用锐角进行旋转
    float q1w = q1.w;
    float q1x = q1.x;
    float q1y = q1.y;
    float q1z = q1.z;
    
    if (cosOmega < 0.0f) {
        q1w = -q1w;
        q1x = -q1x;
        q1y = -q1y;
        q1z = -q1z;
        cosOmega = -cosOmega;
    }
    
    // 我们用的是两个单位四元数，所以点乘结果应该<= 1.0f
    assert(cosOmega < 1.0f);
    
    // 计算插值片，注意检查非常接近的情况
    float k0, k1;
    if (cosOmega > 0.9999f) {
        // 非常接近，即线性插值，防止除零
        k0 = 1.f - t;
        k1 = t;
    } else {
        // TODO...
    }
    
    // TODO
    Quaternion result;
    return result;
}







