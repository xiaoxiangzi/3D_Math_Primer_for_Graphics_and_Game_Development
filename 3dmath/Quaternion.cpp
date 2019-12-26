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



