//
//  Quaternion.hpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/25.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#ifndef Quaternion_hpp
#define Quaternion_hpp

class Vector3;
class EulerAngles;

// 实现在3D中表示角位移的四元数
class Quaternion {
    
public:
    float w, x, y, z;
    
    void identity() {
        w = 1.0f;
        x = y = z = 0.0f;
    }
    
    // 构造执行旋转的四元数
    void setToRotateAboutX(float theta);
    void setToRotateAboutY(float theta);
    void setToRotateAboutZ(float theta);
    void setToRotateAboutAxis(const Vector3& axis, float theta);
    
    // 构造执行物体-惯性旋转的四元数，方位参数用欧拉角形式给出
    void setToRotateObjectToInertial(const EulerAngles& orientation);
    void setToRotateInertialToObject(const EulerAngles& orientation);
    
    // 叉乘（乘法）
    Quaternion operator* (const Quaternion& a) const;
    
    // 赋值乘法
    Quaternion& operator*= (const Quaternion& a);
    
    // 将四元数正则化
    void normalize();
    
    // 提取旋转角和旋转轴
    float getRotationAngles() const;
    Vector3 getRotationAxis() const;
};

// 全局单位四元数
extern const Quaternion kQuaternionIdentity;

// 四元数点乘
extern float dotProduct(const Quaternion& a, const Quaternion& b);

// 球面线性插值
extern Quaternion slerp(const Quaternion& p, const Quaternion& q, float t);

// 四元数共轭
extern Quaternion conjugate(const Quaternion& q);

// 四元数幂
extern Quaternion pow(const Quaternion& q);

#endif /* Quaternion_hpp */
