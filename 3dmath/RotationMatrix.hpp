//
//  RotationMatrix.hpp
//  3dmath
//
//  Created by xiaoxiangzi-imac on 2019/12/31.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#ifndef RotationMatrix_hpp
#define RotationMatrix_hpp

class Vector3;
class EulerAngles;
class Quaternion;

// RotationMatrix类
// 实现了一个3x3的举证， 仅用作旋转矩阵。矩阵假设为正教的，在变换时指定方向。
// 该矩阵表达的是惯性-物体的变换，如果要执行物体到惯性的变换，应该乘以它的转置
class RotationMatrix {
    
public:
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;
    
    // 置为单位矩阵
    void identity();
    
    // 根据指定的方位构造矩阵
    void setup(const EulerAngles& orientation);
    
    // 根据四元数构造矩阵，假设该四元数参数代表指定的变换
    void fromInertialToObjectQuaternion(const Quaternion& q);
    void fromObjectToInertialQuaternion(const Quaternion& q);
    
    // 执行旋转
    Vector3 inertialToObject(const Vector3& v) const;
    Vector3 objectToInertial(const Vector3& v) const;
};

#endif /* RotationMatrix_hpp */
