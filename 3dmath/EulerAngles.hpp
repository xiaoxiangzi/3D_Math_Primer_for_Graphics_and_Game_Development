//
//  EulerAngles.hpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/25.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#ifndef EulerAngles_hpp
#define EulerAngles_hpp

#include <stdio.h>

class Quaternion;
class Matrix4x3;
class RotationMatrix;

// Class EulerAngles
// 该类用于表示heading-pitch-bank欧拉角系统，左手坐标系

class EulerAngles {
    
public:
    // 绕y轴的旋转量，向右旋转为正（从上面看，旋转正方向就是顺时针方向），此时物体坐标系和惯性坐标系重合
    float heading;
    // 绕x轴的旋转量，物体坐标系x轴，向下旋转为正
    float pitch;
    // 绕z轴的旋转量，物体坐标系z轴，从z轴的正端点向原点看，bank的正方向为顺时针
    float bank;
    
    EulerAngles() : heading(0.0f), pitch(0.0f), bank(0.0f) {}
    EulerAngles(float h, float p, float b) : heading(h), pitch(p), bank(b) {}
    
    // 置零
    void identity() {
        heading = pitch = bank = 0.0f;
    }
    
    // 变换为“限制集”欧拉角
    void canonize();
    
    // 从四元数转换到欧拉角
    // 从物体到惯性
    void fromObjectToInertialQuaternion(const Quaternion& q);
    // 从惯性到物体
    void fromInertialQuaternion(const Quaternion& q);
    
    // 从矩阵转换到欧拉角，平移部分省略，假设矩阵是正交
    void fromObjectToInertialMatrix(const Matrix4x3& m);
    void fromInertialToObjectMatrix(const Matrix4x3& m);
    
    // 从旋转矩阵转换到欧拉角
    void fromRotationMatrix(const RotationMatrix& m);
};

extern const EulerAngles kEulerAnglesIdentity;

#endif /* EulerAngles_hpp */
