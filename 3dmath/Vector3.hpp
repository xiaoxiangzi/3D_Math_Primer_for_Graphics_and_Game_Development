//
//  Vector3.hpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/15.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#ifndef Vector3_hpp
#define Vector3_hpp

#include <math.h>
#include <iostream>
#include <sstream>
using namespace std;

class Vector3 {
    
public:
    float x;
    float y;
    float z;
    
    // 默认构造s函数
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    // 拷贝构造函数
    Vector3(const Vector3& a) : x(a.x), y(a.y), z(a.z) {}
    // 带参数构造函数
    Vector3(float nx, float ny, float nz) : x(nx), y(ny), z(nz) {}
    
    Vector3& operator =(const Vector3& a) {
        x = a.x;
        y = a.y;
        z = a.z;
        return *this;
    }
    
    bool operator ==(const Vector3& a) const {
        return x == a.x && y == a.y && z == a.z;
    }
    
    bool operator !=(const Vector3& a) const {
        return x != a.x && y != a.y && z != a.z;
    }
    
    // 置为零向量
    void zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    
    // 一元负运算符
    Vector3 operator -() const {
        return Vector3(-x, -y, -z);
    }
    
    Vector3 operator +(const Vector3& a) const {
        return Vector3(x + a.x, y + a.y, z + a.z);
    }
    
    Vector3 operator -(const Vector3& a) const {
        return Vector3(x - a.x, y - a.y, z - a.z);
    }
    
    // 与标量的乘除法
    Vector3 operator *(float a) const {
        return Vector3(x * a, y * a, z * a);
    }
    
    Vector3 operator /(float a) const {
        assert( a != 0.0f);
        float oneOverA = 1.0f / a;
        return Vector3(x * oneOverA, y * oneOverA, z * oneOverA);
    }
    
    Vector3& operator +=(const Vector3& a) {
        x += a.x;
        y += a.y;
        z += a.z;
        return *this;
    }
    
    Vector3& operator -=(const Vector3& a) {
        x -= a.x;
        y -= a.y;
        z -= a.z;
        return *this;
    }
    
    Vector3& operator *=(const float a) {
        x *= a;
        y *= a;
        z *= a;
        return *this;
    }
    
    Vector3& operator /=(const float a) {
        assert(a != 0.0f);
        float oneOverA = 1.0f / a;
        x *= oneOverA;
        y *= oneOverA;
        z *= oneOverA;
        return *this;
    }
    
    // 向量标准化
    void normalize() {
        float magSq = x * x + y * y + z * z;
        if (magSq > 0.0f) {
            float oneOverMag = 1.0f / sqrt(magSq);
            x *= oneOverMag;
            y *= oneOverMag;
            z *= oneOverMag;
        }
    }
    
    // 向量点乘
    float operator *(const Vector3& a) const {
        return x * a.x + y * a.y + z * a.z;
    }
    
    void print() {
        stringstream ss;
        ss << "v[" << x << ", " << y << ", " << z << "]" << endl;
        cout << ss.str();
    }
};

// 求向量模
inline float vectorMag(const Vector3& a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// 向量叉乘
inline Vector3 crossProduct(const Vector3& a, const Vector3& b) {
    return Vector3(
       a.y * b.z - a.z * b.y,
       a.z * b.x - a.x * b.z,
       a.x * b.y - a.y * b.x);
}

// 标量左乘
inline Vector3 operator *(float k, const Vector3& v) {
    return Vector3(k * v.x, k * v.y, k * v.z);
}

// 计算两点间的距离
inline float distance(const Vector3& a, const Vector3& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// 全局零向量
extern const Vector3 kZeroVector;


#endif /* Vector3_hpp */
