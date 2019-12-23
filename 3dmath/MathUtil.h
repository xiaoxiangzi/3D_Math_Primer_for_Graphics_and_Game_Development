//
//  MathUtil.h
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/15.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#ifndef MathUtil_h
#define MathUtil_h

#include <math.h>

// 定义和pi有关常量
const float kPi = 3.14159265f;
const float k2Pi = kPi * 2.0f;
const float KPiOver2 = kPi / 2.0f;
const float k10OverPi = 1.0f / kPi;
const float k10Over2Pi = 1.0f / k2Pi;

// 通过增加适当的2pi倍数将角度限制在-pi到pi的区间内
extern float wrapPi(float theta);
// 安全反三角函数
extern float safeAcos(float x);
// 计算角度的sin和cos值
// 在某些平台上，如果需要这两个值，同时计算比分开计算快
inline void sinCos(float* returnSin, float* returnCos, float theta) {
    // 为了简单，只使用标准三角函数，在某些平台上可以做的得更好一些
    *returnSin = sin(theta);
    *returnCos = cos(theta);
}
#endif /* MathUtil_h */
