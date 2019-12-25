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


