//
//  main.cpp
//  3dmath
//
//  Created by xiaoxiangzi on 2019/12/15.
//  Copyright © 2019 xiaoxiangzi. All rights reserved.
//

#include <iostream>
#include "Vector3.hpp"
#include "Quaternion.hpp"

void chaper5() {
    // (3) - a
    Vector3 a(3, 10, 7);
    Vector3 b(8, -7, 4);
    (a - b).print();
    
    // (4) - a
    Vector3 c(3, 10, 7);
    Vector3 d(8, -7, 4);
    cout << distance(c, d) << " " << distance(c, d) * distance(c, d) << endl;
    
    // (8)
    crossProduct(a, b).print();
}

void chapter10_3() {
    Quaternion a = {0.233, 0.060, -0.257, 0.935};
    Quaternion b = {-0.752, 0.280, 0.374, 0.495};
    a.print();
    b.print();
    cout << "dotProduct " << dotProduct(a, b) << endl;
}

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    
    Vector3 n(1, 2, 3);
    n.print();
    
    chaper5();
    chapter10_3();
    return 0;
}
