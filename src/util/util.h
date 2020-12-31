//
// Created by hollow on 2020/12/15.
//

#ifndef KCF_WITH_KALMAN_UTIL_H
#define KCF_WITH_KALMAN_UTIL_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

class util {
public:
    static vector<double> PRO_COV_POINT;
    static vector<double> PRO_COV_WIDTH;
    static vector<double> PRO_COV_LENGTH;

    static vector<double> MEASURE_COV_POINT;
    static vector<double> MEASURE_COV_WIDTH;
    static vector<double> MEASURE_COV_LENGTH;

    static bool PROCESS;

    static void LoadParam(); /// 初始化全局参数

};


#endif //KCF_WITH_KALMAN_UTIL_H
