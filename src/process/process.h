//
// Created by hollow on 2020/12/31.
//

#ifndef KCF_WITH_KALMAN_PROCESS_H
#define KCF_WITH_KALMAN_PROCESS_H

#include "kalman/Kalman.h"
#include "kcf/KCF.h"
#include "../util/util.h"

class process {
private:
    Kalman KMF_point_x;
    Kalman KMF_point_y;
    Kalman KMF_width;
    Kalman KMF_length;

    cv::Mat prediction_point_x;
    cv::Mat prediction_point_y;
    cv::Mat prediction_width;
    cv::Mat prediction_length;

    double pre_width{}, pre_length{};
    cv::Point2f pre_point;

    cv::Point2f right_down_left_down;
    float width{};
    float length{};

    void KMFCal(double delta_time, int pre_num);
    void calculate(double delta_time, int pre_num);
public:
    process();
    ~process();
    void run();
};


#endif //KCF_WITH_KALMAN_PROCESS_H
