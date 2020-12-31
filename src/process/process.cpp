//
// Created by hollow on 2020/12/31.
//

#include "process.h"

process::process() {
    KMF_width = Kalman(1, 2);
    KMF_length = Kalman(1, 2);
    KMF_point_x = Kalman(1, 2);
    KMF_point_y = Kalman(1, 2);
}

process::~process() = default;

/**
 * 返回预测结果
 * @param delta_time 这一帧与下一帧的时间间隔
 * @param pre_num 滤波器迭代次数
 */
void process::calculate(double delta_time, int pre_num) {
    if(pre_num > 10){///达到预测要求的最小迭代次数，确保已经滤波器收敛
        pre_point.x = prediction_point_x.at<double>(0) + delta_time*prediction_point_x.at<double>(1);
        pre_point.y = prediction_point_y.at<double>(0) + delta_time*prediction_point_y.at<double>(1);
        pre_width = prediction_width.at<double>(0) + delta_time*prediction_width.at<double>(1);
        pre_length = prediction_length.at<double>(0) + delta_time*prediction_length.at<double>(1);
    }else{///没有达到迭代次数时发送原始值
        pre_point.x = right_down_left_down.x;
        pre_point.y = right_down_left_down.y;
        pre_width = width;
        pre_length = length;
    }
}

/**
 * KMF预测
 * @param delta_time 这一帧与上一帧的时间差
 * @param pre_num 滤波器迭代次数
 */
void process::KMFCal(double delta_time, int pre_num) {
    if(pre_num < 2){
        ///初始化先验状态
        KMF_point_x.statePost = (cv::Mat_<double> (2,1) << right_down_left_down.x,0);
        KMF_point_y.statePost = (cv::Mat_<double> (2,1) << right_down_left_down.y,0);
        KMF_width.statePost = (cv::Mat_<double> (2,1) << width,0);
        KMF_length.statePost = (cv::Mat_<double> (2,1) << length,0);
    }
    KMF_point_x.measurement.at<double>(0) = right_down_left_down.x;
    KMF_point_y.measurement.at<double>(0) = right_down_left_down.y;
    KMF_width.measurement.at<double>(0) = width;
    KMF_length.measurement.at<double>(0) = length;

    ///状态转移矩阵，匀速模型
    KMF_point_x.transitionMatrix = (cv::Mat_<double> (2,2) << 1,delta_time,0,1);
    KMF_point_y.transitionMatrix = (cv::Mat_<double> (2,2) << 1,delta_time,0,1);
    KMF_width.transitionMatrix = (cv::Mat_<double> (2,2) << 1,delta_time,0,1);
    KMF_length.transitionMatrix = (cv::Mat_<double> (2,2) << 1,delta_time,0,1);

    ///滤波结果
    prediction_point_x = KMF_point_x.predict(KMF_point_x.measurement);
    prediction_point_y = KMF_point_y.predict(KMF_point_y.measurement);
    prediction_width = KMF_width.predict(KMF_width.measurement);
    prediction_length = KMF_length.predict(KMF_length.measurement);

    ///下面几行限制速度，避免出现速度过大的异常情况
    if(prediction_point_x.at<double>(1) > 0.1) {
        prediction_point_x.at<double>(1) = 0.1;
    }else if(prediction_point_x.at<double>(1) < -0.1){
        prediction_point_x.at<double>(1) = -0.1;
    }
    if(prediction_point_y.at<double>(1) > 0.1) {
        prediction_point_y.at<double>(1) = 0.1;
    }else if(prediction_point_y.at<double>(1) < -0.1){
        prediction_point_y.at<double>(1) = -0.1;
    }
    if(prediction_width.at<double>(1) > 0.1) {
        prediction_width.at<double>(1) = 0.1;
    }else if(prediction_width.at<double>(1) < -0.1){
        prediction_width.at<double>(1) = -0.1;
    }
    if(prediction_length.at<double>(1) > 0.1) {
        prediction_length.at<double>(1) = 0.1;
    }else if(prediction_length.at<double>(1) < -0.1){
        prediction_length.at<double>(1) = -0.1;
    }
}

void process::run() {
    vector<double> pro_cov_point = util::PRO_COV_POINT;
    vector<double> pro_cov_length = util::PRO_COV_LENGTH;
    vector<double> pro_cov_width = util::PRO_COV_WIDTH;
    vector<double> measure_cov_point = util::MEASURE_COV_POINT;
    vector<double> measure_cov_length = util::MEASURE_COV_LENGTH;
    vector<double> measure_cov_width = util::MEASURE_COV_WIDTH;

    int pre_num = 0; /// 滤波器迭代次数
    bool find = false, last_find = false;
    double delta_time = 10; /// 10ms
    double next_delta_time = 10; /// 10ms
    while (util::PROCESS) {
        if (find && !last_find) {
            KMF_point_x.init(pro_cov_point, measure_cov_point);
            KMF_point_y.init(pro_cov_point, measure_cov_point);
            KMF_length.init(pro_cov_length, measure_cov_length);
            KMF_width.init(pro_cov_width, measure_cov_width);
            pre_num = 0;
            last_find = true;
        }
        KMFCal(delta_time, pre_num);
        calculate(next_delta_time, pre_num);
        pre_num++;
    }
}
