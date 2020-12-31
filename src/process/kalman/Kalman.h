//
// Created by hollow on 2020/12/15.
//

#ifndef KCF_WITH_KALMAN_KALMAN_H
#define KCF_WITH_KALMAN_KALMAN_H

#include <opencv2/opencv.hpp>

class Kalman {
public:
    Kalman() = default;
    Kalman(int _measurenum,int _statenum){
        m_measurenum=_measurenum;
        m_statenum=_statenum;
        transitionMatrix = cv::Mat (m_statenum,m_statenum,CV_64F);
        measurementMatrix = cv::Mat (m_measurenum,m_statenum,CV_64F);
        processNoiseCov = cv::Mat (m_statenum,m_statenum,CV_64F);
        measurementNoiseCov = cv::Mat (m_measurenum,m_measurenum,CV_64F);
        errorCovPre = cv::Mat (m_statenum,m_statenum,CV_64F);
        statePre = cv::Mat (m_statenum,1,CV_64F);
        statePost = cv::Mat::zeros(m_statenum,1,CV_64F);
        gain = cv::Mat (m_statenum,m_measurenum,CV_64F);
        errorCovPost = cv::Mat (m_statenum,m_statenum,CV_64F);
        I=cv::Mat::eye(m_statenum,m_statenum,CV_64F);
        residual = cv::Mat(m_measurenum,1,CV_64F);
        temp4 = cv::Mat(m_measurenum,m_measurenum,CV_64F);
    };
    ~Kalman()= default;;
    void init(double processcov,double measurecov);
    void init(std::vector<double> processcov,std::vector<double> measurecov);
    const cv::Mat& predict(const cv::Mat& measurement);
    cv::Mat measurement;
    cv::Mat transitionMatrix;
    cv::Mat measurementMatrix;
    cv::Mat processNoiseCov;
    cv::Mat measurementNoiseCov;
    cv::Mat errorCovPre;
    cv::Mat statePre;
    cv::Mat statePost;
    cv::Mat gain ;
    cv::Mat errorCovPost;
    cv::Mat I;
    cv::Mat residual;
    cv::Mat temp4;

private:
    int m_measurenum{};
    int m_statenum{};
};


#endif //KCF_WITH_KALMAN_KALMAN_H
