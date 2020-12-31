//
// Created by hollow on 2020/12/15.
//

#include "Kalman.h"

/**
 * @brief 初始化滤波器
 * 滤波器的测量误差矩阵和模型误差矩阵被设置为对角阵，对角线上的值全部一样
 * @param processcov 模型误差矩阵对角线值
 * @param measurecov 测量误差矩阵对角线值
 */
void Kalman::init(double processcov,double measurecov){
    measurement = cv::Mat::zeros(m_measurenum,1,CV_64F);
    setIdentity(processNoiseCov,cv::Scalar::all(processcov));
    setIdentity(measurementNoiseCov,cv::Scalar::all(measurecov));
    setIdentity(measurementMatrix);
    setIdentity(errorCovPost,cv::Scalar::all(1));
}


/**
 * @brief 初始化滤波器
 * 滤波器的测量误差矩阵和模型误差矩阵将被设置为对角阵，值由两个向量提供
 * @param processcov 模型误差矩阵对角线值
 * @param measurecov 测量误差矩阵对角线值
 */
void Kalman::init(std::vector<double> processcov,std::vector<double> measurecov){
    if(processcov.size()==1){
        setIdentity(processNoiseCov,cv::Scalar::all(processcov[0]));
    } else if(processcov.size()==m_statenum){
        setIdentity(processNoiseCov,cv::Scalar::all(1));
        for(int i=0;i<m_statenum;i++){
            processNoiseCov.at<double>(i,i)=processcov[i];
        }
    }else{
        return;
    }
    if(measurecov.size()==1){
        setIdentity(measurementNoiseCov,cv::Scalar::all(measurecov[0]));
    } else if(measurecov.size()==m_measurenum){
        setIdentity(measurementNoiseCov,cv::Scalar::all(1));
        for(int i=0;i<m_measurenum;i++){
            measurementNoiseCov.at<double>(i,i)=measurecov[i];
        }
    }else{
        return;
    }
    measurement = cv::Mat::zeros(m_measurenum,1,CV_64F);
    setIdentity(measurementMatrix);
    setIdentity(errorCovPost,cv::Scalar::all(1));
}

/**
 * @brief 滤波过程：五个方程
 * @param measurement
 * @return
 */
const cv::Mat& Kalman::predict(const cv::Mat& _measurement)
{
    /***时间更新方程，卡尔曼滤波为一种高斯滤波，均值和协方差可以表示一个高斯分布*/
    statePre = transitionMatrix*statePost;  ///X'(k+1|k)=F(k)X(k|k)  predict_value 状态更新得到先验证均值（状态）更新
    errorCovPre = transitionMatrix*errorCovPost*transitionMatrix.t()+processNoiseCov;///状态更新得到先验协方差估计

    /***状态更新方程*/
    residual = _measurement - measurementMatrix*statePre;  ///residual=z(k+1)-H(k)X'(k+1|k)  残余
    temp4 = measurementMatrix*errorCovPre*measurementMatrix.t()+measurementNoiseCov;
    gain = errorCovPre*measurementMatrix.t()*temp4.inv();//卡尔曼增益
    statePost = statePre+gain*residual;///后验状态估计（高斯均值）
    errorCovPost=(I-gain*measurementMatrix)*errorCovPre;///后验协方差矩阵
    return statePost;
}