//
// Created by hollow on 2020/12/15.
//

#include "util.h"

using namespace cv;

vector<double> util::PRO_COV_POINT;
vector<double> util::PRO_COV_WIDTH;
vector<double> util::PRO_COV_LENGTH;
vector<double> util::MEASURE_COV_POINT;
vector<double> util::MEASURE_COV_WIDTH;
vector<double> util::MEASURE_COV_LENGTH;
bool util::PROCESS;

void util::LoadParam() {
    PROCESS = true;

    FileStorage fs(cv::String("../data/init/param.yaml"), FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "param.yaml didn't open correctly" << std::endl;
        exit(0);
    }
    FileNode node_Kalman = fs["Kalman"];

    node_Kalman["pro_cov_point"] >> util::PRO_COV_POINT;
    node_Kalman["pro_cov_length"] >> util::PRO_COV_LENGTH;
    node_Kalman["pro_cov_width"] >> util::PRO_COV_WIDTH;
    node_Kalman["measure_cov_point"] >> util::MEASURE_COV_POINT;
    node_Kalman["measure_cov_length"] >> util::MEASURE_COV_LENGTH;
    node_Kalman["measure_cov_width"] >> util::MEASURE_COV_WIDTH;
}
