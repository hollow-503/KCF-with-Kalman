//
// Created by hollow on 2020/12/15.
//

#include "util/util.h"
#include "kalman/Kalman.h"
#include "kcf/KCF.h"

using namespace std;

int main() {
    cv::Mat img; //声明一个保存图像的类
    img = cv::imread("../data/1.png"); //读取图像，根据图片所在位置填写路径即可
    if (img.empty()) {
        cout << "请确认图像文件名称是否正确" << endl;
        return -1;
    }
    cv::namedWindow("test", cv::WINDOW_NORMAL);
    imshow("test", img);
    cv::waitKey(0);
    return 0;
}
