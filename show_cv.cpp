#include <stdio.h>
#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define FILLED -1

std::vector<cv::Point> points;
cv::Mat srcImg;
const cv::Scalar colorGreen(0, 255, 0);
const std::string originalWindowName  = "original";

void setInitPoints(int event, int x, int y, int flags, void* userdata)
{
    if (event != cv::EVENT_LBUTTONDOWN || points.size() > 12) {
        return;
    }

    printf("[%d, %d]\n", x, y);
    cv::Point p = cv::Point(x, y);
    points.push_back(p);
    cv::circle(srcImg, p, 10, colorGreen, FILLED);
    cv::imshow(originalWindowName, srcImg);
}

int main(int argc, char** argv )
{
    cv::Mat cpImg, bluredImg, grayImg, sobelX, sobelY;
    srcImg = cv::imread("../test.jpg", cv::IMREAD_COLOR);

    if ( !srcImg.data )
    {
        printf("No image data \n");
        return -1;
    }

    srcImg.copyTo(cpImg);
    int ksize = 7;
    cv::GaussianBlur(cpImg, bluredImg, cv::Size(ksize, ksize), 10);
    cv::cvtColor(bluredImg, grayImg, cv::COLOR_BGR2GRAY);
    cv::Sobel(grayImg, sobelX, CV_16S, 1, 0);
    cv::Sobel(grayImg, sobelY, CV_16S, 1, 0);

    cv::Mat absGradX, absGradY, mixGrad;
    cv::convertScaleAbs(sobelX, absGradX);
    cv::convertScaleAbs(sobelY, absGradY);

    cv::addWeighted(absGradX, 1.0, absGradY, 1.0, 0, mixGrad);

    cv::namedWindow(originalWindowName, cv::WINDOW_AUTOSIZE );
    cv::imshow(originalWindowName, srcImg);
    cv::setMouseCallback(originalWindowName, setInitPoints);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}