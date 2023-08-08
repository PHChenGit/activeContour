#include <stdio.h>
#include <vector>
#include <limits> 

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_POINTS 8
#define MAX_INTERATIONS 1000
#define ONE_MICROSECOND 1000
#define TWO_MICROSECOND 2000
#define CONTOURS_THICKNESS 3
#define WINDOW_NAME_ORIGINAL "original"
#define CIRCLE_SIZE 5

std::vector<cv::Point> points;
cv::Mat srcImg;
const cv::Scalar colorGreen(0, 255, 0);

void setInitPoints(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && points.size() < MAX_POINTS) {
        printf("[%d, %d]\n", x, y);
        cv::Point p = cv::Point(x, y);
        cv::circle(srcImg, p, CIRCLE_SIZE, colorGreen, cv::LineTypes::FILLED);
        points.push_back(p);
        cv::imshow(WINDOW_NAME_ORIGINAL, srcImg);
    }
}

int main(int argc, char** argv )
{
    srcImg = cv::imread("../test.jpg", cv::IMREAD_COLOR);

    if ( !srcImg.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::Mat grayScale;
    srcImg.copyTo(grayScale);
    cv::cvtColor(grayScale, grayScale, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayScale, grayScale, cv::Size(3,3), 10);

    cv::Mat gradX, gradY, gradient;
    cv::Sobel(grayScale, gradX, CV_32F, 1, 0);
    cv::Sobel(grayScale, gradY, CV_32F, 0, 1);
    cv::addWeighted(gradX, 1.0, gradY, 1.0, 0.0, gradient);

    cv::namedWindow(WINDOW_NAME_ORIGINAL, cv::WINDOW_AUTOSIZE );
    cv::setMouseCallback(WINDOW_NAME_ORIGINAL, setInitPoints);

    while (points.size() < MAX_POINTS) {
        cv::imshow(WINDOW_NAME_ORIGINAL, srcImg);
        cv::waitKey(10);
    }
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(points);
    printf("drawing contours\n");
    cv::drawContours(srcImg, contours, 0, colorGreen, CONTOURS_THICKNESS);
    cv::imshow(WINDOW_NAME_ORIGINAL, srcImg);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}