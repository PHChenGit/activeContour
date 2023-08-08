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

/**
 * energy total = energy image + energy contour
 * energy image = gamma * del I square
 * energy contour = alpha * energy elastic + beta * energy smooth
 */
void activeContour(cv::Mat gradient)
{
    const double alpha = 0.1;
    const double beta = 0.3;
    const double rvl_gamma = 0.1;

    double minEnergy = std::numeric_limits<double>::max();
    cv::Point currPoint;
    std::vector<cv::Point> newPoints;
    const int searchingRadius = 1;
    int centerX = srcImg.rows / 2;
    int centerY = srcImg.cols / 2;
    for (int idx = 0; idx < MAX_POINTS; idx++) {
        cv::Point point = points[idx];
        cv::Point prevPoint = points[(idx + points.size() - 1) % points.size()];
        cv::Point nexPoint = points[(idx + 1) % points.size()];

        // set searching area
        int xDirection = point.x < centerX ? -1 : 1;
        int yDirection = point.y < centerY ? -1 : 1;
        int xStart = point.x < centerX ? searchingRadius : -searchingRadius;
        int xEnd = point.x < centerX ? -searchingRadius : searchingRadius;
        int yStart = point.y < centerY ? searchingRadius : -searchingRadius;
        int yEnd = point.y < centerY ? -searchingRadius : searchingRadius;

        for (int curX = xStart; point.x < centerX ? curX >= xEnd : curX <= xEnd ; curX += xDirection) {
            for (int curY = yStart; point.y < centerY ? curY >= yEnd : curY <= yEnd; curY += yDirection) {
                currPoint = cv::Point(point.x + curX, point.y + curY);
                double energyCont = cv::norm(currPoint - prevPoint);
                double energyCurv = cv::norm(prevPoint - (2 * currPoint) + nexPoint);
                double energyImg = cv::norm(gradient.at<double>(currPoint));
                double energyTotal = alpha * energyCont + beta * energyCurv + rvl_gamma * energyImg;

                if (energyTotal < minEnergy) {
                    minEnergy = energyTotal;
                    point = currPoint;
                }
            }
        }

        newPoints.push_back(point);
    }
    points = newPoints;
}

void set_debug_points(cv::Point point)
{
    cv::circle(srcImg, point, CIRCLE_SIZE, colorGreen, cv::LineTypes::FILLED);
    points.push_back(point);
    cv::imshow(WINDOW_NAME_ORIGINAL, srcImg);
}

int main(int argc, char** argv )
{
    srcImg = cv::imread("../test.jpg", cv::IMREAD_COLOR);

    if ( !srcImg.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::Mat grayScale, blur;
    srcImg.copyTo(grayScale);
    cv::cvtColor(grayScale, grayScale, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayScale, blur, cv::Size(3,3), 0);

    cv::Mat gradX, gradY, absGradX, absGradY, gradient;
    cv::Sobel(blur, gradX, CV_64F, 1, 0);
    cv::Sobel(blur, gradY, CV_64F, 0, 1);
    convertScaleAbs(gradX, absGradX);
    convertScaleAbs(gradY, absGradY);
    // cv::add(gradX, gradY, gradient);
    // cv::magnitude(gradX, gradY, gradient);
    addWeighted(absGradX, 1.0, absGradY, 1.0, 0, gradient);

    cv::namedWindow(WINDOW_NAME_ORIGINAL, cv::WINDOW_AUTOSIZE );
    // cv::setMouseCallback(WINDOW_NAME_ORIGINAL, setInitPoints);

    // while (points.size() < MAX_POINTS) {
    //     cv::imshow(WINDOW_NAME_ORIGINAL, srcImg);
    //     cv::waitKey(10);
    // }

    // for debuging
    set_debug_points(cv::Point(135, 200));
    set_debug_points(cv::Point(173, 341));
    set_debug_points(cv::Point(234, 659));
    set_debug_points(cv::Point(410, 909));
    set_debug_points(cv::Point(693, 918));
    set_debug_points(cv::Point(858, 569));
    set_debug_points(cv::Point(789, 307));
    set_debug_points(cv::Point(767, 76));

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat paintImg;

    for (int step = 0; step < MAX_INTERATIONS; step++) {
        printf("step: %d\n", step);
        contours.clear();
        contours.push_back(points);
        srcImg.copyTo(paintImg);
        cv::drawContours(paintImg, contours, 0, colorGreen, CONTOURS_THICKNESS);
        cv::imshow(WINDOW_NAME_ORIGINAL, paintImg);
        cv::waitKey(400);

        activeContour(gradient);
    }

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}