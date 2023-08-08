#include <opencv2/opencv.hpp>

cv::Mat image;
std::vector<cv::Point> points;

// Mouse event handler
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        points.push_back(cv::Point(x, y));
        cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Set Points", image);
        
        if (points.size() == 8) {
            cv::destroyWindow("Set Points");
        }
    }
}

int main() {
    // Create a blank image
    image = cv::imread("../test.jpg", cv::IMREAD_COLOR);

    cv::namedWindow("Set Points");
    cv::setMouseCallback("Set Points", onMouse);

    while (points.size() < 8) {
        cv::imshow("Set Points", image);
        cv::waitKey(10);
    }

    // Draw contours
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(points);
    cv::drawContours(image, contours, 0, cv::Scalar(0, 255, 0), 2);

    cv::imshow("Contours", image);
    cv::waitKey(0);

    return 0;
}
