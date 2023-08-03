#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv )
{
    // if ( argc != 2 )
    // {
    //     printf("usage: DisplayImage.out <Image_Path>\n");
    //     return -1;
    // }

    cv::Mat srcImg, bluredImg;
    // image = imread( argv[1], IMREAD_COLOR );

    // srcImg = cv::imread("../test.jpg", cv::IMREAD_COLOR);
    srcImg = cv::imread("../lena.jpg", cv::IMREAD_COLOR);

    if ( !srcImg.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", srcImg);
    // cv::waitKey(0);

    int ksize = 7;
    cv::GaussianBlur(srcImg, bluredImg, cv::Size(ksize, ksize), 10);

    cv::namedWindow("Blur", cv::WINDOW_AUTOSIZE);
    cv::imshow("Blur", bluredImg);

    cv::Mat grayImg;
    cv::cvtColor(bluredImg, grayImg, cv::COLOR_BGR2GRAY);

    cv::namedWindow("Display Gray Img", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Gray Img", grayImg);

    cv::waitKey(0);

    return 0;
}