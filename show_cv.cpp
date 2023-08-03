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

    cv::Mat srcImg;
    // image = imread( argv[1], IMREAD_COLOR );

    srcImg = cv::imread("../test.jpg", cv::IMREAD_COLOR);

    if ( !srcImg.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", srcImg);
    cv::waitKey(0);

    return 0;
}