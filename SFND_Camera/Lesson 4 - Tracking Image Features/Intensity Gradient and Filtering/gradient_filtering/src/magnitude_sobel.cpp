#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;

    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    cv::Mat smoothed;
    int kernelSize = 3;
    float sigma = 1.5;
    cv::GaussianBlur(imgGray, smoothed, cv::Size(kernelSize, kernelSize), sigma, sigma);

    // create filter kernels using the cv::Mat datatype both for x and y
    float sobel[9] = {-1, 0, +1,
                      -2, 0, +2,
                      -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel);
    cv::Mat kernel_y = kernel_x.t();

    // apply filter using the OpenCv function filter2D()
    cv::Mat gradient_x;
    cv::Mat gradient_y;
    cv::filter2D(smoothed, gradient_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(smoothed, gradient_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson
    cv::Mat magnitude = imgGray.clone();
    for (int r=0; r<magnitude.rows; ++r) {
      for (int c=0; c<magnitude.cols; ++c) {
          magnitude.at<uchar>(r,c) = std::sqrt(
            std::pow(gradient_x.at<uchar>(r,c), 2) +
            std::pow(gradient_y.at<uchar>(r,c), 2)
          );
      }
    }

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}
