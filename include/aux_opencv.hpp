#ifndef __AUX_OPENCV_HPP_
#define __AUX_OPENCV_HPP_

#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

// opencv auxiliary functions
std::vector<int> colorPicker(std::string path = "");
cv::Mat getMask(cv::Mat bgr, std::vector<int> hsvRanges, bool kernel = true);
cv::Mat getEdges(cv::Mat img, bool isMask = true);
std::vector<std::vector<cv::Point>> getAllContours(cv::Mat mask);
int getMaxAreaContourId(std::vector<std::vector<cv::Point>> contours);
std::vector<cv::Point> getMaxAreaContour(cv::Mat mask);
cv::Point getContourCenter(std::vector<cv::Point> contour);
void crossHair(cv::Mat img, cv::Point point, int size = 20, cv::Scalar color = {0, 0, 255});
std::vector<cv::Vec3f> findCircles(cv::Mat img, bool isMask = true);
std::vector<cv::Vec4i> findLines(cv::Mat img, bool isMask = true);
cv::Point getVanishingPoint(cv::Mat img, std::vector<cv::Vec4i> lines);
double getAngleWithVertical(double m);
std::vector<cv::Point> getAllContoursCenter(cv::Mat bgr, std::vector<std::vector<cv::Point>> contours, cv::Rect roi, std::string direction = "");
cv::Rect cropImg(cv::Mat bgr, std::string direction = "");
double linearRegression(cv::Mat bgr, cv::Point *pmin = nullptr, std::string direction = "");
void detectAruco(cv::Mat *bgr, std::vector<int> *id);

#endif