#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/ply_io.h>
#include "lshaped_fitting.h"

using namespace std;
using namespace cv;

void ShowLShapeFittingResult(cv::Mat& disMap, 
  std::vector<cv::Point2f>& initPoints, std::vector<cv::Point2f> resultPoints) 
{
  double FLAT_COEFF = 20.0;
  for (uint32_t i = 0U; i < initPoints.size(); ++i) {
    initPoints[i].x *= FLAT_COEFF;
    initPoints[i].y *= FLAT_COEFF;
    cv::circle(disMap, initPoints[i], 2, Scalar(0, 255, 120), -1);
  }

  for (uint32_t i = 0U; i < resultPoints.size(); ++i) {
    resultPoints[i].x *= FLAT_COEFF;
    resultPoints[i].y *= FLAT_COEFF;
    cv::circle(disMap, resultPoints[i], 5, Scalar(0, 0, 255), -1);
  }
  cv::imshow("disMap", disMap);
  cv::imwrite("../result/disMap.jpg", disMap);
	cv::waitKey(0);
}

int main(int argc, char** argv)
{

    cout << "lshaped_fitting test" << endl;
    
    // 3D Cloud --> 2D Dimension

    // Every Cluster Points.
    std::vector<cv::Point2f> hull;

    // Load Point Cloud For Shaped-BBox Fit.
    hull.push_back(cv::Point2f(19.61026979843742, 7.52767729296133));
    hull.push_back(cv::Point2f(17.03273076303270, 7.58346032483931));
    hull.push_back(cv::Point2f(15.13402863573614, 7.71117274328501));
    hull.push_back(cv::Point2f(14.72757894312114, 8.50297166732246));
    hull.push_back(cv::Point2f(14.77044667030188, 9.59204022261293));
    hull.push_back(cv::Point2f(14.76137992714666, 10.7247702891167));
    hull.push_back(cv::Point2f(14.74829228186517, 11.9429316067475));

    // Do Shaped-BBox Fit.
    fafn::perception::LShapedFIT lshaped;

    cv::RotatedRect rr = lshaped.FitBox(&hull);
    std::cout << "Shaped-BBox Message : " << rr.size.width << " " << rr.size.height << " " << rr.angle;

    // --- Vertex Standard Output Coordinates.
    // [19.60907603446491,  7.493492365484474]
    // [19.75834431780993,  11.76797673445868]
    // [14.74829228186517,  11.94293160674757]
    // [14.59902399852014,  7.668447237773362]
    std::vector<cv::Point2f> vertices = lshaped.getRectVertex();

    std::cout << "Top 4 Vertices" << std::endl;
    for (size_t i = 0; i < vertices.size(); ++i)
        std::cout << "  " << vertices[i].x << "  " << vertices[i].y << std::endl;

    // for show
    cv::Mat disMap(600, 600, CV_8UC3, Scalar(0,0,0));
    ShowLShapeFittingResult(disMap, hull, vertices);

    return 0;
}