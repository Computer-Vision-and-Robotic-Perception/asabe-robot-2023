//
// Created by xinha on 6/21/2022.
//

#include "Douglas_Peucker.h"
#include "cmath"


std::vector<RDP::Point2d> RDP::DouglasP::pointFilter(std::vector<Point2d>& pointMatrix, float epsilon)
{
    std::vector<Point2d> resultMatrix;

    // Find the point with the maximum distance
    float dmax = 0;
    int index = 0;
    for (int i = 1; i < pointMatrix.size() - 1; ++i)
    {
        float d = perpendicularD(pointMatrix[i], pointMatrix[0], pointMatrix[pointMatrix.size() - 1]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, filter out points
    if (dmax > epsilon)
    {
        std::vector<Point2d> pre_part, next_part;
        for (int i = 0; i <= index; ++i){
            pre_part.push_back(pointMatrix[i]);
        }
        for (int i = index; i < pointMatrix.size(); ++i){
            next_part.push_back(pointMatrix[i]);
        }

        std::vector<Point2d> tempMatrix1 = pointFilter(pre_part, epsilon);
        std::vector<Point2d> tempMatrix2 = pointFilter(next_part, epsilon);

        // put filtered points together
        resultMatrix.insert(resultMatrix.end(), tempMatrix1.begin(), tempMatrix1.end());
        resultMatrix.insert(resultMatrix.end(), tempMatrix2.begin()+1, tempMatrix2.end());
    }

    // already in the range, put points together
    else
    {
        resultMatrix.push_back(pointMatrix[0]);
        resultMatrix.push_back(pointMatrix[pointMatrix.size() - 1]);
    }

    return resultMatrix;
}

// define a function to find the perpendicular distance
float RDP::DouglasP::perpendicularD(const Point2d& p, const Point2d& line_p1, const Point2d& line_p2)
{
    Point2d vec1 = Point2d(p.x_ - line_p1.x_, p.y_ - line_p1.y_);
    Point2d vec2 = Point2d(line_p2.x_ - line_p1.x_, line_p2.y_ - line_p1.y_);
    float d_vec2 = sqrt(vec2.x_*vec2.x_ + vec2.y_*vec2.y_);
    float cross_product = vec1.x_*vec2.y_ - vec2.x_*vec1.y_;
    float d = abs(cross_product / d_vec2);
    return d;
}
