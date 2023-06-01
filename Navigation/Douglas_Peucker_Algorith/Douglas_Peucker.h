//
// Created by xinha on 6/21/2022.
//

#include <vector>

// create a template for 2D vector
namespace RDP
{
    struct Point2d
    {
        float x_, y_;
        Point2d() :x_(0.0), y_(0.0) {}
        Point2d(float x, float y) :x_(x), y_(y) {}
    };
}

// Declare functions
namespace RDP
{
    class DouglasP
    {
    public:
        static std::vector<Point2d> pointFilter(std::vector<Point2d>& pointList, float epsilon);
    private:
        static float perpendicularD(const Point2d& p, const Point2d& line_p1, const Point2d& line_p2);
    };
}

#ifndef DOUGLAS_PEUCKER_ALGORITHM_DOUGLAS_PEUCKER_H
#define DOUGLAS_PEUCKER_ALGORITHM_DOUGLAS_PEUCKER_H

#endif //DOUGLAS_PEUCKER_ALGORITHM_DOUGLAS_PEUCKER_H
