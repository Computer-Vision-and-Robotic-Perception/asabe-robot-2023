#include <iostream>

#include "Douglas_Peucker.h"
#include <iostream>

int main()
{
    std::vector<RDP::Point2d> pointMatrixTest;
    int i = 0;
    while (i < 100) {
        float x_cor = rand() % 10;
        float y_cor = rand() % 10;
        pointMatrixTest.push_back(RDP::Point2d(x_cor, y_cor));
        i++;
    }

    std::vector<RDP::Point2d> result = RDP::DouglasP::pointFilter(pointMatrixTest, 1.2);
    for (auto p:result)
        std::cout << p.x_ << " " << p.y_ << "\n";

    return 0;
}
