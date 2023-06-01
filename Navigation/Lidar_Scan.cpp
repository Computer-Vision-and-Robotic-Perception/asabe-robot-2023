#include <iostream>
#include <cmath>
using namespace std;


double lidar_scan(double arr[][]) {
    int rows =  sizeof arr / sizeof arr[0];
    int cols = sizeof arr[0] / sizeof(int);
    double converted_array[rows][cols];
    /*
     * calculate x
     */
    for (int i = 0; i < cols; i++) {
        converted_array[0][i] = arr[1][i]* cos(arr[0][i])
    }
    /*
     * calculate y
     */
    for (int j = 0; j < cols; j++) {
        converted_array[1][j] = arr[1][j]* cos(arr[0][j])
    }
}