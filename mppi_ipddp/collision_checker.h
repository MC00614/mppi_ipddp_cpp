#pragma once

#include "model_base.h"

#include <eigen3/Eigen/Dense>

#include <vector>
#include <array>

class CollisionChecker {
public:
    CollisionChecker();
    ~CollisionChecker();

    // x, y, r, r^2
    std::vector<std::array<double, 4>> circles;
    // x1, x2, y1, y2
    std::vector<std::array<double, 4>> rectangles;

    void addCircle(double x, double y, double r);
    void addRectangle(double x, double y, double w, double h);

    bool getCollisionGrid(const Eigen::VectorXd &x);
    bool getCollisionCircle(const Eigen::MatrixXd &z);
};

CollisionChecker::CollisionChecker() {
    circles.clear();
    rectangles.clear();
}

CollisionChecker::~CollisionChecker() {
}

void CollisionChecker::addCircle(double x, double y, double r) {
    circles.push_back({x, y, r, r*r});
}

void CollisionChecker::addRectangle(double x, double y, double w, double h) {
    rectangles.push_back({x, x + w, y, y + h});
}

bool CollisionChecker::getCollisionGrid(const Eigen::VectorXd &x) {
    double dx;
    double dy;
    double distance_2;
    // Circle
    for (int i = 0; i < circles.size(); ++i) {
        dx = circles[i][0] - x(0);
        dy = circles[i][1] - x(1);
        distance_2 = (dx * dx) + (dy * dy);
        if (distance_2 <= circles[i][3]) {return true;}
        else {continue;}
    }
    // Rectangle
    for (int i = 0; i < rectangles.size(); ++i) {
        if (x(0) < rectangles[i][0]) {continue;}  
        else if (rectangles[i][1] < x(0)) {continue;}  
        else if (x(1) < rectangles[i][2]) {continue;}  
        else if (rectangles[i][3] < x(1)) {continue;}  
        else {return true;}
    }
    return false;
}

bool CollisionChecker::getCollisionCircle(const Eigen::MatrixXd &z) {
    Eigen::VectorXd zj;
    double dx;
    double dy;
    double distance_2;
    double dc;
    // Circle
    for (int i = 0; i < circles.size(); ++i) {
        dx = circles[i][0] - z(0);
        dy = circles[i][1] - z(1);
        distance_2 = (dx * dx) + (dy * dy);
        dc = circles[i][2] + z(2);
        if (distance_2 <= (dc*dc)) {return true;}
        else {continue;}
    }
    // Rectangle
    for (int i = 0; i < rectangles.size(); ++i) {
        if ((z(0) + z(2)) < rectangles[i][0]) {continue;}
        else if (rectangles[i][1] < (z(0) - z(2))) {continue;}
        else if ((z(1) + z(2)) < rectangles[i][2]) {continue;}
        else if (rectangles[i][3] < (z(1) - z(2))) {continue;}
        else {return true;}
    }
    // // Circle
    // for (int i = 0; i < circles.size(); ++i) {
    //     for (int j = 0; j < z.cols(); ++j) {
    //         zj = z.col(j);
    //         dx = circles[i][0] - zj(0);
    //         dy = circles[i][1] - zj(1);
    //         distance_2 = (dx * dx) + (dy * dy);
    //         dc = circles[i][2] + zj(2);
    //         if (distance_2 <= (dc*dc)) {return true;}
    //         else {continue;}
    //     }
    // }
    // // Rectangle
    // for (int i = 0; i < rectangles.size(); ++i) {
    //     for (int j = 0; j < z.cols(); ++j) {
    //         zj = z.col(j);
    //         if ((zj(0) + zj(2)) < rectangles[i][0]) {continue;}
    //         else if (rectangles[i][1] < (zj(0) - zj(2))) {continue;}
    //         else if ((zj(1) + zj(2)) < rectangles[i][2]) {continue;}
    //         else if (rectangles[i][3] < (zj(1) - zj(2))) {continue;}
    //         else {return true;}
    //     }
    // }
    return false;
}