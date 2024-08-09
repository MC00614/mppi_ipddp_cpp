#pragma once

#include "model_base.h"

#include <eigen3/Eigen/Dense>

#include <vector>
#include <array>
#include <fstream>

class CollisionChecker {
public:
    CollisionChecker();
    ~CollisionChecker();

    void loadMap(const std::string& file_path, double resolution);

    // x, y, r, r^2
    std::vector<std::array<double, 4>> circles;
    // x1, x2, y1, y2
    std::vector<std::array<double, 4>> rectangles;

    void addCircle(double x, double y, double r);
    void addRectangle(double x, double y, double w, double h);

    bool getCollisionGrid(const Eigen::VectorXd &x);
    bool getCollisionCircle(const Eigen::VectorXd &z);
    bool getCollisionGrid_polygon(const Eigen::VectorXd &x);
    bool getCollisionCircle_polygon(const Eigen::VectorXd &z);
    bool getCollisionGrid_map(const Eigen::VectorXd &x);
    bool getCollisionCircle_map(const Eigen::VectorXd &z);

private:
    bool with_map;
    std::vector<std::vector<double>> map;
    double resolution;
    int max_row;
    int max_col;
};

CollisionChecker::CollisionChecker() {
    circles.clear();
    rectangles.clear();
    with_map = false;
}

CollisionChecker::~CollisionChecker() {
}

void CollisionChecker::loadMap(const std::string& file_path, double resolution) {
    map.clear();
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::string::size_type sz = 0;

        if (!file.is_open()) {
            throw std::runtime_error("Error opening file: " + file_path);
        }

        while (sz < line.length()) {
            double value = std::stod(line, &sz);
            row.push_back(value);
            line = line.substr(sz);
        }

        map.push_back(row);
    }

    file.close();

    with_map = true;
    this->resolution = resolution;
    max_row = map.size();
    max_col = map[0].size();
}

void CollisionChecker::addCircle(double x, double y, double r) {
    circles.push_back({x, y, r, r*r});
}

void CollisionChecker::addRectangle(double x, double y, double w, double h) {
    rectangles.push_back({x, x + w, y, y + h});
}

bool CollisionChecker::getCollisionGrid(const Eigen::VectorXd &x) {
    if (with_map) {return getCollisionGrid_map(x);}
    else {return getCollisionGrid_polygon(x);}
}

bool CollisionChecker::getCollisionCircle(const Eigen::VectorXd &x) {
    if (with_map) {return getCollisionCircle_map(x);}
    else {return getCollisionCircle_polygon(x);}
}

bool CollisionChecker::getCollisionGrid_polygon(const Eigen::VectorXd &x) {
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

bool CollisionChecker::getCollisionCircle_polygon(const Eigen::VectorXd &z) {
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
    return false;
}

bool CollisionChecker::getCollisionGrid_map(const Eigen::VectorXd &x) {
    // Need to check comparison double
    int nx = round(x(0)/resolution);
    int ny = round(x(1)/resolution);
    if (nx < 0 || max_row <= nx) {return false;}
    if (nx < 0 || max_col <= ny) {return false;}
    if (map[nx][ny] == 10) {return true;}
    return false;
}

bool CollisionChecker::getCollisionCircle_map(const Eigen::VectorXd &z) {
    // Need to check comparison double
    int cx = round(z(0)/resolution);
    int cy = round(z(1)/resolution);
    double r = z(2)/resolution + 0.5;
    if (cx < 0 || max_row <= cx) {return true;}
    if (cy < 0 || max_col <= cy) {return true;}
    if (map[cx][cy] == 10) {return true;}
    if (map[cx][cy] == 5) {return false;}
    if (map[cx][cy] < r) {return true;}
    return false;
}