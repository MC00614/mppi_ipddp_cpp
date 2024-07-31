#include <Eigen/Dense>

#include <iomanip>

Eigen::MatrixXd firstDerivative(const Eigen::MatrixXd &matrix) {
    int rows = matrix.rows();
    int cols = matrix.cols();
    Eigen::MatrixXd derivative(rows, cols - 1);

    for (int i = 0; i < cols - 1; ++i) {
        derivative.col(i) = matrix.col(i + 1) - matrix.col(i);
    }

    return derivative;
}

Eigen::MatrixXd secondDerivative(const Eigen::MatrixXd &matrix) {
    Eigen::MatrixXd first_deriv = firstDerivative(matrix);
    return firstDerivative(first_deriv);
}

double meanSquaredCurvature(const Eigen::MatrixXd &matrix) {
    Eigen::MatrixXd second_deriv = secondDerivative(matrix);
    double mean_squared_curvature = second_deriv.array().square().mean();
    return mean_squared_curvature;
}

double totalVariation(const Eigen::MatrixXd &matrix) {
    Eigen::MatrixXd first_deriv = firstDerivative(matrix);
    double total_variation = first_deriv.array().abs().sum();
    return total_variation;
}