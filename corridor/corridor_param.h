#include <Eigen/Dense>

struct CorridorParam {
    int max_iter;
    int Nz;
    double gamma_z;
    Eigen::MatrixXd sigma_z;
    double lambda_c;
    double lambda_r;
    double r_max;
};