#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "parameter.h"

#include <chrono>

int main() {
    // Model
    auto model = WMRobot();

    // Collision Checker
    CollisionChecker collision_checker;
    // collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    // collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    // collision_checker.addCircle(0.5, 1.0, 0.25);
    collision_checker.addCircle(0.0, 1.0, 0.3);
    collision_checker.addCircle(-0.85, 3.5, 1.6);
    collision_checker.addCircle(2.0, 3.5, 1.0);

    clock_t start;
    clock_t finish;

    Eigen::VectorXd final_state(model.dim_x);
    final_state << 0.0, 6.0, M_PI_2;
    Eigen::MatrixXd res_X = model.X.col(0);
    Eigen::MatrixXd res_U = model.U.col(0);
    int max_iter = 100;
    double total_duration = 0.0;
    bool is_failed;
    int fail = 0;
    int sim_maxiter = 1;

    double total_msc_x = 0.0;
    double total_msc_u = 0.0;
    double total_tv_x = 0.0;
    double total_tv_u = 0.0;
    double msc_x = 0.0;
    double msc_u = 0.0;
    double tv_x = 0.0;
    double tv_u = 0.0;

    std::cout << "MPPI-IPDDP (" << sim_maxiter << " simulations)" << std::endl;
    std::cout << "iter_duration\titer\tfinal_error\tfailed\tmsc_x\t\tmsc_u\t\ttv_x\t\ttv_u" << std::endl;

    for (int t = 0; t < sim_maxiter; ++t) {
        is_failed = false;
        // MPPI Parameter
        MPPIParam mppi_param;
        mppi_param.Nu = 300;
        mppi_param.gamma_u = 100.0;
        Eigen::VectorXd sigma_u(model.dim_u);
        sigma_u << 0.5, 0.5;
        mppi_param.sigma_u = sigma_u.asDiagonal();
        
        // Corridor Parameter
        CorridorParam corridor_param;
        corridor_param.max_iter = 3;
        corridor_param.Nz = 1000;
        corridor_param.gamma_z = 1000.0;
        Eigen::VectorXd sigma_z(model.center_point + 1);
        sigma_z << 0.3, 0.3, 0.1;
        corridor_param.sigma_z = sigma_z.asDiagonal();
        corridor_param.lambda_c = 35.0;
        corridor_param.lambda_r = 35.0;
        corridor_param.r_max = 0.5;

        // IPDDP Parameter
        Param ipddp_param;
        ipddp_param.tolerance = 1e-7;
        ipddp_param.max_iter = 100;
        ipddp_param.mu = 0.01;
        ipddp_param.infeasible = true;
        ipddp_param.q = 0.001;

        // MPPI_IPDDP
        MPPI_IPDDP mppi_ipddp(model);
        mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
        mppi_ipddp.setCollisionChecker(&collision_checker);

        double mppi_ipddp_duration = 0.0;
        double mppi_ipddp_duration1 = 0.0;
        double mppi_ipddp_duration2 = 0.0;
        double mppi_ipddp_duration3 = 0.0;
        int i;
        for (i = 0; i < max_iter; ++i) {
            // MPPI_IPDDP
            mppi_ipddp.solve(1);

            res_X.conservativeResize(res_X.rows(), res_X.cols()+1);
            res_X.col(res_X.cols()-1) = mppi_ipddp.X.col(0);
            res_U.conservativeResize(res_U.rows(), res_U.cols()+1);
            res_U.col(res_U.cols()-1) = mppi_ipddp.U.col(0);
            std::cout<<mppi_ipddp.X.col(0).transpose()<<std::endl;

            mppi_ipddp.move();

            if ((final_state - mppi_ipddp.X.col(0)).norm() < 0.1) {
                break;
            }
        }
    }
    std::cout<<"res_X"<<std::endl;
    for (int i = 0; i < res_X.rows(); ++i) {
        for (int j = 0; j < res_X.cols(); ++j) {
            std::cout<<res_X.row(i).col(j)<<std::endl;
        }
        std::cout<<"\n"<<std::endl;
    }
    std::cout<<"res_U"<<std::endl;
    for (int i = 0; i < res_U.rows(); ++i) {
        for (int j = 0; j < res_U.cols(); ++j) {
            std::cout<<res_U.row(i).col(j)<<std::endl;
        }
        std::cout<<"\n"<<std::endl;
    }

    return 0;
}