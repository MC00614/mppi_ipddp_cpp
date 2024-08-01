#include "wmrobot.h"

#include "collision_checker.h"
#include "mppi_ipddp.h"
#include "show.h"

#include "log_mppi.h"
#include "smooth_mppi.h"

#include <chrono>

int main() {
    // Model
    auto model = WMRobot();

    // MPPI Parameter
    MPPIParam mppi_param;
    mppi_param.Nu = 100;
    mppi_param.gamma_u = 100.0;
    Eigen::VectorXd sigma_u(model.dim_u);
    sigma_u << 0.5, 0.5;
    mppi_param.sigma_u = sigma_u.asDiagonal();
    
    // Corridor Parameter
    CorridorParam corridor_param;
    corridor_param.max_iter = 5;
    corridor_param.Nz = 100;
    corridor_param.gamma_z = 1000.0;
    Eigen::VectorXd sigma_z(model.center_point + 1);
    sigma_z << 0.3, 0.3, 0.08;
    corridor_param.sigma_z = sigma_z.asDiagonal();
    corridor_param.lambda_c = 35.0;
    corridor_param.lambda_r = 35.0;
    corridor_param.r_max = 0.5;

    // IPDDP Parameter
    Param ipddp_param;
    ipddp_param.tolerance = 1e-7;
    ipddp_param.max_iter = 500;
    ipddp_param.mu = 0.01;
    ipddp_param.infeasible = true;
    ipddp_param.q = 0.001;

    // Collision Checker
    CollisionChecker collision_checker;
    collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    collision_checker.addCircle(0.5, 1.0, 0.25);

    // MPPI_IPDDP
    MPPI_IPDDP mppi_ipddp(model);
    mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
    mppi_ipddp.setCollisionChecker(&collision_checker);

    // Log_MPPI
    LogMPPI log_mppi(model);
    MPPIParam log_mppi_param1;
    log_mppi_param1.Nu = 5000;
    log_mppi_param1.gamma_u = 10.0;
    Eigen::VectorXd log_mppi_sigma_u(model.dim_u);
    log_mppi_sigma_u << 0.5, 0.5;
    log_mppi_param1.sigma_u = log_mppi_sigma_u.asDiagonal();
    log_mppi.init(log_mppi_param1);
    log_mppi.setCollisionChecker(&collision_checker);

    // Smooth_MPPI
    SmoothMPPI smooth_mppi(model);
    MPPIParam smooth_mppi_param1;
    smooth_mppi_param1.Nu = 30000;
    smooth_mppi_param1.gamma_u = 10.0;
    Eigen::VectorXd sigma_u_1(model.dim_u);
    sigma_u_1 << 0.5, 0.5;
    smooth_mppi_param1.sigma_u = sigma_u_1.asDiagonal();
    SmoothMPPIParam smooth_mppi_param2;
    smooth_mppi_param2.dt = 1.0;
    smooth_mppi_param2.lambda = 15.0;
    Eigen::VectorXd w(model.dim_u);
    w << 0.8, 0.8;
    smooth_mppi_param2.w = w.asDiagonal();

    smooth_mppi.init2(smooth_mppi_param1, smooth_mppi_param2);
    smooth_mppi.setCollisionChecker(&collision_checker);

    clock_t start;
    clock_t finish;
    double mppi_ipddp_duration = 0.0;
    double log_mppi_duration = 0.0;
    double smooth_mppi_duration = 0.0;

    int t = 0;
    Eigen::VectorXd final_state(model.dim_x);
    final_state << 0.0, 6.0, M_PI_2;
    Eigen::MatrixXd res_X, res_U;
    for (t = 0; t < 1000; ++t) {
        // MPPI_IPDDP
        mppi_ipddp.solve(1);
        mppi_ipddp_duration += mppi_ipddp.mppi_duration + mppi_ipddp.corridor_duration + mppi_ipddp.ipddp_duration;

        if ((final_state - mppi_ipddp.X.col(model.N)).norm() < 0.1) {
            std::cout<<(final_state - mppi_ipddp.X.col(model.N)).norm()<<std::endl;
            break;
        }
        
        // // Log_MPPI
        // auto start = std::chrono::high_resolution_clock::now();
        // log_mppi.solve();
        // auto finish = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = finish - start;
        // log_mppi_duration += elapsed.count();

        // res_X = log_mppi.getResX();
        // if ((final_state - res_X.col(model.N)).norm() < 0.3) {
        //     bool is_collision = false;
        //     for (int j = 0; j < model.N; ++j) {
        //         if (collision_checker.getCollisionGrid(res_X.col(j))) {
        //             is_collision = true;
        //             break;
        //         }
        //     }
        //     if (!is_collision) {
        //         break;
        //     }
        //     break;
        // }
        // else {continue;}

        // start = clock();
        // smooth_mppi.solve();
        // finish = clock();
        // smooth_mppi_duration += (double)(finish - start) / CLOCKS_PER_SEC;
        // res_X = smooth_mppi.getResX();

        // if ((final_state - res_X.col(model.N)).norm() < 0.1) {
        //     bool is_collision = false;
        //     for (int j = 0; j < model.N; ++j) {
        //         if (collision_checker.getCollisionGrid(res_X.col(j))) {
        //             is_collision = true;
        //             break;
        //         }
        //     }
        //     if (!is_collision) {
        //         break;
        //     }
        //     break;
        // }
        // else {continue;}

        
        // std::cout << mppi_ipddp.mppi_duration << '\t' << mppi_ipddp.corridor_duration << '\t' << mppi_ipddp.ipddp_duration << std::endl;
        // std::cout << "" << std::endl;

        // show2D(log_mppi.getResX(), log_mppi.getResU(), smooth_mppi.getResX(), smooth_mppi.getResU(), mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
    }
    std::cout << " Iteration : " << t << std::endl;
    std::cout << "MPPI-IPDDP : " << mppi_ipddp_duration << " Seconds" << std::endl;
    // std::cout << "SMOOTH-MPPI : " << smooth_mppi_duration << " Seconds" << std::endl;
    // std::cout << "LOG-MPPI : " << log_mppi_duration << " Seconds" << std::endl;

    // std::cout<<"Final Error : "<<(final_state - res_X.col(model.N)).norm()<<std::endl;

    // show2D(smooth_mppi.getResX(), smooth_mppi.getResU(), mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
    // show2D(log_mppi.getResX(), log_mppi.getResU(), mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);

    return 0;
}