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
    collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    collision_checker.addCircle(0.5, 1.0, 0.25);

    Eigen::VectorXd final_state(model.dim_x);
    final_state << 0.0, 6.0, M_PI_2;
    Eigen::MatrixXd res_X, res_U;
    int max_iter = 100;

    // PARAMETERS // PARAMETERS // PARAMETERS //
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
    // PARAMETERS // PARAMETERS // PARAMETERS //

    // MPPI_IPDDP
    MPPI_IPDDP mppi_ipddp(model);
    mppi_ipddp.init(mppi_param, corridor_param, ipddp_param);
    mppi_ipddp.setCollisionChecker(&collision_checker);

    for (int i = 0; i < max_iter; ++i) {
        // MPPI_IPDDP
        mppi_ipddp.solve(1);

        // double mppi_ipddp_duration = mppi_ipddp.mppi_duration + mppi_ipddp.corridor_duration + mppi_ipddp.ipddp_duration;

        res_X = mppi_ipddp.X;
        res_U = mppi_ipddp.U;
        std::cout<<"\nX_last = "<<res_X.col(model.N).transpose()<<std::endl;
        std::cout<<"Offset = "<<(res_X.col(model.N) - final_state).norm()<<std::endl;
        
        if ((res_X.col(model.N) - final_state).norm() < 0.1) {
            std::cout<<"In Tolerance"<<std::endl;
            break;
        }
    }
    return 0;
}