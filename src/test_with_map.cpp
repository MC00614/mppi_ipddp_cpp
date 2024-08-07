#include "wmrobot_map.h"

#include "collision_checker.h"
#include "show.h"
#include "parameter.h"

#include "mppi.h"
#include "log_mppi.h"
#include "smooth_mppi.h"
#include "mppi_ipddp.h"

#include <chrono>

int main(int argc, char* argv[]) {
    std::string target;

    if (argc > 1) {
        target = argv[1];
    }

    // Model
    auto model = WMRobotMap();

    // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //
    // MPPI
    MPPIParam mppi_param;

    // Log_MPPI
    MPPIParam log_mppi_param;

    // Smooth_MPPI
    MPPIParam smooth_mppi_param1;
    SmoothMPPIParam smooth_mppi_param2;

    //MPPI_IPDDP
    MPPIParam mi_mppi_param;
    CorridorParam corridor_param;
    // Corridor Parameter
    corridor_param.max_iter = 3;
    corridor_param.Nz = 1000;
    corridor_param.gamma_z = 1000.0;
    Eigen::VectorXd sigma_z(model.center_point + 1);
    sigma_z << 0.3, 0.3, 0.1;
    corridor_param.sigma_z = sigma_z.asDiagonal();
    corridor_param.lambda_c = 35.0;
    corridor_param.lambda_r = 35.0;
    corridor_param.r_max = 0.5;

    Param ipddp_param;
    // IPDDP Parameter
    ipddp_param.tolerance = 1e-7;
    ipddp_param.max_iter = 100;
    ipddp_param.mu = 0.01;
    ipddp_param.infeasible = true;
    ipddp_param.q = 0.001;
    // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //

    clock_t start;
    clock_t finish;

    Eigen::VectorXd final_state(model.dim_x);
    final_state << 1.5, 5.0, M_PI_2;
    Eigen::MatrixXd res_X, res_U;
    double max_sim_duration = 1.0;
    bool is_failed;

    std::cout<<"Target = "<<target<<std::endl;

    std::cout << "map_number\titer_duration\tfinal_error\tfailed\tmsc_x\t\tmsc_u\t\ttv_x\t\ttv_u" << std::endl;
    // std::cout << "N\tS_u\tP\tF\ta_msc_x\ta_msc_u\ta_tv_x\t\ta_tv_u\t\tavg_time\tmin_time\tmax_time" << std::endl;

    int fail = 0;
    double total_msc_x = 0.0;
    double total_msc_u = 0.0;
    double total_tv_x = 0.0;
    double total_tv_u = 0.0;

    double total_duration = 0.0;
    double max_duration = 0.0;
    double min_duration = max_sim_duration;

    for (int i = 0; i < 300; ++i) {
    // for (int i = 299; i > -1; --i) {
        // Collision Checker
        CollisionChecker collision_checker;
        std::string map_file_path = "../BARN_dataset/inflated_txt_files/output_" + std::to_string(i) + ".txt";
        collision_checker.loadMap(map_file_path, 0.1);

        // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //
        if (target == "MPPI") {
            mppi_param.Nu = 3200;
            mppi_param.gamma_u = 100.0;
            mppi_param.sigma_u = 0.2 * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
        }
        else if (target == "Log-MPPI") {
            log_mppi_param.Nu = 3200;
            log_mppi_param.gamma_u = 100.0;
            log_mppi_param.sigma_u = 0.1 * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
        }
        else if (target == "Smooth-MPPI") {
            smooth_mppi_param1.Nu = 12800;
            smooth_mppi_param1.gamma_u = 10.0;
            smooth_mppi_param1.sigma_u = 0.3 * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
            smooth_mppi_param2.dt = 1.0;
            smooth_mppi_param2.lambda = 15.0;
            Eigen::VectorXd w(model.dim_u);
            w << 0.8, 0.8;
            smooth_mppi_param2.w = w.asDiagonal();
        }
        else if (target == "MPPI-IPDDP") {
            mi_mppi_param.Nu = 1600;
            mi_mppi_param.gamma_u = 100.0;
            mi_mppi_param.sigma_u = 0.4 * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
        }
        // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //
        
        double msc_x = 0.0; 
        double msc_u = 0.0;
        double tv_x = 0.0;
        double tv_u = 0.0;

        is_failed = false;
        
        // MPPI
        MPPI mppi(model);

        // Log_MPPI
        LogMPPI log_mppi(model);

        // Smooth_MPPI
        SmoothMPPI smooth_mppi(model);

        // MPPI_IPDDP
        MPPI_IPDDP mppi_ipddp(model);

        if (target == "MPPI") {
            mppi.init(mppi_param);
            mppi.setCollisionChecker(&collision_checker);            
        }
        else if (target == "Log-MPPI") {
            log_mppi.init(log_mppi_param);
            log_mppi.setCollisionChecker(&collision_checker);
        }
        else if (target == "Smooth-MPPI") {
            smooth_mppi.init2(smooth_mppi_param1, smooth_mppi_param2);
            smooth_mppi.setCollisionChecker(&collision_checker);
        }
        else if (target == "MPPI-IPDDP") {
            mppi_ipddp.init(mi_mppi_param, corridor_param, ipddp_param);
            mppi_ipddp.setCollisionChecker(&collision_checker);
        }
        
        double iter_duration = 0.0;
        while (true) {
            if (target == "MPPI") {
                auto start = std::chrono::high_resolution_clock::now();
                mppi.solve();
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                iter_duration += elapsed.count();
                res_X = mppi.getResX();
                res_U = mppi.getResU();
            }
            else if (target == "Log-MPPI") {
                auto start = std::chrono::high_resolution_clock::now();
                log_mppi.solve();
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                iter_duration += elapsed.count();
                res_X = log_mppi.getResX();
                res_U = log_mppi.getResU();
            }
            else if (target == "Smooth-MPPI") {
                auto start = std::chrono::high_resolution_clock::now();
                smooth_mppi.solve();
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                iter_duration += elapsed.count();
                res_X = smooth_mppi.getResX();
                res_U = smooth_mppi.getResU();
            }
            else if (target == "MPPI-IPDDP") {
                mppi_ipddp.solve(1);
                iter_duration += mppi_ipddp.mppi_duration+mppi_ipddp.corridor_duration+mppi_ipddp.ipddp_duration;
                res_X = mppi_ipddp.X;
                res_U = mppi_ipddp.U;
                // show2D(res_X, res_U, mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
            }

            // show2D(res_X, res_U, map_file_path, 0.1);
            // show2D(res_X, res_U, mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);


            if (max_sim_duration < iter_duration) {
                is_failed = true;
                break;
            }

            if ((final_state - res_X.col(model.N)).norm() < 0.1) {
                bool is_collision = false;
                for (int j = 0; j < model.N; ++j) {
                    if (collision_checker.getCollisionGrid(res_X.col(j))) {
                        is_collision = true;
                        break;
                    }
                }
                if (!is_collision) {
                    break;
                }
            }
        }
        if (!is_failed) {
            total_duration += iter_duration;
            min_duration = std::min(min_duration, iter_duration);
            max_duration = std::max(max_duration, iter_duration);
            msc_x = meanSquaredCurvature(res_X);
            msc_u = meanSquaredCurvature(res_U);
            tv_x = totalVariation(res_X);
            tv_u = totalVariation(res_U);

            // show2D(res_X, res_U, map_file_path, 0.1);
            // show2D(res_X, res_U, mppi_ipddp.X, mppi_ipddp.U, mppi_ipddp.C, mppi_ipddp.R, collision_checker.circles, collision_checker.rectangles);
        }
        else {fail++;}
        double fs_error = (final_state - res_X.col(model.N)).norm();
        total_msc_x += msc_x;
        total_msc_u += msc_u;
        total_tv_x += tv_x;
        total_tv_u += tv_u;
        std::cout << std::fixed << std::setprecision(6);
        std::cout.fill('0');
        std::cout.width(8);
        std::cout<<i<<'\t'<<iter_duration<<'\t'<<fs_error<<'\t'<<(int)is_failed<<"\t";
        std::cout.fill('0');
        std::cout.width(8);
        std::cout<<msc_x<<'\t'<<msc_u<<'\t'<<tv_x<<'\t'<<tv_u<<std::endl;

        // std::cout << std::fixed << std::setprecision(2);
        // std::cout.fill(' ');
        // std::cout.width(8);
        // int success = std::max(1, sim_maxiter - fail);
        // std::cout<<100<<'\t'<<0.1<<'\t'<<sim_maxiter - fail<<'\t'<<fail<<"\t";
        // std::cout << std::fixed << std::setprecision(6);
        // std::cout.fill('0');
        // std::cout.width(8);
        // std::cout<<(total_msc_x/success)<<'\t'<<(total_msc_u/success)<<'\t'<<(total_tv_x/success)<<'\t'<<(total_tv_u/success)<<'\t'<<total_duration/success<<'\t'<<min_duration<<'\t'<<max_duration<<std::endl;
    }
    // std::cout << "Parameter (N = " << 100 << ", Sigma_u = " << 0.1 << ")" << std::endl;
    std::cout << "Success Rate : " << (int)(((300 - fail)/(float)300)*100.0) << "% (Fail : " << fail << "/" << 300 << ")" << std::endl;
    std::cout << "Mean Squared Curvature X : " << total_msc_x << std::endl;
    std::cout << "Mean Squared Curvature U : " << total_msc_u << std::endl;
    std::cout << "Total Variation X : " << total_tv_x << std::endl;
    std::cout << "Total Variation U : " << total_tv_u << std::endl;
    // std::cout << "Average : " << total_duration/(300-fail) << " Seconds (Total " << total_duration << ")" << std::endl;
    return 0;
}