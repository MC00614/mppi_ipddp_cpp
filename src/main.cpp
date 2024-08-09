#include "wmrobot.h"

#include "collision_checker.h"
#include "parameter.h"

#include "mppi.h"
#include "log_mppi.h"
#include "smooth_mppi.h"
#include "mppi_ipddp.h"

#include <chrono>

int main(int argc, char* argv[]) {
    std::string target;
    int sim_maxiter;

    if (argc > 2) {
        target = argv[1];
        sim_maxiter = std::stoi(argv[2]);
    }
    else {
        std::cerr << "Error: No target specified.\n";
        std::cerr << "Usage: " << argv[0] << " <target name>"  << " <sim iteration>\n";
        return 1;
    }

    // Model
    auto model = WMRobot();

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

    // Collision Checker
    CollisionChecker collision_checker;
    collision_checker.addRectangle(-2.5, 2.0, 3.0, 2.0);
    collision_checker.addRectangle(1.0, 2.0, 3.0, 2.0);
    collision_checker.addCircle(0.5, 1.0, 0.25);

    clock_t start;
    clock_t finish;

    Eigen::VectorXd final_state(model.dim_x);
    final_state << 0.0, 6.0, M_PI_2;
    Eigen::MatrixXd res_X, res_U;
    double max_sim_duration = 10.0;
    bool is_failed;

    // 100 200 400 800 1600 3200 6400 12800 25600
    std::vector<int> N;
    for (int n = 0; n < 9; ++n) {
        N.push_back(100 * std::pow(2, n));
    }
    std::vector<double> SIGMA_U;
    for (int n = 1; n < 10; ++n) {
        SIGMA_U.push_back(0.1 * n);
    }

    std::cout<<"Target = "<<target<<std::endl;
    std::cout<<"Simulate "<<sim_maxiter<<" times for each"<<std::endl;

    std::cout << "N\tS_u\tP\tF\ta_msc_x\ta_msc_u\ta_tv_x\t\ta_tv_u\t\tavg_time\tmin_time\tmax_time" << std::endl;

    for (int p1 = 0; p1 < N.size(); ++p1) {
        for (int p2 = 0; p2 < SIGMA_U.size(); ++p2) {
            // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //
            if (target == "MPPI") {
                mppi_param.Nu = N[p1];
                mppi_param.gamma_u = 100.0;
                mppi_param.sigma_u = SIGMA_U[p2] * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
            }
            else if (target == "Log-MPPI") {
                log_mppi_param.Nu = N[p1];
                log_mppi_param.gamma_u = 100.0;
                log_mppi_param.sigma_u = SIGMA_U[p2] * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
            }
            else if (target == "Smooth-MPPI") {
                smooth_mppi_param1.Nu = N[p1];
                smooth_mppi_param1.gamma_u = 10.0;
                smooth_mppi_param1.sigma_u = SIGMA_U[p2] * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);
                smooth_mppi_param2.dt = 1.0;
                smooth_mppi_param2.lambda = 15.0;
                Eigen::VectorXd w(model.dim_u);
                w << 0.8, 0.8;
                smooth_mppi_param2.w = w.asDiagonal();
            }
            else if (target == "MPPI-IPDDP") {
                mi_mppi_param.Nu = N[p1];
                mi_mppi_param.gamma_u = 100.0;
                mi_mppi_param.sigma_u = SIGMA_U[p2] * Eigen::MatrixXd::Identity(model.dim_u, model.dim_u);

            }
            // PARAMETERS // PARAMETERS // PARAMETERS // PARAMETERS //

            int fail = 0;

            double total_msc_x = 0.0;
            double total_msc_u = 0.0;
            double total_tv_x = 0.0;
            double total_tv_u = 0.0;

            double total_duration = 0.0;
            double max_duration = 0.0;
            double min_duration = max_sim_duration;
            
            for (int t = 0; t < sim_maxiter; ++t) {
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
                    }

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
                }
                else {fail++;}
                double fs_error = (final_state - res_X.col(model.N)).norm();
                total_msc_x += msc_x;
                total_msc_u += msc_u;
                total_tv_x += tv_x;
                total_tv_u += tv_u;
            }
            std::cout << std::fixed << std::setprecision(2);
            // std::cout.fill(' ');
            // std::cout.width(8);
            int success = std::max(1, sim_maxiter - fail);
            std::cout<<N[p1]<<'\t'<<SIGMA_U[p2]<<'\t'<<sim_maxiter - fail<<'\t'<<fail<<"\t";
            std::cout << std::fixed << std::setprecision(6);
            std::cout.fill('0');
            std::cout.width(8);
            std::cout<<(total_msc_x/success)<<'\t'<<(total_msc_u/success)<<'\t'<<(total_tv_x/success)<<'\t'<<(total_tv_u/success)<<'\t'<<total_duration/success<<'\t'<<min_duration<<'\t'<<max_duration<<std::endl;
        }
    }

    return 0;
}