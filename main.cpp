#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <fmt/core.h>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Sophus::SE3d;

using ceres::AutoDiffCostFunction;
using ceres::LocalParameterization;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class CustomCeresCallback : public ceres::IterationCallback {
    public:
        CustomCeresCallback(const Problem& _problem):
            problem(_problem) {
            logger.open("../data/log.csv", std::ios_base::out);
            logger << "iteration, loss, angle, ax, ay, az, tx, ty, tz\n";
        }
            
        virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
            std::vector<double*> paramBlocks;
            problem.GetParameterBlocks(&paramBlocks);
            auto blockCount = problem.NumParameterBlocks();
            
            for(const auto& block : paramBlocks){
                auto size = problem.ParameterBlockSize(block);
                
                Eigen::Map<SE3d const> const pose(block);
                Quaterniond so3(pose.so3().data());
                AngleAxisd aa(so3);

                logger << summary.iteration << ", "
                        << summary.cost << ", "
                        << aa.angle() << ", "
                        << aa.axis()[0] << ", "
                        << aa.axis()[1] << ", "
                        << aa.axis()[2] << ", "
                        << pose.translation()[0] << ", "
                        << pose.translation()[1] << ", "
                        << pose.translation()[2] << "\n";
            }
            return ceres::SOLVER_CONTINUE;
        }
    private:
        const Problem& problem;
        std::ofstream logger;
};

class LocalParameterizationSE3 : public LocalParameterization {
    public:
        virtual ~LocalParameterizationSE3() {}

        // SE3 plus operation for Ceres
        //
        //  T * exp(x)
        //
        virtual bool Plus(double const* T_raw, double const* delta_raw,
                          double* T_plus_delta_raw) const {
            Eigen::Map<SE3d const> const T(T_raw);
            Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
            Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * SE3d::exp(delta);
            return true;
        }

        // Jacobian of SE3 plus operation for Ceres
        //
        // Dx T * exp(x)  with  x=0
        //
        virtual bool ComputeJacobian(double const* T_raw,
                                     double* jacobian_raw) const {
            Eigen::Map<SE3d const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, 
                                     SE3d::num_parameters, 
                                     SE3d::DoF, 
                                     Eigen::RowMajor>>jacobian(jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        virtual int GlobalSize() const { return SE3d::num_parameters; }

        virtual int LocalSize() const { return SE3d::DoF; }
};

struct CostFunctor {
    CostFunctor(const Vector3d& target,
                const Vector3d& input):
        input(input), target(target){}

    template <typename T>
    bool operator()(T const * const se3_raw, T* residual_raw) const {
        Eigen::Map<Sophus::SE3<T> const> const se3(se3_raw);
        Eigen::Map<Eigen::Vector<T,3>> residual(residual_raw);
        residual = target - se3*input;
        return true;
    }

    Vector3d input;
    Vector3d target;
};

enum ICP_SOLVER_TYPE{
    SteepestDescent = 0,
    LBFGS = 1,
    LM = 2
};

inline const char* ToString(ICP_SOLVER_TYPE& v){
    switch (v) {
        case ICP_SOLVER_TYPE::SteepestDescent:
            return "SteepestDescent";
        case ICP_SOLVER_TYPE::LBFGS:
            return "LBFGS";
        case ICP_SOLVER_TYPE::LM:
            return "LM";
        default:
            return "Invalid solver type.";
    }
}

int main(int argc, char** argv){
    ICP_SOLVER_TYPE solverType;

    if (argc!=2){
        fmt::print("No argument provided, default solver set to LM\n");
        solverType = ICP_SOLVER_TYPE::LM;
    }
    else{
        int input = std::stoi(argv[1]);
        assert(input<3 && "solver type: 0 [SteepestDescent], 1 [LBFGS], 2 [LM]");
        solverType = ICP_SOLVER_TYPE(input);
    }
    fmt::print("solverType: {0}\n", ToString(solverType));

    Vector3d axis_groundtruth_inv;
    axis_groundtruth_inv << 0,0,1;
    AngleAxisd aa_groundtruth_inv(-M_PI/3.0,axis_groundtruth_inv.normalized());
    Quaterniond q_groundtruth_inv(aa_groundtruth_inv);
        
    Vector3d t_groundtruth_inv;
    t_groundtruth_inv << -0.300,0.100,0.;    
    
    SE3d T_tmp(q_groundtruth_inv,t_groundtruth_inv);
    auto T_groundtruth = T_tmp.inverse();
    std::cout<<"SE3 in HTM:"<<std::endl;
    std::cout<<T_groundtruth.matrix()<<std::endl;
    std::cout<<std::endl;
        
    double angle_groundtruth;
    double ax_groundtruth,ay_groundtruth,az_groundtruth;  
    double tx_groundtruth,ty_groundtruth,tz_groundtruth;

    Quaterniond q_groundtruth(T_groundtruth.so3().data());
    AngleAxisd  aa_groundtruth(q_groundtruth);
    angle_groundtruth = aa_groundtruth.angle();
    ax_groundtruth=aa_groundtruth.axis()[0];
    ay_groundtruth=aa_groundtruth.axis()[1];;
    az_groundtruth=aa_groundtruth.axis()[2];;
    tx_groundtruth=T_groundtruth.translation()[0];
    ty_groundtruth=T_groundtruth.translation()[1];
    tz_groundtruth=T_groundtruth.translation()[2];
    
    std::cout<<"angle_groundtruth: "<<angle_groundtruth<<std::endl;
    std::cout<<"ax_groundtruth: "<<ax_groundtruth<<std::endl;
    std::cout<<"ay_groundtruth: "<<ay_groundtruth<<std::endl;
    std::cout<<"az_groundtruth: "<<az_groundtruth<<std::endl;
    std::cout<<"tx_groundtruth: "<<tx_groundtruth<<std::endl;
    std::cout<<"ty_groundtruth: "<<ty_groundtruth<<std::endl;
    std::cout<<"tz_groundtruth: "<<tz_groundtruth<<std::endl;
    std::cout<<std::endl;

    std::string dataFile = "../data/bun_zipper.xyz";
    unsigned line_count;
    {
        std::ifstream fileForCounting(dataFile);
        // new lines will be skipped unless we stop it from happening:
        fileForCounting.unsetf(std::ios_base::skipws);
        // count the newlines with an algorithm specialized for counting:
        line_count= std::count(
                std::istream_iterator<char>(fileForCounting),
                std::istream_iterator<char>(),
                '\n');
        std::cout << "Line count: " << line_count << std::endl;
    }

    int N = line_count;
    MatrixXd points_(N,3);
    std::ifstream file(dataFile);
    std::string line;
    int currentIdx = 0;
    while(std::getline(file, line)){
        std::vector<std::string> substrings;
        boost::split(substrings,line,boost::is_any_of(" "));
        if(substrings.size()==4){
            points_.row(currentIdx)<<std::stod(substrings[0]),
                            std::stod(substrings[1]),
                            std::stod(substrings[2]);
            currentIdx+=1;
        }
    }
    assert(currentIdx == N && "Loaded vertices count does not match file line count.");
    std::cout<<N<<" points loaded:"<<std::endl;
    auto targetPoints = points_.transpose();

    MatrixXd sourcePoints(3,N);
    for(uint32_t i=0; i<N; i++){
        Vector3d p = targetPoints.col(i);
        auto transP = T_tmp*p;
        sourcePoints.col(i)=transP;
    }
    
    double angle(0);
    Vector3d axis;
    axis << 0,0,0;
    AngleAxisd aa(angle,axis.normalized());
    Quaterniond q(aa);
    
    Vector3d t;
    t << 0,0,0;
    SE3d T(q,t);
    
    double angle_init = angle;
    double ax_init = axis[0];
    double ay_init = axis[1];
    double az_init = axis[2];
    double tx_init = t[0];
    double ty_init = t[1];
    double tz_init = t[2];
    
    ceres::Problem problem;
    problem.AddParameterBlock(T.data(),
                                SE3d::num_parameters,
                                new LocalParameterizationSE3);
    for(uint32_t i=0; i<N; i++){
        Vector3d target = targetPoints.col(i);
        Vector3d input = sourcePoints.col(i);
        CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 
                                     3,                     // number of residual
                                     SE3d::num_parameters   // number of parameter for first parameter block 
                                    >(new CostFunctor(target, input));
        problem.AddResidualBlock(cost_function, NULL,T.data());
    }
    
    // Run the solver!
    Solver::Options options;
    options.max_num_iterations = 50;
    options.parameter_tolerance = 1e-6;

    switch (solverType) {
        case ICP_SOLVER_TYPE::SteepestDescent:
            // Steepest descent
            options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
            options.line_search_direction_type = ceres::LineSearchDirectionType::STEEPEST_DESCENT;
            options.line_search_type = ceres::LineSearchType::ARMIJO;
            options.line_search_interpolation_type = ceres::LineSearchInterpolationType::CUBIC;
            break;
        case ICP_SOLVER_TYPE::LBFGS:
            // L-BFGS
            options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
            options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
            options.max_lbfgs_rank = 20;
            options.line_search_type = ceres::LineSearchType::WOLFE;
            options.line_search_interpolation_type = ceres::LineSearchInterpolationType::CUBIC;
            break;
        case ICP_SOLVER_TYPE::LM:
            // Levenberg-Marquardt
            options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
            options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
            options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
            break;
        default:
            fmt::print("Invalid solver type.\n");
            return -1;
    }
    
    options.update_state_every_iteration = true;
    CustomCeresCallback myCallback(problem);
    options.callbacks.push_back(&myCallback);
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << std::endl;
    
    Quaterniond q_out(T.so3().data());
    AngleAxisd aa_out(q_out);
    auto ax_optimized = aa_out.axis()[0];
    auto ay_optimized = aa_out.axis()[1];
    auto az_optimized = aa_out.axis()[2];
    
    std::cout<<"\tInitial\t\tOptimized\tGroundTruth"<<std::endl;
    fmt::print("angle\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",angle_init,aa_out.angle(),angle_groundtruth);
    
    fmt::print("axis.x\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",ax_init,ax_optimized,ax_groundtruth);
    fmt::print("axis.y\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",ay_init,ay_optimized,ay_groundtruth);
    fmt::print("axis.z\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",az_init,az_optimized,az_groundtruth);
    fmt::print("tx\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",tx_init,T.translation()[0],tx_groundtruth);
    fmt::print("ty\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",ty_init,T.translation()[1],ty_groundtruth);
    fmt::print("tz\t{0:.4f}\t\t{1:.4f}\t\t{2:.4f}\n",tz_init,T.translation()[2],tz_groundtruth);
    
    return 0;
}

