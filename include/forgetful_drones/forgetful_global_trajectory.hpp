#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "matplotlibcpp/matplotlibcpp.h"

#define DEBUG_LOG(msg) if(m_DebugEnabled)std::cout<<"\033[37;46m"<<" FORGETFUL GLOBAL TRAJECTORY "<<"\033[0m"<<"\033[36m"<<"    "<<msg<<"\033[0m"<<std::endl
#define ERROR_LOG(msg) std::cout<<"\033[31;46m"<<" FORGETFUL GLOBAL TRAJECTORY "<<"\033[0m"<<"\033[31m"<<"    "<<msg<<"\033[0m"<<std::endl



namespace forgetful_drone
{


template<typename Type> class ForgetfulGlobalTrajectory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    typedef Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> MAT;
    typedef Eigen::Matrix<Type, Eigen::Dynamic, 1> VEC;
    Eigen::IOFormat MyFmt(const int& num_tabs);


private:
    const bool m_DebugEnabled; // If true, a lot of std::cout
    
    const int m_Wp_W_N; // Number of waypoint spatial coordinates
    const int m_Wp_T_N; // Number of waypoint temporal coordinates
    const int m_Spl_N; // Number of splines
    const int m_Spl_PolyCff_N; // Number of polynomial coefficients per spline
    const int m_TrajPolyCff_N; // Number of polynomial coefficients of all splines (= number of spatial decision variables)
    
    const int m_ContDiff_N; // Number of continuos derivatives of splines
    Type m_Traj_MaxSpeed_Required; // Required max. speed of dimensional trajectory
    Type m_Traj_MaxThrust_Required; // Required max. normalized thrust of dimensional trajectory
    Type m_Traj_MaxRollPitchRate_Required; // Required max. roll/pitch rate of dimensional trajectory
    Type m_Traj_SamplingTime_Dim; // Sampling time of dimensional trajectory

    const int m_GurobiNumericFocus; // Gurobi Parameter: Computation Speed vs Numeric Effort
    const Type m_NonDimMinMax_T; // Range of non-dimensional temporal coordinates (!= 1 for numeric reasons)
    const Type m_NonDimMinMax_W; // Range of non-dimensional spatial coordinates (!= 1 for numeric reasons)
    const int m_MainOptLoopIter_MaxN; // Max. number of iterations of main optimization loop

    VEC m_Wp_X_Dim; // Dimensional waypoint spatial-x coordinates
    VEC m_Wp_Y_Dim; // Dimensional waypoint spatial-y coordinates
    VEC m_Wp_Z_Dim; // Dimensional waypoint spatial-y coordinates
    VEC m_Wp_T_Dim; // Dimensional waypoint temporal coordinates
    Type m_Shift_T; // Shift applied to dimensional waypoint spatial-x coordinates
    Type m_Shift_X; // Shift applied to dimensional waypoint spatial-y coordinates
    Type m_Shift_Y; // Shift applied to dimensional waypoint spatial-z coordinates 
    Type m_Shift_Z; // Shift applied to dimensional waypoint temporal coordinates
    Type m_Scale_T; // Scale applied to shifted dimensional waypoint spatial-x coordinates
    Type m_Scale_X; // Scale applied to shifted dimensional waypoint spatial-y coordinates
    Type m_Scale_Y; // Scale applied to shifted dimensional waypoint spatial-z coordinates 
    Type m_Scale_Z; // Scale applied to shifted dimensional waypoint temporal coordinates
    VEC m_Wp_T_NonDim; // Non-dimensional waypoint temporal coordinates (after shifting and scaling)
    VEC m_Wp_X_NonDim; // Non-dimensional waypoint spatial-x coordinates (after shifting and scaling)
    VEC m_Wp_Y_NonDim; // Non-dimensional waypoint spatial-y coordinates (after shifting and scaling)
    VEC m_Wp_Z_NonDim; // Non-dimensional waypoint spatial-z coordinates (after shifting and scaling)
    

    Type m_Traj_Duration_NonDim; // Duration of non-dim. trajectory (= last non-dimensional waypoint temporal coordinate)
    Type m_Traj_SamplingTime_NonDim; // Sampling time of non-dim. trajectory
    int m_Traj_T_N; // Number of temporal coord. of trajectory
    
    std::vector<VEC> m_Spl_T_Dim;
    std::vector<VEC> m_Spl_PosX_Dim;
    std::vector<VEC> m_Spl_PosY_Dim;
    std::vector<VEC> m_Spl_PosZ_Dim;
    
public:

    VEC m_Traj_T_Dim;
    
    VEC m_Traj_PosX_Dim;
    VEC m_Traj_PosY_Dim;
    VEC m_Traj_PosZ_Dim;
    
    VEC m_Traj_VelX_Dim;
    VEC m_Traj_VelY_Dim;
    VEC m_Traj_VelZ_Dim;

    VEC m_Traj_AccX_Dim;
    VEC m_Traj_AccY_Dim;
    VEC m_Traj_AccZ_Dim;

    VEC m_Traj_JerX_Dim;
    VEC m_Traj_JerY_Dim;
    VEC m_Traj_JerZ_Dim;

    Type m_Traj_MaxSpeed; // Max. speed of dimensional trajectory
    Type m_Traj_MaxThrust; // Max. normalized thrust of dimensional trajectory
    Type m_Traj_MaxRollPitchRate; // Max. roll/pitch rate of dimensional trajectory

    Type m_Traj_MinSpeed; // Min. speed of dimensional trajectory
    Type m_Traj_MinThrust; // Min. normalized thrust of dimensional trajectory
    Type m_Traj_MinRollPitchRate; // Min. roll/pitch rate of dimensional trajectory

    bool m_Successful;



public:
    ForgetfulGlobalTrajectory(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& waypoints,
        const int& polynomial_order,
        const int& m_GTContiOrder,
        const Type& max_speed,
        const Type& max_thrust,
        const Type& max_roll_pitch_rate,
        const Type& sampling_time,
        const int& gurobi_numeric_focus,
        const Type& non_dim_range_temporal,
        const Type& non_dim_range_spatial,
        const bool& DebugOn);
    void dedimensionalizeCoordinates(
        const VEC& in_xDim,
        const Type& in_xNonDim_MinMax,
        VEC& out_xNonDim,
        Type& out_xDim_Shift,
        Type& out_xDim_Scale);
    MAT buildDerivMat(
        const int& SplCff_N, 
        const int& Deriv_N) const;
    MAT buildObjMat() const;
    MAT buildPolyMat(
        const VEC& W,
        const int& PolyCoeffs_N) const; // = Polynomial order + 1
    MAT buildEquMat(
        const VEC& Wp_T_NonDim) const;
    VEC buildEquVec(
        const VEC& Waypoints_w) const;
    void checkQPRank(
        const MAT& ObjMat,
        const MAT& EquMat) const;
    bool solveQPwithGurobi(
        const GRBEnv& GurobiEnv,
        VEC& out_Solution,
        Type& out_Cost,
        const MAT& ObjMat,
        VEC ObjVec,
        const MAT& IneMat,
        const VEC& IneVec,
        const MAT& EquMat,
        const VEC& EquVec,
        const std::vector<std::string> DecVar_Names,
        const char& DecVar_Type,
        const Type& DecVar_LowerBound,
        const Type& DecVar_UpperBound,
        const int& ModelSense,
        const int& NumericFocus,
        const bool& DebugOn);
    bool computeTrajectory(
        const GRBEnv& GurobiEnv,
        VEC& out_QP_Solution_DecVar_X,
        VEC& out_QP_Solution_DecVar_Y,
        VEC& out_QP_Solution_DecVar_Z,
        Type& out_QP_Solution_Cost,
        const MAT& in_QP_ObjMat,
        const VEC& in_QP_EquVec_X,
        const VEC& in_QP_EquVec_Y,
        const VEC& in_QP_EquVec_Z,
        const std::vector<std::string>& in_QP_DecVar_Names_X,
        const std::vector<std::string>& in_QP_DecVar_Names_Y,
        const std::vector<std::string>& in_QP_DecVar_Names_Z,
        const VEC& Wp_T_NonDim);
    VEC computeCostGrad(
        const VEC& Spl_Duration_NonDim);
    void optimizeTemporalCoordinates();
    void sampleTrajectory(
        const VEC& in_QP_Solution_DecVar_X,
        const VEC& in_QP_Solution_DecVar_Y,
        const VEC& in_QP_Solution_DecVar_Z);
    VEC redimensionalizeTemporalCoordinates(
        const VEC& T_NonDim);
    VEC redimensionalizeSpatialCoordinates(
        const VEC& wNonDim,
        const int& DiffOrder,
        Type& wDim_Shift,
        Type& wDim_Scale);
    void plotTrajectory();
    void plotWaypoints(
        const VEC& Wp_X,
        const VEC& Wp_Y);
    void computeTempScaleEnsuringRequirements(
        const VEC& in_QP_Solution_DecVar_X,
        const VEC& in_QP_Solution_DecVar_Y,
        const VEC& in_QP_Solution_DecVar_Z
        );
    void computeRangesOfSpeedThrustAndRollPitchRate();

};



template <typename Type>
void 
ForgetfulGlobalTrajectory<Type>::computeRangesOfSpeedThrustAndRollPitchRate()
{
    m_Traj_MaxSpeed = 0.0;
    m_Traj_MaxThrust = 0.0;
    m_Traj_MaxRollPitchRate = 0.0;

    m_Traj_MinSpeed = 1e10;
    m_Traj_MinThrust = 1e10;
    m_Traj_MinRollPitchRate = 1e10;

    for (int T_i = 0; T_i < m_Traj_T_N; T_i++)
    {
        Eigen::Matrix<Type, 3, 1>  Velocity {
            m_Traj_VelX_Dim[T_i], 
            m_Traj_VelY_Dim[T_i], 
            m_Traj_VelZ_Dim[T_i]};
        Type Speed = Velocity.norm();

        Eigen::Matrix<Type, 3, 1> Acceleration {
            m_Traj_AccX_Dim[T_i], 
            m_Traj_AccY_Dim[T_i], 
            m_Traj_AccZ_Dim[T_i] + 9.81};
        Type Thrust = Acceleration.norm();

        Eigen::Matrix<Type, 3, 1> Jerk {
            m_Traj_JerX_Dim[T_i], 
            m_Traj_JerY_Dim[T_i], 
            m_Traj_JerZ_Dim[T_i]};
        
        Eigen::Matrix<Type, 3, 1> Jerk_ThrustNormalized = Jerk/Thrust;

        Type RollPitchRate = std::sqrt(
            std::pow(Jerk_ThrustNormalized.norm(), 2.0)
            - std::pow(Acceleration.normalized().dot(Jerk_ThrustNormalized), 2.0));


        m_Traj_MaxSpeed = std::max(m_Traj_MaxSpeed, Speed);
        m_Traj_MaxThrust = std::max(m_Traj_MaxThrust, Thrust);
        m_Traj_MaxRollPitchRate = std::max(m_Traj_MaxRollPitchRate, RollPitchRate);

        m_Traj_MinSpeed = std::min(m_Traj_MinSpeed, Speed);
        m_Traj_MinThrust = std::min(m_Traj_MinThrust, Thrust);
        m_Traj_MinRollPitchRate = std::min(m_Traj_MinRollPitchRate, RollPitchRate);
    }

    
    
    
    
    
    
}


template <typename Type>
void 
ForgetfulGlobalTrajectory<Type>::computeTempScaleEnsuringRequirements(
    const VEC& in_QP_Solution_DecVar_X,
    const VEC& in_QP_Solution_DecVar_Y,
    const VEC& in_QP_Solution_DecVar_Z
){
    computeRangesOfSpeedThrustAndRollPitchRate();

    Type MaxSpeed_Ratio;
    Type MaxThrust_Ratio;
    Type MaxRollPitchRate_Ratio;

    MaxSpeed_Ratio = m_Traj_MaxSpeed / m_Traj_MaxSpeed_Required;
    MaxThrust_Ratio = m_Traj_MaxThrust / m_Traj_MaxThrust_Required;
    MaxRollPitchRate_Ratio 
        = m_Traj_MaxRollPitchRate / m_Traj_MaxRollPitchRate_Required;

    //std::cout << "\tm_Traj_MaxSpeed: " << m_Traj_MaxSpeed << std::endl << std::endl;
    //std::cout << "\tm_Traj_MaxThrust: " << m_Traj_MaxThrust << std::endl << std::endl;
    //std::cout << "\tm_Traj_MaxRollPitchRate: " << m_Traj_MaxRollPitchRate << std::endl << std::endl;

    //std::cout << "\tm_Traj_MaxSpeed_Required: " << m_Traj_MaxSpeed_Required << std::endl << std::endl;
    //std::cout << "\tm_Traj_MaxThrust_Required: " << m_Traj_MaxThrust_Required << std::endl << std::endl;
    //std::cout << "\tm_Traj_MaxRollPitchRate_Required: " << m_Traj_MaxRollPitchRate_Required << std::endl << std::endl;

    //std::cout << "\tMaxSpeed_Ratio: " << MaxSpeed_Ratio << std::endl << std::endl;
    //std::cout << "\tMaxThrust_Ratio: " << MaxThrust_Ratio << std::endl << std::endl;
    //std::cout << "\tMaxRollPitchRate_Ratio: " << MaxRollPitchRate_Ratio << std::endl << std::endl;

    // Must be violated the first iteration since preliminary temporal coordinates were
    // computed with m_Traj_MaxSpeed_Required on straight lines from waypoint to waypoint.
    if (MaxSpeed_Ratio > 1)
    {
        m_Scale_T *= (MaxSpeed_Ratio + 1e-6);
        m_Shift_T *= (MaxSpeed_Ratio + 1e-6);
        m_Wp_T_Dim *= (MaxSpeed_Ratio + 1e-6);
        sampleTrajectory(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);
        computeTempScaleEnsuringRequirements(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);

        return;
    }

    if (MaxThrust_Ratio > 1)
    {
        m_Scale_T *= 1.01; // iteratively because of gravity term in thrust
        m_Shift_T *= 1.01;
        m_Wp_T_Dim *= 1.01;
        sampleTrajectory(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);
        computeTempScaleEnsuringRequirements(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);
        
        return;
    }

    if (MaxRollPitchRate_Ratio > 1)
    {
        m_Scale_T *= 1.01; // iteratively because of gravity term in thrust
        m_Shift_T *= 1.01;
        m_Wp_T_Dim *= 1.01;
        sampleTrajectory(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);
        computeTempScaleEnsuringRequirements(
            in_QP_Solution_DecVar_X,
            in_QP_Solution_DecVar_Y,
            in_QP_Solution_DecVar_Z);

        return;
    }
}


template <typename Type>
ForgetfulGlobalTrajectory<Type>::ForgetfulGlobalTrajectory(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& waypoints,
    const int& polynomial_order,
    const int& continuity_order,
    const Type& max_speed,
    const Type& max_thrust,
    const Type& max_roll_pitch_rate,
    const Type& sampling_time,
    const int& gurobi_numeric_focus,
    const Type& non_dim_range_temporal,
    const Type& non_dim_range_spatial,
    const bool& debug_enabled)
    :
    m_DebugEnabled {debug_enabled},
    
    m_Wp_W_N {static_cast<int>(waypoints.size())},
    m_Wp_T_N {m_Wp_W_N + 1},
    m_Spl_N {m_Wp_W_N},
    m_Spl_PolyCff_N {polynomial_order + 1},
    m_TrajPolyCff_N {m_Spl_N * m_Spl_PolyCff_N},
    
    m_ContDiff_N {continuity_order},
    m_Traj_MaxSpeed_Required {max_speed},
    m_Traj_MaxThrust_Required {max_thrust},
    m_Traj_MaxRollPitchRate_Required {max_roll_pitch_rate},
    m_Traj_SamplingTime_Dim {sampling_time},

    
    m_GurobiNumericFocus {gurobi_numeric_focus},
    m_NonDimMinMax_T {non_dim_range_temporal},
    m_NonDimMinMax_W {non_dim_range_spatial},
    m_MainOptLoopIter_MaxN {20},

    m_Wp_X_Dim {VEC::Zero(m_Wp_W_N)},
    m_Wp_Y_Dim {VEC::Zero(m_Wp_W_N)},
    m_Wp_Z_Dim {VEC::Zero(m_Wp_W_N)},
    m_Wp_T_Dim {VEC::Zero(m_Wp_T_N)},
    m_Shift_T {std::numeric_limits<Type>::quiet_NaN()},
    m_Shift_X {std::numeric_limits<Type>::quiet_NaN()},
    m_Shift_Y {std::numeric_limits<Type>::quiet_NaN()},
    m_Shift_Z {std::numeric_limits<Type>::quiet_NaN()},
    m_Scale_T {std::numeric_limits<Type>::quiet_NaN()},
    m_Scale_X {std::numeric_limits<Type>::quiet_NaN()},
    m_Scale_Y {std::numeric_limits<Type>::quiet_NaN()},
    m_Scale_Z {std::numeric_limits<Type>::quiet_NaN()},
    m_Wp_T_NonDim {VEC::Zero(m_Wp_W_N)},
    m_Wp_X_NonDim {VEC::Zero(m_Wp_W_N)},
    m_Wp_Y_NonDim {VEC::Zero(m_Wp_W_N)},
    m_Wp_Z_NonDim {VEC::Zero(m_Wp_T_N)},
    

    m_Traj_Duration_NonDim {2*m_NonDimMinMax_T},
    m_Traj_SamplingTime_NonDim {std::numeric_limits<Type>::quiet_NaN()},
    m_Traj_T_N {std::numeric_limits<int>::quiet_NaN()},


    m_Spl_T_Dim {},
    m_Spl_PosX_Dim {},
    m_Spl_PosY_Dim {},
    m_Spl_PosZ_Dim {},
    
    m_Traj_T_Dim {},
    m_Traj_PosX_Dim {},
    m_Traj_PosY_Dim {},
    m_Traj_PosZ_Dim {},
    m_Traj_VelX_Dim {},
    m_Traj_VelY_Dim {},
    m_Traj_VelZ_Dim {},
    m_Traj_AccX_Dim {},
    m_Traj_AccY_Dim {},
    m_Traj_AccZ_Dim {},
    m_Traj_JerX_Dim {},
    m_Traj_JerY_Dim {},
    m_Traj_JerZ_Dim {},


    m_Traj_MaxSpeed {0},
    m_Traj_MaxThrust {0},
    m_Traj_MaxRollPitchRate {0},
    m_Successful {false}
{
    DEBUG_LOG(__FUNCTION__ << "() entered.");

    
    // Set spatial coordinates from input
    for (int i = 0; i < m_Wp_W_N; i++) {
        m_Wp_X_Dim(i) = waypoints[i].cast<Type>().x();
        m_Wp_Y_Dim(i) = waypoints[i].cast<Type>().y();
        m_Wp_Z_Dim(i) = waypoints[i].cast<Type>().z();
    }
    DEBUG_LOG("m_Wp_X_Dim:" << m_Wp_X_Dim.transpose().format(MyFmt(2)));
    DEBUG_LOG("m_Wp_Y_Dim:" << m_Wp_Y_Dim.transpose().format(MyFmt(2)));
    DEBUG_LOG("m_Wp_Z_Dim:" << m_Wp_Z_Dim.transpose().format(MyFmt(2))); 
                                                                                    
    // Compute preliminary temporal coordinates
    m_Wp_T_Dim(0) = 0;
    for (int i = 0; i < m_Spl_N; i++) {
        const Eigen::Vector3d& wp_0 = waypoints[i];
        const Eigen::Vector3d& wp_1 = waypoints[(i + 1) % m_Spl_N];
        const Type spl_dist = (wp_1.cast<Type>() - wp_0.cast<Type>()).norm();
        const Type spl_dur = spl_dist / max_speed;                              
        
        m_Wp_T_Dim(i + 1) = m_Wp_T_Dim(i) + spl_dur;                                        
    }
    DEBUG_LOG("m_Wp_T_Dim:" << m_Wp_T_Dim.transpose().format(MyFmt(2)));

    // Dedimensionalize temporal and spatial coordinates
    dedimensionalizeCoordinates(m_Wp_T_Dim, m_NonDimMinMax_T, m_Wp_T_NonDim, m_Shift_T, m_Scale_T);                                                                      
    dedimensionalizeCoordinates(m_Wp_X_Dim, m_NonDimMinMax_W, m_Wp_X_NonDim, m_Shift_X, m_Scale_X);                                       
    dedimensionalizeCoordinates(m_Wp_Y_Dim, m_NonDimMinMax_W, m_Wp_Y_NonDim, m_Shift_Y, m_Scale_Y);                                       
    dedimensionalizeCoordinates(m_Wp_Z_Dim, m_NonDimMinMax_W, m_Wp_Z_NonDim, m_Shift_Z, m_Scale_Z);
    DEBUG_LOG("m_Shift_T:\t\t" << m_Shift_T);
    DEBUG_LOG("m_Scale_T:\t\t" << m_Scale_T);
    DEBUG_LOG("m_Wp_T_NonDim:" << m_Wp_T_NonDim.transpose().format(MyFmt(2)));
    DEBUG_LOG("m_Shift_X:\t\t" << m_Shift_X); 
    DEBUG_LOG("m_Scale_X:\t\t" << m_Scale_X); 
    DEBUG_LOG("m_Wp_X_NonDim:" << m_Wp_X_NonDim.transpose().format(MyFmt(2)));
    DEBUG_LOG("m_Shift_Y:\t\t" << m_Shift_Y); 
    DEBUG_LOG("m_Scale_Y:\t\t" << m_Scale_Y); 
    DEBUG_LOG("m_Wp_Y_NonDim:" << m_Wp_Y_NonDim.transpose().format(MyFmt(2)));
    DEBUG_LOG("m_Shift_Z:\t\t" << m_Shift_Z); 
    DEBUG_LOG("m_Scale_Z:\t\t" << m_Scale_Z); 
    DEBUG_LOG("m_Wp_Z_NonDim:" << m_Wp_Z_NonDim.transpose().format(MyFmt(2)));
    
    //plotWaypoints(m_Wp_X_NonDim, m_Wp_Y_NonDim);

    m_Traj_Duration_NonDim = m_Wp_T_NonDim(m_Wp_T_N - 1) - m_Wp_T_NonDim(0);
    m_Traj_SamplingTime_NonDim = m_Traj_SamplingTime_Dim / m_Scale_T;
    m_Traj_T_N = 1 + static_cast<int>(m_Traj_Duration_NonDim / m_Traj_SamplingTime_NonDim);
    DEBUG_LOG("m_Traj_Duration_NonDim:\t" << m_Traj_Duration_NonDim);
    DEBUG_LOG("m_Traj_SamplingTime_Dim:\t" << m_Traj_SamplingTime_Dim);
    DEBUG_LOG("m_Traj_SamplingTime_NonDim:\t" << m_Traj_SamplingTime_NonDim);
    DEBUG_LOG("m_Traj_T_N:\t\t" << m_Traj_T_N);



    const MAT QP_ObjMat = buildObjMat(); // m_TrajPolyCff_N >< m_TrajPolyCff_N
    const VEC QP_EquVec_X = buildEquVec(m_Wp_X_NonDim); // m_Spl_N * (m_ContDiff_N + 2) >< 1
    const VEC QP_EquVec_Y = buildEquVec(m_Wp_Y_NonDim); // m_Spl_N * (m_ContDiff_N + 2) >< 1
    const VEC QP_EquVec_Z = buildEquVec(m_Wp_Z_NonDim); // m_Spl_N * (m_ContDiff_N + 2) >< 1    
    //DEBUG_LOG("QP_ObjMat:\n" << QP_ObjMat.format(MyFmt(2)));
    //DEBUG_LOG("QP_EquVec_X:" << QP_EquVec_X.transpose().format(MyFmt(2)));
    //DEBUG_LOG("QP_EquVec_Y:" << QP_EquVec_Y.transpose().format(MyFmt(2)));
    //DEBUG_LOG("QP_EquVec_Z:" << QP_EquVec_Z.transpose().format(MyFmt(2)));


    // Set names of decision variables
    std::vector<std::string> QP_DecVar_Names_X, QP_DecVar_Names_Y, QP_DecVar_Names_Z;
    for (int i = 0; i < m_Spl_N; i++)
        for (int j = 0; j < m_Spl_PolyCff_N; j++) {
            QP_DecVar_Names_X.push_back("X-Spline_" + std::to_string(i) + ": c" + std::to_string(j));
            QP_DecVar_Names_Y.push_back("Y-Spline_" + std::to_string(i) + ": c" + std::to_string(j));
            QP_DecVar_Names_Z.push_back("Z-Spline_" + std::to_string(i) + ": c" + std::to_string(j));
        }
    //DEBUG_LOG("QP_DecVar_Names_X:\t" << QP_DecVar_Names_X);
    //DEBUG_LOG("QP_DecVar_Names_Y:\t" << QP_DecVar_Names_Y);
    //DEBUG_LOG("QP_DecVar_Names_Z:\t" << QP_DecVar_Names_Z);



    
    
    Type QP_Solution_NewCost;
    std::vector<Type> QP_Solution_Cost;
    VEC QP_Solution_DecVar_X;
    VEC QP_Solution_DecVar_Y;
    VEC QP_Solution_DecVar_Z;
    GRBEnv GurobiEnv = GRBEnv();
    //try {
    //    const_cast<GRBEnv&>(GurobiEnv) = GRBEnv();
    //}
    //catch(GRBException e) {
    //    ERROR_LOG("GUROBI Exception, error code: " << e.getErrorCode() << ", message: " << e.getMessage());
    //    return;
    //}

    // Compute trajectory with preliminary temporal coordinates
    m_Successful = computeTrajectory(
        GurobiEnv,
        QP_Solution_DecVar_X,
        QP_Solution_DecVar_Y,
        QP_Solution_DecVar_Z,
        QP_Solution_NewCost,
        QP_ObjMat,
        QP_EquVec_X,
        QP_EquVec_Y,
        QP_EquVec_Z,
        QP_DecVar_Names_X,
        QP_DecVar_Names_Y,
        QP_DecVar_Names_Z,
        m_Wp_T_NonDim);
    
    if (!m_Successful) return;

    QP_Solution_Cost.push_back(QP_Solution_NewCost);                                    //std::cout << "\tInitial Cost: " << QP_Solution_NewCost << std::endl << std::endl;

    //matplotlibcpp::figure_size(3840, 2160); // Ultra HD :D
    //matplotlibcpp::title("Optimization of spline durations in x/y-Plane");
    //matplotlibcpp::scatter(
    //    StdVector_From_EigenVector(m_Wp_X_Dim.template cast<double>()),
    //    StdVector_From_EigenVector(m_Wp_Y_Dim.template cast<double>()),
    //    800
    //    );

    //Type TextShift_X = (m_Wp_X_Dim.maxCoeff() - m_Wp_X_Dim.minCoeff())/100;
    //Type TextShift_Y = (m_Wp_Y_Dim.maxCoeff() - m_Wp_Y_Dim.minCoeff())/100;
    //for (int Wp_W_i = 0; Wp_W_i < m_Wp_W_N; Wp_W_i++)
    //{
    //    matplotlibcpp::text(m_Wp_X_Dim[Wp_W_i] - TextShift_X, m_Wp_Y_Dim[Wp_W_i] - TextShift_Y, 
    //        "Wp " + std::to_string(Wp_W_i));
    //}
    //sampleTrajectory(QP_Solution_DecVar_X, QP_Solution_DecVar_Y, QP_Solution_DecVar_Z);
    //matplotlibcpp::named_plot(
    //    std::string("Initial Trajectory") + ", Cost: " + std::to_string(QP_Solution_Cost.end()[-1]),
    //    StdVector_From_EigenVector(m_Traj_PosX_Dim.template cast<double>()), 
    //    StdVector_From_EigenVector(m_Traj_PosY_Dim.template cast<double>()));

                                                                                        //std::cout << "\t// for (int Iter_i = -1; Iter_i < "<<m_MainOptLoopIter_MaxN<<"; Iter_i++)" << std::endl << std::endl;
    for (int Iter_i = 0; Iter_i < m_MainOptLoopIter_MaxN; Iter_i++)
    {                                                                                   //std::cout << "\t\t// Iter_i: " << Iter_i << std::endl << std::endl;                                                                                   
        Type BreakCond_MaxCostDifference = 1e-20;
        
        // Spline durations
        const VEC Spl_Duration_NonDim = VEC::Zero(m_Spl_N);                         
            for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
                const_cast<VEC&>(Spl_Duration_NonDim)(Spl_i) 
                    = m_Wp_T_NonDim(Spl_i + 1) - m_Wp_T_NonDim(Spl_i);                  //std::cout << "\t\t// m_Wp_T_NonDim: " << m_Wp_T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;    
                                                                                        //std::cout << "\t\t// Spl_Duration_NonDim: " << Spl_Duration_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

        // Gradient computation
        VEC Gradient = VEC::Zero(m_Spl_N);                                              //std::cout << "\t\t// for (int Spl_i = 0; Spl_i < "<<m_Spl_N<<"; Spl_i++)" << std::endl << std::endl;
        for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
        {                                                                               //std::cout << "\t\t\t// Spl_i: " << Spl_i << std::endl << std::endl; 
            Type Gradient_StepSize = 0.01 / m_Scale_T;                                  //std::cout << "\t\t\t// Gradient_StepSize: " << Gradient_StepSize << std::endl << std::endl; 

            VEC Gradient_SplDurationIncrement_NonDim 
                = - Gradient_StepSize / (m_Spl_N - 1) * VEC::Ones(m_Spl_N);
            Gradient_SplDurationIncrement_NonDim(Spl_i) = Gradient_StepSize;            //std::cout << "\t\t\t// Gradient_SplDurationIncrement_NonDim: " << Gradient_SplDurationIncrement_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl; 

            
            VEC Gradient_SplDuration_NonDim = 
                Spl_Duration_NonDim + Gradient_SplDurationIncrement_NonDim;             //std::cout << "\t\t\t// Gradient_SplDuration_NonDim: " << Gradient_SplDuration_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl; 
            VEC Gradient_Wp_T_NonDim = m_Wp_T_NonDim;
            for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
                Gradient_Wp_T_NonDim(Spl_i + 1) 
                    = m_Wp_T_NonDim(0) 
                    + Gradient_SplDuration_NonDim.segment(0, Spl_i + 1).sum();          //std::cout << "\t\t\t// Gradient_Wp_T_NonDim: " << Gradient_Wp_T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl; 

            Type Gradient_Cost;
            VEC Gradient_X; VEC Gradient_Y; VEC Gradient_Z;
            m_Successful = computeTrajectory(
                GurobiEnv, 
                Gradient_X, Gradient_Y, Gradient_Z, Gradient_Cost,
                QP_ObjMat, QP_EquVec_X, QP_EquVec_Y, QP_EquVec_Z,
                QP_DecVar_Names_X, QP_DecVar_Names_Y, QP_DecVar_Names_Z,
                Gradient_Wp_T_NonDim); 
            if (!m_Successful) return;

            Gradient(Spl_i) = 
                (Gradient_Cost - QP_Solution_Cost.back()) / Gradient_StepSize;
        }                                                                               //std::cout << "\t\t// Gradient: " << Gradient.transpose().format(MyFmt(2)) << std::endl << std::endl; 

        // Search vector computation
        VEC SearchVec = - Gradient;                                                     //std::cout << "\t\t// SearchVec: " << SearchVec.transpose().format(MyFmt(2)) << std::endl << std::endl;
                                                                                        //std::cout << "\t\t// for (int Spl_i = 0; Spl_i < "<<m_Spl_N<<"; Spl_i++)" << std::endl << std::endl;
        for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++) 
        {                                                                               //std::cout << "\t\t\t// Spl_i: " << Spl_i << std::endl << std::endl; 
            const Type SearchVec_SplDuration_MaxUpdateRatio = 0.5;
                                                                                        //std::cout << "\t\t\t// SearchVec(Spl_i): " << SearchVec(Spl_i) << std::endl << std::endl;
                                                                                        //std::cout << "\t\t\t// SearchVec_SplDuration_MaxUpdateRatio * Spl_Duration_NonDim(Spl_i): " << SearchVec_SplDuration_MaxUpdateRatio * Spl_Duration_NonDim(Spl_i) << std::endl << std::endl; 
            if (
                fabs(SearchVec(Spl_i)) 
                > SearchVec_SplDuration_MaxUpdateRatio * Spl_Duration_NonDim(Spl_i))
            {
                // then scale vector down such that: =
                SearchVec /= SearchVec(Spl_i);
                SearchVec *= SearchVec_SplDuration_MaxUpdateRatio 
                                * Spl_Duration_NonDim(Spl_i);                           //std::cout << "\t\t\t// SearchVec: " << SearchVec.transpose().format(MyFmt(2)) << std::endl << std::endl;
            }
        }
        Type SearchVec_NormFactor                                                       
            = (Spl_Duration_NonDim + SearchVec).sum() 
                / m_Traj_Duration_NonDim;                                               //std::cout << "\t\t// SearchVec_NormFactor: " << SearchVec_NormFactor << std::endl << std::endl;
        SearchVec = (Spl_Duration_NonDim + SearchVec) / SearchVec_NormFactor            
                        - Spl_Duration_NonDim;                                          //std::cout << "\t\t// SearchVec: " << SearchVec.transpose().format(MyFmt(2)) << std::endl << std::endl;


        // Backtracking line search
        VEC BLS_SplDuration_NonDim;
        Type BLS_LearningRate = 1.0;                                                    //std::cout << "\t\t// BLS_LearningRate: " << BLS_LearningRate << std::endl << std::endl;
        const Type BLS_Alpha = 0.1;                                                     //std::cout << "\t\t// BLS_Alpha: " << BLS_Alpha << std::endl << std::endl;
        const Type BLS_Beta = 0.5;                                                      //std::cout << "\t\t// BLS_Beta: " << BLS_Beta << std::endl << std::endl;
                                                                                        //std::cout << "\t\t// while (true)" << std::endl << std::endl;
        //while (true)                                                                
        for (int _=0; _<20; _++)
        {
            VEC BLS_Step = BLS_LearningRate * SearchVec;                                //std::cout << "\t\t\t// BLS_Step: " << BLS_Step.transpose().format(MyFmt(2)) << std::endl << std::endl;

            BLS_SplDuration_NonDim = Spl_Duration_NonDim + BLS_Step;                    //std::cout << "\t\t\t// Spl_Duration_NonDim: " << Spl_Duration_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
                                                                                        //std::cout << "\t\t\t// BLS_SplDuration_NonDim: " << BLS_SplDuration_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

            VEC BLS_Wp_T_NonDim = m_Wp_T_NonDim;
                for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
                    BLS_Wp_T_NonDim(Spl_i + 1) = m_Wp_T_NonDim(0) 
                        + BLS_SplDuration_NonDim.segment(0, Spl_i + 1).sum();           //std::cout << "\t\t\t// BLS_Wp_T_NonDim: " << BLS_Wp_T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

            Type BLS_Cost;
            VEC BLS_X; VEC BLS_Y; VEC BLS_Z;
            m_Successful = computeTrajectory(
                GurobiEnv, 
                BLS_X, BLS_Y, BLS_Z, BLS_Cost,
                QP_ObjMat, QP_EquVec_X, QP_EquVec_Y, QP_EquVec_Z,
                QP_DecVar_Names_X, QP_DecVar_Names_Y, QP_DecVar_Names_Z,
                BLS_Wp_T_NonDim); 
                if (!m_Successful) return;

            BLS_LearningRate *= BLS_Beta;                                               //std::cout << "\t\t\t// BLS_LearningRate: " << BLS_LearningRate << std::endl << std::endl;
            Type BLS_BreakCond_MaxCost
                = QP_Solution_Cost.back() + BLS_Alpha * BLS_Step.dot(Gradient);         //std::cout << "\t\t\t// BLS_Cost: " << BLS_Cost << std::endl << std::endl;
                                                                                        //std::cout << "\t\t\t// BLS_BreakCond_MaxCost: " << BLS_BreakCond_MaxCost << std::endl << std::endl;
            if (BLS_Cost < BLS_BreakCond_MaxCost)
            {
                m_Wp_T_NonDim = BLS_Wp_T_NonDim;
                m_Wp_T_Dim = redimensionalizeTemporalCoordinates(m_Wp_T_NonDim);            
                break;
            }
        }


        Type QP_Solution_NewCost;
        m_Successful = computeTrajectory(
            GurobiEnv,
            QP_Solution_DecVar_X,
            QP_Solution_DecVar_Y,
            QP_Solution_DecVar_Z,
            QP_Solution_NewCost,
            QP_ObjMat,
            QP_EquVec_X,
            QP_EquVec_Y,
            QP_EquVec_Z,
            QP_DecVar_Names_X,
            QP_DecVar_Names_Y,
            QP_DecVar_Names_Z,
            m_Wp_T_NonDim); 
        
        if (!m_Successful) return;

        QP_Solution_Cost.push_back(QP_Solution_NewCost);
                                                                                        
        Type CostDifference
            = QP_Solution_Cost.end()[-2] 
            - QP_Solution_Cost.end()[-1];
                                                                                        //std::cout << "\t\t// CostDifference: " << CostDifference << std::endl << std::endl;
                                                                                        //std::cout << "\t\t// BreakCond_MaxCostDifference: " << BreakCond_MaxCostDifference << std::endl << std::endl;
        if (fabs(CostDifference) < BreakCond_MaxCostDifference) { break; }

        //sampleTrajectory(QP_Solution_DecVar_X, QP_Solution_DecVar_Y, QP_Solution_DecVar_Z);
        
        //std::vector<std::string> PlotFormats{}
        //matplotlibcpp::named_plot(
        //    "Trajectory #" + std::to_string(Iter_i) + ", Cost: " + std::to_string(QP_Solution_Cost.end()[-1]),
        //    StdVector_From_EigenVector(m_Traj_PosX_Dim.template cast<double>()), 
        //    StdVector_From_EigenVector(m_Traj_PosY_Dim.template cast<double>()));
                
    }
    //matplotlibcpp::legend();
    ////matplotlibcpp::save("/home/fm/Desktop/TEst/TempRange_"+std::to_string(m_NonDimMinMax_T)+"_SpatRange_"+std::to_string(m_NonDimMinMax_W)+".png");
    //matplotlibcpp::show();

    
    //std::cout << "\t// QP_Solution_Cost: ";
    //for (const Type& i: QP_Solution_Cost) std::cout << i << ' ';
    //std::cout << std::endl << std::endl;


    if (m_Successful)
    {
        sampleTrajectory(
            QP_Solution_DecVar_X, QP_Solution_DecVar_Y, QP_Solution_DecVar_Z);
        //std::cout << "m_Wp_T_NonDim: " << m_Wp_T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Wp_T_Dim: " << m_Wp_T_Dim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Traj_T_Dim: " << m_Traj_T_Dim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Scale_T: " << m_Scale_T << std::endl << std::endl;
        //std::cout << "m_Shift_T: " << m_Shift_T << std::endl << std::endl;
        computeTempScaleEnsuringRequirements(
            QP_Solution_DecVar_X, QP_Solution_DecVar_Y, QP_Solution_DecVar_Z);
        //std::cout << "m_Wp_T_NonDim: " << m_Wp_T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Wp_T_Dim: " << m_Wp_T_Dim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Traj_T_Dim: " << m_Traj_T_Dim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        //std::cout << "m_Scale_T: " << m_Scale_T << std::endl << std::endl;
        //std::cout << "m_Shift_T: " << m_Shift_T << std::endl << std::endl;
                                                                                                
        //plotTrajectory();
    }
    else
    {
        std::cout << "\n\n\nERROR: COULD NOT SOLVE QP!!!\n\n\n";
    }

}





template <typename Type> Eigen::IOFormat ForgetfulGlobalTrajectory<Type>::MyFmt (const int& num_tabs) {
    std::string row_prefix = "";
    for (int i = 0; i < num_tabs; i++) row_prefix += "\t";
    row_prefix += "[ ";

     // Precision, Flags, Coefficient separator , Row separator, Row prefix, Row suffix, Matrix prefix, Matrix suffix
    return Eigen::IOFormat(/*precision*/ 3, /*flags*/ 0, 
        /*coeffSeparator*/ " ",  /*rowSeparator*/ "\n", 
        /*rowPrefix*/ row_prefix.c_str(), /*rowSuffix*/ " ]", 
        /*matPrefix*/ "", /*matSuffix*/ ""
    );
}










template<typename Type>
void 
ForgetfulGlobalTrajectory<Type>::sampleTrajectory(
    const VEC& in_QP_Solution_DecVar_X,
    const VEC& in_QP_Solution_DecVar_Y,
    const VEC& in_QP_Solution_DecVar_Z
){
    //std::cout
    //    << std::endl
    //    << std::endl
    //    << "\tSAMPLE TRAJECTORY" << std::endl
    //    << std::endl;


    MAT Spline_i___X_PolyCoeff_NonDim_j = MAT::Zero(m_Spl_N, m_Spl_PolyCff_N);
    MAT Spline_i___Y_PolyCoeff_NonDim_j = MAT::Zero(m_Spl_N, m_Spl_PolyCff_N);
    MAT Spline_i___Z_PolyCoeff_NonDim_j = MAT::Zero(m_Spl_N, m_Spl_PolyCff_N);

    // Stack Spline Coefficients
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
        for (int PolyOrd_i = 0; PolyOrd_i < m_Spl_PolyCff_N; PolyOrd_i++)
        {
            // m_Splines_N >< m_Splines_Coeffs_N
            Spline_i___X_PolyCoeff_NonDim_j(Spl_i, PolyOrd_i) 
                = in_QP_Solution_DecVar_X(Spl_i * m_Spl_PolyCff_N + PolyOrd_i); 
            Spline_i___Y_PolyCoeff_NonDim_j(Spl_i, PolyOrd_i) 
                = in_QP_Solution_DecVar_Y(Spl_i * m_Spl_PolyCff_N + PolyOrd_i); 
            Spline_i___Z_PolyCoeff_NonDim_j(Spl_i, PolyOrd_i) 
                = in_QP_Solution_DecVar_Z(Spl_i * m_Spl_PolyCff_N + PolyOrd_i);
        }                                                                                                       
                                                                                                                // std::cout << "\t\t// Spline_i___X_PolyCoeff_NonDim_j:\n" << Spline_i___X_PolyCoeff_NonDim_j.format(MyFmt(3)) << std::endl << std::endl;
                                                                                                                // std::cout << "\t\t// Spline_i___Y_PolyCoeff_NonDim_j:\n" << Spline_i___Y_PolyCoeff_NonDim_j.format(MyFmt(3)) << std::endl << std::endl;
                                                                                                                // std::cout << "\t\t// Spline_i___Z_PolyCoeff_NonDim_j:\n" << Spline_i___Z_PolyCoeff_NonDim_j.format(MyFmt(3)) << std::endl << std::endl;

    
    m_Traj_SamplingTime_NonDim = m_Traj_SamplingTime_Dim / m_Scale_T;                                           // std::cout << "\t// m_Traj_SamplingTime_Dim: " << m_Traj_SamplingTime_Dim << std::endl << std::endl; 
    m_Traj_T_N 
        = 1 + static_cast<int>(m_Traj_Duration_NonDim / m_Traj_SamplingTime_NonDim);                            // std::cout << "\t\t\t// m_Traj_T_N: " << m_Traj_T_N << std::endl << std::endl;


    m_Traj_T_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_PosX_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_PosY_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_PosZ_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_VelX_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_VelY_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_VelZ_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_AccX_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_AccY_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_AccZ_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_JerX_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_JerY_Dim = VEC::Zero(m_Traj_T_N);
    m_Traj_JerZ_Dim = VEC::Zero(m_Traj_T_N);
    
    m_Spl_T_Dim.resize(m_Spl_N);
    m_Spl_PosX_Dim.resize(m_Spl_N);
    m_Spl_PosY_Dim.resize(m_Spl_N);
    m_Spl_PosZ_Dim.resize(m_Spl_N);


    int Traj_T_N_Actual = 0;                                                                             // std::cout << "\t\t// for (int Spl_i = 0; Spl_i < " << m_Spl_N << "; Spl_i++) " << std::endl << std::endl;
    //Type LastSpl_T1_NonDim = 0 - m_Traj_SamplingTime_NonDim;
    Type LastSpl_T1_NonDim = m_Wp_T_NonDim(0) - m_Traj_SamplingTime_NonDim;
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
                                                                                                                // std::cout << "\t\t\t// Spl_i: " << Spl_i << std::endl << std::endl;
        const Type T0_NonDim = LastSpl_T1_NonDim + m_Traj_SamplingTime_NonDim;                                  // std::cout << "\t\t\t// T0_NonDim: " << T0_NonDim << std::endl << std::endl;
        
        std::vector<Type> T_NonDim_AsStdVector;
        
        for (
            Type Ti_NonDim = T0_NonDim; 
            Ti_NonDim < m_Wp_T_NonDim[Spl_i + 1]; 
            Ti_NonDim += m_Traj_SamplingTime_NonDim
        ){ T_NonDim_AsStdVector.push_back(Ti_NonDim); }
        
        const int T_N_NonDim = T_NonDim_AsStdVector.size();                                                     // std::cout << "\t\t\t// T_N_NonDim: " << T_N_NonDim << std::endl << std::endl;
        VEC T_NonDim = VEC::Zero(T_N_NonDim);
        for (int T_i = 0; T_i < T_N_NonDim; T_i++)
            T_NonDim(T_i) = T_NonDim_AsStdVector[T_i];
        
        LastSpl_T1_NonDim = T_NonDim_AsStdVector.back();
        
        //const Type T1_NonDim = m_Wp_T_NonDim[Spl_i + 1] - m_Traj_SamplingTime_NonDim;                           std::cout << "\t\t\t// T1_NonDim: " << T1_NonDim << std::endl << std::endl;

        //const Type Duration_NonDim = T1_NonDim - T0_NonDim;                                                     std::cout << "\t\t\t// Duration_NonDim: " << Duration_NonDim << std::endl << std::endl;
        

        //VEC T_NonDim = VEC::LinSpaced(T_N_NonDim, T0_NonDim, T1_NonDim);                                        std::cout << "\t\t\t// T_NonDim:" << T_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        MAT T_Poly_NonDim = buildPolyMat(T_NonDim, m_Spl_PolyCff_N);                                            // std::cout << "\t\t\t// T_Poly_NonDim:\n" << T_Poly_NonDim.format(MyFmt(4)) << std::endl << std::endl;

        VEC PosX_NonDim = T_Poly_NonDim * Spline_i___X_PolyCoeff_NonDim_j.row(Spl_i).transpose();               // std::cout << "\t\t\t// PosX_NonDim:" << PosX_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC PosY_NonDim = T_Poly_NonDim * Spline_i___Y_PolyCoeff_NonDim_j.row(Spl_i).transpose();               // std::cout << "\t\t\t// PosY_NonDim:" << PosY_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC PosZ_NonDim = T_Poly_NonDim * Spline_i___Z_PolyCoeff_NonDim_j.row(Spl_i).transpose();               // std::cout << "\t\t\t// PosZ_NonDim:" << PosZ_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

        VEC VelX_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 1) 
            * Spline_i___X_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// VelX_NonDim:" << VelX_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC VelY_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 1) 
            * Spline_i___Y_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// VelY_NonDim:" << VelY_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC VelZ_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 1) 
            * Spline_i___Z_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// VelZ_NonDim:" << VelZ_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

        VEC AccX_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 2) 
            * Spline_i___X_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// AccX_NonDim:" << AccX_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC AccY_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 2) 
            * Spline_i___Y_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// AccY_NonDim:" << AccY_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC AccZ_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 2) 
            * Spline_i___Z_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// AccZ_NonDim:" << AccZ_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;

        VEC JerX_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 3) 
            * Spline_i___X_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// AccX_NonDim:" << AccX_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC JerY_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 3) 
            * Spline_i___Y_PolyCoeff_NonDim_j.row(Spl_i).transpose();                                           // std::cout << "\t\t\t// AccY_NonDim:" << AccY_NonDim.transpose().format(MyFmt(2)) << std::endl << std::endl;
        VEC JerZ_NonDim = T_Poly_NonDim * buildDerivMat(m_Spl_PolyCff_N, 3) 
            * Spline_i___Z_PolyCoeff_NonDim_j.row(Spl_i).transpose();

        m_Spl_T_Dim[Spl_i] = redimensionalizeTemporalCoordinates(T_NonDim);                                     // std::cout << "\t\t\t// m_Spl_T_Dim[" <<Spl_i << "]:" << m_Spl_T_Dim[Spl_i].transpose().format(MyFmt(2)) << std::endl << std::endl;
        
        m_Spl_PosX_Dim[Spl_i] = redimensionalizeSpatialCoordinates(PosX_NonDim, 0, m_Shift_X, m_Scale_X);       // std::cout << "\t\t\t// m_Spl_PosX_Dim[" <<Spl_i << "]:" << m_Spl_PosX_Dim[Spl_i].transpose().format(MyFmt(2)) << std::endl << std::endl;
        m_Spl_PosY_Dim[Spl_i] = redimensionalizeSpatialCoordinates(PosY_NonDim, 0, m_Shift_Y, m_Scale_Y);       // std::cout << "\t\t\t// m_Spl_PosY_Dim[" <<Spl_i << ":" << m_Spl_PosY_Dim[Spl_i].transpose().format(MyFmt(2)) << std::endl << std::endl;
        m_Spl_PosZ_Dim[Spl_i] = redimensionalizeSpatialCoordinates(PosZ_NonDim, 0, m_Shift_Z, m_Scale_Z);       // std::cout << "\t\t\t// m_Spl_PosZ_Dim[" <<Spl_i << "]:" << m_Spl_PosZ_Dim[Spl_i].transpose().format(MyFmt(2)) << std::endl << std::endl;

        m_Traj_T_Dim.segment(Traj_T_N_Actual, T_N_NonDim) = m_Spl_T_Dim[Spl_i];                       // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_PosX_Dim.segment(Traj_T_N_Actual, T_N_NonDim) = m_Spl_PosX_Dim[Spl_i];                    // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_PosY_Dim.segment(Traj_T_N_Actual, T_N_NonDim) = m_Spl_PosY_Dim[Spl_i];                    // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_PosZ_Dim.segment(Traj_T_N_Actual, T_N_NonDim) = m_Spl_PosZ_Dim[Spl_i];                    // std::cout << "\t\t\t// ..." << std::endl;
        
        m_Traj_VelX_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(VelX_NonDim, 1, m_Shift_X, m_Scale_X);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_VelY_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(VelY_NonDim, 1, m_Shift_Y, m_Scale_Y);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_VelZ_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(VelZ_NonDim, 1, m_Shift_Z, m_Scale_Z);                         // std::cout << "\t\t\t// ..." << std::endl;
        
        m_Traj_AccX_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(AccX_NonDim, 2, m_Shift_X, m_Scale_X);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_AccY_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(AccY_NonDim, 2, m_Shift_Y, m_Scale_Y);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_AccZ_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(AccZ_NonDim, 2, m_Shift_Z, m_Scale_Z);                        // std::cout << "\t\t\t// ..." << std::endl;

        m_Traj_JerX_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(JerX_NonDim, 3, m_Shift_X, m_Scale_X);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_JerY_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(JerY_NonDim, 3, m_Shift_Y, m_Scale_Y);                         // std::cout << "\t\t\t// ..." << std::endl;
        m_Traj_JerZ_Dim.segment(Traj_T_N_Actual, T_N_NonDim) 
            = redimensionalizeSpatialCoordinates(JerZ_NonDim, 3, m_Shift_Z, m_Scale_Z);

        Traj_T_N_Actual += T_N_NonDim;                                                                   //std::cout << "\t\t\t// Traj_T_N_Actual: " << Traj_T_N_Actual << std::endl << std::endl;
    }

    if (Traj_T_N_Actual != m_Traj_T_N)
    {
        std::cout
            << std::endl
            << "\t\tWARNING:    Actual number of sample points :  " << Traj_T_N_Actual << " != " << m_Traj_T_N << "  : expected number sample points" << std::endl;
    }


}



template<typename Type>
void 
ForgetfulGlobalTrajectory<Type>::plotTrajectory()
{
    matplotlibcpp::figure_size(3840, 2160); // Ultra HD :D
    matplotlibcpp::title("Trajectory-building splines and waypoints in x/y-Plane");


    const Type& MaxT_Dim = m_Traj_T_Dim(m_Traj_T_Dim.size() - 1);
    
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        const VEC& PosX_Dim = m_Spl_PosX_Dim[Spl_i];
        const VEC& PosY_Dim = m_Spl_PosY_Dim[Spl_i];
        const VEC& T_Dim = m_Spl_T_Dim[Spl_i];
        for (int T_i = 0; T_i < m_Spl_PosX_Dim[Spl_i].size(); T_i++)
        {
            const std::vector<Type> X {PosX_Dim(T_i)};
            const std::vector<Type> Y {PosY_Dim(T_i)};
            const int marker_size = 50 + static_cast<int>(350 * T_Dim(T_i) / MaxT_Dim);
            const std::vector<int> color {marker_size};
            matplotlibcpp::scatter_colored(
                X,
                Y,
                color,
                marker_size);
        }
    }

    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        //std::vector<std::string> PlotFormats{}
        matplotlibcpp::named_plot(
            "Spline #" + std::to_string(Spl_i),
            StdVector_From_EigenVector(m_Spl_PosX_Dim[Spl_i].template cast<double>()), 
            StdVector_From_EigenVector(m_Spl_PosY_Dim[Spl_i].template cast<double>()));
    }

    matplotlibcpp::scatter(
        StdVector_From_EigenVector(m_Wp_X_Dim.template cast<double>()),
        StdVector_From_EigenVector(m_Wp_Y_Dim.template cast<double>()),
        800
        );

    Type TextShift_X = (m_Wp_X_Dim.maxCoeff() - m_Wp_X_Dim.minCoeff())/100;
    Type TextShift_Y = (m_Wp_Y_Dim.maxCoeff() - m_Wp_Y_Dim.minCoeff())/100;
    for (int Wp_W_i = 0; Wp_W_i < m_Wp_W_N; Wp_W_i++)
    {
        matplotlibcpp::text(m_Wp_X_Dim[Wp_W_i] - TextShift_X, m_Wp_Y_Dim[Wp_W_i] - TextShift_Y, 
            ("Wp " + std::to_string(Wp_W_i)) + ": T = " + std::to_string(m_Wp_T_Dim[Wp_W_i])
            );
    }
    
    matplotlibcpp::legend();
    //matplotlibcpp::save("/home/fm/Desktop/TEst/TempRange_"+std::to_string(m_NonDimMinMax_T)+"_SpatRange_"+std::to_string(m_NonDimMinMax_W)+".png");
    matplotlibcpp::show();



    //matplotlibcpp::figure_size(3840, 2160); // Ultra HD :D
    //matplotlibcpp::title("Trajectory and waypoints in 3D");
    //matplotlibcpp::scatter(
    //    StdVector_From_EigenVector(m_Traj_PosX_Dim.template cast<double>()),
    //    StdVector_From_EigenVector(m_Traj_PosY_Dim.template cast<double>()),
    //    StdVector_From_EigenVector(m_Traj_PosZ_Dim.template cast<double>()),
    //    400
    //    );
    //matplotlibcpp::scatter(
    //    StdVector_From_EigenVector(m_Wp_X_Dim.template cast<double>()),
    //    StdVector_From_EigenVector(m_Wp_Y_Dim.template cast<double>()),
    //    StdVector_From_EigenVector(m_Wp_Z_Dim.template cast<double>()),
    //    800
    //    );
    //matplotlibcpp::show();

}




template<typename Type>
void 
ForgetfulGlobalTrajectory<Type>::plotWaypoints(
    const VEC& Wp_X,
    const VEC& Wp_Y
){
    matplotlibcpp::figure_size(3840, 2160); // Ultra HD :D
    matplotlibcpp::title("Waypoints in x/y-Plane");

    matplotlibcpp::scatter(
        StdVector_From_EigenVector(Wp_X.template cast<double>()),
        StdVector_From_EigenVector(Wp_Y.template cast<double>()),
        100
        );

    Type TextShift_X = (Wp_X.maxCoeff() - Wp_X.minCoeff())/150;
    Type TextShift_Y = (Wp_Y.maxCoeff() - Wp_Y.minCoeff())/150;
    for (int Wp_W_i = 0; Wp_W_i < m_Wp_W_N; Wp_W_i++)
    {
        matplotlibcpp::text(Wp_X[Wp_W_i] + TextShift_X, Wp_Y[Wp_W_i] + TextShift_Y, std::to_string(Wp_W_i));
    }
    
    //matplotlibcpp::legend();
    //matplotlibcpp::save("./basic.png");
    matplotlibcpp::show();
}







template<typename Type>
void 
ForgetfulGlobalTrajectory<Type>::checkQPRank(
    const MAT& ObjMat,
    const MAT& EquMat
) const {
    const int& EquConstr_N = EquMat.rows();
    const int& DecVar_N = EquMat.cols();

    Eigen::ColPivHouseholderQR<MAT> EquMat_Decomp(EquMat);
    const int& EquMat_Rank = EquMat_Decomp.rank();

    MAT ObjEquMat = MAT::Zero(DecVar_N + EquConstr_N, DecVar_N);
    ObjEquMat.block(0, 0, DecVar_N, DecVar_N) = ObjMat;
    ObjEquMat.block(DecVar_N, 0, EquConstr_N, DecVar_N) = EquMat;
    Eigen::ColPivHouseholderQR<MAT> ObjEquMat_Decomp(ObjEquMat);
    const int& ObjEquMat_Rank = ObjEquMat_Decomp.rank();

    bool QPEquDef = EquConstr_N != EquMat_Rank;
    bool QPDef =  DecVar_N != ObjEquMat_Rank;

    if (m_DebugEnabled)
    {
        std::string RankDeficStatus = "";
        if ((!QPDef) && (!QPEquDef))        RankDeficStatus = "\t\t\tQuadratic Program Well Conditioned!\n";
        else
        {
            if(QPDef)                       RankDeficStatus += "\t\t\tWARNING:   Rank( [ObjMat]\n\t\t\t                 [EquMat] )   <   # Decision Variables\n";
            if(QPEquDef)                    RankDeficStatus += "\t\t\tWARNING:   Rank( EquMat )     <   # Equality Constraints\n";
        }
        std::cout 
            << "\t\t CHECK QP Rank" << std::endl 
            << std::endl
            << "\t\t\t# Decision Variables:    " << DecVar_N << std::endl
            << "\t\t\t# Equality Constraints:  " << EquConstr_N << std::endl
            << "\t\t\tRank( EquMat ):          " << EquMat_Rank << std::endl
            << "\t\t\tRank( [ObjMat]" << std::endl
            << "\t\t\t      [EquMat] ):        " << ObjEquMat_Rank << std::endl
            << std::endl
            << RankDeficStatus
            << std::endl 
            << std::endl;
    }
    else
    {
        if(QPDef)    std::cout << "\t\t\tWARNING:   Rank( [ObjMat]\n\t\t\t                 [EquMat] )   <   # Decision Variables\n";
        if(QPEquDef) std::cout << "\t\t\tWARNING:   Rank( EquMat )     <   # Equality Constraints\n";
    }
};











template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::VEC 
ForgetfulGlobalTrajectory<Type>::computeCostGrad(
    const VEC& in_Spl_Duration
){
    VEC Gradient_Cost = VEC::Zero(m_Spl_N);

    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        Type Grad_StepSize = 0.01;

        VEC _DeltaVec = - Grad_StepSize / (m_Spl_N - 1) * VEC::Ones(m_Spl_N);
        _DeltaVec(Spl_i) = Grad_StepSize;

        
        VEC Spl_Duration = in_Spl_Duration + _DeltaVec;
        for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
        {
            m_Wp_T_NonDim(Spl_i + 1) = m_Wp_T_NonDim(0) + Spl_Duration.segment(0, Spl_i + 1).sum();
        }



    }
}


template<typename Type>
void
ForgetfulGlobalTrajectory<Type>::optimizeTemporalCoordinates()
{
    VEC Spl_Duration_NonDim = VEC::Zero(m_Spl_N);
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        Spl_Duration_NonDim(Spl_i) = m_Wp_T_NonDim(Spl_i + 1) - m_Wp_T_NonDim(Spl_i);
    }


}



template<typename Type>
bool 
ForgetfulGlobalTrajectory<Type>::computeTrajectory(
    const GRBEnv& GurobiEnv,
    VEC& out_QP_Solution_DecVar_X,
    VEC& out_QP_Solution_DecVar_Y,
    VEC& out_QP_Solution_DecVar_Z,
    Type& out_QP_Solution_Cost,
    const MAT& in_QP_ObjMat,
    const VEC& in_QP_EquVec_X,
    const VEC& in_QP_EquVec_Y,
    const VEC& in_QP_EquVec_Z,
    const std::vector<std::string>& in_QP_DecVar_Names_X,
    const std::vector<std::string>& in_QP_DecVar_Names_Y,
    const std::vector<std::string>& in_QP_DecVar_Names_Z,
    const VEC& Wp_T_NonDim
)
{
    // m_Spl_N * (m_ContDiff_N + 2)   ><   m_TrajPolyCff_N
    MAT QP_EquMat = buildEquMat(Wp_T_NonDim);                                       // std::cout << "\t\t// QP_EquMat:\n" << QP_EquMat.format(MyFmt(3)) << std::endl << std::endl;
    
    checkQPRank (in_QP_ObjMat, QP_EquMat);                                          


    // Compute Trajectory Coefficients
        Type QP_Solution_Cost_X = std::numeric_limits<Type>::quiet_NaN();
        Type QP_Solution_Cost_Y = std::numeric_limits<Type>::quiet_NaN();
        Type QP_Solution_Cost_Z = std::numeric_limits<Type>::quiet_NaN();
    bool Success_X = solveQPwithGurobi(
        GurobiEnv,
        out_QP_Solution_DecVar_X, QP_Solution_Cost_X,
        in_QP_ObjMat, VEC(), MAT(), VEC(), QP_EquMat, in_QP_EquVec_X,
        in_QP_DecVar_Names_X, GRB_CONTINUOUS, -GRB_INFINITY, GRB_INFINITY,
        1, m_GurobiNumericFocus,
        m_DebugEnabled);

    bool Success_Y = solveQPwithGurobi(
        GurobiEnv,
        out_QP_Solution_DecVar_Y, QP_Solution_Cost_Y,
        in_QP_ObjMat, VEC(), MAT(), VEC(), QP_EquMat, in_QP_EquVec_Y,
        in_QP_DecVar_Names_Y, GRB_CONTINUOUS, -GRB_INFINITY, GRB_INFINITY,
        1, m_GurobiNumericFocus,
        m_DebugEnabled);

    bool Success_Z = solveQPwithGurobi(
        GurobiEnv,
        out_QP_Solution_DecVar_Z, QP_Solution_Cost_Z,
        in_QP_ObjMat, VEC(), MAT(), VEC(), QP_EquMat, in_QP_EquVec_Z,
        in_QP_DecVar_Names_Z, GRB_CONTINUOUS, -GRB_INFINITY, GRB_INFINITY,
        1, m_GurobiNumericFocus,
        m_DebugEnabled);
                                                                                    // std::cout << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_Cost_X: " << QP_Solution_Cost_X << std::endl << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_DecVar_X:" << out_QP_Solution_DecVar_X.transpose().format(MyFmt(2)) << std::endl << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_Cost_Y: " << QP_Solution_Cost_Y << std::endl << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_DecVar_Y:" << out_QP_Solution_DecVar_Y.transpose().format(MyFmt(2)) << std::endl << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_Cost_Z: " << QP_Solution_Cost_Z << std::endl << std::endl;
                                                                                    // std::cout << "\t\t// QP_Solution_DecVar_Z:" << out_QP_Solution_DecVar_Z.transpose().format(MyFmt(2)) << std::endl << std::endl;
    out_QP_Solution_Cost 
        = QP_Solution_Cost_X + QP_Solution_Cost_Y + QP_Solution_Cost_Z;

    return (Success_X && Success_Y && Success_Z);
}







template<typename Type>
bool 
ForgetfulGlobalTrajectory<Type>::solveQPwithGurobi(
    const GRBEnv& GurobiEnv,
    VEC& out_Solution,
    Type& out_Cost,
    const MAT& ObjMat,
    VEC ObjVec,
    const MAT& IneMat,
    const VEC& IneVec,
    const MAT& EquMat,
    const VEC& EquVec,
    const std::vector<std::string> DecVar_Names,
    const char& DecVar_Type,
    const Type& DecVar_LowerBound,
    const Type& DecVar_UpperBound,
    const int& ModelSense,
    const int& NumericFocus,
    const bool& DebugOn
)
{
    



    // 1) Process input
    if (ObjVec == Eigen::Matrix<Type, Eigen::Dynamic, 1>{})
    {
        ObjVec = Eigen::Matrix<Type, Eigen::Dynamic, 1>::Zero(ObjMat.rows());
    }
    
    const int DecVar_N = ObjMat.rows();
    const int IneCon_N = IneMat.rows();
    const int EquCon_N = EquMat.rows();

    if (m_DebugEnabled)
    {
        std::string dec_var_type;
        if (DecVar_Type == GRB_CONTINUOUS) {dec_var_type = " Continuos";}
        if (DecVar_Type == GRB_SEMICONT) {dec_var_type = " Semi-continuos";}
        if (DecVar_Type == GRB_BINARY) {dec_var_type = " Binary";}
        if (DecVar_Type == GRB_INTEGER) {dec_var_type = " Integer";}
        if (DecVar_Type == GRB_SEMIINT) {dec_var_type = " Semi-integer";}

        std::cout
            << std::endl 
            << std::endl 
            << std::endl 
            << std::endl
            << " .-------------------------------------------------------------------." << std::endl
            << " | SOLVE QP WITH GUROBI |                                            |" << std::endl
            << " |----------------------'                                            |" << std::endl
            << " |                                                                   |" << std::endl
            << " | min/max     DecVec^T * ObjMat * DecVec + ObjVec^T * DecVec        |" << std::endl 
            << " | s.t.        IneMat * DecVec <= IneVec                             |" << std::endl 
            << " |             EquMat * DecVec  = EquVec                             |" << std::endl 
            << " |             LowerBound <= DecVec[i] <= Upper Bound                |" << std::endl 
            << " |             DecVec[i] (semi-)continuous, binary or (semi-)integer |" << std::endl
            << " '-------------------------------------------------------------------'" << std::endl
            << std::endl 
            << std::endl
            << "ObjMat" << std::endl
            << std::endl
            << ObjMat.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << "ObjVec" << std::endl
            << std::endl
            << ObjVec.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << std::endl
            << std::endl
            << "IneMat" << std::endl
            << std::endl
            << IneMat.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << "IneVec" << std::endl
            << std::endl
            << IneVec.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << std::endl
            << std::endl
            << "EquMat" << std::endl
            << std::endl
            << EquMat.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << "EquVec" << std::endl
            << std::endl
            << EquVec.format(MyFmt(2)) << std::endl
            << std::endl
            << std::endl
            << std::endl
            << std::endl
            << "-> #" << DecVar_N << dec_var_type << " Decision Variables in [ " << DecVar_LowerBound << ", " << DecVar_UpperBound << " ]" << std::endl
            << std::endl
            << std::endl
            << "-> #" << IneCon_N << " Inequality Constraints" << std::endl
            << std::endl
            << std::endl
            << "-> #" << EquCon_N << " Equality Constraints" << std::endl
            << std::endl
            << std::endl
            << std::endl
            << std::endl;
    }


    // 2) Check input
    if(ObjMat.rows() != ObjMat.cols())
    {
        std::cout 
            << "ERROR: Objective matrix is not a square matrix!" << std::endl;
        return false;
    }
    if(ObjVec.rows() != DecVar_N)
    {
        std::cout 
            << "ERROR: #Rows of objective vector is not equal to #rows/#columns of objective matrix!" << std::endl;
        return false;
    }
    if (IneCon_N != 0)
    {
        if(IneMat.cols() != DecVar_N)
        {
            std::cout 
                << "ERROR: #Columns of inequality matrix is not equal to #rows/#columns of objective matrix!" << std::endl;
            return false;
        }
        if(IneVec.rows() != IneCon_N)
        {
            std::cout 
                << "ERROR: #Rows of inequality matrix is not equal to #rows of inequality vector!" << std::endl;
            return false;
        }
    }
    if (EquCon_N != 0)
    {
        if(EquMat.cols() != DecVar_N)
        {
            std::cout 
                << "ERROR: #Columns of equality matrix is not equal to #rows/#columns of objective matrix!" << std::endl;
            return false;
        }
        if(EquVec.rows() != EquCon_N)
        {
            std::cout 
                << "ERROR: #Rows of equality matrix is not equal to #rows of equality vector!" << std::endl;
            return false;
        }
    }
    

    


    // 3) Create environment and model
    try {
        //GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(GurobiEnv);
        model.set(GRB_IntParam_OutputFlag, 0);
        model.set(GRB_IntParam_NumericFocus, NumericFocus);
        model.set(GRB_IntAttr_ModelSense, ModelSense);
        


        // 4) Create vector of decision variables
        //GRBVar* DecVarVec = model.addVars(DecVar_N, VarType);
        std::vector<GRBVar> DecVars;
        for (int DecVar_i = 0; DecVar_i < DecVar_N; DecVar_i++)
        {
            const Type ObjectiveCoefficient = 0.0;
            
            DecVars.push_back(
                model.addVar(
                    DecVar_LowerBound, DecVar_UpperBound,
                    ObjectiveCoefficient,
                    DecVar_Type,
                    DecVar_Names[DecVar_i]
                    ));
        }


        // 5) Set objective (cost function) of model
        GRBQuadExpr ObjFct = 0;
        for (int DecVar_i = 0; DecVar_i < DecVar_N; DecVar_i++)
        {
            for (int DecVar_j = 0; DecVar_j < DecVar_N; DecVar_j++)
            {
                GRBQuadExpr _ = DecVars[DecVar_i] * ObjMat(DecVar_i, DecVar_j) * DecVars[DecVar_j];
                //std::cout << _;
                ObjFct += _;
                        //+ ObjVec(DecVar_i) * (*(DecVarVec + DecVar_i));
            }
        }
        model.setObjective(ObjFct);


        // 6) Add inequality constraints
        if (IneCon_N != 0)
        {
            GRBLinExpr IneR;
            GRBLinExpr IneL;
            for (int IneCon_i = 0; IneCon_i < IneCon_N; IneCon_i++)
            {
                IneR = IneVec[IneCon_i];

                IneL = 0;
                for (int DecVar_i = 0; DecVar_i < DecVar_N; DecVar_i++)
                {
                    IneL += IneMat(IneCon_i, DecVar_i) * DecVars[DecVar_i];
                }

                std::string ConstrName = "IneConstr" + std::to_string(IneCon_i);
                model.addConstr(IneL, GRB_LESS_EQUAL, IneR, ConstrName);
            }
        }



        // 7) Add equality constraints
        if (EquCon_N != 0)
        {
            GRBLinExpr EquR;
            GRBLinExpr EquL;
            for (int EquCon_i = 0; EquCon_i < EquCon_N; EquCon_i++)
            {
                EquR = EquVec[EquCon_i];

                EquL = 0;
                for (int DecVar_i = 0; DecVar_i < DecVar_N; DecVar_i++)
                {
                    EquL += EquMat(EquCon_i, DecVar_i) * DecVars[DecVar_i];
                }

                std::string ConstrName = "EquConstr" + std::to_string(EquCon_i);
                model.addConstr(EquL, GRB_EQUAL, EquR, ConstrName);
            }
        }
        


        // 8) Run optimization of the model
        std::cout << "\033[41m"; model.optimize(); std::cout << "\033[0m";


        // Output 
        out_Cost = model.get(GRB_DoubleAttr_ObjVal);

        if (m_DebugEnabled)
        {
            std::cout
                << "Cost: " << out_Cost << std::endl
                << std::endl
                << std::endl
                << "Solution:" << std::endl
                << std::endl;
        }
        
        // 9) Set return values
        out_Solution = VEC::Zero(DecVar_N);
        for (int DecVar_i = 0; DecVar_i < DecVar_N; DecVar_i++)
        {
            if (m_DebugEnabled)
            {
                std::cout 
                    << "  " << DecVars[DecVar_i].get(GRB_StringAttr_VarName) << ":    " << DecVars[DecVar_i].get(GRB_DoubleAttr_X) << std::endl;
            }
            
            out_Solution[DecVar_i] = DecVars[DecVar_i].get(GRB_DoubleAttr_X);
        }

        
    } 
    catch(GRBException e) 
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
        return false;
    } 
    catch(...) 
    {
        std::cout << "Exception during optimization" << std::endl;
        return false;
    }

    return true;
}










template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::VEC
ForgetfulGlobalTrajectory<Type>::buildEquVec
(
    const VEC& Waypoints_w
) const {
    const int Spl_N = Waypoints_w.rows();

    // ii) 0th-derivative (position) constraints:
    //     Each spline start and end at its corresponding waypoints:
    //       2 Constraints per spline
    
    VEC EquVec_w_Deriv0_Start = VEC::Zero(Spl_N);
    VEC EquVec_w_Deriv0_End = VEC::Zero(Spl_N);

    for (int Spl_i = 0; Spl_i < Spl_N; Spl_i++)
    {
        const int WpStart_i = Spl_i;
        const int WpEnd_i = (Spl_i + 1) % Spl_N;

        // Write spatial coordinates into vectors
        EquVec_w_Deriv0_Start[Spl_i] = Waypoints_w[WpStart_i];
        EquVec_w_Deriv0_End[Spl_i] = Waypoints_w[WpEnd_i];
    }

    // Vertcially stack start and stop into vector
    VEC EquVec_w_Deriv0 = VEC::Zero(2 * Spl_N);
    EquVec_w_Deriv0.segment(0, Spl_N) = EquVec_w_Deriv0_Start;
    EquVec_w_Deriv0.segment(Spl_N, Spl_N) = EquVec_w_Deriv0_End;



    // iii) 1st to $(m_ContDiff_N)th derivative constraints:
    //      Enforce continuity of trajectory at waypoints
    //        One constraint per waypoint and continuos derivative
    VEC EquVec_w_Deriv1toN = VEC::Zero(Spl_N * m_ContDiff_N);




    // iv) Final matrix and vector
    VEC EquVec_w = VEC::Zero(Spl_N * (m_ContDiff_N + 2));
    
    EquVec_w.segment(0, 2 * Spl_N) = EquVec_w_Deriv0;
    EquVec_w.segment(2 * Spl_N, m_ContDiff_N * Spl_N) = EquVec_w_Deriv1toN;

    return EquVec_w;
};









template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::MAT 
ForgetfulGlobalTrajectory<Type>::buildEquMat(
    const VEC& Wp_T_NonDim
) const {
    const MAT Wp_tNonDim_Poly = buildPolyMat (Wp_T_NonDim, m_Spl_PolyCff_N);

    // ii) 0th-derivative (position) constraints:
    //     Each spline start and end at its corresponding waypoints:
    //       2 Constraints per spline
    MAT EquMat_Deriv0_Start = MAT::Zero(m_Spl_N, m_TrajPolyCff_N);
    MAT EquMat_Deriv0_End = MAT::Zero(m_Spl_N, m_TrajPolyCff_N);

    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        const int RowStart_i = Spl_i;
        const int ColStart_i = Spl_i * m_Spl_PolyCff_N;
        const int Rows_N = 1;
        const int Cols_N = m_Spl_PolyCff_N;

        const int TStart_i = Spl_i;
        const int TEnd_i = Spl_i + 1;

        
        // Write polynomial powered, preliminary temporal coordinates into matrix
        EquMat_Deriv0_Start.block(RowStart_i, ColStart_i, Rows_N, Cols_N)
            = Wp_tNonDim_Poly.row(TStart_i);

        EquMat_Deriv0_End.block(RowStart_i, ColStart_i, Rows_N, Cols_N)
            = Wp_tNonDim_Poly.row(TEnd_i);
    }

    // Vertcially stack start and stop into matrix and vectors
    MAT EquMat_Deriv0 = MAT::Zero(2 * m_Spl_N, m_TrajPolyCff_N);
    EquMat_Deriv0.block(0, 0, m_Spl_N, m_TrajPolyCff_N) = EquMat_Deriv0_Start;
    EquMat_Deriv0.block(m_Spl_N, 0, m_Spl_N, m_TrajPolyCff_N) = EquMat_Deriv0_End;
    



    // iii) 1st to $(m_ContDiff_N)th derivative constraints:
    //      Enforce continuity of trajectory at waypoints
    //        One constraint per waypoint and continuos derivative
    MAT EquMat_Deriv1toN = MAT::Zero(m_Spl_N * m_ContDiff_N, m_TrajPolyCff_N);

    for (int Deriv_i = 0; Deriv_i < m_ContDiff_N; Deriv_i++)
    {
        // Compute the corresponding derivative matrix
        MAT DerivMat // [SplCff_N x SplCff_N]
            = buildDerivMat(m_Spl_PolyCff_N, Deriv_i + 1);

        // For each spline
        for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
        {
            const int RowStart_i = Deriv_i * m_Spl_N + Spl_i;
            const int Rows_N = 1;

            const int ColStart_i1 = Spl_i * m_Spl_PolyCff_N;
            const int ColsN1 = m_Spl_PolyCff_N;

            const int ColStart_i2 = ((Spl_i + 1) * m_Spl_PolyCff_N) % m_TrajPolyCff_N;
            const int ColsN2 = m_Spl_PolyCff_N;

            // End of current spline
            EquMat_Deriv1toN.block(RowStart_i, ColStart_i1, Rows_N, ColsN1)
                = Wp_tNonDim_Poly.row(Spl_i + 1) * DerivMat;
            // Start of next spline, -
            EquMat_Deriv1toN.block(RowStart_i, ColStart_i2, Rows_N, ColsN2)
                =  - Wp_tNonDim_Poly.row((Spl_i + 2) % m_Wp_T_N) * DerivMat;
        }

        // For each temporal coordinate
        for (int T_i = 1; T_i < m_Wp_T_N; T_i++)
        {
            const int RowStart_i = Deriv_i * m_Spl_N + (T_i - 1);
            const int Rows_N = 1;

            const int ColStart_i1 = (T_i - 1) * m_Spl_PolyCff_N;
            const int ColsN1 = m_Spl_PolyCff_N;

            const int ColStart_i2 = (T_i * m_Spl_PolyCff_N) % m_TrajPolyCff_N;
            const int ColsN2 = m_Spl_PolyCff_N;

            // End of current spline
            EquMat_Deriv1toN.block(RowStart_i, ColStart_i1, Rows_N, ColsN1)
                = Wp_tNonDim_Poly.row(T_i) * DerivMat;
            // Start of next spline, -
            EquMat_Deriv1toN.block(RowStart_i, ColStart_i2, Rows_N, ColsN2)
                = (T_i != m_Wp_T_N - 1)?  
                        - Wp_tNonDim_Poly.row(T_i) * DerivMat
                    :   - Wp_tNonDim_Poly.row(0) * DerivMat;
        }



    }


    // iv) Final matrix
    MAT EquMat = MAT::Zero(m_Spl_N * (m_ContDiff_N + 2), m_TrajPolyCff_N);
    
    EquMat.block(0, 0, 2 * m_Spl_N, m_TrajPolyCff_N) = EquMat_Deriv0;
    EquMat.block(2 * m_Spl_N, 0, m_ContDiff_N * m_Spl_N, m_TrajPolyCff_N) = EquMat_Deriv1toN;


    return EquMat;
}








//m_Spl_N, m_Spl_PolyCff_N, m_ContDiff_N

template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::MAT
ForgetfulGlobalTrajectory<Type>::buildObjMat() const 
{
    //Snap

    // R^[SplCff_N x SplCff_N]
    MAT ObjMat_Spline 
        = buildDerivMat(m_Spl_PolyCff_N, m_ContDiff_N).transpose()
        * buildDerivMat(m_Spl_PolyCff_N, m_ContDiff_N);

    // R^[m_TrajPolyCff_N x m_TrajPolyCff_N]
    MAT ObjMat_Snap = MAT::Zero(m_TrajPolyCff_N, m_TrajPolyCff_N);
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        const int RowStart_i = Spl_i * m_Spl_PolyCff_N;
        const int ColStart_i = Spl_i * m_Spl_PolyCff_N;
        const int Rows_N = m_Spl_PolyCff_N;
        const int Cols_N = m_Spl_PolyCff_N;
        
        ObjMat_Snap.block(RowStart_i, ColStart_i, Rows_N, Cols_N)
            = ObjMat_Spline;
    }

    //Jerk (for numerical stability)

    // R^[SplCff_N x SplCff_N]
    ObjMat_Spline 
        = buildDerivMat(m_Spl_PolyCff_N, m_ContDiff_N - 1).transpose()
        * buildDerivMat(m_Spl_PolyCff_N, m_ContDiff_N - 1);

    // R^[m_TrajPolyCff_N x m_TrajPolyCff_N]
    MAT ObjMat_Jerk = MAT::Zero(m_TrajPolyCff_N, m_TrajPolyCff_N);
    for (int Spl_i = 0; Spl_i < m_Spl_N; Spl_i++)
    {
        const int RowStart_i = Spl_i * m_Spl_PolyCff_N;
        const int ColStart_i = Spl_i * m_Spl_PolyCff_N;
        const int Rows_N = m_Spl_PolyCff_N;
        const int Cols_N = m_Spl_PolyCff_N;
        
        ObjMat_Jerk.block(RowStart_i, ColStart_i, Rows_N, Cols_N)
            = ObjMat_Spline;
    }


    //return ObjMat_Snap + ObjMat_Jerk/1000 + MAT::Identity(m_TrajPolyCff_N, m_TrajPolyCff_N);
    return ObjMat_Snap; //+ MAT::Identity(m_TrajPolyCff_N, m_TrajPolyCff_N);
}
















// wN >< PolyCoeffs_N
template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::MAT
ForgetfulGlobalTrajectory<Type>::buildPolyMat
(
    const VEC& w,
    const int& PolyCoeffs_N // = Polynomial order + 1
) const {
    const int& Coord_N = w.size();
    
    MAT PolyMat = MAT::Zero(Coord_N, PolyCoeffs_N);
    
    for (int i = 0; i < Coord_N; i++)
    {
        for (int j = 0; j < PolyCoeffs_N; j++)
        {
            PolyMat(i, j) = pow(w(i), j);
        }
    }

    return PolyMat;
}









// SplCff_N >< SplCff_N
template<typename Type> 
typename ForgetfulGlobalTrajectory<Type>::MAT 
ForgetfulGlobalTrajectory<Type>::buildDerivMat(
    const int& SplCff_N, 
    const int& Deriv_N
) const {    
    // Build first order derivative matrix
    MAT Deriv1Mat = MAT::Zero(SplCff_N, SplCff_N);
    for (int SplCff_i = 1; SplCff_i < SplCff_N; SplCff_i++)
    {
        Deriv1Mat(SplCff_i -1, SplCff_i) = SplCff_i;
    }

    // Build $(Deriv_N)-th order derivative matrix
    MAT DerivNMat = Deriv1Mat;
    for (int Deriv_i = 1; Deriv_i < Deriv_N; Deriv_i++)
    {
        DerivNMat *= Deriv1Mat;
    }

    return DerivNMat;
}







template<typename Type> 
void 
ForgetfulGlobalTrajectory<Type>::dedimensionalizeCoordinates(
    const VEC& in_xDim,
    const Type& in_xNonDim_MinMax,
    VEC& out_xNonDim,
    Type& out_xDim_Shift,
    Type& out_xDim_Scale
){
    // Dedimensionalize 'in_xDim':
    //   (xDim – out_xDim_Shift) / out_xDim_Scale = out_xNonDim in [-in_xNonDim_MinMax, in_xNonDim_MinMax]
    //
    // Then redimensionalization is:
    //   in_xDim = out_xDim_Shift + out_xDim_Scale * out_xNonDim

    out_xDim_Shift = (in_xDim.maxCoeff() + in_xDim.minCoeff()) / 2;
    out_xDim_Scale = (in_xDim.maxCoeff() - in_xDim.minCoeff()) / 2;

    out_xDim_Scale /= in_xNonDim_MinMax;

    out_xNonDim = (in_xDim.array() - out_xDim_Shift) / out_xDim_Scale;
}

template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::VEC
ForgetfulGlobalTrajectory<Type>::redimensionalizeTemporalCoordinates(
    const VEC& tNonDim
){
    return (tNonDim * m_Scale_T).array() + m_Shift_T;
}
    
template<typename Type>
typename ForgetfulGlobalTrajectory<Type>::VEC
ForgetfulGlobalTrajectory<Type>::redimensionalizeSpatialCoordinates(
    const VEC& wNonDim,
    const int& DiffOrder,
    Type& wDim_Shift,
    Type& wDim_Scale
){
                                                                                
    VEC W_Dim = wDim_Scale / std::pow(m_Scale_T, DiffOrder) * wNonDim;         
    
    if (DiffOrder == 0)
    { 
        W_Dim = W_Dim.array() + wDim_Shift;                             
    }

    return W_Dim;
}


   

}