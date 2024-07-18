#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <map_manager/PCSmap_manager.h>
#include <swept_volume/sw_manager.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <utils/minco.hpp>
#include <utils/config.hpp>
#include <utils/flatness.hpp>
#include <utils/lbfgs.hpp>
#include <utils/lmbm.h>
#include <utils/Visualization.hpp>
#include <utils/debug_publisher.hpp>
#include <omp.h>
#define TRAJ_ORDER 5
#define GNOW ros::Time::now()
#define DEBUGOUT(x) std::cout << x << std::endl




class TrajOptimizer
{
public:

    int map_type;

    Eigen::VectorXd opt_x;

    double total_opt_time  = 0.0;
    double total_sdf_time  = 0.0;
    double total_AABB_time = 0.0;

    // ros
    ros::NodeHandle nh;
    ros::Publisher debug_pub;
    ros::Publisher debug_wp_pub;
    ros::Publisher debug_vec_pub;
    ros::Publisher debug_wplists_pub;
    ros::Publisher test_map_pub;
    ros::Publisher debugesdfpub;
    minco::MINCO_S3NU minco;
    flatness::FlatnessMap flatmap;
    SweptVolumeManager::Ptr sv_manager; 
    PCSmapManager::Ptr pcsmap_manager;
    Trajectory<TRAJ_ORDER> step_traj;
    vis::Visualization::Ptr vis;

    double rho;
    Eigen::Matrix3d initState;
    Eigen::Matrix3d finalState;
    double *x_variable; 

    int pieceN;
    int spatialDim;
    int temporalDim;

    int integralRes;
    Eigen::VectorXd physicalPm;

    Eigen::Matrix3Xd points;
    Eigen::VectorXd times;
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX3d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;

    Config conf;


    double vmax;
    double omgmax;
    double thetamax;
    double weight_v;
    double weight_a;
    double weight_p;
    double weight_omg;
    double weight_theta;
    double smooth_fac;
    double safety_hor;

    double bdx;//=config.kernel_size*config.occupancy_resolution;  bounding box x
    double bdy;//=config.kernel_size*config.occupancy_resolution;  bounding box y
    double bdz;//=config.kernel_size*config.occupancy_resolution;  bounding box z


    vector<Eigen::Matrix<double, 6, 3>> last_gdc;
    vector<double> last_gdt;
    vector<double> last_ts;

    int threads_num{30}; //1.5*kernel size of cpu
    int parallel_points_num;
    std::vector<Eigen::Vector3d> parallel_points; 
    std::vector<double> lastTstar;      
    
    double cost_pos;
    double cost_other;
    double cost_total;
    double ori_cost_pos;

    bool exit = false;


    int iter = 0;

    


    void drawDebug();
    void writeIterTraj(int index);

    void renderAABBpoints();

    inline Eigen::Matrix3d getQuatTransDW(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << 0, 2 * z, -2 * y,
            -2 * z, 0, 2 * x,
            2 * y, -2 * x, 0;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDX(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << 0, 2 * y, 2 * z,
            2 * y, -4 * x, 2 * w,
            2 * z, -2 * w, -4 * x;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDY(const Eigen::Vector4d &quat)
    {
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << -4 * y, 2 * x, -2 * w,
            2 * x, 0, 2 * z,
            2 * w, 2 * z, -4 * y;
        return ret;
    }
    inline Eigen::Matrix3d getQuatTransDZ(const Eigen::Vector4d &quat)
    { 
        Eigen::Matrix3d ret;
        double w = quat(0);
        double x = quat(1);
        double y = quat(2);
        double z = quat(3);
        ret << -4 * z, 2 * w, 2 * x,
            -2 * w, -4 * z, 2 * y,
            2 * x, 2 * y, 0;
        return ret;
    }

    static inline void forwardP(const Eigen::VectorXd &xi,
                                Eigen::Matrix3Xd &P)
    {
        const int sizeP = xi.size() / 3;
        P.resize(3, sizeP);
        for (int i = 0; i < sizeP; i++)
        {
            P.col(i) = xi.segment(3 * i, 3);
        }
        return;
    }
    static inline void forwardP(const double* xi,
                                Eigen::Matrix3Xd &P,int Space_dim)
    {
       
        P.resize(3, Space_dim);
        for (int i = 0; i < Space_dim; i++)
        {
             Eigen::Map<const Eigen::VectorXd> vec(xi+3*i, 3);
             P.col(i)=vec;
        }
        return;
    }
    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::Matrix3Xd &P,
                                 EIGENVEC &xi)
    {
        const int sizeP = P.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            xi.segment(3 * i, 3) = P.col(i);
        }
        return;
    }

    // tao--->T
    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T)
    {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++)
        {
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
        return;
    }
    // tao--->T
    static inline void forwardT(const double *tau,
                                Eigen::VectorXd &T, const int Time_dim)
    {
        T.resize(Time_dim);
        double temp;
        for (int i = 0; i < Time_dim; i++)
        {
            temp = *(tau + i);
            T(i) = temp > 0.0
                       ? ((0.5 * temp + 1.0) * temp + 1.0)
                       : 1.0 / ((0.5 * temp-1.0) * (temp) + 1.0);
        }
        return;
    }
    // T--->tao
    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &T,
                                 EIGENVEC &tau)
    {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++)
        {
            tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                     const Eigen::VectorXd &gradT,
                                     EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;

        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

      static inline void backwardGradT(const double* tau,
                                     const Eigen::VectorXd &gradT,
                                     double * gradTau,
                                     const int sizeTau)
    {
        double denSqrt;

        for (int i = 0; i < sizeTau; i++)
        {
            if (tau[i] > 0)
            {
                gradTau[i] = gradT[i] * (tau[i] + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau[i] - 1.0) * tau[i] + 1.0;
                gradTau[i] = gradT[i] * (1.0 - tau[i]) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradP(const Eigen::VectorXd &xi,
                                     const Eigen::Matrix3Xd &gradP,
                                     EIGENVEC &gradXi)
    {
        const int sizeP = gradP.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            gradXi.segment(3 * i, 3) = gradP.col(i);
        }
        return;
    }
     static inline void backwardGradP(
                                     const Eigen::Matrix3Xd &gradP,
                                     double *gradXi,
                                     const int Space_dim)
    {

        for (int i = 0; i < Space_dim; ++i)
        {
             Eigen::Map<Eigen::VectorXd>(gradXi+ 3 * i, 3) = gradP.col(i);
        }
        return;
    }

    static inline bool smoothedL1(const double &x,
                                  const double &mu,
                                  double &f,
                                  double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

 

 static inline double costFunctionLmbmParallel(void *ptr,
                                          const double *x_variable,
                                          double *g,
                                          const int n)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const int dimTau     = obj.temporalDim;
        const int dimXi      = obj.spatialDim;
        const double weightT = obj.rho;
        forwardT(x_variable, obj.times, dimTau); 
        forwardP((x_variable + dimTau), obj.points, dimXi / 3);

        double energy_cost = 0.0;
        double pos_cost = 0.0;
        double dyn_cost = 0.0;
        double time_cost = 0.0;

        double cost;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs); // ∂E/∂c
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);   // ∂E/∂T
        obj.minco.getTrajectory(obj.step_traj);
        obj.sv_manager->updateTraj(obj.step_traj);
        energy_cost = cost;

        auto start1 = chrono::high_resolution_clock::now();


        addSaftyPenaOnSweptVolumeParallelTrueSDF( ptr,
                                   obj.times,
                                   obj.minco.getCoeffs(),
                                   cost,
                                   obj.partialGradByTimes,
                                   obj.partialGradByCoeffs );
        
    
       
        auto end1     = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::nanoseconds>(end1 - start1);
        pos_cost = cost - energy_cost;

       
        dyn_cost = cost - pos_cost - energy_cost;

        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, // ∂Cost/∂c,T -> ∂Cost/∂q,T
                                obj.gradByPoints, obj.gradByTimes);
        cost += weightT * obj.times.sum();
        time_cost = cost - pos_cost - energy_cost - dyn_cost;


        obj.gradByTimes.array() += weightT;
        obj.cost_pos = pos_cost;
        obj.cost_other = cost - pos_cost;
        obj.cost_total = cost;
      
        backwardGradT(x_variable, obj.gradByTimes, g,dimTau);
        backwardGradP(obj.gradByPoints, g+dimTau,dimXi/3);

    

        return cost;


    }
    

    static inline void addTimeIntPenalty( void *ptr,
                                          const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          double &cost,
                                          double &pos_cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const double velSqrMax = obj.vmax   * obj.vmax;   
        const double omgSqrMax = obj.omgmax * obj.omgmax ;   
        const double thetaMax  = obj.thetamax;                        
        
        const double weightVel = obj.weight_v;    
        const double weightPos = obj.weight_p;
        const double weightOmg = obj.weight_omg;
        const double weightTheta = obj.weight_theta;

        double totalGradPsi, totalGradPsiD;
        double thr, cos_theta;
        double gradThr;
        Eigen::Vector3d pos, vel, acc, jer, sna;
        Eigen::Vector3d totalGradVel, totalGradAcc;
        Eigen::Vector4d quat;
        double omg;
        Eigen::Vector4d gradQuat, violaQuatPenaGrad;
        Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg, violaPosPenaGrad;
        Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
        Eigen::Matrix3d rotate,St;

        double gradPsiT, gradDPsiT;

        double step, alpha;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        Eigen::Vector3d outerNormal;
        double violaVel, violaAcc, violaOmg, violaTheta, violaThrust;
        double violaVelPenaD, violaOmgPenaD, violaThetaPenaD;
        double violaVelPena, violaOmgPena, violaPosPena, violaThetaPena;
        double node, pena;
        double last_ttime = 0.0;

        const int pieceNum        = T.size();
        const int integralResolution = obj.integralRes;
        const double smoothFactor = obj.smooth_fac;
        const double integralFrac = 1.0 / integralResolution;

        pos_cost = 0;
        for (int i = 0; i < pieceNum; i++)
        {
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
            step = T(i) * integralFrac;

            for (int j = 0; j <= integralResolution; j++)
            {
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;
                sna = c.transpose() * beta4;
                
                omg = vel(2);

                St     = obj.sv_manager -> getScale(s1);  
                pena   = 0.0;
                gradVel.setZero();
                gradAcc.setZero();
                gradPos.setZero();
                gradOmg.setZero();
                gradQuat.setZero();
                gradPosTotal.setZero();
                gradVelTotal.setZero();
                gradAccTotal.setZero();
                gradJerTotal.setZero();

                cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                violaVel = vel.head(2).squaredNorm() - velSqrMax;
                violaOmg = omg*omg - omgSqrMax;


                double grad_yaw;
                if (obj.grad_cost_p(pos, rotate, St, quat, violaPosPenaGrad, grad_yaw, violaPosPena)) 
                {
                    violaPosPenaGrad(2)= 0;
                    gradPos    = weightPos * violaPosPenaGrad;
                    gradPos(2) = weightPos * grad_yaw;
                    pena       = weightPos * violaPosPena;
                    pos_cost += node * step * pena;
                }
                gradPosTotal = gradPos;   
             
                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                alpha = j * integralFrac;
                gradC.block<6, 3>(i * 6, 0) += (beta0 * gradPosTotal.transpose() +
                                                beta1 * gradVelTotal.transpose() +
                                                beta2 * gradAccTotal.transpose() +
                                                beta3 * gradJerTotal.transpose()) *
                                               node * step;

                gradT(i) += (gradPosTotal.dot(vel) +
                             gradVelTotal.dot(acc) +
                             gradAccTotal.dot(jer) +
                             gradJerTotal.dot(sna)) *
                                alpha * node * step +
                            node * integralFrac * pena;

                cost += node * step * pena;

            }
            last_ttime += T(i);
        }
        return;
    }

    static inline void addTimeIntPenaltyParallel( void *ptr,
                                          const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          double &cost,
                                          double &pos_cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const double velSqrMax = obj.vmax   * obj.vmax;  
        const double omgSqrMax = obj.omgmax * obj.omgmax ; 
        const double thetaMax  = obj.thetamax;                      
        const double weightVel = obj.weight_v;    
        const double weightPos = obj.weight_p;
        const double weightOmg = obj.weight_omg;
        const double weightTheta = obj.weight_theta;
        const int pieceNum        = T.size();
        const int integralResolution = obj.integralRes;
        const double smoothFactor = obj.smooth_fac;
        const double integralFrac = 1.0 / integralResolution;

        pos_cost = 0;
        #pragma omp parallel for num_threads(obj.threads_num) schedule(dynamic)
        for (int count = 0; count < pieceNum * (integralResolution + 1); count++)
        {
            int j = count % (integralResolution + 1);
            int i = count / (integralResolution + 1);
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
            double step = T(i) * integralFrac;            
            double s1 = j * step;
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
            beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
            beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
            beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
            beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
            Eigen::Vector3d pos = c.transpose() * beta0;
            Eigen::Vector3d vel = c.transpose() * beta1;
            Eigen::Vector3d acc = c.transpose() * beta2;
            Eigen::Vector3d jer = c.transpose() * beta3;
            Eigen::Vector3d sna = c.transpose() * beta4;
            double omg = vel(2);
            Eigen::Matrix3d St     = obj.sv_manager -> getScale(s1); 
            double pena   = 0.0;
            Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
            gradVel.setZero();
            gradAcc.setZero();
            gradPos.setZero();
            gradOmg.setZero();
            gradQuat.setZero();
            gradVelTotal.setZero();
            gradAccTotal.setZero();
            gradPosTotal.setZero();
            gradJerTotal.setZero();

            double node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            double alpha = j * integralFrac;


            #pragma omp critical
            {
            gradC.block<6, 3>(i * 6, 0) += (beta0 * gradPosTotal.transpose() +
                                            beta1 * gradVelTotal.transpose() +
                                            beta2 * gradAccTotal.transpose() +
                                            beta3 * gradJerTotal.transpose()) *
                                           node * step;

            gradT(i) += (gradPosTotal.dot(vel) +
                        gradVelTotal.dot(acc) +
                        gradAccTotal.dot(jer) +
                        gradJerTotal.dot(sna)) *
                            alpha * node * step +
                        node * integralFrac * pena;

            cost += node * step * pena;
            }
        }
        return;
    }




    static inline void addSaftyPenaOnSweptVolume(   void *ptr,
                                                    const Eigen::VectorXd &T,
                                                    const Eigen::MatrixX3d &coeffs,
                                                    double &cost,
                                                    Eigen::VectorXd &gradT,
                                                    Eigen::MatrixX3d &gradC)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        int interation=obj.iter;
        double weightPos = obj.weight_p;
        
        // weightPos+=(double)interation*1.0;//trick

        double thr, cos_theta, pena;
        pena = 0;

        Eigen::Vector3d pos, vel, acc, jer, sna, tmp_pos;
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;
        double gradPsiT, gradDPsiT;

        double s1, s2, s3, s4, s5;
        double violaVelPenaD, violaAccPenaD;
        double violaVelPena, violaAccPena, violaPosPena;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        double grad_yaw, violaYawGrad;
        Eigen::Vector3d gradVel, gradAcc, gradPos, gradOmg, violaPosPenaGrad;
        Eigen::Vector3d gradVelTotal, gradAccTotal, gradPosTotal, gradJerTotal;
        Eigen::Matrix3d rotate;

        Eigen::Vector3d gradp_rel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gradp_eva{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d Rt,St;
        Eigen::Vector3d pos_eva;
        Eigen::Vector3d last_pos_eva;
        bool first_obs;
        bool set_ts;
        double neighbor_dis = 2*obj.conf.occupancy_resolution;
        double sdf_value = 0.0;
        double time_star = 0.0;
        double time_seed_f = 0.0;
        double time_seed_tmp = 0.0;
        double time_local = 0.0;
        double yaw;

        const int pieceNum = T.size();

        first_obs = true;
        vector<Eigen::Vector3d> pos_eva_vec;

        int index = 0;
        obj.ori_cost_pos = 0.0;
        for( auto iter = obj.pcsmap_manager->aabb_points.begin(); iter != obj.pcsmap_manager->aabb_points.end(); ++iter)
        {
            pos_eva    = iter -> second;
            pos_eva(2) = 0;
            set_ts    = false;

           
            sdf_value      = obj.sv_manager -> getTrueSDFofSweptVolume<true>(pos_eva, time_star, gradp_rel, set_ts);  

            time_seed_f = time_star;
            time_local  = time_seed_f;
            int i = obj.step_traj.locatePieceIdx(time_local); // 改变这个esdf_temp，locatePieceIdx为引用，不能传入time_seed_f
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0); // 确定这段的Id
            s1 = time_local; 
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
            beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
            beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
            beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
            beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;

            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;

            gradVel.setZero();
            gradAcc.setZero();
            gradPos.setZero();
            gradOmg.setZero();
            grad_yaw = 0.0;
            gradPosTotal.setZero();
            gradVelTotal.setZero();
            gradAccTotal.setZero();
            gradJerTotal.setZero();
            yaw = pos(2);
            violaYawGrad=0;
            violaPosPena=0;
            rotate = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
            St     = obj.sv_manager -> getScale(time_seed_f);
            pena   = 0.0;
            pos(2) = 0.0;
                     
            if( sdf_value < 0 ) {gradp_rel = rotate.transpose() * gradp_rel;}
            if (obj.grad_cost_p_sw( pos_eva, pos, rotate, yaw, 
                                    St, sdf_value, gradp_rel, 
                                    violaPosPenaGrad, violaYawGrad, violaPosPena ))
            {
                gradPos  += weightPos * violaPosPenaGrad;
                grad_yaw += weightPos * violaYawGrad;
                pena     += weightPos * violaPosPena;
                cost     += pena; 
                obj.ori_cost_pos += violaPosPena;
            }

            gradPosTotal    = gradPos;
            gradPosTotal(2) = grad_yaw;
            Eigen::Matrix<double , 6, 3> gdC = ( beta0  * gradPosTotal.transpose()   +  
                                                 beta1  * gradVelTotal.transpose()   +  
                                                 beta2  * gradAccTotal.transpose()   +
                                                 beta3  * gradJerTotal.transpose()  );

            double gdT =    (   -gradPosTotal.dot(vel) +
                                -gradVelTotal.dot(acc) +
                                -gradAccTotal.dot(jer) +
                                -gradJerTotal.dot(sna)  );

            // if(std::isnan(gdT) || std::isinf(gdT) ) {
            // std::cout<<"gdT = " <<gdT <<std::endl;
            // std::cout<<"gradPosTotal:"<<gradPosTotal.transpose()<<std::endl;
            // std::cout<<"gradVelTotal:"<<gradVelTotal.transpose()<<std::endl;
            // std::cout<<"gradAccTotal:"<<gradAccTotal.transpose()<<std::endl;
            // std::cout<<"gradJerTotal:"<<gradJerTotal.transpose()<<std::endl;
            // std::cout<<"vel"<<vel.transpose()<<std::endl;
            // std::cout<<"acc"<<acc.transpose()<<std::endl;
            // std::cout<<"jer"<<jer.transpose()<<std::endl;
            // std::cout<<"sna"<<sna.transpose()<<std::endl;
            // abort();
                
            // }
            

            gradC.block<6, 3>(i * 6, 0) += gdC;
            for (int j = 0; j < i; j++) 
            {
                gradT(j) += gdT;
            }
            index++ ;
        }


        return;
    }

    static inline void addSaftyPenaOnSweptVolumeParallelTrueSDF(   void *ptr,
                                                    const Eigen::VectorXd &T,
                                                    const Eigen::MatrixX3d &coeffs,
                                                    double &cost,
                                                    Eigen::VectorXd &gradT,
                                                    Eigen::MatrixX3d &gradC)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)ptr;
        const double weightPos = obj.weight_p;
        const double neighbor_dis = 2 * obj.conf.occupancy_resolution;
        const int pieceNum = T.size();
#pragma omp parallel for num_threads(obj.threads_num) schedule(dynamic)
        for(int pointindex = 0; pointindex < obj.parallel_points_num; ++pointindex)
        {
            // int thread_num = omp_get_thread_num();
            // printf("Thread number: %d\n", thread_num);
            Eigen::Vector3d pos_eva = obj.parallel_points[pointindex]; 
            pos_eva(2) = 0;
            Eigen::Vector3d gradp_rel;
            double time_star;
            double sdf_value      = obj.sv_manager -> getTrueSDFofSweptVolume<true>(pos_eva, time_star, gradp_rel, false);  
            double time_seed_f = time_star;
            double time_local  = time_seed_f;
            int i = obj.step_traj.locatePieceIdx(time_local); // 改变这个esdf_temp，locatePieceIdx为引用，不能传入time_seed_f
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0); // 确定这段的Id
            double s1 = time_local; 
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
            beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
            beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
            beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
            beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;

            Eigen::Vector3d pos = c.transpose() * beta0;
            Eigen::Vector3d vel = c.transpose() * beta1;
            Eigen::Vector3d acc = c.transpose() * beta2;
            Eigen::Vector3d jer = c.transpose() * beta3;
            Eigen::Vector3d sna = c.transpose() * beta4;
            Eigen::Vector3d gradPos, gradOmg, violaPosPenaGrad, gradPosTotal, gradVelTotal, gradAccTotal, gradJerTotal;

            gradPos.setZero();
            gradOmg.setZero();
            double grad_yaw = 0.0;
            gradPosTotal.setZero();
            gradVelTotal.setZero();
            gradAccTotal.setZero();
            gradJerTotal.setZero();
            double yaw = pos(2);
            Eigen::Matrix3d rotate = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
            Eigen::Matrix3d St     = obj.sv_manager -> getScale(time_seed_f);
            double pena   = 0.0;
            pos(2) = 0.0;
            double violaYawGrad=0;
            double violaPosPena=0;
             if( sdf_value < 0 ) {gradp_rel = rotate.transpose() * gradp_rel;}//内部世界坐标系下SDF梯度不对
            if (obj.grad_cost_p_sw( pos_eva, pos, rotate, yaw, 
                                    St, sdf_value, gradp_rel, 
                                    violaPosPenaGrad, violaYawGrad, violaPosPena))
            {
                gradPos  += weightPos * violaPosPenaGrad;
                grad_yaw += weightPos * violaYawGrad;
                pena     += weightPos * violaPosPena;
                // cost     += pena; //移到后面进行critical section 操作
                obj.ori_cost_pos += violaPosPena;
            }

            gradPosTotal    = gradPos;
            gradPosTotal(2) = grad_yaw;
            Eigen::Matrix<double , 6, 3> gdC = ( beta0  * gradPosTotal.transpose()   +  
                                                 beta1  * gradVelTotal.transpose()   +  
                                                 beta2  * gradAccTotal.transpose()   +
                                                 beta3  * gradJerTotal.transpose()  );

            double gdT =    (   -gradPosTotal.dot(vel) +
                                -gradVelTotal.dot(acc) +
                                -gradAccTotal.dot(jer) +
                                -gradJerTotal.dot(sna)  );
            #pragma omp critical
            {
                cost     += pena; 
                gradC.block<6, 3>(i * 6, 0) += gdC;
                for (int j = 0; j < i; j++) 
                {
                    gradT(j) += gdT;
                }
            }
           
        }


        return;
    }

  

public:
    TrajOptimizer() {}
    ~TrajOptimizer() {}

    void setParam(ros::NodeHandle &nh, Config &config)
    {
        this->conf = config;
        this->nh   = nh;

        vis.reset(new vis::Visualization(nh));

        vmax           = config.vmax;
        omgmax         = config.omgmax;
        thetamax       = config.thetamax;
        threads_num    = config.threads_num;
        rho            = config.rho;
        weight_a       = config.weight_a;
        weight_p       = config.weight_p;
        weight_v       = config.weight_v;
        weight_omg     = config.weight_omg;
        weight_theta   = config.weight_theta;

        smooth_fac      = config.smoothingEps;
        integralRes     = config.integralIntervs;
        safety_hor      = config.safety_hor;
        bdx = bdy = bdz = config.kernel_size * config.occupancy_resolution;
        // kernel_size    = config.kernel_size;
        // side_size      = max(max(bdx,bdy),bdz);

        ROS_WARN_STREAM("==[back_end]==");
        ROS_WARN_STREAM("==vmax=="      << vmax);
        ROS_WARN_STREAM("==omgmax=="    << omgmax);
        ROS_WARN_STREAM("==thetamax=="  << thetamax);
        ROS_WARN_STREAM("==rho=="       << rho);
        ROS_WARN_STREAM("==weight_v=="  << weight_v);
        ROS_WARN_STREAM("==weight_a=="  << weight_a);
        ROS_WARN_STREAM("==weight_p=="  << weight_p);
        ROS_WARN_STREAM("==smooth_fac==" << smooth_fac);
        ROS_WARN_STREAM("==safety_hor==" << safety_hor);
        ROS_WARN_STREAM("==init time allocate perpiece for minco==" << config.inittime); 
        ROS_WARN_STREAM("bounding box bdx"<<bdx);
        ROS_WARN_STREAM("bounding box bdy"<<bdy);
        ROS_WARN_STREAM("bounding box bdz"<<bdz);

   

        // flatmap;
        flatmap.reset(config.vehicleMass , config.gravAcc   ,  config.horizDrag,
                      config.vertDrag    , config.parasDrag ,  config.speedEps);


        debug_pub           = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path", 10);
        debugesdfpub        = nh.advertise<std_msgs::Float32MultiArray>("/traj_opt/esdf", 10);
        debug_wplists_pub   = nh.advertise<visualization_msgs::MarkerArray>("/traj_opt/debug_wplists", 10);
        debug_wp_pub        = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_wp", 10);
        debug_vec_pub       = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_vec", 10);
        test_map_pub        = nh.advertise<sensor_msgs::PointCloud2>("/traj_opt/debug_test_map", 10);


    }

    void setEnvironment(SweptVolumeManager::Ptr sv) { sv_manager = sv; }
    void setGridMap(PCSmapManager::Ptr psm) { pcsmap_manager = psm; }

    void clearvisAABBpoints();
   


    int optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                       const Eigen::Matrix3d &finalS,
                       Eigen::VectorXd &opt_x,
                       const int N,
                       Trajectory<TRAJ_ORDER> &traj);

    inline void cubic( double x,  double& cost_x , double& grad_x )
    {
        if(x < 0){cost_x = 0 ; grad_x = 0; return ;}
        cost_x = pow(x,3);
        grad_x = 3*x*x;
        return;
    }


    bool inline grad_cost_p(const Eigen::Vector3d &pos,
                            Eigen::Matrix3d &R,
                            const Eigen::Matrix3d &St,   
                            const Eigen::Vector4d &quat, 
                            Eigen::Vector3d &gradp,
                            double &grad_yaw,
                            double &costp)
    {
        double yaw = pos[2];
        costp = 0.0;
        gradp.setZero();
        grad_yaw = 0.0;
        R(0,0) = cos(yaw);
        R(0,1) = -sin(yaw);
        R(1,0) = sin(yaw);
        R(1,1) = cos(yaw);
        R(2,2) = 1;

        double sdf_value = 0.0;
        double sdf_cost = 0.0;
        double grad_out = 0.0;
        Eigen::Vector3d sdf_grad{Eigen::Vector3d::Zero()};
        Eigen::Vector3d gradp_rel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_minus_x;
        Eigen::Vector3d pos_eva;

        double turcation = safety_hor;                           
        std::vector<Eigen::Vector3d> ob_pts;
        ob_pts.clear();

        ros::Time t1 = ros::Time::now();

        pcsmap_manager -> getPointsInAABB(pos, bdx/2,bdy/2,bdz/2, ob_pts);
    
        ros::Time t2 = ros::Time::now();
        total_AABB_time += (t2-t1).toSec()*1000;

        Eigen::Vector3d p_rel;
        for (int i = 0; i < ob_pts.size(); i++) 
        {
            pos_eva   = ob_pts[i];
            p_minus_x = pos_eva - pos;

            p_rel = R.transpose()*p_minus_x;
            Eigen::Matrix3d VR_theta = Eigen::Matrix3d::Zero();
            if(abs(p_rel.x())>bdx/2||abs(p_rel.y())>bdy/2||abs(p_rel.z())>bdz/2)
            {
                continue; 
            }
            ros::Time t1 = ros::Time::now();
            sdf_value = sv_manager -> getSDFWithGradWhenRobotAtState(pos_eva, gradp_rel, pos, R);

            ros::Time t2 = ros::Time::now();
            total_sdf_time += (t2-t1).toSec() * 1000;
            sdf_cost = -1.0;
            smoothedL1(turcation - sdf_value, 0.01, sdf_cost, grad_out );
      
            sdf_grad = -grad_out * (-R * gradp_rel);

            if (sdf_cost > 0)
            {
                costp += sdf_cost;
                gradp += sdf_grad; 
                VR_theta(0,0) = -sin(yaw);
                VR_theta(0,1) = -cos(yaw);
                VR_theta(1,0) = cos(yaw);
                VR_theta(1,1) = -sin(yaw);
                VR_theta(2,2) = 1;
                grad_yaw     += -grad_out * gradp_rel.transpose() * (VR_theta.transpose() * p_minus_x);
            }
        }
        return (costp > 0);
    }


    bool inline grad_cost_p_sw( const Eigen::Vector3d& pos_eva,
                                const Eigen::Vector3d& pos_obj, 
                                const Eigen::Matrix3d& rotate,
                                const double& yaw,
                                const Eigen::Matrix3d& St,
                                const double sdf_value,
                                const Eigen::Vector3d& gradp_rel,
                                Eigen::Vector3d &gradp,
                                double &grad_yaw,
                                double &costp)
    {
        costp = 0.0;
        gradp.setZero();
        grad_yaw = 0.0;
        double sdf_cost = -1.0;
        Eigen::Vector3d p_minus_x;
        Eigen::Matrix3d VR_theta = Eigen::Matrix3d::Zero();
        double sdf_out_grad = 0.0;
        smoothedL1(safety_hor - sdf_value ,0.01, sdf_cost ,sdf_out_grad);
        Eigen::Vector3d sdf_grad = -sdf_out_grad * (-(St.inverse()).transpose()* rotate * gradp_rel);

        if (sdf_cost > 0)
        {   
            costp        += sdf_cost;
            gradp        += sdf_grad; 
            p_minus_x     = pos_eva - pos_obj;
            VR_theta(0,0) = -sin(yaw);
            VR_theta(0,1) = -cos(yaw);
            VR_theta(1,0) = cos(yaw);
            VR_theta(1,1) = -sin(yaw);
            VR_theta(2,2) = 1;
            grad_yaw      = -sdf_out_grad * gradp_rel.transpose() * (VR_theta.transpose() * p_minus_x);

        }
        return (costp > 0);
    }

    static void AsyncSleepMS(const int ms)
    {
        auto start    = std::chrono::steady_clock::now();
        auto duration = std::chrono::milliseconds(ms);
        bool done = false;
        while(!done)
        {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if(elapsed > duration) {
                done = true;
            }
            else{
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

    }
    static inline int earlyExitLMBM(    void *instance,
                                        const double *x,
                                        const int k      )
    {
        TrajOptimizer &obj = *(TrajOptimizer *)instance;
        obj.iter++;
        std::cout << "iter = " << obj.iter << std::endl;
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebug();
            vector<double> costs;
            costs.push_back(obj.cost_pos);
            costs.push_back(obj.cost_other);
            debug_publisher::DBSendLogCost(costs);
            AsyncSleepMS(obj.conf.debugpause);
            std::cout<<"[total cost] = "<<obj.cost_total <<" , [other cost] = " <<obj.cost_other <<std::endl;

        }

        if( obj.exit ){
            obj.exit = false;
             std::cout<<"[early exit stop in earlyExitLMBM] "<<std::endl;
            return 1;
        }
       
        return obj.iter > 8 * 1e3;
    }
    
   static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls)
    {
        TrajOptimizer &obj = *(TrajOptimizer *)instance;
        obj.iter++;
        std::cout << "iter = " << obj.iter << " step = " << step << std::endl;
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebug();
            AsyncSleepMS(obj.conf.debugpause);
            std::cout << "[total cost] = " << obj.cost_total << " , [other cost] = " << obj.cost_other << std::endl;

        }
        if (obj.exit)
        {
            obj.exit = false;
            std::cout<<"[early exit stop in earlyExit] "<<std::endl;
            return 1;
        }
        return obj.iter > 5e4;
    }




public:
    typedef shared_ptr<TrajOptimizer> Ptr;
};
















