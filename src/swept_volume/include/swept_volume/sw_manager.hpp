#ifndef SW_MANAGER_HPP
#define SW_MANAGER_HPP
#define IGL_STATIC_LIBRARY 1 
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigen>
#include "utils/trajectory.hpp"
#include "utils/Shape.hpp"
#include "utils/Visualization.hpp"
#include "utils/config.hpp"
#include "utils/flatness.hpp"
#include "utils/lbfgs.hpp"
#include <swept_volume/sw_calculate.hpp>
#include <igl/write_triangle_mesh.h> 
#include <sys/stat.h> 
#define TRAJ_ORDER 5
#define useScale false  // 定义是否使用伸缩变换
#define useNumer false // 是否使用数值梯度计算扫略体积与优化

#define pi 3.1415926536

using namespace shape;
using namespace Eigen;

class CircleCoord2D
{
public:
    double r_k; // [0,1]
    double theta;
    CircleCoord2D() {}
    CircleCoord2D(double r_k_value, double theta_value)
    {
        r_k = r_k_value;
        theta = theta_value;
    }
    inline Eigen::Vector2d getPosition(const Eigen::Vector2d &center, const double r) const
    {
        return center + Eigen::Vector2d(r_k * r * cos(theta), r_k * r * sin(theta));
    }
};
class SampleSet2D
{
public:
    int k;
    double theta_res;
    double theta0;
    double rk0;
    double rk_res;
    Eigen::Vector2d center;
    double r;

    pcl::PointCloud<pcl::PointXYZI> history_Yks;

    inline void setCircleCenter(const Eigen::Vector2d &c) { center = c; }
    inline void setCircleRadius(const double radius) { r = radius; }

    inline Eigen::Vector2d getElementPos(const CircleCoord2D &item) { return item.getPosition(center, r); }

    inline std::vector<CircleCoord2D> getElements()
    {
        std::vector<CircleCoord2D> elements;
        for (double rk = rk0; rk > 0; rk -= rk_res)
        {
            for (double theta = theta0; theta < theta0 + 2 * PI; theta += theta_res)
            {
                elements.emplace_back(CircleCoord2D{rk, theta});
            }
        }
        return elements;
    }
    inline pcl::PointCloud<pcl::PointXYZI> getHistoryYkCloud() { return history_Yks; }

    inline void initSet(const Eigen::Vector2d &vel, const Eigen::Vector2d &center, const double r_init)
    {
        k = 0;
        history_Yks.points.clear();
        setCircleCenter(center);
        setCircleRadius(r_init);

        theta0 = atan2(vel(0), -vel(1));
        if (theta0 < 0)
        {
            theta0 += 2 * PI;
        }

        theta_res = PI + 0.1;
        rk_res = 1.5;
        rk0 = 1.0;

        // just for visualize
        std::vector<CircleCoord2D> elements = getElements();
        Eigen::Vector2d pos;
        pcl::PointXYZI point;
        for (auto e : elements)
        {
            pos = e.getPosition(center, r);
            point.x = pos(0);
            point.y = pos(1);
            point.z = 1.0;
            point.intensity = 1;
            history_Yks.points.emplace_back(point);
        }
    }

    inline void expandSet(int theta_dense, double new_theta0)
    {
        k++;
        theta_res /= (theta_dense + 1);
        theta_res = std::max(0.3, theta_res);
        theta0 = new_theta0;
        // just for visualize
        std::vector<CircleCoord2D> elements = getElements();
        for (auto e : elements)
        {
            pcl::PointXYZI point;
            Eigen::Vector2d pos = e.getPosition(center, r);
            point.x = pos(0);
            point.y = pos(1);
            point.z = 1.0;
            point.intensity = k;
            history_Yks.points.push_back(point);
        }
    }
};

class SweptVolumeManager 
{
public:
    uint8_t *map_kernel;
    int map_Xsize;
    int map_Ysize;
    int map_Zsize;

    Eigen::Vector3d current_pos_eva;
    double traj_duration;
    Config config;
    Eigen::Matrix3d lastRotate;
    bool writetoobj{false};

private:

    Trajectory<TRAJ_ORDER> traj;
    double eps{0.01};
    Eigen::RowVector3d p0;
    const double iso = 0.001; 
    int num_seeds{100};       

    double momentum{0.0}; 
    double trajStamp;     
    double t_min{0.0};
    double t_max{1.0};
    int kernel_count; 
    int kernelsize;

    
    Eigen::MatrixXd X, N, B;
    Eigen::VectorXi I;
    double minx, miny, minz;
    std::vector<double> init_times;              
    std::vector<Eigen::RowVector3d> init_points; 
    std::vector<Eigen::RowVector3i> init_voxels; 

    Eigen::MatrixXi F_sw_obj;       
    Eigen::MatrixXd V_sw_obj;       


public:
    sw_calculate::Ptr swept_cal; 
   
    BasicShape *current_robot_shape{nullptr}; 

    Visualization::Ptr vis;
    flatness::FlatnessMap flatness;

public:
    inline void writeSVtoObj(const std::string& output_path)
    {
        std::string sw_path=output_path+"/sw.obj";
        std::string start_path=output_path+"/start.obj";
        std::string end_path=output_path+"/end.obj";
        igl::write_triangle_mesh(sw_path, V_sw_obj, F_sw_obj);
        igl::write_triangle_mesh(start_path, current_robot_shape->V_start, current_robot_shape->F);
        igl::write_triangle_mesh(end_path, current_robot_shape->V_end, current_robot_shape->F);
        return;
    }

    std::map<std::string, std::function<BasicShape *(const Config &)>> shapeConstructors = {
        {"sdUnevenCapsule", [](const Config &conf)
         { std::cout << "\033[35m=======sdUnevenCapsule======= "<< "\033[0m" << std::endl;
        return new sdUnevenCapsule(conf); }},
        {"sdCutDisk", [](const Config &conf)
         { std::cout << "\033[35m=======sdCutDisk======= "<< "\033[0m" << std::endl;
        return new sdCutDisk(conf); }},
        {"sdTrapezoid", [](const Config &conf)
         { std::cout << "\033[35m=======sdTrapezoid======= "<< "\033[0m" << std::endl;
        return new sdTrapezoid(conf); }},
        {"sdRhombus", [](const Config &conf)
         { std::cout << "\033[35m=======sdRhombus======= "<< "\033[0m" << std::endl;
        return new sdRhombus(conf); }},
        {"star", [](const Config &conf)
         { std::cout << "\033[35m=======star======= "<< "\033[0m" << std::endl;
        return new star(conf); }},
        {"sdTunnel", [](const Config &conf)
         { std::cout << "\033[35m=======sdTunnel======= "<< "\033[0m" << std::endl;
        return new sdTunnel(conf); }},
        {"sdHorseshoe", [](const Config &conf)
         { std::cout << "\033[35m=======sdHorseshoe======= "<< "\033[0m" << std::endl;
        return new sdHorseshoe(conf); }},
        {"sdHeart", [](const Config &conf)
         { std::cout << "\033[35m=======sdHeart======= "<< "\033[0m" << std::endl;
        return new sdHeart(conf); }},
        {"sdOrientedVesica", [](const Config &conf)
         { std::cout << "\033[35m=======sdOrientedVesica======= "<< "\033[0m" << std::endl;
        return new sdOrientedVesica(conf); }},
        {"sdRoundedCross", [](const Config &conf)
         { std::cout << "\033[35m=======sdRoundedCross======= "<< "\033[0m" << std::endl;
        return new sdRoundedCross(conf); }},
        {"sdRoundedX", [](const Config &conf)
         { std::cout << "\033[35m=======sdRoundedX======= "<< "\033[0m" << std::endl;
        return new sdRoundedX(conf); }},
        {"bigX", [](const Config &conf)
         { std::cout << "\033[35m=======bigX======= "<< "\033[0m" << std::endl;
        return new bigX(conf); }},
        {"sdMoon", [](const Config &conf)
         { std::cout << "\033[35m=======sdMoon======= "<< "\033[0m" << std::endl;
        return new sdMoon(conf); }},
        {"sdPie", [](const Config &conf)
         { std::cout << "\033[35m=======sdPie======= "<< "\033[0m" << std::endl;
        return new sdPie(conf); }},
        {"sdPie2", [](const Config &conf)
         { std::cout << "\033[35m=======sdPie2======= "<< "\033[0m" << std::endl;
        return new sdPie2(conf); }},
        {"sdArc", [](const Config &conf)
         { std::cout << "\033[35m=======sdArc======= "<< "\033[0m" << std::endl;
        return new sdArc(conf); }}};

    SweptVolumeManager() = delete;
    SweptVolumeManager(const Config &conf) : config(conf)
    {
        eps = conf.eps;
        kernelsize = config.kernel_size;
        kernel_count = config.kernel_yaw_num;
        swept_cal.reset(new sw_calculate); 
    }
    ~SweptVolumeManager()
    {
        delete current_robot_shape;
        delete[] map_kernel;
    }
    typedef shared_ptr<SweptVolumeManager> Ptr;

public:
    /**
     * 设置map_kernel
     */
    inline void setMapKernel(uint8_t *mk, int mx, int my, int mz)
    {
        map_kernel = mk;
        map_Xsize = mx;
        map_Ysize = my;
        map_Zsize = mz;
    }


    inline void getInitSeedsforSwept()
    {
        srand(100);
        igl::per_face_normals(current_robot_shape->V, current_robot_shape->F, Eigen::Vector3d(0.0, 0.0, -1.0).normalized(), N);
        igl::random_points_on_mesh(num_seeds, current_robot_shape->V, current_robot_shape->F, B, I, X); 
        init_times.clear();
        init_voxels.clear();
        init_points.clear();
        init_times.push_back(0.0);
        init_times.push_back(0.0);
        init_voxels.push_back(Eigen::RowVector3i(0, 0, 0));
        for (int i = 0; i < X.rows(); i++)
        {                                    
            Eigen::RowVector3d P = X.row(i); 
            Eigen::Matrix3d VRt, Rt;
            Eigen::Vector3d xt, vt;
            Eigen::RowVector3d pos, point_velocity, normal;

            Eigen::RowVector3d candidate;
            for (double t = t_min; t <= t_max; t = t + 0.1)
            {
                getStateOnTrajStamp(t, xt, vt, Rt, VRt);
                pos = (Rt * P.transpose()).transpose() + Eigen::RowVector3d(xt(0),xt(1),0);                                                  
                point_velocity = (VRt * P.transpose()).transpose() + vt.transpose() + ( Rt * P.transpose()).transpose();
                point_velocity.normalize();
                normal = (Rt * N.row(I(i)).transpose()).transpose();
                normal.normalize();
                if ((fabs(normal.dot(point_velocity)) < 0.05) || (normal.dot(point_velocity) < 0.0 && t == t_min) || (normal.dot(point_velocity) > 0.0 && t == t_max))
                {
                    candidate = pos + iso * normal;
                    init_points.push_back(candidate);
                    init_times.push_back(t);
                    minx = std::min(minx, candidate(0));
                    miny = std::min(miny, candidate(1));
                    minz = std::min(minz, candidate(2));
                }
            }
        }
        p0(0) = minx;
        p0(1) = miny;
        p0(2) = minz;
        Eigen::RowVector3d this_point;
        int ix, iy, iz;
        for (int s = 0; s < init_points.size(); s++)
        {
            this_point = init_points[s];                   
            ix = std::floor((this_point[0] - minx) / eps); 
            iy = std::floor((this_point[1] - miny) / eps);
            iz = std::floor((this_point[2] - minz) / eps);
            init_voxels.push_back(Eigen::RowVector3i(ix, iy, iz)); 
        }
        std::cout << "======Starting continuation with====== " << init_voxels.size() << " seeds." << std::endl;
        std::cout << "======Starting continuation with====== " << init_times.size() << " init_times." << std::endl;
    }


    inline void calculateSwept(Eigen::MatrixXd &U_, Eigen::MatrixXi &G_)
    {
        double t_min=0.0;
        double t_max=traj.getTotalDuration();
        swept_cal->updatebounds(t_min, t_max);
        getInitSeedsforSwept();
        std::cout << "t_min:" << t_min << "t_max:" << t_max << std::endl;
        std::cout << "======after init ====== " << std::endl;
        std::cout << "=========use analytic method to calculate swept volume==========" << std::endl;
        swept_cal->calculation(p0, init_voxels, init_times, scalarFunc, eps, 1000000);
        std::cout << "======after calculation ====== " << std::endl;
        swept_cal->getmesh(U_, G_);
        vis->visMesh("sweptmesh2D", "sweptedge", U_, G_, 0.1, vis::Color::blue, 1);
        F_sw_obj=G_;
        V_sw_obj=U_;
    }


    inline void init(ros::NodeHandle &nh, Config &conf)
    {
        vis.reset(new vis::Visualization(nh));
        flatness.reset(conf.vehicleMass, conf.gravAcc, conf.horizDrag,
                       conf.vertDrag, conf.parasDrag, conf.speedEps);

        momentum = conf.momentum;
        initShape(conf);
    }


    inline void initShape(Config &conf)
    {
        std::string inputdata = conf.inputdata;
        size_t start = inputdata.find_last_of("/") + 1; 
        size_t end = inputdata.find_last_of(".");      
        std::string shapetype = inputdata.substr(start, end - start);

        if (shapeConstructors.count(shapetype) > 0)
        {
            std::cout << "\033[35m======= Analytic Shape now======= "
                      << "\033[0m" << std::endl;
            current_robot_shape = shapeConstructors[shapetype](conf);
        }
        else
        {
            Eigen::MatrixX2d rect = Eigen::MatrixX2d::Zero(4, 2);
            rect.row(0) = Eigen::Vector2d(6, -0.1);
            rect.row(1) = Eigen::Vector2d(6, 0.1);
            rect.row(2) = Eigen::Vector2d(-6, 0.1);
            rect.row(3) = Eigen::Vector2d(-6, -0.1);
            current_robot_shape = new Polygon(conf, rect);
            std::cout << "===============================6874" << std::endl;
        }
    }


    inline void updateTraj(const Trajectory<TRAJ_ORDER> &new_traj)
    {
        this->traj = new_traj;
        double td = traj.getTotalDuration();
        if (td < 3 * 1e2)
        {
            traj_duration = td;
            t_max = td; 
        }
    }


    inline void initShapeByString(std::string input_shape, Config &conf)
    {
        std::string inputdata = input_shape;
        size_t start = inputdata.find_last_of("/") + 1; 
        size_t end = inputdata.find_last_of(".");      
        std::string shapetype = inputdata.substr(start, end - start);

        if (shapeConstructors.count(shapetype) > 0)
        {
            std::cout << "\033[35m======= Analytic Shape now======= "
                      << "\033[0m" << std::endl;
            current_robot_shape = shapeConstructors[shapetype](conf);
        }
        else
        {
            Eigen::MatrixX2d rect = Eigen::MatrixX2d::Zero(4,2);
            rect.row(0) = Eigen::Vector2d(6,-0.1);
            rect.row(1) = Eigen::Vector2d(6,0.1);
            rect.row(2) = Eigen::Vector2d(-6,0.1);
            rect.row(3) = Eigen::Vector2d(-6,-0.1);
            current_robot_shape = new Polygon(conf, rect); 
            std::cout<<"===============================6874"<<std::endl;
        }
    }

    inline void clearCurrentShape(){
        delete current_robot_shape;
    }


    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt)
    {
        xt = traj.getPos(time_stamp);
        vt = traj.getVel(time_stamp);

        double yaw = xt(2);
        Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        VRt = Eigen::Matrix3d::Identity();
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        VRt(0, 0) = -syaw;
        VRt(0, 1) = -cyaw;
        VRt(1, 0) = cyaw;
        VRt(1, 1) = -syaw;
    }

    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt,
                                    Eigen::Matrix3d &St,
                                    Eigen::Matrix3d &dSt)
    {
        assert((time_stamp > traj_duration) && ("[getStateOnTrajStamp] 传入的time_stamp参数大于轨迹的总时长。"));
        xt = traj.getPos(time_stamp);
        vt = traj.getVel(time_stamp);

        double yaw = xt(2);
        Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        VRt = Eigen::Matrix3d::Identity();
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        VRt(0, 0) = -syaw;
        VRt(0, 1) = -cyaw;
        VRt(1, 0) = cyaw;
        VRt(1, 1) = -syaw;

        St = getScale(time_stamp);
        dSt = getDotScale(time_stamp);
    }


    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt)
    {
 
        xt = traj.getPos(time_stamp);

        double yaw = xt(2);
        Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    inline void getStateOnTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &St)
    {
        assert((time_stamp > traj_duration) && ("[getStateOnTrajStamp] 传入的time_stamp参数大于轨迹的总时长。"));
        xt = traj.getPos(time_stamp);

        double yaw = xt(2);
        Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        St = getScale(time_stamp);
    }

    /**
     * 获得轨迹上具体某一时刻的机器人尺寸因子
     * @param time_stamp 时刻
     * @return time_stamp 时刻机器人尺寸因子
     * @attention 当前返回单位矩阵，需要按需修改,注意奇异点问题Scale缩放分母除以0，求inverse问题
     */
    inline Eigen::Matrix3d getScale(const double t)
    {
        // Eigen::Matrix3d St = Eigen::Matrix3d::Identity();
        // St<<0.8+sin(1.5*t-1.0)*0.6,           0,            0,
        //     0,                 sin(1.8*t)*0.4+0.8,      0,
        //     0,           0,                        1;
        // return St;
        return Eigen::Matrix3d::Identity();
    }

    /**
     * 获得轨迹上具体某一时刻的机器人尺寸因子变化导数
     * @param time_stamp 时刻
     * @return time_stamp 时刻机器人尺寸因子
     * @attention 当前返回0矩阵，需要按需修改
     */
    inline Eigen::Matrix3d getDotScale(const double t) 
    {
        Eigen::Matrix3d St = Matrix3d::Zero();
        // St<<1.5*cos(1.5*t-1.0)*0.6,           0,            0,
        //     0,  1.8*cos(1.8*t)*0.4,                     0,
        //     0,           0,                     0;
        return St;
    }


    inline Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                                      const Eigen::Vector3d &xt,
                                      const Eigen::Matrix3d &Rt)
    {
        return (Rt.transpose() * (pos_eva - xt));
    }

    inline Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                                      const Eigen::Vector3d &xt,
                                      const Eigen::Matrix3d &Rt,
                                      const Eigen::Matrix3d &St)
    {
        // return (pos_eva - xt);
        return (Rt.transpose() * St.inverse() * (pos_eva - xt));
    }


    template <bool withscale = false>
    inline double choiceTInit(const Eigen::Vector3d &pos_eva, double dt)
    {

        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;

        double min_dis = 1e9;
        double dis = 1e9;
        double time_seed = 0.0;

        int pricision_layers = 4;
        int current_layer = 1;

        double loop_terminal = traj_duration;

        double t = 0.0;
        while (current_layer <= pricision_layers)
        {
            if (current_layer == 1)
            {
                t = 0.0;
            }
            if (current_layer > 1)
            {
                t = std::max(0.0, time_seed - 10 * dt);
                loop_terminal = std::min(traj_duration, time_seed + 10 * dt);
            }

            for (; t <= loop_terminal; t += dt)
            {
                getStateOnTrajStamp(t, xt, Rt);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                if (dis < min_dis)
                {
                    time_seed = t;
                    min_dis = dis;
                }
            }
            dt *= 0.1;
            current_layer += 1;
        }
        return time_seed;
    }


    template <bool withscale = false>
    inline double choiceTInit(const Eigen::Vector3d &pos_eva, const double dt, vector<double> &range_l, vector<double> &range_r, vector<double> &range_ts)
    {
        double rough_dt = 0.2;

        double mindis = 1e9;
        double range_mindis = 1e9;
        double dis = 1e9;
        double time_seed = 0.0;
        double range_time_seed = 0.0;

        double dis_rough = 1e9;
        double min_dis_rough = 1e9;
        double second_dis_rough = 1e9;
        double tou_lb = 0;
        double tou_ub = 0;
        double safty_hor_inf = 2 * config.safety_hor + 0.1;

        range_l.clear();
        range_r.clear();
        range_ts.clear();
        bool flag_in_range = false;

        Eigen::Vector3d xt, vt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            for (double t = 0; t < traj_duration; t += rough_dt)
            {
                getStateOnTrajStamp(t, xt, Rt);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }

                if (dis < safty_hor_inf)
                {
                    if (flag_in_range == false)
                    {
                        flag_in_range = true;
                        range_mindis = dis;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else
                    {
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else
                {
                    if (flag_in_range == true)
                    {
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for (size_t i = 0; i < N; i++)
            {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for (double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
                    if (dis < range_mindis)
                    {
                        range_time_seed = t;
                        range_mindis = dis;
                    }
                }
                range_ts.push_back(range_time_seed);
            }

            return time_seed;
        }

        else
        {
            Eigen::Matrix3d St;
            for (double t = 0; t < traj_duration; t += dt) 
            {
                getStateOnTrajStamp(t, xt, Rt, St);
                dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
                if (dis < mindis)
                {
                    time_seed = t;
                    mindis = dis;
                }

                if (dis < safty_hor_inf)
                {
                    if (flag_in_range == false)
                    {
                        flag_in_range = true;
                        tou_lb = std::max(0.0, t - rough_dt);
                        tou_ub = t;
                    }
                    else
                    {
                        tou_ub = std::min(traj_duration, t + rough_dt);
                    }
                }
                else
                {
                    if (flag_in_range == true)
                    {
                        flag_in_range = false;
                        tou_ub = std::min(traj_duration, t + rough_dt);
                        range_l.push_back(tou_lb);
                        range_r.push_back(tou_ub);
                    }
                }
            }

            int N = range_l.size();
            for (size_t i = 0; i < N; i++)
            {
                tou_lb = range_l[i];
                tou_ub = range_r[i];
                for (double t = tou_lb; t < tou_ub; t += dt)
                {
                    getStateOnTrajStamp(t, xt, Rt, St);
                    dis = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
                    if (dis < range_mindis)
                    {
                        range_time_seed = t;
                        range_mindis = dis;
                    }
                }
                range_ts.push_back(range_time_seed);
            }
            return time_seed;
        }
    }

    inline double getSDFWithGradWhenRobotAtState(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                 const Eigen::Matrix3d &Rt, const Eigen::Matrix3d &St)
    {
        return current_robot_shape->getSDFwithGrad1(posEva2Rel(pos_eva, xt, Rt, St), grad_prel);
    }


    inline double getSDFWithGradWhenRobotAtState(const Eigen::Vector3d &pos_eva, Eigen::Vector3d &grad_prel, const Eigen::Vector3d &xt,
                                                 const Eigen::Matrix3d &Rt)
    {
        return current_robot_shape->getSDFwithGrad1(posEva2Rel(pos_eva, xt, Rt), grad_prel);
    }


    template <bool withscale = false>
    inline double getSDFAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlySDF(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    template <bool withscale = false>
    inline double getSDFAtTimeStamp_igl(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            xt(2)=0;
            return current_robot_shape->getonlySDF_igl(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            xt(2)=0;
            return current_robot_shape->getonlySDF_igl(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }

    template <bool withscale = false>
    inline Eigen::Vector3d getGradPrelAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {
        Eigen::Vector3d xt;
        Eigen::Matrix3d Rt;
        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, Rt);
            return current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt));
        }
        else
        {
            Eigen::Matrix3d St;
            getStateOnTrajStamp(time_stamp, xt, Rt, St);
            return current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt, St));
        }
    }


    template <bool withscale = false>
    inline double getSDF_DOTAtTimeStamp(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {

        double t1 = std::max(0.0, time_stamp - 0.000001);
        double t2 = std::min(traj_duration, time_stamp + 0.000001);
        double sdf1 = getSDFAtTimeStamp<withscale>(pos_eva, t1);
        double sdf2 = getSDFAtTimeStamp<withscale>(pos_eva, t2);
        return (sdf2 - sdf1) * 500000;

        Eigen::Vector3d sdf_grad1;
        Eigen::Vector3d point_velocity;
        Eigen::Vector3d xt, vt;
        Eigen::Matrix3d Rt, VRt, Rt_trans;

        if (!withscale)
        {
            getStateOnTrajStamp(time_stamp, xt, vt, Rt, VRt);
            Rt_trans = Rt.transpose();
            sdf_grad1 = current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt));
            point_velocity = -(Rt_trans * vt + VRt.transpose() * (pos_eva - xt));
        }
        else
        {
            Eigen::Matrix3d St, dSt;
            getStateOnTrajStamp(time_stamp, xt, vt, Rt, VRt, St, dSt);
            Rt_trans = Rt.transpose();
            sdf_grad1 = current_robot_shape->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt, St)); 
            point_velocity = -(Rt_trans * St.inverse() * vt + VRt.transpose() * St.inverse() * (pos_eva - xt) + Rt_trans * St.inverse() * dSt * St.inverse() * (pos_eva - xt));
        } 
        return sdf_grad1.dot(point_velocity);
    }


    template <bool withscale = false>
    inline double getSDF_DOTAtTimeStamp_igl(const Eigen::Vector3d &pos_eva, const double &time_stamp)
    {

        double t1 = std::max(0.0, time_stamp - 0.000001);
        double t2 = std::min(traj_duration, time_stamp + 0.000001);
        double sdf1 = getSDFAtTimeStamp_igl<withscale>(pos_eva, t1); 
        double sdf2 = getSDFAtTimeStamp_igl<withscale>(pos_eva, t2);
        return (sdf2 - sdf1) * 500000;
    }


    template <bool set_ts = false, bool need_grad_prel = true>
    inline double getSDFofSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel)
    {
        Eigen::Vector3d xt, vt;
        double ts = time_seed_f;
        double t_star;
        double sdf_star;
        double dtime = 0.15;
        if (set_ts == false)
        {
            ts = choiceTInit<useScale>(pos_eva, dtime); 
        }
        double tmin_ = std::max(0.0, ts - 3.4);           
        double tmax_ = std::min(ts + 3.4, traj_duration); 
        gradientDescent(momentum, tmin_, tmax_, ts, sdf_star, t_star, pos_eva);
       
        if (need_grad_prel)
        {
            grad_prel = getGradPrelAtTimeStamp<useScale>(pos_eva, t_star);
        }
        time_seed_f = t_star;
        return sdf_star;
    }


    template <bool need_grad_prel = true>
    inline double getSDFofSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel, bool set_ts, bool debug = false)
    {
        Eigen::Vector3d xt, vt;
        double ts = time_seed_f;
        double t_star;
        double sdf_star = 1e4;
        double min_sdf_star = 1e4;
        double dtime = 0.02;

        std::vector<double> range_l;
        std::vector<double> range_r;
        std::vector<double> range_ts;

        if (set_ts == false)
        {
            ts = choiceTInit<useScale>(pos_eva, dtime, range_l, range_r, range_ts);
        }

        int range_count = range_l.size();
        if (debug)
        {
            std::cout << "=============================in rangecount=============================" << std::endl;
            std::cout << "=============================range_count=============================" << range_count << std::endl;
        }
        for (int i = 0; i < range_count; i++)
        {
            double temp_ts = range_ts[i];
            double tmin_ = std::max(0.0, range_l[i]);           
            double tmax_ = std::min(range_r[i], traj_duration); 
            gradientDescent(momentum, tmin_, tmax_, temp_ts, sdf_star, t_star, pos_eva, debug);
           
            if (sdf_star < min_sdf_star)
            {

                min_sdf_star = sdf_star;
                time_seed_f = t_star;
                if (need_grad_prel)
                {
                    grad_prel = getGradPrelAtTimeStamp<useScale>(pos_eva, t_star);
                }
            }
        }
        return min_sdf_star;
    }


    template <bool need_grad_prel = true>
    inline double getTrueSDFofSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel, bool set_ts, bool debug = false)
    {
        double argmin_dis = 0.0;
        argmin_dis = getSDFofSweptVolume<false, true>(pos_eva, time_seed_f, grad_prel);
        if (argmin_dis > 0)
        { // outside case
            return argmin_dis;
        }

        // USE GSIP SOLVER TO SOLVE
        double r0 = 10;

        Eigen::Vector3d vel = traj.getVel(time_seed_f);
        if (vel.norm() < 0.01)
        {
            if (time_seed_f < 0.1)
            {
                for (double t_scan = time_seed_f; t_scan <= traj_duration; t_scan += 0.1)
                {
                    vel = traj.getVel(t_scan);
                    if (vel.norm() >= 0.01)
                    {
                        break;
                    }
                }
            }
            else if (time_seed_f > traj_duration - 0.1)
            {
                for (double t_scan = time_seed_f; t_scan >= 0; t_scan -= 0.1)
                {
                    vel = traj.getVel(t_scan);
                    if (vel.norm() >= 0.01)
                    {
                        break;
                    }
                }
            }
        }
        Eigen::Vector2d vel_2d = vel.head(2);
        Eigen::Vector2d obs_2d = pos_eva.head(2);
        SampleSet2D Y_threadsafe;
        Y_threadsafe.initSet(vel_2d, obs_2d, r0);

        int Y_size = 0;
        Eigen::Vector2d yk;
        Eigen::Vector3d yk_3d;
        CircleCoord2D yk_star;
        double r_star;
        double max_g;
        double cur_g;
        double real_t_star;
        int iter = 1;

        while (true)
        {
            // STEP1 calc r* under Yk
            max_g = -100000;
            std::vector<CircleCoord2D> elements = Y_threadsafe.getElements();
            Y_size = elements.size();

            for (int i = 0; i < Y_size; i++)
            {
                yk = Y_threadsafe.getElementPos(elements[i]);
                yk_3d = Eigen::Vector3d(yk[0], yk[1], 0.0);
                cur_g = getSDFofSweptVolume<false, true>(yk_3d, time_seed_f, grad_prel);

                if (cur_g > max_g)
                {
                    max_g = cur_g;
                    real_t_star = time_seed_f;
                    yk_star = elements[i];
                }
            }
            
            r_star = Y_threadsafe.r - max_g;
            Y_threadsafe.setCircleRadius(r_star);

        
            if (iter > 8)
            {
                // cout << "\033[32m !!!!!!!!!possible error!!!!!!!!! \033[0m" << endl;
                break;
            }

            if (abs(max_g) < 0.1)
            {
                break;
            }

            // STEP2 calc y* under r*
            Y_threadsafe.expandSet(2, yk_star.theta);
            iter++;
        }

        Eigen::Vector2d cor_point = yk_star.getPosition(Y_threadsafe.center, r_star);
        Eigen::Vector3d cor_point_3d = Eigen::Vector3d(cor_point[0], cor_point[1], 0.0);
        grad_prel = (cor_point_3d - pos_eva);
        grad_prel(2) = 0.0;
        grad_prel.normalize();
        time_seed_f = real_t_star;
        return -r_star;
    }


    inline void setTrajStamp(const double &traj_stamp)
    {
        trajStamp = traj_stamp;
    }

    bool isIndexValid(const int ix, const int iy, const int iz) const
    {
    }
    bool isIndexOccupied(int ix, int iy, int iz)
    {
    }
  
    template <bool useByteKernel = true>
    inline bool kernelConv(int kernel_i, const Eigen::Vector3i &ind)
    {
        int kernel_size = config.kernel_size;
        int side_size = (kernel_size - 1) / 2;

        int ind_x = ind(0);
        int ind_y = ind(1);

        if (useByteKernel == false)
        {
            int a, b, c;
            for (int off_x = -side_size; off_x <= side_size; off_x++)
            {
                for (int off_y = -side_size; off_y <= side_size; off_y++)
                {
                    if (isIndexValid(off_x + ind_x, off_y + ind_y, 0) == false)
                    {
                        continue;
                    }                                                                                  
                    a = off_x + side_size;                                                             
                    b = off_y + side_size;                                                             
                    if (current_robot_shape->shape_kernels[kernel_i].map[a * kernelsize + b] == false) 
                    {
                        continue;
                    } 
                    if (isIndexOccupied(off_x + ind_x, off_y + ind_y, 0) == false)
                    {
                        continue;
                    }             
                    return false; 
                }
            }
            return true; 
        }

        else
        {
            int half_size = (kernel_size - 1) / 2;
            int bytes_len_of_last_dim = (kernel_size + 7) / 8;

            int map_Xsize_inflated = map_Xsize + 2 * half_size;
            int map_Ysize_inflated = map_Ysize + 2 * half_size;
            int map_last_dim_bytes_len = (map_Ysize_inflated + 7) / 8;

            int box_min_x = ind[0]; 
            int box_min_y = ind[1];
            for (int i = 0; i < kernel_size; i++)
            { 
                int startByteIdx = (box_min_x + i) * map_last_dim_bytes_len + (box_min_y / 8);
                int offsetIdx = box_min_y % 8;
                for (int j = 0; j < bytes_len_of_last_dim; j++)
                { 
                    int kernelByteIdx = i * bytes_len_of_last_dim + j;
                    uint8_t block = (map_kernel[startByteIdx + j] << offsetIdx) | (map_kernel[startByteIdx + j + 1] >> (8 - offsetIdx));
                   
                    uint8_t res = current_robot_shape->byte_shape_kernels[kernel_i].map[kernelByteIdx] & block;
                    
                    if (res)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
    //

    inline bool visit_kernels_by_distance(int &returni, int start_x, const Eigen::Vector3i &ind, int maxdeepth = 10) 
    {

        int count = kernel_count;           
        vector<bool> visited(count, false); 
        queue<int> q;                       
        q.push(start_x);
        visited[start_x] = true;
        vector<int> directions = {-1, 1}; 
        int deep = 0;
        bool conv_result = false;
        bool result = false;
   
        while (!q.empty())
        {
            deep++;
            int x = q.front();
            q.pop();
            conv_result = kernelConv<true>(x, ind);
     
            if (conv_result)
            {
                returni = x;
                result = true;
                break;
            }
            
            for (auto &dir : directions)
            {
                int nx = x + dir;
           
                if (nx < 0)
                {
                    nx = count - 1;
                }
                if (nx >= count)
                {
                    nx = 0;
                }
                if (visited[nx])
                {
                    continue;
                }
                visited[nx] = true;
                q.push(nx);
            }
            if (deep > maxdeepth) 
            {
                result = false;
                break;
            }
        }
       
        return result;
    }

    inline bool checkKernelValue(double father_yaw, double &child_yaw, const Eigen::Vector3i &ind)
    {
        int father_i = int(kernel_count * ((father_yaw + pi) / (2 * pi)));
        int ret_i = father_i;
        bool res = false; 
        if (visit_kernels_by_distance(ret_i, father_i, ind))
        {
            child_yaw = 2 * pi * (ret_i) / kernel_count - pi;
            return true;
        }
        return false;
    }

    inline bool checkSubSWCollision(const Eigen::Vector3d &father_state, const Eigen::Vector3d &child_state, const vector<Eigen::Vector2d> &aabb_points)
    {

        Eigen::Vector3d linear_state = Eigen::Vector3d::Zero();
        Eigen::Vector3d pos_eva = Eigen::Vector3d::Zero();
        Eigen::Vector3d vt = Eigen::Vector3d::Zero();
        vt = child_state - father_state;
        vt(2) = 0.0;

        double yaw = 0.0;
        double dt = 0.02;
        double min_sdf = 1e9;
        double temp_sdf = 1e8;
        double sub_ts = 0.0;
        for (size_t i = 0; i < aabb_points.size(); i++)
        {
            pos_eva.head(2) = aabb_points[i];
            min_sdf = 1e9;
            temp_sdf = 1e8;
            sub_ts = 0.0;
            for (double kt = 0.0; kt <= 1.0; kt += dt)
            {
                linear_state = kt * child_state + (1 - kt) * father_state;
                yaw = linear_state(2);
                linear_state(2) = 0.0;
                Eigen::Matrix3d Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                temp_sdf = current_robot_shape->getonlySDF(posEva2Rel(pos_eva, linear_state, Rt));
                if (temp_sdf < min_sdf)
                {
                    min_sdf = temp_sdf;
                    sub_ts = kt;
                }
                if (min_sdf < 0)
                {
                    return false;
                }
            }
        }

        return true;
    }


    inline void process(const Trajectory<TRAJ_ORDER> &traj)
    {

        vis->visPolytope(current_robot_shape->mesh, "Polymeshoriginal", "Polyedgeoriginal", false, 1, 121, vis::steelblue, 1);

        Eigen::Vector3d trans;
        if(traj.getTotalDuration()>0)
        {
        Eigen::Matrix3d Rttemp,Sttemp;
        getStateOnTrajStamp(0, trans, Rttemp,Sttemp);
        trans(2)=0;
        current_robot_shape->setStartEnd(Rttemp, trans,Sttemp,true);
        getStateOnTrajStamp(traj.getTotalDuration(), trans, Rttemp,Sttemp);
          current_robot_shape->setStartEnd(Rttemp, trans,Sttemp,false);
        }

        const double delta = ros::Time::now().toSec() - trajStamp;
        if (delta > 0.0 && delta < traj.getTotalDuration())
        {
            Eigen::Matrix3d St;
            trans = traj.getPos(delta);
            St = getScale(delta);

            double yaw = trans(2);
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
           
            trans(2) = 0.0;
            current_robot_shape->Transform(Rt, trans, St);
            vis->visPolytope(current_robot_shape->mesh_var, "Polymesh", "Polyedge", true, 1,3212,vis::Color::lightyellow);
            vis->visPolytopeOffset(current_robot_shape->mesh_var, "Polymeshoffset", "Polyedgeoffset", true, 2,3212,vis::Color::lightyellow);
            vis->visABall(current_robot_shape->interior_var+Eigen::Vector3d(0,0,10), 0.15 * 2, "interior", vis::Color::blue);
        }
    }


    inline void gradientDescent(double momentum, double t_min, double t_max,
                                const double x0, double &fx, double &x, const Eigen::Vector3d &pos_eva, bool debug = false)
    {
        assert((t_max > 0) && (t_max < 1000) && " in gradient descent,t_max must > 0 and t_max <100");
        assert((t_min >= 0) && " in gradient descent,t_min must >=0");
        assert((momentum >= 0) && (momentum <= 1) && "momentum must between 0~1");

        int max_iter = 1000;
        double alpha = 0.01; 
        double tau = alpha;
        double g = 0.0; 
        double tol = 1e-16;
        double min = t_min;
        double max = t_max;
        double xmin = x0; 
        double xmax = x0;
        x = x0;
        if (debug)
        {
            cout << "\033[32m ==================gradientDescent==================:\033[0m" << endl;
            cout << "t_min:" << t_min << " t_max:" << t_max << endl;
        }

        double projection = 0;
        double change = 0;

        double prev_x = 10000000.0;
        int iter = 0;
        bool stop = false;
        double x_candidate;
        double fx_candidate;

        g = 100.0;
        while (iter < max_iter && !stop && abs(x - prev_x) > tol)
        {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);

            if (iter == 0)
            {
                fx = getSDFAtTimeStamp<useScale>(pos_eva, x);
            }
            g = getSDF_DOTAtTimeStamp<useScale>(pos_eva, x);

            tau = alpha;
            prev_x = x;
            for (int div = 1; div < 30; div++) 
            {
                iter = iter + 1;
                assert(iter < max_iter && "不满足iter < max_iter");
                projection = x;
                g = getSDF_DOTAtTimeStamp<useScale>(pos_eva, projection);
                change = -tau * ((int)(g > 0) - (g < 0));
                x_candidate = x + change;
                x_candidate = std::max(std::min(x_candidate, t_max), t_min);
                fx_candidate = getSDFAtTimeStamp<useScale>(pos_eva, x_candidate);
                if (debug)
                {
                    cout << "\033[32m iter:\033[0m" << iter << endl;
                    cout << "\033[32m x_candidate:\033[0m" << x_candidate << endl;
                    cout << "\033[32m gradient:\033[0m" << g << endl;
                    cout << "\033[32m fx_candidate:\033[0m" << fx_candidate << endl;
                }
                if ((fx_candidate - fx) < 0)
                {
                    x = x_candidate;
                    fx = fx_candidate;
                    break;
                }
                tau = 0.5 * tau;
                if (div == 29)
                {
                    stop = true;
                }
            }
        }
    }


    void gradient_descent(double momentum, double t_min, double t_max, const double x0,
                          double &fx, double &x, const Eigen::Vector3d &pos_eva, std::vector<double> &intervals,
                          std::vector<double> &values, std::vector<double> &minima)
    {
        assert((t_max > 0) && (t_max < 100) && " in gradient descent,t_max must > 0 and t_max <100");
        assert((t_min >= 0) && " in gradient descent,t_min must >=0");
        assert((momentum >= 0) && (momentum <= 1) && "momentum must between 0~1");
        int max_iter = 1000;
        double alpha = 0.02; 
        double tau = alpha;
        double g = 0.0; 
        double min = t_min;
        double max = t_max;
        double tol = 1e-3;
        double xmin = x0; 
        double xmax = x0;
        x = x0;

        double projection = 0;
        double change = 0;

        double prev_x = 10000000.0;
        int iter = 0;
        bool stop = false;
        double x_candidate, fx_candidate;
        g = 100.0;

        int in_existing_interval = -1;
     
        while (iter < max_iter && !stop && abs(x - prev_x) > tol) 
        {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);

            // t1 = GNOW;
            for (int mm = 0; mm < (intervals.size() / 2); mm++)
            {
                if ((x >= (intervals[2 * mm] - 1e-6)) && (x <= (intervals[2 * mm + 1] + 1e-6)))
                {
                    fx = values[mm];
                    x = minima[mm];
                    in_existing_interval = mm;
                    break;

                }
            }
            if (in_existing_interval > -1)
            {
                break; 
            }

            if (iter == 0)
            {
                fx = getSDFAtTimeStamp_igl<useScale>(pos_eva, x);
            }
            g = getSDF_DOTAtTimeStamp_igl<useScale>(pos_eva, x);
            tau = alpha;
            prev_x = x;
            for (int div = 1; div < 10; div++) 
            {
                iter = iter + 1;
                assert(iter < max_iter && "不满足iter < max_iter");
                projection = x + momentum * change;
                g = getSDF_DOTAtTimeStamp_igl<useScale>(pos_eva, projection);
                change = momentum * change - tau * ((double)(g > 0) - (g < 0));
                x_candidate = x + change;
                x_candidate = std::max(std::min(x_candidate, t_max), t_min); 
                fx_candidate = getSDFAtTimeStamp_igl<useScale>(pos_eva, x_candidate);
                if ((fx_candidate - fx) < (0.5 * (x_candidate - x) * g)) 
                {
                    x = x_candidate;
                    fx = fx_candidate;
                    break;
                }
                tau = 0.5 * tau;
                if (div == 9)
                {
                    stop = true; 
                }
            }
        }

        if (in_existing_interval == -1) 
        {
            intervals.push_back(xmin);
            intervals.push_back(xmax);
            values.push_back(fx);
            minima.push_back(x);
        }
        else
        {

            intervals[2 * in_existing_interval] = std::min(intervals[2 * in_existing_interval], xmin);
            intervals[2 * in_existing_interval + 1] = std::max(intervals[2 * in_existing_interval + 1], xmax);
        }
    }


        std::function<double(const Eigen::RowVector3d &, double &, std::vector<std::vector<double>> &,
                         std::vector<std::vector<double>> &, std::vector<std::vector<double>> &)>
        scalarFunc = [&](const Eigen::RowVector3d &P, double &time_seed, 
                         std::vector<std::vector<double>> &intervals, std::vector<std::vector<double>> &values,
                         std::vector<std::vector<double>> &minima) -> double 
    {
        Eigen::RowVector3d running_closest_point = current_robot_shape->V.row(0); 
        double running_sign = 1.0;
        double distance, seed;
        if (intervals.size() == 0)
        {
            std::vector<double> temp_interval;
            temp_interval.resize(0);
            intervals.push_back(temp_interval);
            values.push_back(temp_interval);
            minima.push_back(temp_interval);
        }
        gradient_descent(0, std::max(time_seed - 0.1, t_min), std::min(time_seed + 0.1, t_max), time_seed, distance, seed, P.transpose(), intervals[0], values[0], minima[0]); 
        time_seed = seed;                                                                                                                                                     
        return distance;
    };
    
};
#endif
