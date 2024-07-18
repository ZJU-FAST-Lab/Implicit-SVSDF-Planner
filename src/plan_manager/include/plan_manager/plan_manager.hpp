#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <planner_algorithm/front_end_Astar.hpp>
#include <planner_algorithm/back_end_optimizer.hpp>
#include <planner_algorithm/mid_end.hpp>
#include <utils/se3_state.hpp>
#include <map_manager/PCSmap_manager.h>
#include <swept_volume/sw_manager.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>


using namespace Eigen;


class PlannerManager
{
  public:
    PlannerManager(){};
    ~PlannerManager(){};

    void clear();
    double bdx;
    double bdy;
    double bdz;

    void init( ros::NodeHandle& nh, ros::NodeHandle& nh_prev );
    void mapRcvCallBack(const std_msgs::Empty& msg);
    void iniPoseRcvCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void settingRcvCallBack(const std_msgs::Int8MultiArray& msg);
    void targetRcvCallBack(const geometry_msgs::PoseStamped& msg);
    void viscallback(const ros::TimerEvent & event);
    // vis
    void reShowTraj( const std_msgs::Empty::Ptr msg);

    void LoadStartEnd();


    double getMinSVSDF();
    void autoBenchMark();


    bool generatePath( Vector3d start, Vector3d end );


    void generateTraj( vector<Vector3d> path );



    void process();
    


    // params
    double traj_parlength{1.0};   

    // stream control
    #define STEP_NOPOINT      0
    #define STEP_HAVE_START   1
    #define STEP_HAVE_TARGET  2

    int step_state = STEP_NOPOINT;

    //odom
    nav_msgs::Odometry recent_odom;
    bool has_odom;

    // setting
    int current_front_end;

    Eigen::Vector3d Start_record;
    Eigen::Vector3d End_record;
    bool Loadstartandend{false};


    // planning
    vector<Vector3d> recent_path;
    vector<SE3State> recent_se3_path;
    vector<double> recent_se3_path_roll;
    vector<double> recent_se3_path_pitch;
    Vector3d start_pos;
    Vector3d end_pos;
    Trajectory<TRAJ_ORDER> recent_traj;
    int recent_ret;

    // ros
    ros::Subscriber target_sub;       
    ros::Subscriber odom_sub;         
    ros::Subscriber rcvmap_signal_sub;  
    ros::Subscriber setting_sub;      
    ros::Subscriber rs;               

    ros::Publisher traj_pub;

    // visualization
    Visualization::Ptr visulizer;

    // map
    PCSmapManager::Ptr pcsmap_manager;

    // front_end
    AstarPathSearcher::Ptr    astar_searcher;

    // back_end
    OriTraj::Ptr ori_traj_generator;
    TrajOptimizer::Ptr minco_traj_optimizer;
    SweptVolumeManager::Ptr sv_manager;
    Config config;


  
  public:
    typedef shared_ptr<PlannerManager> Ptr;
};


class DebugAssistant
{
  public:
    DebugAssistant(){};
    ~DebugAssistant(){};

    void init(ros::NodeHandle& nh, PlannerManager::Ptr pm);
    void debugMsgcallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  private:
    ros::Subscriber debug_sub;
    PlannerManager::Ptr planner_manager;
  public:
    typedef std::shared_ptr<DebugAssistant> Ptr;
};


PlannerManager::Ptr planner_manager;
DebugAssistant::Ptr debug_assistant;

#endif