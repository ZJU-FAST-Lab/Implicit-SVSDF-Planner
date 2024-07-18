#include <plan_manager/plan_manager.hpp>
#include <utils/Visualization.hpp>
#include <iomanip>
#include <limits>
using namespace std;
using namespace ros;
using namespace Eigen;

#define USE_MIASTAR 1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_manager");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv("~");

  debug_publisher::init(nh_);
  ros::Duration(0.5).sleep();

  debug_publisher::DBSendNew("plan_manager", "Program start");
  planner_manager.reset(new PlannerManager);
  planner_manager->init(nh_, nh_priv);

  debug_assistant.reset(new DebugAssistant);
  debug_assistant->init(nh_, planner_manager);
  debug_publisher::DBSendNew("plan_manager", "Program init done!");

  double test_rate = planner_manager->config.testRate;
  if (test_rate > 0.0)
  {
    ros::Rate lr(test_rate);
    while (ros::ok())
    {
      planner_manager->process();
      ros::spinOnce();
      lr.sleep();
    }
  }
  else
  {
    ros::spin();
  }
  ros::spin();
  return 0;
}

void PlannerManager::init(ros::NodeHandle &nh, ros::NodeHandle &nh_prev)
{

  debug_publisher::DBSendNew("plan_manager", "planner_manager init start");
  config.loadParameters(nh_prev);
  Loadstartandend = config.loadStartEnd;
  if (Loadstartandend)
  {
    LoadStartEnd();
  }
  bdx = config.kernel_size * config.occupancy_resolution;
  bdy = config.kernel_size * config.occupancy_resolution;
  bdz = config.kernel_size * config.occupancy_resolution;

  pcsmap_manager.reset(new PCSmapManager(config));
  pcsmap_manager->init(nh);
  debug_publisher::DBSendNew("plan_manager", "pcsmap_manager init done");

  sv_manager.reset(new SweptVolumeManager(config));
  sv_manager->init(nh, config);
  debug_publisher::DBSendNew("plan_manager", "sv_manager init done");

  astar_searcher.reset(new AstarPathSearcher);
  astar_searcher->init(nh);
  astar_searcher->kernel_size = config.kernel_size;

  debug_publisher::DBSendNew("plan_manager", "front_end init done");

  traj_parlength = 3.0;
  minco_traj_optimizer.reset(new TrajOptimizer);
  minco_traj_optimizer->setParam(nh, config);
  minco_traj_optimizer->setEnvironment(sv_manager);

  debug_publisher::DBSendNew("plan_manager", "back_end init done");

  ori_traj_generator.reset(new OriTraj);
  ori_traj_generator->setParam(nh, config);

  current_front_end = USE_MIASTAR;

  visulizer.reset(new vis::Visualization(nh));
  target_sub = nh.subscribe("/goal", 1, &PlannerManager::targetRcvCallBack, this);
  rcvmap_signal_sub = nh.subscribe("/rcvmap_signal", 1, &PlannerManager::mapRcvCallBack, this);

  setting_sub = nh.subscribe("/setting_receive", 1, &PlannerManager::settingRcvCallBack, this);
  rs = nh.subscribe("/reshow", 1, &PlannerManager::reShowTraj, this);
  debug_publisher::DBSendNew("plan_manager", "planner_manager init done");
}

bool PlannerManager::generatePath(Vector3d start, Vector3d end)
{

  astar_searcher->AstarPathSearch(start, end);
  if (astar_searcher->success_flag)
  {
    recent_path = astar_searcher->getPath();
    visulizer->visR3Path("front_end_path", recent_path);
    Eigen::Matrix3d Rt;
    Eigen::Vector3d trans;
    for (size_t i = 0; i < recent_path.size(); i++)
    {
      trans = recent_path[i];
      double yaw = trans(2);
      trans(2) = 0.0;
      Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      sv_manager->current_robot_shape->Transform(Rt, trans);
      visulizer->visPolytope(sv_manager->current_robot_shape->mesh_var, "front_end_pose", "front_end_pose_edge", false, 2, i + 11452, Color::steelblue, 0.4);
    }

    recent_se3_path = astar_searcher->getastarSE3Path();
    visulizer->visSE3Path("SE3path", sv_manager->current_robot_shape->mesh, recent_se3_path);
    ROS_WARN("[A*] search success.");
  }
  else
  {
    ROS_WARN("[A*] search failed.");
  }
  astar_searcher->reset();
  return (astar_searcher->success_flag);
}

void PlannerManager::generateTraj(vector<Vector3d> path)
{
  int N;

  int path_size = path.size();
  double temp_traj_parlength = traj_parlength;
  int index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));

  while (index_gap >= path_size - 1)
  {
    temp_traj_parlength /= 1.5;
    index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
  }

  bool ret_opt;
  Matrix3d initState = Matrix3d::Zero();
  Matrix3d finalState = Matrix3d::Zero();
  initState.col(0) = path.front();

  finalState.col(0) = path.back();

  vector<Vector3d> Q;
  vector<Vector3d> acc_list;
  Vector3d wp;
  Vector3d tmp_pos{999, 999, 999};
  Vector3d dir;
  Matrix3d rotate;
  vector<Matrix3d> rotatelist;
  minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
  for (int ind = index_gap; ind < path_size - 1; ind += index_gap)
  {
    wp = path[ind];
    rotate = recent_se3_path[ind].getRotMatrix();
    dir = rotate * Eigen::Vector3d(0, 0, 1);
    Q.push_back(wp);
    rotatelist.push_back(rotate);
    acc_list.push_back(dir);
    minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(wp, tmp_pos, bdx / 3.0, bdy / 3.0, bdz / 3.0);
    tmp_pos = wp;
  }
  minco_traj_optimizer->parallel_points.clear();
  for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points)
  {
    minco_traj_optimizer->parallel_points.push_back(pair.second);
  }
  minco_traj_optimizer->parallel_points_num = minco_traj_optimizer->pcsmap_manager->aabb_points.size();
  cout << "\033[32m=========parallel_points_num:==========\033[0m" << minco_traj_optimizer->parallel_points_num << endl;
  minco_traj_optimizer->lastTstar = vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);

  minco_traj_optimizer->renderAABBpoints();

  N = Q.size() + 1;

  Matrix3Xd inPts = MatrixXd::Zero(3, N);
  for (int i = 0; i < N - 1; i++)
  {
    inPts.col(i) = Q[i];
  }
  VectorXd ts = config.inittime * VectorXd::Ones(N);
  bool ret;
  bool mid_ret;
  ros::Time t1;
  visulizer->visSE3Vec("path_vec", Q, acc_list, 156468);
  Eigen::VectorXd opt_x;
  mid_ret = ori_traj_generator->getOriTraj(initState, finalState, Q, ts, acc_list, rotatelist, N, recent_traj, opt_x);

  sv_manager->updateTraj(recent_traj);
  clear();
  std::cout << "[planner manager] Ori trajectory generated successfully!" << std::endl;
  if (mid_ret)
  {
    ret = minco_traj_optimizer->optimize_traj_lmbm(initState, finalState, opt_x, N, recent_traj);
    recent_ret = ret;
  }
  std::cout << "==========mid_ret:" << mid_ret << " ret:=============== " << ret << std::endl;
  if (mid_ret && ret)
  {
    std::cout << "[planner manager] Trajectory optimization is successful! " << std::endl;
    visulizer->visTraj("traj", recent_traj, 1, true);
    std::cout << " tmax for traj: " << recent_traj.getTotalDuration() << std::endl;
    sv_manager->updateTraj(recent_traj);


    auto lasttrans = recent_traj.getPos(recent_traj.getTotalDuration() - 1e-5);
    double lastyaw = lasttrans(2);
    sv_manager->lastRotate = Eigen::AngleAxisd(lastyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    sv_manager->current_robot_shape->Transform(planner_manager->sv_manager->lastRotate, planner_manager->recent_traj.getPos(planner_manager->recent_traj.getTotalDuration()) + Eigen::Vector3d(0, 0, 5));
    sv_manager->vis->visPolytope(planner_manager->sv_manager->current_robot_shape->mesh_var, "Polymeshoriginal", "Polyedgeoriginal", false, 1, 3, vis::blue, 1.0); // 末端可视化
    sv_manager->current_robot_shape->Transform(Eigen::Matrix3d::Identity(), planner_manager->recent_traj.getPos(0) + Eigen::Vector3d(0, 0, 5));
    sv_manager->vis->visPolytope(planner_manager->sv_manager->current_robot_shape->mesh_var, "Polymeshoriginal", "Polyedgeoriginal", false, 1, 2, vis::blue, 1.0); // 首可视化

    sv_manager->process(recent_traj);

    sv_manager->setTrajStamp(ros::Time::now().toSec());
  }
  else
  {
    std::cout << "[planner manager] Trajectory optimization is failed! " << std::endl;
  }
}

void PlannerManager::clear()
{
  planner_manager->sv_manager->swept_cal->sweptvolumeclear();
}

void PlannerManager::mapRcvCallBack(const std_msgs::Empty &msg)
{
  debug_publisher::DBSendNew("plan_manager", "Try to gene byte kernel");
  uint8_t *map_kernel = pcsmap_manager->generateMapKernel2D(config.kernel_size);
  debug_publisher::DBSendNew("plan_manager", "gene byte kernel done!");

  int sizeX = pcsmap_manager->occupancy_map->X_size;
  int sizeY = pcsmap_manager->occupancy_map->Y_size;
  int sizeZ = pcsmap_manager->occupancy_map->Z_size;
  sv_manager->setMapKernel(map_kernel, sizeX, sizeY, sizeZ);
  debug_publisher::DBSendNew("plan_manager", "set byte kernel done!");
  astar_searcher->initGridMap(pcsmap_manager, sv_manager);

  minco_traj_optimizer->setGridMap(pcsmap_manager);

  cout << "init map A* --" << endl;
}

void PlannerManager::settingRcvCallBack(const std_msgs::Int8MultiArray &msg)
{
#define CLEAR_MAP 1
#define SETTING_FRONT_END 2
#define SETTING_SHAPE 3

  if (msg.data.size() < 2)
  {
    return;
  }
  int head = msg.data[0];

  if (head == CLEAR_MAP)
  {
    pcsmap_manager->clearMap();
    std::cout << "[planner manager] clear map!" << std::endl;
  }
}

void PlannerManager::targetRcvCallBack(const geometry_msgs::PoseStamped &msg)
{
  if (step_state == STEP_NOPOINT)
  {
    clear();
    start_pos(0) = msg.pose.position.x;
    start_pos(1) = msg.pose.position.y;
    start_pos(2) = 0.0;
    if (Loadstartandend)
    {
      cout << "\033[32m=========load start point========\033[0m" << endl;
      start_pos = Start_record;

      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

      std::cout << "\033[32m Loaded Start_record pose: " << Start_record.x() << ", " << Start_record.y() << ", " << Start_record.z() << "\033[0m" << std::endl;

      std::cout << std::setprecision(std::numeric_limits<double>::digits10);

      start_pos(2) = 0.0;
    }
    step_state = STEP_HAVE_START;
    std::cout << "[plan_manager] Get start position! " << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Get start position!");
  }
  else if (step_state == STEP_HAVE_START)
  {
    end_pos(0) = msg.pose.position.x;
    end_pos(1) = msg.pose.position.y;
    end_pos(2) = 0.0;
    if (Loadstartandend)
    {
      cout << "\033[32m=========load End point========\033[0m" << endl;
      end_pos = End_record;

      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

      std::cout << "\033[32m Loaded End_record pose: " << End_record.x() << ", " << End_record.y() << ", " << End_record.z() << "\033[0m" << std::endl;

      std::cout << std::setprecision(std::numeric_limits<double>::digits10);

      end_pos(2) = 0.0;
    }
    step_state = STEP_HAVE_TARGET;
    std::cout << "[plan_manager] Get target position! " << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Get target position!");
  }
  else if (step_state == STEP_HAVE_TARGET)
  {
    start_pos(0) = msg.pose.position.x;
    start_pos(1) = msg.pose.position.y;
    start_pos(2) = 0.0;
    if (Loadstartandend)
    {
      cout << "\033[32m=========load start point========\033[0m" << endl;
      start_pos = Start_record;

      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

      std::cout << "\033[32m Loaded start pose: " << Start_record.x() << ", " << Start_record.y() << ", " << Start_record.z() << "\033[0m" << std::endl;

      std::cout << std::setprecision(std::numeric_limits<double>::digits10);

      start_pos(2) = 0.0;
    }
    step_state = STEP_HAVE_START;
    std::cout << "[plan_manager] Get start position! " << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Get target position!");
  }

  if (step_state == STEP_HAVE_TARGET)
  {
    std::cout << "[plan_manager] Try to generate path. " << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Try to generate path.");
    if (generatePath(start_pos, end_pos))
    {
      std::cout << "[plan_manager] Path generated successfully! " << std::endl;
      debug_publisher::DBSendNew("plan_manager", "Path generated successfully!");
      generateTraj(recent_path);
    }
  }
}

void PlannerManager::process()
{
  this->sv_manager->process(recent_traj);
}

void PlannerManager::LoadStartEnd()
{
  std::string fullPath = config.inputdata;
  std::string prefix = "shapes/";
  size_t startPos = fullPath.find(prefix);
  std::string shapeString;
  if (startPos != std::string::npos)
  {
    startPos += prefix.length();
    size_t endPos = fullPath.find(".obj", startPos);

    if (endPos != std::string::npos)
    {
      shapeString = fullPath.substr(startPos, endPos - startPos);
      std::cout << "ShapeString : " << shapeString << std::endl;
    }
    else
    {
      std::cout << "The string does not contain '.obj' after 'shapes/'" << std::endl;
    }
  }
  else
  {
    std::cout << "The string does not contain 'shapes/'" << std::endl;
  }

  cout << "\033[32m========load traj start and end points=======\033[0m" << endl;
  std::string filename = ros::package::getPath(string("plan_manager")) + "/pcds/trajectory_" + shapeString + ".txt";
  std::ifstream file(filename);
  std::string line;

  if (!file.is_open())
  {
    std::cerr << "无法打开文件 " << filename << std::endl;
  }
  std::string labelstart{"Start: "};

  while (std::getline(file, line))
  {
    if (line.find(labelstart) != std::string::npos)
    {
      std::istringstream iss(line.substr(labelstart.length()));
      iss >> Start_record.x() >> Start_record.y() >> Start_record.z();
      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
      std::cout << "\033[32m Load start pose: " << Start_record.x() << ", " << Start_record.y() << ", " << Start_record.z() << "\033[0m" << std::endl;
      std::cout << std::setprecision(std::numeric_limits<double>::digits10);
      break;
    }
  }
  std::string labelend("End: ");
  while (std::getline(file, line))
  {
    if (line.find(labelend) != std::string::npos)
    {
      std::istringstream iss(line.substr(labelend.length()));
      iss >> End_record.x() >> End_record.y() >> End_record.z();
      std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
      std::cout << "\033[32m Load End pose: " << End_record.x() << ", " << End_record.y() << ", " << End_record.z() << "\033[0m" << std::endl;
      std::cout << std::setprecision(std::numeric_limits<double>::digits10);
      break;
    }
  }
  file.close();
}

void PlannerManager::reShowTraj(const std_msgs::Empty::Ptr msg)
{
  sv_manager->setTrajStamp(ros::Time::now().toSec());
  sv_manager->process(recent_traj);
}

const uint8_t bit_sel[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
void DebugAssistant::debugMsgcallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  planner_manager->sv_manager->swept_cal->sweptvolumeclear();

  if (msg->data.size() < 1)
  {
    return;
  }
  // stop optimization
  if (msg->data[0] == 21)
  {
    planner_manager->minco_traj_optimizer->exit = true;
  }

}

void DebugAssistant::init(ros::NodeHandle &nh, PlannerManager::Ptr pm)
{

  debug_publisher::DBSendNew("plan_manager", "debug_assistant init start");
  debug_sub = nh.subscribe("/debug_cmd", 10, &DebugAssistant::debugMsgcallback, this);
  planner_manager = pm;
  debug_publisher::DBSendNew("plan_manager", "debug_assistant init done");
}