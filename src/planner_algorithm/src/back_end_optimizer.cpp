#include <planner_algorithm/back_end_optimizer.hpp>

int TrajOptimizer::optimize_traj_lmbm(const Eigen::Matrix3d &initS,
                                  const Eigen::Matrix3d &finalS,
                                  Eigen::VectorXd &opt_x,
                                  const int N,
                                  Trajectory<TRAJ_ORDER> &traj)
{
  total_opt_time = 0.0;
  total_sdf_time = 0.0;
  total_AABB_time = 0.0;
  ros::Time start_time = ros::Time::now();
  pieceN = N;
  temporalDim = N;
  spatialDim = 3 * (N - 1);
  initState = initS;
  finalState = finalS;
  minco.setConditions(initState, finalState, pieceN);
  const int total_opt_variable_num = temporalDim + spatialDim;
  x_variable = (double *)malloc(sizeof(double) * total_opt_variable_num);
  Eigen::Map<Eigen::VectorXd>(x_variable, total_opt_variable_num) = opt_x;
  iter = 0;
  std::vector<double> clear_flag;
  clear_flag.push_back(-1);
  debug_publisher::DBSendOptiStep(clear_flag);
  debug_publisher::DBSendLogCost(clear_flag);

  double final_cost;
  lmbm::lmbm_parameter_t param;
  int ret = lmbm::lmbm_optimize(total_opt_variable_num,
                                x_variable,
                                &final_cost,
                                costFunctionLmbmParallel,
                                this,
                                earlyExitLMBM,
                                &param);
  {
    printf("-----------\n");
    printf("| Output: |\n");
    printf("-----------\n");
    printf("%-16s %f\n", "Final value:", final_cost);
  }

  if (ret >= 0)
  {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "[Optimization] Optimization Success. Final Cost = "
              << final_cost
              << std::endl;
              
    for(int i = 0; i < total_opt_variable_num; i++)
    {
      opt_x(i) = x_variable[i];
    }
    
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    if(ret==0)
    {
      ret=1;
    }
    return ret;
  }
  else
  {
    forwardT(x_variable, times, temporalDim); // tao--->T
    forwardP((x_variable + temporalDim), points, spatialDim / 3);
    minco.setConditions(initState, finalState, pieceN);
    minco.setParameters(points, times);
    minco.getTrajectory(traj);
    std::cout << "\x33"
              << "[Optimization] Optimization Failed: "
              << " code = " << ret
              << std::endl;
    
    for(int i = 0; i < total_opt_variable_num; i++)
    {
      opt_x(i) = x_variable[i];
    }
    
    if (x_variable != NULL)
    {
      free(x_variable);
      x_variable = NULL;
    }
    return ret;
  }

  return ret;
}


void TrajOptimizer::drawDebug()
{
  vis->visTraj("step_traj", step_traj, 114515,true);
}

void TrajOptimizer::renderAABBpoints()
{
 vector<Vector3d> aabbpoints;
for (const auto &b: pcsmap_manager -> aabb_points)
{
    aabbpoints.emplace_back(b.second);
}
vis->visPointcloudByVector(aabbpoints,"aabbpoints");
}


void TrajOptimizer::clearvisAABBpoints()
{
  visualization_msgs::MarkerArray mk;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  mk.markers.emplace_back(marker);
  debug_wplists_pub.publish(mk);
}
