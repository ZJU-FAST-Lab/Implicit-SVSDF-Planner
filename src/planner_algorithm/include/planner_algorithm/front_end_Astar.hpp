#ifndef ASTAR_H
#define ASTAR_H

#include <Eigen/Eigen>
#include "map_manager/PCSmap_manager.h"
#include "swept_volume/sw_manager.hpp"


using namespace Eigen;
using namespace std;

class GridNode
{
    public:
        GridNode(){
            coord = Vector3d(0,0,0);
            index = Vector3i(0,0,0);
            father = NULL;
            gScore = 0;
            fScore = 0;
            yaw    = 0;
            id     = 0;
        }
        GridNode(const Vector3i& ind,const Vector3d& cor){
            coord = cor;
            index = ind;
            father = NULL;
            gScore = 0;
            fScore = 0;
            yaw    = 0;
            id     = 0;
        }
        ~GridNode(){}

        void reset()
        {
            father = NULL;
            gScore = 0;
            fScore = 0;
            yaw    = 0;
            id     = 0;
        }
    
        Vector3d coord;
        Vector3i index;
        double gScore;
        double fScore;
        double yaw;
        int id;
        GridNode* father;
};

class AstarPathSearcher
{ 
    public:
        GridNode ****GridNodeMap;
        GridNode *terminatePtr;
        int GLX_SIZE;  
        int GLY_SIZE; 
        int GLZ_SIZE;  
        int GLYZ_SIZE; 
        int GLXYZ_SIZE;
        uint8_t * data;

        double total_time        = 0.0;
        double total_kernel_time = 0.0;
        int total_kernel  = 0;
        int kernel_size   = 3;
    
    public:


        inline vector<SE3State> getastarSE3Path();

 
        inline vector<Vector3d> getPath();
        
        inline void reset();
        inline void init(ros::NodeHandle& nh);

  
 
        inline void initGridMap(PCSmapManager::Ptr env , SweptVolumeManager::Ptr sv);


        inline void AstarGetSucc(GridNode* currentPtr, vector<GridNode*> & neighborPtrSets, vector<double> & edgeCostSets);


        inline void AstarPathSearch(Vector3d start, Vector3d end);

        inline double getHeu(GridNode* node1, GridNode* node2);
        inline double getCustomCost(GridNode* node_neighbor, GridNode* node_current);
        bool success_flag;

    private:
        Vector3i goalIdx;
        Vector3d start_pt;
        Vector3d end_pt;
        SweptVolumeManager::Ptr sv_manager;

        std::multimap<double, GridNode*> openSet;

    private:
        PCSmapManager::Ptr environment;
    
    public:
         typedef shared_ptr<AstarPathSearcher> Ptr;
};



inline void AstarPathSearcher::init(ros::NodeHandle& nh)
{
    
}

inline void AstarPathSearcher::initGridMap(PCSmapManager::Ptr env, SweptVolumeManager::Ptr sv)
{   
    #define COUT(x) std::cout<<x<<std::endl
    COUT(1);
    environment = env;
    sv_manager  = sv;
    GLX_SIZE    = (environment -> occupancy_map) -> X_size;
    GLY_SIZE    = (environment -> occupancy_map) -> Y_size;
    GLZ_SIZE    = (environment -> occupancy_map) -> Z_size;
    GLYZ_SIZE   = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE  = GLX_SIZE * GLYZ_SIZE;
  
    COUT(2);
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    COUT(3);
    COUT(GLX_SIZE);
    COUT(GLY_SIZE);
    COUT(GLZ_SIZE);
    GridNodeMap = new GridNode *** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNode ** [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNode* [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE; k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = (environment -> occupancy_map) -> getGridCubeCenter(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }

    COUT(4);

}

inline void AstarPathSearcher::reset()
{
    for(int i = 0; i < GLX_SIZE; i++){
        for(int j = 0; j < GLY_SIZE; j++){
            for( int k = 0; k < GLZ_SIZE; k++){
                GridNodeMap[i][j][k] -> reset();
            }
        }
    }
}

inline double AstarPathSearcher::getHeu(GridNode* node1, GridNode* node2)
{

    double cost;
    double p = 1.0/1000;


    Eigen::Vector3i d = node1->index-node2->index;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = min(dx,min(dy,dz));
    int dmax = max(dx,max(dy,dz));
    int dmid =  dx+dy+dz-dmin-dmax;
    double h = sqrt(3)*dmin + sqrt(2)*(dmid-dmin) + (dmax-dmid);

    cost = h*(1+p);

    return cost;
}


// in this function, add customized costs
inline double AstarPathSearcher::getCustomCost(GridNode* node_neighbor, GridNode* node_current)
{
    double cost = 0.0;
    return cost;
}

inline void AstarPathSearcher::AstarGetSucc(GridNode* currentPtr, vector<GridNode*> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    double fy = currentPtr -> yaw;
    double cy = fy;
    double dis_cfy = 0.0;
    double min_dis_cfy = 0.0;
    double dir = 1.0;
    bool cond;
    vector<Eigen::Vector2d> aabb_points;
    Eigen::Vector3d state1 = currentPtr -> coord;
    state1(2) = fy;
    Eigen::Vector3d state2 = Eigen::Vector3d::Zero();
    ros::Time t1,t2,t3;
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            aabb_points.clear();
            Eigen::Vector3i vi(i,j,0);
            vi = vi + currentPtr -> index;

    
                cond = (environment -> occupancy_map) -> isIndexValid(vi) && !(environment -> occupancy_map) -> isIndexOccupiedFlate(vi, 0) ;
             
                cond = cond && sv_manager -> checkKernelValue(fy, cy, vi) ; 


                state2 = (environment -> occupancy_map)->getGridCubeCenter(vi);
                state2(2) = 0;
                environment -> getPointsInAABB2D(state2,  kernel_size/2+1, kernel_size/2+1, aabb_points);
                state2(2) = cy;
                cond = cond && sv_manager -> checkSubSWCollision(state1, state2, aabb_points);


            if (cond)
            {
                GridNode* p = GridNodeMap[vi(0)][vi(1)][vi(2)];
                if(p -> id == 0)
                {
                    p -> yaw = cy;
                }
                neighborPtrSets.push_back(p);
                edgeCostSets.push_back(sqrt(i*i + j*j));
            }
        }
    }
    
}

inline void AstarPathSearcher::AstarPathSearch(Vector3d start, Vector3d end)
{
    
    total_time        = 0.0;
    total_kernel_time = 0.0;
    ros::Time time_1 = ros::Time::now();    
    if( !(environment -> occupancy_map) -> isInMap(start) || !(environment -> occupancy_map) -> isInMap(end) )
    {
       ROS_ERROR("[A*] start or target position is out of map.");
       success_flag = false;
       return ;
    }
    //index of start_point and end_point
    Vector3i start_idx = (environment -> occupancy_map) -> getGridIndex(start);
    Vector3i end_idx   = (environment -> occupancy_map) -> getGridIndex(end);

    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = (environment -> occupancy_map) -> getGridCubeCenter(start_idx);
    end_pt   = (environment -> occupancy_map) -> getGridCubeCenter(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNode* startPtr = new GridNode(start_idx, start_pt);
    GridNode* endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNode* currentPtr  = NULL;
    GridNode* neighborPtr = NULL;


    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    startPtr -> yaw = 0.0;

    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
 
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> id = startPtr -> id; 
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> gScore = startPtr -> gScore;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> fScore = startPtr -> fScore;

    vector<GridNode*> neighborPtrSets;
    vector<double> edgeCostSets;


    // this is the main loop
    while ( !openSet.empty() ){

        auto iter  = std::begin(openSet);
        currentPtr = iter -> second;
        openSet.erase(iter);
        currentPtr -> id = -1;

        // if the current node is the goal 
        if( currentPtr -> index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            success_flag = true;
           
            ROS_WARN("[A*]{sucess}  Time in GetNeighbor is %f ms, kernel time is %f ms, about %f %", total_time, total_kernel_time, 100*total_kernel_time/total_time);       
            ROS_WARN("[A*]{sucess}  get_pose = %d, aver %f ms", total_kernel, total_kernel_time/total_kernel);           
                       
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){

            neighborPtr = neighborPtrSets[i];
            double ec = edgeCostSets[i];
            if(neighborPtr -> id == 0){

                double tg = ec + currentPtr -> gScore; 
                
                neighborPtr -> father = currentPtr;
                neighborPtr -> gScore = tg;
                neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);

                neighborPtr -> id = 1;
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                
                continue;
            }
        
            else if(neighborPtr -> id == 1){ 
                double tg = ec + currentPtr->gScore;
                if (tg < neighborPtr->gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);;
                }
                continue;
            }
        
            else{

                double tg = ec + currentPtr->gScore;
                if(tg < neighborPtr -> gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);;
                     
                    neighborPtr -> id = 1;
                    openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                }
                continue;
            }
        }      
    }
    //if search fails
    success_flag = false;
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

inline vector<Vector3d> AstarPathSearcher::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNode*> gridPath;
    GridNode* p = terminatePtr;
    while (p -> father != NULL)
    {
        gridPath.push_back(p);
        p = p -> father;
    }
    gridPath.push_back(p);

    for (auto ptr: gridPath) {
        Eigen::Vector3d coord = ptr -> coord;
        coord(2) = ptr -> yaw;
        path.push_back(coord);

    }
    reverse(path.begin(),path.end());



    return path;
}

inline vector<SE3State> AstarPathSearcher::getastarSE3Path()
{
    vector<SE3State> se3_path;
    SE3State nstate;
    GridNode* p = terminatePtr;
    double rotz;
    while (p -> father != NULL)
    {
        nstate.position = p -> coord;
        rotz = p -> yaw;
        nstate.rot = Eigen::AngleAxisd(rotz, Eigen::Vector3d::UnitZ());
        se3_path.push_back(nstate);
        p = p -> father;
    }
    nstate.position = p -> coord;
    rotz = p -> yaw;
    nstate.rot = Eigen::AngleAxisd(rotz, Eigen::Vector3d::UnitZ());
    se3_path.push_back(nstate);
    reverse(se3_path.begin(), se3_path.end());
    return se3_path;
}





#endif

