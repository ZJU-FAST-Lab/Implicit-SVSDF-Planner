#ifndef PCSMAP_MANAGER_H
#define PCSMAP_MANAGER_H

#include <string.h>
#include <ros/ros.h>
#include <unordered_map>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/config.hpp>
#include <utils/debug_publisher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "map_manager/GridMap3D.h"



class PCSmapManager
{
    public:
        PCSmapManager(const Config&conf);
        ~PCSmapManager();
        static constexpr uint8_t or_mask[8]={0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
        // param load
        void init(ros::NodeHandle& nh);

        std::vector<Eigen::Vector3d> obs_forblender;
        // callback functions
        void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& globalmap);
        void rcvOdomHandler(const nav_msgs::Odometry odom);
        void rcvRenderGrad(const std_msgs::Int16 msg);


       inline uint8_t* generateMapKernel(const int& kernel_size)
       {
            
            int side_size = (kernel_size - 1)/2;
            int x_size    = occupancy_map -> X_size + 2 * side_size;
            int y_size    = occupancy_map -> Y_size + 2 * side_size;
            int z_size    = occupancy_map -> Z_size + 2 * side_size;
            int bytes_len_of_last_dim = (z_size + 7) / 8;
            int size_yz   = y_size * bytes_len_of_last_dim;
            int bytes_len = x_size * y_size * bytes_len_of_last_dim;
            uint8_t *map_kernel = new uint8_t[bytes_len]();


            
            for (int x = 0; x < occupancy_map -> X_size; x++)
            { 
                for (int y = 0; y < occupancy_map -> Y_size; y++)
                { 
                    for (int z = 0; z < occupancy_map -> Z_size; z++)
                    { 
                        int flate_x  = x + side_size;
                        int flate_y  = y + side_size;
                        int flate_z  = z + side_size;

                        int byte_idx = flate_x * size_yz + flate_y * bytes_len_of_last_dim + (flate_z + 0) / 8;
                        int byte_offset = flate_z % 8;
                        if ( occupancy_map -> isIndexOccupied(x,y,z) == true)
                        {
                            map_kernel[byte_idx] |= or_mask[byte_offset];
                        }
                    }
                }
            }
            mk = map_kernel;
            return map_kernel;
       }


       inline uint8_t* generateMapKernel2D(const int& kernel_size)
       {
            
            int side_size = (kernel_size - 1)/2;
            int x_size    = occupancy_map -> X_size + 2 * side_size;
            int y_size    = occupancy_map -> Y_size + 2 * side_size;
            int bytes_len_of_last_dim = (y_size + 7) / 8;
            int bytes_len = x_size * bytes_len_of_last_dim;
            uint8_t *map_kernel = new uint8_t[bytes_len]();

            for (int x = 0; x < occupancy_map -> X_size; x++)
            { 
                for (int y = 0; y < occupancy_map -> Y_size; y++)
                { 
                    int flate_x  = x + side_size;
                    int flate_y  = y + side_size;

                    int byte_idx    = flate_x  * bytes_len_of_last_dim + (flate_y + 0) / 8;
                    int byte_offset = flate_y % 8;
                    if ( occupancy_map -> isIndexOccupied(x,y,0) == true)
                    {
                        map_kernel[byte_idx] |= or_mask[byte_offset];
                    }
                }
            }
            mk = map_kernel;
            return map_kernel;
       }


        inline void clearMap() {
            if( recieved_globalmap == true ){
                occupancy_map -> releaseMemory();
                recieved_globalmap = false;
            }
        }

        inline int unifiedID(const int& i, const int& j, const int& k) const
        {
            int unified_id = 0;
            unified_id += k * (occupancy_map->X_size) * (occupancy_map->Y_size);
            unified_id += j * (occupancy_map->X_size) ;
            unified_id += i;
            return unified_id;
        }


        inline void projInMap(Eigen::Vector3d& pt) const{
            if( pt(0) < boundary_xyzmin(0) ){ pt(0) = boundary_xyzmin(0);}
            if( pt(1) < boundary_xyzmin(1) ){ pt(1) = boundary_xyzmin(1);}
            if( pt(2) < boundary_xyzmin(2) ){ pt(2) = boundary_xyzmin(2);}
            if( pt(0) > boundary_xyzmax(0) ){ pt(0) = boundary_xyzmax(0);}
            if( pt(1) > boundary_xyzmax(1) ){ pt(1) = boundary_xyzmax(1);}
            if( pt(2) > boundary_xyzmax(2) ){ pt(2) = boundary_xyzmax(2);}
        }
        
        inline void getPointsInAABB2D(const Eigen::Vector3d &center, double halfbdx,double halfbdy, std::vector<Eigen::Vector2d>& ob_pts) const
        {
            Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, 0);
            Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, 0);
            corner1(2) = 0.0;
            corner2(2) = 0.0;
            projInMap(corner1);
            projInMap(corner2);
            const Eigen::Vector3i idcorner1 = occupancy_map -> getGridIndex( corner1 );
            const Eigen::Vector3i idcorner2 = occupancy_map -> getGridIndex( corner2 );
            
            for (int i = idcorner1(0); i <= idcorner2(0); i++)
            {
                for (int j = idcorner1(1); j <= idcorner2(1); j++)
                {
                    if( occupancy_map -> isIndexOccupied(i,j,0) )
                    {
                        ob_pts.emplace_back( occupancy_map->getGridCubeCenter(i,j,0).head(2) ); //posI2D
                    }
                }
            }
        }

        inline void getPointsInAABB(const Eigen::Vector3d &center, double halfbdx,double halfbdy,double halfbdz, std::vector<Eigen::Vector3d>& ob_pts) const
        {
            Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            projInMap(corner1);
            projInMap(corner2);
            const Eigen::Vector3i idcorner1 = occupancy_map -> getGridIndex( corner1 );
            const Eigen::Vector3i idcorner2 = occupancy_map -> getGridIndex( corner2 );

            for (int i = idcorner1(0); i <= idcorner2(0); i++)
            {
                for (int j = idcorner1(1); j <= idcorner2(1); j++)
                {
                    for (int k = idcorner1(2); k <= idcorner2(2); k++)
                    {
                        if( occupancy_map -> isIndexOccupied(i,j,k) )
                        {
                            ob_pts.emplace_back( occupancy_map->getGridCubeCenter(i,j,k) ); //posI2D
                        }
                    }
                }
            }
        }

        inline void getPointsInAABBOutOfLastOne(const Eigen::Vector3d &center, const Eigen::Vector3d &center_last, double halfbdx,double halfbdy,double halfbdz)
        {
            Eigen::Vector3d corner1 = center - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            Eigen::Vector3d corner2 = center + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            projInMap(corner1);
            projInMap(corner2);
            const Eigen::Vector3i idcorner1 = occupancy_map -> getGridIndex( corner1 );
            const Eigen::Vector3i idcorner2 = occupancy_map -> getGridIndex( corner2 );

            Eigen::Vector3d corner1_l = center_last - Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            Eigen::Vector3d corner2_l = center_last + Eigen::Vector3d(halfbdx, halfbdy, halfbdz);
            projInMap(corner1_l);
            projInMap(corner2_l);
            const Eigen::Vector3i idcorner1_l = occupancy_map -> getGridIndex( corner1_l );
            const Eigen::Vector3i idcorner2_l = occupancy_map -> getGridIndex( corner2_l );

   
            for (int i = idcorner1(0); i <= idcorner2(0); i++)
            {
                for (int j = idcorner1(1); j <= idcorner2(1); j++)
                {
                    for (int k = idcorner1(2); k <= idcorner2(2); k++)
                    {   
                        if( i > idcorner2_l(0) || i < idcorner1_l(0) ||
                            j > idcorner2_l(1) || j < idcorner1_l(1) ||
                            k > idcorner2_l(2) || k < idcorner1_l(2) )
                        {
                            if( occupancy_map -> isIndexOccupied(i,j,k) )
                            {
                                aabb_points.emplace( unifiedID(i,j,k), occupancy_map -> getGridCubeCenter(i,j,k) );
                            }
                        }
                    }
                }
            }
        }

        // map size box
        Vector3d boundary_xyzmin;
        Vector3d boundary_xyzmax;

        // map resolutions
        double occupancy_resolution;

        // some params
        // point count threshold while generating occupancy grid map using point cloud
        int sta_threshold;
        // map received flag
        bool recieved_globalmap;
        // debug output switch
        bool debug_output;
        
        // global 3D occupancy grid map
        GridMap3D::Ptr occupancy_map;

        uint8_t *mk;

        std::unordered_map<int, Eigen::Vector3d> aabb_points;


    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree;

        // ros
        ros::Subscriber globalmap_sub;
        ros::Subscriber odometry_sub;
        ros::Subscriber debug_grad_sub;

        ros::Publisher  globalmap_vis_pub;
        ros::Publisher  gridmap_vis_pub;
        ros::Publisher  rcvmap_signal_pub;
        ros::Publisher  debug_grad_pub;
        
     public:
         typedef shared_ptr<PCSmapManager> Ptr;
};

#endif