#ifndef VIS_HPP
#define VIS_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/quickhull.hpp>
#include <utils/geo_utils.hpp>
#include <utils/trajectory.hpp>
#include <string>
#include <utils/se3_state.hpp>


#define TRAJ_ORDER 5

using namespace Eigen;
using namespace std;
namespace vis
{
    
    struct BALL
    {
        Eigen::Vector3d center;
        double radius;
        BALL(const Eigen::Vector3d &c, double r) : center(c), radius(r){};
        BALL(){};
    };

    struct ELLIPSOID
    {
        Eigen::Vector3d c;
        double rx, ry, rz;
        Eigen::Matrix3d R;
        ELLIPSOID(const Eigen::Vector3d &center, const Eigen::Vector3d &r, const Eigen::Matrix3d &rot)
            : c(center), rx(r.x()), ry(r.y()), rz(r.z()), R(rot){};
        ELLIPSOID(){};
    };

    using PublisherMap = std::unordered_map<std::string, ros::Publisher>; 
                                                                        
    enum Color
    {
        white,
        red,
        green,
        blue,
        light_blue,
        yellow,
        chartreuse,
        black,
        gray,
        orange,
        purple,
        pink,
        steelblue,
        lightyellow,
        lightred,
        darkred
    };


    class Visualization
    {
    private:
        ros::NodeHandle nh_;
        PublisherMap publisher_map_; 
        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   Color color = blue,
                                   double a = 1)
        {
            marker.color.a = a;
            switch (color)
            {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case light_blue:
                marker.color.r = 0.4;
                marker.color.g = 0.6;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            case lightyellow:
                marker.color.r = 0.9490196;
                marker.color.g = 0.8862745;
                marker.color.b = 0.5664062;
                break;
            case lightred:
                marker.color.r = 0.9490196;
                marker.color.g = 0.6549019;
                marker.color.b = 0.5019607;
                break;
            case darkred:
                marker.color.r = 0.8470588;
                marker.color.g = 0.4784313;
                marker.color.b = 0.2901960;
                break;
            }
        }

   
        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   double a,
                                   double r,
                                   double g,
                                   double b)
        {
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        }

        inline void setMarkerScale(visualization_msgs::Marker &marker,
                                   const double &x,
                                   const double &y,
                                   const double &z)
        {
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
        }
    
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
       
        template <class ROTATION>
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z,
                                  const ROTATION &R)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            Eigen::Quaterniond r(R);
            marker.pose.orientation.w = r.w();
            marker.pose.orientation.x = r.x();
            marker.pose.orientation.y = r.y();
            marker.pose.orientation.z = r.z();
        }

    public:
        Visualization() = delete;
        Visualization(ros::NodeHandle &nh) : nh_(nh) {}
        typedef std::shared_ptr<Visualization> Ptr;


        

        template <class TOPIC>
        inline void pubFloat64(const TOPIC &topic, const double msg)
        {
            auto got = publisher_map_.find(topic); 
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            std_msgs::Float64 datamsg;
            datamsg.data = msg;
            publisher_map_[topic].publish(datamsg);
        }


        template <class CENTER, class TOPIC>
        inline void visABall(const CENTER &c,
                             const double &r,
                             const TOPIC &topic,
                             const Color color = blue,
                             const double a = 1,
                             const bool keep = false)
        {
            auto got = publisher_map_.find(topic); 
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerColor(marker, color, a);
            setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
            setMarkerPose(marker, c[0], c[1], c[2]);
            marker.header.stamp = ros::Time::now();
            if (keep)
            {
                static int i = 0;
                marker.id = i;
                i++;
            }
            publisher_map_[topic].publish(marker);
        }

        template <class CENTER, class TOPIC>
        inline void visABallWithId(const CENTER &c,
                                     const double &r,
                                     const TOPIC &topic,
                                     const Color color = blue,
                                     const double a = 1,
                                     const int id = 1)
        {
            auto got = publisher_map_.find(topic); 
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerColor(marker, color, a);
            setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
            setMarkerPose(marker, c[0], c[1], c[2]);
            marker.header.stamp = ros::Time::now();
            marker.id = id;
            publisher_map_[topic].publish(marker);
        }



        template <class PC, class TOPIC>
        inline void visPointcloudByVector(const PC &pc, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            sensor_msgs::PointCloud2 point_cloud_msg;
            point_cloud.reserve(pc.size());
            for (const auto &pt : pc)
            {
                point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
            }
            pcl::toROSMsg(point_cloud, point_cloud_msg);
            point_cloud_msg.header.frame_id = "map";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }


        template <class PC, class TOPIC>
        inline void visPointcloudXYZI(const PC &pc, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            sensor_msgs::PointCloud2 point_cloud_msg;
            pcl::toROSMsg(pc, point_cloud_msg);
            point_cloud_msg.header.frame_id = "map";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }


        template <class PATH, class TOPIC>
        inline void visPath(const PATH &path, const TOPIC &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            nav_msgs::Path path_msg;
            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header.frame_id = "map";
            for (const auto &pt : path)
            {
                tmpPose.pose.position.x = pt[0];
                tmpPose.pose.position.y = pt[1];
                tmpPose.pose.position.z = pt[2];
                path_msg.poses.push_back(tmpPose);
            }
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(path_msg);
        }

        template <class BALLS, class TOPIC>
        inline void visBalls(const BALLS &balls,
                                    const TOPIC &topic,
                                    bool keep = false,
                                    const Color color = blue,
                                    const double a = 0.2)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            if (!keep)
            {
                marker.id = 0;
            }
            else
            {
                static int id = 0;
                marker.id = id;
                id++;
            }
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(balls.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto &ball : balls)
            {
                setMarkerPose(marker, ball.center[0], ball.center[1], ball.center[2]);
                auto d = 2 * ball.radius;
                setMarkerScale(marker, d, d, d);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }

        template <class TOPIC>
        inline void visBalls(const Eigen::Matrix3Xd &balls,
                                    const TOPIC &topic,
                                    bool keep = false,
                                    const Color color = blue,
                                    const double a = 0.2,
                                    const double scale = 1)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            if (!keep)
            {
                marker.id = 0;
            }
            else
            {
                static int id = 0;
                marker.id = id;
                id++;
            }
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            int size = balls.cols();
            marker_array.markers.reserve(size + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (int i = 0; i < size; i++)
            {
                setMarkerPose(marker, balls.col(i)[0], balls.col(i)[1], balls.col(i)[2]);
                setMarkerScale(marker, scale, scale, scale);
                marker_array.markers.push_back(marker);
                marker.id++;
            }

            publisher_map_[topic].publish(marker_array);
        }

 
        template <class ELLIPSOIDS, class TOPIC>
        inline void visEllipsoids(const ELLIPSOIDS &ellipsoids,
                                         const TOPIC &topic,
                                         const Color color = blue,
                                         const double a = 0.8)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = 0;
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(ellipsoids.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto &e : ellipsoids)
            {
                setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R); // 导入xyz位置，姿态默认水平
                setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }
        
        
  
        template <class PAIRLINE, class TOPIC>
        inline void visPairline(const PAIRLINE &pairline, const TOPIC &topic, const Color &color = green, double scale = 0.1)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            setMarkerPose(marker, 0, 0, 0);
            setMarkerColor(marker, color, 1);
            setMarkerScale(marker, scale, scale, scale);
            marker.points.resize(2 * pairline.size());
            for (size_t i = 0; i < pairline.size(); ++i)
            {
                marker.points[2 * i + 0].x = pairline[i].first[0];
                marker.points[2 * i + 0].y = pairline[i].first[1];
                marker.points[2 * i + 0].z = pairline[i].first[2];
                marker.points[2 * i + 1].x = pairline[i].second[0];
                marker.points[2 * i + 1].y = pairline[i].second[1];
                marker.points[2 * i + 1].z = pairline[i].second[2];
            }
            publisher_map_[topic].publish(marker);
        }


        template <class ARROWS, class TOPIC>
        inline void visArrows(const ARROWS &arrows, const TOPIC &topic, const Color &color)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            visualization_msgs::Marker clear_previous_msg;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::Marker arrow_msg;
            arrow_msg.type = visualization_msgs::Marker::ARROW;
            arrow_msg.action = visualization_msgs::Marker::ADD;
            arrow_msg.header.frame_id = "map";
            arrow_msg.id = 0;
            arrow_msg.points.resize(2);
            setMarkerPose(arrow_msg, 0, 0, 0);
            setMarkerScale(arrow_msg, 0.4, 0.7, 0);
            setMarkerColor(arrow_msg, color, 0.7);
            visualization_msgs::MarkerArray arrow_list_msg;
            arrow_list_msg.markers.reserve(1 + arrows.size());
            arrow_list_msg.markers.push_back(clear_previous_msg);
            for (const auto &arrow : arrows)
            {
                arrow_msg.points[0].x = arrow.first[0];
                arrow_msg.points[0].y = arrow.first[1];
                arrow_msg.points[0].z = arrow.first[2];
                arrow_msg.points[1].x = arrow.second[0];
                arrow_msg.points[1].y = arrow.second[1];
                arrow_msg.points[1].z = arrow.second[2];
                arrow_list_msg.markers.push_back(arrow_msg);
                arrow_msg.id += 1;
            }
            publisher_map_[topic].publish(arrow_list_msg);
        }


        template <class TOPIC>
        inline void visVector(const Eigen::Vector3d &vec_pos, Eigen::Vector3d vec_dir,  const TOPIC &topic, const Color &color, const int& id, bool clear_old = false)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000);
                publisher_map_[topic] = pub;
            }

            if(clear_old){
                visualization_msgs::Marker clc;
                clc.header.frame_id = "map";
                clc.type = visualization_msgs::Marker::DELETEALL;
                publisher_map_[topic].publish(clc);
            }

            visualization_msgs::Marker vec;
            vec.type = visualization_msgs::Marker::ARROW;
            vec.action = visualization_msgs::Marker::ADD;
            vec.header.frame_id = "map";
            vec.id = id;

            vec.scale.x = 1.0;
            vec.scale.y = 0.2;
            vec.scale.z = 0.2;
            vec.color.a = 1;
            vec.color.r = 1.0;
            vec.color.g = 1.0;
            vec.color.b = 0.0;
            vec.pose.position.x = vec_pos(0);
            vec.pose.position.y = vec_pos(1);
            vec.pose.position.z = vec_pos(2);
            vec_dir.normalize();
            Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + vec_dir);
            // final_pose.normalize();
            Eigen::AngleAxisd t_V(M_PI, final_pose);
            Eigen::Quaterniond q(t_V);
            q.normalize();
            vec.pose.orientation.w = q.w();
            vec.pose.orientation.x = q.x();
            vec.pose.orientation.y = q.y();
            vec.pose.orientation.z = q.z();
            publisher_map_[topic].publish(vec);
        }


        template <class TOPIC>
        inline void visEdge(const TOPIC &edge_topic, const Eigen::MatrixX3d &start, const Eigen::MatrixX3d &end , const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), const int tid = 11542, const double a = 1.0)
        {
            assert(start.rows() == end.rows() && "start point size != end point size");
            auto got = publisher_map_.find(edge_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub;
            }
            visualization_msgs::Marker edgeMarker;
            edgeMarker.header.stamp = ros::Time::now();
            edgeMarker.header.frame_id = "map";
            edgeMarker.pose.orientation.w = 1.00;
            edgeMarker.action = visualization_msgs::Marker::ADD;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = a;
            edgeMarker.scale.x = 0.2;
            edgeMarker.scale.y = 0.2;
            edgeMarker.scale.z = 0.2;
            // id++;
            edgeMarker.id = tid;
            geometry_msgs::Point point;
            int size = start.rows();
            Eigen::Vector3d rpt(0,0,0);
            for (int i = 0; i < start.rows(); i++)
            {
                rpt = rot * start.row(i).transpose() + offset;
                point.x = rpt(0);
                point.y = rpt(1);
                point.z = rpt(2);
                edgeMarker.points.push_back(point);
                rpt = rot * end.row(i).transpose() + offset;
                point.x = rpt(0);
                point.y = rpt(1);
                point.z = rpt(2);
                edgeMarker.points.push_back(point);
            }
            publisher_map_[edge_topic].publish(edgeMarker);
        }
       
        template <class TOPIC>
        inline void visEdge(const TOPIC &edge_topic, const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(), const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity())
        {

            auto got = publisher_map_.find(edge_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub;
            }
            visualization_msgs::Marker edgeMarker;
            edgeMarker.header.stamp = ros::Time::now();
            edgeMarker.header.frame_id = "map";
            edgeMarker.pose.orientation.w = 1.00;
            edgeMarker.action = visualization_msgs::Marker::ADD;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 1.00;
            edgeMarker.color.g = 0.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.00;
            edgeMarker.scale.y = 0.09;
            edgeMarker.scale.z = 0.09;
            static int id = 0;
            id++;
            edgeMarker.id = id;
            geometry_msgs::Point point;
            Eigen::Vector3d rpt;
            rpt = rot * start + offset;
            point.x = rpt(0);
            point.y = rpt(1);
            point.z = rpt(2);
            edgeMarker.points.push_back(point);
            rpt = rot * end + offset;
            point.x = rpt(0);
            point.y = rpt(1);
            point.z = rpt(2);
            edgeMarker.points.push_back(point);

            publisher_map_[edge_topic].publish(edgeMarker);
        }

     

        template <class TOPIC1,class TOPIC2>
        inline void visMesh(const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, const Eigen::MatrixXd &U, const Eigen::MatrixXi &G, double s = 0.1, const Color color = steelblue, const double a = 0.02)
        {
            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000); 
                publisher_map_[meth_topic] = pub1;
            }
            auto got2 = publisher_map_.find(edge_topic);
            if (got2 == publisher_map_.end())
            {
                ros::Publisher pub2 =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub2;
            }

            visualization_msgs::Marker meshMarker, edgeMarker;

            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.action = visualization_msgs::Marker::ADD;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";
            meshMarker.color.r = 0.00;
            meshMarker.color.g = 0.00;
            meshMarker.color.b = 1.00;
            meshMarker.color.a = a;

            setMarkerColor(meshMarker, color,a);

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;

            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.005 * s;
            edgeMarker.scale.y = 0.005 * s;
            edgeMarker.scale.z = 0.005 * s;

            geometry_msgs::Point point;

            int faces = G.rows();
            for (int i = 0; i < faces; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = U.row(G.row(i)[j])[0];
                    point.y = U.row(G.row(i)[j])[1];
                    point.z = U.row(G.row(i)[j])[2];
                    meshMarker.points.push_back(point);

                    edgeMarker.points.push_back(point);
                    point.x = U.row(G.row(i)[(j + 1) % 3])[0];
                    point.y = U.row(G.row(i)[(j + 1) % 3])[1];
                    point.z = U.row(G.row(i)[(j + 1) % 3])[2];
                    edgeMarker.points.push_back(point);
                }
            }
            publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);

        }



        template <class TOPIC1,class TOPIC2>
        inline void visPolytope(const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 114513, const Color color = green,const double a=1.0, bool del = false)
        {

            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000);
                publisher_map_[meth_topic] = pub1;
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << meth_topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << edge_topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }
            auto got2 = publisher_map_.find(edge_topic);
            if (got2 == publisher_map_.end())
            {
                ros::Publisher pub2 =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub2;
                                double wait_time = 0.0;
                while (pub2.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << meth_topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << edge_topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }

            visualization_msgs::Marker meshMarker, edgeMarker;
            std::string se3("SE3path");
            if(meth_topic==se3)
            {
            meshMarker.lifetime=ros::Duration(2.0);
            edgeMarker.lifetime=ros::Duration(2.0);
            }
            if (!keep)
            {
                meshMarker.id = tid;
            }
            else
            {
                static int id = 700;
                meshMarker.id = id;
                id++;
            }

            if(del == true){
                meshMarker.action = visualization_msgs::Marker::DELETEALL;
                edgeMarker=meshMarker;
                publisher_map_[meth_topic].publish(meshMarker);
                publisher_map_[edge_topic].publish(edgeMarker);
                return ;
            }
            else{
                meshMarker.action = visualization_msgs::Marker::ADD;
            }
            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";


            setMarkerColor(meshMarker, lightred,a);

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;

            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = a;
            setMarkerColor(edgeMarker,lightyellow,a);
            edgeMarker.scale.x = 0.005 * s;
            edgeMarker.scale.y = 0.005 * s;
            edgeMarker.scale.z = 0.005 * s;

             meshMarker.id = tid;
            geometry_msgs::Point point;

            int ptnum = mesh.cols(); 

            for (int i = 0; i < ptnum; i++)
            {
                point.x = mesh(0, i);
                point.y = mesh(1, i);
                point.z = mesh(2, i);
                meshMarker.points.push_back(point);
            }

            for (int i = 0; i < ptnum / 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = mesh(0, 3 * i + j);
                    point.y = mesh(1, 3 * i + j);
                    point.z = mesh(2, 3 * i + j);
                    edgeMarker.points.push_back(point);
                    point.x = mesh(0, 3 * i + (j + 1) % 3);
                    point.y = mesh(1, 3 * i + (j + 1) % 3);
                    point.z = mesh(2, 3 * i + (j + 1) % 3);
                    edgeMarker.points.push_back(point);
                }
            }
            // ROS_WARN_STREAM(" pub ");
            publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);
        }

  
        template <class TOPIC1,class TOPIC2>
        inline void visPolytopeOffset(const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 14513, const Color color = green,const double a=1.0, bool del = false)
        {

            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000); 
                publisher_map_[meth_topic] = pub1;
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << meth_topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << edge_topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }
            auto got2 = publisher_map_.find(edge_topic);
            if (got2 == publisher_map_.end())
            {
                ros::Publisher pub2 =
                    nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
                publisher_map_[edge_topic] = pub2;
                double wait_time = 0.0;
                while (pub2.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << meth_topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << edge_topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }

            visualization_msgs::Marker meshMarker, edgeMarker;
       
            meshMarker.id = tid;
     

            if(del == true){
                meshMarker.action = visualization_msgs::Marker::DELETEALL;
                edgeMarker=meshMarker;
                publisher_map_[meth_topic].publish(meshMarker);
                publisher_map_[edge_topic].publish(edgeMarker);
                return ;
            }
            else{
                meshMarker.action = visualization_msgs::Marker::ADD;
            }
            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";


            setMarkerColor(meshMarker, lightred,a);

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;

            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 0.00;
            edgeMarker.color.a = a;
            setMarkerColor(edgeMarker,lightyellow,a);
            edgeMarker.scale.x = 0.005 * s;
            edgeMarker.scale.y = 0.005 * s;
            edgeMarker.scale.z = 0.005 * s;

            meshMarker.id = tid;
            geometry_msgs::Point point;

            int ptnum = mesh.cols(); // ptnum/3

            for (int i = 0; i < ptnum; i++)
            {
                point.x = mesh(0, i);
                point.y = mesh(1, i);
                point.z = mesh(2, i)+3.0;
                meshMarker.points.push_back(point);
            }

            for (int i = 0; i < ptnum / 3; i++) // mesh.cols() 12
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = mesh(0, 3 * i + j);
                    point.y = mesh(1, 3 * i + j);
                    point.z = mesh(2, 3 * i + j)+3.0;
                    edgeMarker.points.push_back(point);
                    point.x = mesh(0, 3 * i + (j + 1) % 3);
                    point.y = mesh(1, 3 * i + (j + 1) % 3);
                    point.z = mesh(2, 3 * i + (j + 1) % 3)+3.0;
                    edgeMarker.points.push_back(point);
                }
            }
           
            publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);
        }


        template <class TOPIC1,class TOPIC2>
        inline void visPolytopeColli(const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 14513, const Color color = green,const double a=1.0, bool del = false)
        {

            auto got = publisher_map_.find(meth_topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000); 
                publisher_map_[meth_topic] = pub1;
                // ROS_WARN_STREAM("mesh pub init");
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << meth_topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << edge_topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }


            visualization_msgs::Marker meshMarker, edgeMarker;
            static int id = 700;
            meshMarker.id = id;
            id++;
           
     

            if(del == true){
                meshMarker.action = visualization_msgs::Marker::DELETEALL;
                // edgeMarker=meshMarker;
                publisher_map_[meth_topic].publish(meshMarker);
                // publisher_map_[edge_topic].publish(edgeMarker);
                return ;
            }
            else{
                meshMarker.action = visualization_msgs::Marker::ADD;
            }
            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "map";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";


            setMarkerColor(meshMarker, pink,0.1);

            meshMarker.scale.x = 1.00;
            meshMarker.scale.y = 1.00;
            meshMarker.scale.z = 1.00;



            meshMarker.id = id;//保留id增长
            geometry_msgs::Point point;

            int ptnum = mesh.cols(); // ptnum/3

            for (int i = 0; i < ptnum; i++)
            {
                point.x = mesh(0, i);
                point.y = mesh(1, i);
                point.z = mesh(2, i)+6.0;
                meshMarker.points.push_back(point);
            }

            for (int i = 0; i < ptnum / 3; i++) // mesh.cols() 12
            {
                for (int j = 0; j < 3; j++)
                {
                    point.x = mesh(0, 3 * i + j);
                    point.y = mesh(1, 3 * i + j);
                    point.z = mesh(2, 3 * i + j)+3.0;
                    // edgeMarker.points.push_back(point);
                    point.x = mesh(0, 3 * i + (j + 1) % 3);
                    point.y = mesh(1, 3 * i + (j + 1) % 3);
                    point.z = mesh(2, 3 * i + (j + 1) % 3)+6.0;
                    // edgeMarker.points.push_back(point);
                }
            }
            // ROS_WARN_STREAM(" pub ");
            // publisher_map_[edge_topic].publish(edgeMarker);
            publisher_map_[meth_topic].publish(meshMarker);
        }


        template <class TOPIC>
        inline void visR3Path( const TOPIC &topic, const vector<Vector3d>& path, int path_id = 114514)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); 
                publisher_map_[topic] = pub1;
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }

            visualization_msgs::Marker sphere, line_strip;
            sphere.lifetime=ros::Duration(4.0);
            line_strip.lifetime=ros::Duration(4.0);

            sphere.header.frame_id = line_strip.header.frame_id = "map";
            sphere.header.stamp    = line_strip.header.stamp = ros::Time::now();

            sphere.type     = visualization_msgs::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

            sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
            sphere.id     = path_id;
            line_strip.id = path_id + 1000;

            sphere.pose.orientation.w   = line_strip.pose.orientation.w = 1.0;
            sphere.color.r              = line_strip.color.r            = 0.4;
            sphere.color.g              = line_strip.color.g            = 1.0;
            sphere.color.b              = line_strip.color.b            = 0.4;
            sphere.color.a              = line_strip.color.a            = 0.8;
            sphere.scale.x              = line_strip.scale.x            = 0.3;
            sphere.scale.y              = line_strip.scale.y            = 0.3;
            sphere.scale.z              = line_strip.scale.z            = 0.3;

            geometry_msgs::Point pt;
            Eigen::Vector3d ptv;
            for (size_t i = 0; i < path.size(); i++)
            {
                ptv = path[i];
                pt.x = ptv(0);
                pt.y = ptv(1);
                pt.z = 0.0;
                sphere.points.push_back(pt);
                line_strip.points.push_back(pt);
            }
            publisher_map_[topic].publish(sphere);
            publisher_map_[topic].publish(line_strip);
        }


        template <class TOPIC>
        inline void visSE3Path( const TOPIC &topic, const Eigen::Matrix3Xd& mesh, const vector<SE3State>& se3_path,const double a=0.1, int se3_path_id = 1918)
        {
            if (se3_path.size() == 0)
            {
                return;
            }
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =nh_.advertise<visualization_msgs::Marker>(topic, 10000); 
                publisher_map_[topic] = pub1;
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }
            
            Eigen::Vector3d pos;
            Eigen::Matrix3d rotate;
            Eigen::Matrix3Xd debugmesh_var;
            bool keep = true;
            visPolytope(debugmesh_var, topic, "SE3edge", keep, 0.1,311,  steelblue, a, true);
            for (int i = 0; i < se3_path.size(); i++)    
            {
                pos    = se3_path[i].position;
                rotate = se3_path[i].getRotMatrix();
                debugmesh_var = (rotate * mesh).colwise() + pos;
                if (i == se3_path.size() - 1){ keep = false;}
                visPolytope(debugmesh_var, topic, "SE3edge", keep,0.1, 10+i,  steelblue, a, false);


            }
        }


        template <class TOPIC>
        inline void visSE3Vec( const TOPIC &topic, const vector<Eigen::Vector3d>& points, const vector<Eigen::Vector3d>& acc_list, int se3_path_id = 191018)
        {
            if (acc_list.size() == 0)
            {
                return;
            }
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                    nh_.advertise<visualization_msgs::Marker>(topic, 10000); 
                publisher_map_[topic] = pub1;
            }
            
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.type   = visualization_msgs::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.id = se3_path_id;

            line_strip.color.r = 1;
            line_strip.color.g = 0;
            line_strip.color.b = 0;
            line_strip.color.a = 1;
            line_strip.scale.x = 0.05 / 2;

            geometry_msgs::Point pt;
            for (double i = 0; i < points.size(); i++)
            {
                Eigen::Vector3d dur_p = points[i];
                pt.x = dur_p(0);
                pt.y = dur_p(1);
                pt.z = dur_p(2);
                line_strip.points.push_back(pt);
            }
            publisher_map_[topic].publish(line_strip);

            const double alpha = 0.1;
            for (double i = 0; i < points.size(); i++)
            {
                Eigen::Vector3d arr_p = points[i];
                Eigen::Vector3d arr_a = acc_list[i];

                visualization_msgs::Marker acc_dir;
                acc_dir.header.frame_id = "map";
                acc_dir.id   = se3_path_id + 1 + i;
                acc_dir.type = visualization_msgs::Marker::ARROW;
                //   acc_dir.scale.x = arr_p.norm();
                acc_dir.scale.x = 1.0;
                acc_dir.scale.y = 0.2;
                acc_dir.scale.z = 0.2;
                acc_dir.color.a = 1;
                acc_dir.color.r = 0.2;
                acc_dir.color.g = 0.2;
                acc_dir.color.b = 1.0;
                acc_dir.pose.position.x = arr_p(0);
                acc_dir.pose.position.y = arr_p(1);
                acc_dir.pose.position.z = arr_p(2);

                arr_a.normalize();
                Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + arr_a);

                Eigen::AngleAxisd t_V(M_PI, final_pose);
                Eigen::Quaterniond q(t_V);
                q.normalize();
                acc_dir.pose.orientation.w = q.w();
                acc_dir.pose.orientation.x = q.x();
                acc_dir.pose.orientation.y = q.y();
                acc_dir.pose.orientation.z = q.z();
                publisher_map_[topic].publish(acc_dir);
            }
        }


        template <class TOPIC>
        inline void visTraj(const TOPIC &topic, Trajectory<TRAJ_ORDER> traj, int traj_id = 15526015,bool ontop=false)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub1 =
                nh_.advertise<visualization_msgs::Marker>(topic, 10000); 
                publisher_map_[topic] = pub1;
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                            ros::Duration(0.1).sleep();
                            wait_time += 0.1;
                            if(wait_time > 0.5){
                                std::cout<<"[Livz] 话题 " << topic <<"似乎没有订阅者,请检查rviz配置。"<< "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                                break;
                            }
                        } 
            }
            visualization_msgs::Marker traj_vis;
            std::string st("step_traj");
            if(topic==st)
            {
            traj_vis.lifetime=ros::Duration(0.5);
            }
            traj_vis.header.stamp       = ros::Time::now();
            traj_vis.header.frame_id    = "map";
            traj_vis.id                 = traj_id;
            traj_vis.type               = visualization_msgs::Marker::LINE_STRIP;
            traj_vis.scale.x            = 0.3;
            traj_vis.scale.y            = 0.3;
            traj_vis.scale.z            = 0.3;
            traj_vis.pose.orientation.x = 0.0;
            traj_vis.pose.orientation.y = 0.0;
            traj_vis.pose.orientation.z = 0.0;
            traj_vis.pose.orientation.w = 1.0;

            traj_vis.color.a            = 1.0;
            traj_vis.color.r            = 1.0;
            traj_vis.color.g            = 1.0;
            traj_vis.color.b            = 0.5;
            setMarkerColor(traj_vis,vis::Color::darkred,1.0);

            geometry_msgs::Point pt;
            Vector3d pos;
            double t_duration = traj.getTotalDuration();
            for (double t = 0; t < t_duration; t += 0.05)
            {
                pos = traj.getPos(t);
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                if(ontop)
                {
                   pt.z = 8; 
                }
                traj_vis.points.push_back(pt);
            }
            publisher_map_[topic].publish(traj_vis);
            
        }

}; // namespace vis
}
#endif