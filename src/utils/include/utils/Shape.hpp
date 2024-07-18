#ifndef SHAPE_HPP
#define SHAPE_HPP
#define IGL_STATIC_LIBRARY 1 
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <utils/config.hpp>
#include <utils/Visualization.hpp>
#include <utils/quickhull.hpp>
#include <utils/geo_utils.hpp>
#include <unordered_set>
#include <igl/read_triangle_mesh.h> 
#include <igl/write_triangle_mesh.h> 

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/signed_distance.h>
#include <igl/sparse_voxel_grid.h>
#include <igl/marching_cubes.h>
#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>
#include <igl/writeDMAT.h>
#include <igl/random_points_on_mesh.h>
#include <igl/AABB.h>
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/readDMAT.h>
#define PI 3.14159265358979323846
using namespace vis;
using namespace std;
#define DEFINE_USEFUL_FUNCTION()                                                            \
    inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)                  \
    {                                                                                       \
        double dx = 0.000001;                                                                \
        Eigen::RowVector3d temp = pos_rel;                                                  \
        temp(0) -= dx;                                                                      \
        double sdfold = getonlySDF(temp);                                                   \
        temp(0) += 2 * dx;                                                                  \
        double gradx = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(1) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(1) += 2 * dx;                                                                  \
        double grady = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);                 \
        return grad;                                                           \
    }                                                                                       \
    inline Eigen::Vector3d helperfunc(const Eigen::RowVector3d &pos_rel, double &sdf)       \
    {                                                                                       \
        double dx = 0.000001;                                                                \
        Eigen::RowVector3d temp = pos_rel;                                                  \
        sdf = getonlySDF(pos_rel);                                                          \
        temp(0) -= dx;                                                                      \
        double sdfold = getonlySDF(temp);                                                   \
        temp(0) += 2 * dx;                                                                  \
        double gradx = getonlySDF(temp) - sdfold;                                           \
                                                                                            \
        temp = pos_rel;                                                                     \
                                                                                            \
        temp(1) -= dx;                                                                      \
        sdfold = getonlySDF(temp);                                                          \
        temp(1) += 2 * dx;                                                                  \
        double grady = getonlySDF(temp) - sdfold;                                           \
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);                 \
        return grad;                                                           \
    }                                                                                       \
    inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad) \
    {                                                                                       \
        double sdf;                                                                         \
        grad = helperfunc(pos_rel, sdf);                                                    \
        return sdf;                                                                         \
    }

class Range
{
public:
    double left_board;
    double right_board;
    Range(double l, double r)
    {
        left_board = l;
        right_board = r;
    }
};

namespace shape
{

    const uint8_t or_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
    class BasicShape
    {
    private:
        Config config;

        typedef class shapeKernel
        {
        public:
            bool *map;
            double yaw;
            int totalsize;
            int X_size;
            int Y_size;
            shapeKernel()
            {
                map = nullptr;
            }
            shapeKernel(const shapeKernel &other)
            { 
                map = new bool[other.totalsize];
                memcpy(map, other.map, other.totalsize * sizeof(bool));

                yaw = other.yaw;
                totalsize = other.totalsize;
                X_size = other.X_size;
                Y_size = other.Y_size;
            }
            ~shapeKernel()
            {
                delete[] map;
            }

            inline void init(int xsize_, int ysize_)
            {
                X_size = xsize_;
                Y_size = ysize_;
                totalsize = X_size * Y_size;
                map = new bool[totalsize](); // 初始化全为0即false
            }

            inline int toAddr(const int idx, const int idy) const
            {
                return idx * Y_size + idy;
            }


            inline bool getOccupied(const int idx, const int idy) const
            {
                return map[toAddr(idx, idy)];
            }
        } Shapekernel;

        typedef class byteShapeKernel
        {
        public:
            uint8_t *map;
            double yaw;
            int totalsize;
            int X_size;
            int Y_size;
            byteShapeKernel()
            {
                map = nullptr;
            }
            byteShapeKernel(const byteShapeKernel &other)
            {
                map = new uint8_t[other.totalsize];
                memcpy(map, other.map, other.totalsize * sizeof(uint8_t));

                yaw = other.yaw;
                totalsize = other.totalsize;
                X_size = other.X_size;
                Y_size = other.Y_size;
            }
            ~byteShapeKernel()
            {
                delete[] map;
            }


            inline bool getOccupied(const int idx, const int idy) const
            {
                int bytes_of_last_dim = (Y_size + 7) / 8;
                int byteIdx = (idx * bytes_of_last_dim) + (idy) / 8;
                int offset = idy % 8;

                if ((map[byteIdx] & or_mask[offset]))
                {
                    return true;
                }
                else
                {
                    return false;
                }
                return false;
            }


            inline void generateByteKernel(int kernel_size, const Shapekernel &ori_kernel)
            {
                int kernel_size_y = kernel_size;
                int bytes_len_of_last_dim = (kernel_size + 7) / 8;
                int bytes_len = kernel_size * bytes_len_of_last_dim;
                X_size = Y_size = kernel_size;
                map = new uint8_t[bytes_len];
                memset(map, 0, bytes_len);

                
                for (int x = 0; x < kernel_size; x++)
                { 
                    for (int y = 0; y < kernel_size; y++)
                    { 
                        int byte_idx = (x * bytes_len_of_last_dim) + (y + 0) / 8;
                        int byte_offset = y % 8;
                        if (ori_kernel.getOccupied(x, y) == true)
                        {
                            map[byte_idx] |= or_mask[byte_offset];
                        }
                    }
                }
            }

        } ByteShapeKernel;

        // ================for collision kernels==================
        int kernel_count{-1};  
        int kernelsize{-1};    
        double kernelresu{-1}; 

        typedef struct numSDFGridCell
        {
            Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
            double distance{0.0};
        } NumSDFGridCell;

        // for numeric gradients
        NumSDFGridCell *num_sdf_map;
        int num_sdf_map_X_size;
        int num_sdf_map_Y_size;
        double num_sdf_map_res;     
        double num_sdf_map_xlength; 
        double num_sdf_map_ylength; 
        double x_min;
        double y_min;
        bool initsdfdone{false};
        bool initselfkerneldone{false};

    public:
        Eigen::Vector3d trans;                                 
        Eigen::Matrix3d Rotate;                                
        double yaw;                                            
        Eigen::Matrix3Xd mesh_var;                             
        Eigen::Vector3d interior_var{Eigen::Vector3d::Zero()}; 
        Eigen::Matrix3Xd vertices;                             
        Eigen::Matrix3Xd vertices_var;                         
        Eigen::Matrix3Xd mesh;                                 
        Eigen::MatrixXi F;                                     
        Eigen::MatrixXd V;                                     
        Eigen::MatrixXd V_var;                                 

        Eigen::MatrixXd V_end;         // blender obj
        Eigen::MatrixXd V_start;         // blender obj

        typedef std::shared_ptr<BasicShape> Ptr;
        BasicShape() = delete;
        Shapekernel *shape_kernels{nullptr};          
        ByteShapeKernel *byte_shape_kernels{nullptr}; 
        igl::AABB<Eigen::MatrixXd, 3> tree;
        igl::FastWindingNumberBVH fwn_bvh;

        virtual double getonlySDF(const Eigen::RowVector3d &pos_rel){};
        virtual double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj){};
        virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel){};
        virtual double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad){};
        virtual Eigen::Matrix3d getonlyGrad2(const Eigen::RowVector3d &pos_rel){};
        BasicShape(const Config &conf) : config(conf)
        {
            kernel_count = conf.kernel_yaw_num;
            kernelsize = conf.kernel_size;
            kernelresu = conf.occupancy_resolution;

            num_sdf_map_res = 0.1;
            num_sdf_map_xlength = num_sdf_map_ylength = 1.0;
            num_sdf_map_X_size = num_sdf_map_Y_size = 10;
            
            std::vector<double> para;
            para = config.poly_params;
            assert(para.size() == 3 && " set the para size==3");
            string input = ros::package::getPath(string("plan_manager")) + "/" + config.inputdata;
            igl::read_triangle_mesh(input, V, F); // read V and F from obj
            ROS_WARN_STREAM("use input obj file as analytic shape");
            trans = Eigen::Vector3d(para[0], para[1], 0.0);
            yaw = (para[2] * PI / 180.0);
            Rotate.setZero();
            Rotate(0, 0) = std::cos(yaw);
            Rotate(0, 1) = -std::sin(yaw);
            Rotate(1, 0) = std::sin(yaw);
            Rotate(1, 1) = std::cos(yaw);
            Rotate(2, 2) = 1;

            Eigen::Matrix4d Trans{Eigen::Matrix4d::Identity()};
            Trans.block(0, 0, 3, 3) = Rotate;
            Eigen::Matrix4d Transorig{Eigen::Matrix4d::Identity()};
            Trans.block(0, 3, 3, 1) = trans;
            const Eigen::Matrix4d TT = Trans.cast<double>().transpose();
            V = (V.rowwise().homogeneous() * TT).rowwise().hnormalized(); // x3d rotation transform
            vertices = V.transpose();
            mesh.conservativeResize(3, 3 * F.rows());
            int faces = F.rows();
            for (int i = 0; i < faces; i++)
            {
                mesh.col(3 * i) = V.row(F.row(i)[0]);
                mesh.col(3 * i + 1) = V.row(F.row(i)[1]);
                mesh.col(3 * i + 2) = V.row(F.row(i)[2]);
            }
        tree.init(V, F);
        igl::fast_winding_number(V, F, 2, fwn_bvh);
  
        }
        
        void inline writetoobjfile(const std::string& output_path)
        {
              V_var=vertices_var.transpose();
              igl::write_triangle_mesh(output_path, V_var, F);
        }

        void inline Transform(const Matrix3d &R, const Vector3d &trans_ = Vector3d::Zero(), const Matrix3d &St = Matrix3d::Identity())
        {
            trans = trans_;
            Rotate = R;
            vertices_var = (St * R * vertices).colwise() + trans;
            mesh_var = (St * R * mesh).colwise() + trans;
            interior_var = trans;
        }

    
        virtual inline double getonlySDF_igl(const Eigen::RowVector3d &point_relative)
        {
            int i;                                                     
            Eigen::RowVector3d c;                                      
            Eigen::VectorXd w{Eigen::VectorXd::Zero(1)};               
            igl::fast_winding_number(fwn_bvh, 2.0, point_relative, w); 
            double s = 1. - 2. * w(0);
            return s * sqrt(tree.squared_distance(V, F, point_relative, i, c)); 
        }

        virtual void inline setStartEnd(const Matrix3d &R, const Vector3d &trans_, const Matrix3d &St = Matrix3d::Identity(),bool start=true)
        {
            trans = trans_;
            Rotate = R;
            vertices_var = (St * R * vertices).colwise() + trans;
            if(start)
            {
            V_start=vertices_var.transpose();
            }
            else
            {
             V_end=vertices_var.transpose();
            }
        }

        void getTransform(Matrix3d &R, RowVector3d &trans_)
        {
            R = Rotate;
            trans_ = trans.transpose();
            interior_var = trans;
        }
        ~BasicShape()
        {
            std::cout << "==========delete BasicShape=========" << std::endl;
            delete[] num_sdf_map;
            delete[] shape_kernels;
            delete[] byte_shape_kernels;
        }


        vector<double> getbound()
        {
            vector<double> bounds;
            bounds.clear();
            double xmax = x_min + (num_sdf_map_X_size - 1) * num_sdf_map_res;
            double ymax = y_min + (num_sdf_map_Y_size - 1) * num_sdf_map_res;
            bounds.emplace_back(x_min);
            bounds.emplace_back(y_min);
            bounds.emplace_back(xmax);
            bounds.emplace_back(ymax);
            return bounds;
        }


        void initShape(const double ndx, const double ndy, const double nres = 0.1)
        {
            std::cout << "==============enter initShape==============" << std::endl;
            std::cout << "==============calculate self kernels==============" << std::endl;
            int zero_yaw_ind = (kernel_count) / 2;
            int size_side = 0.5 * (kernelsize - 1);
            double yaw_, x, y, sdf;
            Eigen::Matrix3d rot_mat;
            Eigen::RowVector3d pos_ori;
            shape_kernels = new Shapekernel[kernel_count];
            byte_shape_kernels = new ByteShapeKernel[kernel_count];
            int ind;
            ind = 0;
            double safemargin = max(config.front_end_safeh, config.occupancy_resolution / 2);
            double yaw_res = 2 * PI / kernel_count;
            for (double yaw = -PI; yaw < PI; yaw += yaw_res, ind++)
            {
                shape_kernels[ind].init(kernelsize, kernelsize);
                rot_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                shape_kernels[ind].yaw = yaw;
                byte_shape_kernels[ind].yaw = yaw;

                for (int a = 0; a < kernelsize; a++)
                {
                    for (int b = 0; b < kernelsize; b++)
                    {
                        x = kernelresu * a - size_side * kernelresu;
                        y = kernelresu * b - size_side * kernelresu;
                        pos_ori(0) = x;
                        pos_ori(1) = y;
                        pos_ori(2) = 0.0;
                        sdf = getonlySDF(pos_ori, rot_mat); 

                        if (sdf <= safemargin) 
                        {
                            shape_kernels[ind].map[a * kernelsize + b] = true;
                        }
                    }
                }
                byte_shape_kernels[ind].generateByteKernel(kernelsize, shape_kernels[ind]); 
                std::cout << " Byte kernel " << ind << " init done. " << std::endl;
            }
            initselfkerneldone = true;
        }
    };

    class Circle : public BasicShape
    {
#define CIRCLE_EDGE_NUM 30
    public:
        double radius;
        Eigen::MatrixX3d edge_start;
        Eigen::MatrixX3d edge_end;
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};

    public:
        Circle(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            radius = 1.0;
            edge_start.resize(CIRCLE_EDGE_NUM, 3);
            edge_end.resize(CIRCLE_EDGE_NUM, 3);
            for (int i = 0; i < CIRCLE_EDGE_NUM; i++)
            {
                Eigen::Vector3d start, end;
                start = Eigen::Vector3d(radius * cos(i * 2 * PI / CIRCLE_EDGE_NUM), radius * sin(i * 2 * PI / CIRCLE_EDGE_NUM), 1);
                end = Eigen::Vector3d(radius * cos((i + 1) * 2 * PI / CIRCLE_EDGE_NUM), radius * sin((i + 1) * 2 * PI / CIRCLE_EDGE_NUM), 1);
                edge_start.row(i) = start;
                edge_end.row(i) = end;
            }
        }
        Circle(double r, double num_res, Config &conf) : BasicShape(conf)
        {
            radius = r;
            initShape(2 * r, 2 * r, num_res);
            edge_start.resize(CIRCLE_EDGE_NUM, 3);
            edge_end.resize(CIRCLE_EDGE_NUM, 3);
            for (int i = 0; i < CIRCLE_EDGE_NUM; i++)
            {
                Eigen::Vector3d start, end;
                start = Eigen::Vector3d(radius * cos(i * 2 * PI / CIRCLE_EDGE_NUM), radius * sin(i * 2 * PI / CIRCLE_EDGE_NUM), 1);
                end = Eigen::Vector3d(radius * cos((i + 1) * 2 * PI / CIRCLE_EDGE_NUM), radius * sin((i + 1) * 2 * PI / CIRCLE_EDGE_NUM), 1);
                edge_start.row(i) = start;
                edge_end.row(i) = end;
            }
        }

    public:
        double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2); 
            return (Pos_rel).norm() - radius;
        }
        double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            return (Pos_rel).norm() - radius;
        }

        Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)
        {
       
            Eigen::Vector2d pos_rel2D = ((pos_rel - trans) * Rotate).head(2);
            pos_rel2D.normalize();
            Eigen::Vector3d grad1;
            grad1.setZero();
            grad1.head(2) = pos_rel2D;
            return grad1.transpose();
        }
        double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
        {
            Eigen::Vector2d pos_rel2D = ((pos_rel - trans) * Rotate).head(2);
            pos_rel2D.normalize();
            grad.setZero();
            grad.head(2) = pos_rel2D;
            return (pos_rel.head(2)).norm() - radius;
        }
        Eigen::Matrix3d getonlyGrad2(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::Matrix3d ret;
            ret.setZero();
            return ret;
        };
    };

    class sdUnevenCapsule : public BasicShape
    {
    public:
        const double r1{2.0};
        const double r2{1.0};
        const double h{5.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdUnevenCapsule(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdUnevenCapsule> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double b = (r1 - r2) / h;
            double a = std::sqrt(1.0 - b * b);
            double k = Pos_rel.dot(Vector2d(-b, a));
            if (k < 0.0)
                return Pos_rel.norm() - r1;
            if (k > a * h)
                return (Pos_rel - Vector2d(0.0, h).transpose()).norm() - r2;
            return Pos_rel.dot(Vector2d(a, b)) - r1;
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double b = (r1 - r2) / h;
            double a = std::sqrt(1.0 - b * b);
            double k = Pos_rel.dot(Vector2d(-b, a));
            if (k < 0.0)
                return Pos_rel.norm() - r1;
            if (k > a * h)
                return (Pos_rel - Vector2d(0.0, h).transpose()).norm() - r2;
            return Pos_rel.dot(Vector2d(a, b)) - r1;
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdUnevenCapsule(){};
    };

    class star : public BasicShape
    {
    public:
        const double r{2.8};
        const double rf{0.6};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        star(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<star> Ptr;
        inline double clip(double value, double min_val, double max_val)
        {
            return std::max(std::min(value, max_val), min_val);
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);

            const Eigen::RowVector2d k1(0.809016994375, -0.587785252292);
            const Eigen::RowVector2d k2(-k1.x(), k1.y());
            Pos_rel.x() = std::abs(Pos_rel.x());
            Pos_rel -= 2.0 * std::max(k1.dot(Pos_rel), 0.0) * k1;
            Pos_rel -= 2.0 * std::max(k2.dot(Pos_rel), 0.0) * k2;
            Pos_rel.x() = std::abs(Pos_rel.x());
            Pos_rel.y() -= r;

            Eigen::RowVector2d ba = rf * Eigen::RowVector2d(-k1.y(), k1.x()) - Eigen::RowVector2d(0, 1);
            double h = clip(Pos_rel.dot(ba) / ba.dot(ba), 0.0, r);
            Eigen::RowVector2d diff = Pos_rel - ba * h;

            return diff.norm() * std::copysign(1.0, Pos_rel.y() * ba.x() - Pos_rel.x() * ba.y());
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            const Eigen::RowVector2d k1(0.809016994375, -0.587785252292);
            const Eigen::RowVector2d k2(-k1.x(), k1.y());
            Pos_rel.x() = std::abs(Pos_rel.x());
            Pos_rel -= 2.0 * std::max(k1.dot(Pos_rel), 0.0) * k1;
            Pos_rel -= 2.0 * std::max(k2.dot(Pos_rel), 0.0) * k2;
            Pos_rel.x() = std::abs(Pos_rel.x());
            Pos_rel.y() -= r;

            Eigen::RowVector2d ba = rf * Eigen::RowVector2d(-k1.y(), k1.x()) - Eigen::RowVector2d(0, 1);
            double h = clip(Pos_rel.dot(ba) / ba.dot(ba), 0.0, r);
            Eigen::RowVector2d diff = Pos_rel - ba * h;

            return diff.norm() * std::copysign(1.0, Pos_rel.y() * ba.x() - Pos_rel.x() * ba.y());
        }
        DEFINE_USEFUL_FUNCTION()
        ~star(){};
    };

    class sdTunnel : public BasicShape
    {
    public:
        const  Eigen::Vector2d wh{2.5,1.5};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdTunnel(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdTunnel> Ptr;


        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d p = ((pos_rel - trans) * Rotate).head(2);
           
            p.x() = std::abs(p.x());
            p.y() = -p.y();

            Eigen::Vector2d q = p.transpose() - wh;

            double d1 = std::pow(std::max(q.x(), 0.0), 2) + q.y() * q.y();
            q.x() = (p.y() > 0.0) ? q.x() : std::sqrt(p.x() * p.x() + p.y() * p.y()) - wh.x();
            double d2 = std::pow(q.x(), 2) + std::pow(std::max(q.y(), 0.0), 2);
            double d = std::sqrt(std::min(d1, d2));

            return (std::max(q.x(), q.y()) < 0.0) ? -d : d;

        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d p = ((pos_rel - trans) * Rotate * R_obj).head(2);
            p.x() = std::abs(p.x());
            p.y() = -p.y();

            Eigen::Vector2d q = p.transpose() - wh;

            double d1 = std::pow(std::max(q.x(), 0.0), 2) + q.y() * q.y();
            q.x() = (p.y() > 0.0) ? q.x() : std::sqrt(p.x() * p.x() + p.y() * p.y()) - wh.x();
            double d2 = std::pow(q.x(), 2) + std::pow(std::max(q.y(), 0.0), 2);
            double d = std::sqrt(std::min(d1, d2));

            return (std::max(q.x(), q.y()) < 0.0) ? -d : d;
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdTunnel(){};
    };

    class sdCutDisk : public BasicShape
    {
    public:
        const  double r{5.0};
        const  double h{2.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdCutDisk(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdCutDisk> Ptr;


        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
         Eigen::RowVector2d p = ((pos_rel - trans) * Rotate).head(2);
           
         const double w = std::sqrt(r*r - h*h); // constant for a given shape
         p.x()=std::abs(p.x());
         // select circle or segment
         double s = std::max((h - r) * p.x()*p.x() + w*w*(h + r - 2.0*p.y()), h*p.x() - w*p.y());
    
         return (s < 0.0) ? p.norm() - r :        // circle
         (p.x() < w) ? h - p.y() :     // segment line
         (p - Eigen::RowVector2d(w, h)).norm(); // segment corner

        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
        Eigen::RowVector2d p = ((pos_rel - trans) * Rotate * R_obj).head(2);
         const double w = std::sqrt(r*r - h*h); // constant for a given shape
         p.x()=std::abs(p.x());
         // select circle or segment
         double s = std::max((h - r) * p.x()*p.x() + w*w*(h + r - 2.0*p.y()), h*p.x() - w*p.y());
    
         return (s < 0.0) ? p.norm() - r :        // circle
         (p.x() < w) ? h - p.y() :     // segment line
         (p - Eigen::RowVector2d(w, h)).norm(); // segment corner
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdCutDisk(){};
    };

    class sdTrapezoid : public BasicShape
    {
    public:
    // 1.0,3.0,2.0
        const  double r1{1.0};
        const  double r2{3.0};
        const  double he{2.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdTrapezoid(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdTrapezoid> Ptr;

        inline double clip(double value, double min_val, double max_val)
        {
            return std::max(std::min(value, max_val), min_val);
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
         Eigen::RowVector2d p = ((pos_rel - trans) * Rotate).head(2);
         Eigen::RowVector2d k1(r2, he);
         Eigen::RowVector2d k2(r2 - r1, 2.0 * he);
         p.x()=std::abs(p.x());
         Eigen::RowVector2d ca(std::max(0.0, p.x() - ((p.y() < 0.0) ? r1 : r2)), std::abs(p.y()) - he);
         Eigen::RowVector2d cb = p - k1 + k2 * clip((k1 - p).dot(k2) / k2.squaredNorm(), 0.0, 1.0);

         double s = (cb.x() < 0.0 && ca.y() < 0.0) ? -1.0 : 1.0;
         return s * std::sqrt(std::min(ca.squaredNorm(), cb.squaredNorm()));


        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
         Eigen::RowVector2d p = ((pos_rel - trans) * Rotate * R_obj).head(2);
         Eigen::RowVector2d k1(r2, he);
         Eigen::RowVector2d k2(r2 - r1, 2.0 * he);
         p.x()=std::abs(p.x());
         Eigen::RowVector2d ca(std::max(0.0, p.x() - ((p.y() < 0.0) ? r1 : r2)), std::abs(p.y()) - he);
         Eigen::RowVector2d cb = p - k1 + k2 * clip((k1 - p).dot(k2) / k2.squaredNorm(), 0.0, 1.0);

         double s = (cb.x() < 0.0 && ca.y() < 0.0) ? -1.0 : 1.0;
         return s * std::sqrt(std::min(ca.squaredNorm(), cb.squaredNorm()));

        }
        DEFINE_USEFUL_FUNCTION()
        ~sdTrapezoid(){};
    };

    class sdRhombus : public BasicShape
    {
    public:
    // 1.0,3.0,2.0
        const  Eigen::RowVector2d b{1.0,4.5};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdRhombus(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdRhombus> Ptr;

        inline double clip(double value, double min_val, double max_val)
        {
            return std::max(std::min(value, max_val), min_val);
        }
        inline double ndot(const RowVector2d& a, const RowVector2d& b ) { return a.x()*b.x() - a.y()*b.y(); }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
         Eigen::RowVector2d p = ((pos_rel - trans) * Rotate).head(2);
         p.x()=std::abs(p.x());
         p.y()=std::abs(p.y());
        
         Eigen::RowVector2d modifiedB = b - 2.0 * p;

         double dotProduct = b.dot(b);
         double h = clip(ndot(modifiedB, b) / dotProduct, -1.0, 1.0);
         Eigen::RowVector2d bHalf = 0.5 * b;
         Eigen::RowVector2d vectorH = Eigen::Vector2d(1.0 - h, 1.0 + h);
         double d = (p - bHalf.cwiseProduct(vectorH)).norm();

         double signTerm = std::signbit(p.x() * b.y() + p.y() * b.x() - b.x() * b.y()) ? -1.0 : 1.0;
         return d * signTerm;

        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
         Eigen::RowVector2d p = ((pos_rel - trans) * Rotate * R_obj).head(2);
         p.x()=std::abs(p.x());
         p.y()=std::abs(p.y());
        
         Eigen::RowVector2d modifiedB = b - 2.0 * p;

         double dotProduct = b.dot(b);
         double h = clip(ndot(modifiedB, b) / dotProduct, -1.0, 1.0);
         Eigen::RowVector2d bHalf = 0.5 * b;
         Eigen::RowVector2d vectorH = Eigen::Vector2d(1.0 - h, 1.0 + h);
         double d = (p - bHalf.cwiseProduct(vectorH)).norm();

         double signTerm = std::signbit(p.x() * b.y() + p.y() * b.x() - b.x() * b.y()) ? -1.0 : 1.0;
         return d * signTerm;

        }
        DEFINE_USEFUL_FUNCTION()
        ~sdRhombus(){};
    };

    class sdHorseshoe : public BasicShape
    {
    public:
    // math.cos(20),math.sin(20)),0.5,(0.55,0.10)
        const double r{1.5};
        const Eigen::Vector2d c{cos(20.5),sin(20.5)};
        const Eigen::Vector2d w{1.55,0.20};

        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdHorseshoe(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdHorseshoe> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel.x()=std::abs(Pos_rel.x());
            double l=Pos_rel.norm(); 
            double px=Pos_rel.x(); double py=Pos_rel.y();         // [-cx cy][px]
            Pos_rel.x()=-c.x()*px+c.y()*py;                      // [cy  cx][py]
            Pos_rel.y()= c.y()*px+c.x()*py;  
            px=Pos_rel.x();
            if(px<=0&&Pos_rel.y()<=0)
            {
                Pos_rel.x()=l*std::copysign(1.0f, - c.x());
            }
            if(px<=0)
            {
                Pos_rel.y()=l;
            }
             Pos_rel.x()= Pos_rel.x()-w.x();
             Pos_rel.y()= std::abs(Pos_rel.y()-r)-w.y();
            Vector2d temp{std::max(Pos_rel.x(),0.0),std::max(Pos_rel.y(),0.0)};
            return temp.norm()+std::min(0.0,std::max(Pos_rel.x(),Pos_rel.y()));
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel.x()=std::abs(Pos_rel.x());
            double l=Pos_rel.norm(); 
            double px=Pos_rel.x(); double py=Pos_rel.y();         // [-cx cy][px]
            Pos_rel.x()=-c.x()*px+c.y()*py;                      // [cy  cx][py]
            Pos_rel.y()= c.y()*px+c.x()*py;  
            px=Pos_rel.x();
            if(px<=0&&Pos_rel.y()<=0)
            {
                Pos_rel.x()=l*std::copysign(1.0f, - c.x());
            }
            if(px<=0)
            {
                Pos_rel.y()=l;
            }
             Pos_rel.x()= Pos_rel.x()-w.x();
             Pos_rel.y()= std::abs(Pos_rel.y()-r)-w.y();
            Vector2d temp{std::max(Pos_rel.x(),0.0),std::max(Pos_rel.y(),0.0)};
            return temp.norm()+std::min(0.0,std::max(Pos_rel.x(),Pos_rel.y()));
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdHorseshoe(){};
    };

    class sdHeart : public BasicShape
    {
    public:
    // math.cos(20),math.sin(20)),0.5,(0.55,0.10)
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdHeart(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdHeart> Ptr;
        inline double dot2(const Eigen::RowVector2d &v) {
            return v.dot(v);
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
             
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);//scale 4
            Pos_rel=Pos_rel/4.0;//scale 4
            Pos_rel.x()=std::abs(Pos_rel.x());
            if (Pos_rel.y() + Pos_rel.x() > 1.0)
                return 4*(std::sqrt(dot2(Pos_rel - Eigen::RowVector2d(0.25, 0.75))) - std::sqrt(2.0) / 4.0);//scale 4
            
            double value1 = dot2(Pos_rel - Eigen::RowVector2d(0.0, 1.0));
            double temp=std::max(Pos_rel.x() + Pos_rel.y(), 0.0);
            double value2 = dot2(Pos_rel - 0.5 * Eigen::RowVector2d(temp,temp));
            return 4*(std::sqrt(std::min(value1, value2)) * std::copysign(1.0, Pos_rel.x() - Pos_rel.y()));
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel- trans) * Rotate * R_obj).head(2);//scale 4
            Pos_rel=Pos_rel/4.0;//scale 4
            Pos_rel.x()=std::abs(Pos_rel.x());
            if (Pos_rel.y() + Pos_rel.x() > 1.0)
                return 4*(std::sqrt(dot2(Pos_rel - Eigen::RowVector2d(0.25, 0.75))) - std::sqrt(2.0) / 4.0);//scale 4
            
            double value1 = dot2(Pos_rel - Eigen::RowVector2d(0.0, 1.0));
            double temp=std::max(Pos_rel.x() + Pos_rel.y(), 0.0);
            double value2 = dot2(Pos_rel - 0.5 * Eigen::RowVector2d(temp,temp));
            return 4*(std::sqrt(std::min(value1, value2)) * std::copysign(1.0, Pos_rel.x() - Pos_rel.y()));
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdHeart(){};
    };

    class sdRoundedX : public BasicShape
    {
    public:
        const double w{3.0};
        const double r{0.25};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdRoundedX(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdRoundedX> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Eigen::RowVector2d absP = Pos_rel.array().abs();
            double minValue = (absP.x() + absP.y() > w) ? (w * 0.5f) : (absP.x() + absP.y()) * 0.5f;
            return (absP - RowVector2d{minValue, minValue}).norm() - r;
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Eigen::RowVector2d absP = Pos_rel.array().abs();
            double minValue = (absP.x() + absP.y() > w) ? (w * 0.5f) : (absP.x() + absP.y()) * 0.5f;
            return (absP - RowVector2d{minValue, minValue}).norm() - r;
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdRoundedX(){};
    };

    class bigX : public BasicShape
    {
    public:
        const double w{5.0};
        const double r{0.25};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        bigX(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<bigX> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Eigen::RowVector2d absP = Pos_rel.array().abs();
            double minValue = (absP.x() + absP.y() > w) ? (w * 0.5f) : (absP.x() + absP.y()) * 0.5f;
            return (absP - RowVector2d{minValue, minValue}).norm() - r;
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Eigen::RowVector2d absP = Pos_rel.array().abs();
            double minValue = (absP.x() + absP.y() > w) ? (w * 0.5f) : (absP.x() + absP.y()) * 0.5f;
            return (absP - RowVector2d{minValue, minValue}).norm() - r;
        }
        DEFINE_USEFUL_FUNCTION()
        ~bigX(){};
    };

    class sdRoundedCross : public BasicShape
    {
    public:
        const double h{1.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdRoundedCross(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdRoundedCross> Ptr;
        double dot2(const Eigen::Vector2d& v) {
            return v.dot(v);
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel=Pos_rel/2.0;//scale 2
            double k = 0.5 * (h + 1.0 / h);
             Eigen::RowVector2d abs_p = Pos_rel.cwiseAbs();

            if (abs_p.x() < 1.0 && abs_p.y() < abs_p.x() * (k - h) + h) {
                return 2*(k - std::sqrt(dot2(abs_p - Eigen::RowVector2d(1, k))));//scale 4
            } else {
                return 2*std::sqrt(std::min(dot2(abs_p - Eigen::RowVector2d(0, h)), 
                                        dot2(abs_p - Eigen::RowVector2d(1, 0))));//scale 2
            }
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel=Pos_rel/2.0;//scale 4
            double k = 0.5 * (h + 1.0 / h);
             Eigen::RowVector2d abs_p = Pos_rel.cwiseAbs();

            if (abs_p.x() < 1.0 && abs_p.y() < abs_p.x() * (k - h) + h) {
                return 2*(k - std::sqrt(dot2(abs_p - Eigen::RowVector2d(1, k))));//scale 2
            } else {
                return 2*std::sqrt(std::min(dot2(abs_p - Eigen::RowVector2d(0, h)), 
                                        dot2(abs_p - Eigen::RowVector2d(1, 0))));//scale 2
            }
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdRoundedCross(){};
    };
    class sdOrientedVesica : public BasicShape
    {
    public:
        const Vector2d a{2,4};
        const Vector2d b{-2,-4};
        const double w{0.8};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdOrientedVesica(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdOrientedVesica> Ptr;
        double dot2(const Eigen::Vector2d& v) {
            return v.dot(v);
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel=Pos_rel/1.0;//scale 1.5
            // Calculate r
            double r = 0.5 * (b - a).norm();

            // Calculate d
            double d = 0.5 * (r * r - w * w) / w;

            // Calculate vector v
            Eigen::Vector2d v = (b - a) / r;

            // Calculate point c
            Eigen::Vector2d c = 0.5 * (b + a);

            // Create rotation matrix and calculate q
            Eigen::Matrix2d rotation;
            rotation << v.y(), v.x(), -v.x(), v.y();
            Eigen::Vector2d q = 0.5 * (rotation * (Pos_rel.transpose() - c)).cwiseAbs();

            // Create vector h based on condition
            Eigen::Vector3d h;
            if (r * q.x() < d * (q.y() - r)) {
                h = Eigen::Vector3d(0.0, r, 0.0);
            } else {
                h = Eigen::Vector3d(-d, 0.0, d + w);
            }

            // Return final distance
            return 1.0*((q - h.head<2>()).norm() - h.z());//scale 1
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel=Pos_rel/1.0;//scale 1
            // Calculate r
            double r = 0.5 * (b - a).norm();

            // Calculate d
            double d = 0.5 * (r * r - w * w) / w;

            // Calculate vector v
            Eigen::Vector2d v = (b - a) / r;

            // Calculate point c
            Eigen::Vector2d c = 0.5 * (b + a);

            // Create rotation matrix and calculate q
            Eigen::Matrix2d rotation;
            rotation << v.y(), v.x(), -v.x(), v.y();
            Eigen::Vector2d q = 0.5 * (rotation * (Pos_rel.transpose() - c)).cwiseAbs();

            // Create vector h based on condition
            Eigen::Vector3d h;
            if (r * q.x() < d * (q.y() - r)) {
                h = Eigen::Vector3d(0.0, r, 0.0);
            } else {
                h = Eigen::Vector3d(-d, 0.0, d + w);
            }

            // Return final distance
            return 1.0*((q - h.head<2>()).norm() - h.z());//scale 2
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdOrientedVesica(){};
    };

    class sdMoon : public BasicShape
    {
    public:
        const double d{0.8};
        const double ra{3.0};
        const double rb{2.4};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdMoon(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdMoon> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d q = ((pos_rel - trans) * Rotate).head(2);
            q.y() = std::abs(q.y());
            double a = (ra * ra - rb * rb + d * d) / (2.0 * d);
            double b = std::sqrt(std::max(ra * ra - a * a, 0.0));
            bool condition = d * (q.x() * b - q.y() * a) > d * d * std::max(b - q.y(), 0.0);
            double dist1 = (q - RowVector2d(a, b)).norm();
            double dist2 = std::max(
                q.norm() - ra,
                -(q - RowVector2d(d, 0.0)).norm() + rb);
            return condition ? dist1 : dist2;
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d q = ((pos_rel - trans) * Rotate * R_obj).head(2);
            q.y() = std::abs(q.y());
            double a = (ra * ra - rb * rb + d * d) / (2.0 * d);
            double b = std::sqrt(std::max(ra * ra - a * a, 0.0));
            bool condition = d * (q.x() * b - q.y() * a) > d * d * std::max(b - q.y(), 0.0);
            double dist1 = (q - RowVector2d(a, b)).norm();
            double dist2 = std::max(
                q.norm() - ra,
                -(q - RowVector2d(d, 0.0)).norm() + rb);
            return condition ? dist1 : dist2;
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdMoon(){};
    };
    class sdPie : public BasicShape
    {
    public:
        const Eigen::RowVector2d c{RowVector2d(std::cos(43), std::sin(43))};
        const double r{3.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdPie(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdPie> Ptr;
        inline double clip(double value, double min_val, double max_val)
        {
            return std::max(std::min(value, max_val), min_val);
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double l = Pos_rel.norm() - r;
            double m = (Pos_rel - c * clip(Pos_rel.dot(c), 0.0, r)).norm();
            return std::max(l, m * std::copysign(1.0f, c.y() * Pos_rel.x() - c.x() * Pos_rel.y()));
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double l = Pos_rel.norm() - r;
            double m = (Pos_rel - c * clip(Pos_rel.dot(c), 0.0, r)).norm();
            return std::max(l, m * std::copysign(1.0f, c.y() * Pos_rel.x() - c.x() * Pos_rel.y()));
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdPie(){};
    };
    class sdPie2 : public BasicShape
    {
    public:
        const Eigen::RowVector2d c{RowVector2d(std::cos(1), std::sin(1))};
        const double r{3.0};
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        sdPie2(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdPie2> Ptr;
        inline double clip(double value, double min_val, double max_val)
        {
            return std::max(std::min(value, max_val), min_val);
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double l = Pos_rel.norm() - r;
            double m = (Pos_rel - c * clip(Pos_rel.dot(c), 0.0, r)).norm();
            return std::max(l, m * std::copysign(1.0f, c.y() * Pos_rel.x() - c.x() * Pos_rel.y()));
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel(0) = std::abs(Pos_rel(0));
            double l = Pos_rel.norm() - r;
            double m = (Pos_rel - c * clip(Pos_rel.dot(c), 0.0, r)).norm();
            return std::max(l, m * std::copysign(1.0f, c.y() * Pos_rel.x() - c.x() * Pos_rel.y()));
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdPie2(){};
    };

    class sdArc : public BasicShape
    {
    public:
        Eigen::RowVector3d trans{Eigen::RowVector3d::Zero()};
        Eigen::Matrix3d Rotate{Eigen::Matrix3d::Identity()};
        const Eigen::Vector2d sc{Eigen::Vector2d(std::sin(20), std::cos(20))};
        const double ra{2.3333};
        const double rb{0.5};

        sdArc(const Config &conf) : BasicShape(conf)
        {
            getTransform(Rotate, trans);
            std::cout << "======Rotation:" << Rotate << "======" << std::endl;
            std::cout << "======trans:" << trans.transpose() << "======" << std::endl;
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        typedef std::shared_ptr<sdArc> Ptr;
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate).head(2);
            Pos_rel.x() = std::abs(Pos_rel.x());
            bool condition = sc.y() * Pos_rel.x() > sc.x() * Pos_rel.y();
            double dist1 = (Pos_rel - sc.transpose() * ra).norm();
            double dist2 = std::abs(Pos_rel.norm() - ra);
            double result = (condition ? dist1 : dist2) - rb;
            return result;
        }

        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix3d &R_obj)
        {
            Eigen::RowVector2d Pos_rel = ((pos_rel - trans) * Rotate * R_obj).head(2);
            Pos_rel.x() = std::abs(Pos_rel.x());
            bool condition = sc.y() * Pos_rel.x() > sc.x() * Pos_rel.y();
            double dist1 = (Pos_rel - sc.transpose() * ra).norm();
            double dist2 = std::abs(Pos_rel.norm() - ra);
            double result = (condition ? dist1 : dist2) - rb;
            return result;
        }
        DEFINE_USEFUL_FUNCTION()
        ~sdArc(){};
    };

   

    class Polygon : public BasicShape
    {
    public:
        typedef class PolygonEdge
        {
        public:
            Eigen::Vector2d start;
            Eigen::Vector2d end;
            PolygonEdge(const Eigen::Vector2d &s, const Eigen::Vector2d &e) : start(s), end(e) {}
            inline bool isCrossRayOnXDir(Eigen::Vector2d source)
            {
                Eigen::Vector2d s2p = start - source;
                Eigen::Vector2d e2p = end - source;
                double theta_s = atan2(s2p(1), s2p(0));
                double theta_e = atan2(e2p(1), e2p(0));
                theta_s = (theta_s < 0.0) ? (theta_s + 2 * PI) : theta_s;
                theta_e = (theta_e < 0.0) ? (theta_e + 2 * PI) : theta_e;
                double d1 = abs(theta_s - theta_e);
                if (d1 < PI)
                    return false;
                else
                    return true;
            }
            inline double dis2Seg(const Eigen::Vector2d &p, Eigen::Vector2d &c)
            {
                Eigen::Vector2d v = end - start;
                Eigen::Vector2d w = p - start;
                double t = w.dot(v) / (v.squaredNorm());
                if (t < 0.0)
                {
                    t = 0.0;
                }
                else if (t > 1.0)
                {
                    t = 1.0;
                }

                c = start + t * v;
                return (p - c).norm();
            }
        } Edge;

    public:
        Eigen::MatrixX3d edge_start;
        Eigen::MatrixX3d edge_end;
        std::vector<Edge> edges; // 可视化不会访问到
    public:
        Polygon(const Config &conf) : BasicShape(conf)
        {

#define POLYGON_DEFAULT_EDGES 7
            edge_start.resize(POLYGON_DEFAULT_EDGES, 3);
            edge_end.resize(POLYGON_DEFAULT_EDGES, 3);
            Eigen::Vector2d edge_s, edge_e;
            double dangle = 2 * PI / POLYGON_DEFAULT_EDGES;
            for (int i = 0; i < POLYGON_DEFAULT_EDGES; i++)
            {
                edge_s = Eigen::Vector2d(4 * cos((i - 0.5) * dangle), 4 * sin((i - 0.5) * dangle));
                edge_e = Eigen::Vector2d(4 * cos((i + 0.5) * dangle), 4 * sin((i + 0.5) * dangle));
                Edge e(edge_s, edge_e);
                edges.push_back(e);
                edge_start.row(i) = Eigen::Vector3d(edge_s(0), edge_s(1), 0);
                edge_end.row(i) = Eigen::Vector3d(edge_e(0), edge_e(1), 0);
            }
            initShape(8, 8, conf.occupancy_resolution);
        }

        Polygon(const Config &conf, const Eigen::MatrixX2d &vertexes) : BasicShape(conf)
        {

            const int edge_count = vertexes.rows();
            edge_start.resize(edge_count, 3);
            edge_end.resize(edge_count, 3);
            Eigen::Vector2d edge_s, edge_e;
            for (size_t i = 0; i < edge_count; i++)
            {
                edge_s = vertexes.row(i);
                edge_e = vertexes.row((i + 1) % edge_count);
                Edge e(edge_s, edge_e);
                edges.push_back(e);
                edge_start.row(i) = Eigen::Vector3d(edge_s(0), edge_s(1), 0);
                edge_end.row(i) = Eigen::Vector3d(edge_e(0), edge_e(1), 0);
            }
            initShape(8, 8, conf.occupancy_resolution);
        }

    public:
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel)
        {
            double dis_min = 1e9;
            Eigen::Vector2d prel2(pos_rel(0), pos_rel(1));
            Eigen::Vector2d c;
            Eigen::Vector2d cmin;
            int rs = 0;
            for (size_t i = 0; i < edges.size(); i++)
            {
                double dis = edges[i].dis2Seg(prel2, c);
                if (dis < dis_min)
                {
                    dis_min = dis;
                    cmin = c;
                }
                if (edges[i].isCrossRayOnXDir(prel2))
                {
                    rs++;
                }
            }
            if (rs % 2 == 0)
            {
                return dis_min;
            }
            else
            {
                return -dis_min;
            }
        }
        inline double getonlySDF(const Eigen::RowVector3d &pos_rel, const Eigen::Matrix2d &R_obj)
        {
            std::cout << "getOnlySDF " << std::endl;
            double dis_min = 1e9;
            Eigen::Vector2d c;
            Eigen::Vector2d cmin;
            int rs = 0;
            Eigen::Vector2d pos_rel_rotated = R_obj.transpose() * pos_rel.head(2).transpose();
            for (size_t i = 0; i < edges.size(); i++)
            {
                double dis = edges[i].dis2Seg(pos_rel_rotated, c);
                if (dis < dis_min)
                {
                    dis_min = dis;
                    cmin = c;
                }
                if (edges[i].isCrossRayOnXDir(pos_rel_rotated))
                {
                    rs++;
                }
            }
            if (rs % 2 == 0)
            {
                return dis_min;
            }
            else
            {
                return -dis_min;
            }
        }
      
        inline Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel)
        {
            double dis_min = 1e9;
            Eigen::Vector2d c;
            Eigen::Vector2d cmin;
            int rs = 0;
            for (size_t i = 0; i < edges.size(); i++)
            {
                double dis = edges[i].dis2Seg(pos_rel.transpose().head(2), c);
                if (dis < dis_min)
                {
                    dis_min = dis;
                    cmin = c;
                }
                if (edges[i].isCrossRayOnXDir(pos_rel.transpose().head(2)))
                {
                    rs++;
                }
            }
            if (rs % 2 == 0)
            {
                return (pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
            }
            else
            {
                return -(pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
            }
        }
        inline double getSDFwithGrad1(const Eigen::RowVector3d &pos_rel, Eigen::Vector3d &grad)
        {
            double dis_min = 1e9;
            Eigen::Vector2d c;
            Eigen::Vector2d cmin;
            int rs = 0;
            for (size_t i = 0; i < edges.size(); i++)
            {
                double dis = edges[i].dis2Seg(pos_rel.transpose().head(2), c);
                if (dis < dis_min)
                {
                    dis_min = dis;
                    cmin = c;
                }
                if (edges[i].isCrossRayOnXDir(pos_rel.transpose().head(2)))
                {
                    rs++;
                }
            }
            if (rs % 2 == 0)
            {
                grad = (pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
                return dis_min;
            }
            else
            {
                grad = -(pos_rel.transpose() - Eigen::Vector3d(cmin(0), cmin(1), pos_rel(2))).normalized();
                return -dis_min;
            }
        }
        inline Eigen::Matrix3d getonlyGrad2(const Eigen::RowVector3d &pos_rel)
        {
            Eigen::Matrix3d ret;
            ret.setZero();
            return ret;
        };
    };
};

#endif
