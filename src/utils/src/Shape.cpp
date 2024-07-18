#include <ros/ros.h>
#include <ros/package.h>
#include <utils/Shape.hpp>
#include <utils/config.hpp>
#include <utils/Visualization.hpp>
#include <utils/quickhull.hpp>
#include <utils/geo_utils.hpp>
#include <igl/read_triangle_mesh.h> // 多面体机体可视化采用obj file时候
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
namespace shape
{
    using namespace vis;
}