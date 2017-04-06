
// Eigen
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <map>

using namespace std;

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "boost/python.hpp"
using namespace boost::python;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
PointCloud::Ptr ToPCL(double x, double y, double z, int r, int g, int b);

PointCloud::Ptr ToPCL(double x, double y, double z, int r, int g, int b)
{
	PointT p;
	PointCloud::Ptr cloud ( new PointCloud );
	p.x = x;
	p.y = y;
	p.z = z;
	p.r = r;
	p.g = g;
	p.b = b;
	cloud->points.push_back( p );
	return cloud;
}

BOOST_PYTHON_MODULE(Test)
{
	def("ToPCL", ToPCL);
}