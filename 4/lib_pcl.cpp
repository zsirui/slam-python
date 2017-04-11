#include <iostream>
#include <iomanip>

//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLHeader.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

//boost
#include <boost/python.hpp>
#include <boost/python/list.hpp>

//Eigen
#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace std;
using namespace boost::python;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef boost::shared_ptr < pcl::PointXYZRGBA > PointXYZRGBA_ptr;
typedef boost::shared_ptr < PointCloud::Ptr > PointCloud_ptr;

PointT toPointXYZRGBA(long double x, long double y, long double z, uint8_t r, uint8_t g, uint8_t b);
void showList(const boost::python::list& pyList);

PointT toPointXYZRGBA(long double x, long double y, long double z, uint8_t r, uint8_t g, uint8_t b)
{
	PointT p;
	p.x = x;
	p.y = y;
	p.z = z;
	p.r = r;
	p.g = g;
	p.b = b;
	return p;
}

void showList(const boost::python::list& pyList)
{
	PointT p;
	p = toPointXYZRGBA(boost::python::extract<long double>(pyList[0]), boost::python::extract<long double>(pyList[1]), boost::python::extract<long double>(pyList[2]), boost::python::extract<uint8_t>(pyList[3]), boost::python::extract<uint8_t>(pyList[4]), boost::python::extract<uint8_t>(pyList[5]));
	cout<<"p.x "<<p.x<<endl;
	cout<<"p.y "<<p.y<<endl;
	cout<<"p.z "<<p.z<<endl;
	cout<<"p.rgba "<<p.rgba<<endl;
}

BOOST_PYTHON_MODULE(lib_pcl)
{
	register_ptr_to_python <PointXYZRGBA_ptr>();
	register_ptr_to_python <PointCloud_ptr>();
	def("showList", showList);
	class_<pcl::PCLHeader>("PCLHeader")
		.def_readwrite("seq", &pcl::PCLHeader::seq)
		.def_readwrite("stamp", &pcl::PCLHeader::stamp)
		.def_readwrite("frame_id", &pcl::PCLHeader::frame_id);
	class_<pcl::PointXYZRGBA>("PointXYZRGBA")
		.def_readwrite("x", &pcl::PointXYZRGBA::x)
		.def_readwrite("y", &pcl::PointXYZRGBA::y)
		.def_readwrite("z", &pcl::PointXYZRGBA::z)
		.def_readwrite("r", &pcl::PointXYZRGBA::r)
		.def_readwrite("g", &pcl::PointXYZRGBA::g)
		.def_readwrite("b", &pcl::PointXYZRGBA::b)
		.def_readwrite("a", &pcl::PointXYZRGBA::a)
		.def_readwrite("rgba", &pcl::PointXYZRGBA::rgba);
	class_<PointCloud>("PointCloud", init<>())
		.def_readwrite("width", &PointCloud::width)
		.def_readwrite("height", &PointCloud::height)
		.def_readwrite("is_dense", &PointCloud::is_dense)
		.def_readwrite("header", &PointCloud::header)
		.def_readwrite("points", &PointCloud::points)
		.def(self + PointCloud())
		.def("size", &PointCloud::size)
		.def("Ptr", &PointCloud::makeShared)
		.def("push_back", &PointCloud::push_back)
		.def("clear", &PointCloud::clear)
		.def("resize", &PointCloud::resize);
}
