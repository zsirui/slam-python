#ReadME

### pcl库python的简易接口

目前实现的接口如下：

* PointCloud类
* PointXYZRGBA类
* PCLHeader类

### 编译链接方法：

在命令行中输入以下命令将lib_pcl.cpp编译成python可调用的.so库：

```
g++ -c -fPIC lib_pcl.cpp -o lib_pcl.o
g++ -shared -Wl,-soname,lib_pcl.so -o lib_pcl.so  lib_pcl.o -lpython2.7 -lboost_python -lboost_system
```

编译之后，在python命令行中输入以下命令测试：

```
Python 2.7.6 (default, Oct 26 2016, 20:30:19) 
[GCC 4.8.4] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import lib_pcl
>>> PointT = lib_pcl.PointXYZRGBA()
>>> PointT.x = 1
>>> PointT.y = 2
>>> PointT.z = 3
>>> PointT.r = 128
>>> PointT.g = 255
>>> PointT.b = 127
>>> PointT.rgba
4286644095
>>> PointT.a
255
>>> lib_pcl.showList([1,2,3,128,255,127])
p.x 1
p.y 2
p.z 3
p.rgba 4286644095
```

在python命令行中输入help(lib_pcl)：

```
Help on module lib_pcl:

NAME
    lib_pcl

FILE
    /home/ros/Downloads/test/4/lib_pcl.so

CLASSES
    Boost.Python.instance(__builtin__.object)
        PCLHeader
        PointCloud
        PointXYZRGBA
    
    class PCLHeader(Boost.Python.instance)
     |  Method resolution order:
     |      PCLHeader
     |      Boost.Python.instance
     |      __builtin__.object
     |  
     |  Methods defined here:
     |  
     |  __init__(...)
     |      __init__( (object)arg1) -> None :
     |      
     |          C++ signature :
     |              void __init__(_object*)
     |  
     |  __reduce__ = <unnamed Boost.Python function>(...)
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors defined here:
     |  
     |  frame_id
     |  
     |  seq
     |  
     |  stamp
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes defined here:
     |  
     |  __instance_size__ = 40
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors inherited from Boost.Python.instance:
     |  
     |  __dict__
     |  
     |  __weakref__
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes inherited from Boost.Python.instance:
     |  
     |  __new__ = <built-in method __new__ of Boost.Python.class object>
     |      T.__new__(S, ...) -> a new object with type S, a subtype of T
    
    class PointCloud(Boost.Python.instance)
     |  Method resolution order:
     |      PointCloud
     |      Boost.Python.instance
     |      __builtin__.object
     |  
     |  Methods defined here:
     |  
     |  Ptr(...)
     |      Ptr( (PointCloud)arg1) -> PointCloud :
     |      
     |          C++ signature :
     |              boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > Ptr(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue})
     |  
     |  __add__(...)
     |      __add__( (PointCloud)arg1, (PointCloud)arg2) -> object :
     |      
     |          C++ signature :
     |              _object* __add__(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue},pcl::PointCloud<pcl::PointXYZRGBA>)
     |  
     |  __init__(...)
     |      __init__( (object)arg1) -> None :
     |      
     |          C++ signature :
     |              void __init__(_object*)
     |  
     |  __reduce__ = <unnamed Boost.Python function>(...)
     |  
     |  clear(...)
     |      clear( (PointCloud)arg1) -> None :
     |      
     |          C++ signature :
     |              void clear(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue})
     |  
     |  push_back(...)
     |      push_back( (PointCloud)arg1, (PointXYZRGBA)arg2) -> None :
     |      
     |          C++ signature :
     |              void push_back(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue},pcl::PointXYZRGBA)
     |  
     |  resize(...)
     |      resize( (PointCloud)arg1, (int)arg2) -> None :
     |      
     |      
     |          C++ signature :
     |              void resize(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue},unsigned long)
     |  
     |  size(...)
     |      size( (PointCloud)arg1) -> int :
     |      
     |          C++ signature :
     |              unsigned long size(pcl::PointCloud<pcl::PointXYZRGBA> {lvalue})
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors defined here:
     |  
     |  header
     |  
     |  height
     |  
     |  is_dense
     |  
     |  points
     |  
     |  width
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes defined here:
     |  
     |  __instance_size__ = 144
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors inherited from Boost.Python.instance:
     |  
     |  __dict__
     |  
     |  __weakref__
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes inherited from Boost.Python.instance:
     |  
     |  __new__ = <built-in method __new__ of Boost.Python.class object>
     |      T.__new__(S, ...) -> a new object with type S, a subtype of T
    
    class PointXYZRGBA(Boost.Python.instance)
     |  Method resolution order:
     |      PointXYZRGBA
     |      Boost.Python.instance
     |      __builtin__.object
     |  
     |  Methods defined here:
     |  
     |  __init__(...)
     |      __init__( (object)arg1) -> None :
     |      
     |          C++ signature :
     |              void __init__(_object*)
     |  
     |  __reduce__ = <unnamed Boost.Python function>(...)
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors defined here:
     |  
     |  a
     |  
     |  b
     |  
     |  g
     |  
     |  r
     |  
     |  rgba
     |  
     |  x
     |  
     |  y
     |  
     |  z
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes defined here:
     |  
     |  __instance_size__ = 48
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors inherited from Boost.Python.instance:
     |  
     |  __dict__
     |  
     |  __weakref__
     |  
     |  ----------------------------------------------------------------------
     |  Data and other attributes inherited from Boost.Python.instance:
     |  
     |  __new__ = <built-in method __new__ of Boost.Python.class object>
     |      T.__new__(S, ...) -> a new object with type S, a subtype of T

FUNCTIONS
    showList(...)
        showList( (list)arg1) -> None :
        
            C++ signature :
                void showList(boost::python::list)

(END)
```

