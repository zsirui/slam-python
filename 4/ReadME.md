`g++ -c -fPIC lib_pcl.cpp -o lib_pcl.o`

`g++ -shared -Wl,-soname,lib_pcl.so -o lib_pcl.so  lib_pcl.o -lpython2.7 -lboost_python -lboost_system`

将lib_pcl.cpp编译成lib_pcl.so之后，在python命令行中输入以下命令测试：

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
