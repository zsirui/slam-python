`g++ -c -fPIC lib_pcl.cpp -o lib_pcl.o`

`g++ -shared -Wl,-soname,lib_pcl.so -o lib_pcl.so  lib_pcl.o -lpython2.7 -lboost_python -lboost_system`
