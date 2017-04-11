PYTHON_VERSION = 2.7
PYTHON_INC = /usr/include/python$(PYTHON_VERSION)
PYTHON_CONFIG = /usr/lib/python2.7/config

BOOST_INC = /usr/include/boost
BOOST_LIB = /usr/lib/x86_64-linux-gnu/libboost_system.so

SRC = $(wildcard *.cpp)
OBJ = $(lib_pcl.o, $(basename $(SRC)))

# this target has to match whatever you have in BOOST_PYTHON_MODULE()
TARGET = lib_pcl

# make this a so library
$(TARGET).so : $(OBJ)
	g++ -c -fPIC $(TARGET).cpp -o $(TARGET).o
	g++ -shared -Wl,-soname,lib_pcl.so -o lib_pcl.so  lib_pcl.o -lpython2.7 -lboost_python -lboost_system

%.o : %.cpp
	g++ -I$(PYTHON_INC) -I$(BOOST_INC) -fPIC -o $@ -c $<

clean:
	-rm -f *.o *.so
