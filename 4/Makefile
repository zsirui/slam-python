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
	g++ -shared -Wl,--export-dynamic $(OBJ) -L$(BOOST_LIB) -lboost_python -L$(PYTHON_CONFIG) -lpython2.7 -o $(TARGET).so

%.o : %.cpp
	g++ -I$(PYTHON_INC) -I$(BOOST_INC) -fPIC -o $@ -c $<

clean:
	-rm -f *.o *.so
