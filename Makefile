CXX=llvm-g++
COMPILE=-c -std=c++17 \
-I/usr/local/Cellar/opencv/4.3.0/include/opencv4 \
-I/usr/local/Cellar/eigen/3.3.7/include/eigen3 \
-I/usr/local/Cellar/glm/0.9.9.8/include \
-I/Users/YJ-work/cpp/sdf2d/header
LINK=-L/usr/local/Cellar/opencv/4.3.0/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs \
-lopencv_imgproc
SRC_DIR=/Users/YJ-work/cpp/sdf2d/src

all: main test mpm2d

main: main.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

main.o: $(SRC_DIR)/main.cpp
	$(CXX) $(COMPILE) $^ -o $@


test: test.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

test.o: test.cpp
	$(CXX) $(COMPILE) $^ -o $@

dummy.o: dummy.cpp
	$(CXX) $(COMPILE) $^ -o $@

mpm2d: mpm2d.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

mpm2d.o: mpm2d.cpp
	$(CXX) $(COMPILE) $^ -o $@
