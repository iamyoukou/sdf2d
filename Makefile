CXX=llvm-g++
COMPILE=-c -std=c++17 \
-I/usr/local/Cellar/opencv/4.3.0/include/opencv4 \
-I/usr/local/Cellar/eigen/3.3.7/include/eigen3 \
-I/usr/local/Cellar/glm/0.9.9.8/include \
-I/Users/YJ-work/cpp/sdf2d/header
LINK=-L/usr/local/Cellar/opencv/4.3.0/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs \
-lopencv_imgproc
SRC_DIR=/Users/YJ-work/cpp/sdf2d/src

all: main throwingTrack

main: main.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

main.o: $(SRC_DIR)/main.cpp
	$(CXX) $(COMPILE) $^ -o $@

throwingTrack: throwingTrack.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

throwingTrack.o: $(SRC_DIR)/throwingTrack.cpp
	$(CXX) $(COMPILE) $^ -o $@
