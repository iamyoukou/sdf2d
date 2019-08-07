CXX=llvm-g++
COMPILE=-c -std=c++11 \
-I/usr/local/Cellar/opencv/4.1.0_2/include/opencv4 \
-I/usr/local/Cellar/eigen/3.3.7/include/eigen3
LINK=-L/usr/local/Cellar/opencv/4.1.0_2/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs \
-lopencv_imgproc

all: main test

main: main.o
	$(CXX) $(LINK) $^ -o $@
	rm -rv *.o

main.o: main.cpp
	$(CXX) $(COMPILE) $^ -o $@

test: test.o
	$(CXX) $(LINK) $^ -o $@

test.o: test.cpp
	$(CXX) $(COMPILE) $^ -o $@
