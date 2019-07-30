#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace glm;
using namespace std;
using namespace cv;

const Scalar WHITE = Scalar(255, 255, 255);
const Scalar BLUE = Scalar(255, 0, 0);
const string WND_NAME = "Test";

const int width = 600, height = 600;
float sdfScale;
float sdfCellSize = 1.f;
Mat canvas;
Mat sdfImg;

float readSdf(float fx, float fy) {
  float sd = 0.f;

  int ix = (int)floor(fx / sdfCellSize);
  int iy = (int)floor(fy / sdfCellSize);

  int color = sdfImg.at<Vec3b>(Point(ix, iy))[0];
  sd = float(color) / 255.f;
  sd = (sd * 2.f - 1.f) * sdfScale;

  return sd;
}

vec2 getGrad(int x, int y) {
  vec2 gd;

  // direction
  std::cout << readSdf(x + 1, y) << ", " << readSdf(x - 1, y) << '\n';
  gd.x = readSdf(x + 1, y) - readSdf(x - 1, y);
  gd.y = readSdf(x, y + 1) - readSdf(x, y - 1);
  // std::cout << gd.x << ", " << gd.y << '\n';
  gd = normalize(gd);

  // length
  gd *= readSdf(x, y);

  return gd;
}

void mouseCallback(int event, int x, int y, int flag, void *param) {
  switch (event) {
  case EVENT_MOUSEMOVE:
    // vec2 grad = getGrad(x, y);
    // Point s(x, y), e(x + grad.x, y + grad.y);
    // arrowedLine(canvas, s, e, BLUE);
    // std::cout << "(" << grad.x << ", " << grad.y << '\n';
    std::cout << readSdf(x, y) << '\n';
    break;
  }
}

int RAND(int down, int up) { return (rand() % (up - down - 1) + down); }

int main(int argc, char const *argv[]) {
  canvas = Mat(width, height, CV_8UC3, Scalar(255, 255, 255));
  Mat letterImg = imread("letter.png");
  string wndName("test");

  int rndSize = 200;
  for (int i = 0; i < rndSize; i++) {
    for (int j = 0; j < rndSize; j++) {
      int x, y;
      x = RAND(0, width - 1);
      y = RAND(0, height - 1);

      Vec3b pixelSrc = letterImg.at<Vec3b>(Point(x, y));

      int color = pixelSrc[0] + pixelSrc[1] + pixelSrc[2];
      // some threshold
      if (color < 10) {
        circle(canvas, Point(x, y), 2, Scalar(255, 0, 0), -1);
      }
    }
  }

  imshow(wndName, canvas);
  waitKey(0);

  return 0;
}
