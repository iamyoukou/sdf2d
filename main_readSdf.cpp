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

int main(int argc, char const *argv[]) {

  // read sdf image
  sdfImg = imread("sdf.png");

  vector<Point> object;
  object.push_back(Point(279, 323));
  object.push_back(Point(168, 421));
  object.push_back(Point(286, 540));
  object.push_back(Point(401, 425));

  sdfScale = glm::sqrt(width * width + height * height);

  // set window
  canvas = Mat(height, width, CV_8UC3, WHITE);
  namedWindow(WND_NAME);
  setMouseCallback(WND_NAME, mouseCallback);

  // draw objects
  polylines(canvas, object, true, BLUE);

  imshow(WND_NAME, sdfImg);
  waitKey(0);

  return 0;
}
