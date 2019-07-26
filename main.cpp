#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define BLUE Scalar(1.f, 0.f, 0.f)
#define RED Scalar(0.f, 0.f, 1.f)
#define WHITE Scalar(1.f, 1.f, 1.f)

using namespace glm;
using namespace std;
using namespace cv;

const int width = 600, height = 600;
float sdfScale;
float sdfCellSize = 1.f;
Mat canvas, oriCanvas;
Mat sdf;

using Polygon = std::vector<Point>;

bool intersect(const Point &p1, const Point &p2, const Point &p3,
               const Point &p4) {
  vec3 a(p1.x, p1.y, 0), b(p2.x, p2.y, 0), c(p3.x, p3.y, 0), d(p4.x, p4.y, 0);

  float crossAcAb, crossAbAd, crossDaDc, crossDcDb;
  crossAcAb = cross(c - a, b - a).z;
  crossAbAd = cross(b - a, d - a).z;
  crossDaDc = cross(a - d, c - d).z;
  crossDcDb = cross(c - d, b - d).z;

  if (crossAcAb * crossAbAd > 0 && crossDaDc * crossDcDb > 0) {
    return true;
  } else {
    return false;
  }
}

bool inside_polygon(const Point &p, const Polygon &polygon) {
  int count = 0;
  static const Point q(123532, 532421123);
  for (int i = 0; i < (int)polygon.size(); i++) {
    count += intersect(p, q, polygon[i], polygon[(i + 1) % polygon.size()]);
  }

  return count % 2 == 1;
}

float nearest_distance(const Point &temp_p, const Polygon &poly) {
  vec2 p(temp_p.x, temp_p.y);

  float dist = 9999.f;

  for (int i = 0; i < poly.size(); i++) {
    const Point &temp_a = poly[i];
    const Point &temp_b = poly[(i + 1) % poly.size()];

    //逆时针遍历多边形的每一条边
    vec2 a(temp_a.x, temp_a.y);
    vec2 b(temp_b.x, temp_b.y);

    //当前处理的边定义为 ab
    vec2 ab = b - a;
    float abLength = glm::length(ab);
    vec2 dir = normalize(ab);

    //求点 p 在边 ab 上的投影点 q
    //于是，线段 pq 的长度就是 p 点的有向距离
    vec2 ap = p - a;
    float frac = dot(ap, dir);
    frac = glm::clamp(frac, 0.f, abLength);
    vec2 q = a + frac * dir;
    vec2 pq = q - p;

    dist = glm::min(dist, length(pq));
  }

  return dist;
}

string wndName = "Test";

float getDistance(vec2 p) { return sdf.at<float>(Point(p.x, p.y)); }

vec2 getGradient(vec2 p) {
  vec2 gd;

  gd.x = getDistance(vec2(p.x + 1.f, p.y)) - getDistance(vec2(p.x - 1.f, p.y));

  // sdf.at<float>(Point(x + 1, y)) - sdf.at<float>(Point(x - 1, y));
  gd.y = getDistance(vec2(p.x, p.y + 1.f)) - getDistance(vec2(p.x, p.y - 1.f));

  // sdf.at<float>(Point(x, y + 1)) - sdf.at<float>(Point(x, y - 1));
  gd = normalize(-gd);

  return gd;
}

void mouseCallback(int event, int x, int y, int flag, void *param) {
  oriCanvas.copyTo(canvas);

  vec2 p(x, y);

  switch (event) {
  case EVENT_MOUSEMOVE:
    // sdf gradient at (x, y)
    vec2 grad = getGradient(p);

    // distance
    float dist = getDistance(p);

    // std::cout << "(" << x << ", " << y << ") "
    //           << "distance = " << dist << ", "
    //           << "gradient = " << grad.x << ", " << grad.y << ")" << '\n';

    // distance and direction to object1 boundary
    float sx, sy, ex, ey;
    sx = (float)x;
    sy = (float)y;
    ex = sx + grad.x * dist;
    ey = sy + grad.y * dist;

    // draw arrow
    arrowedLine(canvas, Point(sx, sy), Point(ex, ey), RED);
    imshow(wndName, canvas);
    waitKey(1);

    // std::cout << "(" << sx << ", " << sy << ") to (" << ex << ", " << ey
    // <<
    // ")"
    //           << '\n';

    break;
  }
}

void printSdf(vec2 p) {
  vec2 gd = getGradient(p);
  std::cout << "(" << p.x << ", " << p.y << ") "
            << "distance = " << getDistance(p) << ", "
            << "gradient = " << gd.x << ", " << gd.y << ")" << '\n';
}

void drawSdf(vec2 p) {
  vec2 gd = getGradient(p);
  float dist = getDistance(p);

  float sx, sy, ex, ey;
  sx = p.x;
  sy = p.y;
  ex = sx + gd.x * dist;
  ey = sy + gd.y * dist;

  arrowedLine(canvas, Point(sx, sy), Point(ex, ey), RED);
}

int main(int argc, char const *argv[]) {
  sdf = Mat::zeros(height, width, CV_32F);
  std::vector<Polygon> polygons;

  Polygon object1;
  object1.push_back(Point(293, 174));
  object1.push_back(Point(208, 248));
  object1.push_back(Point(301, 318));
  object1.push_back(Point(401, 262));

  Polygon object2;
  object2.push_back(Point(59, 325));
  object2.push_back(Point(59, 423));
  object2.push_back(Point(169, 500));
  object2.push_back(Point(265, 444));

  Polygon object3;
  object3.push_back(Point(415, 386));
  object3.push_back(Point(466, 522));
  object3.push_back(Point(569, 428));

  polygons.push_back(object1);
  polygons.push_back(object2);
  polygons.push_back(object3);

  sdfScale = glm::sqrt(width * width + height * height);

  canvas = Mat(height, width, CV_32FC3, Scalar(1.f, 1.f, 1.f));
  oriCanvas = Mat(height, width, CV_32FC3, Scalar(1.f, 1.f, 1.f));
  namedWindow(wndName);
  setMouseCallback(wndName, mouseCallback);

  // compute sdf
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      Point p(x, y);

      float dist = 9999.f;
      float temp = 0.f;

      for (int i = 0; i < polygons.size(); i++) {
        temp = (inside_polygon(p, polygons[i])) ? -1.f : 1.f;
        temp *= nearest_distance(p, polygons[i]);
        dist = glm::min(temp, dist);
      }

      // float temp = (inside_polygon(p, object1)) ? -1.f : 1.f;
      // temp *= nearest_distance(p, object1);
      // dist = glm::min(temp, dist);
      //
      // temp = (inside_polygon(p, object2)) ? -1.f : 1.f;
      // temp *= nearest_distance(p, object2);
      // dist = glm::min(temp, dist);
      //
      // temp = (inside_polygon(p, object3)) ? -1.f : 1.f;
      // temp *= nearest_distance(p, object3);
      // dist = glm::min(temp, dist);

      // sdf not as image
      sdf.at<float>(Point(x, y)) = dist;

      // sdf as image
      // scale sdf
      dist = ((dist / sdfScale) + 1.f) * 0.5f;

      Vec3f &pixel = canvas.at<Vec3f>(Point(x, y));
      pixel = Vec3f(dist, dist, dist);

      Vec3f &oriPixel = oriCanvas.at<Vec3f>(Point(x, y));
      oriPixel = Vec3f(dist, dist, dist);
    }
  }

  // draw objects
  for (int i = 0; i < polygons.size(); i++) {
    polylines(canvas, polygons[i], true, BLUE);
    polylines(oriCanvas, polygons[i], true, BLUE);
  }

  // polylines(canvas, object1, true, BLUE);
  // polylines(canvas, object2, true, BLUE);
  // polylines(canvas, object3, true, BLUE);
  //
  // polylines(oriCanvas, object1, true, BLUE);
  // polylines(oriCanvas, object2, true, BLUE);
  // polylines(oriCanvas, object3, true, BLUE);

  // printSdf(vec2(250, 400));
  // printSdf(vec2(251, 400));
  // printSdf(vec2(249, 400));
  // printSdf(vec2(250, 401));
  // printSdf(vec2(250, 399));
  // drawSdf(vec2(250, 400));

  // show image
  imshow(wndName, canvas);
  waitKey(0);

  return 0;
}
