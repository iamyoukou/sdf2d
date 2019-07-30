#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define BLUE Scalar(1.f, 0.f, 0.f)
#define RED Scalar(0.f, 0.f, 1.f)
#define WHITE Scalar(1.f, 1.f, 1.f)

using namespace glm;
using namespace std;
using namespace cv;

struct Particle {
  vec2 pos;
  float m;
  vec2 v;
};

const int width = 600, height = 600;
const float NARROW_BAND = 5.f;
const int BD_OFFSET = 1;
float sdfScale;
float sdfCellSize = 1.f;
Mat canvas, oriCanvas, output;
Mat sdf, letterImg;
vector<Particle> particles;

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

float getDistance(vec2 p) {
  if (p.x < 0.f || p.x > (float)width) {
    return 9999.f;
  } else if (p.y < 0.f || p.y > (float)height) {
    return 9999.f;
  } else {
    return sdf.at<float>(Point(p.x, p.y));
  }
}

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

void drawArrow(vec2 p) {
  vec2 grad = getGradient(p);
  float dist = getDistance(p);

  float sx, sy, ex, ey;
  sx = p.x;
  sy = p.y;
  ex = sx + grad.x * dist;
  ey = sy + grad.y * dist;

  // draw arrow
  arrowedLine(canvas, Point(sx, sy), Point(ex, ey), RED);
}

int main(int argc, char const *argv[]) {
  float alpha = 0.1f;
  float a = 1.f, b = 2.f;
  std::cout << lerp(alpha, a, b) << '\n';

  return 0;
}
