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

  ivec2 ip((int)p.x, (int)p.y);

  float fx, fy;
  fx = p.x - (float)ip.x;
  fy = p.y - (float)ip.y;

  float temp1 =
      getDistance(vec2(p.x + 1.f, p.y)) - getDistance(vec2(p.x - 1.f, p.y));
  float temp2 = getDistance(vec2(p.x + 1.f, p.y + 1.f)) -
                getDistance(vec2(p.x - 1.f, p.y + 1.f));
  gd.x = (1.f - fy) * temp1 + fy * temp2;

  temp1 = getDistance(vec2(p.x, p.y + 1.f)) - getDistance(vec2(p.x, p.y - 1.f));
  temp2 = getDistance(vec2(p.x + 1.f, p.y + 1.f)) -
          getDistance(vec2(p.x + 1.f, p.y - 1.f));
  gd.y = (1.f - fx) * temp1 + fx * temp2;

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

int myRand(int down, int up) { return (rand() % (up - down - 1) + down); }

void createParticles() {
  int rndSize = 200;
  for (int i = 0; i < rndSize; i++) {
    for (int j = 0; j < rndSize; j++) {
      int x, y;
      x = myRand(0, width - 1);
      y = myRand(0, height - 1);

      Vec3b pixelSrc = letterImg.at<Vec3b>(Point(x, y));

      int color = pixelSrc[0] + pixelSrc[1] + pixelSrc[2];
      // some threshold
      if (color < 10) {
        Particle p;

        // rescale
        float fx, fy;
        fx = (float)x / (float)width; // to [0, 1.0]
        fy = (float)y / (float)height;
        fx *= 0.75f;
        fy *= 0.75f;

        // translate
        fx += 0.125f;
        fy += 0.45f;

        p.pos = vec2(fx * (float)width, fy * (float)height);
        p.m = (float)myRand(1, 10);
        p.v = vec2(0.f, 0.f);

        particles.push_back(p);
      }
    }
  }
}

int main(int argc, char const *argv[]) {
  letterImg = imread("letter2.png");
  createParticles();

  sdf = Mat::zeros(height, width, CV_32F);
  std::vector<Polygon> polygons;

  Polygon object1;
  object1.push_back(Point(293, 24));
  object1.push_back(Point(208, 98));
  object1.push_back(Point(301, 168));
  object1.push_back(Point(401, 112));

  Polygon object2;
  object2.push_back(Point(59, 175));
  object2.push_back(Point(59, 273));
  object2.push_back(Point(169, 350));
  object2.push_back(Point(265, 294));

  Polygon object3;
  object3.push_back(Point(415, 236));
  object3.push_back(Point(466, 372));
  object3.push_back(Point(569, 278));

  polygons.push_back(object1);
  polygons.push_back(object2);
  polygons.push_back(object3);

  sdfScale = glm::sqrt(width * width + height * height);

  canvas = Mat(height, width, CV_32FC3, Scalar(1.f, 1.f, 1.f));
  oriCanvas = Mat(height, width, CV_32FC3, Scalar(1.f, 1.f, 1.f));
  output = Mat(height, width, CV_8UC3, Scalar(255, 255, 255));
  namedWindow(wndName);
  // setMouseCallback(wndName, mouseCallback);

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

  int frame = 0;

  float dt = 0.1f;    // s
  vec2 g(0.f, -9.8f); //(m/s^2)

  while (frame < 600) {
    oriCanvas.copyTo(canvas); // clean canvas

    // for each particle
    for (int i = 0; i < particles.size(); i++) {
      vec2 pos = particles[i].pos;
      vec2 v = particles[i].v;
      float m = particles[i].m;

      // draw objects
      circle(canvas, Point(pos.x, pos.y), 2, RED, -1);

      // simulation
      v += g * dt;
      // pos += v * dt;

      // distance
      float dist = getDistance(pos);
      vec2 n;
      bool isCollisionOn = false;

      // 基于 sdf �����撞��测
      // 和边界处的碰撞���测
      // 唯一不同的是法向量 n 的计算方式
      // 而碰撞后的速度，采用同样的处理
      // for sdf collision detection
      // narrow band threshold
      if (abs(dist) < NARROW_BAND) {
        n = -getGradient(pos);
        isCollisionOn = true;
      }

      // boundary situation
      if (pos.x < 0.f) {
        pos.x = 0.f;
        n = vec2(1.f, 0.f);
        isCollisionOn = true;
      } else if (pos.x > (float)width - 1.f) {
        pos.x = (float)width - 1.f;
        n = vec2(-1.f, 0.f);
        isCollisionOn = true;
      } else if (pos.y < 0.f) {
        pos.y = 0.f;
        n = vec2(0, 1.f);
        isCollisionOn = true;
      } else if (pos.y > (float)height - 1.f) {
        pos.y = (float)height - 1.f;
        n = vec2(0, -1.f);
        isCollisionOn = true;
      }

      float vnLength = dot(n, v);

      // if not separating
      if (vnLength < 0.f && isCollisionOn) {
        vec2 vt = v - vnLength * n;
        float mu = 0.55f;

        if (length(vt) <= -mu * vnLength) {
          v = vec2(0.f, 0.f);
        } else {
          v = vt + mu * vnLength * normalize(vt);
        }
      }

      // update position
      pos += v * dt;

      particles[i].v = v;
      particles[i].pos = pos;
    } // end for each particle

    // output to image series
    Mat temp;
    canvas.convertTo(temp, CV_8UC3, 255.f);
    flip(temp, temp, 0);
    imwrite(format("./result/sim%03d.png", frame), temp);

    frame++;
  }

  // convert images to video
  string command =
      "ffmpeg -r 60 -start_number 0 -i ./result/sim%03d.png -vcodec mpeg4 "
      "-b 5000k -s 600x600 ./result.mp4";
  system(command.c_str());

  return 0;
}
