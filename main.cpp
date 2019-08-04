#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define iBLUE Scalar(255, 0, 0)
#define iGREEN Scalar(0, 255, 0)
#define iRED Scalar(0, 0, 255)
#define iWHITE Scalar(255, 255, 255)
#define iBG_COLOR Scalar(222, 187, 171)

#define WND_WIDTH 600
#define WND_HEIGHT 600

using namespace glm;
using namespace std;
using namespace cv;

template <typename T> struct Array2D {
  std::vector<T> data;
  int numOfRows, numOfCols;

  Array2D() {}

  Array2D(int nrows, int ncols) {
    for (int i = 0; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        T d;
        data.push_back(d);
      }
    }

    numOfRows = nrows;
    numOfCols = ncols;
  }

  void resize(int nrows, int ncols) {
    data.clear();
    for (int i = 0; i < nrows; i++) {
      for (int j = 0; j < ncols; j++) {
        T d;
        data.push_back(d);
      }
    }

    numOfRows = nrows;
    numOfCols = ncols;
  }

  T get(int r, int c) { return data[c + r * numOfCols]; }

  void set(int r, int c, T d) { data[c + r * numOfCols] = d; }
};

struct Particle {
  vec2 pos;
  float m;
  vec2 v;
  Scalar color;
};

// if some collision detections fail,
// change NARROW_BAND to a higher value
const float NARROW_BAND = 8.f;

// world space size
const float width = 1000.f;
const float height = 1000.f;

const float sdfCellSize = 2.f;
int sdfWidth, sdfHeight;

float sdfScale;
Mat canvas, oriCanvas;
Mat sdfImg, letterImg;
vector<Particle> particles;
string wndName = "Test";

// same origin for world space and sdf space
const vec2 worldOrigin(0, 0);
const vec2 sdfOrigin(0, 0);

// using Polygon = std::vector<Point>;

struct Polygon {
  std::vector<vec2> vertices;
  vec2 lb, rt; // aabb, left-bottom, right-top
  vec2 v;      // linear velocity
  float omega; // rotate velocity
  Scalar color;
  string name; // for test purpose

  void computeAabb() {
    lb = vertices[0];
    rt = vertices[1];

    for (int i = 0; i < vertices.size(); i++) {
      vec2 vtx = vertices[i];

      lb.x = (lb.x < vtx.x) ? lb.x : vtx.x;
      lb.y = (lb.y < vtx.y) ? lb.y : vtx.y;

      rt.x = (rt.x > vtx.x) ? rt.x : vtx.x;
      rt.y = (rt.y > vtx.y) ? rt.y : vtx.y;
    }
  }

  void add(vec2 vtx) { vertices.push_back(vtx); }

  void translate(vec2 xy) {
    for (int i = 0; i < vertices.size(); i++) {
      vertices[i] += xy;
    }

    // update aabb
    lb += xy;
    rt += xy;
  }

  vec2 applyRotate(vec2 vtx, float theta, vec2 offset) {
    float x, y;

    // translate to origin
    x = vtx.x - offset.x;
    y = vtx.y - offset.y;

    // rotate
    vtx.x = x * cos(theta) - y * sin(theta);
    vtx.y = x * sin(theta) + y * cos(theta);

    // translate back
    vtx.x += offset.x;
    vtx.y += offset.y;

    return vtx;
  }

  void rotate(float theta) {
    // degree to radian
    theta = 3.1415926f / 180.f * theta;

    vec2 center = (lb + rt) * 0.5f;
    vec2 offset = center - worldOrigin; // offset from origin

    float x, y;

    for (int i = 0; i < vertices.size(); i++) {
      vertices[i] = applyRotate(vertices[i], theta, offset);
    }

    // update aabb
    computeAabb();
  }

  vec2 applyScale(vec2 vtx, float factor, vec2 offset) {
    vtx -= offset; // translate to origin
    vtx *= factor;
    vtx += offset; // translate back

    return vtx;
  }

  void scale(float factor) {
    vec2 center = (lb + rt) * 0.5f;
    vec2 offset = center - worldOrigin; // offset from origin

    for (int i = 0; i < vertices.size(); i++) {
      vertices[i] = applyScale(vertices[i], factor, offset);
    }

    // update aabb
    lb = applyScale(lb, factor, offset);
    rt = applyScale(rt, factor, offset);
  }
};

Polygon object1, object2, object3;
Polygon fan, hand;

// create polygons
using Polygon_ptr = Polygon *;
std::vector<Polygon_ptr> polygons;

struct Grid {
  Mat sdf;
  Array2D<Polygon *> polyPtrs;
};

Grid grid;

bool intersect(vec2 p1, vec2 p2, vec2 p3, vec2 p4) {
  vec3 a(p1, 0), b(p2, 0), c(p3, 0), d(p4, 0);

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

bool inside_polygon(vec2 p, Polygon &poly) {
  int count = 0;
  vec2 q(1234567.f, 1234567.f); // a point at the infinity
  int len = poly.vertices.size();

  for (int i = 0; i < len; i++) {
    vec2 &start = poly.vertices[i];
    vec2 &end = poly.vertices[(i + 1) % len];
    count += intersect(p, q, start, end);
  }

  return count % 2 == 1;
}

float nearest_distance(vec2 p, Polygon &poly) {
  float dist = 9999.f;
  int len = poly.vertices.size();

  for (int i = 0; i < len; i++) {
    vec2 &a = poly.vertices[i];
    vec2 &b = poly.vertices[(i + 1) % len];

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

// p is world position
float getDistance(vec2 p) {
  // world position to sdf index
  int idx_x, idx_y;
  idx_x = int(floor(p.x / sdfCellSize));
  idx_y = int(floor(p.y / sdfCellSize));

  if (idx_x < 0 || idx_x > sdfWidth - 1) {
    return 9999.f;
  } else if (idx_y < 0.f || idx_y > sdfHeight - 1) {
    return 9999.f;
  } else {
    return grid.sdf.at<float>(Point(idx_x, idx_y));
  }
}

// p is world position
vec2 getGradient(vec2 p) {
  vec2 gd;

  // world position to sdf position
  vec2 sdfPos, sdfPosFloor;
  sdfPos = p / sdfCellSize;
  sdfPosFloor = floor(sdfPos);

  float fx, fy;
  fx = sdfPos.x - sdfPosFloor.x;
  fy = sdfPos.y - sdfPosFloor.y;

  float temp1 =
      getDistance(vec2(p.x + 1.f, p.y)) - getDistance(vec2(p.x - 1.f, p.y));
  float temp2 = getDistance(vec2(p.x + 1.f, p.y + 1.f)) -
                getDistance(vec2(p.x - 1.f, p.y + 1.f));
  gd.x = lerp(temp1, temp2, fy);

  float temp3 =
      getDistance(vec2(p.x, p.y + 1.f)) - getDistance(vec2(p.x, p.y - 1.f));
  float temp4 = getDistance(vec2(p.x + 1.f, p.y + 1.f)) -
                getDistance(vec2(p.x + 1.f, p.y - 1.f));
  gd.y = lerp(temp3, temp4, fx);

  gd = normalize(-gd);

  return gd;
}

// p is world position
Polygon *getPolygon(vec2 p) {
  // world position to sdf index
  int idx_x, idx_y;
  idx_x = int(floor(p.x / sdfCellSize));
  idx_y = int(floor(p.y / sdfCellSize));

  if (idx_x < 0 || idx_x > sdfWidth - 1) {
    return NULL;
  } else if (idx_y < 0.f || idx_y > sdfHeight - 1) {
    return NULL;
  } else {
    return grid.polyPtrs.get(idx_y, idx_x);
  }
}

void mouseCallback(int event, int x, int y, int flag, void *param) {
  oriCanvas.copyTo(canvas);

  // window position to world position
  vec2 p;
  p.x = float(x) / float(WND_WIDTH) * width;
  p.y = float(y) / float(WND_HEIGHT) * height;

  switch (event) {
  case EVENT_MOUSEMOVE:
    // sdf gradient at (x, y)
    vec2 grad = getGradient(p);

    // distance
    float dist = getDistance(p);

    std::cout << "(" << x << ", " << y << ") "
              << "distance = " << dist
              << ", Object name: " << getPolygon(p)->name << '\n';

    // distance and direction to object1 boundary
    float sx, sy, ex, ey;
    sx = p.x;
    sy = p.y;
    ex = sx + grad.x * dist;
    ey = sy + grad.y * dist;

    // draw arrow
    // world to window
    int isx, isy, iex, iey;
    isx = int(sx / width * float(WND_WIDTH));
    isy = int(sy / height * float(WND_HEIGHT));
    iex = int(ex / width * float(WND_WIDTH));
    iey = int(ey / height * float(WND_HEIGHT));
    arrowedLine(canvas, Point(isx, isy), Point(iex, iey), iRED);
    // circle(canvas, Point(isx, isy), 3, iRED, -1);
    imshow(wndName, canvas);
    waitKey(1);

    // std::cout << "(" << sx << ", " << sy << ") to (" << ex << ", " << ey
    // <<
    // ")"
    //           << '\n';

    break;
  }
}

int myRand(int down, int up) { return (rand() % (up - down - 1) + down); }

void createParticles() {
  int rndSize = 200;
  int imgWidth = letterImg.size().width;
  int imgHeight = letterImg.size().height;

  for (int i = 0; i < rndSize; i++) {
    for (int j = 0; j < rndSize; j++) {
      int x, y;
      x = myRand(0, imgWidth - 1);
      y = myRand(0, imgHeight - 1);

      Vec3b pixelSrc = letterImg.at<Vec3b>(Point(x, y));

      int color = pixelSrc[0] + pixelSrc[1] + pixelSrc[2];
      // some threshold
      if (color < 10) {
        Particle p;

        // rescale
        float fx, fy;
        fx = (float)x / (float)imgWidth; // to [0, 1.0]
        fy = (float)y / (float)imgHeight;
        fx *= 0.75f;
        fy *= 0.75f;

        // translate
        fx += 0.125f;
        fy += 0.25f;

        // back to world space
        p.pos = vec2(fx * (float)width, fy * (float)height);
        p.m = (float)myRand(1, 10);
        p.v = vec2(0.f, 0.f);

        p.color = iWHITE;

        particles.push_back(p);
      }
    }
  }
}

void computeSdf() {
  // compute sdf for (sdfWidth * sdfHeight) world space points
  // the interval between two adjacent points is sdfCellSize
  for (int x = 0; x < sdfWidth; x++) {
    for (int y = 0; y < sdfHeight; y++) {
      // world space point
      vec2 p = worldOrigin + vec2(sdfCellSize * x, sdfCellSize * y);

      float fDist = 9999.f;
      float temp = 0.f;

      for (int i = 0; i < polygons.size(); i++) {
        temp = (inside_polygon(p, (*polygons[i]))) ? -1.f : 1.f;
        temp *= nearest_distance(p, (*polygons[i]));

        // fDist = glm::min(temp, fDist);
        if (temp < fDist) {
          fDist = temp;
          grid.polyPtrs.set(y, x, polygons[i]);
        }
      }

      // save sdf to a lookup table
      grid.sdf.at<float>(Point(x, y)) = fDist;
      // std::cout << "(" << x << ", " << y << ") distance = " << fDist <<
      // '\n';

      // save sdf to image
      fDist = ((fDist / sdfScale) + 1.f) * 0.5f; // to [0.0, 1.0]
      int iDist = (int)(fDist * 255.f);          // to [0, 255]

      Vec3b &pixel = sdfImg.at<Vec3b>(Point(x, y));
      pixel = Vec3b(iDist, iDist, iDist);
    }
  }
}

void createPolygons() {
  // fan
  fan.name = "Fan";
  fan.add(vec2(118.f, 47.f));
  fan.add(vec2(138.f, 243.f));
  fan.add(vec2(249.f, 353.f));
  fan.add(vec2(119.f, 487.f));
  fan.add(vec2(315.f, 466.f));
  fan.add(vec2(426.f, 357.f));
  fan.add(vec2(557.f, 486.f));
  fan.add(vec2(538.f, 291.f));
  fan.add(vec2(427.f, 179.f));
  fan.add(vec2(559.f, 47.f));
  fan.add(vec2(361.f, 68.f));
  fan.add(vec2(250.f, 177.f));
  fan.computeAabb();
  // transform
  vec2 offset;
  offset.x = width * 0.5f - (fan.lb.x + fan.rt.x) * 0.5f;
  offset.y = height * 0.25f - (fan.lb.y + fan.rt.y) * 0.5f;
  fan.translate(offset);
  fan.scale(0.5f);
  fan.rotate(45.f);
  // other properties
  fan.color = Scalar(164, 70, 152);
  fan.v = vec2(0.f, 10.f);
  fan.omega = 1.f;

  // object1.add(vec2(488.3f, 40.f));
  // object1.add(vec2(346.7f, 163.3f));
  // object1.add(vec2(501.7f, 280.f));
  // object1.add(vec2(668.3f, 186.7f));
  // object1.computeAabb();
  //
  // object2.add(vec2(98.3f, 291.7f));
  // object2.add(vec2(98.3f, 455.f));
  // object2.add(vec2(281.7f, 583.3f));
  // object2.add(vec2(441.7f, 490.f));
  // object2.computeAabb();
  //
  // object3.add(vec2(691.7f, 393.3f));
  // object3.add(vec2(776.7f, 620.f));
  // object3.add(vec2(948.3f, 463.3f));
  // object3.computeAabb();
  //
  // hand.name = "Hand";
  // hand.add(vec2(580.f, 60.f));
  // hand.add(vec2(490.f, 110.f));
  // hand.add(vec2(390.f, 210.f));
  // hand.add(vec2(380.f, 500.f));
  // hand.add(vec2(360.f, 660.f));
  // hand.add(vec2(320.f, 750.f));
  // hand.add(vec2(450.f, 880.f));
  // hand.add(vec2(610.f, 680.f));
  // hand.add(vec2(610.f, 510.f));
  // hand.add(vec2(630.f, 390.f));
  // hand.add(vec2(600.f, 360.f));
  // hand.add(vec2(530.f, 410.f));
  // hand.add(vec2(510.f, 320.f));
  // hand.add(vec2(520.f, 270.f));
  // hand.add(vec2(580.f, 190.f));
  // hand.add(vec2(600.f, 90.f));
  // hand.computeAabb();
  // hand.rotate(90.f);
  // hand.scale(0.25f);
  // hand.translate(vec2(-250.f, 250.f));
  // hand.color = Scalar(81, 205, 254);
  // hand.v = vec2(20.f, 0.f);

  // polygons.push_back(&object1);
  // polygons.push_back(&object2);
  // polygons.push_back(&object3);
  // polygons.push_back(&hand);
  polygons.push_back(&fan);
}

void drawSdf() {
  // due to the difference between (sdfWidth, sdfHeight) and (WND_WIDTH,
  // WND_HEIGHT) the drawn sdf may not be continuous
  for (int x = 0; x < WND_WIDTH; x++) {
    for (int y = 0; y < WND_HEIGHT; y++) {
      // window position to world position
      vec2 p;
      p.x = float(x) / float(WND_WIDTH) * width;
      p.y = float(y) / float(WND_HEIGHT) * height;

      float fDist = getDistance(p);
      // sdf as image
      // scale sdf
      fDist = ((fDist / sdfScale) + 1.f) * 0.5f; // to [0.0, 1.0]
      int iDist = (int)(fDist * 255.f);          // to [0, 255]

      // to window space
      int ix = int(p.x / width * WND_WIDTH);
      int iy = int(p.y / height * WND_HEIGHT);

      Vec3b &pixel = canvas.at<Vec3b>(Point(ix, iy));
      pixel = Vec3b(iDist, iDist, iDist);

      Vec3b &oriPixel = oriCanvas.at<Vec3b>(Point(ix, iy));
      oriPixel = Vec3b(iDist, iDist, iDist);
    }
  }

  // save sdf image
  // imwrite("sdfImage.png", sdfImg);
}

void drawPolygons() {
  for (int i = 0; i < polygons.size(); i++) {
    std::vector<Point> pts;

    for (int j = 0; j < (*polygons[i]).vertices.size(); j++) {
      vec2 vtx = (*polygons[i]).vertices[j];

      Point p;
      // world to ndc, ndc to window
      p.x = int(vtx.x / width * WND_WIDTH);
      p.y = int(vtx.y / height * WND_HEIGHT);

      pts.push_back(p);
    }

    // non-filled polygon
    // polylines(canvas, pts, true, iBLUE);
    // polylines(oriCanvas, pts, true, iBLUE);

    // filled polygon
    vector<vector<Point>> pp;
    pp.push_back(pts);
    fillPoly(canvas, pp, (*polygons[i]).color);
    // fillPoly(oriCanvas, pp, (*polygons[i]).color);

    // draw aabb
    // Point p1, p2;
    // p1.x = (int)(polygons[i].lb.x / width * WND_WIDTH);
    // p1.y = (int)(polygons[i].lb.y / height * WND_HEIGHT);
    // p2.x = (int)(polygons[i].rt.x / width * WND_WIDTH);
    // p2.y = (int)(polygons[i].rt.y / height * WND_HEIGHT);
    // rectangle(canvas, p1, p2, iGREEN);
  }
}

void drawParticles() {
  for (int i = 0; i < particles.size(); i++) {
    vec2 pos = particles[i].pos;

    int ix, iy;
    ix = int(pos.x / width * float(WND_WIDTH));
    iy = int(pos.y / height * float(WND_HEIGHT));

    circle(canvas, Point(ix, iy), 2, particles[i].color, -1);
  }
}

void images2video() {
  string command =
      "ffmpeg -r 60 -start_number 0 -i ./result/sim%03d.png -vcodec mpeg4 "
      "-b 5000k -s 600x600 ./result.mp4";
  system(command.c_str());

  // remove images
  // command = "rm ./result/*.png";
  // system(command.c_str());
}

void simulation() {
  // for simulation
  int frame = 0;
  float dt = 0.1f;  // s
  vec2 g(0.f, 0.f); //(m/s^2)

  while (frame < 600) {
    oriCanvas.copyTo(canvas); // clean canvas

    // move polygons
    fan.rotate(fan.omega);
    fan.translate(fan.v * dt);
    // hand.translate(hand.v * dt);

    // redraw polygons
    drawPolygons();

    // update sdf
    computeSdf();

    // for each particle
    for (int i = 0; i < particles.size(); i++) {
      vec2 pos = particles[i].pos;
      vec2 v = particles[i].v;
      float m = particles[i].m;

      // draw objects
      int ix, iy;
      ix = int(pos.x / width * float(WND_WIDTH));
      iy = int(pos.y / height * float(WND_HEIGHT));

      circle(canvas, Point(ix, iy), 2, particles[i].color, -1);

      // simulation
      v += g * dt;

      // distance
      float dist = getDistance(pos);
      vec2 n;
      Polygon *co; // collision object
      bool isCollisionOn = false;

      vec2 vco(0.f, 0.f);

      // 基于 sdf 的碰撞检测
      // 和边界处的碰撞检测
      // 唯一不同的是法向量 n 的计算方式
      // 而碰撞后的速度，采用同样��处理
      // for sdf collision detection
      // narrow band threshold
      if (abs(dist) < NARROW_BAND) {
        n = -getGradient(pos);
        co = getPolygon(pos); // get collision object
        isCollisionOn = true;

        // object velocity
        // linear
        vec2 vlin = co->v;

        // rotational
        vec2 center = (co->lb + co->rt) * 0.5f;
        vec2 r = pos - center;

        vec2 vrot;
        vrot.x = -r.y / length(r);
        vrot.y = r.x / length(r);

        // for visualization convenience
        float scale = 0.25f;

        vrot *= co->omega * length(r) * scale;

        vco = vlin + vrot;

        std::cout << vrot.x << ", " << vrot.y << '\n';
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

      // relative velocity
      vec2 vrel = v - vco;

      float vnLength = dot(n, vrel);
      // std::cout << vnLength << ", " << isCollisionOn << '\n';

      // if not separating
      if (vnLength < 0.f && isCollisionOn) {
        vec2 vt = vrel - vnLength * n;
        float mu = 0.55f;

        if (length(vt) <= -mu * vnLength) {
          vrel = vec2(0.f, 0.f);
        } else {
          vrel = vt + mu * vnLength * normalize(vt);
        }

        // back to particle velocity
        v = vrel + vco;

        // std::cout << v.x << ", " << v.y << '\n';
      }

      // update position
      pos += v * dt;

      particles[i].v = v;
      particles[i].pos = pos;
    } // end for each particle

    // output to image series
    Mat temp;
    canvas.copyTo(temp);
    flip(temp, temp, 0);
    imwrite(format("./result/sim%03d.png", frame), temp);

    frame++;
  }
}

int main(int argc, char const *argv[]) {
  // create particles from images
  // letterImg = imread("letter2.png");
  letterImg = imread("letter.png");
  createParticles();

  // save sdf as cv::Mat
  sdfWidth = int(width / sdfCellSize);
  sdfHeight = int(height / sdfCellSize);

  grid.sdf = Mat::zeros(sdfHeight, sdfWidth, CV_32F);
  grid.polyPtrs.resize(sdfHeight, sdfWidth);
  sdfImg = Mat(sdfHeight, sdfWidth, CV_8UC3, iWHITE);
  sdfScale = glm::sqrt(width * width + height * height);

  createPolygons();

  // create canvas
  canvas = Mat(WND_HEIGHT, WND_WIDTH, CV_8UC3, iBG_COLOR);
  oriCanvas = Mat(WND_HEIGHT, WND_WIDTH, CV_8UC3, iBG_COLOR);
  // output = Mat(height, width, CV_8UC3, Scalar(255, 255, 255));
  namedWindow(wndName);
  // setMouseCallback(wndName, mouseCallback);

  computeSdf();
  // drawSdf();

  drawPolygons();
  // drawParticles();

  simulation();

  // convert images to video
  images2video();

  // flip(canvas, canvas, 0);
  // imshow(wndName, canvas);
  // waitKey(0);

  return 0;
}
