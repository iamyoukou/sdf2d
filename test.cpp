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

const int width = 600, height = 600;
const Scalar iRED(0, 0, 255);
const Scalar iGREEN(0, 255, 0);
const Scalar iBLUE(255, 0, 0);
const Scalar iWHITE(255, 255, 255);

struct Polygon {
  std::vector<vec2> vertices;
  vec2 lb, rt; // aabb, left-bottom, right-top

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
};

void translate2d(std::vector<vec2> &vtxs, float x, float y) {
  for (int i = 0; i < vtxs.size(); i++) {
    vtxs[i] += vec2(x, y);
  }
}

void rotate2d(std::vector<vec2> &vtxs, float theta, vec2 min, vec2 max) {
  // degree to radian
  theta = 3.1415926f / 180.f * theta;

  vec2 center = (min + max) * 0.5f;
  vec2 offset = center - vec2(0.f, 0.f); // offset from origin

  for (int i = 0; i < vtxs.size(); i++) {
    // translate to origin
    float x = vtxs[i].x - offset.x;
    float y = vtxs[i].y - offset.y;

    // rotate
    vtxs[i].x = x * cos(theta) - y * sin(theta);
    vtxs[i].y = x * sin(theta) + y * cos(theta);

    // translate back
    vtxs[i].x += offset.x;
    vtxs[i].y += offset.y;
  }
}

void scale2d(std::vector<vec2> &vtxs, float factor, vec2 min, vec2 max) {
  vec2 center = (min + max) * 0.5f;
  vec2 offset = center - vec2(0.f, 0.f); // offset from origin

  for (int i = 0; i < vtxs.size(); i++) {
    vtxs[i] -= offset; // translate to origin
    vtxs[i] *= factor;
    vtxs[i] += offset; // translate back
  }
}

int main(int argc, char const *argv[]) {
  Polygon poly1;
  poly1.vertices.push_back(vec2(0.58, 0.06));
  poly1.vertices.push_back(vec2(0.49, 0.11));
  poly1.vertices.push_back(vec2(0.39, 0.21));
  poly1.vertices.push_back(vec2(0.38, 0.5));
  poly1.vertices.push_back(vec2(0.36, 0.66));
  poly1.vertices.push_back(vec2(0.32, 0.75));
  poly1.vertices.push_back(vec2(0.45, 0.88));
  poly1.vertices.push_back(vec2(0.61, 0.68));
  poly1.vertices.push_back(vec2(0.61, 0.51));
  poly1.vertices.push_back(vec2(0.63, 0.39));
  poly1.vertices.push_back(vec2(0.6, 0.36));
  poly1.vertices.push_back(vec2(0.53, 0.41));
  poly1.vertices.push_back(vec2(0.51, 0.32));
  poly1.vertices.push_back(vec2(0.52, 0.27));
  poly1.vertices.push_back(vec2(0.58, 0.19));
  poly1.vertices.push_back(vec2(0.6, 0.09));

  poly1.computeAabb();
  rotate2d(poly1.vertices, 90.f, poly1.lb, poly1.rt);
  scale2d(poly1.vertices, 0.5f, poly1.lb, poly1.rt);
  poly1.computeAabb(); // aftter transformation, upate aabb

  Mat canvas(width, height, CV_8UC3, iWHITE);
  string wndName = "test";

  // draw objects
  std::vector<Point> pts;
  for (int i = 0; i < poly1.vertices.size(); i++) {
    vec2 vtx = poly1.vertices[i];
    float x = vtx.x * (float)width;
    float y = vtx.y * (float)height;
    Point p((int)x, (int)y);
    pts.push_back(p);
  }
  polylines(canvas, pts, true, iBLUE);

  Point p1, p2;
  p1.x = (int)(poly1.lb.x * (float)width);
  p1.y = (int)(poly1.lb.y * (float)height);
  p2.x = (int)(poly1.rt.x * (float)width);
  p2.y = (int)(poly1.rt.y * (float)height);
  rectangle(canvas, p1, p2, iGREEN);

  flip(canvas, canvas, 0);
  imshow(wndName, canvas);
  waitKey(0);

  return 0;
}
