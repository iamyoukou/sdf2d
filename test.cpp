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

#define iBG_COLOR Scalar(222, 187, 171)

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

struct Test {
  int a;
};

int main(int argc, char const *argv[]) {
  std::vector<Test *> v;

  Test t;
  t.a = 1;

  v.push_back(&t);

  t.a++;
  // v[0].a++;

  std::cout << (*v[0]).a << '\n';

  return 0;
}
