#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace glm;
using namespace std;
using namespace cv;
using namespace Eigen;

const Scalar iWHITE(255, 255, 255);
const Scalar iRED(0, 0, 255);
const Scalar iGREEN(0, 255, 0);
const Scalar iBLUE(255, 0, 0);

Mat canvas;
const int WND_WIDTH = 600;
const int WND_HEIGHT = 600;

const float width = 8.f;
const float height = 3.f;

const float h = 1.f;

string wndName = "test";

float N(float x) {
  x = abs(x);

  if (x >= 0.f && x < 1.f) {
    return 0.5f * x * x * x - x * x + 0.6667f;
  } else if (x >= 1.f && x < 2.f) {
    return -0.1667f * x * x * x + x * x - 2.f * x + 1.3333f;
  } else {
    return 0.f;
  }
}

// weight function
float wip(vec2 pos, ivec2 node) {
  vec2 fNode(float(node.x) * h, float(node.y) * h);
  vec2 temp = (pos - fNode) / h;

  return N(temp.x) * N(temp.y);
}

void plot(vector<vec2> &pts, Scalar color) {
  // draw axis
  Point xStart, xEnd, yStart, yEnd;
  xStart = Point(0, int(WND_HEIGHT * 0.5f));
  xEnd = xStart + Point(WND_WIDTH, 0);
  yStart = Point(int(WND_WIDTH * 0.5f), 0);
  yEnd = yStart + Point(0, WND_HEIGHT);

  arrowedLine(canvas, xStart, xEnd, iBLUE, 1, 8, 0, 0.025);
  arrowedLine(canvas, yStart, yEnd, iBLUE, 1, 8, 0, 0.025);

  // draw points
  Point offset(int(WND_WIDTH * 0.5f), int(WND_HEIGHT * 0.5f));

  for (int i = 0; i < pts.size(); i++) {
    vec2 pt = pts[i];

    Point P;
    P.x = int(pt.x / width * (float)WND_WIDTH);
    P.y = int(pt.y / height * (float)WND_HEIGHT);
    P += offset;

    // std::cout << P.x << ", " << P.y << '\n';

    circle(canvas, P, 2, color, -1);
  }

  Mat temp;
  flip(canvas, temp, 0);
  imshow(wndName, temp);
  waitKey(1);
}

// svd(A) --> VLR
// V, R: 对称行列
// L: 对角行列
void svd(mat2 A, mat2 &V, mat2 &L, mat2 &R) {
  // glm::mat to Eigen::Matrix
  // note: difference of row-major and column-major
  // Eigen::Matrix(row, col) = glm::mat[col][row]
  Matrix<float, 2, 2> m;
  m(0, 0) = A[0][0];
  m(0, 1) = A[1][0];
  m(1, 0) = A[0][1];
  m(1, 1) = A[1][1];

  JacobiSVD<Matrix<float, 2, 2>> jacobiSvd(m, Eigen::ComputeFullU |
                                                  Eigen::ComputeFullV);
  Matrix<float, 2, 2> v, r;
  v = jacobiSvd.matrixU();
  r = jacobiSvd.matrixV();

  Matrix<float, 2, 1> l;
  l = jacobiSvd.singularValues();

  // output to V
  V[0][0] = v(0, 0);
  V[1][0] = v(0, 1);
  V[0][1] = v(1, 0);
  V[1][1] = v(1, 1);

  // output to L
  L[0][0] = l(0);
  L[1][1] = l(1);

  // output to R
  R[0][0] = r(0, 0);
  R[1][0] = r(0, 1);
  R[0][1] = r(1, 0);
  R[1][1] = r(1, 1);
}

int main(int argc, char const *argv[]) {
  // Matrix<float, 2, 2> m;
  // m << 0, 2.0, 1.0, 0;
  mat2 M;
  M[0][0] = 0.f;
  M[1][0] = 2.f;
  M[0][1] = 1.f;
  M[1][1] = 0.f;

  mat2 V, L, R;

  svd(M, V, L, R);

  std::cout << glm::to_string(V) << '\n';
  std::cout << glm::to_string(L) << '\n';
  std::cout << glm::to_string(R) << '\n';

  return 0;
}
