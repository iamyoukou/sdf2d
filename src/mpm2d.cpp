#include "simulation.h"

using namespace MPM2D;

Array2D<MPM2D::Node> nodes(gridWidth, gridHeight);

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

void simulation() {

  while (frame < 600) {
    oriCanvas.copyTo(canvas); // clean canvas

    // move polygons
    // fan.rotate(fan.omega);
    // fan.translate(fan.v * dt);
    // hand.translate(hand.v * dt);

    // redraw polygons
    drawPolygons();

    // update sdf
    // computeSdf();

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
        float scale = 0.1f;

        vrot *= co->omega * length(r) * scale;

        vco = vlin + vrot;
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

      // if a particle has moved into an object
      // push it out
      float newDist = getDistance(pos);
      if (newDist < 0.f) {
        newDist *= 2.f; // for visualization convenience
        vec2 newGrad = getGradient(pos);
        pos += newDist * newGrad;
      }

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

// N(x) 关于 x 的导数
float dN(float x) {
  float sign = x < 0.f ? -1.f : 1.f;
  float res;

  x = abs(x);

  if (x < 1.f) {
    res = 1.5f * x * x * sign - 2.f * x * sign;
  } else if (x < 2.f) {
    res = -0.5f * x * x * sign + 2.f * x * sign - 2.f * sign;
  } else {
    res = 0.f;
  }

  return res;
}

// weight function
// pos: particle position
// node: node index
float wip(vec2 pos, ivec2 node) {
  vec2 fNode(float(node.x) * h, float(node.y) * h);
  vec2 temp = (pos - fNode) / h;

  return N(temp.x) * N(temp.y);
}

// gradient of weight function
vec2 gdwip(vec2 pos, ivec2 node) {
  vec2 fNode(float(node.x) * h, float(node.y) * h);
  vec2 temp = (pos - fNode) / h;

  vec2 res;
  res.x = dN(temp.x) * N(temp.y);
  res.y = N(temp.x) * dN(temp.y);
  res /= h;

  return res;
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

void initNodes() {
  for (int r = 0; r < nodes.numOfRows; r++) {
    for (int c = 0; c < nodes.numOfCols; c++) {
      // std::cout << nodes.get(r, c).m << '\n';
    }
  }
}

void rasterize() {
  // particle to grid
  for (int r = 0; r < nodes.numOfRows; r++) {
    for (int c = 0; c < nodes.numOfCols; c++) {
      for (int i = 0; i < particles.size(); i++) {
        MPM2D::Node n = nodes.get(r, c);
      }
    }
  }
}

int main(int argc, char const *argv[]) {
  createParticles();
  // createPolygons();
  //
  // initGrid();
  // initWindow();
  //
  // computeSdf();
  //
  // drawPolygons();
  //
  // simulation();
  //
  // images2video();

  // flip(canvas, canvas, 0);
  // imshow(wndName, canvas);
  // waitKey(0);

  // step 1
  for (int r = 0; r < nodes.numOfRows; r++) {
    for (int c = 0; c < nodes.numOfCols; c++) {
      MPM2D::Node n;
      n.m = 0;
      n.v = vec2(0, 0);

      for (int i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];

        // wip 需要重复使用
        // 可以考虑保存在查找表里
        n.m += p.m * wip(p.pos, ivec2(r, c));
        n.m = (n.m == 0) ? 0.00001f : n.m; // avoid zero-division
      }

      // node velocity
      for (int i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        n.v += p.v * p.m * wip(p.pos, ivec2(r, c));
      }
      n.v /= n.m;

      if (frame == 0) {
        float h3 = h * h * h;
        n.rho = n.m / h3;
      }

      nodes.set(r, c, n);
    }
  } // end step 1

  // step 2
  // node density, first frame only
  if (frame == 0) {
    float h3 = h * h * h;

    for (int i = 0; i < particles.size(); i++) {
      Particle &p = particles[i];

      for (int r = 0; r < nodes.numOfRows; r++) {
        for (int c = 0; c < nodes.numOfCols; c++) {
          MPM2D::Node &n = nodes.get(r, c);
          p.rho += n.m * wip(p.pos, ivec2(r, c));
        }
      }
      p.rho /= h3;

      p.vol = p.m / p.rho;
    }
  } // end step 2

  // for (int r = 0; r < nodes.numOfRows; r++) {
  //   for (int c = 0; c < nodes.numOfCols; c++) {
  //     // std::cout << "m_i = " << nodes.get(r, c).m << '\n';
  //     // std::cout << "v_i = " << glm::to_string(nodes.get(r, c).v) << '\n';
  //     // std::cout << "rho_i = " << nodes.get(r, c).rho << '\n';
  //   }
  // }

  for (int i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    std::cout << "rho_p = " << p.rho << '\n';
    std::cout << "vol_p = " << p.vol << '\n';
  }

  // for (int i = 0; i < particles.size(); i++) {
  //   std::cout << to_string(particles[i].pos) << '\n';
  // }

  return 0;
}
