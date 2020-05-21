#include "simulation.h"

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
  // for simulation
  int frame = 0;
  int totalFrame = 100;
  float dt = 0.1f;    // s
  vec2 g(0.f, -9.8f); //(m/s^2)

  while (frame < totalFrame) {
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
        // ref:
        // http://w3e.kanazawa-it.ac.jp/math/physics/category/mechanics/masspoint_mechanics/uniform_circular_motion/henkan-tex.cgi?target=/math/physics/category/mechanics/masspoint_mechanics/uniform_circular_motion/ucm_pos_vel_acc.html
        vec2 center = (co->lb + co->rt) * 0.5f;
        vec2 r = pos - center;

        vec2 vrot;
        vrot.x = -r.y / length(r); // sin
        vrot.y = r.x / length(r);  // cos

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

int main(int argc, char const *argv[]) {
  // create particles from images
  // letterImg = imread("letter2.png");
  letterImg = imread("./res/heart.png");
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
