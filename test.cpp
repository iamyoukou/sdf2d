#include <iostream>
#include <ctime>
#include <chrono>
#include <thread>

using namespace std;

const float FPS60_MS = 16.7f;

int main(int argc, char const *argv[]) {
  clock_t ticks;
  float ms;
  int wait;
  bool isLoopOn = true;

  int frame = 0;

  while (frame < 60) {
    ticks = clock();

    // do something

    frame++;

    ticks = clock() - ticks;
    ms = (float)ticks / CLOCKS_PER_SEC * 1000.f;
    wait = (int)(FPS60_MS - ms);

    std::cout << "frame " << frame << ", wait " << wait << "ms" << '\n';
  }

  return 0;
}
