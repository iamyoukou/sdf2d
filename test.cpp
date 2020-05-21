#include <iostream>
#include <vector>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace glm;
using namespace std;

int RAND(int down, int up) { return (rand() % (up - down - 1) + down); }

int main(int argc, char const *argv[]) {
  vec2 a(0.76, 0.65);
  vec2 b(12.1, -17.5);

  std::cout << dot(a, normalize(b)) << '\n';

  return 0;
}
