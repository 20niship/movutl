#include <doctest/doctest.h>
#include <iostream>
#include <movutl/core/vector.hpp>
#include <vector>

using namespace mu;


void test1() {
  Vec<int> vec;
  CHECK(vec.size() == 0);

  vec.push_back(20);
  vec.push_back(10);
  CHECK(vec[0] == 20);
  CHECK(vec[1] == 10);
}


class A {
public:
  int x;
  A() {
    std::cout << "constructor called!" << std::endl;
    x = 10;
  }
  ~A() { std::cout << "destructor called!" << std::endl; }
};

void test2() {
  /* mu::Vec<A> as; */
  std::vector<A> as;
  {
    std::cout << "pushback" << std::endl;
    as.resize(1);
    as[0] = std::move(A());
    std::cout << "pushback" << std::endl;
    std::cout << as[0].x << std::endl << std::endl;
  }
  std::cout << "pushback done" << std::endl;
  std::cout << as[0].x << std::endl << std::endl;
}

int test4() {
  Vec<int> x = {0, 1, 2, 3, 4, 5};
  Vec<int> y = {6, 7, 8, 9, 10};
  Vec<int> z = {11, 12, 13};
  x.insert(x.end(), y.begin(), y.end());
  x.insert(x.end(), z.begin(), z.end());
  return 0;
}

int test5() {
  Vec<int> x{1, 2, 3, 4, 5, 6};
  DISP(x.size() == 6);
  Vec<int> y;
  y = {0, 1, 2, 3, 4, 5, 6};
  DISP(x.size() == 7);
  return 0;
}

TEST_CASE("vector-test") {
  test1();
  test2();
  test4();
  test5();
}
