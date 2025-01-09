#include <chrono>
#include <iostream>
#include <movutl/app/app.hpp>
#include <movutl/app/ui.hpp>
#include <thread>

int main() {
  mu::init();
  std::cout << "movutl project !!" << std::endl;
  while(!mu::should_terminate()) {
    mu::update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  mu::terminate();
}
