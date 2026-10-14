#pragma once

#include <chrono>
#include <iostream>
#include <string>

class TicTocV2 {
public:
  TicTocV2() {
    tic();
  }
  TicTocV2(bool disp) : disp_(disp) {
    tic();
  }
  void tic() {
    start = std::chrono::system_clock::now();
  }
  void toc(std::string about_task) {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    double elapsed_ms = elapsed_seconds.count() * 1000.0;
    if (disp_) {
      std::cout.precision(3);
      std::cout << about_task << ": " << elapsed_ms << " msec." << std::endl;
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  bool disp_{false};
};
