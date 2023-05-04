#include <utils.hpp>
#include <chrono>

#include <sstream>
#include <iostream>

#include <numeric>

#include <Eigen/Geometry>

namespace util {
  Timer::Timer() {
    count();
  }

  Timer::~Timer() {
    std::cout << this->time_string();
  }

  void Timer::count() {
    this->tp = TS::steady_clock::now();
    this->unit.clear();
  }

  void Timer::restart() {
    count();
  }

  double Timer::get_time() {
    using std::chrono::duration;
    TS::time_point stop = TS::now();

    TS::duration diff = stop - this->tp;
    double c = duration<double, std::micro>(diff).count();
    
    if (c > 1000.) { // check micro
      c /= 1000.;
      if (c > 1000.) { // check milli
        c /= 1000.;
        this->unit = "s";
        return c;
      }
      this->unit = "ms";
      return c;
    }
    this->unit= "us";

    return c;
  }

  std::string Timer::time_string(int prec /* = 4 */) {
    std::ostringstream os;

    double t = this->get_time();

    os.precision(prec);
    os << t << ' ' + this->unit << "\r\n";

    return os.str();
  }

	Eigen::Vector3f rot_to_euler(const Eigen::Matrix4f& matrix) {
		return rot_to_euler(Eigen::Matrix3f(matrix.topLeftCorner(3, 3)));
	}

	Eigen::Vector3f rot_to_euler(const Eigen::Matrix3f& matrix) {
    return matrix.eulerAngles(2,1,0);
	}
};
