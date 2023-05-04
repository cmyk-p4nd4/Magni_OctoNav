#pragma once

#include <chrono>
#include <string>

#include <Eigen/Core>

namespace util {
	/**
	* @brief This timer class can be used in local scope
	* 
	*/
	class Timer
	{
	public:
		using TS = std::chrono::steady_clock;

		Timer();
		~Timer();

		void restart();
		double get_time();

		std::string time_string(int prec = 4);

	protected:
		void count();

	private:
		TS::time_point tp;
		std::string unit;
	};

	/**
	 * @brief decompose rotation matrix into Euler angle
	 * 
	 * @param matrix transformation matrix
	 * @return Eigen::Vector3f 
	 */
	Eigen::Vector3f rot_to_euler(const Eigen::Matrix4f & matrix);
	Eigen::Vector3f rot_to_euler(const Eigen::Matrix3f & matrix);

};
