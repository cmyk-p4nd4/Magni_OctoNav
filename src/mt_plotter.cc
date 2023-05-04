#include <functional>
#include <mt_plotter.hpp>
#include <utility>

MTPlotter::MTPlotter() {
	this->_plotters.reserve(10);
}

MTPlotter::~MTPlotter() {
	for(auto& pair : this->_threads) {
		auto& t = std::get<std::thread>(pair);
		if(t.joinable()) {
			std::get<Plotter *>(pair)->close();
			t.join();
		}
	}
}

void MTPlotter::spin() {
	for(auto& plot : this->_plotters) {
		this->_threads.push_back(
			std::make_pair(&plot, std::thread(&MTPlotter::plotter_spin_handler, this, std::ref(plot))));
	}
}

std::vector<MTPlotter::Plotter>::reference MTPlotter::at(size_t idx) {
	return this->_plotters.at(idx);
}
std::vector<MTPlotter::Plotter>::const_reference MTPlotter::at(size_t idx) const {
	return this->_plotters.at(idx);
}

MTPlotter::Plotter& MTPlotter::addPlotter(std::string name) {
	this->_plotters.push_back(Plotter(name.c_str()));

	return this->_plotters.back();
}

void MTPlotter::plotter_spin_handler(Plotter& plotter) {
	plotter.spin();
	return;
}