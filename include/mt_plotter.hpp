#include <pcl/visualization/pcl_plotter.h>

#include <thread>
#include <vector>
#include <utility>

class MTPlotter
{
public:
	using Plotter = pcl::visualization::PCLPlotter;

	MTPlotter();

	~MTPlotter();

    void spin();

	Plotter& addPlotter(std::string name = "PCL Plotter");
    std::vector<Plotter>::reference at(size_t idx);
    std::vector<Plotter>::const_reference at(size_t idx) const;

protected:
	void plotter_spin_handler(Plotter& plotter);

    /* Object + thread pair for interaction */
	std::vector<std::pair<Plotter *, std::thread>> _threads;
	std::vector<Plotter> _plotters;
};