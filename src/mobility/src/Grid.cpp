#ifndef GRID_H
#define GRID_H

#include <vector>
class Sector;

//keep track of explored area and distribution of tags
class Grid
{
	public:
		Grid(int gridSize, int sectionSize);

		Sector getSector(double r, double a);
		std::vector<Sector> getRing(double r);

		int getRings();
		int getSections(int r);
		int getWidth();
		int getHeight();

	private:
		int sectionSize_;
		int gridSize_;
		double r_, a_;
		double x_, y_;
		std::vector< vector<Sector> > grid;	// 2 d vector so outter sections can have increased precision/ fixed size sector

};

#endif //grid h
