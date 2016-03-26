#include "Sector.h"

Sector::Sector(int r, int a)
{

	r_ = r;
	a_ = a;
	x = null;
	y = null;
	if(coutn == null)
		count = 0;
	id = Sector::count++;
	status = unexplored;
	//tags = vector<Target>();
}

Sector::~Sector()
{

}

int Sector::getId()
{
	return id;
}

int Sector::getStatus()
{
	return status;
}


int Sector::getTagCount()
{
	return tags.size();
}

int Sector::getUnclaimedTagCount()
{
	return 0;
}

vector<Target> Sector::getTags()
{
	return tags;
}

Target Sector::getTag(int i)
{
	return tags[i];
}

void Sector::addTag(Target t)
{
	tags.push_back(t);
}


double Sector::getX()
{
	return x_;
}

double Sector::getY()
{
	return r_;
}

double Sector::getR()
{
	return r_;
}

double Sector::getA()
{
	return a_;
}
