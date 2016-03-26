#ifndef SECTOR_H
#define SECTOR_H

class vector;
class Target;

class Sector
{
	public:
		Sector();
		~Sector();

		enum SEARCH_STATUS {unexpolored, empty, cleared, tagsFound, mineFound, honeyPot};

		static int count;
			int getId();
			int getStatus();
			int getTagCount();
			int getUnclaimedTagCount();
			vector<Target> getTags();
			Target getTag(int i);
			void addTag(Target t);
			double getR();
			double getA();
			double getX();
			double getY();

	private:

			double r_, a_;
			double x_, y_;
			int id;
			int status;
			std::vector<Target> tags;

};

#endif // SECTOR_H