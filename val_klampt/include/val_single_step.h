#include "planner.h"
class ValSingleStep{
	public:
		ValSingleStep(char* simpath, Robot& rob);
		void walk(MultiPath& mpath, int leg, float x, float y, float z);
		Config movecom(Config startconfig, float deltax, float deltay, float zheight);
		Config LiftLeg(int leg, Config startconfig, float height);
		Config moveleg(int leg, float x,float y, float z, Config startconfig);
		Config placeleg(int leg, Config startconfig, float height);
		Config movecomlasttest(Config startconfig, float deltax, float deltay);
		vector<int>* LockJoints(vector<int>* limbIndices);
		Planner* plan;
		RobotWorld world;
		int robot;
		char* worldrobotfile;
	private:
		Config startconfig;
};
