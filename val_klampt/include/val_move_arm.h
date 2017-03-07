#include "planner.h"
class ValMoveArm{
	public:
		ValMoveArm(char* simpath, Robot& rob);
		void move(MultiPath& mpath, int leg, float x, float y, float z);
		Config movearm(int arm, float x,float y, float z, Config startconfig);
		// Config placeleg(int leg, Config startconfig, float height);
		// Config movecomlasttest(Config startconfig, float deltax, float deltay);
		vector<int>* LockJoints(vector<int>* limbIndices);
		Planner* plan;
		RobotWorld world;
		int robot;
		char* worldrobotfile;
	private:
		Config startconfig;
};
