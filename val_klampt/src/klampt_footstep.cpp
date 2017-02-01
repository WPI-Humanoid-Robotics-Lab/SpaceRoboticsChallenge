#include "ros/ros.h"

//klampt_footstep turns footstep requests into robot motions
int main(int argc, char **argv)
{
  ros::init(argc, argv, "klampt_footstep");

	ros::NodeHandle n;

	int leg;
	float x,y,z;
	const char* outputfile="walk.xml";
	char* flatwalkworld="../robonaut_r5.xml";
	Robot rob;
	MultiPath path;
	//FlatGroundWalk groundwalk(flatwalkworld, rob);
	ValSingleStep groundwalk(flatwalkworld, rob);
	leg = 0;
	x =	.00;
	y =	.08;
	z = .08;
	//groundwalk.walk(path);
	//Time scaling strategy
	groundwalk.walk(path, leg, x, y, z);
	groundwalk.walk(path, !leg, x, y, z);
	path.Save(outputfile);
	vector<Config> fullPath;
  vector<double> timeStep;
	InterpolateInverseCosine(rob, path, fullPath, timeStep);
	ofstream walkpath;
  walkpath.open ("walk.path");
  for(int i=0; i<fullPath.size(); i++){
    	walkpath <<timeStep[i] <<" "<<fullPath[i]<<"\n";
  }
  walkpath.close();
}
