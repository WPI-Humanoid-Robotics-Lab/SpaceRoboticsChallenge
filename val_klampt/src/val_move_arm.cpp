#include "val_move_arm.h"

ValMoveArm::ValMoveArm(char* simpath, Robot& rob){
	worldrobotfile=simpath;
	robot=0;
	plan = Planner::getplanner(world, robot, worldrobotfile);
	rob=*world.GetRobot(world.robots[robot]->name);
}


//lock necessary joints for motion.
vector<int>* ValMoveArm::LockJoints(vector<int>* limbIndices){
	// limbIndices->push_back(1);//torso Y
	// limbIndices->push_back(2);//torso Z pos
	limbIndices->push_back(3);//base R
	limbIndices->push_back(4);//base P
	limbIndices->push_back(5);//base y
	// limbIndices->push_back(6);//left_hip yaw
	// limbIndices->push_back(7);//left_hip roll
	// limbIndices->push_back(8);//left_hip pitch


	// limbIndices->push_back(21); //RIGHT FOOT ROLL
	// limbIndices->push_back(11); //LEFT FOOT ROLL
	// //waist
	limbIndices->push_back(24); //torso yaw
	limbIndices->push_back(25); //torso pitch
	limbIndices->push_back(26); //torso roll
	//neck
	for(int j=49; j<51; j++)
	{			limbIndices->push_back(j);	}



	//left leg and foot.
	for(int j=6; j<11; j++)
	{		//if(j != 27 )
				limbIndices->push_back(j);
	}
	//Right leg and foot.
	for(int j=16; j<21; j++)
	{		//if(j != 64 )
				limbIndices->push_back(j);
	}
	return limbIndices;

}
//move leg in Y direction -- in front of robot.
Config ValMoveArm::movearm(int leg, float x, float y, float z, Config startconfig){
	vector<int> limbIndices;
	LockJoints(&limbIndices);
	limbIndices.push_back(0);
	limbIndices.push_back(1);
	limbIndices.push_back(2);
	// limbIndices.push_back(3);
	// limbIndices.push_back(4);
	// limbIndices.push_back(5);

	plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[0]-=x;
	plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[1]-=y;
	plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[5]-=z;
	vector<ContactPoint> cp = plan->rlimbsState.lState[leg].flatHold.contacts;
	for(int i=0; i<cp.size(); i++){
		cp[i].x[1]=cp[i].x[1]+y;
	}
	plan->rlimbsState.lState[leg].flatHold.contacts.clear();
	Config curconfig=plan->straightpath(limbIndices, leg, startconfig);
	plan->rlimbsState.lState[leg].flatHold.contacts=cp;
	return curconfig;
}


//series of executions for walking
//use movCOM, liftleg, moveleg, place leg
void ValMoveArm::move(MultiPath& mpath, int limb, float x, float y, float z){
	Config startconfig=plan->setstartconfig();
	Config robotstartconfig=startconfig;
	Config transitionconfig=startconfig;
	std::cout << "startconfig" <<startconfig<< '\n';


	float leg_height = .05;
	float right_movement = .12;
	float forward_movement = .06;//.5*y;

    //MOVE COM
    // //move to left
    transitionconfig=movearm(limb,x,y,z,startconfig);
    plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
    startconfig=transitionconfig;

}
