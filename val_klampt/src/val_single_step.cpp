#include "val_single_step.h"

ValSingleStep::ValSingleStep(char* simpath, Robot& rob){
	worldrobotfile=simpath;
	robot=0;
	plan = Planner::getplanner(world, robot, worldrobotfile);
	rob=*world.GetRobot(world.robots[robot]->name);
}


//lock necessary joints for motion.
vector<int>* ValSingleStep::LockJoints(vector<int>* limbIndices){
	//0 	- 6		numobj, torso XYZ RPY
	//7 	- 12 	left_leg
	//13 	- 15	zeros
	//16	- 21 	r_leg
	//22 				zero
	//23	- 25	torso
	//26 				zero
	//27	- 46 	left hand
	//47 				zero
	//48	- 50 	head
	//51	- 54 	zeros
	//55 				Unknown%s
	//56 	- 63	zeroes
	//64	- 83 right hand


	// limbIndices->push_back(1);//torso Y
	// limbIndices->push_back(2);//torso Z pos
	limbIndices->push_back(3);//Torso R
	limbIndices->push_back(4);//Torso P
	limbIndices->push_back(5);//Torso y

	limbIndices->push_back(21);
	limbIndices->push_back(11);
	// //waist
	for(int j=24; j<26; j++)
	{			limbIndices->push_back(j);		}
	//neck
	for(int j=49; j<51; j++)
	{			limbIndices->push_back(j);	}

	//left arm and hand, excluding shoulder roll.
	for(int j=28; j<47; j++)
	{		//if(j != 27 )
				limbIndices->push_back(j);
	}
	//Right arm and hand, excluding shoulder roll.
	for(int j=65; j<84; j++)
	{		//if(j != 64 )
				limbIndices->push_back(j);
	}



	return limbIndices;

}


//lit leg in the Z direction
Config ValSingleStep::LiftLeg(int leg, Config startconfig, float height){
	vector<int> limbIndices;
	LockJoints(&limbIndices);
	// limbIndices.push_back(0);
	// limbIndices.push_back(1);
	limbIndices.push_back(2);

	plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[2]+=height;
	vector<ContactPoint> cp = plan->rlimbsState.lState[leg].flatHold.contacts;
	plan->rlimbsState.lState[leg].flatHold.contacts.clear();
	Config curconfig=plan->straightpath(limbIndices, leg, startconfig);
	plan->rlimbsState.lState[leg].flatHold.contacts=cp;
	return curconfig;
}

//shift the COM in the x and the Y direction
Config ValSingleStep::movecom(Config startconfig, float deltax, float deltay, float deltaz){
	vector<int> limbIndices;
	LockJoints(&limbIndices);
	if(!deltay)
	{	limbIndices.push_back(1);}
	if(!deltax)
	{limbIndices.push_back(0);}
	return plan->movecom(limbIndices, startconfig, deltax, deltay, deltaz);
}

//shift the COM in the x and the Y direction
Config ValSingleStep::movecomlasttest(Config startconfig, float deltax, float deltay){
	vector<int> limbIndices;
	LockJoints(&limbIndices);
	if(!deltay)
	{	limbIndices.push_back(1);}
	if(!deltax)
	{limbIndices.push_back(0);}
	return plan->movecomlasttest(limbIndices, startconfig, deltax, deltay);
}

//place leg on ground (lower leg in Z direction. )
Config ValSingleStep::placeleg(int leg, Config startconfig, float height){
	vector<int> limbIndices;
	LockJoints(&limbIndices);
	limbIndices.push_back(0);
	limbIndices.push_back(1);
	limbIndices.push_back(2);

	plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[2]-=height;
	vector<ContactPoint> cp = plan->rlimbsState.lState[leg].flatHold.contacts;
	plan->rlimbsState.lState[leg].flatHold.contacts.clear();
	Config curconfig=plan->straightpath(limbIndices, leg, startconfig);
	plan->rlimbsState.lState[leg].flatHold.contacts=cp;
	return curconfig;

	// plan->rlimbsState.lState[leg].flatHold.ikConstraint.endPosition[2]-=height;
	// return plan->straightpath(limbIndices, leg, startconfig);
}

//move leg in Y direction -- in front of robot.
Config ValSingleStep::moveleg(int leg, float x, float y, float z, Config startconfig){
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
void ValSingleStep::walk(MultiPath& mpath, int leg, float x, float y, float z){
	Config startconfig=plan->setstartconfig();
	Config robotstartconfig=startconfig;
	Config transitionconfig=startconfig;
	std::cout << "startconfig" <<startconfig<< '\n';


	float leg_height = .05;
	float right_movement = .10;
	float forward_movement = .5*y;

  if(leg)
  {
    //MOVE COM
    // //move to left
    transitionconfig=movecom(startconfig, -right_movement, 0, 1.2);
    plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
    startconfig=transitionconfig;

    //
    //MOVE COM
    transitionconfig=movecom(startconfig, 0, forward_movement, 1.2);
    plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
    startconfig=transitionconfig;
  }else{
    //MOVE COM
    // //move to left
    transitionconfig=movecom(startconfig, right_movement, 0, 1.2);
    plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
    startconfig=transitionconfig;
    //
    //MOVE COM
    transitionconfig=movecom(startconfig, 0, forward_movement, 1.2);
    plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
    startconfig=transitionconfig;
  }

	//
	//
  // // lift right
	// transitionconfig=LiftLeg(leg, startconfig, leg_height);
	// plan->addmilestonepath(leg, startconfig, transitionconfig, mpath);
	// startconfig=transitionconfig;
	//
	// //move right
	// transitionconfig=moveleg(leg, x, y, z, startconfig);
	// plan->addmilestonepath(leg, startconfig, transitionconfig, mpath);
	// startconfig=transitionconfig;
	//
	// //place right
	// transitionconfig=placeleg(leg, startconfig,leg_height);
	// plan->addmilestonepath(leg, startconfig, transitionconfig, mpath);
	// startconfig=transitionconfig;
	//
  // if(!leg)
  // {
  //   //MOVE COM
  //   // //move to left
  //   transitionconfig=movecom(startconfig, -right_movement, 0, 1.2);
  //   plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
  //   startconfig=transitionconfig;
  //   //
  //   //MOVE COM
  //   transitionconfig=movecom(startconfig, 0, forward_movement, 1.2);
  //   plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
  //   startconfig=transitionconfig;
  // }else{
  //   //MOVE COM
  //   // //move to left
  //   transitionconfig=movecom(startconfig, right_movement, 0, 1.2);
  //   plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
  //   startconfig=transitionconfig;
  //   //
  //   //MOVE COM
  //   transitionconfig=movecom(startconfig, 0, forward_movement, 1.2);
  //   plan->addmilestonepath(3, startconfig,transitionconfig, mpath);
  //   startconfig=transitionconfig;
  // }
}
