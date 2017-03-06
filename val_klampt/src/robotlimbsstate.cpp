#include "robotlimbsstate.h"

//Load Limb1 Joints
vector<int> RobotLimbsState::setjointstate(int leg){
	vector<int> limbIndices;
	return limbIndices;
}

vector<int> RobotLimbsState::setfixedjoints(int leg){
	//TODO: Use this
	vector<int> joints=setjointstate(leg);
	return joints;
}

vector<int>RobotLimbsState::setrelaxedjoints(int leg){
	//TODO: Use this
	vector<int> joints=setjointstate(leg);
	return joints;
}

Hold RobotLimbsState::gethold(int limb, int flat){
	string holdFolder="src/space_robotics_challenge/val_klampt/footstep_data/";
	string holdfile;
	switch (limb){
			case 1:
				holdfile=holdFolder+"legone.hold";
				break;
			case 2:
				holdfile=holdFolder+"legtwo.hold";
				break;
			case 3:
				holdfile=holdFolder+"arm1.hold";
				break;
			case 4:
				holdfile=holdFolder+"arm2.hold";
				break;
			default:
				break;
	}
	const char* hold_file=holdfile.c_str();
	ifstream in(hold_file, ios::in);
	if(!in){
		cout<<"Error loading hold file\n";
		exit(1);
	}
	Hold limbHold;
	in >> limbHold;
	return limbHold;
}

//Set up the limb states for all limbs
RobotLimbsState::RobotLimbsState(){
	//TODO: Read from configuration files
	const string name[]={"leg1", "leg2","arm1","arm2"};
	for(int i=0; i<numlimbs; i++){
		vector<int> fixedjoints= setfixedjoints(i+1);
		vector<int> relaxedjoints=setrelaxedjoints(i+1);
		LimbState lst(name[i], setjointstate(i+1), fixedjoints, relaxedjoints, gethold(i+1, 0), gethold(i+1, 1), gethold(i+1, 2));
		lState.push_back(lst);
	}

}

//Get the limb states
LimbState RobotLimbsState::getlimbstate(int leg){
	return lState[leg];
}

vector<LimbState> RobotLimbsState::getlimbsstate(){
	vector<LimbState> lstate;
	for(int i=0; i<numlimbs; i++){
		lstate.push_back(getlimbstate(i));
	}
	return lstate;
}
