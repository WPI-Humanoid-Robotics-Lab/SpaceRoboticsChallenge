#include "limbstate.h"
#include <vector>
class RobotLimbsState{
	public:
		static const int numlimbs=4;//(changed from 2)
		RobotLimbsState();
		vector<int> setjointstate(int leg);
		vector<int> setfixedjoints(int leg);
		vector<int> setrelaxedjoints(int leg);
		Hold gethold(int leg, int flat);
		LimbState getlimbstate(int leg);
		vector<LimbState> getlimbsstate();
		vector<LimbState> lState;
};
