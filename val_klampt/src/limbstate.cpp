#include "limbstate.h"
LimbState::LimbState(string name, vector<int> jIndex, vector<int> fixed, vector<int> fixedR, Hold fHold, Hold fHoldC, Hold hHold){
	limb=name;
	jointIndex=jIndex;
	fixedForManipulation=fixed;
	fixedRelaxed=fixedR;
	flatHold=fHold;
	hookHold=hHold;
	flatclimbHold=fHoldC;
}
