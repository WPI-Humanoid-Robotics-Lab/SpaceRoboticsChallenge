#include <string>
#include <cstring>
#include <fstream>
#include "Modeling/Paths.h"
#include "Modeling/MultiPath.h"
class LimbState{
	public:
		string limb;
		LimbState(string name, vector<int> jIndex, vector<int> fixed, vector<int> fixedR, Hold flatHold, Hold flatclimbHold, Hold hookHold);
		vector<int> jointIndex;
		vector<int> fixedForManipulation, fixedRelaxed;
		Hold flatHold, hookHold, flatclimbHold;
};
