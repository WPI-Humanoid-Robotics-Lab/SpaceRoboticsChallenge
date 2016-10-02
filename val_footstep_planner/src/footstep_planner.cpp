#include "../include/footstep_planner.h"
#include <iostream>
int main(){
	//Testing the data structure
	FootstepSequence fsequence;
	for(int i=0; i<10; i++){
		FootLocation fLocation(l_foot, 1.2, 2.2, 2.1,0.5);
		fsequence.sequence.insert(std::pair<int, FootLocation> (i, fLocation));
	}
	
	for(int k=0; k<10; k++){
		
		FootLocation fl=fsequence.sequence.find(k)->second;
		Position x;
		fl.GetPosition(x);
		std::cout<<x.xPos<<"\n";
	}
	return 0;
}
