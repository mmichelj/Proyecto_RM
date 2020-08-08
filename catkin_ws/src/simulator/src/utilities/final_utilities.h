/************************************************************************
*									*
*	final_utilities.h							*
*									*
*      This is a compilation of functions for the final proyect.	*
*									*
*									*
*			Miguel Michel					*
*			DEPFI-UNAM	11-2003				*
*									*
*************************************************************************/				
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>


int doorDetected(float cx, float cy, float nx, float ny, int obs){
	bool doorFlag = (obs != 0);
	bool currentExists = 0;
	bool nextExists = 0;
	if(doorFlag){
    	float doorNodes[8][2] = {
    	{0.2,1.05},
    	{0.23,1.45},
    	{0.6,0.4},
    	{0.92,0.4},
    	{1.2,0.4},
    	{1.5,0.36},
    	{1.5,1.74},
    	{1.2,1.8}
    	};
    	//printf("\nCurrent Node: {%f, %f} Next Node: {%f, %f}\n",cx,cy,nx,ny);
    	for (int i = 0; i <= 7; i++){
        	//printf("\nDoor node: {%f, %f}\n", doorNodes[i][0], doorNodes[i][1]);
        	currentExists = (cx == doorNodes[i][0])&&(cy == doorNodes[i][1]) || currentExists;
        	nextExists = (nx == doorNodes[i][0])&&(ny == doorNodes[i][1]) || nextExists;
    	}

    	printf("\n****doorFlag %d doorNode %d*****\n", doorFlag, currentExists&&nextExists);
    	return currentExists&&nextExists;
    
	}

return doorFlag;
}


