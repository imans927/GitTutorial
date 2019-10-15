#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pred_controller.h"

void sph_dec (float V_Gen_a[9*Nh*Nh],float roh, float U_unc[3*Nh], float U_opt[3*Nh]){




	// Initialize variables
	bool solution_found    = false;  // termination of while loop
	int level              = 0;      // current level of tree-search
	float dist_array[Nh*3]   = {0.0f};
    // accumulated costs
	float dist_temp        = 0;      // temporary costs for current step
	float dist_matmul      = 0;

	// Work with slightly high latency
	//float switch_point[Nh*3];
	//float U[Nh*3];
	//float sol[Nh*3];

	// Work with Low latency
	int switch_point[Nh*3];
	int U[Nh*3];
	int sol[Nh*3];


	//doesn't Work
	//float switch_point[Nh*3]   = {0.0f};   // {-1,0,1} switching possibilities
	//float U[Nh*3]              = {0.0f};
	//float sol[Nh*3]              = {0.0f}; // temporary storage of switching combination


	//static int U_opt[NN]	 = {0};		// optimal solution
	int ii,jj,kk,ll; // loop counter
	int i=0;

	init:for (ll = 0; ll < 3*Nh; ++ll)
	{
		switch_point[ll] = -1;
	}



	//while (solution_found==false)
	sphdec: for (int i=0;i< 35; i++)
		{



			//printf ("\n inside sphere decode algo level: %i, switch_point: %i, %i, %i ", level, switch_point[0],switch_point[1],switch_point[2]);
			U[level] = switch_point[level];
			dist_matmul = 0; // reset value
			dist_matmul:for (ii = 0; ii<12; ii++)
			{
				if (ii<=level){
				//#pragma HLS loop_tripcount min=0 max=12
				//dist_matmul += V_gen[level][ii]*U[ii]; // optimization possible: convert to switch
				if (U[ii]== 1) {dist_matmul += V_Gen(level,ii);}
				else if (U[ii]==-1){dist_matmul -= V_Gen(level,ii);}
				}

			}
			float dist_temp_temp=U_unc[level]-dist_matmul;
			//dist_temp =  (U_unc[level]-dist_matmul)*(U_unc[level]-dist_matmul) + dist_array[level];
			dist_temp = dist_temp_temp*dist_temp_temp+dist_array[level];

			// depth first search
			if (dist_temp <= roh + 1e-6) { //distance smaller than current radius?
				if (level == 3*Nh-1)
				{
					//Copy array U_opt = U;
					U_opt:for (jj = 0; jj < 3*Nh; ++jj)
					{
						U_opt[jj]= U[jj];
						//printf("\n sol[%d]=U[%d]=%d",jj,jj,U[jj]);

					}
					roh   = dist_temp;
					switch_point[3*Nh-1] +=1; // same as switch_point[level]
				}
				else
				{
					level += 1;
					dist_array[level] = dist_temp;
				}
			}	else switch_point[level] +=1;

			// Backtracking
			Backtracking:for (kk = 3*Nh-1; kk > 0; --kk)
			{
				if (switch_point[kk]>1)	{
					switch_point[kk] = -1;
					level = kk-1;
					switch_point[level] += 1;
				}
			}

			// Check if full tree is searched
			if (switch_point[0]>1) {
				solution_found = true;
				//printf("\n value of loop is %d",i);
			}
			//i=i+1;
		}

		//return U_opt; // returns pointer to U_opt
	/*
	datacopy:for (jj = 0; jj < 3*Nh; jj++)
	//for (jj = 0; jj < 3*Nh; jj++)
	{
		//U_opt[jj]=float(U[jj]);
		//U_opt[jj]=float(test);
		U_opt[jj]=sol[jj];
	}

	print:for (jj = 0; jj < 3*Nh; jj++)
	{
		printf("\n U[%d]=%d",jj,U[jj]);
	}
/*
	printf("\n solution is");
	for (jj = 0; jj < 3*Nh; ++jj)
	{
		printf("\n sol[%d]=%d",jj,sol[jj]);
	}*/



}
