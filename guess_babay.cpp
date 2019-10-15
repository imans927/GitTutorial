#include <stdio.h>
#include <string.h>
#include "pred_controller.h"

//void guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh],float *roh_babay)
float guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh])
{

	float babay_rho=0.0f;

	float U_babay[3*Nh];

	float temp;
	// I have added a comment

	U_babay_cal_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		U_babay[row]=0;
		temp=0;

		U_babay_cal_col:for (int col=0; col < 3*Nh ; col++){

			U_babay[row]+= -1 * H_Hat_Inv(row,col)*theta_kk[col];
			//temp-= H_Hat_Inv(row,col)*theta_kk[col];

		}

		//printf("\n U_babay[%d]=%f" ,row,U_babay[row]);

		if (U_babay[row]>=0.5)
		//if (temp>=0.5)
				{ U_babay[row]=1;}

		else if (U_babay[row]<0.5 & U_babay[row]>=-0.5)
		//else if (temp<0.5 & temp>=-0.5)
				{ U_babay[row]=0;}  // rounding of -1 , 0 , 1

		else if (U_babay[row]<-0.5)
		//else if (temp<-0.5)
				{ U_babay[row]=-1; }
	};



	float temp_value;



	roh_babay_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		temp_value=0.0f;

		roh_babay_col:for (int col=0; col <= 3*Nh ; col++){

			if (col<=row){
				temp_value+=V_Gen(row,col)*U_babay[col];
			}

		}

		temp_value=U_unc_kk[row]-temp_value;
		temp_value=temp_value*temp_value;
		babay_rho=babay_rho+temp_value;


	};

	/*

	 printf("\n In IP edu_babay v_gen is\n");

		for (int row=0 ; row < v_gen_row ; row++){
			for (int col=0 ; col < v_gen_col ; col++)
			{

		printf("%f\t"  ,V_Gen(row,col));

			}

			printf("\n");
		}


		printf("In IP educated babay h_hat_inv is\n");

			for (int row=0 ; row < h_hat_inv_row ; row++){
				for (int col=0 ; col < h_hat_inv_col ; col++)
				{

			printf("%f\t"  ,H_Hat_Inv(row,col));

				}

				printf("\n");
			}

			printf("In IP educated babay theta_kk is\n");

				for (int row=0 ; row < 3*Nh ; row++){


				printf("%f\t"  ,theta_kk[row]);

					printf("\n");
				}

*/



	//*roh_babay=babay_rho;
	return babay_rho;

	}

