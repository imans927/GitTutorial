#include <stdio.h>
#include <string.h>
#include "pred_controller.h"

// I have added a comment after first commit
// I have corrected the comment

//float guess_edu (float U_KK_a[3*Nh],float V_Gen_a[9*Nh*Nh],float U_unc_kk[3*Nh])
float guess_edu (float U_KK_a[3*Nh],float V_Gen_a_cpy[9*Nh*Nh],float U_unc_kk[3*Nh])
{



	float educated_rho=0.0;

	float u_educated[3*Nh]={};


	u_educated_cal_1:for (int i=3*Nh-3; i<3*Nh; i++)
	{
		u_educated[i]=U_KK(i);
		//u_educated[i]=u_kk[i];

	};

	u_educated_cal_2:for (int i=0; i < 3*Nh-3 ; i++)
	{

		u_educated[i]=U_KK(i+3);
		//u_educated[i]=u_kk[i+3];

	};


	float temp_value;
	//float temp_fr_tem_value;
	//float temp_roh_edu;



	roh_educated_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		temp_value=0.0f;

		roh_educated_col:for (int col=0; col < 3*Nh ; col++){

			if (col<=row){
			//temp_value+=V_Gen(row,col)*u_educated[col];
			temp_value+=V_Gen_cpy(row,col)*u_educated[col];
			//temp_value+=v_gen[row][col]*u_educated[col];
			}
		}

		//temp_fr_tem_value=temp_value;
		temp_value=U_unc_kk[row]-temp_value;
		//temp_fr_tem_value=temp_value;
		temp_value=temp_value*temp_value;
		//temp_roh_edu=educated_rho;
		educated_rho=educated_rho+temp_value;


	};

	/*
	 printf("\n U_educated is\n");

for (int i=0; i<3*Nh;i++)
{
	printf ("educated[%d]=%f\n,",i,u_educated[i]);

}


	printf("\n In IP edu_guess v_gen is\n");

		for (int row=0 ; row < v_gen_row ; row++){
			for (int col=0 ; col < v_gen_col ; col++)
			{

		printf("%f\t"  ,V_Gen(row,col));

			}

			printf("\n");
		}


		printf("In IP educated guess h_hat_inv is\n");

			for (int row=0 ; row < h_hat_inv_row ; row++){
				for (int col=0 ; col < h_hat_inv_col ; col++)
				{

			printf("%f\t"  ,H_Hat_Inv(row,col));

				}

				printf("\n");
			}

			*/



return educated_rho;

}


