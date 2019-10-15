#include <stdio.h>
#include <string.h>
#include "pred_controller.h"


void unconstrained (float R_Hat_a[8*Nh],float X_KK_a[4],float Y_Hat_a[6*Nh*Nh],float Y_Ref_KK_a[2*Nh],float U_KK_a[3*Nh],float V_Mul_H_Inv_a[9*Nh*Nh],float unconstrained[3*Nh],float theta_kk[3*Nh])
//void unconstrained (float *buff,float unconstrained[3*Nh],float theta_kk[3*Nh],float v_gen_copy[3*Nh][3*Nh],float U_unc_kk_copy[3*Nh])
{

	float temp[2*Nh];
	float accu[4]={0.0f,0.0f,0.0f,0.0f};
	float accu_value=0.0f;

	// calculation of r_hat*xkk=temp
	rHAt_Mul_xkk_row:for (int row=0 ; row < r_hat_row ; row++){  // row and col are defined according to left matrix
						temp[row]=0;
						rHAt_Mul_xkk_col:for (int col=0; col < r_hat_col; col++){
							//temp[row]+=R_Hat(row,col)*X_KK(col);
							accu[col]=R_Hat(row,col)*X_KK(col);
							}

							accu_temp:for(int i=0;i<4;i++)
							{
								accu_value+=accu[i];
								accu[i]=0.0f;
							}

							temp[row]=accu_value;
							accu_value=0.0f;

	}
	/*
		printf ("\n accu is\n");

			for (int row=0;row<4 ; row++){

				printf ("accu[%d]=%f\n,",row,accu[row]);

			}


	printf ("\n Temp is\n");

		for (int row=0;row<2*Nh ; row++){

			printf ("temp[%d]=%f\n,",row,temp[row]);

		}
*/


		// calculation of  Y_hat'*(temp-Yref) - labdau* U(kk)
	Theta_kk_row:for (int row=0 ; row < y_hat_col ; row++){   // row and col are defined according to left matrix
			theta_kk[row]=0;
		Theta_kk_col:for (int col=0 ; col < y_hat_row ; col++){
			theta_kk[row]+=Y_Hat(col,row)*(temp[col]-Y_Ref_KK(col));
			//Theta_kk_sub_col:for(int s_col=0;s_col<4;s_col++){
				//accu[s_col]+=Y_Hat(s_col+col,row)*(temp[s_col+col]-Y_Ref_KK(s_col+col));
		//	}

		}
/*
		accu_thetha:for(int i=0;i<4;i++)
		{
			accu_value+=accu[i];
			accu[i]=0.0f;
		}

		theta_kk[row]=accu_value;
		accu_value=0.0f;
*/


		if (row<3){
			theta_kk[row]=theta_kk[row]- labda_u*U_KK(row);
		}

	}

/*
printf ("\n Theta_kk is\n");

		for (int row=0;row<3*Nh ; row++){

			printf ("Theta_kk[%d]=%f\n,",row,theta_kk[row]);

		}
		*/



	unconstrained_row:for (int row=0 ; row < 3*Nh ; row++){   // row and col are defined according to left matrix

		unconstrained[row]=0;
			unconstrained_col:for (int col=0 ; col < 3*Nh ; col=col+4){

				//unconstrained[row]+= -V_Mul_H_Inv(row,col)*theta_kk[col];

				acc_part:for (int col_s=0;col_s<4;col_s++){

					accu[col_s]+=-V_Mul_H_Inv(row,col+col_s)*theta_kk[col+col_s];
				}
			}

				accu_uncos:for(int i=0;i<4;i++)
				{
					accu_value+=accu[i];
					accu[i]=0.0f;
				}

				unconstrained[row]=accu_value;
				accu_value=0.0f;

		}

/*
	printf ("\n U_unc_kk is\n");

		for (int row=0;row<3*Nh ; row++){

			printf ("U_unc_kk[%d]=%f\n,",row,unconstrained[row]);

		}
*/






}

