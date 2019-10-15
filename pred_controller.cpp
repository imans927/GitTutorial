#include <stdio.h>
#include <string.h>
#include "pred_controller.h"





/*
void predictive_controller(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *Y_HAT_src,
							volatile float *R_HAT_src,volatile float *V_MUL_H_INV_src,
							volatile float *V_GEN_src,volatile float *H_HAT_INV_src,
							volatile float *out)
*/
void predictive_controller(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *out)


{
#pragma HLS INTERFACE m_axi depth=12 port=out offset=slave bundle=data
	/*
#pragma HLS INTERFACE m_axi depth=144 port=H_HAT_INV_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=144 port=V_GEN_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=144 port=V_MUL_H_INV_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=32 port=R_HAT_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=96 port=Y_HAT_src offset=slave bundle=data
*/
#pragma HLS INTERFACE m_axi depth=12 port=U_KK_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=8 port=Y_REF_KK_src offset=slave bundle=data
#pragma HLS INTERFACE m_axi depth=4 port=X_KK_src offset=slave bundle=data

#pragma HLS INTERFACE s_axilite port=return bundle=crtl_bus
//#pragma HLS INTERFACE m_axi depth=597 port=a offset=slave bundle=ctrl_bus

  int i;
  //float buff[SIZE];
  float X_KK_a[4];
  float Y_Ref_KK_a[2*Nh];
  float U_KK_a[3*Nh];

  static float Y_Hat_a[6*Nh*Nh]={
		    0.0198,   -0.0099,   -0.0099,         0,         0,         0,         0,         0,         0,         0,         0,         0,
		   -0.0000,    0.0172,   -0.0172,         0,         0,         0,         0,         0,         0,         0,         0,         0,
		    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,         0,         0,         0,         0,         0,         0,
		   -0.0000,    0.0172,   -0.0172,   -0.0000,    0.0172,   -0.0172,         0,         0,         0,         0,         0,         0,
		    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,         0,         0,        0,
		   -0.0000,    0.0171,   -0.0171,   -0.0000,    0.0172,   -0.0172,   -0.0000,    0.0172,   -0.0172,         0,         0,         0,
		    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,    0.0198,   -0.0099,   -0.0099,
		   -0.0000,    0.0171,   -0.0171,   -0.0000,    0.0171,   -0.0171,   -0.0000,    0.0172,   -0.0172,   -0.0000,    0.0172,   -0.0172,
	};
  static float R_Hat_a[8*Nh]={
		    0.9994,    0.0000,    0.0002,    0.0294,
		   -0.0000,    0.9994,   -0.0294,    0.0002,
		    0.9988,    0.0000,    0.0007,    0.0588,
		   -0.0000,    0.9988,   -0.0588,    0.0007,
		    0.9982,    0.0000,    0.0014,    0.0882,
		   -0.0000,    0.9982,   -0.0882,    0.0014,
		    0.9976,    0.0000,    0.0023,    0.1176,
		   -0.0000,    0.9976,   -0.1176,    0.0023,
	};

  static float V_Mul_H_Inv_a[9*Nh*Nh]={
		    9.7928,    2.6122,    2.6122,    6.5996,    4.2087,    4.2087,    5.0237,    4.9967,    4.9967,    4.5126,    5.2523,    5.2522,
		   -0.0000,    9.4379,    1.9874,    2.5404,    5.6829,    3.2021,    3.7941,    3.8296,    3.8016,    4.2007,    3.2286,    3.9960,
		     0.0000,   -0.0000,    9.2263,    2.0514,    2.0514,    5.1235,    3.0639,    3.0639,    3.0985,    3.3922,    3.3922,    2.4419,
		    0.0000,    0.0000,    0.0000,   10.1026,    2.2270,    2.2270,    7.6689,    3.4439,    3.4439,    6.8013,    3.8777,    3.8776,
		    0.0000,    0.0000,    0.0000,    0.0000,    9.8540,    1.7799,    1.7975,    7.0840,    2.7524,    2.4384,    6.0965,    3.0991,
		   -0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,    9.6920,    1.4975,    1.4975,    6.6970,    2.0314,    2.0313,    5.6293,
		   -0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0000,    0.0000,   10.8981,    1.2979,    1.2979,    9.8773,    1.8083,    1.8083,
		   -0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,   10.8206,    1.1515,    0.6365,    9.7312,    1.6044,
		    0.0000,   -0.0000,    0.0000,    0.0000,    0.0000,    0.0000,         0,    0.0000,   10.7591,    0.5720,    0.5720,    9.6151,
		    0.0000,   -0.0000,    0.0000,         0,   -0.0000,    0.0000,    0.0000,    0.0000,         0,   11.7999,    0.3314,    0.3314,
		    0.0000,         0,    0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,    0.0000,         0,   11.7953,    0.3222,
		    0.0000,   0.0000,   0.0000,    0.0000,    0.0000,   -0.0000,         0,    0.0000,         0,         0,   -0.0000,   11.7909,
	};
  static float V_Gen_a[9*Nh*Nh*1]={
		    0.1021,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
		   -0.0283,    0.1060,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
		   -0.0228,   -0.0228,    0.1084,         0,         0,         0,         0,         0,         0,         0,         0,         0,
		   -0.0550,   -0.0220,   -0.0220,    0.09901,         0,         0,         0,         0,         0,         0,         0,         0,
		   -0.0101,   -0.0514,   -0.0176,   -0.0224,    0.1015,         0,         0,         0,         0,         0,         0,         0,
		   -0.0084,   -0.0084,  -0.0490,   -0.0186,   -0.0186,    0.1032,         0,         0,         0,         0,         0,         0,
		    0.0107,   -0.0053,   -0.0053,   -0.0634,   -0.0142,   -0.0142,    0.0918,         0,         0,         0,         0,         0,
		   -0.0067,    0.0114,   -0.0047,   -0.0067,   -0.0622,   -0.0126,   -0.0110,    0.0924,         0,         0,         0,         0,
		   -0.0060,   -0.0060,    0.0120,   -0.0060,   -0.0060,   -0.0612,   -0.0099,   -0.0099,    0.0929,         0,         0,         0,
		    0.0045,   -0.0022,   -0.0022,    0.0045,   -0.0023,   -0.0023,   -0.0757,   -0.0045,   -0.0045,    0.0847,         0,         0,
		   -0.0024,    0.0046,   -0.0022,   -0.0024,    0.0046,   -0.0022,   -0.0024,   -0.0756,   -0.0044,   -0.0024,    0.0848,         0,
		   -0.0023,   -0.0023,    0.0046,   -0.0023,   -0.0023,    0.0046,   -0.0023,   -0.0023,   -0.0755,   -0.0023,   -0.0023,    0.0848,

	};

  float V_Gen_a_cpy[9*Nh*Nh];
  float H_Hat_Inv_a[9*Nh*Nh]={

		   95.8982,   25.5803,   25.5803,   64.6286,   41.2152,   41.2151,   49.1956,   48.9317,   48.9315,   44.1908,   51.4341,   51.4339,
		   25.5803,   95.8982,   25.5803,   41.2151,   64.6286,   41.2152,   48.9315,   49.1956,   48.9317,   51.4339,   44.1908,   51.4341,
		   25.5803,   25.5803,   95.8982,   41.2152,   41.2151,   64.6286,   48.9317,   48.9315,   49.1956,   51.4341,   51.4339,   44.1908,
		   64.6286,   41.2151,   41.2152,  156.2787,   68.9195,   68.9195,  126.5534,   83.7822,   83.7820,  116.1222,   88.9978,   88.9976,
		   41.2152,   64.6286,   41.2151,   68.9195,  156.2787,   68.9195,   83.7820,  126.5534,   83.7822,   88.9976,  116.1222,   88.9978,
		   41.2151,   41.2152,   64.6286,   68.9195,   68.9195,  156.2787,   83.7822,   83.7820,  126.5534,   88.9978,   88.9976,  116.1222,
		   49.1956 ,  48.9315,   48.9317,  126.5534,   83.7820,   83.7822,  232.0749,  104.5508,  104.5508,  216.2292,  112.4737,  112.4736,
		   48.9317,   49.1956,   48.9315,   83.7822,  126.5534,   83.7820,  104.5508,  232.0749,  104.5508,  112.4736,  216.2292,  112.4737,
		   48.9315,   48.9317,   49.1956,   83.7820,   83.7822,  126.5534,  104.5508,  104.5508,  232.0749,  112.4737,  112.4736,  216.2292,
		   44.1908,   51.4339,   51.4341,  116.1222,   88.9976,   88.9978,  216.2292,  112.4736,  112.4737,  343.3786,  122.4283,  122.4283,
		   51.4341,   44.1908,   51.4339,   88.9978,  116.1222,   88.9976,  112.4737,  216.2292,  112.4736,  122.4283,  343.3786,  122.4283,
		   51.4339,   51.4341,   44.1908,   88.9976,   88.9978,  116.1222,  112.4736,  112.4737,  216.2292,  122.4283,  122.4283,  343.3786,
};
  float dummy[3*Nh];


  float U_unc_kk[3*Nh];
  float U_unc_kk_cpy[3*Nh];
  float roh_educated;
  float roh_babay;
  float theta_kk[3*Nh];
  float U_opt[3*Nh];

  //float roh_try;


  //memcpy creates a burst access to memory
  //multiple calls of memcpy cannot be pipelined and will be scheduled sequentially
  //memcpy requires a local buffer to store the results of the memory transaction


 //memcpy(buff,(const int*)a,597*sizeof(float));
/*
  burst_loop:for (int qq=0; qq<SIZE;qq++){
#pragma HLS PIPELINE
	  buff[qq]=a[qq];
  }
  */

int qq=0;

x_kk_cpy:for(int row=0;row<4;row++){
#pragma HLS PIPELINE
	  	  X_KK_a[row]=X_KK_src[row];
}
////


y_ref_kk_cpy:for(int row=0;row<2*Nh;row++){
#pragma HLS PIPELINE
	Y_Ref_KK_a[row]=Y_REF_KK_src[row];
 }

u_kk_cpy:for(int row=0;row<3*Nh;row++){
#pragma HLS PIPELINE
	U_KK_a[row]=U_KK_src[row];
 }
/*
y_hat_cpy:for(int row=0;row<6*Nh*Nh;row++){

#pragma HLS PIPELINE

	Y_Hat_a[row]=Y_HAT_src[row];
	  }

r_hat_cpy:for(int row=0;row<8*Nh;row++){

#pragma HLS PIPELINE
	R_Hat_a[row]=R_HAT_src[row];
	  }

VHinv_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	V_Mul_H_Inv_a[row]=V_MUL_H_INV_src[row];
	  }

Vgen_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	V_Gen_a[row]=V_GEN_src[row];
	  }


Hhat_inv_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	H_Hat_Inv_a[row]=H_HAT_INV_src[row];
	  }
*/

  unconstrained(R_Hat_a,X_KK_a,Y_Hat_a,Y_Ref_KK_a,U_KK_a,V_Mul_H_Inv_a,U_unc_kk,theta_kk);


  U_Unc_kk_copy:for (int row=0;row<3*Nh;row++)
  {
	  	  U_unc_kk_cpy[row]=U_unc_kk[row];
  }


  V_gen_copy:for (int row=0;row<9*Nh*Nh;row++)
  {
	  	  V_Gen_a_cpy[row]=V_Gen_a[row];
  }




	//roh_educated=guess_edu(buff,U_unc_kk);
  //roh_babay=guess_babay(buff,U_unc_kk,theta_kk);


  //roh_educated=guess_edu (U_KK_a,V_Gen_a,U_unc_kk);
  roh_educated=guess_edu (U_KK_a,V_Gen_a_cpy,U_unc_kk_cpy);
  roh_babay=guess_babay(V_Gen_a,H_Hat_Inv_a,U_unc_kk,theta_kk);



	float roh;

	if (roh_educated < roh_babay)
	{
		roh=roh_educated;
	}
	else
		roh=roh_babay;

	sph_dec(V_Gen_a,roh,U_unc_kk,U_opt);


	printf("\n roh educated is %.9f \n",roh_educated );
	printf("\n roh babay by value is %.9f \n",roh_babay );
	//printf("\n roh babay by reference is %.9f \n",roh_babay );




//a[3*Nh]=roh_educated;
//a[3*Nh+1]=roh_babay;

 memcpy((int *)out,U_opt,3*Nh*sizeof(int));

 //// function end

}




/*
////////////////////////////////////////////////////////////////////////////////
// Printing Matrices in IP to check values // can be commented out, just there to verify
 // matrices copied correctly into IP

 printf("In IP X_KK is");

 	for (int row=0 ; row < x_kk_row ; row++)
 		{

 	printf("\n%f",X_KK(row));
 	printf("\n");

 		}
 	//

 printf("In IP Y_ref_KK is");

 	for (int row=0 ; row < y_ref_kk_row ; row++)
 		{

 	printf("\n%f",Y_Ref_KK(row));
 	printf("\n");

 		}
 	//

 printf("In IP U_KK is");

 	for (int row=0 ; row < u_kk_row ; row++)
 		{

 	printf("\n%f",U_KK(row));
 	printf("\n");

 		}

 printf("In IP Y_hat is\n");

 	for (int row=0 ; row < y_hat_row ; row++){
 		for (int col=0 ; col < y_hat_col ; col++)
 		{

 	printf("%f\t"  ,Y_Hat(row,col));


 		}

 		printf("\n");
 	}


 	printf("In IP R_hat is\n");

 		for (int row=0 ; row < r_hat_row ; row++){
 			for (int col=0 ; col < r_hat_col ; col++)
 			{

 		printf("%f\t"  ,R_Hat(row,col));


 			}

 			printf("\n");
 		}


 printf("In IP V_mul_H_inv is\n");

 	for (int row=0 ; row < v_mul_h_inv_row ; row++){
 		for (int col=0 ; col < v_mul_h_inv_col ; col++)
 		{

 	printf("%f\t"  ,V_Mul_H_Inv(row,col));


 		}

 		printf("\n");
 	}


	 printf("In IP V_gen is\n");

	 	for (int row=0 ; row < v_gen_row ; row++){
	 		for (int col=0 ; col < v_gen_col ; col++)
	 		{

	 	printf("%f\t"  ,V_Gen(row,col));


	 		}

	 		printf("\n");
	 	}



 	 printf("In IP H_hat_inv is\n");

 	 	for (int row=0 ; row < h_hat_inv_row ; row++){
 	 		for (int col=0 ; col < h_hat_inv_col ; col++)
 	 		{

 	 	printf("%f\t"  ,H_Hat_Inv(row,col));


 	 		}

 	 		printf("\n");
 	 	}

////////////////////////////////////////////////////////////////////
}*/
