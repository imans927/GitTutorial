#include <stdio.h>
#include "pred_controller.h"


int main()
{
  int i;

  float x_kk[4];
  float y_ref_kk[2*Nh];
  float u_kk[3*Nh];
  float y_hat[2*Nh][3*Nh];
  float r_hat[2*Nh][4];

  float v_mul_h_inv[3*Nh][3*Nh];
  float v_gen[3*Nh][3*Nh];
  float h_hat_inv[3*Nh][3*Nh];

  float y_hat_src[6*Nh*Nh];
  float r_hat_src[8*Nh];
  float v_mul_H_inv_src[9*Nh*Nh];
  float v_gen_src[9*Nh*Nh];
  float h_hat_inv_src[9*Nh*Nh];
  float out[3*Nh];
  int index=0;

  // For Nh=2 Horizons , different testbench answers
  //float answer[3*Nh+2]={-0.023373,0.080930,-0.060741,-0.010329,0.007841,0.002860,0.002870,0.007221};
  //float answer[3*Nh+2]={0.00,1.00,-1.00,0.00,1.00,-1.00,0.002870,0.007221};
  //float answer[3*Nh+2]={2,-1.00,-1.00,-1.00,-1.00,-1.00,0.002870,0.007221};
  //float answer[3*Nh+2]={1.00,1.00,1.00,1.00,1.00,1.00,0.002870,0.007221};
  //float answer[3*Nh]={0.00,1.00,-1.00,0.00,1.00,-1.00};

  // for Nh=3, Uunc and Uopt test benches answers
  //float answer[3*Nh]={-0.0143,0.1032,-0.1023,-0.0127,0.0432,-0.0322,-0.0058,0.0199,-0.0143};
  //float answer[3*Nh]={0,1,-1,0,1,-1,0,1,-1};

  // for Nh=4 U_unc and U_opt test benche answers

  //float answer[3*Nh]={   -0.0204,0.1065,-0.0985,-0.0230,0.0678,-0.0468,-0.0194,0.0574,-0.0395,-0.0087,0.0256,-0.0172};
  float answer[3*Nh]={0,1,-1,0,1,-1,0,1,-1,0,1,-1};




  //float Mem[50];
  float Mem2[SIZE];
  //float q=0.351;
  int Mem2index;

  printf("HLS AXI-Stream no side-channel data example\n");
  //Initiating Matrices

offline(x_kk,y_ref_kk,u_kk,y_hat,r_hat,v_mul_h_inv,v_gen,h_hat_inv);

  for(i=0; i < SIZE; i++){
   // Mem[i] = 0;
    Mem2[i] = 0;
  }


  //Copying arrays into memory

  Mem2index=x_kk_offset;

	for (int row=0 ; row < x_kk_row ; row++)
		{

			Mem2[Mem2index]=x_kk[row];
			Mem2index++;

		}

	Mem2index=y_ref_kk_offset;


	for (int row=0 ; row < y_ref_kk_row ; row++)
		{

		Mem2[Mem2index]=y_ref_kk[row];
		Mem2index++;

		}

	Mem2index=u_kk_offset;

	for (int row=0 ; row < u_kk_row ; row++)
		{

		Mem2[Mem2index]=u_kk[row];
		Mem2index++;

		}

	Mem2index=y_hat_offset;

	for (int row=0 ;row < y_hat_row ; row++)
		for (int col=0 ;col < y_hat_col ; col++)
		{

			Mem2[Mem2index]=y_hat[row][col];
			Mem2index++;

		}

	Mem2index=r_hat_offset;

	for (int row=0 ;row < r_hat_row ; row++)
		for (int col=0 ;col < r_hat_col ; col++)
		{

			Mem2[Mem2index]=r_hat[row][col];
			Mem2index++;

		}

	Mem2index=v_mul_h_inv_offset;

	for (int row=0 ;row < v_mul_h_inv_row ; row++)
		for (int col=0 ;col < v_mul_h_inv_col ; col++)
		{

			Mem2[Mem2index]=v_mul_h_inv[row][col];
			Mem2index++;

		}


	Mem2index=v_gen_offset;

	for (int row=0 ;row < v_gen_row ; row++)
		for (int col=0 ;col < v_gen_col ; col++)
		{

			Mem2[Mem2index]=v_gen[row][col];
			Mem2index++;

		}

	Mem2index=h_hat_inv_offset;

	for (int row=0 ;row < h_hat_inv_row ; row++)
		for (int col=0 ;col < h_hat_inv_col ; col++)
		{

			Mem2[Mem2index]=h_hat_inv[row][col];
			Mem2index++;

		}





  ////////


	index=0;

	for (int row=0 ;row < y_hat_row ; row++)
		for (int col=0 ;col < y_hat_col ; col++)
		{

			y_hat_src[index]=y_hat[row][col];
			index++;

		}

	index=0;

	for (int row=0 ;row < r_hat_row ; row++)
		for (int col=0 ;col < r_hat_col ; col++)
		{

			r_hat_src[index]=r_hat[row][col];
			index++;

		}

	index=0;

	for (int row=0 ;row < v_mul_h_inv_row ; row++)
		for (int col=0 ;col < v_mul_h_inv_col ; col++)
		{

			v_mul_H_inv_src[index]=v_mul_h_inv[row][col];
			index++;

		}


	index=0;

	for (int row=0 ;row < v_gen_row ; row++)
		for (int col=0 ;col < v_gen_col ; col++)
		{

			v_gen_src[index]=v_gen[row][col];
			index++;

		}

	index=0;

	for (int row=0 ;row < h_hat_inv_row ; row++)
		for (int col=0 ;col < h_hat_inv_col ; col++)
		{

			h_hat_inv_src[index]=h_hat_inv[row][col];
			index++;

		}







  //Call the hardware function
//predictive_controller(Mem2);
	//predictive_controller(x_kk,y_ref_kk,u_kk,y_hat_src,r_hat_src,v_mul_H_inv_src,v_gen_src,h_hat_inv_src,out);
	predictive_controller(x_kk,y_ref_kk,u_kk,out);

for (int k=0;k<3*Nh ; k++){

	printf ("out[%d]=%.9f _ Answer[%d]=%.9f\n,",k,out[k],k,answer[k]);

}


//Compare results
for(i=0; i < 3*Nh; i++){
  if((out[i] - answer[i]) > 0.1f){
    printf("i = %d Mem = %9f Answer= %9f\n",i,out[i],answer[i]);
    printf("ERROR HW and SW results mismatch\n");
    return 1;
  }
}


  printf("Success HW and SW results match\n");
  return 0;
}

