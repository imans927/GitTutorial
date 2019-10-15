#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#ifndef Pred_controller_H_
#define Pred_controller_H_

#define Nh 4
//#define Nhh 5
#define labda_u 6.8e-3

// offset is the location of first element
// First Three positions are reserved for calcualted switch position values
#define x_kk_offset 3*Nh //3*Nh
#define x_kk_row 4


#define y_ref_kk_offset   x_kk_offset+x_kk_row //4+3*Nh
#define y_ref_kk_row   2*Nh


#define u_kk_offset y_ref_kk_offset+y_ref_kk_row // 4+5*Nh
#define u_kk_row  3*Nh


#define y_hat_offset  u_kk_offset+u_kk_row   // 4+8*Nh
#define y_hat_row 2*Nh
#define y_hat_col 3*Nh


#define r_hat_offset y_hat_offset + y_hat_row*y_hat_col // 4+ 8*Nh + 6Nh*Nh
#define r_hat_row 2*Nh
#define r_hat_col 4


#define v_mul_h_inv_offset r_hat_offset+r_hat_col*r_hat_row //4+16*Nh+ 6Nh*Nh
#define v_mul_h_inv_row 3*Nh
#define v_mul_h_inv_col 3*Nh

#define v_gen_offset v_mul_h_inv_offset+v_mul_h_inv_row*v_mul_h_inv_col  // 4+16*Nh+ 15Nh*Nh
#define v_gen_row 3*Nh
#define v_gen_col 3*Nh

#define h_hat_inv_offset v_gen_offset+v_gen_row*v_gen_col  // 4+16*Nh+24Nh*Nh
#define h_hat_inv_row 3*Nh
#define h_hat_inv_col 3*Nh

// Total length =4+16*Nh*33Nh*Nh

#define SIZE 4+16*Nh+33*Nh*Nh




/*
#define X_KK(p) 			(buff[x_kk_offset + p])
#define Y_Ref_KK(p) 		(buff[y_ref_kk_offset + p])
#define U_KK(p)				(buff[u_kk_offset + p])
#define Y_Hat(p,q)			(buff[y_hat_offset + p*y_hat_col +q])
#define R_Hat(p,q)			(buff[r_hat_offset + p*r_hat_col +q])
#define V_Mul_H_Inv(p,q)	(buff[v_mul_h_inv_offset + p*v_mul_h_inv_col +q])
#define V_Gen(p,q)			(buff[v_gen_offset + p*v_gen_col +q])
#define H_Hat_Inv(p,q)		(buff[h_hat_inv_offset + p*h_hat_inv_col +q])
*/



#define X_KK(p) 			(X_KK_a[p])
#define Y_Ref_KK(p) 		(Y_Ref_KK_a[p])
#define U_KK(p)				(U_KK_a[p])
#define Y_Hat(p,q)			(Y_Hat_a[p*y_hat_col +q])
#define R_Hat(p,q)			(R_Hat_a[p*r_hat_col +q])
#define V_Mul_H_Inv(p,q)	(V_Mul_H_Inv_a[p*v_mul_h_inv_col +q])
#define V_Gen(p,q)			(V_Gen_a[p*v_gen_col +q])
#define V_Gen_cpy(p,q)		(V_Gen_a_cpy[p*v_gen_col +q])
#define H_Hat_Inv(p,q)		(H_Hat_Inv_a[p*h_hat_inv_col +q])


/*
void predictive_controller(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *Y_HAT_src,
							volatile float *R_HAT_src,volatile float *V_MUL_H_INV_src,
							volatile float *V_GEN_src,volatile float *H_HAT_INV_src,
							volatile float *out);

*/
void predictive_controller(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *out);


//void unconstrained (float *buff,float unconstrained[3*Nh],float theta_kk[3*Nh],float v_gen_copy[3*Nh][3*Nh],float U_unc_kk_copy[3*Nh]);
//void unconstrained (float *buff,float unconstrained[3*Nh],float theta_kk[3*Nh]);
//float guess_edu (float *buff,float U_unc_kk[3*Nh]);
//float guess_edu (float u_kk[3*Nh],float v_gen[3*Nh][3*Nh],float U_unc_kk[3*Nh]);
//float guess_babay (float *buff,float U_unc_kk[3*Nh],float theta_kk[3*Nh]);
//void sph_dec (float *buff,float roh, float U_unc[3*Nh], float U_opt[3*Nh]);
//void offline(float x_kk[4],float y_ref_kk[2*Nh], float u_kk[3*Nh],float y_hat[2*Nh][3*Nh] , float r_hat[2*Nh][4] , float v_mul_h_inv[3*Nh][3*Nh], float v_gen[3*Nh][3*Nh],float h_hat_inv[3*Nh][3*Nh]);

void unconstrained (float R_Hat_a[8*Nh],float X_KK_a[4],float Y_Hat_a[6*Nh*Nh],float Y_Ref_KK_a[2*Nh],float U_KK_a[3*Nh],float V_Mul_H_Inv_a[9*Nh*Nh],float unconstrained[3*Nh],float theta_kk[3*Nh]);
float guess_edu (float U_KK_a[3*Nh],float V_Gen_a[9*Nh*Nh],float U_unc_kk[3*Nh]);
float guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh]);
//void guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh],float *roh_try);

void sph_dec (float V_Gen_a[9*Nh*Nh],float roh, float U_unc[3*Nh], float U_opt[3*Nh]);
void offline(float x_kk[4],float y_ref_kk[2*Nh], float u_kk[3*Nh],float y_hat[2*Nh][3*Nh] , float r_hat[2*Nh][4] , float v_mul_h_inv[3*Nh][3*Nh], float v_gen[3*Nh][3*Nh],float h_hat_inv[3*Nh][3*Nh]);

//void uncons_copy(float unconstrained[3*Nh],float unconstrained_cpy[3*Nh]);
//void u_kk_copy_fun(float *buff,float u_kk_copy[3*Nh]);
//void v_gen_copy_fun(float *buff,float v_gen_copy[3*Nh][3*Nh]);


#endif
