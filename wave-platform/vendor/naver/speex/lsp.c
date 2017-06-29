
#include "config.h"
#include <math.h>
#include "lsp.h"
#include "stack_alloc.h"
#include "math_approx.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846  /* pi */
#endif

#ifndef NULL
#define NULL 0
#endif

#define FREQ_SCALE 16384

#define ANGLE2X(a) (SHL16(spx_cos(a),2))

#define X2ANGLE(x) (spx_acos(x))

#ifdef BFIN_ASM
#include "lsp_bfin.h"
#endif



/*---------------------------------------------------------------------------*\

  FUNCTION....: cheb_poly_eva()

  AUTHOR......: David Rowe
  DATE CREATED: 24/2/93

  This function evaluates a series of Chebyshev polynomials

  \*---------------------------------------------------------------------------*/

#ifndef OVERRIDE_CHEB_POLY_EVA
static inline spx_word32_t cheb_poly_eva(
		spx_word16_t *coef, /* P or Q coefs in Q13 format               */
		spx_word16_t     x, /* cos of freq (-1.0 to 1.0) in Q14 format  */
		int              m, /* LPC order/2                              */
		char         *stack
		)
{
	int i;
	spx_word16_t b0, b1;
	spx_word32_t sum;

	/*Prevents overflows*/
	if (x>16383)
		x = 16383;
	if (x<-16383)
		x = -16383;

	/* Initialise values */
	b1=16384;
	b0=x;

	/* Evaluate Chebyshev series formulation usin g iterative approach  */
	sum = ADD32(EXTEND32(coef[m]), EXTEND32(MULT16_16_P14(coef[m-1],x)));
	for(i=2;i<=m;i++)
	{
		spx_word16_t tmp=b0;
		b0 = SUB16(MULT16_16_Q13(x,b0), b1);
		b1 = tmp;
		sum = ADD32(sum, EXTEND32(MULT16_16_P14(coef[m-i],b0)));
	}

	return sum;
}
#endif


/*---------------------------------------------------------------------------*\

  FUNCTION....: lpc_to_lsp()

  AUTHOR......: David Rowe
  DATE CREATED: 24/2/93

  This function converts LPC coefficients to LSP
  coefficients.

  \*---------------------------------------------------------------------------*/

#define SIGN_CHANGE(a,b) (((a)&0x70000000)^((b)&0x70000000)||(b==0))


int lpc_to_lsp (spx_coef_t *a,int lpcrdr,spx_lsp_t *freq,int nb,spx_word16_t delta, char *stack)
	/*  float *a 		     	lpc coefficients			*/
	/*  int lpcrdr			order of LPC coefficients (10) 		*/
	/*  float *freq 	      	LSP frequencies in the x domain       	*/
	/*  int nb			number of sub-intervals (4) 		*/
	/*  float delta			grid spacing interval (0.02) 		*/


{
	spx_word16_t temp_xr,xl,xr,xm=0;
	spx_word32_t psuml,psumr,psumm,temp_psumr/*,temp_qsumr*/;
	int i,j,m,flag,k;
	VARDECL(spx_word32_t *Q);                 	/* ptrs for memory allocation 		*/
	VARDECL(spx_word32_t *P);
	VARDECL(spx_word16_t *Q16);         /* ptrs for memory allocation 		*/
	VARDECL(spx_word16_t *P16);
	spx_word32_t *px;                	/* ptrs of respective P'(z) & Q'(z)	*/
	spx_word32_t *qx;
	spx_word32_t *p;
	spx_word32_t *q;
	spx_word16_t *pt;                	/* ptr used for cheb_poly_eval()
										   whether P' or Q' 			*/
	int roots=0;              	/* DR 8/2/94: number of roots found 	*/
	flag = 1;                	/*  program is searching for a root when,
									1 else has found one 			*/
	m = lpcrdr/2;            	/* order of P'(z) & Q'(z) polynomials 	*/

	/* Allocate memory space for polynomials */
	ALLOC(Q, (m+1), spx_word32_t);
	ALLOC(P, (m+1), spx_word32_t);

	/* determine P'(z)'s and Q'(z)'s coefficients where
	   P'(z) = P(z)/(1 + z^(-1)) and Q'(z) = Q(z)/(1-z^(-1)) */

	px = P;                      /* initialise ptrs 			*/
	qx = Q;
	p = px;
	q = qx;

	*px++ = LPC_SCALING;
	*qx++ = LPC_SCALING;
	for(i=0;i<m;i++){
		*px++ = SUB32(ADD32(EXTEND32(a[i]),EXTEND32(a[lpcrdr-i-1])), *p++);
		*qx++ = ADD32(SUB32(EXTEND32(a[i]),EXTEND32(a[lpcrdr-i-1])), *q++);
	}
	px = P;
	qx = Q;
	for(i=0;i<m;i++)
	{
		/*if (fabs(*px)>=32768)
		  speex_warning_int("px", *px);
		  if (fabs(*qx)>=32768)
		  speex_warning_int("qx", *qx);*/
		*px = PSHR32(*px,2);
		*qx = PSHR32(*qx,2);
		px++;
		qx++;
	}
	/* The reason for this lies in the way cheb_poly_eva() is implemented for fixed-point */
	P[m] = PSHR32(P[m],3);
	Q[m] = PSHR32(Q[m],3);

	px = P;             	/* re-initialise ptrs 			*/
	qx = Q;

	/* now that we have computed P and Q convert to 16 bits to
	   speed up cheb_poly_eval */

	ALLOC(P16, m+1, spx_word16_t);
	ALLOC(Q16, m+1, spx_word16_t);

	for (i=0;i<m+1;i++)
	{
		P16[i] = P[i];
		Q16[i] = Q[i];
	}

	/* Search for a zero in P'(z) polynomial first and then alternate to Q'(z).
	   Keep alternating between the two polynomials as each zero is found 	*/

	xr = 0;             	/* initialise xr to zero 		*/
	xl = FREQ_SCALE;               	/* start at point xl = 1 		*/

	for(j=0;j<lpcrdr;j++){
		if(j&1)            	/* determines whether P' or Q' is eval. */
			pt = Q16;
		else
			pt = P16;

		psuml = cheb_poly_eva(pt,xl,m,stack);	/* evals poly. at xl 	*/
		flag = 1;
		while(flag && (xr >= -FREQ_SCALE)){
			spx_word16_t dd;
			/* Modified by JMV to provide smaller steps around x=+-1 */
			dd = MULT16_16_Q15(delta,SUB16(FREQ_SCALE, MULT16_16_Q14(MULT16_16_Q14(xl,xl),14000)));
			if (psuml<512 && psuml>-512)
				dd = PSHR16(dd,1);
			xr = SUB16(xl, dd);                        	/* interval spacing 	*/
			psumr = cheb_poly_eva(pt,xr,m,stack);/* poly(xl-delta_x) 	*/
			temp_psumr = psumr;
			temp_xr = xr;

			/* if no sign change increment xr and re-evaluate poly(xr). Repeat til
			   sign change.
			   if a sign change has occurred the interval is bisected and then
			   checked again for a sign change which determines in which
			   interval the zero lies in.
			   If there is no sign change between poly(xm) and poly(xl) set interval
			   between xm and xr else set interval between xl and xr and repeat till
			   root is located within the specified limits 			*/

			if(SIGN_CHANGE(psumr,psuml))
			{
				roots++;

				psumm=psuml;
				for(k=0;k<=nb;k++){
					xm = ADD16(PSHR16(xl,1),PSHR16(xr,1));        	/* bisect the interval 	*/
					psumm=cheb_poly_eva(pt,xm,m,stack);
					/*if(psumm*psuml>0.)*/
					if(!SIGN_CHANGE(psumm,psuml))
					{
						psuml=psumm;
						xl=xm;
					} else {
						psumr=psumm;
						xr=xm;
					}
				}

				/* once zero is found, reset initial interval to xr 	*/
				freq[j] = X2ANGLE(xm);
				xl = xm;
				flag = 0;       		/* reset flag for next search 	*/
			}
			else{
				psuml=temp_psumr;
				xl=temp_xr;
			}
		}
	}
	return(roots);
}

/*---------------------------------------------------------------------------*\

  FUNCTION....: lsp_to_lpc()

  AUTHOR......: David Rowe
  DATE CREATED: 24/2/93

  Converts LSP coefficients to LPC coefficients.

  \*---------------------------------------------------------------------------*/

void lsp_to_lpc(spx_lsp_t *freq,spx_coef_t *ak,int lpcrdr, char *stack)
	/*  float *freq 	array of LSP frequencies in the x domain	*/
	/*  float *ak 		array of LPC coefficients 			*/
	/*  int lpcrdr  	order of LPC coefficients 			*/
{
	int i,j;
	spx_word32_t xout1,xout2,xin;
	spx_word32_t mult, a;
	VARDECL(spx_word16_t *freqn);
	VARDECL(spx_word32_t **xp);
	VARDECL(spx_word32_t *xpmem);
	VARDECL(spx_word32_t **xq);
	VARDECL(spx_word32_t *xqmem);
	int m = lpcrdr>>1;

	/* 

	   Reconstruct P(z) and Q(z) by cascading second order polynomials
	   in form 1 - 2cos(w)z(-1) + z(-2), where w is the LSP frequency.
	   In the time domain this is:

	   y(n) = x(n) - 2cos(w)x(n-1) + x(n-2)

	   This is what the ALLOCS below are trying to do:

	   int xp[m+1][lpcrdr+1+2]; // P matrix in QIMP
	   int xq[m+1][lpcrdr+1+2]; // Q matrix in QIMP

	   These matrices store the output of each stage on each row.  The
	   final (m-th) row has the output of the final (m-th) cascaded
	   2nd order filter.  The first row is the impulse input to the
	   system (not written as it is known).

	   The version below takes advantage of the fact that a lot of the
	   outputs are zero or known, for example if we put an inpulse
	   into the first section the "clock" it 10 times only the first 3
	   outputs samples are non-zero (it's an FIR filter).
	 */

	ALLOC(xp, (m+1), spx_word32_t*);
	ALLOC(xpmem, (m+1)*(lpcrdr+1+2), spx_word32_t);

	ALLOC(xq, (m+1), spx_word32_t*);
	ALLOC(xqmem, (m+1)*(lpcrdr+1+2), spx_word32_t);

	for(i=0; i<=m; i++) {
		xp[i] = xpmem + i*(lpcrdr+1+2);
		xq[i] = xqmem + i*(lpcrdr+1+2);
	}

	/* work out 2cos terms in Q14 */

	ALLOC(freqn, lpcrdr, spx_word16_t);
	for (i=0;i<lpcrdr;i++) 
		freqn[i] = ANGLE2X(freq[i]);

#define QIMP  21   /* scaling for impulse */

	xin = SHL32(EXTEND32(1), (QIMP-1)); /* 0.5 in QIMP format */

	/* first col and last non-zero values of each row are trivial */

	for(i=0;i<=m;i++) {
		xp[i][1] = 0;
		xp[i][2] = xin;
		xp[i][2+2*i] = xin;
		xq[i][1] = 0;
		xq[i][2] = xin;
		xq[i][2+2*i] = xin;
	}

	/* 2nd row (first output row) is trivial */

	xp[1][3] = -MULT16_32_Q14(freqn[0],xp[0][2]);
	xq[1][3] = -MULT16_32_Q14(freqn[1],xq[0][2]);

	xout1 = xout2 = 0;

	/* now generate remaining rows */

	for(i=1;i<m;i++) {

		for(j=1;j<2*(i+1)-1;j++) {
			mult = MULT16_32_Q14(freqn[2*i],xp[i][j+1]);
			xp[i+1][j+2] = ADD32(SUB32(xp[i][j+2], mult), xp[i][j]);
			mult = MULT16_32_Q14(freqn[2*i+1],xq[i][j+1]);
			xq[i+1][j+2] = ADD32(SUB32(xq[i][j+2], mult), xq[i][j]);
		}

		/* for last col xp[i][j+2] = xq[i][j+2] = 0 */

		mult = MULT16_32_Q14(freqn[2*i],xp[i][j+1]);
		xp[i+1][j+2] = SUB32(xp[i][j], mult);
		mult = MULT16_32_Q14(freqn[2*i+1],xq[i][j+1]);
		xq[i+1][j+2] = SUB32(xq[i][j], mult);
	}

	/* process last row to extra a{k} */

	for(j=1;j<=lpcrdr;j++) {
		int shift = QIMP-13;

		/* final filter sections */
		a = PSHR32(xp[m][j+2] + xout1 + xq[m][j+2] - xout2, shift); 
		xout1 = xp[m][j+2];
		xout2 = xq[m][j+2];

		/* hard limit ak's to +/- 32767 */

		if (a < -32767) a = -32767;
		if (a > 32767) a = 32767;
		ak[j-1] = (short)a;

	}

}


/*Makes sure the LSPs are stable*/
void lsp_enforce_margin(spx_lsp_t *lsp, int len, spx_word16_t margin)
{
	int i;
	spx_word16_t m = margin;
	spx_word16_t m2 = 25736-margin;

	if (lsp[0]<m)
		lsp[0]=m;
	if (lsp[len-1]>m2)
		lsp[len-1]=m2;
	for (i=1;i<len-1;i++)
	{
		if (lsp[i]<lsp[i-1]+m)
			lsp[i]=lsp[i-1]+m;

		if (lsp[i]>lsp[i+1]-m)
			lsp[i]= SHR16(lsp[i],1) + SHR16(lsp[i+1]-m,1);
	}
}


void lsp_interpolate(spx_lsp_t *old_lsp, spx_lsp_t *new_lsp, spx_lsp_t *interp_lsp, int len, int subframe, int nb_subframes)
{
	int i;
	spx_word16_t tmp = DIV32_16(SHL32(EXTEND32(1 + subframe),14),nb_subframes);
	spx_word16_t tmp2 = 16384-tmp;
	for (i=0;i<len;i++)
	{
		interp_lsp[i] = MULT16_16_P14(tmp2,old_lsp[i]) + MULT16_16_P14(tmp,new_lsp[i]);
	}
}

