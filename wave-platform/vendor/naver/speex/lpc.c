#include "config.h"
#include "lpc.h"
#ifdef BFIN_ASM
#include "lpc_bfin.h"
#endif

/* LPC analysis
 *
 * The next two functions calculate linear prediction coefficients
 * and/or the related reflection coefficients from the first P_MAX+1
 * values of the autocorrelation function.
 */

/* Invented by N. Levinson in 1947, modified by J. Durbin in 1959.
 */

/* returns minimum mean square error    */
spx_word32_t _spx_lpc(
spx_coef_t       *lpc, /* out: [0...p-1] LPC coefficients      */
const spx_word16_t *ac,  /* in:  [0...p] autocorrelation values  */
int          p
)
{
   int i, j;  
   spx_word16_t r;
   spx_word16_t error = ac[0];

   if (ac[0] == 0)
   {
      for (i = 0; i < p; i++)
         lpc[i] = 0;
      return 0;
   }

   for (i = 0; i < p; i++) {

      /* Sum up this iteration's reflection coefficient */
      spx_word32_t rr = NEG32(SHL32(EXTEND32(ac[i + 1]),13));
      for (j = 0; j < i; j++) 
         rr = SUB32(rr,MULT16_16(lpc[j],ac[i - j]));
      r = DIV32_16(rr+PSHR32(error,1),ADD16(error,8));
      /*  Update LPC coefficients and total error */
      lpc[i] = r;
      for (j = 0; j < i>>1; j++) 
      {
         spx_word16_t tmp  = lpc[j];
         lpc[j]     = MAC16_16_P13(lpc[j],r,lpc[i-1-j]);
         lpc[i-1-j] = MAC16_16_P13(lpc[i-1-j],r,tmp);
      }
      if (i & 1) 
         lpc[j] = MAC16_16_P13(lpc[j],lpc[j],r);

      error = SUB16(error,MULT16_16_Q13(r,MULT16_16_Q13(error,r)));
   }
   return error;
}


/* Compute the autocorrelation
 *                      ,--,
 *              ac(i) = >  x(n) * x(n-i)  for all n
 *                      `--'
 * for lags between 0 and lag-1, and x == 0 outside 0...n-1
 */

#ifndef OVERRIDE_SPEEX_AUTOCORR
void _spx_autocorr(
const spx_word16_t *x,   /*  in: [0...n-1] samples x   */
spx_word16_t       *ac,  /* out: [0...lag-1] ac values */
int          lag, 
int          n
)
{
   spx_word32_t d;
   int i, j;
   spx_word32_t ac0=1;
   int shift, ac_shift;
   
   for (j=0;j<n;j++)
      ac0 = ADD32(ac0,SHR32(MULT16_16(x[j],x[j]),8));
   ac0 = ADD32(ac0,n);
   shift = 8;
   while (shift && ac0<0x40000000)
   {
      shift--;
      ac0 <<= 1;
   }
   ac_shift = 18;
   while (ac_shift && ac0<0x40000000)
   {
      ac_shift--;
      ac0 <<= 1;
   }
   
   
   for (i=0;i<lag;i++)
   {
      d=0;
      for (j=i;j<n;j++)
      {
         d = ADD32(d,SHR32(MULT16_16(x[j],x[j-i]), shift));
      }
      
      ac[i] = SHR32(d, ac_shift);
   }
}
#endif




