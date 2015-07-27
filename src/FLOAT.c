// #define  TEST_PRINTF    1

#ifdef TEST_PRINTF
#include <stdio.h>
#include <string.h>

#undef  USE_INTERNALS
#else
//  USE_INTERNALS implements code for company-specific linkages
//  at the company that I currently work for.
//  Other users will *not* want this defined!!
// #define  USE_INTERNALS
#endif

#ifdef   USE_INTERNALS
#include <stdio.h>
#include <string.h>

#include <remap.h>
#include <91x_lib.h>

#include "arm_lib.h"
#include "depends.h"

#else

typedef  unsigned char  u8 ;
typedef  unsigned int   uint ;
#endif

static uint use_leading_plus = 0 ;

static const double round_nums[8] = {
   0.5,
   0.05,
   0.005,
   0.0005,
   0.00005,
   0.000005,
   0.0000005,
   0.00000005
} ;

void _dbl2stri(char *outbfr, double dbl, unsigned dec_digits)
{
   static char local_bfr[128] ;
   char *output = (outbfr == 0) ? local_bfr : outbfr ;
   uint mult = 1 ;
   uint idx ;
	 uint wholeNum;
   uint decimalNum;
	 char tbfr[40] ;
   //*******************************************
   //  extract negative info
   //*******************************************
   if (dbl < 0.0) {
      *output++ = '-' ;
      dbl *= -1.0 ;
   } else {
      if (use_leading_plus) {
         *output++ = '+' ;
      }
      
   }

   //  handling rounding by adding .5LSB to the floating-point data
   if (dec_digits < 8) {
      dbl += round_nums[dec_digits] ;
   }

   //**************************************************************************
   //  construct fractional multiplier for specified number of digits.
   //**************************************************************************

   for (idx=0; idx < dec_digits; idx++)
      mult *= 10 ;

   // printf("mult=%u\n", mult) ;
   wholeNum = (uint) dbl ;
   decimalNum = (uint) ((dbl - wholeNum) * mult);

   //*******************************************
   //  convert integer portion
   //*******************************************
   idx = 0 ;
   while (wholeNum != 0) {
      tbfr[idx++] = '0' + (wholeNum % 10) ;
      wholeNum /= 10 ;
   }
   // printf("%.3f: whole=%s, dec=%d\n", dbl, tbfr, decimalNum) ;
   if (idx == 0) {
      *output++ = '0' ;
   } else {
      while (idx > 0) {
         *output++ = tbfr[idx-1] ;  //lint !e771
         idx-- ;
      }
   }
   if (dec_digits > 0) {
      *output++ = '.' ;

      //*******************************************
      //  convert fractional portion
      //*******************************************
      idx = 0 ;
      while (decimalNum != 0) {
         tbfr[idx++] = '0' + (decimalNum % 10) ;
         decimalNum /= 10 ;
      }
      //  pad the decimal portion with 0s as necessary;
      //  We wouldn't want to report 3.093 as 3.93, would we??
      while (idx < dec_digits) {
         tbfr[idx++] = '0' ;
      }
      // printf("decimal=%s\n", tbfr) ;
      if (idx == 0) {
         *output++ = '0' ;
      } else {
         while (idx > 0) {
            *output++ = tbfr[idx-1] ;
            idx-- ;
         }
      }
   }
   *output = 0 ;

   //  prepare output
   output = (outbfr == 0) ? local_bfr : outbfr ;
}

int _decimal_binary(int n)  
{
    int rem, i=1, binary=0;
	
    while (n!=0)
    {
        rem=n%2;
        n/=2;
        binary+=rem*i;
        i*=10;
    }
  
		return binary;
}
