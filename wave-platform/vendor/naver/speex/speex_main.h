#ifndef __SPEEX_MAIN_H__
#define __SPEEX_MAIN_H__

#include "speex/speex.h"

typedef struct tag_Speex_t {

	void *st;
	SpeexBits bits;
	int	nbBits;
	char	cbits[200];


}Speex_t;

extern int spx_decode( Speex_t *spx , short *out_short);
extern int spx_encode( short *in_short, Speex_t *spx );
extern void speex_decode_variable_init( Speex_t *spx );
extern void speex_encode_variable_init( Speex_t *spx );
extern void spx_variable_transfer( Speex_t *spx_en, Speex_t *spx_de );
extern void speex_encode_variable_destroy( Speex_t *spx_en );
extern void speex_decode_variable_destroy( Speex_t *spx_de );

#endif
