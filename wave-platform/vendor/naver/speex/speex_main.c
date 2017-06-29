#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "speex/speex_callbacks.h"
#include "speex/speex.h"
#include "speex_main.h"

#define MAX_NBYTES  200


void speex_encode_variable_init( Speex_t *spx )
{
	spx_int32_t tmp;

	spx->st =  speex_encoder_init(speex_lib_get_mode(SPEEX_MODEID_NB));

	tmp = 0;
	speex_encoder_ctl(spx->st, SPEEX_SET_VBR, &tmp);
	tmp=0;
	speex_encoder_ctl(spx->st, SPEEX_SET_VAD, &tmp);
	tmp=1;
	speex_encoder_ctl(spx->st, SPEEX_SET_QUALITY, &tmp);

	//tmp=5;
   	//speex_encoder_ctl(spx->st, SPEEX_SET_COMPLEXITY, &tmp);

	speex_bits_init(&(spx->bits));


}

void speex_encode_variable_destroy( Speex_t *spx_en )
{
	speex_encoder_destroy( spx_en->st );
	speex_bits_destroy( &(spx_en->bits));
}

void speex_decode_variable_destroy( Speex_t *spx_de )
{
	speex_decoder_destroy( spx_de->st );
	speex_bits_destroy( &(spx_de->bits));
}

void spx_variable_transfer( Speex_t *spx_en, Speex_t *spx_de )
{

	SpeexBits *bits_en;
	SpeexBits *bits_de;

	bits_en = &(spx_en->bits); 
	bits_de = &(spx_de->bits); 

	bits_de->nbBits = bits_en->nbBits;


	memcpy( bits_de->chars, spx_en->cbits, spx_en->nbBits);

}


void speex_decode_variable_init( Speex_t *spx )
{
	spx_int32_t tmp;

	spx->st =  speex_decoder_init(speex_lib_get_mode(SPEEX_MODEID_NB));

	tmp = 1; 
	speex_decoder_ctl(spx->st, SPEEX_SET_ENH, &tmp);
	speex_bits_init(&(spx->bits));
}


int spx_encode( short *in_short, Speex_t *spx )
{
	speex_bits_reset( &(spx->bits));
	speex_encode_int( spx->st, in_short, &(spx->bits));
	spx->nbBits = speex_bits_write( &(spx->bits), spx->cbits, MAX_NBYTES);

	return 0;

}

int spx_decode( Speex_t *spx , short *out_short)
{
	speex_bits_rewind(&(spx->bits));
	speex_decode_int(spx->st, &(spx->bits), out_short);

	return 0;
}
