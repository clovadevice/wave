#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "speex/speex_callbacks.h"
#include "speex/speex.h"

#ifdef FIXED_DEBUG
extern long long spx_mips;
#endif

#define FRAME_SIZE 320
#define MAX_NBYTES	200

int main(int argc, char **argv)
{
	char *inFile, *bitsFile;
	FILE *fin, *fbits=NULL;
	short in_short[FRAME_SIZE];
	char cbits[200];
	int nbBits;
	void *st;
	SpeexBits bits;
	spx_int32_t tmp;
	int bitCount=0;
#ifdef FIXED_DEBUG
	int snr_frames;
#endif

	if ( argc != 3)
	{
		fprintf (stderr, "Usage: encode [in file] [bits file]\nargc = %d", argc);
		exit(1);
	}

	st = speex_encoder_init(speex_lib_get_mode(SPEEX_MODEID_WB));

	tmp=1;
	if( tmp ){
	   	speex_encoder_ctl(st, SPEEX_SET_VBR, &tmp);
	   	tmp=10;
	   	speex_encoder_ctl(st, SPEEX_SET_VBR_QUALITY, &tmp);
	}
	else{
	   	speex_encoder_ctl(st, SPEEX_SET_VBR, &tmp);
	   	tmp=1;
	   	speex_encoder_ctl(st, SPEEX_SET_VAD, &tmp);
	   	tmp=10;
	   	speex_encoder_ctl(st, SPEEX_SET_QUALITY, &tmp);
	}
	tmp=5;
	speex_encoder_ctl(st, SPEEX_SET_COMPLEXITY, &tmp);

	inFile = argv[1];
	fin = fopen(inFile, "rb");
	bitsFile = argv[2];
	fbits = fopen(bitsFile, "wb");

	speex_bits_init(&bits);
#ifdef FIXED_DEBUG
	snr_frames = 0;
#endif
	while ( fread(in_short, sizeof(short), FRAME_SIZE, fin) == FRAME_SIZE ){
		speex_bits_reset(&bits);
		speex_encode_int(st, in_short, &bits);
		nbBits = speex_bits_write(&bits, cbits, MAX_NBYTES);
		bitCount+=bits.nbBits;

		printf("%d %d\n", bits.nbBits, nbBits);

		fwrite(&nbBits, sizeof(int), 1, fbits);
		fwrite(&(bits.nbBits), sizeof(int), 1, fbits);
		fwrite(cbits, 1, nbBits, fbits);


#ifdef FIXED_DEBUG
		snr_frames++;
#endif
	}
	fprintf (stderr, "Total encoded size: %d bits\n", bitCount);
	speex_encoder_destroy(st);
	speex_bits_destroy(&bits);

	fclose(fin);
	fclose(fbits);

#ifdef FIXED_DEBUG
   	printf ("Total: %f MIPS\n", (float)(1e-6*50*spx_mips/snr_frames));
#endif


	return 1;
}
