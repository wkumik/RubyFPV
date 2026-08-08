// Functional test for code/radio/fec.c: encode, erase, decode, verify.
// On ARM builds this also exercises the NEON GF kernels (enabled by the
// self test in fec_init) against the scalar reference implementation.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../radio/fec.h"

#define BLOCK_SIZE 1109  // odd size to exercise vector tails
#define K 8
#define M 4

static unsigned int s_uSeed = 0xC0FFEE42;
static unsigned char rnd_byte(void)
{
   s_uSeed = s_uSeed * 1103515245 + 12345;
   return (unsigned char)(s_uSeed >> 16);
}

int main(void)
{
   unsigned char* pData[K];
   unsigned char* pOrig[K];
   unsigned char* pFec[M];
   int i, j;

   fec_init();

   for( i=0; i<K; i++ )
   {
      pData[i] = malloc(BLOCK_SIZE);
      pOrig[i] = malloc(BLOCK_SIZE);
      for( j=0; j<BLOCK_SIZE; j++ )
         pData[i][j] = rnd_byte();
      memcpy(pOrig[i], pData[i], BLOCK_SIZE);
   }
   for( i=0; i<M; i++ )
   {
      pFec[i] = malloc(BLOCK_SIZE);
      memset(pFec[i], 0, BLOCK_SIZE);
   }

   fec_encode(BLOCK_SIZE, pData, K, pFec, M);

   // Test all erasure counts from 1 to M
   int iTotalFailures = 0;
   for( int iErasures=1; iErasures<=M; iErasures++ )
   {
      // Restore data, then erase blocks spread through the set
      unsigned int erased[M];
      unsigned int fecNos[M];
      unsigned char* pFecUsed[M];
      for( i=0; i<K; i++ )
         memcpy(pData[i], pOrig[i], BLOCK_SIZE);
      for( i=0; i<iErasures; i++ )
      {
         erased[i] = (unsigned int)((i * K) / iErasures);  // strictly increasing
         memset(pData[erased[i]], 0xAA, BLOCK_SIZE);
         fecNos[i] = (unsigned int)i;
         pFecUsed[i] = malloc(BLOCK_SIZE);
         memcpy(pFecUsed[i], pFec[i], BLOCK_SIZE); // decode modifies fec blocks
      }

      int iRes = fec_decode(BLOCK_SIZE, pData, K, pFecUsed, fecNos, erased, (unsigned short)iErasures);

      int iFailures = 0;
      for( i=0; i<K; i++ )
         if ( 0 != memcmp(pData[i], pOrig[i], BLOCK_SIZE) )
            iFailures++;
      printf("erasures=%d: decode result=%d, mismatched blocks=%d -> %s\n",
         iErasures, iRes, iFailures, (iFailures==0 && iRes==0) ? "PASS" : "FAIL");
      if ( iFailures || iRes )
         iTotalFailures++;
      for( i=0; i<iErasures; i++ )
         free(pFecUsed[i]);
   }

   return iTotalFailures ? 1 : 0;
}
