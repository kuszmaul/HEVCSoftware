/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2012, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComTrQuant.cpp
    \brief    transform and quantization class
*/

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "TComTrQuant.h"
#include "TComPic.h"
#include "ContextTables.h"
#include "TComTU.h"
#include "Debug.h"

typedef struct
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
} coeffGroupRDStats;

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define RDOQ_CHROMA                 1           ///< use of RDOQ in chroma

// ====================================================================================================================
// TComTrQuant class member functions
// ====================================================================================================================

TComTrQuant::TComTrQuant()
{
  // allocate temporary buffers
  m_plTempCoeff  = new TCoeff[ MAX_CU_SIZE*MAX_CU_SIZE ];
  
  // allocate bit estimation class  (for RDOQ)
  m_pcEstBitsSbac = new estBitsSbacStruct;
  initScalingList();
}

TComTrQuant::~TComTrQuant()
{
  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    delete [] m_plTempCoeff;
    m_plTempCoeff = NULL;
  }
  
  // delete bit estimation class
  if ( m_pcEstBitsSbac )
  {
    delete m_pcEstBitsSbac;
  }
  destroyScalingList();
}

#if ADAPTIVE_QP_SELECTION
Void TComTrQuant::storeSliceQpNext(TComSlice* pcSlice)
{
  Int qpBase = pcSlice->getSliceQpBase();
  Int sliceQpused = pcSlice->getSliceQp();
  Int sliceQpnext;
  Double alpha = qpBase < 17 ? 0.5 : 1;
  
  Int cnt=0;
  for(int u=1; u<=LEVEL_RANGE; u++)
  { 
    cnt += m_sliceNsamples[u] ;
  }

  if( !m_bUseRDOQ )
  {
    sliceQpused = qpBase;
    alpha = 0.5;
  }

  if( cnt > 120 )
  {
    Double sum = 0;
    Int k = 0;
    for(Int u=1; u<LEVEL_RANGE; u++)
    {
      sum += u*m_sliceSumC[u];
      k += u*u*m_sliceNsamples[u];
    }

    Int v;
    Double q[MAX_QP+1] ;
    for(v=0; v<=MAX_QP; v++)
    {
      q[v] = (Double)(g_invQuantScales[v%6] * (1<<(v/6)))/64 ;
    }

    Double qnext = sum/k * q[sliceQpused] / (1<<ARL_C_PRECISION);

    for(v=0; v<MAX_QP; v++)
    {
      if(qnext < alpha * q[v] + (1 - alpha) * q[v+1] )
      {
        break;
      }
    }
    sliceQpnext = Clip3(sliceQpused - 3, sliceQpused + 3, v);
  }
  else
  {
    sliceQpnext = sliceQpused;
  }

  m_qpDelta[qpBase] = sliceQpnext - qpBase; 
}

Void TComTrQuant::initSliceQpDelta()
{
  for(Int qp=0; qp<=MAX_QP; qp++)
  {
    m_qpDelta[qp] = qp < 17 ? 0 : 1;
  }
}

Void TComTrQuant::clearSliceARLCnt()
{ 
  memset(m_sliceSumC, 0, sizeof(Double)*(LEVEL_RANGE+1));
  memset(m_sliceNsamples, 0, sizeof(Int)*(LEVEL_RANGE+1));
}
#endif



#if MATRIX_MULT
/** NxN forward transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param block pointer to input data (residual)
 *  \param coeff pointer to output data (transform coefficients)
 *  \param uiStride stride of input data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
void xTr(Pel *block, Int *coeff, UInt uiStride, UInt uiTrSize, UInt uiMode)
{
  Int i,j,k,iSum;
  Int tmp[32*32];
  const TMatrixCoeff *iT;
  UInt uiLog2TrSize = g_aucConvertToBit[ uiTrSize ] + 2;

  if (uiTrSize==4)
  {
    iT  = g_aiT4[0];
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[0];
  }
  else
  {
    assert(0);
  }

#if FULL_NBIT
  int shift_1st = uiLog2TrSize - 1 + g_uiBitDepth - 8; // log2(N) - 1 + g_uiBitDepth - 8
#else
  int shift_1st = uiLog2TrSize - 1 + g_uiBitIncrement; // log2(N) - 1 + g_uiBitIncrement
#endif

  int add_1st = 1<<(shift_1st-1);
  int shift_2nd = uiLog2TrSize + 6;
  int add_2nd = 1<<(shift_2nd-1);

  /* Horizontal transform */

  if (uiTrSize==4)
  {
    if ((uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 2 && uiMode <= 25)))
    {
      iT  =  g_as_DST_MAT_4[0];
    }
  }
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*block[j*uiStride+k];
      }
      tmp[i*uiTrSize+j] = (iSum + add_1st)>>shift_1st;
    }
  }
  
  /* Vertical transform */
  if (uiTrSize==4)
  {
    if ((uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 11 && uiMode <= 34)))
    {
      iT  =  g_as_DST_MAT_4[0];
    }
    else
    {
      iT  = g_aiT4[0];
    }
  }
  for (i=0; i<uiTrSize; i++)
  {                 
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*tmp[j*uiTrSize+k];        
      }
      coeff[i*uiTrSize+j] = (iSum + add_2nd)>>shift_2nd; 
    }
  }
}

/** NxN inverse transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param coeff pointer to input data (transform coefficients)
 *  \param block pointer to output data (residual)
 *  \param uiStride stride of output data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
void xITr(Int *coeff, Pel *block, UInt uiStride, UInt uiTrSize, UInt uiMode)
{
  int i,j,k,iSum;
  Int tmp[32*32];
  const TMatrixCoeff *iT;
  
  if (uiTrSize==4)
  {
    iT  = g_aiT4[0];
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[0];
  }
  else
  {
    assert(0);
  }
  
  int shift_1st = SHIFT_INV_1ST;
  int add_1st = 1<<(shift_1st-1);  
#if FULL_NBIT
  int shift_2nd = SHIFT_INV_2ND - (g_uiBitDepth - 8);
#else
  int shift_2nd = SHIFT_INV_2ND - g_uiBitIncrement;
#endif
  int add_2nd = 1<<(shift_2nd-1);
  if (uiTrSize==4)
  {
    if ((uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 11 && uiMode <= 34))) // Check for DCT or DST
    {
      iT  =  g_as_DST_MAT_4[0];
    }
  }
  
  /* Horizontal transform */
  for (i=0; i<uiTrSize; i++)
  {    
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {        
        iSum += iT[k*uiTrSize+i]*coeff[k*uiTrSize+j]; 
      }
      tmp[i*uiTrSize+j] = Clip3(TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (iSum + add_1st)>>shift_1st); // Clipping is normative
    }
  }   
  
  if (uiTrSize==4)
  {
    if ((uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 2 && uiMode <= 25)))   // Check for DCT or DST
    {
      iT  =  g_as_DST_MAT_4[0];
    }
    else  
    {
      iT  = g_aiT4[0];
    }
  }
  
  /* Vertical transform */
  for (i=0; i<uiTrSize; i++)
  {   
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {        
        iSum += iT[k*uiTrSize+j]*tmp[i*uiTrSize+k];
      }
      block[i*uiStride+j] = Clip3(TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (iSum + add_2nd)>>shift_2nd); // Clipping is non-normative
    }
  }
}

#endif //MATRIX_MULT


/** 4x4 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly4(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j;  
  int E[2],O[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {    
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0]      = (g_aiT4[0][0]*E[0] + g_aiT4[0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[2][0]*E[0] + g_aiT4[2][1]*E[1] + add)>>shift;
    dst[line]   = (g_aiT4[1][0]*O[0] + g_aiT4[1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[3][0]*O[0] + g_aiT4[3][1]*O[1] + add)>>shift;

    src += 4;
    dst ++;
  }
}

// Fast DST Algorithm. Full matrix multiplication for DST and Fast DST algorithm 
// give identical results
void fastForwardDst(TCoeff *block, TCoeff *coeff, int shift)  // input block, output coeff
{
  int i, c[4];
  int rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = block[4*i+0] + block[4*i+3];
    c[1] = block[4*i+1] + block[4*i+3];
    c[2] = block[4*i+0] - block[4*i+1];
    c[3] = 74* block[4*i+2];

    coeff[   i] =  ( 29 * c[0] + 55 * c[1]         + c[3]               + rnd_factor ) >> shift;
    coeff[ 4+i] =  ( 74 * (block[4*i+0]+ block[4*i+1] - block[4*i+3])   + rnd_factor ) >> shift;
    coeff[ 8+i] =  ( 29 * c[2] + 55 * c[0]         - c[3]               + rnd_factor ) >> shift;
    coeff[12+i] =  ( 55 * c[2] - 29 * c[1]         + c[3]               + rnd_factor ) >> shift;
  }
}

void fastInverseDst(TCoeff *tmp, TCoeff *block, int shift)  // input tmp, output block
{
  int i, c[4];
  int rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {  
    // Intermediate Variables
    c[0] = tmp[  i] + tmp[ 8+i];
    c[1] = tmp[8+i] + tmp[12+i];
    c[2] = tmp[  i] - tmp[12+i];
    c[3] = 74* tmp[4+i];

    block[4*i+0] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, ( 29 * c[0] + 55 * c[1]     + c[3]      + rnd_factor ) >> shift );
    block[4*i+1] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, ( 55 * c[2] - 29 * c[1]     + c[3]      + rnd_factor ) >> shift );
    block[4*i+2] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, ( 74 * (tmp[i] - tmp[8+i]  + tmp[12+i]) + rnd_factor ) >> shift );
    block[4*i+3] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, ( 55 * c[0] + 29 * c[2]     - c[3]      + rnd_factor ) >> shift );
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse4(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j;    
  int E[2],O[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */    
    O[0] = g_aiT4[1][0]*src[line] + g_aiT4[3][0]*src[3*line];
    O[1] = g_aiT4[1][1]*src[line] + g_aiT4[3][1]*src[3*line];
    E[0] = g_aiT4[0][0]*src[0] + g_aiT4[2][0]*src[2*line];
    E[1] = g_aiT4[0][1]*src[0] + g_aiT4[2][1]*src[2*line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[0] + O[0] + add)>>shift );
    dst[1] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[1] + O[1] + add)>>shift );
    dst[2] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[1] - O[1] + add)>>shift );
    dst[3] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[0] - O[0] + add)>>shift );
            
    src   ++;
    dst += 4;
  }
}

/** 8x8 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly8(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;  
  int E[4],O[4];
  int EE[2],EO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {  
    /* E and O*/
    for (k=0;k<4;k++)
    {
      E[k] = src[k] + src[7-k];
      O[k] = src[k] - src[7-k];
    }    
    /* EE and EO */
    EE[0] = E[0] + E[3];    
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0]      = (g_aiT8[0][0]*EE[0] + g_aiT8[0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[4][0]*EE[0] + g_aiT8[4][1]*EE[1] + add)>>shift; 
    dst[2*line] = (g_aiT8[2][0]*EO[0] + g_aiT8[2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[6][0]*EO[0] + g_aiT8[6][1]*EO[1] + add)>>shift; 

    dst[line]   = (g_aiT8[1][0]*O[0] + g_aiT8[1][1]*O[1] + g_aiT8[1][2]*O[2] + g_aiT8[1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[3][0]*O[0] + g_aiT8[3][1]*O[1] + g_aiT8[3][2]*O[2] + g_aiT8[3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[5][0]*O[0] + g_aiT8[5][1]*O[1] + g_aiT8[5][2]*O[2] + g_aiT8[5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[7][0]*O[0] + g_aiT8[7][1]*O[1] + g_aiT8[7][2]*O[2] + g_aiT8[7][3]*O[3] + add)>>shift;

    src += 8;
    dst ++;
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse8(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;    
  int E[4],O[4];
  int EE[2],EO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++) 
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[ 1][k]*src[line] + g_aiT8[ 3][k]*src[3*line] + g_aiT8[ 5][k]*src[5*line] + g_aiT8[ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[2][0]*src[ 2*line ] + g_aiT8[6][0]*src[ 6*line ];
    EO[1] = g_aiT8[2][1]*src[ 2*line ] + g_aiT8[6][1]*src[ 6*line ];
    EE[0] = g_aiT8[0][0]*src[ 0      ] + g_aiT8[4][0]*src[ 4*line ];
    EE[1] = g_aiT8[0][1]*src[ 0      ] + g_aiT8[4][1]*src[ 4*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */ 
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (k=0;k<4;k++)
    {
      dst[ k   ] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[k] + O[k] + add)>>shift );
      dst[ k+4 ] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[3-k] - O[3-k] + add)>>shift );
    }   
    src ++;
    dst += 8;
  }
}

/** 16x16 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly16(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;
  int E[8],O[8];
  int EE[4],EO[4];
  int EEE[2],EEO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++) 
  {    
    /* E and O*/
    for (k=0;k<8;k++)
    {
      E[k] = src[k] + src[15-k];
      O[k] = src[k] - src[15-k];
    } 
    /* EE and EO */
    for (k=0;k<4;k++)
    {
      EE[k] = E[k] + E[7-k];
      EO[k] = E[k] - E[7-k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];    
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0      ] = (g_aiT16[ 0][0]*EEE[0] + g_aiT16[ 0][1]*EEE[1] + add)>>shift;        
    dst[ 8*line ] = (g_aiT16[ 8][0]*EEE[0] + g_aiT16[ 8][1]*EEE[1] + add)>>shift;    
    dst[ 4*line ] = (g_aiT16[ 4][0]*EEO[0] + g_aiT16[ 4][1]*EEO[1] + add)>>shift;        
    dst[ 12*line] = (g_aiT16[12][0]*EEO[0] + g_aiT16[12][1]*EEO[1] + add)>>shift;

    for (k=2;k<16;k+=4)
    {
      dst[ k*line ] = (g_aiT16[k][0]*EO[0] + g_aiT16[k][1]*EO[1] + g_aiT16[k][2]*EO[2] + g_aiT16[k][3]*EO[3] + add)>>shift;      
    }

    for (k=1;k<16;k+=2)
    {
      dst[ k*line ] = (g_aiT16[k][0]*O[0] + g_aiT16[k][1]*O[1] + g_aiT16[k][2]*O[2] + g_aiT16[k][3]*O[3] + 
        g_aiT16[k][4]*O[4] + g_aiT16[k][5]*O[5] + g_aiT16[k][6]*O[6] + g_aiT16[k][7]*O[7] + add)>>shift;
    }

    src += 16;
    dst ++; 

  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse16(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;  
  int E[8],O[8];
  int EE[4],EO[4];
  int EEE[2],EEO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<8;k++)
    {
      O[k] = g_aiT16[ 1][k]*src[ line] + g_aiT16[ 3][k]*src[ 3*line] + g_aiT16[ 5][k]*src[ 5*line] + g_aiT16[ 7][k]*src[ 7*line] + 
        g_aiT16[ 9][k]*src[ 9*line] + g_aiT16[11][k]*src[11*line] + g_aiT16[13][k]*src[13*line] + g_aiT16[15][k]*src[15*line];
    }
    for (k=0;k<4;k++)
    {
      EO[k] = g_aiT16[ 2][k]*src[ 2*line] + g_aiT16[ 6][k]*src[ 6*line] + g_aiT16[10][k]*src[10*line] + g_aiT16[14][k]*src[14*line];
    }
    EEO[0] = g_aiT16[4][0]*src[ 4*line ] + g_aiT16[12][0]*src[ 12*line ];
    EEE[0] = g_aiT16[0][0]*src[ 0      ] + g_aiT16[ 8][0]*src[ 8*line  ];
    EEO[1] = g_aiT16[4][1]*src[ 4*line ] + g_aiT16[12][1]*src[ 12*line ];
    EEE[1] = g_aiT16[0][1]*src[ 0      ] + g_aiT16[ 8][1]*src[ 8*line  ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */ 
    for (k=0;k<2;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+2] = EEE[1-k] - EEO[1-k];
    }    
    for (k=0;k<4;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+4] = EE[3-k] - EO[3-k];
    }    
    for (k=0;k<8;k++)
    {
      dst[k]   = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[k] + O[k] + add)>>shift );
      dst[k+8] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[7-k] - O[7-k] + add)>>shift );
    }   
    src ++; 
    dst += 16;
  }
}

/** 32x32 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly32(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;
  int E[16],O[16];
  int EE[8],EO[8];
  int EEE[4],EEO[4];
  int EEEE[2],EEEO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {    
    /* E and O*/
    for (k=0;k<16;k++)
    {
      E[k] = src[k] + src[31-k];
      O[k] = src[k] - src[31-k];
    } 
    /* EE and EO */
    for (k=0;k<8;k++)
    {
      EE[k] = E[k] + E[15-k];
      EO[k] = E[k] - E[15-k];
    }
    /* EEE and EEO */
    for (k=0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7-k];
      EEO[k] = EE[k] - EE[7-k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];    
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[ 0       ] = (g_aiT32[ 0][0]*EEEE[0] + g_aiT32[ 0][1]*EEEE[1] + add)>>shift;
    dst[ 16*line ] = (g_aiT32[16][0]*EEEE[0] + g_aiT32[16][1]*EEEE[1] + add)>>shift;
    dst[ 8*line  ] = (g_aiT32[ 8][0]*EEEO[0] + g_aiT32[ 8][1]*EEEO[1] + add)>>shift; 
    dst[ 24*line ] = (g_aiT32[24][0]*EEEO[0] + g_aiT32[24][1]*EEEO[1] + add)>>shift;
    for (k=4;k<32;k+=8)
    {
      dst[ k*line ] = (g_aiT32[k][0]*EEO[0] + g_aiT32[k][1]*EEO[1] + g_aiT32[k][2]*EEO[2] + g_aiT32[k][3]*EEO[3] + add)>>shift;
    }       
    for (k=2;k<32;k+=4)
    {
      dst[ k*line ] = (g_aiT32[k][0]*EO[0] + g_aiT32[k][1]*EO[1] + g_aiT32[k][2]*EO[2] + g_aiT32[k][3]*EO[3] + 
        g_aiT32[k][4]*EO[4] + g_aiT32[k][5]*EO[5] + g_aiT32[k][6]*EO[6] + g_aiT32[k][7]*EO[7] + add)>>shift;
    }       
    for (k=1;k<32;k+=2)
    {
      dst[ k*line ] = (g_aiT32[k][ 0]*O[ 0] + g_aiT32[k][ 1]*O[ 1] + g_aiT32[k][ 2]*O[ 2] + g_aiT32[k][ 3]*O[ 3] + 
        g_aiT32[k][ 4]*O[ 4] + g_aiT32[k][ 5]*O[ 5] + g_aiT32[k][ 6]*O[ 6] + g_aiT32[k][ 7]*O[ 7] +
        g_aiT32[k][ 8]*O[ 8] + g_aiT32[k][ 9]*O[ 9] + g_aiT32[k][10]*O[10] + g_aiT32[k][11]*O[11] + 
        g_aiT32[k][12]*O[12] + g_aiT32[k][13]*O[13] + g_aiT32[k][14]*O[14] + g_aiT32[k][15]*O[15] + add)>>shift;
    }
    src += 32;
    dst ++;
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse32(TCoeff *src, TCoeff *dst, int shift, int line)
{
  int j,k;  
  int E[16],O[16];
  int EE[8],EO[8];
  int EEE[4],EEO[4];
  int EEEE[2],EEEO[2];
  int add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {    
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<16;k++)
    {
      O[k] = g_aiT32[ 1][k]*src[ line  ] + g_aiT32[ 3][k]*src[ 3*line  ] + g_aiT32[ 5][k]*src[ 5*line  ] + g_aiT32[ 7][k]*src[ 7*line  ] + 
        g_aiT32[ 9][k]*src[ 9*line  ] + g_aiT32[11][k]*src[ 11*line ] + g_aiT32[13][k]*src[ 13*line ] + g_aiT32[15][k]*src[ 15*line ] + 
        g_aiT32[17][k]*src[ 17*line ] + g_aiT32[19][k]*src[ 19*line ] + g_aiT32[21][k]*src[ 21*line ] + g_aiT32[23][k]*src[ 23*line ] + 
        g_aiT32[25][k]*src[ 25*line ] + g_aiT32[27][k]*src[ 27*line ] + g_aiT32[29][k]*src[ 29*line ] + g_aiT32[31][k]*src[ 31*line ];
    }
    for (k=0;k<8;k++)
    {
      EO[k] = g_aiT32[ 2][k]*src[ 2*line  ] + g_aiT32[ 6][k]*src[ 6*line  ] + g_aiT32[10][k]*src[ 10*line ] + g_aiT32[14][k]*src[ 14*line ] + 
        g_aiT32[18][k]*src[ 18*line ] + g_aiT32[22][k]*src[ 22*line ] + g_aiT32[26][k]*src[ 26*line ] + g_aiT32[30][k]*src[ 30*line ];
    }
    for (k=0;k<4;k++)
    {
      EEO[k] = g_aiT32[4][k]*src[ 4*line ] + g_aiT32[12][k]*src[ 12*line ] + g_aiT32[20][k]*src[ 20*line ] + g_aiT32[28][k]*src[ 28*line ];
    }
    EEEO[0] = g_aiT32[8][0]*src[ 8*line ] + g_aiT32[24][0]*src[ 24*line ];
    EEEO[1] = g_aiT32[8][1]*src[ 8*line ] + g_aiT32[24][1]*src[ 24*line ];
    EEEE[0] = g_aiT32[0][0]*src[ 0      ] + g_aiT32[16][0]*src[ 16*line ];    
    EEEE[1] = g_aiT32[0][1]*src[ 0      ] + g_aiT32[16][1]*src[ 16*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];    
    for (k=0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+4] = EEE[3-k] - EEO[3-k];
    }    
    for (k=0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+8] = EE[7-k] - EO[7-k];
    }    
    for (k=0;k<16;k++)
    {
      dst[k]    = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[k] + O[k] + add)>>shift );
      dst[k+16] = Clip3( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, (E[15-k] - O[15-k] + add)>>shift );
    }
    src ++;
    dst += 32;
  }
}

/** MxN forward transform (2D)
*  \param block input data (residual)
*  \param coeff output data (transform coefficients)
*  \param iWidth input data (width of transform)
*  \param iHeight input data (height of transform)
*/
void xTrMxN(TCoeff *block, TCoeff *coeff, int iWidth, int iHeight, UInt uiMode)
{
#if FULL_NBIT
  Int shift_1st = ((g_aucConvertToBit[iWidth] + 2) +  g_uiBitDepth          + TRANSFORM_MATRIX_SHIFT) - MAX_TR_DYNAMIC_RANGE;
#else
  Int shift_1st = ((g_aucConvertToBit[iWidth] + 2) + (g_uiBitIncrement + 8) + TRANSFORM_MATRIX_SHIFT) - MAX_TR_DYNAMIC_RANGE;
#endif
  Int shift_2nd = (g_aucConvertToBit[iHeight] + 2) + TRANSFORM_MATRIX_SHIFT;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[ 64 * 64 ];

  switch (iWidth)
  {
    case 4:
      {
#if INTRA_TRANS_SIMP
        if ((iHeight == 4) && (uiMode != REG_DCT))    // Check for DCT or DST
#else
        if ((iHeight == 4) && (uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 2 && uiMode <= 25)))    // Check for DCT or DST
#endif
        {
           fastForwardDst( block, tmp, shift_1st );
        }
        else partialButterfly4 ( block, tmp, shift_1st, iHeight );
      }
      break;

    case 8:     partialButterfly8 ( block, tmp, shift_1st, iHeight );  break;
    case 16:    partialButterfly16( block, tmp, shift_1st, iHeight );  break;
    case 32:    partialButterfly32( block, tmp, shift_1st, iHeight );  break;
    default:
      assert(0); exit (1); break;
  }

  switch (iHeight)
  {
    case 4:
      {
#if INTRA_TRANS_SIMP
        if ((iWidth == 4) && (uiMode != REG_DCT))    // Check for DCT or DST
#else
        if ((iWidth == 4) && (uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 11 && uiMode <= 34)))    // Check for DCT or DST
#endif
        {
          fastForwardDst( tmp, coeff, shift_2nd );
        }
        else partialButterfly4 ( tmp, coeff, shift_2nd, iWidth );
      }
      break;

    case 8:     partialButterfly8 ( tmp, coeff, shift_2nd, iWidth );    break;
    case 16:    partialButterfly16( tmp, coeff, shift_2nd, iWidth );    break;
    case 32:    partialButterfly32( tmp, coeff, shift_2nd, iWidth );    break;
    default:
      assert(0); exit (1); break;
  }
}


/** MxN inverse transform (2D)
*  \param coeff input data (transform coefficients)
*  \param block output data (residual)
*  \param iWidth input data (width of transform)
*  \param iHeight input data (height of transform)
*/
void xITrMxN(TCoeff *coeff, TCoeff *block, int iWidth, int iHeight, UInt uiMode)
{
  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
#if FULL_NBIT
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + MAX_TR_DYNAMIC_RANGE - 1) - (g_uiBitDepth);
#else
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + MAX_TR_DYNAMIC_RANGE - 1) - (g_uiBitIncrement + 8);
#endif

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[64*64];

  switch (iHeight)
  {
    case 4:
      {
#if INTRA_TRANS_SIMP
        if ((iWidth == 4) && (uiMode != REG_DCT))    // Check for DCT or DST
#else
        if ((iWidth == 4) && (uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 11 && uiMode <= 34)))    // Check for DCT or DST
#endif
        {
          fastInverseDst( coeff, tmp, shift_1st);
        }
        else partialButterflyInverse4 ( coeff, tmp, shift_1st, iWidth);
      }
      break;

    case  8: partialButterflyInverse8 ( coeff, tmp, shift_1st, iWidth); break;
    case 16: partialButterflyInverse16( coeff, tmp, shift_1st, iWidth); break;
    case 32: partialButterflyInverse32( coeff, tmp, shift_1st, iWidth); break;
    default:
      assert(0); exit (1); break;
  }

  switch (iWidth)
  {
    case 4:
      {
#if INTRA_TRANS_SIMP
        if ((iHeight == 4) && (uiMode != REG_DCT))    // Check for DCT or DST
#else
        if ((iHeight == 4) && (uiMode != REG_DCT) && ((uiMode == 0) || (uiMode >= 2 && uiMode <= 25)))    // Check for DCT or DST
#endif
        {
          fastInverseDst( tmp, block, shift_2nd );
        }
        else partialButterflyInverse4 ( tmp, block, shift_2nd, iHeight);
      }
      break;

    case  8: partialButterflyInverse8 ( tmp, block, shift_2nd, iHeight); break;
    case 16: partialButterflyInverse16( tmp, block, shift_2nd, iHeight); break;
    case 32: partialButterflyInverse32( tmp, block, shift_2nd, iHeight); break;
    default:
      assert(0); exit (1); break;
  }
}


// To minimize the distortion only. No rate is considered. 
Void TComTrQuant::signBitHidingHDQ( TComDataCU* pcCU, TCoeff* pQCoef, TCoeff* pCoef, Int* deltaU, const TUEntropyCodingParameters &codingParameters )
{
  const UInt width         = codingParameters.widthInGroups  << codingParameters.log2GroupWidth;
  const UInt height        = codingParameters.heightInGroups << codingParameters.log2GroupHeight;
  const UInt log2GroupSize = codingParameters.log2GroupWidth + codingParameters.log2GroupHeight;
  const UInt groupSize     = 1 << log2GroupSize;

  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> log2GroupSize; subSet >= 0; subSet-- )
  {
    Int  subPos = subSet << log2GroupSize;
    Int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += pQCoef[ codingParameters.scan[ n + subPos ]];
    }

    if(lastNZPosInCG>=0 && lastCG==-1) 
    {
      lastCG = 1 ; 
    }
    
    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      UInt signbit = (pQCoef[codingParameters.scan[subPos+firstNZPosInCG]]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        Int minCostInc = MAX_INT,  minPos =-1, finalChange=0, curCost=MAX_INT, curChange=0;
        
        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )
        {
          UInt blkPos   = codingParameters.scan[ n+subPos ];
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos]; 
              curChange=1 ;
            }
            else 
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost=MAX_INT ; 
              }
              else
              {
                curCost = deltaU[blkPos]; 
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              UInt thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = MAX_INT;
              }
              else
              { 
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == TRANSFORM_MAXIMUM || pQCoef[minPos] == TRANSFORM_MINIMUM)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ; 
        }
        else 
        { 
          pQCoef[minPos] -= finalChange ;
        }  
      } // Hide
    }
    if(lastCG==1) 
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}


Void TComTrQuant::xQuant(       TComTU       &rTu,
                                TCoeff      * pSrc,
                                TCoeff      * pDes,
#if ADAPTIVE_QP_SELECTION
                                TCoeff      *&pArlDes,
#endif
                                UInt         &uiAcSum,
                          const ComponentID   compID,
                          const QpParam      &cQP )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();

  TCoeff* piCoef    = pSrc;
  TCoeff* piQCoef   = pDes;
#if ADAPTIVE_QP_SELECTION
  TCoeff* piArlCCoef = pArlDes;
#endif
  const ChromaFormat chFmt = pcCU->getPic()->getChromaFormat();

  const Bool useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);
  Bool useRDOQForTransformSkip = !(m_useTransformSkipFast && useTransformSkip);
  if ( m_bUseRDOQ && (isLuma(compID) || RDOQ_CHROMA) && useRDOQForTransformSkip )
  {
#if ADAPTIVE_QP_SELECTION
    xRateDistOptQuant( rTu, piCoef, pDes, pArlDes, uiAcSum, compID, cQP );
#else
    xRateDistOptQuant( rTu, piCoef, pDes, uiAcSum, compID, cQP );
#endif
  }
  else
  {
    TUEntropyCodingParameters codingParameters;
    getTUEntropyCodingParameters(codingParameters, pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, uiHeight, compID), uiWidth, uiHeight, compID, chFmt);

    Int additionalMultiplier;
    Int additionalShift;
    getAdditionalQuantiserMultiplyAndShift(additionalMultiplier, additionalShift, compID, chFmt, useTransformSkip);

    Int deltaU[32*32] ;

#if ADAPTIVE_QP_SELECTION && ECF__BACKWARDS_COMPATIBILITY_HM
    const Int qpBDOffset   = pcCU->getSlice()->getSPS()->getQpBDOffset(toChannelType(compID));
    const Int iQpBase      = pcCU->getSlice()->getSliceQpBase();
    const Int chromaOffset = 0; // NOTE ECF - with RDOQ disabled, the original code does not use chroma offset. If chroma offsets are non-zero, the resulting reconstructed image will contain additional noise.
    
    QpParam cQpBase;
    setQPforQuant(cQpBase, iQpBase, toChannelType(compID), qpBDOffset, chromaOffset, chFmt, pcCU->getTransformSkip(uiAbsPartIdx,compID));
#endif

#if !REMOVE_NSQT
    const ScalingListDIR dir=getScalingListDIR(uiWidth, uiHeight, pcCU->getPic()->getChromaFormat(), compID);
#endif
    const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);
    assert(compID<MAX_NUM_COMPONENT);

#if REMOVE_NSQT
    Int scalingListType = getScalingListType(pcCU->isIntra(uiAbsPartIdx), uiLog2TrSize, compID);
    assert(scalingListType < SCALING_LIST_NUM);
    Int *piQuantCoeff = 0;
    piQuantCoeff = getQuantCoeff(scalingListType,cQP.getAdjustedQp().rem,uiLog2TrSize-2);
#else
    Int scalingListType = getScalingListType(pcCU->isIntra(uiAbsPartIdx), uiLog2TrSize, compID, dir);
    assert(scalingListType < SCALING_LIST_NUM);
    Int *piQuantCoeff = 0;
    piQuantCoeff = getQuantCoeff(scalingListType,cQP.getAdjustedQp().rem,uiLog2TrSize-2, dir);
#endif


    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */

    // Represents scaling through forward transform
    Int iTransformShift = getTransformShift(uiLog2TrSize) + (roundTransformShiftUp(compID, chFmt, useTransformSkip) ? 1 : 0);
    Int iQBits  = MAX_INT;
#if ADAPTIVE_QP_SELECTION
    Int iQBitsC = MAX_INT;
    Int iAddC   = MAX_INT;

#if   (ECF__BACKWARDS_COMPATIBILITY_HM == 1)
    const Int QP_base_per = cQpBase.getAdjustedQp().per;
    iQBits  = QUANT_SHIFT + QP_base_per + iTransformShift;
    iQBitsC = QUANT_SHIFT + QP_base_per + iTransformShift - ARL_C_PRECISION;
    iAddC   = 1 << (iQBitsC-1);
#elif (ECF__BACKWARDS_COMPATIBILITY_HM == 0)
    //NOTE: ECF - The forward quantisation should be based on the same Qp (cQP) as the dequantisation
    iQBits = QUANT_SHIFT + cQP.getAdjustedQp().per + iTransformShift;

    if (m_bUseAdaptQpSelect)
    {
      iQBitsC = iQBits - ARL_C_PRECISION;
      iAddC   = 1 << (iQBitsC-1);
    }
#endif
#else
    iQBits = QUANT_SHIFT + cQP..getAdjustedQp().per + iTransformShift; // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
#endif
    //NOTE: ECF - QBits will be OK for any bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

    Int iAdd = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);

    Int qBits8 = iQBits-8;

    for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )
    {
      TCoeff iLevel;
      Int    iSign;
      iLevel  = piCoef[uiBlockPos];
      iSign   = (iLevel < 0 ? -1: 1);
            

      const UInt scalingListCoeffIdx = getScalingListCoeffIdx(chFmt, compID, uiBlockPos, uiWidth, uiHeight);
      Int64 tmpLevel = (Int64)abs(iLevel) * piQuantCoeff[scalingListCoeffIdx];

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlCCoef[uiBlockPos] = (TCoeff)((tmpLevel + iAddC ) >> iQBitsC);
      }
#endif

      if ((additionalMultiplier != 1) || (additionalShift != 0))
        tmpLevel = ((tmpLevel * additionalMultiplier) + ((1 << additionalShift) - 1)) >> additionalShift;

      iLevel = (TCoeff)((tmpLevel + iAdd ) >> iQBits);
      deltaU[uiBlockPos] = (Int)((tmpLevel - (iLevel<<iQBits) )>> qBits8);

      uiAcSum += iLevel;
      iLevel *= iSign;        
      piQCoef[uiBlockPos] = Clip3<TCoeff>( TRANSFORM_MINIMUM, TRANSFORM_MAXIMUM, iLevel );
    } // for n

    if( pcCU->getSlice()->getPPS()->getSignHideFlag() )
    {
      if(uiAcSum>=2)
      {
        signBitHidingHDQ( pcCU, piQCoef, piCoef, deltaU, codingParameters ) ;
      }
    }
  } //if RDOQ
  //return;
}

Void TComTrQuant::xDeQuant(       TComTU        &rTu,
                            const TCoeff       * pSrc,
                                  TCoeff       * pDes,
                            const ComponentID    compID,
                            const QpParam       &cQP )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth = rect.width;
  const UInt uiHeight = rect.height;
  
  const TCoeff* piQCoef   = pSrc;
  TCoeff*       piCoef    = pDes;
  const ChromaFormat fmt=rTu.GetChromaFormat();
#if !REMOVE_NSQT
  const ScalingListDIR dir=getScalingListDIR(uiWidth, uiHeight, fmt, compID);
#endif
  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);
  const UInt numSamplesInBlock=uiWidth*uiHeight;
  assert(compID<MAX_NUM_COMPONENT);

  const Bool useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);

  Int scalingListType = getScalingListType(pcCU->isIntra(uiAbsPartIdx), uiLog2TrSize, compID
#if !REMOVE_NSQT
      , dir
#endif
      );
  assert(scalingListType < SCALING_LIST_NUM);

  assert ( uiWidth <= m_uiMaxTrSize );
  

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(uiLog2TrSize) + (roundTransformShiftUp(compID, fmt, useTransformSkip) ? 1 : 0);

  Int additionalMultiplier;
  Int additionalShift;
  getAdditionalInverseQuantiserMultiplyAndShift(additionalMultiplier, additionalShift, compID, fmt, useTransformSkip);

  const Int QP_per = cQP.getAdjustedQp().per;
  const Int QP_rem = cQP.getAdjustedQp().rem;

  const Int rightShift     = (IQUANT_SHIFT - (iTransformShift + QP_per)) + additionalShift + (getUseScalingList() ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
  const Int additionalBits = ((additionalMultiplier == 1) ? 0 : ADDITIONAL_MULTIPLIER_BITS);
  
  TCoeff clipQCoef;
  Intermediate_Int iCoeffQ;

  if(getUseScalingList())
  {
    //from the dequantisation equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx] * additionalMultiplier) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits        +  additionalBits                 - rightShift
    const UInt             dequantCoefBits     = ((1 + IQUANT_SHIFT) + SCALING_LIST_BITS);
    const UInt             targetInputBitDepth = std::min<UInt>((MAX_TR_DYNAMIC_RANGE + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - (dequantCoefBits + additionalBits)));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

#if REMOVE_NSQT
    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);
#else
    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2,dir);
#endif

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      if (additionalMultiplier != 1)
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);

          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          //NOTE: ECF - 64-bit operation required here to accommodate additionalMultiplier
          iCoeffQ   = Intermediate_Int(((Int64(clipQCoef) * piDequantCoef[deQuantIdx] * additionalMultiplier) + iAdd ) >> rightShift);
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
      else
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);

          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift;
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      if (additionalMultiplier != 1)
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);

          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          //NOTE: ECF - 64-bit operation required here to accommodate additionalMultiplier
          iCoeffQ   = Intermediate_Int((Int64(clipQCoef) * piDequantCoef[deQuantIdx] * additionalMultiplier) << leftShift);
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
      else
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);

          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) << leftShift;
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
    }
  }
  else
  {
    const Int scale     = (getInverseQuantScaling(QP_rem, fmt)) * additionalMultiplier;
    const Int scaleBits =          (IQUANT_SHIFT + 1)           + additionalBits;

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const UInt             targetInputBitDepth = std::min<UInt>((MAX_TR_DYNAMIC_RANGE + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      if (additionalMultiplier != 1)
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          //NOTE: ECF - 64-bit operation required here to accommodate additionalMultiplier
          iCoeffQ   = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
      else
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      if (additionalMultiplier != 1)
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          //NOTE: ECF - 64-bit operation required here to accommodate additionalMultiplier
          iCoeffQ   = Intermediate_Int((Int64(clipQCoef) * scale) << leftShift);
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ));
        }
      }
      else
      {
        for( Int n = 0; n < numSamplesInBlock; n++ )
        {
          clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
          iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(TRANSFORM_MINIMUM,TRANSFORM_MAXIMUM,iCoeffQ)); 
        }
      }
    }
  }
}


Void TComTrQuant::init( UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxTrSize, Int iSymbolMode, UInt *aTableLP4, UInt *aTableLP8, UInt *aTableLastPosVlcIndex,
                       Bool bUseRDOQ,  Bool bEnc, Bool useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                       , Bool bUseAdaptQpSelect
#endif
                       )
{
  m_uiMaxTrSize  = uiMaxTrSize;
  m_bEnc         = bEnc;
  m_bUseRDOQ     = bUseRDOQ;
#if ADAPTIVE_QP_SELECTION
  m_bUseAdaptQpSelect = bUseAdaptQpSelect;
#endif
  m_useTransformSkipFast = useTransformSkipFast;
}


Void TComTrQuant::transformNxN(       TComTU        & rTu,
                                const ComponentID     compID,
                                      Pel          *  pcResidual,
                                const UInt            uiStride,
                                      TCoeff       *  rpcCoeff,
#if ADAPTIVE_QP_SELECTION
                                      TCoeff       *& rpcArlCoeff,
#endif
                                      UInt          & uiAbsSum,
                                const QpParam       & cQP
                              )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();

  if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    uiAbsSum=0;
    for (UInt k = 0; k<uiHeight; k++)
    {
      for (UInt j = 0; j<uiWidth; j++)
      {
        rpcCoeff[k*uiWidth+j]= pcResidual[k*uiStride+j];
        uiAbsSum += abs(pcResidual[k*uiStride+j]);
      }
    }
    return;
  }

  const UInt uiMode=getMDDTmode(compID, pcCU, uiAbsPartIdx);  //luma intra pred

  uiAbsSum = 0;
  assert( (pcCU->getSlice()->getSPS()->getMaxTrSize() >= uiWidth) );

  if(pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0)
  {
    xTransformSkip( pcResidual, uiStride, m_plTempCoeff, rTu, compID );
  }
  else
  {
    xT( uiMode, pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight );
  }

  xQuant( rTu, m_plTempCoeff, rpcCoeff,

#if ADAPTIVE_QP_SELECTION
       rpcArlCoeff,
#endif
       uiAbsSum, compID, cQP );
}


Void TComTrQuant::invTransformNxN(      TComTU        &rTu,
                                  const ComponentID    compID,
                                        Pel          *&rpcResidual,
                                  const UInt           uiStride,
                                        TCoeff       * pcCoeff,
                                  const QpParam       &cQP
                                        DEBUG_STRING_FN_DECLAREP(psDebug))
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth = rect.width;
  const UInt uiHeight = rect.height;


#if defined DEBUG_STRING
  if (psDebug)
  {
    std::stringstream ss(stringstream::out);
    printBlockToStream(ss, (compID==0)?"###InvTran ip Ch0: " : ((compID==1)?"###InvTran ip Ch1: ":"###InvTran ip Ch2: "), pcCoeff, uiWidth, uiHeight, uiWidth);
    DEBUG_STRING_APPEND((*psDebug), ss.str())
  }
#endif

  if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    for (UInt k = 0; k<uiHeight; k++)
    {
      for (UInt j = 0; j<uiWidth; j++)
      {
        rpcResidual[k*uiStride+j] = pcCoeff[k*uiWidth+j];
      }
    }
    return;
  }
  
  const UInt uiMode=getMDDTmode(compID, pcCU, uiAbsPartIdx);

  xDeQuant( rTu, pcCoeff, m_plTempCoeff, compID, cQP);

#if defined DEBUG_STRING
  if (psDebug)
  {
    std::stringstream ss(stringstream::out);
    printBlockToStream(ss, "###InvTran deq: ", m_plTempCoeff, uiWidth, uiHeight, uiWidth);
    (*psDebug)+=ss.str();
  }
#endif

  if(pcCU->getTransformSkip(uiAbsPartIdx, compID))
  {
    xITransformSkip( m_plTempCoeff, rpcResidual, uiStride, rTu, compID );

#if defined DEBUG_STRING
    if (psDebug)
    {
      Int iTransformShift = getTransformShift(rTu.GetEquivalentLog2TrSize(compID)) + (roundTransformShiftUp(compID, rTu.GetChromaFormat(), true) ? 1 : 0);
      std::stringstream ss(stringstream::out);
      printBlockToStream(ss, "###InvTran resi: ", rpcResidual, uiWidth, uiHeight, uiStride);
      ss << "TransShift=" << iTransformShift << "\n";
      (*psDebug)+=ss.str();
      (*psDebug)+="(TS)\n";
    }
#endif
  }
  else
  {
    xIT( uiMode, m_plTempCoeff, rpcResidual, uiStride, uiWidth, uiHeight );

#if defined DEBUG_STRING
    if (psDebug)
    {
      std::stringstream ss(stringstream::out);
      printBlockToStream(ss, "###InvTran resi: ", rpcResidual, uiWidth, uiHeight, uiStride);
      (*psDebug)+=ss.str();
      (*psDebug)+="(Non TS)\n";
    }
#endif
  }

  if (((uiWidth == uiHeight) && ((g_debugCounter & uiWidth) == 0)) || ((uiWidth != uiHeight) && ((g_debugCounter & (uiWidth << 5)) == 0)))
    g_debugCounter |= (uiWidth == uiHeight) ? uiWidth : (uiWidth << 5);
}

Void TComTrQuant::invRecurTransformNxN( const ComponentID compID,
                                        Pel* rpcResidual,
                                        UInt uiStride,
                                        TComTU &rTu)
{
  if (!rTu.ProcessComponentSection(compID)) return;

  TComDataCU* pcCU = rTu.getCU();
  UInt absPartIdxTU = rTu.GetAbsPartIdxTU();
  UInt uiTrMode=rTu.GetTransformDepthRel();
  if( !pcCU->getCbf(absPartIdxTU, compID, uiTrMode) )
  {
    return;
  }

  if( uiTrMode == pcCU->getTransformIdx( absPartIdxTU ) )
  {
    const TComRectangle &tuRect=rTu.getRect(compID);
    UInt uiAddr =  (tuRect.x0 + uiStride*tuRect.y0);
    Pel* pResi = rpcResidual + uiAddr;
    TCoeff* rpcCoeff = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);

    QpParam cQP; // NOTE ECF - set QP is now possibly dependent on TS flag (for 4:2:2), and therefore set at the lowest level.
    setQPforQuant( cQP,
                   pcCU->getQP( 0 ),
                   toChannelType(compID),
                   pcCU->getSlice()->getSPS()->getQpBDOffset(toChannelType(compID)),
#if CHROMA_QP_EXTENSION
                   (pcCU->getSlice()->getPPS()->getQpOffset(compID) + pcCU->getSlice()->getSliceChromaQpDelta(compID)),
#else
                   pcCU->getSlice()->getPPS()->getQpOffset(compID),
#endif
                   pcCU->getPic()->getChromaFormat(),
                   (pcCU->getTransformSkip(absPartIdxTU, compID)));

#if defined DEBUG_STRING && DEBUG_INTER_CODING_INV_TRAN
    std::string sTemp;
    invTransformNxN( rTu, compID, pResi, uiStride, rpcCoeff, cQP DEBUG_STRING_PASS_INTO(&sTemp) );
    std::cout << sTemp;
#else
    invTransformNxN( rTu, compID, pResi, uiStride, rpcCoeff, cQP DEBUG_STRING_PASS_INTO(0) );
#endif

  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      invRecurTransformNxN( compID, rpcResidual, uiStride, tuRecurseChild );
    }
    while (tuRecurseChild.nextSection(rTu));
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

/** Wrapper function between HM interface and core NxN forward transform (2D) 
 *  \param piBlkResi input data (residual)
 *  \param psCoeff output data (transform coefficients)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void TComTrQuant::xT( UInt uiMode, Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight )
{
#if MATRIX_MULT
  if( iWidth == iHeight)
  {
    xTr(piBlkResi,psCoeff,uiStride,(UInt)iWidth,uiMode);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  for (Int y = 0; y < iHeight; y++)
    for (Int x = 0; x < iWidth; x++)
    {
      block[(y * iWidth) + x] = piBlkResi[(y * uiStride) + x];
    }

  xTrMxN( block, coeff, iWidth, iHeight, uiMode );

  memcpy(psCoeff, coeff, (iWidth * iHeight * sizeof(TCoeff)));
}

/** Wrapper function between HM interface and core NxN inverse transform (2D) 
 *  \param plCoef input data (transform coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void TComTrQuant::xIT( UInt uiMode, TCoeff* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight )
{
#if MATRIX_MULT
  if( iWidth == iHeight )
  {
    xITr(plCoef,pResidual,uiStride,(UInt)iWidth,uiMode);
    return;
  }
#endif  
 
  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  memcpy(coeff, plCoef, (iWidth * iHeight * sizeof(TCoeff)));
  
  xITrMxN( coeff, block, iWidth, iHeight, uiMode );

  for (Int y = 0; y < iHeight; y++)
    for (Int x = 0; x < iWidth; x++)
    {
      pResidual[(y * uiStride) + x] = Pel(block[(y * iWidth) + x]);
    }
}

/** Wrapper function between HM interface and core 4x4 transform skipping
 *  \param piBlkResi input data (residual)
 *  \param psCoeff output data (transform coefficients)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 */
Void TComTrQuant::xTransformSkip( Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, TComTU &rTu, const ComponentID component )
{
  const TComRectangle &rect = rTu.getRect(component);
  const Int width = rect.width;
  const Int height = rect.height;

  Int iTransformShift = getTransformShift(rTu.GetEquivalentLog2TrSize(component)) + (roundTransformShiftUp(component, rTu.GetChromaFormat(), true) ? 1 : 0);

  if (iTransformShift >= 0)
  {
    for (Int y = 0; y < height; y++)
    {    
      for(Int x = 0; x < width; x++)
      {
        psCoeff[(y * width) + x] = piBlkResi[(y * uiStride) + x] << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const Int offset = 1 << (iTransformShift - 1);
   
    for (Int y = 0; y < height; y++)
    {    
      for(Int x = 0; x < width; x++)
      {
        psCoeff[(y * width) + x] = (piBlkResi[(y * uiStride) + x] + offset) >> iTransformShift;
      }
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping 
 *  \param plCoef input data (coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 */
Void TComTrQuant::xITransformSkip( TCoeff* plCoef, Pel* pResidual, UInt uiStride, TComTU &rTu, const ComponentID component )
{
  const TComRectangle &rect = rTu.getRect(component);
  const Int width = rect.width;
  const Int height = rect.height;

  Int iTransformShift = getTransformShift(rTu.GetEquivalentLog2TrSize(component)) + (roundTransformShiftUp(component, rTu.GetChromaFormat(), true) ? 1 : 0);

  if (iTransformShift >= 0)
  {
    const Int offset = 1 << (iTransformShift - 1);

    for (Int y = 0; y < height; y++)
    {    
      for(Int x = 0; x < width; x++)
      {
        pResidual[(y * uiStride) + x] =  (plCoef[(y * width) + x] + offset) >> iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for (Int y = 0; y < height; y++)
    {    
      for(Int x = 0; x < width; x++)
      {
        pResidual[(y * uiStride) + x] = plCoef[(y * width) + x] << iTransformShift;
      }
    }
  }
}
 
/** RDOQ with CABAC
 * \param pcCU pointer to coding unit structure
 * \param plSrcCoeff pointer to input buffer
 * \param piDstCoeff reference to pointer to output buffer
 * \param uiWidth block width
 * \param uiHeight block height
 * \param uiAbsSum reference to absolute sum of quantized transform coefficient
 * \param eTType plane type / luminance or chrominance
 * \param uiAbsPartIdx absolute partition index
 * \returns Void
 * Rate distortion optimized quantization for entropy
 * coding engines using probability models like CABAC
 */
Void TComTrQuant::xRateDistOptQuant                 (       TComTU       &rTu,
                                                            TCoeff      * plSrcCoeff,
                                                            TCoeff      * piDstCoeff,
#if ADAPTIVE_QP_SELECTION
                                                            TCoeff      *&piArlDstCoeff,
#endif
                                                            UInt         &uiAbsSum,
                                                      const ComponentID   compID,
                                                      const QpParam      &cQP  )
{
  const TComRectangle  & rect             = rTu.getRect(compID);
  const UInt             uiWidth          = rect.width;
  const UInt             uiHeight         = rect.height;
        TComDataCU    *  pcCU             = rTu.getCU();
  const UInt             uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const ChromaFormat     format           = rTu.GetChromaFormat();
  const Bool             useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);


#if !REMOVE_NSQT
  const ScalingListDIR dir = getScalingListDIR(uiWidth, uiHeight, format, compID);
#endif
  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  // Represents scaling through forward transform
  const Int iTransformShift      = getTransformShift(uiLog2TrSize);

  UInt       uiGoRiceParam       = 0;
  Double     d64BlockUncodedCost = 0;
  const UInt uiLog2BlockWidth    = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt uiLog2BlockHeight   = g_aucConvertToBit[ uiHeight ] + 2;
  const UInt uiMaxNumCoeff       = uiWidth * uiHeight;
  assert(compID<MAX_NUM_COMPONENT);
#if REMOVE_NSQT
  Int scalingListType = getScalingListType(pcCU->isIntra(uiAbsPartIdx), uiLog2TrSize, compID);
#else
  Int scalingListType = getScalingListType(pcCU->isIntra(uiAbsPartIdx), uiLog2TrSize, compID, dir);
#endif
  assert(scalingListType < SCALING_LIST_NUM);
 
#if ADAPTIVE_QP_SELECTION
  memset(piArlDstCoeff, 0, sizeof(TCoeff) *  uiMaxNumCoeff);
#endif

  Double pdCostCoeff [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Double pdCostSig   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Double pdCostCoeff0[ MAX_TU_SIZE * MAX_TU_SIZE ];
  memset( pdCostCoeff, 0, sizeof(Double) *  uiMaxNumCoeff );
  memset( pdCostSig,   0, sizeof(Double) *  uiMaxNumCoeff );
  Int rateIncUp   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Int rateIncDown [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Int sigRateDelta[ MAX_TU_SIZE * MAX_TU_SIZE ];
  Int deltaU      [ MAX_TU_SIZE * MAX_TU_SIZE ];
  memset( rateIncUp,    0, sizeof(Int) *  uiMaxNumCoeff );
  memset( rateIncDown,  0, sizeof(Int) *  uiMaxNumCoeff );
  memset( sigRateDelta, 0, sizeof(Int) *  uiMaxNumCoeff );
  memset( deltaU,       0, sizeof(Int) *  uiMaxNumCoeff );

  const Int iQBits = QUANT_SHIFT + cQP.getBaseQp().per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
#if REMOVE_NSQT
  const double *const pdErrScale = getErrScaleCoeff(scalingListType,uiLog2TrSize-2,cQP.getBaseQp().rem,getErrorScaleAdjustmentMode(compID, format));
  const Int    *const piQCoef    = getQuantCoeff(scalingListType,cQP.getAdjustedQp().rem,uiLog2TrSize-2);
#else
  const double *const pdErrScale = getErrScaleCoeff(scalingListType,uiLog2TrSize-2,cQP.getBaseQp().rem,dir,getErrorScaleAdjustmentMode(compID, format));
  const Int    *const piQCoef    = getQuantCoeff(scalingListType,cQP.getAdjustedQp().rem,uiLog2TrSize-2,dir);
#endif

#if ADAPTIVE_QP_SELECTION
  Int iQBitsC = iQBits - ARL_C_PRECISION;
  Int iAddC =  1 << (iQBitsC-1);
#endif

  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, uiHeight, compID), uiWidth, uiHeight, compID, format);
  const UInt log2GroupSize = codingParameters.log2GroupWidth + codingParameters.log2GroupHeight;
  const UInt uiCGSize = (1 << log2GroupSize);

  Int additionalMultiplier;
  Int additionalShift;
  getAdditionalQuantiserMultiplyAndShift(additionalMultiplier, additionalShift, compID, format, useTransformSkip);

  additionalShift += (cQP.getAdjustedQp().per - cQP.getBaseQp().per) + (roundTransformShiftUp(compID, format, useTransformSkip) ? 1 : 0);

  assert(additionalShift >= 0);

  Double pdCostCoeffGroupSig[ MLS_GRP_NUM ];
  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  Int iCGLastScanPos = -1;

  UInt    uiCtxSet            = 0;
  Int     c1                  = 1;
  Int     c2                  = 0;
#if !REMOVE_NUM_GREATER1
  UInt    uiNumOne            = 0;
#endif
  Double  d64BaseCost         = 0;
  Int     iLastScanPos        = -1;

  UInt    c1Idx     = 0;
  UInt    c2Idx     = 0;
  Int     baseLevel;

  memset( pdCostCoeffGroupSig,   0, sizeof(Double) * MLS_GRP_NUM );
  memset( uiSigCoeffGroupFlag,   0, sizeof(UInt) * MLS_GRP_NUM );

  UInt uiCGNum = uiWidth * uiHeight >> log2GroupSize;
  Int iScanPos;
  coeffGroupRDStats rdStats;

  const UInt significanceMapContextOffset = getSignificanceMapContextOffset(compID);

  for (Int iCGScanPos = uiCGNum-1; iCGScanPos >= 0; iCGScanPos--)
  {
    UInt uiCGBlkPos = codingParameters.scanCG[ iCGScanPos ];
    UInt uiCGPosY   = uiCGBlkPos / codingParameters.widthInGroups;
    UInt uiCGPosX   = uiCGBlkPos - (uiCGPosY * codingParameters.widthInGroups);

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    Int patternSigCtx = 0;
    if (!codingParameters.useFixedGridSignificanceMapContext)
    {
      patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);
    }
        
    for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
    {
      iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;
      //===== quantization =====
      UInt    uiBlkPos          = codingParameters.scan[iScanPos];
      // set coeff
      const UInt scalingListCoeffIdx = getScalingListCoeffIdx(format, compID, uiBlkPos, uiWidth, uiHeight);
      Double dTemp = pdErrScale[scalingListCoeffIdx];
      
      Int64 tmpLevel = Int64(abs(plSrcCoeff[ uiBlkPos ])) * piQCoef[scalingListCoeffIdx];

      if (additionalMultiplier != 1) tmpLevel *= additionalMultiplier;
      if (additionalShift      != 0) tmpLevel  = (tmpLevel + (Int64(1) << (additionalShift - 1))) >> additionalShift;

      Int lLevelDouble              = (Int)min<Int64>(tmpLevel, MAX_INT - (1 << (iQBits - 1)));

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlDstCoeff[uiBlkPos]   = (TCoeff)(( lLevelDouble + iAddC) >> iQBitsC );
      }
#endif
      UInt uiMaxAbsLevel        = (lLevelDouble + (1 << (iQBits - 1))) >> iQBits;

      Double dErr               = Double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * dTemp;
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
      {
        iLastScanPos            = iScanPos;
        uiCtxSet                = getContextSetIndex(compID, (iScanPos >> log2GroupSize), 0);
        iCGLastScanPos          = iCGScanPos;
      }

      if ( iLastScanPos >= 0 )
      {
        //===== coefficient level estimation =====
        UInt  uiLevel;
        UInt  uiOneCtx         = (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet) + c1;
        UInt  uiAbsCtx         = (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet) + c2;

        if( iScanPos == iLastScanPos )
        {
          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ], 
                                                  lLevelDouble, uiMaxAbsLevel, significanceMapContextOffset, uiOneCtx, uiAbsCtx, uiGoRiceParam, 
                                                  c1Idx, c2Idx, iQBits, dTemp, 1 );
        }
        else
        {
          UShort uiCtxSig      = significanceMapContextOffset + getSigCtxInc( patternSigCtx, codingParameters, iScanPos, uiLog2BlockWidth, uiLog2BlockHeight, toChannelType(compID) );

          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                  lLevelDouble, uiMaxAbsLevel, uiCtxSig, uiOneCtx, uiAbsCtx, uiGoRiceParam, 
                                                  c1Idx, c2Idx, iQBits, dTemp, 0 );

          sigRateDelta[ uiBlkPos ] = m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 1 ] - m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 0 ];
        }

        deltaU[ uiBlkPos ]        = (lLevelDouble - ((Int)uiLevel << iQBits)) >> (iQBits-8);
         
        if( uiLevel > 0 )
        {
          Int rateNow = xGetICRate( uiLevel, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx );
          rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx ) - rateNow;
          rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx ) - rateNow;
        }
        else // uiLevel == 0
        {
          rateIncUp   [ uiBlkPos ] = m_pcEstBitsSbac->m_greaterOneBits[ uiOneCtx ][ 0 ];
        }
        piDstCoeff[ uiBlkPos ] = uiLevel;
        d64BaseCost           += pdCostCoeff [ iScanPos ];

        baseLevel = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;
        if( uiLevel >= baseLevel )
        {
          if(uiLevel > 3*(1<<uiGoRiceParam))
          {
            uiGoRiceParam = min<UInt>((uiGoRiceParam+1), 4);
          }
        }
        if ( uiLevel >= 1)
        {
          c1Idx ++;
        }

        //===== update bin model =====
        if( uiLevel > 1 )
        {
          c1 = 0; 
          c2 += (c2 < 2);
#if !REMOVE_NUM_GREATER1
          uiNumOne++;
#endif
          c2Idx ++;
        }
        else if( (c1 < 3) && (c1 > 0) && uiLevel)
        {
          c1++;
        }

        //===== context set update =====
        if( ( iScanPos % uiCGSize == 0 ) && ( iScanPos > 0 ) )
        {
#if REMOVE_NUM_GREATER1
          uiCtxSet          = getContextSetIndex(compID, ((iScanPos - 1) >> log2GroupSize), (c1 == 0)); //(iScanPos - 1) because we do this **before** entering the final group
#else
          uiCtxSet          = getContextSetIndex(compID, ((iScanPos - 1) >> log2GroupSize), (uiNumOne > 0)); //(iScanPos - 1) because we do this **before** entering the final group
#endif
          c1                = 1;
          c2                = 0;
          uiGoRiceParam     = 0;
          c1Idx             = 0;
          c2Idx             = 0; 
#if !REMOVE_NUM_GREATER1
          uiNumOne        >>= 1;
#endif
        }
      }
      else
      {
        d64BaseCost    += pdCostCoeff0[ iScanPos ];
      }
      rdStats.d64SigCost += pdCostSig[ iScanPos ];
      if (iScanPosinCG == 0 )
      {
        rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];
      }
      if (piDstCoeff[ uiBlkPos ] )
      {
        uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;
        rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];
        rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];
        if ( iScanPosinCG != 0 )
        {
          rdStats.iNNZbeforePos0++;
        }
      }
    } //end for (iScanPosinCG)

    if (iCGLastScanPos >= 0) 
    {
      if( iCGScanPos )
      {
        if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
        {
          UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );
          d64BaseCost += xGetRateSigCoeffGroup(0, uiCtxSig) - rdStats.d64SigCost;;  
          pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig);  
        } 
        else
        {
          if (iCGScanPos < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.
          {
            if ( rdStats.iNNZbeforePos0 == 0 ) 
            {
              d64BaseCost -= rdStats.d64SigCost_0;
              rdStats.d64SigCost -= rdStats.d64SigCost_0;
            }
            // rd-cost if SigCoeffGroupFlag = 0, initialization
            Double d64CostZeroCG = d64BaseCost;

            // add SigCoeffGroupFlag cost to total cost
            UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );

            if (iCGScanPos < iCGLastScanPos)
            {
              d64BaseCost  += xGetRateSigCoeffGroup(1, uiCtxSig); 
              d64CostZeroCG += xGetRateSigCoeffGroup(0, uiCtxSig);  
              pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(1, uiCtxSig); 
            }

            // try to convert the current coeff group from non-zero to all-zero
            d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
            d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
            d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

            // if we can save cost, change this block to all-zero block
            if ( d64CostZeroCG < d64BaseCost )      
            {
              uiSigCoeffGroupFlag[ uiCGBlkPos ] = 0;
              d64BaseCost = d64CostZeroCG;
              if (iCGScanPos < iCGLastScanPos)
              {
                pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig); 
              }
              // reset coeffs to 0 in this block                
              for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
              {
                iScanPos      = iCGScanPos*uiCGSize + iScanPosinCG;
                UInt uiBlkPos = codingParameters.scan[ iScanPos ];

                if (piDstCoeff[ uiBlkPos ])
                {
                  piDstCoeff [ uiBlkPos ] = 0;
                  pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
                  pdCostSig  [ iScanPos ] = 0;
                }
              }
            } // end if ( d64CostAllZeros < d64BaseCost )      
          }
        } // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
      }
      else
      {
        uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;
      }
    }
  } //end for (iCGScanPos)

  //===== estimate last position =====
  if ( iLastScanPos < 0 )
  {
    return;
  }

  Double  d64BestCost         = 0;
  Int     ui16CtxCbf          = 0;
  Int     iBestLastIdxP1      = 0;
  if( !pcCU->isIntra( uiAbsPartIdx ) && isLuma(compID) && pcCU->getTransformIdx( uiAbsPartIdx ) == 0 )
  {
    ui16CtxCbf   = 0;
    d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 0 ] );
    d64BaseCost += xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 1 ] );
  }
  else
  {
    ui16CtxCbf   = pcCU->getCtxQtCbf( rTu, toChannelType(compID), false );
    ui16CtxCbf  += getCBFContextOffset(compID);
    d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 0 ] );
    d64BaseCost += xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 1 ] );
  }


  Bool bFoundLast = false;
  for (Int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)
  {
    UInt uiCGBlkPos = codingParameters.scanCG[ iCGScanPos ];

    d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ]; 
    if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
    {     
      for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
      {
        iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;

        if (iScanPos > iLastScanPos) continue;
        UInt   uiBlkPos     = codingParameters.scan[iScanPos];

        if( piDstCoeff[ uiBlkPos ] )
        {
          UInt   uiPosY       = uiBlkPos >> uiLog2BlockWidth;
          UInt   uiPosX       = uiBlkPos - ( uiPosY << uiLog2BlockWidth );

          Double d64CostLast= codingParameters.scanType == SCAN_VER ? xGetRateLast( uiPosY, uiPosX, uiWidth, compID ) : xGetRateLast( uiPosX, uiPosY, uiWidth, compID );
          Double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

          if( totalCost < d64BestCost )
          {
            iBestLastIdxP1  = iScanPos + 1;
            d64BestCost     = totalCost;
          }
          if( piDstCoeff[ uiBlkPos ] > 1 )
          {
            bFoundLast = true;
            break;
          }
          d64BaseCost      -= pdCostCoeff[ iScanPos ];
          d64BaseCost      += pdCostCoeff0[ iScanPos ];
        }
        else
        {
          d64BaseCost      -= pdCostSig[ iScanPos ];
        }
      } //end for 
      if (bFoundLast)
      {
        break;
      }
    } // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
  } // end for 


  for ( Int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )
  {
    Int blkPos = codingParameters.scan[ scanPos ];
    Int level  = piDstCoeff[ blkPos ];
    uiAbsSum += level;
    piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
  }
  
  //===== clean uncoded coefficients =====
  for ( Int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )
  {
    piDstCoeff[ codingParameters.scan[ scanPos ] ] = 0;
  }


  if( pcCU->getSlice()->getPPS()->getSignHideFlag() && uiAbsSum>=2)
  {
    const Double inverseQuantScale = Double(getInverseQuantScaling(cQP.getBaseQp().rem, format));
    Int64 rdFactor = (Int64)(inverseQuantScale * inverseQuantScale * Double(1 << (2 * cQP.getBaseQp().per)) / m_dLambda / 16 / Double(1 << (2 * g_uiBitIncrement)) + 0.5);

    Int lastCG = -1;
    Int absSum = 0 ;
    Int n ;

    for( Int subSet = (uiWidth*uiHeight-1) >> log2GroupSize; subSet >= 0; subSet-- )
    {
      Int  subPos     = subSet << log2GroupSize;
      Int  firstNZPosInCG=uiCGSize , lastNZPosInCG=-1 ;
      absSum = 0 ;

      for(n = uiCGSize-1; n >= 0; --n )
      {
        if( piDstCoeff[ codingParameters.scan[ n + subPos ]] )
        {
          lastNZPosInCG = n;
          break;
        }
      }

      for(n = 0; n <uiCGSize; n++ )
      {
        if( piDstCoeff[ codingParameters.scan[ n + subPos ]] )
        {
          firstNZPosInCG = n;
          break;
        }
      }

      for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
      {
        absSum += piDstCoeff[ codingParameters.scan[ n + subPos ]];
      }

      if(lastNZPosInCG>=0 && lastCG==-1)
      {
        lastCG = 1;
      }
      
      if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
      {
        UInt signbit = (piDstCoeff[codingParameters.scan[subPos+firstNZPosInCG]]>0?0:1);
        if( signbit!=(absSum&0x1) )  // hide but need tune
        {
          // calculate the cost 
          Int64 minCostInc = MAX_INT64, curCost = MAX_INT64;
          Int minPos = -1, finalChange = 0, curChange = 0;

          for( n = (lastCG==1?lastNZPosInCG:uiCGSize-1) ; n >= 0; --n )
          {
            UInt uiBlkPos   = codingParameters.scan[ n + subPos ];
            if(piDstCoeff[ uiBlkPos ] != 0 )
            {
              Int64 costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos];
              Int64 costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos] 
                               -   ( abs(piDstCoeff[uiBlkPos])==1?((1<<15)+sigRateDelta[uiBlkPos]):0 );

              if(lastCG==1 && lastNZPosInCG==n && abs(piDstCoeff[uiBlkPos])==1)
              {
                costDown -= (4<<15);
              }

              if(costUp<costDown)
              {  
                curCost = costUp;
                curChange =  1;
              }
              else               
              {
                curChange = -1;
                if(n==firstNZPosInCG && abs(piDstCoeff[uiBlkPos])==1)
                {
                  curCost = MAX_INT64;
                }
                else
                {
                  curCost = costDown; 
                }
              }
            }
            else
            {
              curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<15) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ; 
              curChange = 1 ;

              if(n<firstNZPosInCG)
              {
                UInt thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
                if(thissignbit != signbit )
                {
                  curCost = MAX_INT64;
                }
              }
            }

            if( curCost<minCostInc)
            {
              minCostInc = curCost;
              finalChange = curChange;
              minPos = uiBlkPos;
            }
          }
          const Int scaledCoeffIdx = minPos<0 ? minPos : (Int)(getScalingListCoeffIdx(format, compID, minPos, uiWidth, uiHeight));
          if(piQCoef[scaledCoeffIdx] == TRANSFORM_MAXIMUM || piQCoef[scaledCoeffIdx] == TRANSFORM_MINIMUM)
          {
            finalChange = -1;
          }

          if(plSrcCoeff[minPos]>=0)
          {
            piDstCoeff[minPos] += finalChange ;
          }
          else
          {
            piDstCoeff[minPos] -= finalChange ; 
          }          
        }
      }

      if(lastCG==1)
      {
        lastCG=0 ;  
      }
    }
  }
}


/** Pattern decision for context derivation process of significant_coeff_flag
 * \param sigCoeffGroupFlag pointer to prior coded significant coeff group
 * \param uiCGPosX column of current coefficient group
 * \param uiCGPosY row of current coefficient group
 * \param width width of the block
 * \param height height of the block
 * \returns pattern for current coefficient group
 */
Int  TComTrQuant::calcPatternSigCtx( const UInt* sigCoeffGroupFlag, UInt uiCGPosX, UInt uiCGPosY, UInt widthInGroups, UInt heightInGroups )
{
  const Bool rightAvailable = uiCGPosX < (widthInGroups  - 1);
  const Bool belowAvailable = uiCGPosY < (heightInGroups - 1);

  UInt sigRight = 0;
  UInt sigLower = 0;

  if (rightAvailable) sigRight = ((sigCoeffGroupFlag[ (uiCGPosY * widthInGroups) + uiCGPosX + 1 ] != 0) ? 1 : 0);
  if (belowAvailable) sigLower = ((sigCoeffGroupFlag[ (uiCGPosY + 1) * widthInGroups + uiCGPosX ] != 0) ? 1 : 0);


  if (patternSigCtxCopyMissingGroupFromAvailableGroup())
  {
    if (!rightAvailable) sigRight = sigLower;
    if (!belowAvailable) sigLower = sigRight;
  }

  return sigRight + (sigLower << 1);
}


/** Context derivation process of coeff_abs_significant_flag
 * \param pcCoeff pointer to prior coded transform coefficients
 * \param posX column of current scan position
 * \param posY row of current scan position
 * \param blockType log2 value of block size if square block, or 4 otherwise
 * \param width width of the block
 * \param height height of the block
 * \param ChannelType channel type (CHANNEL_TYPE_LUMA/CHROMA)
 * \returns ctxInc for current scan position
 */
Int TComTrQuant::getSigCtxInc    (       Int                        patternSigCtx,
                                   const TUEntropyCodingParameters &codingParameters,
                                   const Int                        scanPosition,
                                   const Int                        log2BlockWidth,
                                   const Int                        log2BlockHeight,
                                   const ChannelType                chanType)
{
  const UInt rasterPosition = codingParameters.scan[scanPosition];
  const UInt posY           = rasterPosition >> log2BlockWidth;
  const UInt posX           = rasterPosition - (posY << log2BlockWidth);

  if ((posX + posY) == 0) return 0; //special case for the DC context variable

  Int offset = MAX_INT;

  if (codingParameters.useFixedGridSignificanceMapContext)
  {
    const TUEntropyCodingParameters::FixedGridContextParameters &gridParameters = codingParameters.fixedGridContextParameters;

    offset = gridParameters.grid[ (gridParameters.stride * (posY >> gridParameters.heightScale)) + (posX >> gridParameters.widthScale) ];
  }
  else
  {
    Int cnt = 0;

    switch (patternSigCtx)
    {
      //------------------

      case 0: //neither neighbouring group is significant
        {
#if REMOVAL_8x2_2x8_CG
          const Int posXinSubset     = posX & ((1 << codingParameters.log2GroupWidth) - 1);
          const Int posYinSubset     = posY & ((1 << codingParameters.log2GroupHeight) - 1);
          const Int posTotalInSubset = posXinSubset + posYinSubset;

          //first N coefficients in scan order use 2; the next few use 1; the rest use 0.
#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
          const UInt context1Threshold = codingParameters.neighbourhoodContextParameters.pattern00Context1Threshold;
          const UInt context2Threshold = codingParameters.neighbourhoodContextParameters.pattern00Context2Threshold;
#else
          const UInt context1Threshold = NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4;
          const UInt context2Threshold = NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4;
#endif

          cnt = (posTotalInSubset >= context1Threshold) ? 0 : ((posTotalInSubset >= context2Threshold) ? 1 : 2);
#else
          const Int log2GroupSize   = codingParameters.log2GroupWidth + codingParameters.log2GroupHeight;
          const Int posScanInSubset = scanPosition & ((1 << log2GroupSize) - 1);

          //first N coefficients in scan order use 1; the rest use 0.
#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
          const UInt contextThreshold = codingParameters.neighbourhoodContextParameters.pattern00ContextThreshold;
#else
          const UInt contextThreshold = NEIGHBOURHOOD_00_CONTEXT_THRESHOLD_4x4;
#endif

          cnt = (posScanInSubset >= contextThreshold) ? 0 : 1;
#endif
        }
        break;

      //------------------

      case 1: //right group is significant, below is not
        {
          const Int posYinSubset = posY & ((1 << codingParameters.log2GroupHeight) - 1);
          const Int groupHeight  = 1 << codingParameters.log2GroupHeight;

#if REMOVAL_8x2_2x8_CG
          cnt = (posYinSubset >= (groupHeight >> 1)) ? 0 : ((posYinSubset >= (groupHeight >> 2)) ? 1 : 2); //top quarter uses 2; second-from-top quarter uses 1; bottom half uses 0
#else
          cnt = (posYinSubset >= (groupHeight >> 1)) ? 0 : 1; //top half uses 1; bottom half uses 0
#endif
        }
        break;

      //------------------

      case 2: //below group is significant, right is not
        {
          const Int posXinSubset = posX & ((1 << codingParameters.log2GroupWidth) - 1);
          const Int groupWidth   = 1 << codingParameters.log2GroupWidth;

#if REMOVAL_8x2_2x8_CG
          cnt = (posXinSubset >= (groupWidth >> 1)) ? 0 : ((posXinSubset >= (groupWidth >> 2)) ? 1 : 2); //left quarter uses 2; second-from-left quarter uses 1; right half uses 0
#else
          cnt = (posXinSubset >= (groupWidth >> 1)) ? 0 : 1; //left half uses 1; right half uses 0
#endif
        }
        break;

      //------------------

      case 3: //both neighbouring groups are significant
        {
#if REMOVAL_8x2_2x8_CG
          cnt = 2;
#else
          const Int log2GroupSize   = codingParameters.log2GroupWidth + codingParameters.log2GroupHeight;
          const Int posScanInSubset = scanPosition & ((1 << log2GroupSize) - 1);

          //first N coefficients in scan order use 2; the rest use 1.
#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
          const UInt contextThreshold = codingParameters.neighbourhoodContextParameters.pattern11ContextThreshold;
#else
          const UInt contextThreshold = NEIGHBOURHOOD_11_CONTEXT_THRESHOLD_4x4;
#endif

          cnt = (posScanInSubset >= contextThreshold) ? 1 : 2;
#endif
        }
        break;

      //------------------

      default:
        std::cerr << "ERROR: Invalid patternSigCtx \"" << (int)patternSigCtx << "\" in getSigCtxInc" << std::endl;
        exit(1);
        break;
    }

    //------------------------------------------------

    const Bool notFirstGroup = ((posX >> codingParameters.log2GroupWidth) + (posY >> codingParameters.log2GroupHeight)) > 0;

    offset = (notFirstGroup ? notFirstGroupNeighbourhoodContextOffset[chanType] : 0) + cnt;
  }

  return codingParameters.firstSignificanceMapContext + offset;
}


/** Get the best level in RD sense
 * \param rd64CodedCost reference to coded cost
 * \param rd64CodedCost0 reference to cost when coefficient is 0
 * \param rd64CodedCostSig reference to cost of significant coefficient
 * \param lLevelDouble reference to unscaled quantized level
 * \param uiMaxAbsLevel scaled quantized level
 * \param ui16CtxNumSig current ctxInc for coeff_abs_significant_flag
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice current Rice parameter for coeff_abs_level_minus3
 * \param iQBits quantization step size
 * \param dTemp correction factor
 * \param bLast indicates if the coefficient is the last significant
 * \returns best quantized transform level for given scan position
 * This method calculates the best quantized transform level for a given scan position.
 */
__inline UInt TComTrQuant::xGetCodedLevel ( Double&                         rd64CodedCost,
                                            Double&                         rd64CodedCost0,
                                            Double&                         rd64CodedCostSig,
                                            Int                             lLevelDouble,
                                            UInt                            uiMaxAbsLevel,
                                            UShort                          ui16CtxNumSig,
                                            UShort                          ui16CtxNumOne,
                                            UShort                          ui16CtxNumAbs,
                                            UShort                          ui16AbsGoRice,
                                            UInt                            c1Idx,
                                            UInt                            c2Idx,
                                            Int                             iQBits,
                                            Double                          dTemp,
                                            Bool                            bLast        ) const
{
  Double dCurrCostSig   = 0; 
  UInt   uiBestAbsLevel = 0;
  
  if( !bLast && uiMaxAbsLevel < 3 )
  {
    rd64CodedCostSig    = xGetRateSigCoef( 0, ui16CtxNumSig ); 
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;
    if( uiMaxAbsLevel == 0 )
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( 1, ui16CtxNumSig );
  }

  UInt uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );
  for( Int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )
  {
    Double dErr         = Double( lLevelDouble  - ( uiAbsLevel << iQBits ) );
    Double dCurrCost    = dErr * dErr * dTemp + xGetICRateCost( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx );
    dCurrCost          += dCurrCostSig;

    if( dCurrCost < rd64CodedCost )
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \returns cost of given absolute transform level
 */
__inline Double TComTrQuant::xGetICRateCost  ( UInt                            uiAbsLevel,
                                               UShort                          ui16CtxNumOne,
                                               UShort                          ui16CtxNumAbs,
                                               UShort                          ui16AbsGoRice,
                                               UInt                            c1Idx,
                                               UInt                            c2Idx
                                               ) const
{
  Double iRate = xGetIEPRate();

  UInt baseLevel  =  (c1Idx < C1FLAG_NUMBER)? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

  if ( uiAbsLevel >= baseLevel )
  {
    UInt symbol     = uiAbsLevel - baseLevel;
    UInt length;
#if COEF_REMAIN_BIN_REDUCTION
    if (symbol < (COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice))
#else
    if (symbol < (8 << ui16AbsGoRice))
#endif
    {
      length = symbol>>ui16AbsGoRice;
      iRate += (length+1+ui16AbsGoRice)<< 15;
    }
    else
    {
      length = ui16AbsGoRice;
#if COEF_REMAIN_BIN_REDUCTION
      symbol  = symbol - ( COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice); 
#else
      symbol  = symbol - ( 8 << ui16AbsGoRice);    
#endif 
      while (symbol >= (1<<length))
      {
        symbol -=  (1<<(length++));    
      }
#if COEF_REMAIN_BIN_REDUCTION
      iRate += (COEF_REMAIN_BIN_REDUCTION+length+1-ui16AbsGoRice+length)<< 15;
#else
      iRate += (8+length+1-ui16AbsGoRice+length)<< 15;
#endif
    }

    if (c1Idx < C1FLAG_NUMBER)
    {
      iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];

      if (c2Idx < C2FLAG_NUMBER)
      {
        iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ];
      }
    }
  }
  else

  if( uiAbsLevel == 1 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 0 ];
  }
  else if( uiAbsLevel == 2 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];
    iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 0 ];
  }
  else
  {
    assert (0);
  }
  return xGetICost( iRate );
}


__inline Int TComTrQuant::xGetICRate  ( UInt                            uiAbsLevel,
                                       UShort                          ui16CtxNumOne,
                                       UShort                          ui16CtxNumAbs,
                                       UShort                          ui16AbsGoRice,
                                       UInt                            c1Idx,
                                       UInt                            c2Idx
                                       ) const
{
  Int iRate = 0;

  UInt baseLevel  =  (c1Idx < C1FLAG_NUMBER)? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

  if ( uiAbsLevel >= baseLevel )
  {
    UInt uiSymbol     = uiAbsLevel - baseLevel;
    UInt uiMaxVlc     = g_auiGoRiceRange[ ui16AbsGoRice ];
    Bool bExpGolomb   = ( uiSymbol > uiMaxVlc );

    if( bExpGolomb )
    {
      uiAbsLevel  = uiSymbol - uiMaxVlc;
      int iEGS    = 1;  for( UInt uiMax = 2; uiAbsLevel >= uiMax; uiMax <<= 1, iEGS += 2 );
      iRate      += iEGS << 15;
      uiSymbol    = min<UInt>( uiSymbol, ( uiMaxVlc + 1 ) );
    }

    UShort ui16PrefLen = UShort( uiSymbol >> ui16AbsGoRice ) + 1;
    UShort ui16NumBins = min<UInt>( ui16PrefLen, g_auiGoRicePrefixLen[ ui16AbsGoRice ] ) + ui16AbsGoRice;

    iRate += ui16NumBins << 15;

    if (c1Idx < C1FLAG_NUMBER)
    {
      iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];

      if (c2Idx < C2FLAG_NUMBER)
      {
        iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ];
      }
    }
  }
  else

  if( uiAbsLevel == 0 )
  {
    return 0;
  }
  else if( uiAbsLevel == 1 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 0 ];
  }
  else if( uiAbsLevel == 2 )
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];
    iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 0 ];
  }
  else
  {
    assert(0);
  }
  return iRate;
}

__inline Double TComTrQuant::xGetRateSigCoeffGroup  ( UShort                    uiSignificanceCoeffGroup,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantCoeffGroupBits[ ui16CtxNumSig ][ uiSignificanceCoeffGroup ] );
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
__inline Double TComTrQuant::xGetRateLast   ( const UInt                      uiPosX,
                                              const UInt                      uiPosY,
                                              const UInt                      uiBlkWdth,
                                              const ComponentID               component  ) const
{
  UInt uiCtxX   = g_uiGroupIdx[uiPosX];
  UInt uiCtxY   = g_uiGroupIdx[uiPosY];

#if   (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 2)
  Double uiCost = m_pcEstBitsSbac->lastXBits[component][ uiCtxX ] + m_pcEstBitsSbac->lastYBits[component][ uiCtxY ];
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 1)
  Double uiCost = m_pcEstBitsSbac->lastXBits[toChannelType(component)][ uiCtxX ] + m_pcEstBitsSbac->lastYBits[toChannelType(component)][ uiCtxY ];
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 0)
  Double uiCost = m_pcEstBitsSbac->lastXBits[ uiCtxX ] + m_pcEstBitsSbac->lastYBits[ uiCtxY ];
#endif
  
  if( uiCtxX > 3 )
  {
    uiCost += xGetIEPRate() * ((uiCtxX-2)>>1);
  }
  if( uiCtxY > 3 )
  {
    uiCost += xGetIEPRate() * ((uiCtxY-2)>>1);
  }
  return xGetICost( uiCost );
}

 /** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxBase current global offset for coeff_abs_level_greater1 and coeff_abs_level_greater2
 * \returns cost of given absolute transform level
 */
__inline Double TComTrQuant::xGetRateSigCoef  ( UShort                          uiSignificance,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantBits[ ui16CtxNumSig ][ uiSignificance ] );
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
__inline Double TComTrQuant::xGetICost        ( Double                          dRate         ) const
{
  return m_dLambda * dRate;
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
__inline Double TComTrQuant::xGetIEPRate      (                                               ) const
{
  return 32768;
}

/** Context derivation process of coeff_abs_significant_flag
 * \param uiSigCoeffGroupFlag significance map of L1
 * \param uiBlkX column of current scan position
 * \param uiBlkY row of current scan position
 * \param uiLog2BlkSize log2 value of block size
 * \returns ctxInc for current scan position
 */
UInt TComTrQuant::getSigCoeffGroupCtxInc  (const UInt*  uiSigCoeffGroupFlag,
                                           const UInt   uiCGPosX,
                                           const UInt   uiCGPosY,
                                           const UInt   widthInGroups,
                                           const UInt   heightInGroups)
{
  UInt sigRight = 0;
  UInt sigLower = 0;

  if (uiCGPosX < (widthInGroups  - 1)) sigRight = ((uiSigCoeffGroupFlag[ (uiCGPosY * widthInGroups) + uiCGPosX + 1 ] != 0) ? 1 : 0);
  if (uiCGPosY < (heightInGroups - 1)) sigLower = ((uiSigCoeffGroupFlag[ (uiCGPosY + 1) * widthInGroups + uiCGPosX ] != 0) ? 1 : 0);

  return ((sigRight + sigLower) != 0) ? 1 : 0;
}


/** set quantized matrix coefficient for encode
 * \param scalingList quantaized matrix address
 */
Void TComTrQuant::setScalingList(TComScalingList *scalingList, const ChromaFormat format)
{
  const Int minimumQp = -getQpRemTableIndexOffset();
  const Int maximumQp = SCALING_LIST_REM_NUM - getQpRemTableIndexOffset();

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < g_scalingListNum[size]; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListEnc(scalingList,list,size,qp,format);
        xSetScalingListDec(scalingList,list,size,qp,format);

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
        {
#if REMOVE_NSQT
          setErrScaleCoeff(list,size,qp,ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
#else
          setErrScaleCoeff(list,size,qp,SCALING_LIST_SQT,ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
          if(size == SCALING_LIST_32x32 || size == SCALING_LIST_16x16)
          {
            setErrScaleCoeff(list,size-1,qp,SCALING_LIST_HOR,ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
            setErrScaleCoeff(list,size-1,qp,SCALING_LIST_VER,ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
          }
#endif
        }
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 */
Void TComTrQuant::setScalingListDec(TComScalingList *scalingList, const ChromaFormat format)
{
  const Int minimumQp = -getQpRemTableIndexOffset();
  const Int maximumQp = SCALING_LIST_REM_NUM - getQpRemTableIndexOffset();

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < g_scalingListNum[size]; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListDec(scalingList,list,size,qp,format);
      }
    }
  }
}
/** set error scale coefficients
 * \param list List ID
 * \param uiSize Size
 * \param uiQP Quantization parameter
 */
#if REMOVE_NSQT
Void TComTrQuant::setErrScaleCoeff(UInt list,UInt size, Int qp, ErrorScaleAdjustmentMode errorScaleAdjustmentMode)
#else
Void TComTrQuant::setErrScaleCoeff(UInt list,UInt size, Int qp, ScalingListDIR dir, ErrorScaleAdjustmentMode errorScaleAdjustmentMode)
#endif
{
  UInt uiLog2TrSize = g_aucConvertToBit[ g_scalingListSizeX[size] ] + 2;
#if FULL_NBIT
  UInt uiBitDepth = g_uiBitDepth;
#else
  UInt uiBitDepth = g_uiBitDepth + g_uiBitIncrement;  
#endif

  Int iTransformShift = MAX_TR_DYNAMIC_RANGE - uiBitDepth - uiLog2TrSize;  // Represents scaling through forward transform

  UInt i,uiMaxNumCoeff = g_scalingListSize[size];
  Int *piQuantcoeff;
  double *pdErrScale;
#if REMOVE_NSQT
  piQuantcoeff   = getQuantCoeff(list, qp,size);
  pdErrScale     = getErrScaleCoeff(list, size, qp, errorScaleAdjustmentMode);
#else
  piQuantcoeff   = getQuantCoeff(list, qp,size,dir);
  pdErrScale     = getErrScaleCoeff(list, size, qp, dir, errorScaleAdjustmentMode);
#endif

  double dOffset = iTransformShift + ((errorScaleAdjustmentMode == ERROR_SCALE_ADJUSTMENT_MODE_422) ? 0.5 : 0.0);

  double dErrScale = (double)(1<<SCALE_BITS);                              // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale = dErrScale*pow(2.0,(-2.0*dOffset));                     // Compensate for scaling through forward transform

  for(i=0;i<uiMaxNumCoeff;i++)
  {
    pdErrScale[i] =  dErrScale/(double)piQuantcoeff[i]/(double)piQuantcoeff[i]/(double)(1<<(2*g_uiBitIncrement));
  }
}

/** set quantized matrix coefficient for encode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param uiQP Quantization parameter
 */
Void TComTrQuant::xSetScalingListEnc(TComScalingList *scalingList, UInt listId, UInt sizeId, Int qp, const ChromaFormat format)
{
  UInt width  = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio  = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *quantcoeff;
  Int *coeff  = scalingList->getScalingListAddress(sizeId,listId);
#if REMOVE_NSQT
  quantcoeff  = getQuantCoeff(listId, qp, sizeId);
#else
  quantcoeff  = getQuantCoeff(listId, qp, sizeId, SCALING_LIST_SQT);
#endif

  Int quantScales = getQuantScaling(qp, format);

  processScalingListEnc(coeff,quantcoeff,quantScales<<4,height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));

#if !REMOVE_NSQT
  if(sizeId == SCALING_LIST_32x32 || sizeId == SCALING_LIST_16x16) //for NSQT
  {
    quantcoeff   = getQuantCoeff(listId, qp, sizeId-1,SCALING_LIST_VER);
    processScalingListEnc(coeff,quantcoeff,quantScales<<4,height,width>>2,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));

    quantcoeff   = getQuantCoeff(listId, qp, sizeId-1,SCALING_LIST_HOR);
    processScalingListEnc(coeff,quantcoeff,quantScales<<4,height>>2,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
  }
#endif
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param list List index
 * \param size size index
 * \param uiQP Quantization parameter
 */
Void TComTrQuant::xSetScalingListDec(TComScalingList *scalingList, UInt listId, UInt sizeId, Int qp, const ChromaFormat format)
{
  UInt width  = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio  = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *dequantcoeff;
  Int *coeff  = scalingList->getScalingListAddress(sizeId,listId);

#if REMOVE_NSQT
  dequantcoeff = getDequantCoeff(listId, qp, sizeId);
#else
  dequantcoeff = getDequantCoeff(listId, qp, sizeId,SCALING_LIST_SQT);
#endif
  
  Int invQuantScale = getInverseQuantScaling(qp, format);

  processScalingListDec(coeff,dequantcoeff,invQuantScale,height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));


#if !REMOVE_NSQT
  if(sizeId == SCALING_LIST_32x32 || sizeId == SCALING_LIST_16x16)
  {
    dequantcoeff   = getDequantCoeff(listId, qp, sizeId-1,SCALING_LIST_VER);
    processScalingListDec(coeff,dequantcoeff,invQuantScale,height,width>>2,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));

    dequantcoeff   = getDequantCoeff(listId, qp, sizeId-1,SCALING_LIST_HOR);
    processScalingListDec(coeff,dequantcoeff,invQuantScale,height>>2,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
  }
#endif
}

/** set flat matrix value to quantized coefficient
 */
Void TComTrQuant::setFlatScalingList(const ChromaFormat format)
{
  const Int minimumQp = -getQpRemTableIndexOffset();
  const Int maximumQp = SCALING_LIST_REM_NUM - getQpRemTableIndexOffset();

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < g_scalingListNum[size]; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xsetFlatScalingList(list,size,qp,format);

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
        {
#if REMOVE_NSQT
          setErrScaleCoeff(list,size,qp, ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
#else
          setErrScaleCoeff(list,size,qp,SCALING_LIST_SQT, ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
          if(size == SCALING_LIST_32x32 || size == SCALING_LIST_16x16)
          {
            setErrScaleCoeff(list,size-1,qp,SCALING_LIST_HOR, ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
            setErrScaleCoeff(list,size-1,qp,SCALING_LIST_VER, ErrorScaleAdjustmentMode(errorScaleAdjustmentModeID));
          }
#endif
        }
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param uiQP Quantization parameter
 * \param uiSize Size
 */
Void TComTrQuant::xsetFlatScalingList(UInt list, UInt size, Int qp, const ChromaFormat format)
{
  UInt i,num = g_scalingListSize[size];
  Int *quantcoeff;
  Int *dequantcoeff;

  Int quantScales    = getQuantScaling       (qp, format);
  Int invQuantScales = getInverseQuantScaling(qp, format) << 4;

#if REMOVE_NSQT
  quantcoeff   = getQuantCoeff(list, qp, size);
  dequantcoeff = getDequantCoeff(list, qp, size);
#else
  quantcoeff   = getQuantCoeff(list, qp, size,SCALING_LIST_SQT);
  dequantcoeff = getDequantCoeff(list, qp, size,SCALING_LIST_SQT);
#endif

  for(i=0;i<num;i++)
  { 
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }

#if !REMOVE_NSQT
  const UInt numDiv4 = num>>2;
  if(size == SCALING_LIST_32x32 || size == SCALING_LIST_16x16)
  {
    quantcoeff   = getQuantCoeff(list, qp, size-1, SCALING_LIST_HOR);
    dequantcoeff = getDequantCoeff(list, qp, size-1, SCALING_LIST_HOR);

    for(i=0;i<numDiv4;i++)
    {
      *quantcoeff++ = quantScales;
      *dequantcoeff++ = invQuantScales;
    }
    quantcoeff   = getQuantCoeff(list, qp, size-1 ,SCALING_LIST_VER);
    dequantcoeff = getDequantCoeff(list, qp, size-1 ,SCALING_LIST_VER);

    for(i=0;i<numDiv4;i++)
    {
      *quantcoeff++ = quantScales;
      *dequantcoeff++ = invQuantScales;
    }
  }
#endif
}

/** set quantized matrix coefficient for encode
 * \param coeff quantaized matrix address
 * \param quantcoeff quantaized matrix address
 * \param quantScales Q(QP%6)
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
#if !REMOVE_NSQT
  Int nsqth = (height < width) ? 4: 1; //height ratio for NSQT
  Int nsqtw = (width < height) ? 4: 1; //width ratio for NSQT
#endif
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
#if REMOVE_NSQT
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j / ratio) + i / ratio];
#else
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j * nsqth / ratio) + i * nsqtw /ratio];
#endif
    }
  }

  if(ratio > 1)
  {
    quantcoeff[0] = quantScales / dc;
  }
}
/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListDec( Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
#if !REMOVE_NSQT
  Int nsqth = (height < width) ? 4: 1; //height ratio for NSQT
  Int nsqtw = (width < height) ? 4: 1; //width ratio for NSQT
#endif
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
#if REMOVE_NSQT
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
#else
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j * nsqth / ratio) + i * nsqtw /ratio];
#endif
    }
  }

  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
Void TComTrQuant::initScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    const UInt lastSourceEntry = g_scalingListNum[sizeId]-1;

    for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
    {
      for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
      {
#if REMOVE_NSQT
        m_quantCoef   [sizeId][listId][qp] = new Int [g_scalingListSize[sizeId]];
        m_dequantCoef [sizeId][listId][qp] = new Int [g_scalingListSize[sizeId]];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          m_errScale[sizeId][listId][qp][errorScaleAdjustmentModeID] = new double [g_scalingListSize[sizeId]];
#else
        m_quantCoef   [sizeId][listId][qp][SCALING_LIST_SQT] = new Int [g_scalingListSize[sizeId]];
        m_dequantCoef [sizeId][listId][qp][SCALING_LIST_SQT] = new Int [g_scalingListSize[sizeId]];
  
        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          m_errScale[sizeId][listId][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID] = new double [g_scalingListSize[sizeId]];
        
#if ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA != 1
        if(sizeId == SCALING_LIST_8x8 || (sizeId == SCALING_LIST_16x16 && listId < 2))
#else
        if(sizeId == SCALING_LIST_8x8 || (sizeId == SCALING_LIST_16x16 ))
#endif
        {
          // considering only SCALING_LIST_8x8   --> NSQT VER/HOR 16x4 / 4x16 Y, Cb, Cr Inter & Intra (!)
          // and              SCALING_LIST_16x16 --> NSQT VER/HOR 8x32 / 32x8 Y, Inter & Intra (!)  - reassigned later!
          for(UInt dir = SCALING_LIST_VER; dir < SCALING_LIST_DIR_NUM; dir++)
          {
            m_quantCoef   [sizeId][listId][qp][dir] = new Int [g_scalingListSize[sizeId]];
            m_dequantCoef [sizeId][listId][qp][dir] = new Int [g_scalingListSize[sizeId]];

            for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
              m_errScale[sizeId][listId][qp][dir][errorScaleAdjustmentModeID] = new double [g_scalingListSize[sizeId]];
          }
        }
#endif
      } // listID loop
      if (g_scalingListNum[sizeId] < 2*MAX_NUM_COMPONENT)
      {
        // make the first entry for inter be the second intra entry...
#if REMOVE_NSQT
        m_quantCoef   [sizeId][MAX_NUM_COMPONENT][qp] = m_quantCoef   [sizeId][lastSourceEntry][qp];
        m_dequantCoef [sizeId][MAX_NUM_COMPONENT][qp] = m_dequantCoef [sizeId][lastSourceEntry][qp];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          m_errScale[sizeId][MAX_NUM_COMPONENT][qp][errorScaleAdjustmentModeID] = m_errScale    [sizeId][lastSourceEntry][qp][errorScaleAdjustmentModeID];
#else
        m_quantCoef   [sizeId][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT] = m_quantCoef   [sizeId][lastSourceEntry][qp][SCALING_LIST_SQT];
        m_dequantCoef [sizeId][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT] = m_dequantCoef [sizeId][lastSourceEntry][qp][SCALING_LIST_SQT];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          m_errScale[sizeId][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID] = m_errScale    [sizeId][lastSourceEntry][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID];
#endif
      }
    }
  }
#if !REMOVE_NSQT
#if ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA != 1
  {
    UInt lastSourceEntry=1;
    // Duplicate pointers for the cases where there is no chroma scaling list.
    for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
    {
      // Reassigned luma for SCALING_LIST_16x16 --> NSQT VER/HOR 8x32 / 32x8 Y, Inter & Intra (!)
      for(UInt dir = SCALING_LIST_VER; dir < SCALING_LIST_DIR_NUM; dir++)
      {
        m_quantCoef   [SCALING_LIST_16x16][MAX_NUM_COMPONENT][qp][dir] = m_quantCoef   [SCALING_LIST_16x16][lastSourceEntry][qp][dir];
        m_dequantCoef [SCALING_LIST_16x16][MAX_NUM_COMPONENT][qp][dir] = m_dequantCoef [SCALING_LIST_16x16][lastSourceEntry][qp][dir];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          m_errScale[SCALING_LIST_16x16][MAX_NUM_COMPONENT][qp][dir][errorScaleAdjustmentModeID] = m_errScale    [SCALING_LIST_16x16][lastSourceEntry][qp][dir][errorScaleAdjustmentModeID];
      }
      m_quantCoef   [SCALING_LIST_32x32][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT] = m_quantCoef   [SCALING_LIST_32x32][lastSourceEntry][qp][SCALING_LIST_SQT];
      m_dequantCoef [SCALING_LIST_32x32][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT] = m_dequantCoef [SCALING_LIST_32x32][lastSourceEntry][qp][SCALING_LIST_SQT];

      for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
        m_errScale[SCALING_LIST_32x32][MAX_NUM_COMPONENT][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID] = m_errScale    [SCALING_LIST_32x32][lastSourceEntry][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID];
    }
  }
#endif
#endif
}
/** destroy quantization matrix array
 */
Void TComTrQuant::destroyScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
#if REMOVE_NSQT
        if(m_quantCoef   [sizeId][listId][qp]) delete [] m_quantCoef   [sizeId][listId][qp];
        if(m_dequantCoef [sizeId][listId][qp]) delete [] m_dequantCoef [sizeId][listId][qp];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          if(m_errScale[sizeId][listId][qp][errorScaleAdjustmentModeID]) delete [] m_errScale[sizeId][listId][qp][errorScaleAdjustmentModeID];
#else
        if(m_quantCoef   [sizeId][listId][qp][SCALING_LIST_SQT]) delete [] m_quantCoef   [sizeId][listId][qp][SCALING_LIST_SQT];
        if(m_dequantCoef [sizeId][listId][qp][SCALING_LIST_SQT]) delete [] m_dequantCoef [sizeId][listId][qp][SCALING_LIST_SQT];

        for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
          if(m_errScale[sizeId][listId][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID]) delete [] m_errScale[sizeId][listId][qp][SCALING_LIST_SQT][errorScaleAdjustmentModeID];

#if ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA != 1
        if(sizeId == SCALING_LIST_8x8 || (sizeId == SCALING_LIST_16x16 && listId < 2))
#else
        if(sizeId == SCALING_LIST_8x8 || (sizeId == SCALING_LIST_16x16 ))
#endif
        {
          for(UInt dir = SCALING_LIST_VER; dir < SCALING_LIST_DIR_NUM; dir++)
          {
            if(m_quantCoef   [sizeId][listId][qp][dir]) delete [] m_quantCoef   [sizeId][listId][qp][dir];
            if(m_dequantCoef [sizeId][listId][qp][dir]) delete [] m_dequantCoef [sizeId][listId][qp][dir];

            for (UInt errorScaleAdjustmentModeID = 0; errorScaleAdjustmentModeID < NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES; errorScaleAdjustmentModeID++)
              if(m_errScale[sizeId][listId][qp][dir][errorScaleAdjustmentModeID]) delete [] m_errScale[sizeId][listId][qp][dir][errorScaleAdjustmentModeID];
          }
        }
#endif
      }
    }
  }
}

//! \}
