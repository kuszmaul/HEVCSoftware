/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
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
#include <limits>
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
  // NOTE: RExt - does this work with negative QPs or when some blocks are transquant-bypass enabled?

  Int qpBase = pcSlice->getSliceQpBase();
  Int sliceQpused = pcSlice->getSliceQp();
  Int sliceQpnext;
  Double alpha = qpBase < 17 ? 0.5 : 1;

  Int cnt=0;
  for(Int u=1; u<=LEVEL_RANGE; u++)
  {
    cnt += m_sliceNsamples[u] ;
  }

  if( !m_useRDOQ )
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
void xTr(Int bitDepth, Pel *block, TCoeff *coeff, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxTrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;
  UInt uiLog2TrSize = g_aucConvertToBit[ uiTrSize ] + 2;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[0] : g_aiT4[0]);
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

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
  static const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
#endif

  const Int shift_1st = (uiLog2TrSize +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxTrDynamicRange;
  const Int shift_2nd = uiLog2TrSize + TRANSFORM_MATRIX_SHIFT;
  const Int add_1st = (shift_1st>0) ? (1<<(shift_1st-1)) : 0;
  const Int add_2nd = 1<<(shift_2nd-1);

  /* Horizontal transform */

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
void xITr(Int bitDepth, TCoeff *coeff, Pel *block, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxTrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[0] : g_aiT4[0]);
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

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
  static const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
#endif

  const Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  const Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxTrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxTrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxTrDynamicRange) - 1;
  assert(shift_2nd>=0);
  const Int add_1st = 1<<(shift_1st-1);
  const Int add_2nd = (shift_2nd>0) ? (1<<(shift_2nd-1)) : 0;

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

      //NOTE: RExt - Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
      tmp[i*uiTrSize+j] = Clip3<TCoeff>(clipMinimum, clipMaximum, (iSum + add_1st)>>shift_1st);
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

      block[i*uiStride+j] = Clip3<TCoeff>(std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), (iSum + add_1st)>>shift_1st);
    }
  }
}

#endif //MATRIX_MULT


/** 4x4 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly4(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    dst[0]      = (g_aiT4[TRANSFORM_FORWARD][0][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[TRANSFORM_FORWARD][2][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][2][1]*E[1] + add)>>shift;
    dst[line]   = (g_aiT4[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][3][1]*O[1] + add)>>shift;
#else
    dst[0]      = (g_aiT4[0][0]*E[0] + g_aiT4[0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[2][0]*E[0] + g_aiT4[2][1]*E[1] + add)>>shift;
    dst[line]   = (g_aiT4[1][0]*O[0] + g_aiT4[1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[3][0]*O[0] + g_aiT4[3][1]*O[1] + add)>>shift;
#endif

    src += 4;
    dst ++;
  }
}

// Fast DST Algorithm. Full matrix multiplication for DST and Fast DST algorithm
// give identical results
void fastForwardDst(TCoeff *block, TCoeff *coeff, Int shift)  // input block, output coeff
{
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    // Intermediate Variables
    c[0] = block[4*i+0];
    c[1] = block[4*i+1];
    c[2] = block[4*i+2];
    c[3] = block[4*i+3];

    for (Int row = 0; row < 4; row++)
    {
      TCoeff result = 0;
      for (int column = 0; column < 4; column++)
        result += c[column] * g_as_DST_MAT_4[TRANSFORM_FORWARD][row][column]; // use the defined matrix, rather than hard-wired numbers

      coeff[(row * 4) + i] = rightShift((result + rnd_factor), shift);
    }
#else
    // Intermediate Variables
    c[0] = block[4*i+0] + block[4*i+3];
    c[1] = block[4*i+1] + block[4*i+3];
    c[2] = block[4*i+0] - block[4*i+1];
    c[3] = 74* block[4*i+2];

    coeff[   i] =  ( 29 * c[0] + 55 * c[1]         + c[3]               + rnd_factor ) >> shift;
    coeff[ 4+i] =  ( 74 * (block[4*i+0]+ block[4*i+1] - block[4*i+3])   + rnd_factor ) >> shift;
    coeff[ 8+i] =  ( 29 * c[2] + 55 * c[0]         - c[3]               + rnd_factor ) >> shift;
    coeff[12+i] =  ( 55 * c[2] - 29 * c[1]         + c[3]               + rnd_factor ) >> shift;
#endif
  }
}

void fastInverseDst(TCoeff *tmp, TCoeff *block, Int shift, const TCoeff outputMinimum, const TCoeff outputMaximum)  // input tmp, output block
{
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    // Intermediate Variables
    c[0] = tmp[   i];
    c[1] = tmp[4 +i];
    c[2] = tmp[8 +i];
    c[3] = tmp[12+i];

    for (Int column = 0; column < 4; column++)
    {
      TCoeff &result = block[(i * 4) + column];

      result = 0;
      for (int row = 0; row < 4; row++)
        result += c[row] * g_as_DST_MAT_4[TRANSFORM_INVERSE][row][column]; // use the defined matrix, rather than hard-wired numbers

      result = Clip3( outputMinimum, outputMaximum, rightShift((result + rnd_factor), shift));
    }
#else
    // Intermediate Variables
    c[0] = tmp[  i] + tmp[ 8+i];
    c[1] = tmp[8+i] + tmp[12+i];
    c[2] = tmp[  i] - tmp[12+i];
    c[3] = 74* tmp[4+i];

    block[4*i+0] = Clip3( outputMinimum, outputMaximum, ( 29 * c[0] + 55 * c[1]     + c[3]      + rnd_factor ) >> shift );
    block[4*i+1] = Clip3( outputMinimum, outputMaximum, ( 55 * c[2] - 29 * c[1]     + c[3]      + rnd_factor ) >> shift );
    block[4*i+2] = Clip3( outputMinimum, outputMaximum, ( 74 * (tmp[i] - tmp[8+i]  + tmp[12+i]) + rnd_factor ) >> shift );
    block[4*i+3] = Clip3( outputMinimum, outputMaximum, ( 55 * c[0] + 29 * c[2]     - c[3]      + rnd_factor ) >> shift );
#endif
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse4(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    O[0] = g_aiT4[TRANSFORM_INVERSE][1][0]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][0]*src[3*line];
    O[1] = g_aiT4[TRANSFORM_INVERSE][1][1]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][1]*src[3*line];
    E[0] = g_aiT4[TRANSFORM_INVERSE][0][0]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][0]*src[2*line];
    E[1] = g_aiT4[TRANSFORM_INVERSE][0][1]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][1]*src[2*line];
#else
    O[0] = g_aiT4[1][0]*src[line] + g_aiT4[3][0]*src[3*line];
    O[1] = g_aiT4[1][1]*src[line] + g_aiT4[3][1]*src[3*line];
    E[0] = g_aiT4[0][0]*src[0]    + g_aiT4[2][0]*src[2*line];
    E[1] = g_aiT4[0][1]*src[0]    + g_aiT4[2][1]*src[2*line];
#endif

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( outputMinimum, outputMaximum, (E[0] + O[0] + add)>>shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, (E[1] + O[1] + add)>>shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, (E[1] - O[1] + add)>>shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, (E[0] - O[0] + add)>>shift );

    src   ++;
    dst += 4;
  }
}

/** 8x8 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterfly8(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

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

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    dst[0]      = (g_aiT8[TRANSFORM_FORWARD][0][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[TRANSFORM_FORWARD][4][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][4][1]*EE[1] + add)>>shift;
    dst[2*line] = (g_aiT8[TRANSFORM_FORWARD][2][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[TRANSFORM_FORWARD][6][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][6][1]*EO[1] + add)>>shift;

    dst[line]   = (g_aiT8[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][1][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][1][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][3][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][3][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[TRANSFORM_FORWARD][5][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][5][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][5][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[TRANSFORM_FORWARD][7][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][7][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][7][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][7][3]*O[3] + add)>>shift;
#else
    dst[0]      = (g_aiT8[0][0]*EE[0] + g_aiT8[0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[4][0]*EE[0] + g_aiT8[4][1]*EE[1] + add)>>shift;
    dst[2*line] = (g_aiT8[2][0]*EO[0] + g_aiT8[2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[6][0]*EO[0] + g_aiT8[6][1]*EO[1] + add)>>shift;

    dst[line]   = (g_aiT8[1][0]*O[0] + g_aiT8[1][1]*O[1] + g_aiT8[1][2]*O[2] + g_aiT8[1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[3][0]*O[0] + g_aiT8[3][1]*O[1] + g_aiT8[3][2]*O[2] + g_aiT8[3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[5][0]*O[0] + g_aiT8[5][1]*O[1] + g_aiT8[5][2]*O[2] + g_aiT8[5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[7][0]*O[0] + g_aiT8[7][1]*O[1] + g_aiT8[7][2]*O[2] + g_aiT8[7][3]*O[3] + add)>>shift;
#endif

    src += 8;
    dst ++;
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse8(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[TRANSFORM_INVERSE][ 1][k]*src[line]   + g_aiT8[TRANSFORM_INVERSE][ 3][k]*src[3*line] +
             g_aiT8[TRANSFORM_INVERSE][ 5][k]*src[5*line] + g_aiT8[TRANSFORM_INVERSE][ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[TRANSFORM_INVERSE][2][0]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][0]*src[ 6*line ];
    EO[1] = g_aiT8[TRANSFORM_INVERSE][2][1]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][1]*src[ 6*line ];
    EE[0] = g_aiT8[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][0]*src[ 4*line ];
    EE[1] = g_aiT8[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][1]*src[ 4*line ];
#else
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[ 1][k]*src[line] + g_aiT8[ 3][k]*src[3*line] + g_aiT8[ 5][k]*src[5*line] + g_aiT8[ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[2][0]*src[ 2*line ] + g_aiT8[6][0]*src[ 6*line ];
    EO[1] = g_aiT8[2][1]*src[ 2*line ] + g_aiT8[6][1]*src[ 6*line ];
    EE[0] = g_aiT8[0][0]*src[ 0      ] + g_aiT8[4][0]*src[ 4*line ];
    EE[1] = g_aiT8[0][1]*src[ 0      ] + g_aiT8[4][1]*src[ 4*line ];
#endif

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (k=0;k<4;k++)
    {
      dst[ k   ] = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[ k+4 ] = Clip3( outputMinimum, outputMaximum, (E[3-k] - O[3-k] + add)>>shift );
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
void partialButterfly16(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

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

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    dst[ 0      ] = (g_aiT16[TRANSFORM_FORWARD][ 0][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 0][1]*EEE[1] + add)>>shift;
    dst[ 8*line ] = (g_aiT16[TRANSFORM_FORWARD][ 8][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 8][1]*EEE[1] + add)>>shift;
    dst[ 4*line ] = (g_aiT16[TRANSFORM_FORWARD][ 4][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][ 4][1]*EEO[1] + add)>>shift;
    dst[ 12*line] = (g_aiT16[TRANSFORM_FORWARD][12][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][12][1]*EEO[1] + add)>>shift;

    for (k=2;k<16;k+=4)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*EO[3] + add)>>shift;
    }

    for (k=1;k<16;k+=2)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*O[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*O[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*O[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*O[3] +
                       g_aiT16[TRANSFORM_FORWARD][k][4]*O[4] + g_aiT16[TRANSFORM_FORWARD][k][5]*O[5] +
                       g_aiT16[TRANSFORM_FORWARD][k][6]*O[6] + g_aiT16[TRANSFORM_FORWARD][k][7]*O[7] + add)>>shift;
    }
#else
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
#endif

    src += 16;
    dst ++;

  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse16(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    for (k=0;k<8;k++)
    {
      O[k] = g_aiT16[TRANSFORM_INVERSE][ 1][k]*src[ line]   + g_aiT16[TRANSFORM_INVERSE][ 3][k]*src[ 3*line] +
             g_aiT16[TRANSFORM_INVERSE][ 5][k]*src[ 5*line] + g_aiT16[TRANSFORM_INVERSE][ 7][k]*src[ 7*line] +
             g_aiT16[TRANSFORM_INVERSE][ 9][k]*src[ 9*line] + g_aiT16[TRANSFORM_INVERSE][11][k]*src[11*line] +
             g_aiT16[TRANSFORM_INVERSE][13][k]*src[13*line] + g_aiT16[TRANSFORM_INVERSE][15][k]*src[15*line];
    }
    for (k=0;k<4;k++)
    {
      EO[k] = g_aiT16[TRANSFORM_INVERSE][ 2][k]*src[ 2*line] + g_aiT16[TRANSFORM_INVERSE][ 6][k]*src[ 6*line] +
              g_aiT16[TRANSFORM_INVERSE][10][k]*src[10*line] + g_aiT16[TRANSFORM_INVERSE][14][k]*src[14*line];
    }
    EEO[0] = g_aiT16[TRANSFORM_INVERSE][4][0]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][0]*src[ 12*line ];
    EEE[0] = g_aiT16[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][0]*src[ 8*line  ];
    EEO[1] = g_aiT16[TRANSFORM_INVERSE][4][1]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][1]*src[ 12*line ];
    EEE[1] = g_aiT16[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][1]*src[ 8*line  ];
#else
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
#endif

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
      dst[k]   = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+8] = Clip3( outputMinimum, outputMaximum, (E[7-k] - O[7-k] + add)>>shift );
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
void partialButterfly32(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

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

#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    dst[ 0       ] = (g_aiT32[TRANSFORM_FORWARD][ 0][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][ 0][1]*EEEE[1] + add)>>shift;
    dst[ 16*line ] = (g_aiT32[TRANSFORM_FORWARD][16][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][16][1]*EEEE[1] + add)>>shift;
    dst[ 8*line  ] = (g_aiT32[TRANSFORM_FORWARD][ 8][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][ 8][1]*EEEO[1] + add)>>shift;
    dst[ 24*line ] = (g_aiT32[TRANSFORM_FORWARD][24][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][24][1]*EEEO[1] + add)>>shift;
    for (k=4;k<32;k+=8)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EEO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EEO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EEO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EEO[3] + add)>>shift;
    }
    for (k=2;k<32;k+=4)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EO[3] +
                       g_aiT32[TRANSFORM_FORWARD][k][4]*EO[4] + g_aiT32[TRANSFORM_FORWARD][k][5]*EO[5] +
                       g_aiT32[TRANSFORM_FORWARD][k][6]*EO[6] + g_aiT32[TRANSFORM_FORWARD][k][7]*EO[7] + add)>>shift;
    }
    for (k=1;k<32;k+=2)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][ 0]*O[ 0] + g_aiT32[TRANSFORM_FORWARD][k][ 1]*O[ 1] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 2]*O[ 2] + g_aiT32[TRANSFORM_FORWARD][k][ 3]*O[ 3] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 4]*O[ 4] + g_aiT32[TRANSFORM_FORWARD][k][ 5]*O[ 5] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 6]*O[ 6] + g_aiT32[TRANSFORM_FORWARD][k][ 7]*O[ 7] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 8]*O[ 8] + g_aiT32[TRANSFORM_FORWARD][k][ 9]*O[ 9] +
                       g_aiT32[TRANSFORM_FORWARD][k][10]*O[10] + g_aiT32[TRANSFORM_FORWARD][k][11]*O[11] +
                       g_aiT32[TRANSFORM_FORWARD][k][12]*O[12] + g_aiT32[TRANSFORM_FORWARD][k][13]*O[13] +
                       g_aiT32[TRANSFORM_FORWARD][k][14]*O[14] + g_aiT32[TRANSFORM_FORWARD][k][15]*O[15] + add)>>shift;
    }
#else
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
#endif
    src += 32;
    dst ++;
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 */
void partialButterflyInverse32(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
    for (k=0;k<16;k++)
    {
      O[k] = g_aiT32[TRANSFORM_INVERSE][ 1][k]*src[ line    ] + g_aiT32[TRANSFORM_INVERSE][ 3][k]*src[ 3*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 5][k]*src[ 5*line  ] + g_aiT32[TRANSFORM_INVERSE][ 7][k]*src[ 7*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 9][k]*src[ 9*line  ] + g_aiT32[TRANSFORM_INVERSE][11][k]*src[ 11*line ] +
             g_aiT32[TRANSFORM_INVERSE][13][k]*src[ 13*line ] + g_aiT32[TRANSFORM_INVERSE][15][k]*src[ 15*line ] +
             g_aiT32[TRANSFORM_INVERSE][17][k]*src[ 17*line ] + g_aiT32[TRANSFORM_INVERSE][19][k]*src[ 19*line ] +
             g_aiT32[TRANSFORM_INVERSE][21][k]*src[ 21*line ] + g_aiT32[TRANSFORM_INVERSE][23][k]*src[ 23*line ] +
             g_aiT32[TRANSFORM_INVERSE][25][k]*src[ 25*line ] + g_aiT32[TRANSFORM_INVERSE][27][k]*src[ 27*line ] +
             g_aiT32[TRANSFORM_INVERSE][29][k]*src[ 29*line ] + g_aiT32[TRANSFORM_INVERSE][31][k]*src[ 31*line ];
    }
    for (k=0;k<8;k++)
    {
      EO[k] = g_aiT32[TRANSFORM_INVERSE][ 2][k]*src[ 2*line  ] + g_aiT32[TRANSFORM_INVERSE][ 6][k]*src[ 6*line  ] +
              g_aiT32[TRANSFORM_INVERSE][10][k]*src[ 10*line ] + g_aiT32[TRANSFORM_INVERSE][14][k]*src[ 14*line ] +
              g_aiT32[TRANSFORM_INVERSE][18][k]*src[ 18*line ] + g_aiT32[TRANSFORM_INVERSE][22][k]*src[ 22*line ] +
              g_aiT32[TRANSFORM_INVERSE][26][k]*src[ 26*line ] + g_aiT32[TRANSFORM_INVERSE][30][k]*src[ 30*line ];
    }
    for (k=0;k<4;k++)
    {
      EEO[k] = g_aiT32[TRANSFORM_INVERSE][ 4][k]*src[  4*line ] + g_aiT32[TRANSFORM_INVERSE][12][k]*src[ 12*line ] +
               g_aiT32[TRANSFORM_INVERSE][20][k]*src[ 20*line ] + g_aiT32[TRANSFORM_INVERSE][28][k]*src[ 28*line ];
    }
    EEEO[0] = g_aiT32[TRANSFORM_INVERSE][8][0]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][0]*src[ 24*line ];
    EEEO[1] = g_aiT32[TRANSFORM_INVERSE][8][1]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][1]*src[ 24*line ];
    EEEE[0] = g_aiT32[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][0]*src[ 16*line ];
    EEEE[1] = g_aiT32[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][1]*src[ 16*line ];
#else
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
#endif

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
      dst[k]    = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+16] = Clip3( outputMinimum, outputMaximum, (E[15-k] - O[15-k] + add)>>shift );
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
void xTrMxN(Int bitDepth, TCoeff *block, TCoeff *coeff, Int iWidth, Int iHeight, Bool useDST, const Int maxTrDynamicRange)
{
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
  static const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
#endif

  const Int shift_1st = ((g_aucConvertToBit[iWidth] + 2) +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxTrDynamicRange;
  const Int shift_2nd = (g_aucConvertToBit[iHeight] + 2) + TRANSFORM_MATRIX_SHIFT;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[ MAX_TU_SIZE * MAX_TU_SIZE ];

  switch (iWidth)
  {
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
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
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
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
void xITrMxN(Int bitDepth, TCoeff *coeff, TCoeff *block, Int iWidth, Int iHeight, Bool useDST, const Int maxTrDynamicRange)
{
#if RExt__INDEPENDENT_FORWARD_AND_INVERSE_TRANSFORMS
  static const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
#endif

  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxTrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxTrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxTrDynamicRange) - 1;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];

  switch (iHeight)
  {
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( coeff, tmp, shift_1st, clipMinimum, clipMaximum);
        }
        else partialButterflyInverse4 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum);
      }
      break;

    case  8: partialButterflyInverse8 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 16: partialButterflyInverse16( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 32: partialButterflyInverse32( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;

    default:
      assert(0); exit (1); break;
  }

  switch (iWidth)
  {
    //NOTE: RExt - Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( tmp, block, shift_2nd, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
        }
        else partialButterflyInverse4 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max());
      }
      break;

    case  8: partialButterflyInverse8 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 16: partialButterflyInverse16( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 32: partialButterflyInverse32( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;

    default:
      assert(0); exit (1); break;
  }
}


// To minimize the distortion only. No rate is considered.
Void TComTrQuant::signBitHidingHDQ( const ComponentID compID, TCoeff* pQCoef, TCoeff* pCoef, TCoeff* deltaU, const TUEntropyCodingParameters &codingParameters )
{
  const UInt width     = codingParameters.widthInGroups  << MLS_CG_LOG2_WIDTH;
  const UInt height    = codingParameters.heightInGroups << MLS_CG_LOG2_HEIGHT;
  const UInt groupSize = 1 << MLS_CG_SIZE;

  const TCoeff entropyCodingMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff entropyCodingMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> MLS_CG_SIZE; subSet >= 0; subSet-- )
  {
    Int  subPos = subSet << MLS_CG_SIZE;
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
      absSum += Int(pQCoef[ codingParameters.scan[ n + subPos ]]); //NOTE: RExt - only one bit is actually required!
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
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        Int minPos =-1, finalChange=0, curChange=0;

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
                curCost = std::numeric_limits<TCoeff>::max();
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
                curCost = std::numeric_limits<TCoeff>::max();
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

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)
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
                                TCoeff       &uiAcSum,
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

  const Bool useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);

  Bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;
  if ( useRDOQ && (isLuma(compID) || RDOQ_CHROMA) )
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
    getTUEntropyCodingParameters(codingParameters, rTu, compID);

    const TCoeff entropyCodingMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
    const TCoeff entropyCodingMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

    TCoeff deltaU[MAX_TU_SIZE * MAX_TU_SIZE];

    const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

    Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
    assert(scalingListType < SCALING_LIST_NUM);
    Int *piQuantCoeff = getQuantCoeff(scalingListType,cQP.getAdjustedQp().rem,uiLog2TrSize-2);

    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */

    // Represents scaling through forward transform
    Int iTransformShift = getTransformShift(toChannelType(compID), uiLog2TrSize);
    if (useTransformSkip && pcCU->getSlice()->getSPS()->getUseExtendedPrecision())
    {
      iTransformShift = std::max<Int>(0, iTransformShift);
    }

    const Int iQBits = QUANT_SHIFT + cQP.getAdjustedQp().per + iTransformShift;
    // NOTE: RExt - QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

#if ADAPTIVE_QP_SELECTION
    Int iQBitsC = MAX_INT;
    Int iAddC   = MAX_INT;

    if (m_bUseAdaptQpSelect)
    {
      iQBitsC = iQBits - ARL_C_PRECISION;
      iAddC   = 1 << (iQBitsC-1);
    }
#endif

    Int iAdd = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);

    Int qBits8 = iQBits-8;

    for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )
    {
      TCoeff iLevel;
      Int    iSign;
      iLevel  = piCoef[uiBlockPos];
      iSign   = (iLevel < 0 ? -1: 1);


#if RExt__SQUARE_TRANSFORM_CHROMA_422
      const UInt scalingListCoeffIdx = uiBlockPos;
#else
      const UInt scalingListCoeffIdx = getScalingListCoeffIdx(chFmt, compID, uiBlockPos, uiWidth, uiHeight);
#endif
      Int64 tmpLevel = (Int64)abs(iLevel) * piQuantCoeff[scalingListCoeffIdx];

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlCCoef[uiBlockPos] = (TCoeff)((tmpLevel + iAddC ) >> iQBitsC);
      }
#endif

      iLevel = (TCoeff)((tmpLevel + iAdd ) >> iQBits);
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel - (iLevel<<iQBits) )>> qBits8);

      uiAcSum += iLevel;
      iLevel *= iSign;

      piQCoef[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, iLevel );
    } // for n

    if( pcCU->getSlice()->getPPS()->getSignHideFlag() )
    {
      if(uiAcSum>=2)
      {
        signBitHidingHDQ( compID, piQCoef, piCoef, deltaU, codingParameters ) ;
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
#if !RExt__SQUARE_TRANSFORM_CHROMA_422
  const ChromaFormat fmt=rTu.GetChromaFormat();
#endif

  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);
  const UInt numSamplesInBlock=uiWidth*uiHeight;
  assert(compID<MAX_NUM_COMPONENT);

  const TCoeff transformMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff transformMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
  assert(scalingListType < SCALING_LIST_NUM);
  assert ( uiWidth <= m_uiMaxTrSize );


  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(toChannelType(compID), uiLog2TrSize);
  if ((pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0) && pcCU->getSlice()->getSPS()->getUseExtendedPrecision())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Int QP_per = cQP.getAdjustedQp().per;
  const Int QP_rem = cQP.getAdjustedQp().rem;

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (getUseScalingList() ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  TCoeff clipQCoef;
  Intermediate_Int iCoeffQ;

  if(getUseScalingList())
  {
    //from the dequantisation equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
#if RExt__SQUARE_TRANSFORM_CHROMA_422
        const Int deQuantIdx = n;
#else
        const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);
#endif

        clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
#if RExt__SQUARE_TRANSFORM_CHROMA_422
        const Int deQuantIdx = n;
#else
        const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, n, uiWidth, uiHeight);
#endif

        clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) << leftShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
    const Int scale     =  g_invQuantScales[QP_rem];
    const Int scaleBits =     (IQUANT_SHIFT + 1)   ;

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
}


Void TComTrQuant::init(   UInt  uiMaxTrSize,
                          Bool  bUseRDOQ,
                          Bool  bUseRDOQTS,  
                          Bool  bEnc,
                          Bool  useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                        , Bool bUseAdaptQpSelect
#endif
                       )
{
  m_uiMaxTrSize  = uiMaxTrSize;
  m_bEnc         = bEnc;
  m_useRDOQ      = bUseRDOQ;
  m_useRDOQTS    = bUseRDOQTS;
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
                                      TCoeff        & uiAbsSum,
                                const QpParam       & cQP
                              )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const UInt uiOrgTrDepth   = rTu.GetTransformDepthRel();

  uiAbsSum=0;

  //transform and quantise
  if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
#if RDPCM_INTER_LOSSLESS
    TCoeff temporaryResidual[MAX_TU_SIZE * MAX_TU_SIZE];
    if( (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
    {
      xInterResidueDpcm(rTu, pcResidual, uiStride, temporaryResidual, compID, uiAbsSum);
    }
    else
    {
      for (UInt line = 0; line < uiHeight; line++)
        for (UInt column = 0; column < uiWidth; column++)
        {
          temporaryResidual[(line * uiWidth) + column] = pcResidual[(line * uiStride) + column];
        }
    }
#endif
#if RExt__NRCE2_RESIDUAL_ROTATION
    const Bool rotateResidual = pcCU->isResidualRotated(uiWidth);
    const UInt lastColumn     = uiWidth  - 1;
    const UInt lastRow        = uiHeight - 1;
#endif
    for (UInt k = 0; k<uiHeight; k++)
    {
      for (UInt j = 0; j<uiWidth; j++)
      {
#if RDPCM_INTER_LOSSLESS
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - k) * uiWidth) + (lastColumn - j)) : ((k * uiWidth) + j));
        rpcCoeff[coefficientIndex] = temporaryResidual[(k * uiWidth) + j];
#else
        rpcCoeff[k*uiWidth+j]= temporaryResidual[k*uiWidth+j];
#endif
        uiAbsSum += abs(temporaryResidual[k*uiWidth+j]);
#else
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - k) * uiWidth) + (lastColumn - j)) : ((k * uiWidth) + j));
        rpcCoeff[coefficientIndex] = pcResidual[(k * uiStride) + j];
#else
        rpcCoeff[k*uiWidth+j]= pcResidual[k*uiStride+j];
#endif
        uiAbsSum += TCoeff(abs(pcResidual[k*uiStride+j]));
#endif
      }
    }
  }
  else
  {
#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to transform\n";
    printBlock(pcResidual, uiWidth, uiHeight, uiStride);
#endif

    const Bool useDST = isLuma(compID) && (pcCU->isIntra(uiAbsPartIdx));

    assert( (pcCU->getSlice()->getSPS()->getMaxTrSize() >= uiWidth) );

    if(pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0)
    {
      xTransformSkip( pcResidual, uiStride, m_plTempCoeff, rTu, compID );
    }
    else
    {
      xT( compID, useDST, pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight );
    }

#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between transform and quantiser\n";
    printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

#if RDPCM_INTER_LOSSY
    if( pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0 && (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
    {
      //  Inter coded TU with transform skip: select the best inter RDPCM mode
      xQuantInterRdpcm(rTu, m_plTempCoeff, rpcCoeff, 
#if ADAPTIVE_QP_SELECTION
        rpcArlCoeff,
#endif
        uiAbsSum, compID, cQP);
    }
    else
    {
#endif

      xQuant( rTu, m_plTempCoeff, rpcCoeff,

#if ADAPTIVE_QP_SELECTION
          rpcArlCoeff,
#endif
          uiAbsSum, compID, cQP );
#if RDPCM_INTER_LOSSY
    }
#endif

#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of quantiser\n";
    printBlock(rpcCoeff, uiWidth, uiHeight, uiWidth);
#endif
  }

  //set the CBF
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  pcCU->setCbfPartRange((((uiAbsSum > 0) ? 1 : 0) << uiOrgTrDepth), compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(compID));
#else
  pcCU->setCbfSubParts((((uiAbsSum > 0) ? 1 : 0) << uiOrgTrDepth), compID, uiAbsPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif
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

#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  if (uiWidth != uiHeight) //for intra, the TU will have been split above this level, so this condition won't be true, hence this only affects inter
  {
    //------------------------------------------------

    //recurse deeper

    TComTURecurse subTURecurse(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

    do
    {
      //------------------
   
      const UInt lineOffset = subTURecurse.GetSectionNumber() * subTURecurse.getRect(compID).height;

      Pel    *subTUResidual     = rpcResidual + (lineOffset * uiStride);
      TCoeff *subTUCoefficients = pcCoeff     + (lineOffset * subTURecurse.getRect(compID).width);

      invTransformNxN(subTURecurse, compID, subTUResidual, uiStride, subTUCoefficients, cQP DEBUG_STRING_PASS_INTO(psDebug));

      //------------------

    }
    while (subTURecurse.nextSection(rTu));

    //------------------------------------------------

    return;
  }
#endif

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
#if RExt__NRCE2_RESIDUAL_ROTATION
    const Bool rotateResidual = pcCU->isResidualRotated(uiWidth);
    const UInt lastColumn     = uiWidth  - 1;
    const UInt lastRow        = uiHeight - 1;
#endif

    for (UInt k = 0; k<uiHeight; k++)
    {
      for (UInt j = 0; j<uiWidth; j++)
      {
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - k) * uiWidth) + (lastColumn - j)) : ((k * uiWidth) + j));
        rpcResidual[(k * uiStride) + j] = Pel(pcCoeff[coefficientIndex]);
#else
        rpcResidual[k*uiStride+j] = Pel(pcCoeff[k*uiWidth+j]);
#endif
      }
    }
#if RDPCM_INTER_LOSSLESS
    if( (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
    {
      xInterInverseRdpcm(rTu, rpcResidual, uiStride, compID);
    }
#endif
  }
  else
  {
#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to dequantiser\n";
    printBlock(pcCoeff, uiWidth, uiHeight, uiWidth);
#endif

    const Bool useDST = isLuma(compID) && (pcCU->isIntra(uiAbsPartIdx));

    xDeQuant(rTu, pcCoeff, m_plTempCoeff, compID, cQP);

#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between dequantiser and inverse-transform\n";
    printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

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

#if RDPCM_INTER_LOSSY
      if( (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
      {
        //  Undo inter RDPCM before the final shift down for transform skip
        xInvInterRdpcm(rTu, m_plTempCoeff, compID);
      }
#endif
      xITransformSkip( m_plTempCoeff, rpcResidual, uiStride, rTu, compID );

#if defined DEBUG_STRING
      if (psDebug)
      {
        std::stringstream ss(stringstream::out);
        printBlockToStream(ss, "###InvTran resi: ", rpcResidual, uiWidth, uiHeight, uiStride);
        (*psDebug)+=ss.str();
        (*psDebug)+="(TS)\n";
      }
#endif
    }
    else
    {
      xIT( compID, useDST, m_plTempCoeff, rpcResidual, uiStride, uiWidth, uiHeight );

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

#ifdef DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of inverse-transform\n";
    printBlock(rpcResidual, uiWidth, uiHeight, uiStride);
    g_debugCounter++;
#endif
  }
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

    QpParam cQP; // NOTE: RExt - setQPforQuant is now dependent on TS flag (for 4:2:2), and therefore set at the lowest level.
    setQPforQuant( cQP,
                   pcCU->getQP( 0 ),
                   toChannelType(compID),
                   pcCU->getSlice()->getSPS()->getQpBDOffset(toChannelType(compID)),
                   (pcCU->getSlice()->getPPS()->getQpOffset(compID) + pcCU->getSlice()->getSliceChromaQpDelta(compID)),
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
Void TComTrQuant::xT( const ComponentID compID, Bool useDST, Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight )
{
#if MATRIX_MULT
  if( iWidth == iHeight)
  {
    xTr(g_bitDepth[toChannelType(compID)], piBlkResi, psCoeff, uiStride, (UInt)iWidth, useDST, g_maxTrDynamicRange[toChannelType(compID)]);
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

  xTrMxN( g_bitDepth[toChannelType(compID)], block, coeff, iWidth, iHeight, useDST, g_maxTrDynamicRange[toChannelType(compID)] );

  memcpy(psCoeff, coeff, (iWidth * iHeight * sizeof(TCoeff)));
}

/** Wrapper function between HM interface and core NxN inverse transform (2D)
 *  \param plCoef input data (transform coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iSize transform size (iSize x iSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void TComTrQuant::xIT( const ComponentID compID, Bool useDST, TCoeff* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight )
{
#if MATRIX_MULT
  if( iWidth == iHeight )
  {
    xITr(g_bitDepth[toChannelType(compID)], plCoef, pResidual, uiStride, (UInt)iWidth, useDST, g_maxTrDynamicRange[toChannelType(compID)]);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  memcpy(coeff, plCoef, (iWidth * iHeight * sizeof(TCoeff)));

  xITrMxN( g_bitDepth[toChannelType(compID)], coeff, block, iWidth, iHeight, useDST, g_maxTrDynamicRange[toChannelType(compID)] );

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

  Int iTransformShift = getTransformShift(toChannelType(component), rTu.GetEquivalentLog2TrSize(component));
  if (rTu.getCU()->getSlice()->getSPS()->getUseExtendedPrecision())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

#if RExt__NRCE2_RESIDUAL_ROTATION
  const Bool rotateResidual = rTu.getCU()->isResidualRotated(width);
  const UInt lastColumn     = width  - 1;
  const UInt lastRow        = height - 1;
#endif

  if (iTransformShift >= 0)
  {
    for (Int y = 0; y < height; y++)
    {
      for(Int x = 0; x < width; x++)
      {
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - y) * width) + (lastColumn - x)) : ((y * width) + x));
        psCoeff[coefficientIndex] = TCoeff(piBlkResi[(y * uiStride) + x]) << iTransformShift;
#else
        psCoeff[(y * width) + x]  = TCoeff(piBlkResi[(y * uiStride) + x]) << iTransformShift;
#endif
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << (iTransformShift - 1);

    for (Int y = 0; y < height; y++)
    {
      for(Int x = 0; x < width; x++)
      {
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - y) * width) + (lastColumn - x)) : ((y * width) + x));
        psCoeff[coefficientIndex] = (TCoeff(piBlkResi[(y * uiStride) + x]) + offset) >> iTransformShift;
#else
        psCoeff[(y * width) + x]  = (TCoeff(piBlkResi[(y * uiStride) + x]) + offset) >> iTransformShift;
#endif
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

  Int iTransformShift = getTransformShift(toChannelType(component), rTu.GetEquivalentLog2TrSize(component));
  if (rTu.getCU()->getSlice()->getSPS()->getUseExtendedPrecision())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

#if RExt__NRCE2_RESIDUAL_ROTATION
  const Bool rotateResidual = rTu.getCU()->isResidualRotated(width);
  const UInt lastColumn     = width  - 1;
  const UInt lastRow        = height - 1;
#endif

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift==0 ? 0 : (1 << (iTransformShift - 1));

    for (Int y = 0; y < height; y++)
    {
      for(Int x = 0; x < width; x++)
      {
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - y) * width) + (lastColumn - x)) : ((y * width) + x));
        pResidual[(y * uiStride) + x] =  Pel((plCoef[coefficientIndex] + offset) >> iTransformShift);
#else
        pResidual[(y * uiStride) + x] =  Pel((plCoef[(y * width) + x]  + offset) >> iTransformShift);
#endif
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
#if RExt__NRCE2_RESIDUAL_ROTATION
        const UInt coefficientIndex = (rotateResidual ? (((lastRow - y) * width) + (lastColumn - x)) : ((y * width) + x));
        pResidual[(y * uiStride) + x] = Pel(plCoef[coefficientIndex] << iTransformShift);
#else
        pResidual[(y * uiStride) + x] = Pel(plCoef[(y * width) + x]  << iTransformShift);
#endif
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
                                                            TCoeff       &uiAbsSum,
                                                      const ComponentID   compID,
                                                      const QpParam      &cQP  )
{
  const TComRectangle  & rect             = rTu.getRect(compID);
  const UInt             uiWidth          = rect.width;
  const UInt             uiHeight         = rect.height;
        TComDataCU    *  pcCU             = rTu.getCU();
  const UInt             uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const ChannelType      channelType      = toChannelType(compID);
  const UInt             uiLog2TrSize     = rTu.GetEquivalentLog2TrSize(compID);

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  // Represents scaling through forward transform
  Int iTransformShift            = getTransformShift(channelType, uiLog2TrSize);
  if ((pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0) && pcCU->getSlice()->getSPS()->getUseExtendedPrecision())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  UInt       uiGoRiceParam       = 0;
  Double     d64BlockUncodedCost = 0;
  const UInt uiLog2BlockWidth    = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt uiLog2BlockHeight   = g_aucConvertToBit[ uiHeight ] + 2;
  const UInt uiMaxNumCoeff       = uiWidth * uiHeight;
  assert(compID<MAX_NUM_COMPONENT);

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
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
  TCoeff deltaU   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  memset( rateIncUp,    0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( rateIncDown,  0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( sigRateDelta, 0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( deltaU,       0, sizeof(TCoeff) *  uiMaxNumCoeff );

  const Int iQBits = QUANT_SHIFT + cQP.getBaseQp().per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  const Double *const pdErrScale = getErrScaleCoeff(scalingListType, (uiLog2TrSize-2), cQP.getBaseQp().rem);
  const Int    *const piQCoef    = getQuantCoeff(scalingListType, cQP.getAdjustedQp().rem, (uiLog2TrSize-2));

#if ADAPTIVE_QP_SELECTION
  Int iQBitsC = iQBits - ARL_C_PRECISION;
  Int iAddC =  1 << (iQBitsC-1);
#endif

  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, rTu, compID);
  const UInt uiCGSize = (1 << MLS_CG_SIZE);

  const Bool applyAdditionalShift = (cQP.getAdjustedQp().per > cQP.getBaseQp().per);

  Double pdCostCoeffGroupSig[ MLS_GRP_NUM ];
  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  Int iCGLastScanPos = -1;

  UInt    uiCtxSet            = 0;
  Int     c1                  = 1;
  Int     c2                  = 0;
  Double  d64BaseCost         = 0;
  Int     iLastScanPos        = -1;

  UInt    c1Idx     = 0;
  UInt    c2Idx     = 0;
  Int     baseLevel;

  memset( pdCostCoeffGroupSig,   0, sizeof(Double) * MLS_GRP_NUM );
  memset( uiSigCoeffGroupFlag,   0, sizeof(UInt) * MLS_GRP_NUM );

  UInt uiCGNum = uiWidth * uiHeight >> MLS_CG_SIZE;
  Int iScanPos;
  coeffGroupRDStats rdStats;

  const UInt significanceMapContextOffset = getSignificanceMapContextOffset(compID);

  for (Int iCGScanPos = uiCGNum-1; iCGScanPos >= 0; iCGScanPos--)
  {
    UInt uiCGBlkPos = codingParameters.scanCG[ iCGScanPos ];
    UInt uiCGPosY   = uiCGBlkPos / codingParameters.widthInGroups;
    UInt uiCGPosX   = uiCGBlkPos - (uiCGPosY * codingParameters.widthInGroups);

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);

    for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)
    {
      iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;
      //===== quantization =====
      UInt    uiBlkPos          = codingParameters.scan[iScanPos];
      // set coeff
#if RExt__SQUARE_TRANSFORM_CHROMA_422
      const UInt scalingListCoeffIdx = uiBlkPos;
#else
      const UInt scalingListCoeffIdx = getScalingListCoeffIdx(format, compID, uiBlkPos, uiWidth, uiHeight);
#endif
      Double dTemp = pdErrScale[scalingListCoeffIdx];

      Int64 tmpLevel = Int64(abs(plSrcCoeff[ uiBlkPos ])) * piQCoef[scalingListCoeffIdx];

      if (applyAdditionalShift) tmpLevel = (tmpLevel + 1) >> 1;

      Intermediate_Int lLevelDouble = (Intermediate_Int)min<Int64>(tmpLevel, MAX_INTERMEDIATE_INT - (Intermediate_Int(1) << (iQBits - 1)));

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlDstCoeff[uiBlkPos]   = (TCoeff)(( lLevelDouble + iAddC) >> iQBitsC );
      }
#endif
      UInt uiMaxAbsLevel        = UInt((lLevelDouble + (Intermediate_Int(1) << (iQBits - 1))) >> iQBits);

      Double dErr               = Double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * dTemp;
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
      {
        iLastScanPos            = iScanPos;
        uiCtxSet                = getContextSetIndex(compID, (iScanPos >> MLS_CG_SIZE), 0);
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
          UShort uiCtxSig      = significanceMapContextOffset + getSigCtxInc( patternSigCtx, codingParameters, iScanPos, uiLog2BlockWidth, uiLog2BlockHeight, channelType );

          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                  lLevelDouble, uiMaxAbsLevel, uiCtxSig, uiOneCtx, uiAbsCtx, uiGoRiceParam,
                                                  c1Idx, c2Idx, iQBits, dTemp, 0 );

          sigRateDelta[ uiBlkPos ] = m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 1 ] - m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 0 ];
        }

        deltaU[ uiBlkPos ]        = TCoeff((lLevelDouble - (Intermediate_Int(uiLevel) << iQBits)) >> (iQBits-8));

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
          c2Idx ++;
        }
        else if( (c1 < 3) && (c1 > 0) && uiLevel)
        {
          c1++;
        }

        //===== context set update =====
        if( ( iScanPos % uiCGSize == 0 ) && ( iScanPos > 0 ) )
        {
          uiCtxSet          = getContextSetIndex(compID, ((iScanPos - 1) >> MLS_CG_SIZE), (c1 == 0)); //(iScanPos - 1) because we do this **before** entering the final group
          c1                = 1;
          c2                = 0;
          uiGoRiceParam     = 0;
          c1Idx             = 0;
          c2Idx             = 0;
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
    ui16CtxCbf   = pcCU->getCtxQtCbf( rTu, channelType );
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

          Double d64CostLast= codingParameters.scanType == SCAN_VER ? xGetRateLast( uiPosY, uiPosX, compID ) : xGetRateLast( uiPosX, uiPosY, compID );
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
    TCoeff level = piDstCoeff[ blkPos ];
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
    const Double inverseQuantScale = Double(g_invQuantScales[cQP.getBaseQp().rem]);
    Int64 rdFactor = (Int64)(inverseQuantScale * inverseQuantScale * (1 << (2 * cQP.getBaseQp().per))
                             / m_dLambda / 16 / (1 << (2 * DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[channelType] - 8)))
                             + 0.5);

    Int lastCG = -1;
    Int absSum = 0 ;
    Int n ;

    const TCoeff entropyCodingMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
    const TCoeff entropyCodingMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

    for( Int subSet = (uiWidth*uiHeight-1) >> MLS_CG_SIZE; subSet >= 0; subSet-- )
    {
      Int  subPos     = subSet << MLS_CG_SIZE;
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
        absSum += Int(piDstCoeff[ codingParameters.scan[ n + subPos ]]); //NOTE: RExt - only one bit is actually required!
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
                               -   ((abs(piDstCoeff[uiBlkPos]) == 1) ? sigRateDelta[uiBlkPos] : 0);

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

          if(piDstCoeff[minPos] == entropyCodingMaximum || piDstCoeff[minPos] == entropyCodingMinimum)
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
  if ((widthInGroups <= 1) && (heightInGroups <= 1)) return 0;

  const Bool rightAvailable = uiCGPosX < (widthInGroups  - 1);
  const Bool belowAvailable = uiCGPosY < (heightInGroups - 1);

  UInt sigRight = 0;
  UInt sigLower = 0;

  if (rightAvailable) sigRight = ((sigCoeffGroupFlag[ (uiCGPosY * widthInGroups) + uiCGPosX + 1 ] != 0) ? 1 : 0);
  if (belowAvailable) sigLower = ((sigCoeffGroupFlag[ (uiCGPosY + 1) * widthInGroups + uiCGPosX ] != 0) ? 1 : 0);

  return sigRight + (sigLower << 1);
}


/** Context derivation process of coeff_abs_significant_flag
 * \param patternSigCtx pattern for current coefficient group
 * \param codingParameters coding parmeters for the TU (includes the scan)
 * \param scanPosition current position in scan order
 * \param log2BlockWidth log2 width of the block
 * \param log2BlockHeight log2 height of the block
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
  if (codingParameters.firstSignificanceMapContext == significanceMapContextSetStart[chanType][CONTEXT_TYPE_SINGLE])
  {
    //single context mode
    return significanceMapContextSetStart[chanType][CONTEXT_TYPE_SINGLE];
  }

  const UInt rasterPosition = codingParameters.scan[scanPosition];
  const UInt posY           = rasterPosition >> log2BlockWidth;
  const UInt posX           = rasterPosition - (posY << log2BlockWidth);

  if ((posX + posY) == 0) return 0; //special case for the DC context variable

  Int offset = MAX_INT;

  if ((log2BlockWidth == 2) && (log2BlockHeight == 2)) //4x4
  {
    offset = ctxIndMap4x4[ (4 * posY) + posX ];
  }
  else
  {
    Int cnt = 0;

    switch (patternSigCtx)
    {
      //------------------

      case 0: //neither neighbouring group is significant
        {
          const Int posXinSubset     = posX & ((1 << MLS_CG_LOG2_WIDTH)  - 1);
          const Int posYinSubset     = posY & ((1 << MLS_CG_LOG2_HEIGHT) - 1);
          const Int posTotalInSubset = posXinSubset + posYinSubset;

          //first N coefficients in scan order use 2; the next few use 1; the rest use 0.
          const UInt context1Threshold = NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4;
          const UInt context2Threshold = NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4;

          cnt = (posTotalInSubset >= context1Threshold) ? 0 : ((posTotalInSubset >= context2Threshold) ? 1 : 2);
        }
        break;

      //------------------

      case 1: //right group is significant, below is not
        {
          const Int posYinSubset = posY & ((1 << MLS_CG_LOG2_HEIGHT) - 1);
          const Int groupHeight  = 1 << MLS_CG_LOG2_HEIGHT;

          cnt = (posYinSubset >= (groupHeight >> 1)) ? 0 : ((posYinSubset >= (groupHeight >> 2)) ? 1 : 2); //top quarter uses 2; second-from-top quarter uses 1; bottom half uses 0
        }
        break;

      //------------------

      case 2: //below group is significant, right is not
        {
          const Int posXinSubset = posX & ((1 << MLS_CG_LOG2_WIDTH)  - 1);
          const Int groupWidth   = 1 << MLS_CG_LOG2_WIDTH;

          cnt = (posXinSubset >= (groupWidth >> 1)) ? 0 : ((posXinSubset >= (groupWidth >> 2)) ? 1 : 2); //left quarter uses 2; second-from-left quarter uses 1; right half uses 0
        }
        break;

      //------------------

      case 3: //both neighbouring groups are significant
        {
          cnt = 2;
        }
        break;

      //------------------

      default:
        std::cerr << "ERROR: Invalid patternSigCtx \"" << Int(patternSigCtx) << "\" in getSigCtxInc" << std::endl;
        exit(1);
        break;
    }

    //------------------------------------------------

    const Bool notFirstGroup = ((posX >> MLS_CG_LOG2_WIDTH) + (posY >> MLS_CG_LOG2_HEIGHT)) > 0;

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
                                            Intermediate_Int                lLevelDouble,
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
    Double dErr         = Double( lLevelDouble  - ( Intermediate_Int(uiAbsLevel) << iQBits ) );
    Double dCurrCost    = dErr * dErr * dTemp + xGetICost( xGetICRate( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx ) );
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
__inline Int TComTrQuant::xGetICRate         ( UInt                            uiAbsLevel,
                                               UShort                          ui16CtxNumOne,
                                               UShort                          ui16CtxNumAbs,
                                               UShort                          ui16AbsGoRice,
                                               UInt                            c1Idx,
                                               UInt                            c2Idx
                                               ) const
{
  Int  iRate      = Int(xGetIEPRate()); // cost of sign bit
  UInt baseLevel  = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

  if ( uiAbsLevel >= baseLevel )
  {
    UInt symbol     = uiAbsLevel - baseLevel;
    UInt length;
    if (symbol < (COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice))
    {
      length = symbol>>ui16AbsGoRice;
      iRate += (length+1+ui16AbsGoRice)<< 15;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol  = symbol - ( COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice);
      while (symbol >= (1<<length))
      {
        symbol -=  (1<<(length++));
      }
      iRate += (COEF_REMAIN_BIN_REDUCTION+length+1-ui16AbsGoRice+length)<< 15;
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
    iRate = 0;
  }

  return  iRate;
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
                                              const ComponentID               component  ) const
{
  UInt uiCtxX   = g_uiGroupIdx[uiPosX];
  UInt uiCtxY   = g_uiGroupIdx[uiPosY];

  Double uiCost = m_pcEstBitsSbac->lastXBits[toChannelType(component)][ uiCtxX ] + m_pcEstBitsSbac->lastYBits[toChannelType(component)][ uiCtxY ];

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
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListEnc(scalingList,list,size,qp,format);
        xSetScalingListDec(scalingList,list,size,qp,format);
        setErrScaleCoeff(list,size,qp);
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 */
Void TComTrQuant::setScalingListDec(TComScalingList *scalingList, const ChromaFormat format)
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
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
Void TComTrQuant::setErrScaleCoeff(UInt list, UInt size, Int qp)
{
  const UInt uiLog2TrSize = g_aucConvertToBit[ g_scalingListSizeX[size] ] + 2;
  const ChannelType channelType = ((list == 0) || (list == MAX_NUM_COMPONENT)) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;

  const Int iTransformShift = getTransformShift(channelType, uiLog2TrSize);  // Represents scaling through forward transform

  UInt i,uiMaxNumCoeff = g_scalingListSize[size];
  Int *piQuantcoeff;
  Double *pdErrScale;
  piQuantcoeff   = getQuantCoeff(list, qp,size);
  pdErrScale     = getErrScaleCoeff(list, size, qp);

  Double dErrScale = (Double)(1<<SCALE_BITS);                                // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale = dErrScale*pow(2.0,(-2.0*iTransformShift));                     // Compensate for scaling through forward transform

  for(i=0;i<uiMaxNumCoeff;i++)
  {
    pdErrScale[i] =  dErrScale / piQuantcoeff[i] / piQuantcoeff[i] / (1 << DISTORTION_PRECISION_ADJUSTMENT(2 * (g_bitDepth[channelType] - 8)));
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
  quantcoeff  = getQuantCoeff(listId, qp, sizeId);

  Int quantScales = g_quantScales[qp];

  processScalingListEnc(coeff,quantcoeff,quantScales<<4,height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
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

  dequantcoeff = getDequantCoeff(listId, qp, sizeId);

  Int invQuantScale = g_invQuantScales[qp];

  processScalingListDec(coeff,dequantcoeff,invQuantScale,height,width,ratio,min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]),scalingList->getScalingListDC(sizeId,listId));
}

/** set flat matrix value to quantized coefficient
 */
Void TComTrQuant::setFlatScalingList(const ChromaFormat format)
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xsetFlatScalingList(list,size,qp,format);
        setErrScaleCoeff(list,size,qp);
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

  Int quantScales    = g_quantScales   [qp];
  Int invQuantScales = g_invQuantScales[qp] << 4;

  quantcoeff   = getQuantCoeff(list, qp, size);
  dequantcoeff = getDequantCoeff(list, qp, size);

  for(i=0;i<num;i++)
  {
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }
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
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j / ratio) + i / ratio];
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
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
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
    for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
    {
      for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        m_quantCoef   [sizeId][listId][qp] = new Int    [g_scalingListSize[sizeId]];
        m_dequantCoef [sizeId][listId][qp] = new Int    [g_scalingListSize[sizeId]];
        m_errScale    [sizeId][listId][qp] = new Double [g_scalingListSize[sizeId]];
      } // listID loop
    }
  }
}

/** destroy quantization matrix array
 */
Void TComTrQuant::destroyScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        if(m_quantCoef   [sizeId][listId][qp]) delete [] m_quantCoef   [sizeId][listId][qp];
        if(m_dequantCoef [sizeId][listId][qp]) delete [] m_dequantCoef [sizeId][listId][qp];
        if(m_errScale    [sizeId][listId][qp]) delete [] m_errScale    [sizeId][listId][qp];
      }
    }
  }
}

#if RExt__NRCE2_RESIDUAL_DPCM
Void TComTrQuant::transformSkipQuantOneSample(TComTU &rTu, ComponentID compID, Int resiDiff, TCoeff* pcCoeff, UInt uiPos, const QpParam &cQP )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

#if !RExt__SQUARE_TRANSFORM_CHROMA_422
  const TComRectangle &rect= rTu.getRect(compID);

  const UInt    uiWidth           = rect.width;
  const UInt    uiHeight          = rect.height;
  const ChromaFormat chFmt        = rTu.GetChromaFormat();
#endif

  // transform skip params
  Int iTransformShift = getTransformShift(toChannelType(compID), rTu.GetEquivalentLog2TrSize(compID));
  Int offset = 0;

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
  assert( scalingListType < SCALING_LIST_NUM );
  Int *piQuantCoeff = getQuantCoeff( scalingListType, cQP.getAdjustedQp().rem, (rTu.GetEquivalentLog2TrSize(compID)-2) );

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  const Int iQBits = QUANT_SHIFT + cQP.getAdjustedQp().per + iTransformShift;
  // NOTE: RExt - QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  Int iAdd = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);

  TCoeff iLevel;
  Int    iSign;

  UInt scalingListCoeffIdx;
  Int64 tmpLevel;

  TCoeff tmpCoef;

  // transform-skip
  if (iTransformShift >= 0)
  {
    tmpCoef = resiDiff << iTransformShift;
  }
  else // for very high bit depths
  {
    Int iTrShiftNeg = -iTransformShift;
    offset = 1 << (iTrShiftNeg - 1);
    tmpCoef = ( resiDiff + offset ) >> iTrShiftNeg;
  }

  // quantization
  iLevel  = tmpCoef;
  iSign   = (iLevel < 0 ? -1: 1);

#if RExt__SQUARE_TRANSFORM_CHROMA_422
  scalingListCoeffIdx = uiPos;
#else
  scalingListCoeffIdx = getScalingListCoeffIdx(chFmt, compID, uiPos, uiWidth, uiHeight);
#endif
  tmpLevel = (Int64) abs(iLevel) * piQuantCoeff[scalingListCoeffIdx];

  iLevel = (TCoeff) ((tmpLevel + iAdd ) >> iQBits);

  iLevel *= iSign;
  const TCoeff entropyCodingMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff entropyCodingMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;
  pcCoeff[ uiPos ] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, iLevel );
}

Void TComTrQuant::invTrSkipDeQuantOneSample( TComTU &rTu, ComponentID compID, TCoeff inSample, TCoeff &deQuantSample, const QpParam &cQP, UInt uiPos DEBUG_STRING_PASS_INTO(TCoeff &transformedDequantisedSample) )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

#if !RExt__SQUARE_TRANSFORM_CHROMA_422
  const TComRectangle &rect= rTu.getRect(compID);

  const UInt    uiWidth           = rect.width;
  const UInt    uiHeight          = rect.height;
  const ChromaFormat chFmt        = rTu.GetChromaFormat();
#endif

  const Int QP_per = cQP.getAdjustedQp().per;
  const Int QP_rem = cQP.getAdjustedQp().rem;

  // transform skip params
  Int iTransformShift = getTransformShift(toChannelType(compID), rTu.GetEquivalentLog2TrSize(compID));

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (getUseScalingList() ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);

  const TCoeff transformMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff transformMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  TCoeff clipQCoef;
  Intermediate_Int iCoeffQ;
  TCoeff tmpCoef;

  if(getUseScalingList())
  {
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);
#if RExt__SQUARE_TRANSFORM_CHROMA_422
      const Int deQuantIdx = uiPos;
#else
      const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, uiPos, uiWidth, uiHeight);
#endif

      clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift;
      tmpCoef   = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
    else
    {
      const Int leftShift = -rightShift;
#if RExt__SQUARE_TRANSFORM_CHROMA_422
      const Int deQuantIdx = uiPos;
#else
      const Int deQuantIdx = getScalingListCoeffIdx(fmt, compID, uiPos, uiWidth, uiHeight);
#endif

      clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) << leftShift;
      tmpCoef   = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }
  else
  {
    const Int scale     =  g_invQuantScales[QP_rem];
    const Int scaleBits =     (IQUANT_SHIFT + 1)   ;

    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);
      clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

      tmpCoef = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
    else
    {
      const Int leftShift = -rightShift;
      clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

      tmpCoef = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }

  // Inverse transform-skip
#if defined DEBUG_STRING
  transformedDequantisedSample = tmpCoef;
#endif

  if (iTransformShift >= 0)
  {
    TCoeff offset = iTransformShift==0 ? 0 : (1 << (iTransformShift - 1));
    deQuantSample =  ( tmpCoef + offset ) >> iTransformShift;
  }
  else //for very high bit depths
  {
    Int iTrShiftNeg = -iTransformShift;
    deQuantSample = tmpCoef << iTrShiftNeg;
  }
}

#endif // RExt__NRCE2_RESIDUAL_DPCM

#if RDPCM_INTER_LOSSLESS
/** Performs inverse RDPCM for inter predicted residuals. It does a recursion step in case of 4:2:2 chroma format
 * \param rTu current TU where the residulas belong to
 * \param stride incremental step to index the residuals array
 * \param compID component identifier
 */
Void TComTrQuant::xInterInverseRdpcm( TComTU &rTu, Pel *&residuals, UInt stride, ComponentID compID )
{

  const UInt absPartIdx = rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &rect = rTu.getRect(compID);
  TComDataCU *cu = rTu.getCU();
  const UInt width = rect.width;
  const UInt height = rect.height;

  UInt direction = cu->getInterRdpcmMode(compID, absPartIdx);

  assert(height == width);

  if(direction == DPCM_OFF || direction == NUMBER_OF_INTER_RDPCM_MODES) return;

  Pel *residualsPtr = residuals;

  if(direction == DPCM_HOR)
  {
    for(int r = 0; r < height; r++)
    {
      for(int c = 1; c < width; c++)
      {
        residualsPtr[c] += residualsPtr[c-1];
      }
      residualsPtr += stride;
    }
  }
  else if(direction == DPCM_VER)
  {
    Pel *previousRowResidualsPtr = residuals;
    residualsPtr += stride;
    for(int r = 1; r < height; r++)
    {
      for(int c = 0; c < width; c++)
      {
        residualsPtr[c] += previousRowResidualsPtr[c];
      }
      residualsPtr += stride;
      previousRowResidualsPtr += stride;
    }
  }
  else
  {
    assert(0);
  }
}

/** Selects the best inter RDPCM mode by minimizing the sum of absolute differences for the predicted residuals
 * \param rTu current TU data structure
 * \param residuals pointer to the array which will contains the inter predicted residuals
 * \param stride incremental step to index the residuals array
 * \param coefficients pointer to the array which will contain the RDPCM predicted residuals
 * \param compID component identifier
 * \param bestInterRdpcmMode pointer to the variable which will contain the best inter RDPCM mode for this TU
 * \param absSum pointer to the variable which will contain the SAD for the best inter RDPCM mode
 */
Void TComTrQuant::xInterResidueDpcm( TComTU      &rTu, 
                                     Pel*        inputResiduals, 
                                     UInt        stride,
                                     TCoeff*     outputResiduals,
                                     ComponentID compID,
                                     TCoeff      &absSum)
{

  const TComRectangle &rect= rTu.getRect(compID);
  TComDataCU *cu = rTu.getCU();
  const UInt absPartIdx = rTu.GetAbsPartIdxTU(compID);

  UInt height = rect.height;
  UInt width  = rect.width;

  assert(height == width);

  TCoeff tempResiduals[MAX_TU_SIZE * MAX_TU_SIZE];
  TCoeff *tempResidualsPtr;
  Pel *inputResidualsPtr;
  TCoeff currentSad, bestSad = std::numeric_limits<TCoeff>::max();
  UInt bestInterRdpcmMode = NUMBER_OF_INTER_RDPCM_MODES;

  for(UInt direction = DPCM_OFF; direction < NUMBER_OF_INTER_RDPCM_MODES; direction++)
  {
    currentSad = 0;
    tempResidualsPtr = tempResiduals;
    inputResidualsPtr = inputResiduals;

    if(direction == DPCM_OFF)
    {
      for(int r = 0; r < height; r++)
      {
        for(int c = 0; c < width; c++)
        {
          tempResidualsPtr[c] = (TCoeff)inputResidualsPtr[c];
          currentSad += abs(tempResidualsPtr[c]);
        }
        inputResidualsPtr += stride;
        tempResidualsPtr += width;
      }
    }
    else if(direction == DPCM_HOR)
    {
      for(int r = 0; r < height; r++)
      {
        tempResidualsPtr[0] = (TCoeff)inputResidualsPtr[0];
        currentSad += abs(tempResidualsPtr[0]);
        for(int c = 1; c < width; c++)
        {
          tempResidualsPtr[c] = (TCoeff)(inputResidualsPtr[c] - inputResidualsPtr[c-1]);
          currentSad += abs(tempResidualsPtr[c]);
        }
        inputResidualsPtr += stride;
        tempResidualsPtr += width;
      }
    }
    else
    {
      //  Vertical RDPCM
      //  First row
      for(int c = 0; c < width; c++)
      {
        tempResidualsPtr[c] = (TCoeff)inputResidualsPtr[c];
        currentSad += abs(tempResidualsPtr[c]);
      }
      Pel *previousRowTempResidualsPtr = inputResidualsPtr;
      inputResidualsPtr += stride;
      tempResidualsPtr += width;

      for(int r = 1; r < height; r++)
      {
        for(int c = 0; c < width; c++)
        {
          tempResidualsPtr[c] = (TCoeff)(inputResidualsPtr[c] - previousRowTempResidualsPtr[c]);
          currentSad += abs(tempResidualsPtr[c]);
        }
        inputResidualsPtr += stride;
        tempResidualsPtr += width;
        previousRowTempResidualsPtr += stride;
      }
    }

    //  Check for best SAD
    if(currentSad < bestSad)
    {
      bestInterRdpcmMode = direction;
      bestSad = currentSad;
      absSum  = currentSad;
      //::memcpy(outputResiduals, tempResiduals, width*height*sizeof(TCoeff));
      for(int r = 0; r < height; r++)
      {
        for(int c = 0; c < width; c++)
        {
          outputResiduals[r*width + c] = tempResiduals[r*width + c];
        }
      }
    }
  }

  assert(bestInterRdpcmMode != NUMBER_OF_INTER_RDPCM_MODES);

  //  Set inter Rdpcm mode and cbf
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  cu->setInterRdpcmModePartRange(bestInterRdpcmMode, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));  
#else
  cu->setInterRdpcmModeSubParts(bestInterRdpcmMode, compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif
}
#endif

#if RDPCM_INTER_LOSSY
/** Performs HEVC quantisation of one sample. Main purpose of this function is to make the code for "xQuantInterRdpcm" more readable
 * \param[in] rTu current TU where the sample belongs to
 * \param[in] residual sample value to be quantised
 * \param[in|out] quantisedLevel pointer to the memory location which will contain the quantised value for the residual parameter
 * \param[in|out] quantisedArlLevel pointer to the memory location which will contain the quantised value for adaptive residual quantisation
 * \param[in] compID current component identifier
 * \param[in] cQP quantiser class
 * \param[in] quantIdx index for the quantiser scale
 * \param[in|out] deltaU pointer to the memory location which will contain the value for the quantisation error associated to this sample
 */
inline Void TComTrQuant::xQuantiseSample(       TComTU      &rTu, 
                                                TCoeff      residual, 
                                                TCoeff      &quantisedLevel,
#if ADAPTIVE_QP_SELECTION
                                                TCoeff      &quantisedArlLevel,
#endif
                                                ComponentID compID,
                                                const QpParam     &cQP, 
                                                Int         quantIdx,
                                                TCoeff      &deltaU
                                        )
{
  const UInt log2TrSize = rTu.GetEquivalentLog2TrSize(compID);
  const UInt absPartIdx = rTu.GetAbsPartIdxTU();
  TComDataCU *cu = rTu.getCU();
  const Int scalingListType = getScalingListType(cu->getPredictionMode(absPartIdx), compID);

  TCoeff level, signLevel;
  Int64 tmpLevel;


  //  Get quantiser scale
  Int *quantiserScale = getQuantCoeff(scalingListType, cQP.getAdjustedQp().rem, log2TrSize-2);

  //  Get transform shift
  Int transformShift = getTransformShift(toChannelType(compID), log2TrSize);

  //  Get quantiser right shift quantity
  const Int quantiserRightShift = QUANT_SHIFT + cQP.getAdjustedQp().per + transformShift;

  //  Get offset for quantiser right shift
  const Int rightShiftOffset = (cu->getSlice()->getSliceType() == I_SLICE ? 171 : 85) << (quantiserRightShift-9);

  const Int qBits8 = quantiserRightShift - 8;

#if ADAPTIVE_QP_SELECTION
  Int iQBitsC = MAX_INT;
  Int iAddC   = MAX_INT;
    
  iQBitsC = quantiserRightShift - ARL_C_PRECISION;
  iAddC   = 1 << (iQBitsC-1);
#endif

  signLevel = (residual < 0 ? -1: 1);
  tmpLevel = (Int64)abs(residual) * quantiserScale[quantIdx];

#if ADAPTIVE_QP_SELECTION
  quantisedArlLevel = (TCoeff)((tmpLevel + iAddC ) >> iQBitsC);
#endif

  level = (TCoeff)((tmpLevel + rightShiftOffset ) >> quantiserRightShift);

  deltaU = (TCoeff)((tmpLevel - (level<<quantiserRightShift) )>> qBits8);

  level *= signLevel;

  const TCoeff entropyCodingMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff entropyCodingMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;
  quantisedLevel = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, level );
}


/** Performs HEVC inverse quantisation of one sample. Main purpose of this function is to make the code for "xQuantInterRdpcm" more readable
 * \param[in] rTu current TU where the sample belongs to
 * \param[in] quantisedResidual level of the quantised sample
 * \param[in|out] reconCoeff pointer to the memory location which will contain the reconstructed value for the quantisedResidual parameter
 * \param[in] compID current component identifier
 * \param[in] cQP quantiser class
 * \param[in] deQuantIdx index for the inverse quantiser scale
 */
inline Void TComTrQuant::xDequantiseSample(
                                                  TComTU      &rTu,
                                                  TCoeff      quantisedResidual, 
                                                  TCoeff      &reconCoeff,
                                                  ComponentID compID, 
                                            const QpParam     &cQP, 
                                            const Int         deQuantIdx
  )
{
  const UInt log2TrSize = rTu.GetEquivalentLog2TrSize(compID);
  const UInt absPartIdx = rTu.GetAbsPartIdxTU();
  TComDataCU *cu        = rTu.getCU();

  const Int scalingListType = getScalingListType(cu->getPredictionMode(absPartIdx), compID);

  const TCoeff transformMinimum = -(1 << g_maxTrDynamicRange[toChannelType(compID)]);
  const TCoeff transformMaximum =  (1 << g_maxTrDynamicRange[toChannelType(compID)]) - 1;

  assert(scalingListType < SCALING_LIST_NUM);

  // Get the forward transform shift
  Int transformShift = getTransformShift(toChannelType(compID), log2TrSize);

  const Int QP_per = cQP.getAdjustedQp().per;
  const Int QP_rem = cQP.getAdjustedQp().rem;

  //  Get the inverse quantiser right shift
  const Int quantiserRightShift = (IQUANT_SHIFT - (transformShift + QP_per)) + (getUseScalingList() ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  if(getUseScalingList())
  {
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + quantiserRightShift) - dequantCoefBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;
    TCoeff clipQuantisedCoeff;
    Intermediate_Int iCoeffQ;
    Int *quantiserInverseScale = getDequantCoeff(scalingListType, QP_rem, log2TrSize-2);

    if(quantiserRightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (quantiserRightShift - 1);

      clipQuantisedCoeff = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, quantisedResidual));
      iCoeffQ   = ((Intermediate_Int(clipQuantisedCoeff) * quantiserInverseScale[deQuantIdx]) + iAdd ) >> quantiserRightShift;
      reconCoeff = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
    else
    {
      const Int quantiserLeftShift = -quantiserRightShift;

      clipQuantisedCoeff = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, quantisedResidual));
      iCoeffQ   = (Intermediate_Int(clipQuantisedCoeff) * quantiserInverseScale[deQuantIdx]) << quantiserLeftShift;
      reconCoeff = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }
  else
  {
    const Int              scale               = g_invQuantScales[QP_rem];
    const Int              scaleBits           = IQUANT_SHIFT + 1;
    const UInt             targetInputBitDepth = std::min<UInt>((g_maxTrDynamicRange[toChannelType(compID)] + 1), (((sizeof(Intermediate_Int) * 8) + quantiserRightShift) - scaleBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;
    TCoeff clipQuantisedCoeff;
    Intermediate_Int iCoeffQ;

    if (quantiserRightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (quantiserRightShift - 1);

      clipQuantisedCoeff = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, quantisedResidual));
      iCoeffQ   = (Intermediate_Int(clipQuantisedCoeff) * scale + iAdd) >> quantiserRightShift;
      reconCoeff = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
    else
    {
      const Int quantiserLeftShift = -quantiserRightShift;

      clipQuantisedCoeff = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, quantisedResidual));
      iCoeffQ   = (Intermediate_Int(clipQuantisedCoeff) * scale) << quantiserLeftShift;
      reconCoeff = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }
}

/** Selects the best inter RDPCM mode (off, horizontal or vertical) by minimising the sum of absolute difference associated to the quantised levels.
 ** RDOQ is not performed unless the best selected mode is DPCM_OFF. Moreover sign data hiding is avoided when RDPCM is either horizontal or vertical
 * \param[in] rTu current TU where the coefficients belong to
 * \param[in] pSrc pointer to the array of input coefficients to be quantised
 * \param[in|out] pDes pointer to the array of output quantised coefficients
 * \param[in|out] pArlDes pointer to the array of output quantised coefficients when adaptive residual quantisation is used
 * \param[in|out] uiAcSum pointer to the memory location which will contain the value for the sum of absolute difference associated to the quantised levels
 * \param[in] compID current component identifier
 * \param[in] cQP quantiser class
 */
Void TComTrQuant::xQuantInterRdpcm( TComTU       &rTu,
                                    TCoeff      * pSrc,
                                    TCoeff      * pDes,
#if ADAPTIVE_QP_SELECTION
                                    TCoeff      *&pArlDes,
#endif
                                    TCoeff       &uiAcSum,
                                    const ComponentID   compID,
                                    const QpParam      &cQP)
{
  TComDataCU *cu = rTu.getCU();
  const TComRectangle &rect = rTu.getRect(compID);
  UInt height = rect.height;
  UInt width  = rect.width;
#if !RExt__SQUARE_TRANSFORM_CHROMA_422
  ChromaFormat chromaFormat = cu->getPic()->getChromaFormat();
#endif

  //  Check whether this function is called only for 4x4 blocks
  assert(height <= (1<<cu->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) && width <= (1<<cu->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) );

  const UInt absPartIdx = rTu.GetAbsPartIdxTU();

  //  Temporary variables for mode decision
  TCoeff currentSad, bestSad = std::numeric_limits<TCoeff>::max();
  UInt bestRdpcmMode = 0;  
  TCoeff tempQCoeff[MAX_TU_SIZE*MAX_TU_SIZE], tempRecCoeff[MAX_TU_SIZE*MAX_TU_SIZE], *tempQCoeffPtr, *tempCoeffPtr;
#if ADAPTIVE_QP_SELECTION
  TCoeff tempArlCoeff[MAX_TU_SIZE*MAX_TU_SIZE], *arlCoeffPtr;
#endif
  TCoeff deltaU[MAX_TU_SIZE*MAX_TU_SIZE], tempDeltaU[MAX_TU_SIZE*MAX_TU_SIZE];

  for(int direction = DPCM_OFF; direction < NUMBER_OF_INTER_RDPCM_MODES; direction++)
  {
    currentSad = 0;
    tempCoeffPtr = pSrc;
    tempQCoeffPtr = tempQCoeff;
#if ADAPTIVE_QP_SELECTION
    arlCoeffPtr = tempArlCoeff;
#endif

    if(direction == DPCM_OFF)
    {
      for(int r = 0, samplePosition = 0; r < height; r++)
      {
        for(int c = 0; c < width; c++, samplePosition++)
        {
#if RExt__SQUARE_TRANSFORM_CHROMA_422
          const Int quantIdx = samplePosition;
#else
          const Int quantIdx = getScalingListCoeffIdx(chromaFormat, compID, samplePosition, width, height);
#endif
          xQuantiseSample(rTu, tempCoeffPtr[c], tempQCoeffPtr[c], 
#if ADAPTIVE_QP_SELECTION
            arlCoeffPtr[c], 
#endif
            compID, cQP, quantIdx, tempDeltaU[samplePosition]);
          currentSad += abs(tempQCoeffPtr[c]);
        }
        tempCoeffPtr += width;
        tempQCoeffPtr += width;
#if ADAPTIVE_QP_SELECTION
        arlCoeffPtr += width;
#endif
      }
    }
    else if (direction == DPCM_HOR)
    {
      //  Processing is column-wise
      //  First column: quantisation only
      for(int r = 0, samplePosition = 0; r < height; r++, samplePosition += width)
      {
#if RExt__SQUARE_TRANSFORM_CHROMA_422
        const Int quantIdx = samplePosition;
#else
        const Int quantIdx = getScalingListCoeffIdx(chromaFormat, compID, samplePosition, width, height);
#endif
        xQuantiseSample(rTu, tempCoeffPtr[samplePosition], tempQCoeffPtr[samplePosition], 
#if ADAPTIVE_QP_SELECTION
          arlCoeffPtr[samplePosition], 
#endif
          compID, cQP, quantIdx, tempDeltaU[samplePosition]);
        currentSad += abs(tempQCoeffPtr[samplePosition]);
        xDequantiseSample(rTu, tempQCoeffPtr[samplePosition], tempRecCoeff[samplePosition], compID, cQP, quantIdx);
      }
      
      int samplePosition;
      //  Remaining columns
      for(int c = 1; c < width; c++)
      {
        for(int r = 0; r < height; r++)
        {
          samplePosition = r*width + c;
#if RExt__SQUARE_TRANSFORM_CHROMA_422
          const Int quantIdx = samplePosition;
#else
          const Int quantIdx = getScalingListCoeffIdx(chromaFormat, compID, samplePosition, width, height);
#endif
          xQuantiseSample(rTu, tempCoeffPtr[samplePosition] - tempRecCoeff[samplePosition - 1], tempQCoeff[samplePosition], 
#if ADAPTIVE_QP_SELECTION
            arlCoeffPtr[samplePosition],
#endif
            compID, cQP, quantIdx, tempDeltaU[samplePosition]);
          currentSad += abs(tempQCoeffPtr[samplePosition]);
          xDequantiseSample(rTu, tempQCoeffPtr[samplePosition], tempRecCoeff[samplePosition], compID, cQP, quantIdx);
          tempRecCoeff[samplePosition] += tempRecCoeff[samplePosition - 1];
        }
      }
    }
    else
    {
      //  Processing is row-wise
      //  First row: quantisation only
      for(int c = 0; c < width; c++)
      {
        xQuantiseSample(rTu, tempCoeffPtr[c], tempQCoeffPtr[c], 
#if ADAPTIVE_QP_SELECTION
          arlCoeffPtr[c],
#endif
          compID, cQP, c, tempDeltaU[c]);
        currentSad += abs(tempQCoeffPtr[c]);
        xDequantiseSample(rTu, tempQCoeffPtr[c], tempRecCoeff[c], compID, cQP, c);
      }

      //  Remaining rows
      int samplePosition;
      for(int r = 1; r < height; r++)
      {
        for(int c = 0; c < width; c++)
        {
          samplePosition = r*width + c;
#if RExt__SQUARE_TRANSFORM_CHROMA_422
          const Int quantIdx = samplePosition;
#else
          const Int quantIdx = getScalingListCoeffIdx(chromaFormat, compID, samplePosition, width, height);
#endif
          xQuantiseSample(rTu, tempCoeffPtr[samplePosition] - tempRecCoeff[samplePosition - width], tempQCoeffPtr[samplePosition],
#if ADAPTIVE_QP_SELECTION
            arlCoeffPtr[samplePosition],
#endif
            compID, cQP, quantIdx, tempDeltaU[samplePosition]);
          currentSad += abs(tempQCoeffPtr[samplePosition]);
          xDequantiseSample(rTu, tempQCoeffPtr[samplePosition], tempRecCoeff[samplePosition], compID, cQP, quantIdx);
          tempRecCoeff[samplePosition] += tempRecCoeff[samplePosition - width];
        }
      }
    }

    //  Optimisation step
    if(currentSad < bestSad)
    {
      bestSad = currentSad;
      uiAcSum = currentSad;
      bestRdpcmMode = direction;
      tempQCoeffPtr = tempQCoeff;
#if ADAPTIVE_QP_SELECTION
      arlCoeffPtr = tempArlCoeff;
#endif
      //NOTE: RExt - deltaU is not used in this function
      memcpy(deltaU, tempDeltaU, (MAX_TU_SIZE * MAX_TU_SIZE * sizeof(TCoeff)));
      for(int r = 0; r < height; r++)
      {
        for(int c = 0; c < width; c++)
        {
          pDes[r*width + c] = tempQCoeffPtr[c];
#if ADAPTIVE_QP_SELECTION
          pArlDes[r*width + c] = arlCoeffPtr[c];
#endif
        }
        tempQCoeffPtr += width;
#if ADAPTIVE_QP_SELECTION
        arlCoeffPtr += width;
#endif
      }
      if(direction == DPCM_OFF && currentSad == 0)
      {
        //  This happens when all the residues are zero: we can avoid trying others RDPCM modes
        break;
      }
    }
  }

  //  Set the best RDPCM in the current CU sata structure
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  cu->setInterRdpcmModePartRange(bestRdpcmMode, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));  
#else  
  cu->setInterRdpcmModeSubParts(bestRdpcmMode, compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif

  if(bestRdpcmMode == DPCM_OFF)
  {
    uiAcSum = 0;
    xQuant( rTu, pSrc, pDes,
#if ADAPTIVE_QP_SELECTION
      pArlDes, 
#endif
      uiAcSum, compID, cQP );
  }
}

/** Performs inverse RDPCM over the reconstructed residuals. 
 * \param[in] rTu current TU where the coefficients belong to
 * \param[in|out] reconCoeff pointer to the array of reconstructed coefficients which will contain the modified values after inverse RDPCM
 * \param[in] compID current component identifier
 */
Void TComTrQuant::xInvInterRdpcm (       TComTU      &rTu, 
                                         TCoeff      *reconCoeff,
                                   const ComponentID compID)
{
  const TComRectangle &rect = rTu.getRect(compID);
  TComDataCU *cu = rTu.getCU();
  UInt absParIdx = rTu.GetAbsPartIdxTU();
  UInt height = rect.height;
  UInt width  = rect.width;

  assert(height <= (1<<cu->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) && width <= (1<<cu->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) );

  UInt interRdpcmMode = cu->getInterRdpcmMode(compID, absParIdx);

  if(interRdpcmMode == DPCM_OFF)
  {
    return;
  }
  else if(interRdpcmMode == DPCM_HOR)
  {
    for(int r = 0; r < height; r++)
    {
      for(int c = 1; c < width; c++)
      {
        reconCoeff[r*width + c] += reconCoeff[r*width + c - 1];
      }
    }
  }
  else if (interRdpcmMode == DPCM_VER)
  {
    for(int r = 1; r < height; r++)
    {
      for(int c = 0; c < width; c++)
      {
        reconCoeff[r*width + c] += reconCoeff[(r-1)*width + c];
      }
    }
  }
  else
  {
    assert(0);
  }
}
#endif

//! \}
