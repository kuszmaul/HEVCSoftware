/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
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

/** \file     TComPicYuv.cpp
    \brief    picture YUV buffer class
*/

#include <cstdlib>
#include <assert.h>
#include <memory.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

#include "TComPicYuv.h"
#include "TLibVideoIO/TVideoIOYuv.h"

//! \ingroup TLibCommon
//! \{

TComPicYuv::TComPicYuv()
{
  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    m_apiPicBuf[i]    = NULL;   // Buffer (including margin)
    m_piPicOrg[i]     = NULL;    // m_apiPicBufY + m_iMarginLuma*getStride() + m_iMarginLuma
  }

  for(UInt i=0; i<MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_cuOffset[i]=0;
    m_buOffset[i]=0;
  }

  m_bIsBorderExtended = false;
}




TComPicYuv::~TComPicYuv()
{
}




Void TComPicYuv::create( const Int  iPicWidth,    const  Int iPicHeight,    const ChromaFormat chromaFormatIDC,
                         const UInt uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth )
{
  m_iPicWidth         = iPicWidth;
  m_iPicHeight        = iPicHeight;
  m_iLcuWidth         = uiMaxCUWidth;
  m_iLcuHeight        = uiMaxCUHeight;
  m_chromaFormatIDC   = chromaFormatIDC;
  m_iMarginX          = g_uiMaxCUWidth  + 16; // for 16-byte alignment
  m_iMarginY          = g_uiMaxCUHeight + 16;  // margin for 8-tap filter and infinite padding
  m_bIsBorderExtended = false;

  // assign the picture arrays and set up the ptr to the top left of the original picture
  {
    Int chan=0;
    for(; chan<getNumberValidComponents(); chan++)
    {
      const ComponentID ch=ComponentID(chan);
      m_apiPicBuf[chan] = (Pel*)xMalloc( Pel, getStride(ch)       * getTotalHeight(ch));
      m_piPicOrg[chan]  = m_apiPicBuf[chan] + (m_iMarginY >> getComponentScaleY(ch))   * getStride(ch)       + (m_iMarginX >> getComponentScaleX(ch));
    }
    for(;chan<MAX_NUM_COMPONENT; chan++)
    {
      m_apiPicBuf[chan] = NULL;
      m_piPicOrg[chan]  = NULL;
    }
  }


  const Int numCuInWidth  = m_iPicWidth  / m_iLcuWidth  + (m_iPicWidth  % m_iLcuWidth  != 0);
  const Int numCuInHeight = m_iPicHeight / m_iLcuHeight + (m_iPicHeight % m_iLcuHeight != 0);
  for(Int chan=0; chan<2; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Int lcuHeight=m_iLcuHeight>>getComponentScaleY(ch);
    const Int lcuWidth=m_iLcuWidth>>getComponentScaleX(ch);
    const Int stride = getStride(ch);

    m_cuOffset[chan] = new Int[numCuInWidth * numCuInHeight];

    for (Int cuRow = 0; cuRow < numCuInHeight; cuRow++)
      for (Int cuCol = 0; cuCol < numCuInWidth; cuCol++)
        m_cuOffset[chan][cuRow * numCuInWidth + cuCol] = stride * cuRow * lcuHeight + cuCol * lcuWidth;

    m_buOffset[chan] = new Int[(size_t)1 << (2 * uiMaxCUDepth)];

    const Int numSubBlockPartitions=(1<<uiMaxCUDepth);
    const Int minSubBlockHeight    =(lcuHeight >> uiMaxCUDepth);
    const Int minSubBlockWidth     =(lcuWidth  >> uiMaxCUDepth);

    for (Int buRow = 0; buRow < numSubBlockPartitions; buRow++)
      for (Int buCol = 0; buCol < numSubBlockPartitions; buCol++)
        m_buOffset[chan][(buRow << uiMaxCUDepth) + buCol] = stride  * buRow * minSubBlockHeight + buCol * minSubBlockWidth;
  }
  return;
}



Void TComPicYuv::destroy()
{
  for(Int chan=0; chan<MAX_NUM_COMPONENT; chan++)
  {
    m_piPicOrg[chan] = NULL;

    if( m_apiPicBuf[chan] ){ xFree( m_apiPicBuf[chan] );    m_apiPicBuf[chan] = NULL; }
  }

  for(UInt chan=0; chan<MAX_NUM_CHANNEL_TYPE; chan++)
  {
    if (m_cuOffset[chan]) delete[] m_cuOffset[chan]; m_cuOffset[chan] = NULL;
    if (m_buOffset[chan]) delete[] m_buOffset[chan]; m_buOffset[chan] = NULL;
  }
}



Void  TComPicYuv::copyToPic (TComPicYuv*  pcPicYuvDst) const
{
  assert( m_iPicWidth  == pcPicYuvDst->getWidth(COMPONENT_Y)  );
  assert( m_iPicHeight == pcPicYuvDst->getHeight(COMPONENT_Y) );
  assert( m_chromaFormatIDC == pcPicYuvDst->getChromaFormat() );

  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    ::memcpy ( pcPicYuvDst->getBuf(ch), m_apiPicBuf[ch], sizeof (Pel) * getStride(ch) * getTotalHeight(ch));
  }
  return;
}


Void TComPicYuv::extendPicBorder ()
{
  if ( m_bIsBorderExtended ) return;

  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    Pel *piTxt=getAddr(ch); // piTxt = point to (0,0) of image within bigger picture.
    const Int iStride=getStride(ch);
    const Int iWidth=getWidth(ch);
    const Int iHeight=getHeight(ch);
    const Int iMarginX=getMarginX(ch);
    const Int iMarginY=getMarginY(ch);

    Pel*  pi = piTxt;
    // do left and right margins
    for (Int y = 0; y < iHeight; y++)
    {
      for (Int x = 0; x < iMarginX; x++ )
      {
        pi[ -iMarginX + x ] = pi[0];
        pi[    iWidth + x ] = pi[iWidth-1];
      }
      pi += iStride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (iStride + iMarginX);
    // pi is now the (-marginX, height-1)
    for (Int y = 0; y < iMarginY; y++ )
    {
      ::memcpy( pi + (y+1)*iStride, pi, sizeof(Pel)*(iWidth + (iMarginX<<1)) );
    }

    // pi is still (-marginX, height-1)
    pi -= ((iHeight-1) * iStride);
    // pi is now (-marginX, 0)
    for (Int y = 0; y < iMarginY; y++ )
    {
      ::memcpy( pi - (y+1)*iStride, pi, sizeof(Pel)*(iWidth + (iMarginX<<1)) );
    }
  }

  m_bIsBorderExtended = true;
}



//NOTE: RExt - This function is never called
Void TComPicYuv::dump (const Char* pFileName, Bool bAdd) const
{
  FILE* pFile;
  if (!bAdd)
  {
    pFile = fopen (pFileName, "wb");
  }
  else
  {
    pFile = fopen (pFileName, "ab");
  }


  for(Int chan = 0; chan < getNumberValidComponents(); chan++)
  {
    const ComponentID  ch     = ComponentID(chan);
    const Int          shift  = g_bitDepth[toChannelType(ch)] - 8;
    const Int          offset = (shift>0)?(1<<(shift-1)):0;
    const Pel         *pi     = getAddr(ch);
    const Int          stride = getStride(ch);
    const Int          height = getHeight(ch);
    const Int          width  = getWidth(ch);

    for (Int y = 0; y < height; y++ )
    {
      for (Int x = 0; x < width; x++ )
      {
        UChar uc = (UChar)Clip3<Pel>(0, 255, (pi[x]+offset)>>shift);
        fwrite( &uc, sizeof(UChar), 1, pFile );
      }
      pi += stride;
    }
  }

  fclose(pFile);
}
#if SCM__R0147_RGB_YUV_RD_ENC
Void TComPicYuv::DefaultConvertPix(TComPicYuv* pcSrcPicYuv)
{
  assert(m_iPicWidth       == pcSrcPicYuv->m_iPicWidth);
  assert(m_iPicHeight      == pcSrcPicYuv->m_iPicHeight);
  assert(m_chromaFormatIDC == CHROMA_444);

  Int  iMaxLuma   = (1<<g_bitDepth[CHANNEL_TYPE_LUMA])   - 1;
  Int  iMaxChroma = (1<<g_bitDepth[CHANNEL_TYPE_CHROMA]) - 1;
  Int  iChromaOffset = (1<<(g_bitDepth[CHANNEL_TYPE_CHROMA]-1));

  Pel* pSrc0  = pcSrcPicYuv->getAddr(COMPONENT_Y);
  Pel* pSrc1  = pcSrcPicYuv->getAddr(COMPONENT_Cb);
  Pel* pSrc2  = pcSrcPicYuv->getAddr(COMPONENT_Cr);

  Pel* pDst0  = getAddr(COMPONENT_Y);
  Pel* pDst1  = getAddr(COMPONENT_Cb);
  Pel* pDst2  = getAddr(COMPONENT_Cr);

  const Int  iSrcStride0 = pcSrcPicYuv->getStride(COMPONENT_Y);
  const Int  iSrcStride1 = pcSrcPicYuv->getStride(COMPONENT_Cb);
  const Int  iSrcStride2 = pcSrcPicYuv->getStride(COMPONENT_Cr);

  const Int  iDstStride0 = getStride(COMPONENT_Y);
  const Int  iDstStride1 = getStride(COMPONENT_Cb);
  const Int  iDstStride2 = getStride(COMPONENT_Cr);

  for(Int y = 0; y < m_iPicHeight; y++) 
  {
    for(Int x = 0; x < m_iPicWidth; x++) 
    {
      Int r, g, b;
      r = pSrc2[x];
      g = pSrc0[x];
      b = pSrc1[x];

      pDst0[x] = ((g<<1) + r+b + 2)>>2;                     
      pDst1[x] = ((((g<<1)-r-b + 2)>>2) + iChromaOffset);   
      pDst2[x] = ((((r-b)+1)>>1) + iChromaOffset);          

      pDst0[x] = Clip3( 0, iMaxLuma,   Int(pDst0[x]) );   
      pDst1[x] = Clip3( 0, iMaxChroma, Int(pDst1[x]) );   
      pDst2[x] = Clip3( 0, iMaxChroma, Int(pDst2[x]) );   
    }

    pSrc0 += iSrcStride0;
    pSrc1 += iSrcStride1;
    pSrc2 += iSrcStride2;

    pDst0 += iDstStride0;
    pDst1 += iDstStride1;
    pDst2 += iDstStride2; 
  }
}
#endif
//! \}
