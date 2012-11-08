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


//NOTE: ECF - This function is never called
Void TComPicYuv::getMinMax(const ComponentID id, Int *pMin, Int *pMax ) const
{
  const Pel*  piY   = getAddr(id);
#if FULL_NBIT
  Int   iMin  = (1<<(g_uiBitDepth))-1;
#else
  Int   iMin  = (1<<(g_uiBitDepth + g_uiBitIncrement))-1;
#endif
  Int   iMax  = 0;
  Int   x, y;
  const Int stride=getStride(id);
  const Int height=getHeight(id);
  const Int width=getWidth(id);
  
  for ( y = 0; y < height; y++ )
  {
    for ( x = 0; x < width; x++ )
    {
      if ( piY[x] < iMin ) iMin = piY[x];
      if ( piY[x] > iMax ) iMax = piY[x];
    }
    piY += stride;
  }
  
  *pMin = iMin;
  *pMax = iMax;
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



//NOTE: ECF - This function is never called
Void TComPicYuv::dump (const char* pFileName, Bool bAdd) const
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
  
  const Int  shift = g_uiBitIncrement;
  const Int  offset = (shift>0)?(1<<(shift-1)):0;
#if FULL_NBIT
  const Pel  iMax = ((1<<(g_uiBitDepth))-1);
#else
  const Pel  iMax = ((1<<(g_uiBitDepth + g_uiBitIncrement))-1);
#endif

  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel*  pi   = getAddr(ch);
    const Int  stride=getStride(ch);
    const Int  height=getHeight(ch);
    const Int  width= getWidth(ch);

    for (Int y = 0; y < height; y++ )
    {
      for (Int x = 0; x < width; x++ )
      {
        UChar uc = (UChar)Clip3<Pel>(0, iMax, (pi[x]+offset)>>shift);
        fwrite( &uc, sizeof(UChar), 1, pFile );
      }
      pi += stride;
    }
  }
  
  fclose(pFile);
}

#if FIXED_ROUNDING_FRAME_MEMORY
Void TComPicYuv::xFixedRoundingPic()
{
  const UInt numberValidComponents = getNumberValidComponents(getChromaFormat());
#if FULL_NBIT
  const Int  iOffset               = ((g_uiBitDepth-8)>0)?(1<<(g_uiBitDepth-8-1)):0;
  const Int  iMask                 = (~0<<(g_uiBitDepth-8));
#if (IBDI_NOCLIP_RANGE == 0)
  const Int  iMaxBdi               = g_uiBASE_MAX<<(g_uiBitDepth-8);
#endif
#else
  const Int  iOffset               = (g_uiBitIncrement>0)?(1<<(g_uiBitIncrement-1)):0;
  const Int  iMask                 = (~0<<g_uiBitIncrement);
#if (IBDI_NOCLIP_RANGE == 0)
  const Int  iMaxBdi               = g_uiBASE_MAX<<g_uiBitIncrement;
#endif

  for (UInt component = 0; component < numberValidComponents; component++)
  {
    const ComponentID  compID  = ComponentID(component);
          Pel         *pRec    = getAddr(compID);
    const Int          iStride = getStride(compID);
    const Int          iWidth  = getWidth(compID);
    const Int          iHeight = getHeight(compID);
#endif

    for(Int y = 0; y < iHeight; y++ )
    {
      for(Int x = 0; x < iWidth; x++ )
      {
#if IBDI_NOCLIP_RANGE
        pRec[x] = ( pRec[x] + iOffset ) & iMask;
#else
        pRec[x] = ( pRec[x]+iOffset>iMaxBdi)? iMaxBdi : ((pRec[x]+iOffset) & iMask);
#endif
      }
      pRec += iStride;
    }
  }
}
#endif

//! \}
