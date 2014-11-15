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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/

#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComInterpolationFilter.h"

//! \ingroup TLibCommon
//! \{

TComYuv::TComYuv()
{
  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_apiBuf[comp] = NULL;
  }
}

TComYuv::~TComYuv()
{
}

Void TComYuv::create( UInt iWidth, UInt iHeight, ChromaFormat chromaFormatIDC )
{
  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_chromaFormatIDC = chromaFormatIDC;

  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    // memory allocation
    m_apiBuf[ch]  = (Pel*)xMalloc( Pel, getWidth(ComponentID(ch))*getHeight(ComponentID(ch)) );
  }
}

Void TComYuv::destroy()
{
  // memory free
  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    if (m_apiBuf[ch]!=NULL) { xFree( m_apiBuf[ch] ); m_apiBuf[ch] = NULL; }
  }
}

Void TComYuv::clear()
{
  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    if (m_apiBuf[ch]!=NULL)
      ::memset( m_apiBuf[ch], 0, ( getWidth(ComponentID(ch)) * getHeight(ComponentID(ch))  )*sizeof(Pel) );
  }
}




Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, const UInt ctuRsAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyToPicComponent  ( ComponentID(ch), pcPicYuvDst, ctuRsAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
}

Void TComYuv::copyToPicComponent  ( const ComponentID ch, TComPicYuv* pcPicYuvDst, const UInt ctuRsAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  const Int iWidth  = getWidth(ch) >>uiPartDepth;
  const Int iHeight = getHeight(ch)>>uiPartDepth;

  const Pel* pSrc     = getAddr(ch, uiPartIdx, iWidth);
        Pel* pDst     = pcPicYuvDst->getAddr ( ch, ctuRsAddr, uiAbsZorderIdx );

  const UInt  iSrcStride  = getStride(ch);
  const UInt  iDstStride  = pcPicYuvDst->getStride(ch);

  for ( Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyFromPicYuv   ( const TComPicYuv* pcPicYuvSrc, const UInt ctuRsAddr, const UInt uiAbsZorderIdx )
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyFromPicComponent  ( ComponentID(ch), pcPicYuvSrc, ctuRsAddr, uiAbsZorderIdx );
}

Void TComYuv::copyFromPicComponent  ( const ComponentID ch, const TComPicYuv* pcPicYuvSrc, const UInt ctuRsAddr, const UInt uiAbsZorderIdx )
{
        Pel* pDst     = getAddr(ch);
  const Pel* pSrc     = pcPicYuvSrc->getAddr ( ch, ctuRsAddr, uiAbsZorderIdx );

  const UInt iDstStride  = getStride(ch);
  const UInt iSrcStride  = pcPicYuvSrc->getStride(ch);
  const Int  iWidth=getWidth(ch);
  const Int  iHeight=getHeight(ch);

  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyToPartComponent  ( ComponentID(ch), pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToPartComponent( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  const Pel* pSrc     = getAddr(ch);
        Pel* pDst     = pcYuvDst->getAddr( ch, uiDstPartIdx );

  const UInt iSrcStride  = getStride(ch);
  const UInt iDstStride  = pcYuvDst->getStride(ch);
  const Int  iWidth=getWidth(ch);
  const Int  iHeight=getHeight(ch);

  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyPartToComponent  ( ComponentID(ch), pcYuvDst, uiSrcPartIdx );
}

Void TComYuv::copyPartToComponent( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  const Pel* pSrc     = getAddr(ch, uiSrcPartIdx);
        Pel* pDst     = pcYuvDst->getAddr(ch, 0 );

  const UInt  iSrcStride  = getStride(ch);
  const UInt  iDstStride  = pcYuvDst->getStride(ch);

  const UInt uiHeight = pcYuvDst->getHeight(ch);
  const UInt uiWidth = pcYuvDst->getWidth(ch);

  for ( UInt y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidth, const UInt iHeight ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyPartToPartComponent   (ComponentID(ch), pcYuvDst, uiPartIdx, iWidth>>getComponentScaleX(ComponentID(ch)), iHeight>>getComponentScaleY(ComponentID(ch)) );
}

Void TComYuv::copyPartToPartComponent  ( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidthComponent, const UInt iHeightComponent ) const
{
  const Pel* pSrc =           getAddr(ch, uiPartIdx);
        Pel* pDst = pcYuvDst->getAddr(ch, uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(ch);
  const UInt  iDstStride = pcYuvDst->getStride(ch);
  for ( UInt y = iHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidthComponent * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::copyPartToPartComponentMxN  ( const ComponentID ch, TComYuv* pcYuvDst, const TComRectangle &rect) const
{
  const Pel* pSrc =           getAddrPix( ch, rect.x0, rect.y0 );
        Pel* pDst = pcYuvDst->getAddrPix( ch, rect.x0, rect.y0 );
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(ch);
  const UInt  iDstStride = pcYuvDst->getStride(ch);
  const UInt uiHeightComponent=rect.height;
  const UInt uiWidthComponent=rect.width;
  for ( UInt y = uiHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, uiWidthComponent * sizeof( Pel ) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::addClip( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(ch);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(ch);

    const Pel* pSrc0 = pcYuvSrc0->getAddr(ch, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr(ch, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr(ch, uiTrUnitIdx, uiPartWidth );

    const UInt iSrc0Stride = pcYuvSrc0->getStride(ch);
    const UInt iSrc1Stride = pcYuvSrc1->getStride(ch);
    const UInt iDstStride  = getStride(ch);
    const Int clipbd = g_bitDepth[toChannelType(ch)];
#if O0043_BEST_EFFORT_DECODING
    const Int bitDepthDelta = g_bitDepthInStream[toChannelType(ch)] - g_bitDepth[toChannelType(ch)];
#endif

    for ( Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for ( Int x = uiPartWidth-1; x >= 0; x-- )
      {
#if O0043_BEST_EFFORT_DECODING
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + rightShiftEvenRounding<Pel>(pSrc1[x], bitDepthDelta), clipbd));
#else
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + Int(pSrc1[x]), clipbd));
#endif
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::subtract( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(ch);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(ch);

    const Pel* pSrc0 = pcYuvSrc0->getAddr( ch, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr( ch, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr( ch, uiTrUnitIdx, uiPartWidth );

    const Int  iSrc0Stride = pcYuvSrc0->getStride(ch);
    const Int  iSrc1Stride = pcYuvSrc1->getStride(ch);
    const Int  iDstStride  = getStride(ch);

    for (Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for (Int x = uiPartWidth-1; x >= 0; x-- )
      {
        pDst[x] = pSrc0[x] - pSrc1[x];
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::addAvg( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt iPartUnitIdx, const UInt uiWidth, const UInt uiHeight )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel* pSrc0  = pcYuvSrc0->getAddr( ch, iPartUnitIdx );
    const Pel* pSrc1  = pcYuvSrc1->getAddr( ch, iPartUnitIdx );
    Pel* pDst   = getAddr( ch, iPartUnitIdx );

    const UInt  iSrc0Stride = pcYuvSrc0->getStride(ch);
    const UInt  iSrc1Stride = pcYuvSrc1->getStride(ch);
    const UInt  iDstStride  = getStride(ch);
    const Int   clipbd      = g_bitDepth[toChannelType(ch)];
    const Int   shiftNum    = std::max<Int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
    const Int   offset      = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

    const Int   iWidth      = uiWidth  >> getComponentScaleX(ch);
    const Int   iHeight     = uiHeight >> getComponentScaleY(ch);

    if (iWidth&1)
    {
      assert(0);
      exit(-1);
    }
    else if (iWidth&2)
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=2 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
    else
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=4 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
          pDst[ x + 2 ] = ClipBD( rightShift(( pSrc0[ x + 2 ] + pSrc1[ x + 2 ] + offset ), shiftNum), clipbd );
          pDst[ x + 3 ] = ClipBD( rightShift(( pSrc0[ x + 3 ] + pSrc1[ x + 3 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
  }
}

Void TComYuv::removeHighFreq( const TComYuv* pcYuvSrc, const UInt uiPartIdx, const UInt uiWidth, UInt const uiHeight )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
#if !DISABLING_CLIP_FOR_BIPREDME
    const ChannelType chType=toChannelType(ch);
#endif

    const Pel* pSrc  = pcYuvSrc->getAddr(ch, uiPartIdx);
    Pel* pDst  = getAddr(ch, uiPartIdx);

    const Int iSrcStride = pcYuvSrc->getStride(ch);
    const Int iDstStride = getStride(ch);
    const Int iWidth  = uiWidth >>getComponentScaleX(ch);
    const Int iHeight = uiHeight>>getComponentScaleY(ch);

    for ( Int y = iHeight-1; y >= 0; y-- )
    {
      for ( Int x = iWidth-1; x >= 0; x-- )
      {
#if DISABLING_CLIP_FOR_BIPREDME
        pDst[x ] = (2 * pDst[x]) - pSrc[x];
#else
        pDst[x ] = Clip((2 * pDst[x]) - pSrc[x], chType);
#endif
      }
      pSrc += iSrcStride;
      pDst += iDstStride;
    }
  }
}

Void TComYuv::convert(const UInt uiPixX, const UInt uiPixY, const UInt uiWidth, Bool bForwardConversion, Bool bLossless, TComYuv* pcYuvNoCorrResi)
{
  assert(getChromaFormat() == CHROMA_444);
  UInt uiPartSize = uiWidth;
#if !SCM_S0180_ACT_BIT_DEPTH_ALIGN
  const Int iRound = 2;
#endif 

  Pel* pOrg0  = getAddrPix( (ComponentID)0, uiPixX, uiPixY );
  Pel* pOrg1  = getAddrPix( (ComponentID)1, uiPixX, uiPixY );
  Pel* pOrg2  = getAddrPix( (ComponentID)2, uiPixX, uiPixY );

  Pel* pDst0  = getAddrPix( (ComponentID)0, uiPixX, uiPixY );
  Pel* pDst1  = getAddrPix( (ComponentID)1, uiPixX, uiPixY );
  Pel* pDst2  = getAddrPix( (ComponentID)2, uiPixX, uiPixY );

  if(pcYuvNoCorrResi)
  {
    pDst0  = pcYuvNoCorrResi->getAddrPix( (ComponentID)0, uiPixX, uiPixY );
    pDst1  = pcYuvNoCorrResi->getAddrPix( (ComponentID)1, uiPixX, uiPixY );
    pDst2  = pcYuvNoCorrResi->getAddrPix( (ComponentID)2, uiPixX, uiPixY );
  }
  const Int  iStride0 = getStride((ComponentID)0);
  const Int  iStride1 = getStride((ComponentID)1);
  const Int  iStride2 = getStride((ComponentID)2);

  if(bForwardConversion)
  {
    if(!bLossless)
    {
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
      Int maxBitDepth  = std::max(g_bitDepth[CHANNEL_TYPE_LUMA], g_bitDepth[CHANNEL_TYPE_CHROMA]);
      Int iShiftLuma   = maxBitDepth - g_bitDepth[CHANNEL_TYPE_LUMA];
      Int iShiftChroma = maxBitDepth - g_bitDepth[CHANNEL_TYPE_CHROMA];
      Int iRoundLuma   = 1<<(1+iShiftLuma);
      Int iRoundChroma = 1<<(1+iShiftChroma);
#endif 
      for(Int y=0; y<uiPartSize; y++) 
      { 
        for(Int x=0; x<uiPartSize; x++) 
        {
          Int r, g, b;
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
          r = pOrg2[x]<<iShiftChroma;
          g = pOrg0[x]<<iShiftLuma;
          b = pOrg1[x]<<iShiftChroma;
#else
          r = pOrg2[x];
          g = pOrg0[x];
          b = pOrg1[x];
#endif

          pDst0[x] = (g<<1) +r+b  ;   
          pDst1[x] = (g<<1) -r-b  ;   
          pDst2[x] = ((r-b) << 1) ;
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
          pDst0[x] = (pDst0[x] + iRoundLuma)   >> (2+iShiftLuma);
          pDst1[x] = (pDst1[x] + iRoundChroma) >> (2+iShiftChroma);
          pDst2[x] = (pDst2[x] + iRoundChroma) >> (2+iShiftChroma);
#else
          pDst0[x] = (pDst0[x] + iRound) >> 2;
          pDst1[x] = (pDst1[x] + iRound) >> 2;
          pDst2[x] = (pDst2[x] + iRound) >> 2;
#endif
        }
        pOrg0 += iStride0;
        pOrg1 += iStride1;
        pOrg2 += iStride2;
        pDst0 += iStride0;
        pDst1 += iStride1;
        pDst2 += iStride2;
      }
    }
    else
    {
      for(Int y=0; y<uiPartSize; y++)
      {
        for(Int x=0; x<uiPartSize; x++)
        {
          Int r, g, b;
          r = pOrg2[x];
          g = pOrg0[x];
          b = pOrg1[x];

          Int Co = r-b;
          Int t = b + (Co>>1);
          Int Cg = g - t;
          pDst0[x] = t + (Cg>>1);
          pDst1[x] = Cg;
          pDst2[x] = Co;
        }
        pOrg0 += iStride0;
        pOrg1 += iStride1;
        pOrg2 += iStride2;
        pDst0 += iStride0;
        pDst1 += iStride1;
        pDst2 += iStride2;
      }
    }
  }
  else
  {
#if SCM_S0254_ACT_UNIFICATION==0
    if(!bLossless)
    {
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
      Int maxBitDepth  = std::max(g_bitDepth[CHANNEL_TYPE_LUMA], g_bitDepth[CHANNEL_TYPE_CHROMA]);
      Int iShiftLuma   = maxBitDepth - g_bitDepth[CHANNEL_TYPE_LUMA];
      Int iShiftChroma = maxBitDepth - g_bitDepth[CHANNEL_TYPE_CHROMA];
      Int iRoundLuma   = (iShiftLuma == 0)?0: (1<<(iShiftLuma-1));
      Int iRoundChroma = (iShiftChroma == 0)?0: (1<<(iShiftChroma-1));
#endif
      for(Int y=0; y<uiPartSize; y++) 
      { 
        for(Int x=0; x<uiPartSize; x++) 
        { 
          Int y0, cg, co;
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
          y0 = pOrg0[x]<<iShiftLuma; 
          cg = pOrg1[x]<<iShiftChroma; 
          co = pOrg2[x]<<iShiftChroma;

          pDst0[x] = ( y0 + cg      + iRoundLuma   )>>iShiftLuma;
          pDst1[x] = ( y0 - cg - co + iRoundChroma )>>iShiftChroma;
          pDst2[x] = ( y0 - cg + co + iRoundChroma )>>iShiftChroma;
#else
          y0 = pOrg0[x]; 
          cg = pOrg1[x]; 
          co = pOrg2[x];

          pDst0[x] = ( y0 + cg      );
          pDst1[x] = ( y0 - cg - co );
          pDst2[x] = ( y0 - cg + co );
#endif
        }

        pOrg0 += iStride0;
        pOrg1 += iStride1;
        pOrg2 += iStride2;
        pDst0 += iStride0;
        pDst1 += iStride1;
        pDst2 += iStride2; 
      }
    }
    else
#endif
    {
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN && SCM_S0254_ACT_UNIFICATION
      Int maxBitDepth  = std::max(g_bitDepth[CHANNEL_TYPE_LUMA], g_bitDepth[CHANNEL_TYPE_CHROMA]);
      Int iShiftLuma   = maxBitDepth - g_bitDepth[CHANNEL_TYPE_LUMA];
      Int iShiftChroma = maxBitDepth - g_bitDepth[CHANNEL_TYPE_CHROMA];
      Int iRoundLuma   = iShiftLuma? (1<<(iShiftLuma-1)): 0;
      Int iRoundChroma = iShiftChroma? (1<<(iShiftChroma-1)): 0;
#endif
      for(Int y=0; y<uiPartSize; y++)
      {
        for(Int x=0; x<uiPartSize; x++)
        {
          Int y0, cg, co;
          y0 = pOrg0[x];
          cg = pOrg1[x];
          co = pOrg2[x];
#if SCM_S0254_ACT_UNIFICATION
          if(!bLossless)
          {
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
            y0 <<= iShiftLuma;
            cg <<= (iShiftChroma + 1);
            co <<= (iShiftChroma + 1);
#else
            cg <<= 1;
            co <<= 1;
#endif
          }
#endif
          Int t = y0 - (cg>>1);
          pDst0[x] = cg + t;
          pDst1[x] = t - (co>>1);
          pDst2[x] = co + pDst1[x];
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN && SCM_S0254_ACT_UNIFICATION
          if(!bLossless)
          {
            pDst0[x] = (pDst0[x] + iRoundLuma)   >> iShiftLuma;
            pDst1[x] = (pDst1[x] + iRoundChroma) >> iShiftChroma;
            pDst2[x] = (pDst2[x] + iRoundChroma) >> iShiftChroma;
          }
#endif
        }
        pOrg0 += iStride0;
        pOrg1 += iStride1;
        pOrg2 += iStride2;
        pDst0 += iStride0;
        pDst1 += iStride1;
        pDst2 += iStride2;
      }
    }
  }
}


Void TComYuv::DefaultConvertPix(const UInt uiPixX, const UInt uiPixY, const UInt uiWidth)
{
  assert(getChromaFormat() == CHROMA_444);
  UInt uiPartSize = uiWidth;
  Int  iMaxLuma   = (1<<g_bitDepth[CHANNEL_TYPE_LUMA])   - 1;
  Int  iMaxChroma = (1<<g_bitDepth[CHANNEL_TYPE_CHROMA]) - 1;
  Int  iChromaOffset = (1<<(g_bitDepth[CHANNEL_TYPE_CHROMA]-1));
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
  Int maxBitDepth  = std::max(g_bitDepth[CHANNEL_TYPE_LUMA], g_bitDepth[CHANNEL_TYPE_CHROMA]);
  Int iShiftLuma   = maxBitDepth - g_bitDepth[CHANNEL_TYPE_LUMA];
  Int iShiftChroma = maxBitDepth - g_bitDepth[CHANNEL_TYPE_CHROMA];
  Int iRoundLuma   = 1<<(1+iShiftLuma);
  Int iRoundChroma = 1<<(1+iShiftChroma);
#endif

  Pel* pDst0  = getAddrPix( (ComponentID)0, uiPixX, uiPixY );
  Pel* pDst1  = getAddrPix( (ComponentID)1, uiPixX, uiPixY );
  Pel* pDst2  = getAddrPix( (ComponentID)2, uiPixX, uiPixY );

  const Int  iStride0 = getStride((ComponentID)0);
  const Int  iStride1 = getStride((ComponentID)1);
  const Int  iStride2 = getStride((ComponentID)2);

  for(Int y=0; y<uiPartSize; y++) 
  {
    for(Int x=0; x<uiPartSize; x++) 
    {
      Int r, g, b;
#if SCM_S0180_ACT_BIT_DEPTH_ALIGN
      r = pDst2[x]<<iShiftChroma;
      g = pDst0[x]<<iShiftLuma;
      b = pDst1[x]<<iShiftChroma;

      pDst0[x] = ((g<<1)+r+b + iRoundLuma)>>(2+iShiftLuma);
      pDst1[x] = ((g<<1)-r-b + iRoundChroma)>>(2+iShiftChroma);
      pDst2[x] = (((r-b)<<1)  + iRoundChroma)>>(2+iShiftChroma);

      pDst1[x] += iChromaOffset;
      pDst2[x] += iChromaOffset;
#else
      r = pDst2[x];
      g = pDst0[x];
      b = pDst1[x];

      pDst0[x] = ((g<<1) + r+b + 2)>>2;
      pDst1[x] = ((((g<<1)-r-b + 2)>>2) + iChromaOffset);
      pDst2[x] = ((((r-b)+1)>>1) + iChromaOffset);
#endif 
      pDst0[x] = Clip3( 0, iMaxLuma,   Int(pDst0[x]) );
      pDst1[x] = Clip3( 0, iMaxChroma, Int(pDst1[x]) );
      pDst2[x] = Clip3( 0, iMaxChroma, Int(pDst2[x]) );
    }

    pDst0 += iStride0;
    pDst1 += iStride1;
    pDst2 += iStride2; 
  }
}

//! \}
