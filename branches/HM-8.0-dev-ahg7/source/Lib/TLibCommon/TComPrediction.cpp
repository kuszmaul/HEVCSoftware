/* The copyright in this software is beinOMg made available under the BSD
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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
  },
  { // Chroma
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
  }

};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
}

TComPrediction::~TComPrediction()
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
  }
  
  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC)
{
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = g_uiMaxCUWidth + 16; 
    Int extHeight = g_uiMaxCUHeight + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (g_uiMaxCUHeight*2+1) * (g_uiMaxCUWidth*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acYuvPred[i] .create( g_uiMaxCUWidth, g_uiMaxCUHeight, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( g_uiMaxCUWidth, g_uiMaxCUHeight, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (g_uiMaxCUWidth>>1) + 1)
  {
    m_iLumaRecStride =  (g_uiMaxCUWidth>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }

#if !REMOVE_LMCHROMA
  Int shift = g_uiBitDepth + g_uiBitIncrement + 4;

  for( Int i = 32; i < 64; i++ )
  {
    m_uiaShift[i-32] = ( ( 1 << shift ) + i/2 ) / i;
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, ChannelType channelType, ChromaFormat format, Bool bAbove, Bool bLeft )
{
  Int iInd, iSum = 0;
  Pel pDcVal;

  if (bAbove)
  {
    for (iInd = 0;iInd < iWidth;iInd++)
    {
      iSum += pSrc[iInd-iSrcStride];
    }

    if (doubleWeightIntraDCAboveSamples(channelType, format))
    {
      iSum   <<= 1;
      iWidth <<= 1;
    }
  }
  if (bLeft)
  {
    for (iInd = 0;iInd < iHeight;iInd++)
    {
      iSum += pSrc[iInd*iSrcStride-1];
    }
  }

  if (bAbove && bLeft)
  {
    pDcVal = (iSum + iWidth) / (iWidth + iHeight);
  }
  else if (bAbove)
  {
    pDcVal = (iSum + iWidth/2) / iWidth;
  }
  else if (bLeft)
  {
    pDcVal = (iSum + iHeight/2) / iHeight;
  }
  else
  {
    pDcVal = pSrc[-1]; // Default DC value already calculated and placed in the prediction array if no neighbors are available
  }
  
  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 * \param dirMode the intra prediction mode index
 * \param blkAboveAvailable boolean indication if the block above is available
 * \param blkLeftAvailable boolean indication if the block to the left is available
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
Void TComPrediction::xPredIntraAng( const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType, ChromaFormat format,
                                          UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable )
{
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar
  const Bool modeDC        = dirMode==DC_IDX;

  // Do the DC prediction
  if (modeDC)
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, channelType, format, blkAboveAvailable, blkLeftAvailable);

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;
      }
    }
  }
  else // Do angular predictions
  {
    const Bool       bIsModeVer         = (dirMode >= 18);
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
    const Int        absAngMode         = abs(intraPredAngleMode);
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
    const FilterMode filterMode         = getIntraEdgeFilterMode(channelType, format);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;

    if ((channelType == CHANNEL_TYPE_CHROMA) && (format == CHROMA_422))
    {
      intraPredAngle = bIsModeVer ? (intraPredAngle>>1) : 2*intraPredAngle;
      invAngle       = bIsModeVer ? 2*invAngle          : (invAngle>>1);
    }

    Pel* refMain;
    Pel* refSide;

    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];
    
    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;
      const Int refMainOffset         = height - 1;
      for (Int x=0;x<width+1;x++)
      {
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<height+1;y++)
      {
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];
      }
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (Int x=0;x<2*width+1;x++)
      {
        refAbove[x] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
      refMain = bIsModeVer ? refAbove : refLeft ;
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal
    {
      for (Int y=0;y<height;y++)
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];
        }
      }

      if ((filterMode == FILTER_BOTH_DIRECTIONS) || ((bIsModeVer) ? (filterMode == FILTER_HORIZONTAL_ONLY) : (filterMode == FILTER_VERTICAL_ONLY)))
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip ( pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
      {
        const Int deltaInt   = deltaPos >> 5;
        const Int deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
          const Pel *pRM=refMain+deltaInt+1;
          Int lastRefMainPel=*pRM++;
          for (Int x=0;x<width;pRM++,x++)
          {
            Int thisRefMainPel=*pRM;
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );
            lastRefMainPel=thisRefMainPel;
          }
        }
        else
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (!bIsModeVer)
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}


#ifdef ECF__NON_SCALED_INTRA_CHROMA_422_ENABLED

Void TComPrediction::xPredIntraAngChroma422( const Pel* pSrc, Int srcStride, Pel*& rpDst, Int dstStride, UInt width, UInt height, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable )
{
  Int k,l;
  Pel* pDst          = rpDst;

  // Map the mode index to main prediction direction and angle
  assert( dirMode > 0 ); //no planar
  Bool modeDC        = dirMode < 2;
  Bool modeHor       = !modeDC && (dirMode < 18);
  Bool modeVer       = !modeDC && !modeHor;
  Int intraPredAngle = modeVer ? (Int)dirMode - VER_IDX : modeHor ? -((Int)dirMode - HOR_IDX) : 0;
  Int absAng         = abs(intraPredAngle);
  Int signAng        = intraPredAngle < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
  Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
  Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
  Int invAngle       = invAngTable[absAng];
  absAng             = angTable[absAng];
  const Int iShift         = 5;
  const Int iInt           = (1<<iShift);
  const Int iMask          = iInt-1;
  const Int iAdd           = (1<<(iShift-1));
  intraPredAngle     = signAng * absAng;

  // Do the DC prediction
  if (modeDC)
  {
    Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height, CHANNEL_TYPE_CHROMA, CHROMA_422, blkAboveAvailable, blkLeftAvailable);

    for (k=0;k<height;k++)
    {
      for (l=0;l<width;l++)
      {
        pDst[k*dstStride+l] = dcval;
      }
    }
  }
  // Do angular predictions
  else
  {
    Pel* refMain;
    Pel* refSide;
    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialise the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      for (k=0;k<width+1;k++)
      {
        refAbove[k+height-1] = pSrc[k-srcStride-1];
      }
      for (k=0;k<height+1;k++)
      {
        refLeft[k+width-1] = pSrc[(k-1)*srcStride-1];
      }
      refMain = (modeVer ? (refAbove+height-1) : (refLeft+width-1));
      refSide = (modeVer ? (refLeft+width-1) : (refAbove+height-1));
      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      Int size = (modeVer ? height : width);
      for (k=-1; k>size*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
      for (k=0;k<width+height+1;k++)
      {
        refAbove[k] = pSrc[k-srcStride-1];
      }
      for (k=0;k<height+width+1;k++)
      {
        refLeft[k] = pSrc[(k-1)*srcStride-1];
      }
      refMain = modeVer ? refAbove : refLeft;
      refSide = modeVer ? refLeft  : refAbove;
    }

    if (intraPredAngle == 0)
    {
      if (modeVer){
        const Int step = 1;
        for (k=0;k<height;k++)
        {
          for (l=0;l<width;l++)
          {
            pDst[k*dstStride+l] = refMain[(l+1)*step];
          }
        }
      }
      else{
        for (k=0;k<height;k++)
        {
          for (l=0;l<width;l++)
          {
            pDst[k*dstStride+l] = refMain[k+1];
          }
        }
      }
    }
    else
    {
      Int deltaPos=0;
      Int deltaInt;
      Int deltaFract;
      Int refMainIndex;

      const Int step = 1;
      if (modeVer){
        for (k=0;k<height;k++)
        {
          deltaPos += intraPredAngle;
          deltaInt   = deltaPos >> iShift;
          deltaFract = deltaPos & iMask;

          if (deltaFract)
          {
            // Do linear filtering
            for (l=0;l<width;l++)
            {
              refMainIndex        = l*step+deltaInt+step;
              pDst[k*dstStride+l] = (Pel) ( ((iInt-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+iAdd) >> iShift );
            }
          }
          else
          {
            // Just copy the integer samples
            for (l=0;l<width;l++)
            {
              pDst[k*dstStride+l] = refMain[l*step+deltaInt+step];
            }
          }
        }
      }
      else{
        for (l=0;l<width;l++)
        {
          deltaPos += intraPredAngle;
          deltaInt   = deltaPos >> iShift;
          deltaFract = deltaPos & iMask;

          if (deltaFract)
          {
            // Do linear filtering
            for (k=0;k<height;k++)
            {
              refMainIndex        = k+deltaInt+1;
              pDst[k*dstStride+l] = (Pel) ( ((iInt-deltaFract)*refMain[refMainIndex]+deltaFract*refMain[refMainIndex+1]+iAdd) >> iShift );
            }
          }
          else
          {
            // Just copy the integer samples
            for (k=0;k<height;k++)
            {
              pDst[k*dstStride+l] = refMain[k+deltaInt+1];
            }
          }
        }
      }
    }
  }
}

#endif


Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piPred, UInt uiStride, TComTU &rTu, Bool bAbove, Bool bLeft, const Bool bUseFilteredPredSamples )
{
  const ChromaFormat   format      = rTu.GetChromaFormat();
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;
  const Int            iHeight     = rect.height;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
  //assert( iWidth == iHeight  );

        Pel *pDst = piPred;
  const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples ) ;

  // get starting pixel in block
  const Int sw = (nonScaledIntraChroma422(channelType, format) ? (iWidth + iHeight + 1) : (2 * iWidth + 1));

  if ( uiDirMode == PLANAR_IDX )
  {
    xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format );
  }
  else
  {
#ifdef ECF__NON_SCALED_INTRA_CHROMA_422_ENABLED
    if (nonScaledIntraChroma422(channelType, format))
    {
      // Create the prediction
      xPredIntraAngChroma422( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode, bAbove, bLeft );
    }
    else
#endif
    {
      // Create the prediction
      xPredIntraAng( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format, uiDirMode, bAbove, bLeft );
    }

    if(( uiDirMode == DC_IDX ) && bAbove && bLeft )
    {
      xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format );
    }
  }
}

/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}


Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred, iPartIdx );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred, iPartIdx );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartInter(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx );
      }
      xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx );
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, iPartIdx );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred, iPartIdx );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred, iPartIdx );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Int iPartIdx, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);

  for (UInt ch=COMPONENT_Y; ch<rpcYuvPred->getNumberValidComponents(); ch++)
    xPredInterBlk  (ComponentID(ch),  pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, rpcYuvPred, bi );
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvPred, Int iPartIdx )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};

  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[refList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[refList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, iPartIdx, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, iPartIdx, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, iPartIdx );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, rpcYuvPred, iPartIdx );
  }
  else
  {
    xWeightedAverage( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, rpcYuvPred );
  }
}

/**
 * \brief Generate motion-compensated block
 *
 * \param cu       Pointer to current CU
 * \param refPic   Pointer to reference picture
 * \param partAddr Address of block within CU
 * \param mv       Motion vector
 * \param width    Width of block
 * \param height   Height of block
 * \param dstPic   Pointer to destination picture
 * \param bi       Flag indicating whether bipred is used
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));
  
  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;
  
  Pel*    ref     = refPic->getAddr(compID, cu->getAddr(), cu->getZorderIdxInCU() + partAddr ) + refOffset;
  
  Pel*    dst = dstPic->getAddr( compID, partAddr );
  
  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);
  
  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();
  
  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt);
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt);
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = useLumaInterpFilter(compID, chFmt, 1) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, !bi, chFmt);
  }
}

Void TComPrediction::xWeightedAverage( TComDataCU* pcCU, TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    rpcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( rpcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcCU->getAMVPMode(uiPartAddr) == AM_NONE || (pcAMVPInfo->iN <= 1 && pcCU->getAMVPMode(uiPartAddr) == AM_EXPL) )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc pointer to reconstructed sample array
 * \param srcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param dstStride the stride of the prediction sample array
 * \param width the width of the block
 * \param height the height of the block
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height, ChannelType channelType, ChromaFormat format )
{
  assert(width <= height);

#if ECF__BACKWARDS_COMPATIBILITY_HM
  Int leftColumn[MAX_CU_SIZE], topRow[MAX_CU_SIZE], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
#else
  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
#endif
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;

  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight   = topRow[width];

  if (intraPlanarSingleStageCalculation(channelType, format))
  {
    for(Int k=0;k<width;k++)
    {
      for(Int l=0;l<height;l++)
      {
        // NOTE: ECF - rounding point changed from 'height' to 'width'.
        // NOTE: ECF - The intermediate shift left could be rolled into the final shift left,
        //             thereby increasing the accuracy of the calculation
        // eg rpDst[l*dstStride+k] = ( (  ((height-l-1)*topRow[k]    +(l+1)*bottomLeft)) +
        //                           (  ((width-k-1)*leftColumn[l]+(k+1)*topRight    )*2    ) + height) >> (shift1Dver+1);
        rpDst[l*dstStride+k] = ( (  ((height-l-1)*topRow[k]    +(l+1)*bottomLeft+1)>>1) +
                                 (  ((width-k-1)*leftColumn[l]+(k+1)*topRight    )    ) + width) >> (shift1Dhor+1);
      }
    }
  }
  else
  {
    // NOTE: ECF - mistakes fixed to match above multiply-based calculation
    for(Int k=0;k<width;k++)
    {
      bottomRow[k]  = bottomLeft - topRow[k];
      topRow[k]     <<= shift1Dver;
    }

    for(Int k=0;k<height;k++)
    {
      rightColumn[k]  = topRight - leftColumn[k];
      leftColumn[k]   <<= shift1Dhor;
    }

    const UInt topRowShift = (isChroma(channelType) && (format == CHROMA_422)) ? 1 : 0;

    // Generate prediction signal
    for (Int y=0;y<height;y++)
    {
      Int horPred = leftColumn[y] + width;
      for (Int x=0;x<width;x++)
      {
        horPred += rightColumn[y];
        topRow[x] += bottomRow[x];

        // NOTE: ECF - The intermediate shift left could be rolled into the final shift left,
        //             thereby increasing the accuracy of the calculation
        // eg  rpDst[y*dstStride+x] = ( (horPred<<topRowShift) + topRow[x] ) >> (shift1Dver+1);
        Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
        rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
      }
    }
  }
}

#if !REMOVE_LMCHROMA
/** Function for deriving chroma LM intra prediction.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param piSrc pointer to reconstructed chroma sample array
 * \param pPred pointer for the prediction sample array
 * \param uiPredStride the stride of the prediction sample array
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 * \param uiChromaId boolean indication of chroma component
 *
 * This function derives the prediction samples for chroma LM mode (chroma intra coding)
 */
Void TComPrediction::predLMIntraChroma( const ComponentID compID, Pel* pPred, UInt uiPredStride, UInt uiTuWidth, UInt uiTuHeight, const ChromaFormat chFmt DEBUG_STRING_FN_DECLARE(sDebug) )
{
  Pel*  piSrc = getPredictorPtr( compID, false ) ;
  const UInt stride = ( nonScaledIntraChroma422(toChannelType(compID), chFmt) ? uiTuWidth + uiTuHeight : 2*uiTuWidth ) + 1;

  xGetLLSPrediction( piSrc+stride+1, stride, pPred, uiPredStride, uiTuWidth, uiTuHeight, 1, chFmt DEBUG_STRING_PASS_INTO(sDebug) );
}

/** Function for deriving downsampled luma sample of current chroma block and its above, left causal pixel
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 *
 * This function derives downsampled luma sample of current chroma block and its above, left causal pixel
 */
Void TComPrediction::getLumaRecPixels( TComTU &rTu )
{
  const ChromaFormat chFmt=rTu.GetChromaFormat();
  const TComRectangle &rectC=rTu.getRect(COMPONENT_Cb);
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiCWidth =rectC.width;
  const UInt uiCHeight=rectC.height;

  const UInt csx = getComponentScaleX(COMPONENT_Cb, chFmt);
  const UInt csy = getComponentScaleY(COMPONENT_Cb, chFmt);

  const UInt uiWidth444  = uiCWidth  << csx;
  const UInt uiHeight444 = uiCHeight << csy;

  TComPicYuv* pPicYuv=pcCU->getPic()->getPicYuvRec();

  Int iRecSrcStride = pPicYuv->getStride(COMPONENT_Y);
  Int iDstStride = m_iLumaRecStride;

  Pel* pRecSrc = pPicYuv->getAddr(COMPONENT_Y, pcCU->getAddr(), rTu.GetAbsPartIdxTU()+pcCU->getZorderIdxInCU());
  Pel* pDst0 = m_pLumaRecBuffer + m_iLumaRecStride + 1;

  assert(uiWidth444==uiHeight444);  // should always be the same!
  Int iSrcStride = ( max( uiWidth444, uiHeight444 ) << 1 ) + 1;

  Pel*  ptrSrc = getPredictorPtr( COMPONENT_Y, false ) ;

  // initial pointers
  Pel* pDst = pDst0 - 1 - iDstStride;
  Pel* piSrc = ptrSrc;

  // top left corner downsampled from ADI buffer
  // don't need this point

  // top row downsampled from ADI buffer
  pDst++;     
  piSrc ++;
  if (csx==1)
  {
    for (Int i = 0; i < uiCWidth; i++)
    {
      pDst[i] = ((piSrc[2*i] * 2 ) + piSrc[2*i - 1] + piSrc[2*i + 1] + 2) >> 2;
    }
  }
  else
  {
    for (Int i = 0; i < uiCWidth; i++)
    {
      pDst[i] = piSrc[i];
    }
  }

  // left column downsampled from ADI buffer
  pDst = pDst0 - 1; 
  piSrc = ptrSrc + iSrcStride;
  if (csy==1)
  {
    for (Int j = 0; j < uiCHeight; j++)
    {
      pDst[0] = ( piSrc[0] + piSrc[iSrcStride] ) >> 1;
      piSrc += iSrcStride << 1;
      pDst += iDstStride;
    }
  }
  else
  {
    for (Int j = 0; j < uiCHeight; j++)
    {
      pDst[0] = piSrc[0];
      piSrc += iSrcStride;
      pDst += iDstStride;
    }
  }

  // inner part from reconstructed picture buffer

  if (csx==0 && csy==0)
  {
    for( Int j = 0; j < uiCHeight; j++ )
    {
      for (Int i = 0; i < uiCWidth; i++)
      {
        pDst0[i] = (pRecSrc[i]);
      }

      pDst0 += iDstStride;
      pRecSrc += iRecSrcStride;
    }
  }
  else
  {
    Int iRecSrcStrideAdj = (iRecSrcStride << csy);
    Int secondSampleOffset = iRecSrcStrideAdj - iRecSrcStride; // for 4:2:0, this is iRecSrcStride. For 4:2:2, this is 0
    for( Int j = 0; j < uiCHeight; j++ )
    {
      for (Int i = 0; i < uiCWidth; i++)
      {
        pDst0[i] = (pRecSrc[i<<csx] + pRecSrc[(i<<csx) + secondSampleOffset]) >> 1;
      }

      pDst0 += iDstStride;
      pRecSrc += iRecSrcStrideAdj;
    }
  }
}

/** Function for deriving the positon of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the positon of first non-zero binary bit of a value
 */
Int GetFloorLog2( UInt x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits ++;
    x >>= 1;
  }
  return bits;
}


/** Function for deriving LM intra prediction.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param pSrc0 pointer to reconstructed chroma sample array
 * \param iSrcStride the stride of reconstructed chroma sample array
 * \param pDst0 reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param uiExt0 line number of neiggboirng pixels for calculating LM model parameter, default value is 1
 *
 * This function derives the prediction samples for chroma LM mode (chroma intra coding)
 */
Void TComPrediction::xGetLLSPrediction(  const Pel* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0, const ChromaFormat chFmt DEBUG_STRING_FN_DECLARE(sDebug) )
{

  Pel  *pDst, *pLuma;
  const Pel  *pSrc;

  Int  iLumaStride = m_iLumaRecStride;
  Pel* pLuma0 = m_pLumaRecBuffer + uiExt0 * iLumaStride + uiExt0;

  Int i, j, iCountShift = 0;
  UInt uiInternalBitDepth = g_uiBitDepth + g_uiBitIncrement;

  UInt uiExt = uiExt0;

  // LLS parameters estimation -->

  Int x = 0, y = 0, xx = 0, xy = 0;

  pSrc  = pSrc0  - iSrcStride;
  pLuma = pLuma0 - iLumaStride;
  const Int csx=(chFmt==CHROMA_422) ? 1:0; // for 4:2:2, double up samples horizontally (in effect).

#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
  std::stringstream ss(stringstream::out);

  ss << "###: ~ top row: " << std::endl;
#endif

  for( j = 0; j < uiWidth; j++ )
  {
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
    ss << "###: ~ pLuma[" << j << "]=" << pLuma[j] << ", pSrc[" << j << "]=" << pSrc[j] << std::endl;
#endif
    x += pLuma[j] << csx;
    y += pSrc[j] << csx;
    xx += (pLuma[j] * pLuma[j]) << csx;
    xy += (pLuma[j] * pSrc[j]) << csx;
  }

  pSrc  = pSrc0 - uiExt;
  pLuma = pLuma0 - uiExt;

#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
  ss << "###: ~ left column: , iSrcStride=" << iSrcStride << ", iLumaStride=" << iLumaStride << std::endl;
#endif

  for( i = 0; i < uiHeight; i++ )
  {
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
    ss << "###: ~ pLuma[0]=" << pLuma[0] << ", pSrc[0]=" << pSrc[0] << std::endl;
#endif
    x += pLuma[0];
    y += pSrc[0];
    xx += pLuma[0] * pLuma[0];
    xy += pLuma[0] * pSrc[0];

    pSrc  += iSrcStride;
    pLuma += iLumaStride;
  }

  iCountShift = g_aucConvertToBit[ uiHeight ] + 3;
  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if(iTempShift > 0)
  {
    x  = ( x +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }

  Int avgLuma =  x   >> iCountShift;
  Int avgSrc =  y  >> iCountShift;
  Int RErrLuma = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrSrc =  y & ( ( 1 << iCountShift ) - 1 );

  Int a, b, iShift = 13;

   Int iB = 7;
   iShift -= iB;

  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgLuma*avgSrc << iCountShift ) - avgLuma*RErrSrc - avgSrc*RErrLuma;
    Int a2 = xx - ( avgLuma*avgLuma << iCountShift ) - 2*avgLuma*RErrLuma;

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }
      
    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }
      
    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_uiaShift[ a2s - 32 ] ;
      a2t = Clip( a2t );
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }
      
    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
      
    a = Clip3(-( 1 << (15-iB) ), ( 1 << (15-iB )) - 1, a);
    a = a << iB;

    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs( a ) + ( (a < 0 ? -1 : 1) - 1)/2 ) - 5;
    }

    iShift =(iShift+iB)-n;

    a = a>>n;
    b = avgSrc - ( (  a * avgLuma ) >> iShift );
  }   

  // <-- end of LLS parameters estimation

  // get prediction -->
  uiExt = uiExt0;
  pLuma = pLuma0;
  pDst = pDst0;


  for( i = 0; i < uiHeight; i++ )
  {
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
    ss << "###: ~ pLuma pred " << i << ": ";
#endif
    for( j = 0; j < uiWidth; j++ )
    {
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
      ss << pLuma[j] << " ";
#endif
      pDst[j] = Clip( ( ( a * pLuma[j] ) >> iShift ) + b );
    }
    pDst  += iDstStride;
    pLuma += iLumaStride;
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
    ss << std::endl;
#endif
  }
#if defined DEBUG_STRING && DEBUG_LM_REF_SAMPLES
  ss << "###: ~ a=" << a << ", b=" << b << ", iShift=" << iShift << std::endl;
  DEBUG_STRING_APPEND(sDebug, ss.str())
#endif
  // <-- end of get prediction
}
#endif

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param rpDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType, ChromaFormat format )
{
  Pel* pDst = rpDst;
  Int x, y, iDstStride2, iSrcStride2;
  const FilterMode mode = getIntraDCFilterMode(channelType, format);

  // boundary pixels processing
  if (mode != FILTER_DISABLED)
  {
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);
  }

  //top row (vertical filter)
  if ((mode == FILTER_BOTH_DIRECTIONS) || (mode == FILTER_VERTICAL_ONLY))
  {
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }
  }

  //left column (horizontal filter)
  if ((mode == FILTER_BOTH_DIRECTIONS) || (mode == FILTER_HORIZONTAL_ONLY))
  {
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}
//! \}
