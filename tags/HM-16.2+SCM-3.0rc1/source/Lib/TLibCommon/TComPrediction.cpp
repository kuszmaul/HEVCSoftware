/* The copyright in this software is beinOMg made available under the BSD
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
  m_iPLTErrLimit = 3;
}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
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
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

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
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
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
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, ChannelType channelType, ChromaFormat format, Bool bAbove, Bool bLeft )
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  if (bAbove)
  {
    for (iInd = 0;iInd < iWidth;iInd++)
    {
      iSum += pSrc[iInd-iSrcStride];
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
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType, ChromaFormat format,
                                          UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable
                                  , const Bool bEnableEdgeFilters
                                  )
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
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;

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

      if (edgeFilter)
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
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

Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, Bool bAbove, Bool bLeft, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
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

  // get starting pixel in block
  const Int sw = (2 * iWidth + 1);

  if ( bUseLosslessDPCM )
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );

    if ( uiDirMode == PLANAR_IDX )
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format );
    }
    else
    {
      // Create the prediction
            TComDataCU *const pcCU              = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
#if SCM_S0102_IBF_SPS_CONTROL
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx)) && !pcCU->getSlice()->getSPS()->getDisableIntraBoundaryFilter();
#else
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));
#endif

#if O0043_BEST_EFFORT_DECODING
      xPredIntraAng( g_bitDepthInStream[channelType], ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format, uiDirMode, bAbove, bLeft, enableEdgeFilters );
#else
      xPredIntraAng( g_bitDepth[channelType], ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, format, uiDirMode, bAbove, bLeft, enableEdgeFilters );
#endif
#if SCM_S0102_IBF_SPS_CONTROL
      if(( uiDirMode == DC_IDX ) && bAbove && bLeft && enableEdgeFilters)
#else
      if(( uiDirMode == DC_IDX ) && bAbove && bLeft )
#endif
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );
      }
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

Void TComPrediction::intraBlockCopy ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

  TComMv      cMv         = pcCU->getCUMvField( REF_PIC_LIST_INTRABC )->getMv( uiPartAddr );

  xPredIntraBCBlk( COMPONENT_Y, pcCU, pcCU->getPic()->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred );

  if( pcYuvPred->getChromaFormat() != CHROMA_400 )
  {
    if( pcCU->getWidth(0) == 8 && pcCU->getPartitionSize(0) != SIZE_2Nx2N && pcYuvPred->getChromaFormat() != CHROMA_444 )
    {
      // the chroma PU will be smaller than 4x4, so join with neighbouring chroma PU(s) to form a bigger block
      // chroma PUs will use the luma MV from the bottom right most of the merged chroma PUs.
      UInt uiMvSrcAddr = ( pcYuvPred->getChromaFormat() == CHROMA_422 && iPartIdx < ( pcCU->getNumPartitions() >> 1 ) ? 1 : 3 );
      cMv = pcCU->getCUMvField( REF_PIC_LIST_INTRABC )->getMv( uiMvSrcAddr );
    }

    xPredIntraBCBlk( COMPONENT_Cb, pcCU, pcCU->getPic()->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred );
    xPredIntraBCBlk( COMPONENT_Cr, pcCU, pcCU->getPic()->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred );
  }

  return;
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
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
#if SCM_S0085_ADAPTIVE_MV_RESOLUTION
  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
    cMv <<= 2;
  }
#endif
  pcCU->clipMv(cMv);

  for (UInt ch=COMPONENT_Y; ch<pcYuvPred->getNumberValidComponents(); ch++)
    xPredInterBlk  (ComponentID(ch),  pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi );
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred )
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
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) ||
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
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


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

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

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, !bi, chFmt);
  }
}

Void TComPrediction::xPredIntraBCBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);

  Int mvx = mv->getHor() >>  refPic->getComponentScaleX(compID);
  Int mvy = mv->getVer() >>  refPic->getComponentScaleY(compID);

  Int     refOffset  = mvx + mvy * refStride;

  Pel*    ref = refPic->getAddr( compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;
  Pel*    dst = dstPic->getAddr( compID, partAddr );

  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  for (Int row = 0; row < cxHeight; row++)
  {
    for (Int col = 0; col < cxWidth; col++)
    {
      dst[col] = ref[col];
    }

    ref += refStride;
    dst += dstStride;
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcAMVPInfo->iN <= 1 )
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

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
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

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)
  {
    Int horPred = leftColumn[y] + width;
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
    }
  }
}

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
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

#if !SCM_S0156_PLT_ENC_RDO
Void TComPrediction::deriveRun(TComDataCU* pcCU, Pel* pOrg[3],  Pel *pPalette[3],  Pel* pValue, UChar* pSPoint,
                               Pel** paPixelValue, Pel ** paRecoValue, TCoeff* pRun,
                               UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize)
{
  UInt uiTotal = uiHeight * uiWidth, uiIdx = 0;
  UInt uiStartPos = 0,  uiRun = 0, uiCopyRun = 0;
  Int iTemp = 0;
#if !SCM_S0258_PLT_ESCAPE_SIG
  UInt uiStride = uiWidth;
#endif
  UInt uiTraIdx;  //unified position variable (raster scan)

#if SCM_S0258_PLT_ESCAPE_SIG
  UChar *pEscapeFlag  = pcCU->getEscapeFlag(COMPONENT_Y);
  Pel *pcIndexBlock = m_cIndexBlock;
#endif

  //Test Run
  while (uiIdx < uiTotal)
  {
    uiStartPos = uiIdx;
    uiRun = 0;

#if SCM_S0258_PLT_ESCAPE_SIG
    Bool RunValid = calLeftRun(pValue, pSPoint, uiStartPos, uiTotal, uiRun, pEscapeFlag);
#else
    Bool RunValid = calLeftRun(pValue, pSPoint, uiStartPos, uiTotal, uiRun);
#endif

    uiCopyRun = 0;

#if SCM_S0258_PLT_ESCAPE_SIG
    Bool CopyValid = calAboveRun(pValue, pSPoint, uiWidth, uiStartPos, uiTotal, uiCopyRun, pEscapeFlag );
#else
    Bool CopyValid = calAboveRun(pValue, pSPoint, uiWidth, uiStartPos, uiTotal, uiCopyRun);
#endif

    uiTraIdx = m_puiScanOrder[uiIdx];    //unified position variable (raster scan)

#if SCM_S0258_PLT_ESCAPE_SIG
    assert( RunValid || CopyValid );
#else
    if (CopyValid == 0 && RunValid == 0)
    {
      pSPoint[uiTraIdx] = PLT_ESCAPE;
      calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
      uiIdx++;
    }
    else
#endif
    {
      if ( CopyValid && (uiCopyRun + 2 > uiRun))
      {
        pSPoint[uiTraIdx] = PLT_RUN_ABOVE;
        pRun[uiTraIdx]  = uiCopyRun-1;
#if SCM_S0258_PLT_ESCAPE_SIG
        pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

        if( pEscapeFlag[uiTraIdx] )
        {
          calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
        }
#endif
        uiIdx++;

        iTemp = uiCopyRun - 1;
        while (iTemp > 0)
        {
          uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)

#if SCM_S0258_PLT_ESCAPE_SIG
          pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

          if( pEscapeFlag[uiTraIdx] )
          {
            calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
          }
#else
          pValue[uiTraIdx] = pValue[uiTraIdx - uiStride];
#endif

          pSPoint[uiTraIdx] = PLT_RUN_ABOVE;
          uiIdx++;

          iTemp--;
        }
      }
      else
      {
        pSPoint[uiTraIdx] = PLT_RUN_LEFT;
        pRun[uiTraIdx] = uiRun;

#if SCM_S0258_PLT_ESCAPE_SIG
        pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

        if( pEscapeFlag[uiTraIdx] )
        {
          calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
        }
#endif

        uiIdx++;
        iTemp = uiRun;
        while (iTemp > 0)
        {
          uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)
#if SCM_S0258_PLT_ESCAPE_SIG
          pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

          if( pEscapeFlag[uiTraIdx] )
          {
            calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
          }
#else
          pValue[uiTraIdx] = pValue[m_puiScanOrder[uiIdx - 1]];
#endif
          pSPoint[uiTraIdx] = PLT_RUN_LEFT;
          uiIdx++;
          iTemp--;
        }
      }
    }
  }
  assert (uiIdx == uiTotal);
}
#endif

Void TComPrediction::preCalcPLTIndex(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : 3 * getPLTErrLimit();
  UInt uiPos;

  UInt uiBestIdx = 0;
  UChar useEscapeFlag=0;

  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiWidth + uiX;
      UInt uiPLTIdx = 0;
      UInt uiMinError = MAX_UINT;
      while (uiPLTIdx < uiPLTSize)
      {
#if SCM_S0180_BUG_FIX_BIT_DEPTH
        UInt uiAbsError = MAX_UINT;
        if ( bLossless )
        {
          uiAbsError = abs( Palette[0][uiPLTIdx] - pSrc[0][uiPos] ) + abs( Palette[1][uiPLTIdx] - pSrc[1][uiPos] ) + abs( Palette[2][uiPLTIdx] - pSrc[2][uiPos] );
        }
        else
        {
          uiAbsError = ( abs(Palette[0][uiPLTIdx] - pSrc[0][uiPos]) >> DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_LUMA]  -8) )
                     + ( abs(Palette[1][uiPLTIdx] - pSrc[1][uiPos]) >> DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8) )
                     + ( abs(Palette[2][uiPLTIdx] - pSrc[2][uiPos]) >> DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8) );
        }
#else
        UInt uiAbsError = abs(Palette[0][uiPLTIdx] - pSrc[0][uiPos]) + abs(Palette[1][uiPLTIdx] - pSrc[1][uiPos]) + abs(Palette[2][uiPLTIdx] - pSrc[2][uiPos]);
#endif
        if (uiAbsError < uiMinError)
        {
          uiBestIdx = uiPLTIdx;
          uiMinError = uiAbsError;
          if (uiMinError == 0)
          {
            break;
          }
        }
        uiPLTIdx++;
      }
      m_cIndexBlock[uiPos] = uiBestIdx;
      if (uiMinError > iErrorLimit)
      {
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
        m_cIndexBlock[uiPos] -= pcCU->getSlice()->getSPS()->getPLTMaxSize();
#else
        m_cIndexBlock[uiPos] -= MAX_PLT_SIZE;
#endif
        useEscapeFlag=1;
      }
    }
  }

  pcCU->setPLTEscapeSubParts(0, useEscapeFlag,0, pcCU->getDepth(0));
}

Void  TComPrediction::reorderPLT(TComDataCU* pcCU, Pel *pPalette[3], UInt uiNumComp)
{

  UInt uiPLTSizePrev, uiDictMaxSize;
  UInt uiPLTUsedSizePrev;
  Pel * pPalettePrev[3];
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
  UInt uiMaxPLTSize = pcCU->getSlice()->getSPS()->getPLTMaxSize();
  UInt uiMaxPLTPredSize = pcCU->getSlice()->getSPS()->getPLTMaxPredSize();
  Pel* pPaletteTemp[3];
  for (UInt ch = 0; ch < 3; ch++)
  {
    pPaletteTemp[ch] = (Pel*)xMalloc(Pel, uiMaxPLTSize);
  }
#else
  Pel pPaletteTemp[3][MAX_PLT_SIZE];
#endif
  ComponentID compBegin = ComponentID(uiNumComp == 2 ? 1 : 0);

  for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
  {
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCtu(), comp, uiPLTSizePrev, uiPLTUsedSizePrev);
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
    for (UInt i = 0; i < uiMaxPLTSize; i++)
#else
    for (UInt i = 0; i < MAX_PLT_SIZE; i++)
#endif
    {
      pPaletteTemp[comp][i] = pPalette[comp][i];
    }
  }

  pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCtu(), compBegin, uiPLTSizePrev, uiPLTUsedSizePrev);
  uiDictMaxSize = pcCU->getPLTSize(compBegin, 0);

  UInt uiIdxPrev = 0, uiIdxCurr = 0;
  Bool bReused = false;
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
  Bool *bPredicted, *bReusedPrev;
  bPredicted  = (Bool*)xMalloc(Bool, uiMaxPLTSize + 1);
  bReusedPrev = (Bool*)xMalloc(Bool, uiMaxPLTPredSize + 1);
  memset(bPredicted, 0, sizeof(Bool)*(uiMaxPLTSize + 1));
  memset(bReusedPrev, 0, sizeof(Bool)*(uiMaxPLTPredSize + 1));
#else
  Bool bPredicted[MAX_PLT_SIZE + 1], bReusedPrev[MAX_PLT_PRED_SIZE + 1];
  memset(bReusedPrev, 0, sizeof(bReusedPrev));
  memset(bPredicted, 0, sizeof(bPredicted));
#endif
  UInt uiNumPLTRceived = uiDictMaxSize, uiNumPLTPredicted = 0;
  for (uiIdxPrev = 0; uiIdxPrev < uiPLTSizePrev; uiIdxPrev++)
  {
    bReused = false;
    Int iCounter = 0;
    for (uiIdxCurr = 0; uiIdxCurr < uiDictMaxSize; uiIdxCurr++)
    {
      iCounter = 0;

      for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
      {
        if (pPalettePrev[comp][uiIdxPrev] == pPalette[comp][uiIdxCurr])
        {
          iCounter++;
        }
      }
      if (iCounter == uiNumComp)
      {
        bReused = true;
        break;
      }
    }
    bReusedPrev[uiIdxPrev] = bReused;
    bPredicted[uiIdxCurr] = bReused;
    if (bPredicted[uiIdxCurr])
    {
      uiNumPLTRceived--;
      uiNumPLTPredicted++;
    }
  }
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE   
  for (uiIdxPrev = 0; uiIdxPrev < uiMaxPLTPredSize; uiIdxPrev++)
#else
  for (uiIdxPrev = 0; uiIdxPrev < MAX_PLT_PRED_SIZE; uiIdxPrev++)
#endif
  {
    for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
    {
      pcCU->setPrevPLTReusedFlagSubParts(comp, bReusedPrev[uiIdxPrev], uiIdxPrev, 0, pcCU->getDepth(0));
    }
  }
  uiIdxCurr = 0;
  for (UInt uiPrevIdx = 0; uiPrevIdx < uiPLTSizePrev; uiPrevIdx++)
  {
    if (bReusedPrev[uiPrevIdx])
    {
      for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
      {
        pPalette[comp][uiIdxCurr] = pPalettePrev[comp][uiPrevIdx];
      }
      uiIdxCurr++;
    }
  }

  for (UInt uiIdx = 0; uiIdx < uiDictMaxSize; uiIdx++)
  {
    if (bPredicted[uiIdx] == 0)
    {
      for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
      {
        pPalette[comp][uiIdxCurr] = pPaletteTemp[comp][uiIdx];
      }
      uiIdxCurr++;
    }
  }
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
  for (UInt ch = 0; ch < 3; ch++)
  {
    if (pPaletteTemp[ch])
    {
      xFree(pPaletteTemp[ch]);
      pPaletteTemp[ch] = NULL;
    }
  }
  if (bPredicted)
  {
    xFree(bPredicted);
    bPredicted = NULL;
  }
  if (bReusedPrev)
  {
    xFree(bReusedPrev);
    bReusedPrev = NULL;
  }
#endif
}

Void  TComPrediction::derivePLTLossy( TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost )
{
  Int iErrorLimit = getPLTErrLimit();
  UInt uiTotalSize = uiHeight*uiWidth;
  SortingElement *psList = new SortingElement [uiTotalSize];
  SortingElement sElement;
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
  UInt uiDictMaxSize = pcCU->getSlice()->getSPS()->getPLTMaxSize();
  SortingElement *pListSort = new SortingElement [uiDictMaxSize + 1];
#else
  UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElement *pListSort = new SortingElement [MAX_PLT_SIZE + 1];
#endif
  UInt uiIdx = 0;
  UInt uiPos;
  Int last = -1;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      uiPos = uiY * uiWidth+ uiX;
      sElement.setAll (pSrc[0][uiPos], pSrc[1][uiPos], pSrc[2][uiPos]);

      Int besti = last, bestSAD = (last == -1) ? MAX_UINT : psList[last].getSAD(sElement);
      if( bestSAD )
      {
        for( Int i = uiIdx-1; i>=0; i-- )
        {
          UInt sad = psList[i].getSAD(sElement);
          if( sad < bestSAD )
          {
            bestSAD = sad;
            besti = i;
            if( !sad ) break;
          }
        }
      }

      if (besti>=0 && psList[besti].almostEqualData(sElement, iErrorLimit))
      {
        psList[besti].addElement(sElement);
        last = besti;
      }
      else
      {
        psList[uiIdx].copyDataFrom(sElement);
        psList[uiIdx].uiCnt = 1;
        last = uiIdx;
        uiIdx ++;
      }
    }
  }

  for (Int i = 0; i < uiDictMaxSize; i++)
  {
    pListSort[i].uiCnt  = 0;
    pListSort[i].setAll(0, 0, 0) ;
  }

  //bubble sorting
  uiDictMaxSize = 1;
  for (Int i = 0; i < uiIdx; i++)
  {
    if( psList[i].uiCnt > pListSort[uiDictMaxSize-1].uiCnt )
    {
      Int j;
      for (j = uiDictMaxSize; j > 0; j--)
      {
        if (psList[i].uiCnt > pListSort[j-1].uiCnt)
        {
          pListSort[j].copyAllFrom (pListSort[j-1]);
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE   
          uiDictMaxSize = std::min(uiDictMaxSize + 1, pcCU->getSlice()->getSPS()->getPLTMaxSize());
#else
          uiDictMaxSize = std::min(uiDictMaxSize+1,(UInt)MAX_PLT_SIZE);
#endif
        }
        else
        {
          break;
        }
      }
      pListSort[j].copyAllFrom (psList[i]) ;
    }
  }

  uiPLTSize = 0;
  Pel *pPred[3]  = { pcCU->getLastPLTInLcuFinal(0), pcCU->getLastPLTInLcuFinal(1), pcCU->getLastPLTInLcuFinal(2) };
  Double bitCost = pcCost->getLambda() * 24;
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
  for (Int i = 0; i < pcCU->getSlice()->getSPS()->getPLTMaxSize(); i++)
#else
  for (Int i = 0; i < MAX_PLT_SIZE; i++)
#endif
  {
    if( pListSort[i].uiCnt )
    {
      Int iHalf = pListSort[i].uiCnt>>1;
      Palette[0][uiPLTSize] = (pListSort[i].uiSumData[0]+iHalf)/pListSort[i].uiCnt;
      Palette[1][uiPLTSize] = (pListSort[i].uiSumData[1]+iHalf)/pListSort[i].uiCnt;
      Palette[2][uiPLTSize] = (pListSort[i].uiSumData[2]+iHalf)/pListSort[i].uiCnt;

      Int best = -1;
      if( iErrorLimit )
      {
        Double pal[3] = { pListSort[i].uiSumData[0]/(Double)pListSort[i].uiCnt,
                          pListSort[i].uiSumData[1]/(Double)pListSort[i].uiCnt,
                          pListSort[i].uiSumData[2]/(Double)pListSort[i].uiCnt };

        Double err      = pal[0] - Palette[0][uiPLTSize];
#if SCM_S0180_BUG_FIX_BIT_DEPTH
        Double bestCost = (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_LUMA]-8)) );
        err = pal[1] - Palette[1][uiPLTSize]; bestCost += (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8)) );
        err = pal[2] - Palette[2][uiPLTSize]; bestCost += (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8)) );
#else
        Double bestCost = err*err;
        err = pal[1] - Palette[1][uiPLTSize]; bestCost += err*err;
        err = pal[2] - Palette[2][uiPLTSize]; bestCost += err*err;
#endif
        bestCost = bestCost * pListSort[i].uiCnt + bitCost;

        for(int t=0; t<pcCU->getLastPLTInLcuSizeFinal(0); t++)
        {
          err = pal[0] - pPred[0][t];
#if SCM_S0180_BUG_FIX_BIT_DEPTH
          Double cost = (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_LUMA]-8)) );
          err = pal[1] - pPred[1][t]; cost += (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8)) );
          err = pal[2] - pPred[2][t]; cost += (err*err) / ( 1<<(2*DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[CHANNEL_TYPE_CHROMA]-8)) );
#else
          Double cost = err*err;
          err = pal[1] - pPred[1][t]; cost += err*err;
          err = pal[2] - pPred[2][t]; cost += err*err;
#endif
          cost *= pListSort[i].uiCnt;
          if(cost < bestCost)
          {
            best = t;
            bestCost = cost;
          }
        }
        if( best != -1 )
        {
          Palette[0][uiPLTSize] = pPred[0][best];
          Palette[1][uiPLTSize] = pPred[1][best];
          Palette[2][uiPLTSize] = pPred[2][best];
        }
      }

      Bool bDuplicate = false;
      if( pListSort[i].uiCnt == 1 && best == -1 )
      {
        bDuplicate = true;
      }
      else
      {
        for( Int t=0; t<uiPLTSize; t++)
        {
          if( Palette[0][uiPLTSize] == Palette[0][t] && Palette[1][uiPLTSize] == Palette[1][t] && Palette[2][uiPLTSize] == Palette[2][t] )
          {
            bDuplicate = true;
            break;
          }
        }
      }
      if( !bDuplicate ) uiPLTSize++;
    }
    else
    {
      break;
    }
  }

  delete [] psList;
  delete [] pListSort;
}

Void TComPrediction::derivePLTLossless(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  std::vector<SortingElement> psList;
  SortingElement sElement;
  UInt uiIdx = 0;
  UInt uiPos;
  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiWidth + uiX;
      sElement.setAll(pSrc[0][uiPos], pSrc[1][uiPos], pSrc[2][uiPos]);
      Int i = 0;
      for (i = uiIdx - 1; i >= 0; i--)
      {
        if( psList[i].uiData[0] == sElement.uiData[0] && psList[i].uiData[1] == sElement.uiData[1] && psList[i].uiData[2] == sElement.uiData[2] )
        {
          psList[i].uiCnt++;
          break;
        }
      }
      if (i == -1)
      {
        psList.push_back(sElement);
        psList[uiIdx].uiCnt++;
        uiIdx++;
      }
    }
  }
  //insertion sort, high frequency -> low frequency
  std::stable_sort(psList.begin(), psList.end());
  UInt uiPLTSizePrev;
  UInt uiPLTUsedSizePrev;
  Pel *pPalettePrev[3];
  for (UInt comp = 0; comp < 3; comp++)
  {
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCtu(), comp, uiPLTSizePrev, uiPLTUsedSizePrev);
  }

  uiPLTSize = 0;
  for (Int i = 0; i < uiIdx; i++)
  {
    for (Int j = i - 1; j >= 0; j--)
    {
#if SCM_S0180_BUG_FIX_BIT_DEPTH
      if ( psList[j].uiCnt && psList[i].uiData[0] == psList[j].uiData[0]
                           && psList[i].uiData[1] == psList[j].uiData[1]
                           && psList[i].uiData[2] == psList[j].uiData[2]
         )
#else
      if (psList[j].uiCnt && psList[i].almostEqualData(psList[j], 0))
#endif
      {
        psList[i].uiCnt = 0;
        break;
      }
    }

    Bool includeIntoPLT = true;
    if( psList[i].uiCnt == 1 )
    {
      includeIntoPLT = false;
      for( UInt uiIdxPrev = 0; uiIdxPrev < uiPLTSizePrev; uiIdxPrev++ )
      {
        UInt iCounter = 0;

        for( UInt comp = 0; comp < 3; comp++ )
        {
          if( psList[i].uiData[comp] == pPalettePrev[comp][uiIdxPrev] )
          {
            iCounter++;
          }
          else
          {
            break;
          }
        }
        if( iCounter == 3 )
        {
          includeIntoPLT = true;
          break;
        }
      }
    }

    if( includeIntoPLT && psList[i].uiCnt)
    {
      Palette[0][uiPLTSize] = psList[i].uiData[0];
      Palette[1][uiPLTSize] = psList[i].uiData[1];
      Palette[2][uiPLTSize] = psList[i].uiData[2];
      uiPLTSize++;
#if SCM_CE5_MAX_PLT_AND_PRED_SIZE 
      if (uiPLTSize == pcCU->getSlice()->getSPS()->getPLTMaxSize())
#else
      if (uiPLTSize == MAX_PLT_SIZE)
#endif
      {
        break;
      }
    }
  }
}

Void TComPrediction::calcPixelPred(TComDataCU* pcCU, Pel* pOrg [3], Pel *pPalette[3], Pel* pValue, Pel*paPixelValue[3], Pel * paRecoValue[3], UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiIdx )
{
  Bool bLossless = pcCU->getCUTransquantBypass (0);
  Int iQP[3];
  Int iQPrem[3];
  Int iQPper[3];
  Int quantiserScale[3];
  Int quantiserRightShift[3];
  Int rightShiftOffset[3];
  Int InvquantiserRightShift[3];
  Int iAdd[3];
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    QpParam cQP(*pcCU, ComponentID(ch));
    iQP[ch] = cQP.Qp;
    iQPrem[ch] = iQP[ch] % 6;
    iQPper[ch] = iQP[ch] / 6;
    quantiserScale[ch] = g_quantScales[iQPrem[ch]];
    quantiserRightShift[ch] = QUANT_SHIFT + iQPper[ch];
    rightShiftOffset[ch] = 1 << (quantiserRightShift[ch] - 1);
    InvquantiserRightShift[ch] = (IQUANT_SHIFT - iQPper[ch]);
    iAdd[ch] = InvquantiserRightShift[ch] == 0 ? 0 : 1 << (InvquantiserRightShift[ch] - 1);
  }

  UInt uiY, uiX;
  uiY = uiIdx / uiWidth;
  uiX = uiIdx % uiWidth;
  UInt uiScanIdx = uiY * uiStrideOrg + uiX;
  UInt uiYIdxRaster = pcCU->getPLTScanRotationModeFlag(0)? (uiX * uiStrideOrg + uiY) : (uiY * uiStrideOrg + uiX);
  UInt uiMaxBit[3];
  pcCU->xCalcMaxBits(pcCU, uiMaxBit);
  if (bLossless)
  {
    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch ++)
    {
      paPixelValue[ch][uiScanIdx] =  pOrg[ch][uiYIdxRaster];
      paRecoValue[ch][uiYIdxRaster] = pOrg[ch][uiYIdxRaster];
    }
  }
  else
  {
    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch ++)
    {
      paPixelValue[ch][uiScanIdx] = Pel(ClipBD<Int>((pOrg[ch][uiYIdxRaster] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch], uiMaxBit[ch]));
      paRecoValue[ch][uiYIdxRaster] = (paPixelValue[ch] [uiScanIdx]*g_invQuantScales[iQPrem[ch]] + iAdd[ch])>>InvquantiserRightShift[ch];
      paRecoValue[ch][uiYIdxRaster] = Pel(ClipBD<Int>(paRecoValue[ch][uiYIdxRaster], g_bitDepth[ch? 1:0]));
    }
  }
}

#if SCM_S0258_PLT_ESCAPE_SIG
Bool TComPrediction::calLeftRun(Pel* pValue, UChar* pSPoint, UInt uiStartPos, UInt uiTotal, UInt &uiRun, UChar* pEscapeFlag)
#else
Bool TComPrediction::calLeftRun(Pel* pValue, UChar* pSPoint, UInt uiStartPos, UInt uiTotal, UInt &uiRun)
#endif
{
  UInt uiIdx = uiStartPos;
  Pel *pcIndexBlock = m_cIndexBlock;
  while (uiIdx < uiTotal)
  {
    UInt uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)
    pValue[uiTraIdx] = pcIndexBlock[uiTraIdx] < 0 ? pcIndexBlock[uiTraIdx] + MAX_PLT_SIZE : pcIndexBlock[uiTraIdx];
    Bool bMismatch = (pcIndexBlock[uiTraIdx] < 0);

#if SCM_S0258_PLT_ESCAPE_SIG || SCM_S0156_PLT_ENC_RDO
    pSPoint[uiTraIdx] = PLT_RUN_LEFT;
#endif
#if SCM_S0258_PLT_ESCAPE_SIG
    pEscapeFlag[uiTraIdx] = (pcIndexBlock[uiTraIdx] < 0)? 1: 0;    
#else
    if (bMismatch && uiIdx == uiStartPos)
    {
      return false;
    }
#endif
    UInt leftTraIdx = uiIdx ? m_puiScanOrder[uiIdx - 1] : 0;
#if SCM_S0258_PLT_ESCAPE_SIG
    if( uiIdx > uiStartPos &&
      ( ( pcIndexBlock[leftTraIdx] >= 0 && pValue[uiTraIdx] == pValue[leftTraIdx] && !bMismatch ) || ( bMismatch && pcIndexBlock[leftTraIdx] < 0 ) )
      )
#else
    if (uiIdx > uiStartPos && (PLT_ESCAPE != pSPoint[leftTraIdx]) && pValue[uiTraIdx] == pValue[leftTraIdx] && !bMismatch)
#endif
    {
      uiRun++;
    }
    else if (uiIdx > uiStartPos)
    {
      break;
    }
    uiIdx++;
  }
  return true;
}

#if SCM_S0258_PLT_ESCAPE_SIG
Bool  TComPrediction::calAboveRun(Pel* pValue, UChar* pSPoint, UInt uiWidth, UInt uiStartPos, UInt uiTotal, UInt &uiRun, UChar* pEscapeFlag)
#else
Bool  TComPrediction::calAboveRun(Pel* pValue, UChar* pSPoint, UInt uiWidth, UInt uiStartPos, UInt uiTotal, UInt &uiRun)
#endif
{
  UInt uiIdx = uiStartPos;
  UInt uiY = 0;
  Bool valid = false;
  Pel *pcIndexBlock = m_cIndexBlock;
  UInt uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)

#if SCM_S0258_PLT_ESCAPE_SIG
  uiY = uiTraIdx / uiWidth;
  if( uiY == 0 )
  {
    return false;
  }
#endif

  while (uiIdx < uiTotal)
  {
#if !SCM_S0258_PLT_ESCAPE_SIG
    uiY = uiTraIdx / uiWidth;
    if (uiY == 0)
    {
      return false;
    }
#endif
    UInt uiStride = uiWidth;
    uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)

    pValue[uiTraIdx] = pcIndexBlock[uiTraIdx] < 0 ? pcIndexBlock[uiTraIdx] + MAX_PLT_SIZE : pcIndexBlock[uiTraIdx];
    Bool bMismatch = (pcIndexBlock[uiTraIdx] < 0);

#if SCM_S0258_PLT_ESCAPE_SIG || SCM_S0156_PLT_ENC_RDO
    pSPoint[uiTraIdx] = PLT_RUN_ABOVE;
#endif
#if SCM_S0258_PLT_ESCAPE_SIG
    pEscapeFlag[uiTraIdx] = (pcIndexBlock[uiTraIdx] < 0)? 1: 0;

    if ( ( pcIndexBlock[uiTraIdx - uiStride] >= 0 && pValue[uiTraIdx] == pValue[uiTraIdx - uiStride] && !bMismatch ) ||
         ( bMismatch && pcIndexBlock[uiTraIdx - uiStride] < 0 )
       )
#else
    if (uiRun == 0 && bMismatch)
    {
      return false;
    }
    if ((PLT_ESCAPE != pSPoint[uiTraIdx - uiStride]) && pValue[uiTraIdx] == pValue[uiTraIdx - uiStride] && !bMismatch)
#endif
    {
      uiRun++;
      valid = true;
    }
    else
    {
      break;
    }
    uiIdx++;
  }
  return valid;
}

Void  TComPrediction::rotationScan( Pel* pLevel, UInt uiWidth, UInt uiHeight, Bool isInverse )
{
  Pel tmpLevel;
  UInt uiPos = 0;
  UInt* puiScanOrder = g_scanOrder[SCAN_UNGROUPED][SCAN_VER][g_aucConvertToBit[uiWidth] + 2][g_aucConvertToBit[uiHeight] + 2];

  for (UInt j = 1; j < uiHeight; j++)
  {
    uiPos += j;
    for (UInt i = j; i < uiWidth; i++)
    {
      tmpLevel = pLevel[uiPos];
      pLevel[uiPos] = pLevel[puiScanOrder[uiPos]];
      pLevel[puiScanOrder[uiPos]] = tmpLevel;
      uiPos++;
    }
  }
}

//! \}
