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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
Void fillReferenceSamples( TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, const Int unitWidth, const Int iNumUnitsInCu, const Int iTotalUnits,
                           const UInt uiCuWidth, const UInt uiCuHeight, const UInt uiWidth, const UInt uiHeight, const Int iPicStride,
                           const ChannelType chType, const ChromaFormat chFmt, Bool bLMmode );

/// constrained intra prediction
Bool  isAboveLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT );
Int   isAboveAvailable      ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isLeftAvailable       ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
Int   isAboveRightAvailable ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isBelowLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );


// ====================================================================================================================
// Public member functions (TComPatternParam)
// ====================================================================================================================

/** \param  piTexture     pixel data
 \param  iRoiWidth     pattern widthhttp://gbdevapp08/svn/Curie_HVC/HEVC_ECF/source/Lib/TLibCommon/TComPattern.cpp
 \param  iRoiHeight    pattern height
 \param  iStride       buffer stride
 \param  iOffsetLeft   neighbour offset (left)
 \param  iOffsetRight  neighbour offset (right)
 \param  iOffsetAbove  neighbour offset (above)
 \param  iOffsetBottom neighbour offset (bottom)
 */
Void TComPatternParam::setPatternParamPel ( Pel* piTexture,
                                           Int iRoiWidth,
                                           Int iRoiHeight,
                                           Int iStride
                                           )
{
  m_piROIOrigin    = piTexture;
  m_iROIWidth       = iRoiWidth;
  m_iROIHeight      = iRoiHeight;
  m_iPatternStride  = iStride;
}

// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

Void TComPattern::initPattern (Pel* piY,
                               Int iRoiWidth,
                               Int iRoiHeight,
                               Int iStride)
{
  m_cPatternY. setPatternParamPel( piY,  iRoiWidth, iRoiHeight, iStride);
}


// NOTE: ECF - this has been kept in this C++ file to allow easier tracking/comparison against HM.
Void TComPrediction::initAdiPatternChType( TComTU &rTu, Bool& bAbove, Bool& bLeft, const ComponentID compID, const Bool bFilterRefSamples DEBUG_STRING_FN_DECLARE(sDebug), Bool bLMmode)
{
  const ComponentID regionCompID=bLMmode ? COMPONENT_Cb : compID;

  TComDataCU *pcCU=rTu.getCU();
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Int   iUnitSize = 0;
  Int   iNumUnitsInCu = 0;
  Int   iTotalUnits = 0;
  Bool  bNeighborFlags[4 * MAX_NUM_SPU_W + 1];
  Int   iNumIntraNeighbor = 0;

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;

  UInt uiPartDepth=rTu.GetTransformDepthRelAdj(regionCompID);

  const UInt csx=pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const UInt csy=pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);

  const UInt  uiTuWidth_  = pcCU->getWidth (0) >> uiPartDepth;
  const UInt  uiTuHeight_ = pcCU->getHeight(0) >> uiPartDepth;

  pcCU->deriveLeftRightTopIdxAdi( uiPartIdxLT, uiPartIdxRT, uiZorderIdxInPart, uiPartDepth );
  pcCU->deriveLeftBottomIdxAdi  ( uiPartIdxLB,              uiZorderIdxInPart, uiPartDepth );


  iUnitSize      = (g_uiMaxCUWidth >> g_uiMaxCUDepth) >> csx;
  iNumUnitsInCu  = (uiTuWidth_ / iUnitSize) >> csx; // Not actually csx dependent!
  iTotalUnits    = (iNumUnitsInCu << 2) + 1;

  bNeighborFlags[iNumUnitsInCu*2] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor  += (Int)(bNeighborFlags[iNumUnitsInCu*2]);
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, bNeighborFlags+(iNumUnitsInCu*2)+1 );
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, bNeighborFlags+(iNumUnitsInCu*3)+1 );
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, bNeighborFlags+(iNumUnitsInCu*2)-1 );
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, bNeighborFlags+ iNumUnitsInCu   -1 );
  
  bAbove = true;
  bLeft  = true;

  const Int  uiTuWidth   = uiTuWidth_  >> csx;
  const Int  uiTuHeight  = uiTuHeight_ >> csy;
  const Int  uiTuWidth2  = uiTuWidth<<1;
  const Int  uiTuHeight2 = uiTuHeight<<1;
  const ChromaFormat chFmt = rTu.GetChromaFormat();

  UInt uiROIWidth ;
  UInt uiROIHeight;
  if (nonScaledIntraChroma422(toChannelType(compID), chFmt))
  {
    uiROIWidth  = uiTuWidth+uiTuHeight+1;
    uiROIHeight = uiTuHeight+uiTuWidth+1;
  }
  else
  {
    uiROIWidth  = uiTuWidth2+1;
    uiROIHeight = uiTuHeight2+1;
  }

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);

#ifdef DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piAdiTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiZorderIdxInPart);
    fillReferenceSamples ( pcCU, piRoiOrigin, piAdiTemp, bNeighborFlags, iNumIntraNeighbor, iUnitSize, iNumUnitsInCu,
                         iTotalUnits, uiTuWidth, uiTuHeight, uiROIWidth, uiROIHeight, iPicStride, toChannelType(compID), chFmt, getLMChromaSamplesFrom2ndLeftCol(bLMmode, chFmt));


#ifdef DEBUG_STRING
    if (DEBUG_INTRA_REF_SAMPLES)
    {
      if (bLMmode)
        ss << "###: generating LM Ref Samples for channel " << compID << "\n";
      else
        ss << "###: generating Ref Samples for channel " << compID << " and " << rTu.getRect(compID).width << " x " << rTu.getRect(compID).height << "\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
            ss << piAdiTemp[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
        }
        ss << "\n";
      }
    }
#endif

    if (bFilterRefSamples)
    {

      // generate filtered intra prediction samples

      Pel *piFilteredBuf1    = m_piYuvExt[compID][PRED_BUF_FILTERED];
      Int stride=uiROIWidth;
            Pel *piDestPtr=piFilteredBuf1+uiROIWidth*uiTuHeight2; // bottom left
      const Pel *piSrcPtr =piAdiTemp+uiROIWidth*uiTuHeight2;      // bottom left
      *piDestPtr=*piSrcPtr; // bottom left is not filtered
      piDestPtr-=stride; piSrcPtr-=stride;
      const ChannelType chType=toChannelType(compID);
      if (applyFilteredIntraReferenceSamples(chType, chFmt, 1))
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
        }
      }
      else
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = piSrcPtr[0];
        }
      }
      // now looking at the 0 case
      if (applyFilteredIntraReferenceSamples(chType, chFmt, 0))
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      else
      {
        *piDestPtr = piSrcPtr[0];
      }
      piDestPtr+=1; piSrcPtr+=1;
      // now the top row
      if (applyFilteredIntraReferenceSamples(chType, chFmt, 2))
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
        }
      }
      else
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = piSrcPtr[0];
        }
      }

      *piDestPtr=*piSrcPtr; // far right is not filtered

#ifdef DEBUG_STRING
    if (DEBUG_INTRA_REF_SAMPLES)
    {
      if (bLMmode)
        ss << "###: filtered result for LM Ref Samples for channel " << compID <<"\n";
      else
        ss << "###: filtered result for channel " << compID <<"\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
            ss << piFilteredBuf1[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
        }
        ss << "\n";
      }
    }
#endif


    }
  }
  DEBUG_STRING_APPEND(sDebug, ss.str())
}

Void fillReferenceSamples( TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, const Int unitWidth, const Int iNumUnitsInCu, const Int iTotalUnits,
                           const UInt uiCuWidth, const UInt uiCuHeight, const UInt uiWidth, const UInt uiHeight, const Int iPicStride,
                           const ChannelType chType, const ChromaFormat chFmt, Bool bLMmode )
{
  const Pel* piRoiTemp;
  Int  i, j;
  Int  iDCValue = ( 1<<( g_uiBitDepth + g_uiBitIncrement - 1) );

  if (iNumIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (i=0; i<uiWidth; i++)
    {
      piAdiTemp[i] = iDCValue;
    }
    for (i=1; i<uiHeight; i++)
    {
      piAdiTemp[i*uiWidth] = iDCValue;
    }
  }
  else if (iNumIntraNeighbor == iTotalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    piRoiTemp = piRoiOrigin - iPicStride - 1;

    if (nonScaledIntraChroma422(chType, chFmt))
    {
      const Int limitX=uiCuWidth*2;
      for (i=0; i<=limitX; i++)
      {
        piAdiTemp[i] = piRoiTemp[i];
      }
      for (; i<uiWidth; i++)
      {
        piAdiTemp[i] = piRoiTemp[limitX];
      }
    }
    else
    {
      for (i=0; i<uiWidth; i++)
      {
        piAdiTemp[i] = piRoiTemp[i];
      }
    }

    // Fill left and below left border with rec. samples
    piRoiTemp = piRoiOrigin - 1;

    if (bLMmode)
    {
      piRoiTemp --; // move to the second left column
    }

    for (i=1; i<uiHeight; i++)
    {
      piAdiTemp[i*uiWidth] = *(piRoiTemp);
      piRoiTemp += iPicStride;
    }
  }
  else // reference samples are partially available
  {
    const Int unitHeight = unitWidth << ((isChroma(chType) && pcCU->getPic()->getChromaFormat()==CHROMA_422) ? 1 : 0);
    Int  iNumUnits2 = iNumUnitsInCu<<1;
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    Int  iTotalSamples = 2*iNumUnitsInCu*unitHeight + (2*iNumUnitsInCu+1)*unitWidth;
    Pel  piAdiLine[5 * MAX_CU_SIZE];
    Pel  *piAdiLineTemp; 
    const Bool *pbNeighborFlags;


    // Initialize
    for (i=0; i<iTotalSamples; i++)
    {
      piAdiLine[i] = iDCValue;
    }
    
    // Fill top-left sample
    piRoiTemp = piRoiOrigin - iPicStride - 1;
    piAdiLineTemp = piAdiLine + (iNumUnits2*unitHeight);
    pbNeighborFlags = bNeighborFlags + iNumUnits2;
    if (*pbNeighborFlags)
    {
      Pel topLeftVal=piRoiTemp[0];
      for (i=0; i<unitWidth; i++)
      {
        piAdiLineTemp[i] = topLeftVal;
      }
    }

    // Fill left & below-left samples
    piRoiTemp += iPicStride;
    if (bLMmode)
    {
      piRoiTemp --; // move the second left column
    }
    piAdiLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iNumUnits2; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitHeight; i++)
        {
          piAdiLineTemp[-i] = piRoiTemp[i*iPicStride];
        }
      }
      piRoiTemp += unitHeight*iPicStride;
      piAdiLineTemp -= unitHeight;
      pbNeighborFlags--;
    }

    // Fill above & above-right samples (each unit has "unitWidth" samples"
    piRoiTemp = piRoiOrigin - iPicStride;
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piAdiLineTemp = piAdiLine + iNumUnits2*unitHeight + unitWidth;
    pbNeighborFlags = bNeighborFlags + iNumUnits2 + 1;
    for (j=0; j<iNumUnits2; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitWidth; i++)
        {
          piAdiLineTemp[i] = piRoiTemp[i];
        }
      }
      piRoiTemp += unitWidth;
      piAdiLineTemp += unitWidth;
      pbNeighborFlags++;
    }

    // Pad reference samples when necessary
    Int iCurrJnit = 0;
    Pel  *piAdiLineCur   = piAdiLine;
    Pel  * const piAdiLineTopRowAdjusted = piAdiLine + iNumUnits2*unitHeight - iNumUnits2*unitWidth;

    if (!bNeighborFlags[0])
    {
      // very bottom unit of bottom-left; at least one unit will be valid.
      {
        Int   iNext = 1;
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])
        {
          iNext++;
        }
        Pel *piAdiLineNext=(iNext<iNumUnits2) ? (piAdiLine+iNext*unitHeight) : (piAdiLineTopRowAdjusted+iNext*unitWidth);
        const Pel refSample = *piAdiLineNext;
        // Pad unavailable samples with new value
        Int iNextOrTop=iNext < iNumUnits2 ? iNext : iNumUnits2;
        // fill left column
        while (iCurrJnit < iNextOrTop)
        {
          for (i=0; i<unitHeight; i++)
          {
            piAdiLineCur[i] = refSample;
          }
          piAdiLineCur += unitHeight;
          iCurrJnit++;
        }
        // fill top row
        while (iCurrJnit < iNext)
        {
          for (i=0; i<unitWidth; i++)
          {
            piAdiLineCur[i] = refSample;
          }
          piAdiLineCur += unitWidth;
          iCurrJnit++;
        }
      }
    }

    // pad all other reference samples.
    while (iCurrJnit < iTotalUnits)
    {
      if (!bNeighborFlags[iCurrJnit]) // samples not available
      {
        {
          const Int numSamplesInCurrUnit = iCurrJnit >= iNumUnits2 ? unitWidth : unitHeight;
          const Pel refSample = *(piAdiLineCur-1);
          for (i=0; i<numSamplesInCurrUnit; i++)
          {
            piAdiLineCur[i] = refSample;
          }
          piAdiLineCur += numSamplesInCurrUnit;
          iCurrJnit++;
        }
      }
      else
      {
        piAdiLineCur += iCurrJnit >= iNumUnits2 ? unitWidth : unitHeight;
        iCurrJnit++;
      }
    }

    // Copy processed samples
    if (nonScaledIntraChroma422(chType, chFmt))
    {
      piAdiLineTemp = piAdiLine + uiCuHeight*2 + unitWidth - 1;
      const Int limitX=uiCuWidth*2;
      for (i=0; i<limitX; i++)
      {
        piAdiTemp[i] = piAdiLineTemp[i];
      }
      for (; i<uiWidth; i++)
      {
        piAdiTemp[i] = piAdiLineTemp[limitX];
      }

      piAdiLineTemp = piAdiLine + uiCuHeight*2;
      for (i=1; i<uiHeight; i++)
      {
        piAdiTemp[i*uiWidth] = piAdiLineTemp[-i];
      }
    }
    else
    {
      piAdiLineTemp = piAdiLine + uiHeight + unitWidth - 2;
      // top left, top and top right samples
      for (i=0; i<uiWidth; i++)
      {
        piAdiTemp[i] = piAdiLineTemp[i];
      }

      piAdiLineTemp = piAdiLine + uiHeight - 1;
      for (i=1; i<uiHeight; i++)
      {
        piAdiTemp[i*uiWidth] = piAdiLineTemp[-i];
      }
    }
  }
}

/** Get pointer to reference samples for intra prediction
 * \param uiDirMode   prediction mode index
 * \param log2BlkSize size of block (2 = 4x4, 3 = 8x8, 4 = 16x16, 5 = 32x32, 6 = 64x64)
 * \param piAdiBuf    pointer to unfiltered reference samples
 * \return            pointer to (possibly filtered) reference samples
 *
 * The prediction mode index is used to determine whether a smoothed reference sample buffer is returned.
 */

Bool TComPrediction::filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt)
{
  Bool bFilter;
  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt))
  {
    bFilter=false;
  }
  else
  {
    assert(uiTuChWidth>=4 && uiTuChHeight>=4 && uiTuChWidth<128 && uiTuChHeight<128);

#if REMOVE_LMCHROMA
    if (uiDirMode == DC_IDX)
#else
    if (uiDirMode == DC_IDX || uiDirMode == LM_CHROMA_IDX)
#endif
    {
      bFilter=false; //no smoothing for DC or LM chroma
    }
    else
    {
      Int diff = min<Int>(abs((Int) uiDirMode - HOR_IDX), abs((Int)uiDirMode - VER_IDX));
      UInt sizeIndex=g_aucConvertToBit[uiTuChWidth];
      assert(sizeIndex < MAX_INTRA_FILTER_DEPTHS);
      bFilter = diff > m_aucIntraFilter[toChannelType(compID)][sizeIndex];
    }
  }
  return bFilter;
}

Bool isAboveLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT )
{
  Bool bAboveLeftFlag;
  UInt uiPartAboveLeft;
  TComDataCU* pcCUAboveLeft = pcCU->getPUAboveLeft( uiPartAboveLeft, uiPartIdxLT, true, false );
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
  {
    bAboveLeftFlag = ( pcCUAboveLeft && pcCUAboveLeft->getPredictionMode( uiPartAboveLeft ) == MODE_INTRA );
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }
  return bAboveLeftFlag;
}

Int isAboveAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxRT]+1;
  const UInt uiIdxStep = 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartAbove;
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart], true, false );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAbove && pcCUAbove->getPredictionMode( uiPartAbove ) == MODE_INTRA )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if (pcCUAbove)
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxLB]+1;
  const UInt uiIdxStep = pcCU->getPic()->getNumPartInWidth();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartLeft;
    TComDataCU* pcCULeft = pcCU->getPULeft( uiPartLeft, g_auiRasterToZscan[uiRasterPart], true, false );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCULeft && pcCULeft->getPredictionMode( uiPartLeft ) == MODE_INTRA )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCULeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
{
  const UInt uiNumUnitsInPU = g_auiZscanToRaster[uiPartIdxRT] - g_auiZscanToRaster[uiPartIdxLT] + 1;
  const UInt uiPuWidth = uiNumUnitsInPU * pcCU->getPic()->getMinCUWidth();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartAboveRight;
    TComDataCU* pcCUAboveRight = pcCU->getPUAboveRightAdi( uiPartAboveRight, uiPuWidth, uiPartIdxRT, uiOffset, true, false );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAboveRight && pcCUAboveRight->getPredictionMode( uiPartAboveRight ) == MODE_INTRA )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUAboveRight )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
{
  const UInt uiNumUnitsInPU = (g_auiZscanToRaster[uiPartIdxLB] - g_auiZscanToRaster[uiPartIdxLT]) / pcCU->getPic()->getNumPartInWidth() + 1;
  const UInt uiPuHeight = uiNumUnitsInPU * pcCU->getPic()->getMinCUHeight();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartBelowLeft;
    TComDataCU* pcCUBelowLeft = pcCU->getPUBelowLeftAdi( uiPartBelowLeft, uiPuHeight, uiPartIdxLB, uiOffset, true, false );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUBelowLeft && pcCUBelowLeft->getPredictionMode( uiPartBelowLeft ) == MODE_INTRA )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUBelowLeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
//! \}
