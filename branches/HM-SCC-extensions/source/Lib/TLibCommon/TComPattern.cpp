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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"
#if PLT_IMPROVE_GEN
#include "cmath"
#include <algorithm>
#endif
//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
#if RExt__O0043_BEST_EFFORT_DECODING
Void fillReferenceSamples( const Int bitDepth, const Int bitDepthDelta, TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
#else
Void fillReferenceSamples( const Int bitDepth, TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
#endif
                           const Int iNumIntraNeighbor, const Int unitWidth, const Int unitHeight, const Int iAboveUnits, const Int iLeftUnits,
                           const UInt uiCuWidth, const UInt uiCuHeight, const UInt uiWidth, const UInt uiHeight, const Int iPicStride,
                           const ChannelType chType, const ChromaFormat chFmt );

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
 \param  iRoiWidth     pattern width
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


// NOTE: RExt - this has been kept in this C++ file to allow easier tracking/comparison against HM.
Void TComPrediction::initAdiPatternChType( TComTU &rTu, Bool& bAbove, Bool& bLeft, const ComponentID compID, const Bool bFilterRefSamples DEBUG_STRING_FN_DECLARE(sDebug))
{
  // NOTE: RExt - This function has been modified for increased flexibility as part of the Square TU 4:2:2 implementation
  const ChannelType chType    = toChannelType(compID);

  TComDataCU *pcCU=rTu.getCU();
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
  const UInt uiTuWidth2       = uiTuWidth  << 1;
  const UInt uiTuHeight2      = uiTuHeight << 1;

  const Int  iBaseUnitSize    = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;
  const Int  iLeftUnits       = iTUHeightInUnits << 1;

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInWidth(); //NOTE: RExt - despite the name "TComPic::getNumPartInWidth", this gets the number of parts in an **LCU** width, not the picture width
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCU() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_SPU_W + 1];
  Int   iNumIntraNeighbor = 0;

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );

  bAbove = true;
  bLeft  = true;

  const ChromaFormat chFmt       = rTu.GetChromaFormat();
  const UInt         uiROIWidth  = uiTuWidth2+1;
  const UInt         uiROIHeight = uiTuHeight2+1;

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);

#ifdef DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piAdiTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiZorderIdxInPart);
#if RExt__O0043_BEST_EFFORT_DECODING
    fillReferenceSamples (g_bitDepthInStream[chType], g_bitDepthInStream[chType] - g_bitDepth[chType], pcCU, piRoiOrigin, piAdiTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
#else
    fillReferenceSamples (g_bitDepth[chType], pcCU, piRoiOrigin, piAdiTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
#endif
                          uiTuWidth, uiTuHeight, uiROIWidth, uiROIHeight, iPicStride, toChannelType(compID), chFmt);


#ifdef DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
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

            Int          stride    = uiROIWidth;
      const Pel         *piSrcPtr  = piAdiTemp                             + (stride * uiTuHeight2); // bottom left
            Pel         *piDestPtr = m_piYuvExt[compID][PRED_BUF_FILTERED] + (stride * uiTuHeight2); // bottom left

      //------------------------------------------------

      Bool useStrongIntraSmoothing = isLuma(chType) && pcCU->getSlice()->getSPS()->getUseStrongIntraSmoothing();

      const Pel bottomLeft = piAdiTemp[stride * uiTuHeight2];
      const Pel topLeft    = piAdiTemp[0];
      const Pel topRight   = piAdiTemp[uiTuWidth2];

      if (useStrongIntraSmoothing)
      {
#if RExt__O0043_BEST_EFFORT_DECODING
        const Int  threshold     = 1 << (g_bitDepthInStream[chType] - 5);
#else
        const Int  threshold     = 1 << (g_bitDepth[chType] - 5);
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piAdiTemp[stride * uiTuHeight])) < threshold; //difference between the
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piAdiTemp[         uiTuWidth ])) < threshold; //ends and the middle
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))
          useStrongIntraSmoothing = false;
      }

      *piDestPtr = *piSrcPtr; // bottom left is not filtered
      piDestPtr -= stride;
      piSrcPtr  -= stride;

      //------------------------------------------------

      //left column (bottom to top)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);
      }
      else
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      //top-left

      if (useStrongIntraSmoothing)
      {
        *piDestPtr = piSrcPtr[0];
      }
      else
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;
      piSrcPtr  += 1;

      //------------------------------------------------

      //top row (left-to-right)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuWidth] + 3; //log2(uiTuWidth2)

        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++)
        {
          *piDestPtr = (((uiTuWidth2 - i) * topLeft) + (i * topRight) + uiTuWidth) >> shift;
        }

        piSrcPtr += uiTuWidth2 - 1;
      }
      else
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      *piDestPtr=*piSrcPtr; // far right is not filtered

#ifdef DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: filtered result for channel " << compID <<"\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
            ss << m_piYuvExt[compID][PRED_BUF_FILTERED][y*uiROIWidth + x] << ", ";
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

#if RExt__O0043_BEST_EFFORT_DECODING
Void fillReferenceSamples( const Int bitDepth, const Int bitDepthDelta, TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
#else
Void fillReferenceSamples( const Int bitDepth, TComDataCU* pcCU, const Pel* piRoiOrigin, Pel* piAdiTemp, const Bool* bNeighborFlags,
#endif
                           const Int iNumIntraNeighbor, const Int unitWidth, const Int unitHeight, const Int iAboveUnits, const Int iLeftUnits,
                           const UInt uiCuWidth, const UInt uiCuHeight, const UInt uiWidth, const UInt uiHeight, const Int iPicStride,
                           const ChannelType chType, const ChromaFormat chFmt )
{
  // NOTE: RExt - This function has been modified for increased flexibility as part of the Square TU 4:2:2 implementation
  const Pel* piRoiTemp;
  Int  i, j;
  Int  iDCValue = 1 << (bitDepth - 1);
  const Int iTotalUnits = iAboveUnits + iLeftUnits + 1; //+1 for top-left

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

    for (i=0; i<uiWidth; i++)
    {
#if RExt__O0043_BEST_EFFORT_DECODING
      piAdiTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
      piAdiTemp[i] = piRoiTemp[i];
#endif
    }

    // Fill left and below left border with rec. samples
    piRoiTemp = piRoiOrigin - 1;

    for (i=1; i<uiHeight; i++)
    {
#if RExt__O0043_BEST_EFFORT_DECODING
      piAdiTemp[i*uiWidth] = (*(piRoiTemp)) << bitDepthDelta;
#else
      piAdiTemp[i*uiWidth] = *(piRoiTemp);
#endif
      piRoiTemp += iPicStride;
    }
  }
  else // reference samples are partially available
  {
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    const Int  iTotalSamples = (iLeftUnits * unitHeight) + ((iAboveUnits + 1) * unitWidth);
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
    piAdiLineTemp = piAdiLine + (iLeftUnits * unitHeight);
    pbNeighborFlags = bNeighborFlags + iLeftUnits;
    if (*pbNeighborFlags)
    {
#if RExt__O0043_BEST_EFFORT_DECODING
      Pel topLeftVal=piRoiTemp[0] << bitDepthDelta;
#else
      Pel topLeftVal=piRoiTemp[0];
#endif
      for (i=0; i<unitWidth; i++)
      {
        piAdiLineTemp[i] = topLeftVal;
      }
    }

    // Fill left & below-left samples (downwards)
    piRoiTemp += iPicStride;
    piAdiLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iLeftUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitHeight; i++)
        {
#if RExt__O0043_BEST_EFFORT_DECODING
          piAdiLineTemp[-i] = piRoiTemp[i*iPicStride] << bitDepthDelta;
#else
          piAdiLineTemp[-i] = piRoiTemp[i*iPicStride];
#endif
        }
      }
      piRoiTemp += unitHeight*iPicStride;
      piAdiLineTemp -= unitHeight;
      pbNeighborFlags--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    piRoiTemp = piRoiOrigin - iPicStride;
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piAdiLineTemp = piAdiLine + (iLeftUnits * unitHeight) + unitWidth;
    pbNeighborFlags = bNeighborFlags + iLeftUnits + 1;
    for (j=0; j<iAboveUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitWidth; i++)
        {
#if RExt__O0043_BEST_EFFORT_DECODING
          piAdiLineTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
          piAdiLineTemp[i] = piRoiTemp[i];
#endif
        }
      }
      piRoiTemp += unitWidth;
      piAdiLineTemp += unitWidth;
      pbNeighborFlags++;
    }

    // Pad reference samples when necessary
    Int iCurrJnit = 0;
    Pel  *piAdiLineCur   = piAdiLine;
    const UInt piAdiLineTopRowOffset = iLeftUnits * (unitHeight - unitWidth);

    if (!bNeighborFlags[0])
    {
      // very bottom unit of bottom-left; at least one unit will be valid.
      {
        Int   iNext = 1;
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])
        {
          iNext++;
        }
        Pel *piAdiLineNext = piAdiLine + ((iNext < iLeftUnits) ? (iNext * unitHeight) : (piAdiLineTopRowOffset + (iNext * unitWidth)));
        const Pel refSample = *piAdiLineNext;
        // Pad unavailable samples with new value
        Int iNextOrTop = std::min<Int>(iNext, iLeftUnits);
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
          const Int numSamplesInCurrUnit = (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
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
        piAdiLineCur += (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
        iCurrJnit++;
      }
    }

    // Copy processed samples

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

/** Get pointer to reference samples for intra prediction
 * \param uiDirMode   prediction mode index
 * \param log2BlkSize size of block (2 = 4x4, 3 = 8x8, 4 = 16x16, 5 = 32x32, 6 = 64x64)
 * \param piAdiBuf    pointer to unfiltered reference samples
 * \return            pointer to (possibly filtered) reference samples
 *
 * The prediction mode index is used to determine whether a smoothed reference sample buffer is returned.
 */

Bool TComPrediction::filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled)
{
  Bool bFilter;

  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt, intraReferenceSmoothingDisabled))
  {
    bFilter=false;
  }
  else
  {
    assert(uiTuChWidth>=4 && uiTuChHeight>=4 && uiTuChWidth<128 && uiTuChHeight<128);

    if (uiDirMode == DC_IDX)
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
  TComDataCU* pcCUAboveLeft = pcCU->getPUAboveLeft( uiPartAboveLeft, uiPartIdxLT );
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
  {
    bAboveLeftFlag = ( pcCUAboveLeft && pcCUAboveLeft->isConstrainedIntra( uiPartAboveLeft ) );
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
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAbove && pcCUAbove->isConstrainedIntra( uiPartAbove ) )
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
    TComDataCU* pcCULeft = pcCU->getPULeft( uiPartLeft, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCULeft && pcCULeft->isConstrainedIntra( uiPartLeft ) )
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
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartAboveRight;
    TComDataCU* pcCUAboveRight = pcCU->getPUAboveRightAdi( uiPartAboveRight, uiPartIdxRT, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAboveRight && pcCUAboveRight->isConstrainedIntra( uiPartAboveRight ) )
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
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartBelowLeft;
    TComDataCU* pcCUBelowLeft = pcCU->getPUBelowLeftAdi( uiPartBelowLeft, uiPartIdxLB, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUBelowLeft && pcCUBelowLeft->isConstrainedIntra( uiPartBelowLeft ) )
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

#if PALETTE_MODE
Void TComPrediction::deriveRunLuma (TComDataCU* pcCU, Pel* pOrg,  Pel *pPalette,  Pel* pValue, Pel* pSPoint, Pel* pEscapeFlag, Pel* pPixelPredFlag,   Pel * paRecoValue, Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize)
{
  UInt uiTotal = uiHeight * uiWidth, uiIdx = 0, uiStride = uiWidth;
  UInt uiStartPos = 0,  uiRun = 0, uiCopyRun = 0;
  Int iTemp = 0;

  //Test Run
  while (uiIdx < uiTotal)
  {
    uiStartPos = uiIdx;

    uiRun = 0;
    Bool RunValid = CalRun(pValue, pEscapeFlag, uiStartPos, uiTotal, uiRun);
    uiCopyRun = 0;
    Bool CopyValid = CalCopy(pValue, pEscapeFlag, uiWidth, uiStartPos, uiTotal, uiCopyRun);
    if (CopyValid == 0 && RunValid == 0)
    {
      pEscapeFlag[uiIdx] = 1;
      pPixelPredFlag[uiIdx] = CalcPixelPredLuma(pcCU, pOrg, pPalette, pValue, pEscapeFlag,   paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiStartPos );
      pSPoint[uiIdx] = 0;
      uiIdx++;
    }
    else
    {
      pEscapeFlag[uiIdx] = 0;
      pPixelPredFlag[uiIdx] = 0;
      paRecoValue [uiIdx] = pPalette[pValue[uiIdx]];
      if ( CopyValid && (uiCopyRun + 2 > uiRun))
      {
        pSPoint[uiIdx] = 1;
        pRun[uiIdx]  = uiCopyRun-1;

        uiIdx++;
        iTemp = uiCopyRun - 1;
        while (iTemp>0)
        {
          pValue[uiIdx] = pValue[uiIdx - uiStride];
          paRecoValue[uiIdx] = pPalette[pValue[uiIdx]];
          pSPoint[uiIdx] = 1;
          pEscapeFlag[uiIdx] = 0;
          uiIdx++;
          iTemp--;
        }
      }
      else
      {
        pSPoint[uiIdx] = 0;
        pRun[uiIdx] = uiRun;
        uiIdx++;
        iTemp = uiRun;
        while (iTemp>0)
        {
          pValue[uiIdx] = pValue[uiIdx - 1];
          paRecoValue [uiIdx] = pPalette[pValue[uiIdx]];
          pSPoint[uiIdx] = 0;
          pEscapeFlag[uiIdx] = 0;
          uiIdx++;
          iTemp--;
        }
      }
    }
  }
  assert (uiIdx == uiTotal);
}

Void  TComPrediction:: deriveRunChroma (TComDataCU* pcCU, Pel* pOrg [2],  Pel *pPalette [2],  Pel* pValue[2], Pel* pSPoint, Pel* pEscapeFlag, Pel* pPixelPredFlag,   Pel * paRecoValue[2], Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize)
{

  UInt uiTotal = uiHeight * uiWidth, uiIdx = 0;
  UInt uiStartPos = 0,  uiRun = 0, uiCopyRun = 0;

  Int iTemp = 0;
  UInt uiStride = uiWidth;
  //Test Run
  while (uiIdx < uiTotal)
  {
    uiStartPos = uiIdx;
    uiRun = 0;

    Bool RunValid = false;
    RunValid = CalRun(pValue[0], pEscapeFlag, uiStartPos, uiTotal, uiRun, true);
    uiCopyRun = 0;
    Bool CopyValid = false;
    CopyValid = CalCopy(pValue[0], pEscapeFlag, uiWidth, uiStartPos, uiTotal, uiCopyRun, true);
    if (CopyValid == 0 && RunValid == 0)
    {
      pEscapeFlag[uiIdx] = 1;
      pPixelPredFlag[uiIdx] = CalcPixelPredChroma(pcCU, pOrg, pPalette, pValue, pEscapeFlag,   paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiStartPos );
      pSPoint[uiIdx] = 0;
      uiIdx++;
    }
    else
    {
      pEscapeFlag[uiIdx] = 0;
      pPixelPredFlag[uiIdx] = 0;
      paRecoValue[0] [uiIdx] = pPalette[0][pValue[0][uiIdx]];
      paRecoValue[1] [uiIdx] = pPalette[1][pValue[0][uiIdx]];

      if ( CopyValid && (uiCopyRun + 2 > uiRun))
      {
        pSPoint[uiIdx] = 1;
        pRun[uiIdx]  = uiCopyRun-1;
        uiIdx++;
        iTemp = uiCopyRun - 1;
        while (iTemp>0)
        {
          pValue[0][uiIdx] = pValue[0][uiIdx-uiStride];
          paRecoValue[0] [uiIdx] = pPalette[0][pValue[0][uiIdx]];
          paRecoValue[1] [uiIdx] = pPalette[1][pValue[0][uiIdx]];

          pSPoint[uiIdx] = 1;
          pEscapeFlag[uiIdx] = 0;
          uiIdx++;
          iTemp--;
        }
      }
      else
      {
        pSPoint[uiIdx] = 0;
        pRun[uiIdx]  = uiRun;
        uiIdx++;
        iTemp = uiRun;
        while (iTemp>0)
        {
          pValue[0][uiIdx] = pValue[0][uiIdx-1];
          paRecoValue[0] [uiIdx] = pPalette[0][pValue[0][uiIdx]];
          paRecoValue[1] [uiIdx] = pPalette[1][pValue[0][uiIdx]];

          pSPoint[uiIdx] = 0;
          pEscapeFlag[uiIdx] = 0;
          uiIdx++;
          iTemp--;
        }
      }
    }
  }

  assert (uiIdx == uiTotal);
}


Void TComPrediction::deriveRun(TComDataCU* pcCU, Pel* pOrg [3],  Pel *pPalette [3],  Pel* pValue, Pel* pSPoint, Pel* pEscapeFlag, Pel* pPixelPredFlag,  Pel** paPixelValue, Pel ** paRecoValue, Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize)
{
  UInt   uiTotal = uiHeight * uiWidth, uiIdx = 0;
  UInt uiStartPos = 0,  uiRun = 0, uiCopyRun = 0;

  Int iTemp = 0;  
  UInt uiStride = uiWidth;

  UInt uiTraIdx;  //unified position variable (raster scan)

  //Test Run
  while (uiIdx < uiTotal)
  {
    uiStartPos = uiIdx;
    uiRun = 0;

    Bool RunValid = false;
    RunValid = CalRun(pValue, pEscapeFlag, uiStartPos, uiTotal, uiRun);
    uiCopyRun = 0;
    Bool CopyValid = false;
    CopyValid = CalCopy(pValue, pEscapeFlag, uiWidth, uiStartPos, uiTotal, uiCopyRun);
    uiTraIdx = getIdxScanPos(uiIdx);    //unified position variable (raster scan)

    if (CopyValid == 0 && RunValid == 0)
    {
      pEscapeFlag[uiTraIdx] = 1;
      pPixelPredFlag[uiTraIdx] = CalcPixelPred(pcCU, pOrg, pPalette, pValue, pEscapeFlag,  paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx );

      pSPoint[uiTraIdx] = 0;
      uiIdx++;
    }
    else
    {
      pEscapeFlag[uiTraIdx] = 0;
      pPixelPredFlag[uiTraIdx] = 0;
      if ( CopyValid && (uiCopyRun + 2 > uiRun))
      {
        pSPoint[uiTraIdx] = 1;
        pRun[uiTraIdx]  = uiCopyRun-1;
        uiIdx++;
        uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

        iTemp = uiCopyRun - 1;
        while (iTemp>0)
        {
          pValue[uiTraIdx] = pValue[uiTraIdx-uiStride];
          pSPoint[uiTraIdx] = 1;
          pEscapeFlag[uiTraIdx] = 0;
          uiIdx++;
          uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

          iTemp--;
        }
      }
      else
      {
        pSPoint[uiTraIdx] = 0;
        pRun[uiTraIdx] = uiRun;

        uiIdx++;
        uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

        iTemp = uiRun;
        while (iTemp > 0)
        {
          pValue[uiTraIdx] = pValue[getIdxScanPos(uiIdx - 1)];
          pSPoint[uiTraIdx] = 0;
          pEscapeFlag[uiTraIdx] = 0;
          uiIdx++;
          uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

          iTemp--;
        }
      }
    }
  }
  assert (uiIdx == uiTotal);
}
#if PLT_IMPROVE_GEN
Void  TComPrediction::derivePLTLuma(TComDataCU* pcCU, Pel *Palette, Pel* pSrc, UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : getPLTErrLimit();

  UInt uiTotalSize = uiHeight * uiWidth;
  SortingElementLuma *psList = new SortingElementLuma[uiTotalSize];
  SortingElementLuma sElement;

  //UInt uiDictMaxSize = MAX_PLT_SIZE;
  //SortingElementLuma *pListSort = new SortingElementLuma [MAX_PLT_SIZE+1];
  SortingElementLuma *pListSort = new SortingElementLuma[uiTotalSize + 1];

  UInt uiIdx = 0;
  //UInt uiShift = 0;
  UInt uiPos;
  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    //uiShift     =  uiY * uiWidth;
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiWidth + uiX;
      //sElement.setAll (pSrc[uiShift + uiX]);
      sElement.setAll(pSrc[uiPos]);
      Int i = 0;
      for (i = uiIdx - 1; i >= 0; i--)
      {
        //if (psList[i].almostEqualData(sElement, iErrorLimit))
        if (psList[i].exactlyEqualData(sElement))
        {
          psList[i].uiCnt++;
          break;
        }
      }
      if (i == -1)
      {
        psList[uiIdx].copyDataFrom(sElement);
        psList[uiIdx].uiCnt++;
        uiIdx++;
      }
    }
  }

  for (Int i = 0; i < uiIdx; i++)
  {
    pListSort[i].copyAllFrom(psList[i]);
    for (Int j = i; j > 0; j--)
    {
      if (pListSort[j].uiCnt > pListSort[j - 1].uiCnt)
      {
        sElement.copyAllFrom(pListSort[j - 1]);
        pListSort[j - 1].copyAllFrom(pListSort[j]);
        pListSort[j].copyAllFrom(sElement);
      }
      else
      {
        break;
      }
    }
  }

  uiPLTSize = 0;
  for (Int i = 0; i < uiIdx; i++)
  {
    for (Int j = i - 1; j >= 0; j--)
    {
      if (pListSort[j].uiCnt && pListSort[i].almostEqualData(pListSort[j], iErrorLimit))
      {
        pListSort[j].uiCnt += pListSort[i].uiCnt;
        pListSort[i].uiCnt = 0;
        break;
      }
    }

    if (pListSort[i].uiCnt)
    {
      Palette[uiPLTSize] = pListSort[i].uiDataLuma;
      uiPLTSize++;
      if (uiPLTSize == MAX_PLT_SIZE)
      {
        break;
      }
    }
  }

  delete[] psList;
  delete[] pListSort;
}

Void  TComPrediction::derivePLTChroma (TComDataCU* pcCU, Pel *Palette[2], Pel* pSrc[2],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{

  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless? 0 : getPLTErrLimit();

  UInt uiTotalSize = uiHeight * uiWidth;
  SortingElementChroma *psList = new SortingElementChroma [uiTotalSize];
  SortingElementChroma sElement;

  //UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElementChroma *pListSort = new SortingElementChroma[uiTotalSize + 1];

  UInt uiIdx = 0;
  UInt uiPos;
  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      uiPos = uiY * uiWidth + uiX;
      sElement.setAll (pSrc[0][uiPos], pSrc[1][uiPos]);
      Int i = 0;
      for ( i = uiIdx - 1; i >= 0; i--)
      { 
        //if (psList[i].almostEqualData(sElement, iErrorLimit))
        if (psList[i].exactlyEqualData(sElement))
        {
          psList[i].uiCnt++;
          break;
        }
      }

      if (i == -1)
      {
        psList[uiIdx].copyDataFrom(sElement);
        psList[uiIdx].uiCnt++;
        uiIdx++;
      }
    }
  }

  for (Int i = 0; i < uiIdx; i++)
  {
    pListSort[i].copyAllFrom (psList[i]);
    for (Int j = i; j > 0; j--) 
    {
      if (pListSort[j].uiCnt > pListSort[j - 1].uiCnt)
      {
        sElement.copyAllFrom (pListSort[j - 1]) ;
        pListSort[j - 1].copyAllFrom (pListSort[j]) ;
        pListSort[j].copyAllFrom (sElement) ;
      }
      else
      {
        break;
      }
    }
  }

  uiPLTSize = 0;
  for (Int i = 0; i < uiIdx; i++)
  {
    for (Int j = i - 1; j >= 0; j--)
    {
      if (pListSort[j].uiCnt && pListSort[i].almostEqualData(pListSort[j], iErrorLimit))
      {
        pListSort[j].uiCnt += pListSort[i].uiCnt;
        pListSort[i].uiCnt = 0;
        break;
      }
    }

    if (pListSort[i].uiCnt)
    {
      Palette[0][uiPLTSize] = pListSort[i].uiData[0];
      Palette[1][uiPLTSize] = pListSort[i].uiData[1];
      uiPLTSize++;
      if (uiPLTSize == MAX_PLT_SIZE)
      {
        break;
      }
    }
  }

  delete [] psList;
  delete [] pListSort;
}

#else
Void  TComPrediction::derivePLTLuma (TComDataCU* pcCU, Pel *Palette, Pel* pSrc,  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{

  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless? 0: getPLTErrLimit();

  UInt uiTotalSize = uiHeight*uiWidth;
  SortingElementLuma *psList = new SortingElementLuma [uiTotalSize];
  SortingElementLuma sElement;

  UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElementLuma *pListSort = new SortingElementLuma [MAX_PLT_SIZE+1];

  UInt uiIdx = 0;
  UInt uiShift = 0;
  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    uiShift     =  uiY * uiWidth;
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      sElement.setAll (pSrc[uiShift + uiX]);
      Int i = 0;
      for ( i = uiIdx-1; i>=0; i--)
      { 
        if (psList[i].almostEqualData(sElement, iErrorLimit))
        {

          psList[i].uiCnt++;

          break;
        }
      }

      if (i == -1)
      {
        psList[uiIdx].copyDataFrom(sElement);

        psList[uiIdx].uiCnt++;

        uiIdx ++;
      }
    }
  }


  for (Int i = 0; i < uiDictMaxSize; i++)
  {
    pListSort[i].uiCnt  = 0;
    pListSort[i].setAll(0) ;
  }

  for (Int i = 0; i < uiTotalSize; i++)
  {
    pListSort[uiDictMaxSize].copyAllFrom (psList[i]);

    for (Int j = uiDictMaxSize; j > 0; j--) 
    {
      if (pListSort[j].uiCnt > pListSort[j-1].uiCnt)
      {
        sElement.copyAllFrom (pListSort[j-1]) ;
        pListSort[j-1].copyAllFrom (pListSort[j]) ;
        pListSort[j].copyAllFrom (sElement) ;
      }
      else
      {
        break;
      }
    }
  }

  uiPLTSize = MAX_PLT_SIZE;
  for (Int i = 0; i < MAX_PLT_SIZE; i++)
  {
    Palette[i] = pListSort[i].uiDataLuma;
    if (pListSort[i].uiCnt == 0  )
    {
      uiPLTSize = i;
      break;
    }
  }

  delete [] psList;
  delete [] pListSort;


}

Void  TComPrediction::derivePLTChroma (TComDataCU* pcCU, Pel *Palette[2], Pel* pSrc[2],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{

  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless? 0: getPLTErrLimit();

  UInt uiTotalSize = uiHeight*uiWidth;
  SortingElementChroma *psList = new SortingElementChroma [uiTotalSize];
  SortingElementChroma sElement;

  UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElementChroma *pListSort = new SortingElementChroma [MAX_PLT_SIZE+1];

  UInt uiIdx = 0;
  //UInt uiShift = 0, uiCbCrShift = 0;
  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    UInt uiShift     =  uiY * uiWidth;
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      sElement.setAll (pSrc[0][uiShift + uiX], pSrc[1][uiShift + uiX]);
      Int i = 0;
      for ( i = uiIdx-1; i>=0; i--)
      { 
        if (psList[i].almostEqualData(sElement, iErrorLimit))
        {

          psList[i].uiCnt++;

          break;
        }
      }

      if (i == -1)
      {
        psList[uiIdx].copyDataFrom(sElement);

        psList[uiIdx].uiCnt++;

        uiIdx ++;

      }
    }
  }


  for (Int i = 0; i < uiDictMaxSize; i++)
  {
    pListSort[i].uiCnt  = 0;
    pListSort[i].setAll(0, 0) ;
  }

  for (Int i = 0; i < uiTotalSize; i++)
  {
    pListSort[uiDictMaxSize].copyAllFrom (psList[i]);

    for (Int j = uiDictMaxSize; j > 0; j--) 
    {
      if (pListSort[j].uiCnt > pListSort[j-1].uiCnt)
      {
        sElement.copyAllFrom (pListSort[j-1]) ;
        pListSort[j-1].copyAllFrom (pListSort[j]) ;
        pListSort[j].copyAllFrom (sElement) ;
      }
      else
      {
        break;
      }
    }
  }

  uiPLTSize = MAX_PLT_SIZE;
  for (Int i = 0; i < MAX_PLT_SIZE; i++)
  {
    Palette[0][i] = pListSort[i].uiData[0];
    Palette[1][i] = pListSort[i].uiData[1];
    if (pListSort[i].uiCnt == 0  )
    {
      uiPLTSize = i;
      break;
    }
  }

  delete [] psList;
  delete [] pListSort;


}

#endif

Void TComPrediction::preCalcPLTIndexLuma(TComDataCU* pcCU, Pel *Palette, Pel* pSrc, UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : getPLTErrLimit();
  UInt uiPos;
  UInt uiBestIdx = 0;

  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiStride + uiX;
      UInt uiPLTIdx = 0;
      UInt uiMinError = MAX_UINT;

      while (uiPLTIdx < uiPLTSize)
      {
        UInt uiAbsError = abs(Palette[uiPLTIdx] - pSrc[uiPos]);

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
        m_cIndexBlock[uiPos] -= MAX_PLT_SIZE;
      }
    }
  }
}

Void  TComPrediction::preCalcPLTIndexChroma(TComDataCU* pcCU, Pel *Palette[2], Pel* pSrc[2], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : 2 * getPLTErrLimit();
  UInt uiPos;
  UInt uiBestIdx = 0;

  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiStride + uiX;
      UInt uiPLTIdx = 0;
      UInt uiMinError = MAX_UINT;
      while (uiPLTIdx < uiPLTSize)
      {
        UInt uiAbsError = abs(Palette[0][uiPLTIdx] - pSrc[0][uiPos]) + abs(Palette[1][uiPLTIdx] - pSrc[1][uiPos]);
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
      m_cIndexBlockChroma[uiPos] = uiBestIdx;
      if (uiMinError > iErrorLimit)
      {
        m_cIndexBlockChroma[uiPos] -= MAX_PLT_SIZE;
      }
    }
  }
}

Void TComPrediction::preCalcPLTIndex(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : 3 * getPLTErrLimit();
  UInt uiPos;

  UInt uiBestIdx = 0;

#if PLT_CU_ESCAPE_FLAG
  UChar useEscapeFlag=0;
#endif

  for (UInt uiY = 0; uiY < uiHeight; uiY++)
  {
    for (UInt uiX = 0; uiX < uiWidth; uiX++)
    {
      uiPos = uiY * uiWidth + uiX;
      UInt uiPLTIdx = 0;
      UInt uiMinError = MAX_UINT;
      while (uiPLTIdx < uiPLTSize)
      {
        UInt uiAbsError = abs(Palette[0][uiPLTIdx] - pSrc[0][uiPos]) + abs(Palette[1][uiPLTIdx] - pSrc[1][uiPos]) + abs(Palette[2][uiPLTIdx] - pSrc[2][uiPos]);
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
        m_cIndexBlock[uiPos] -= MAX_PLT_SIZE;
#if PLT_CU_ESCAPE_FLAG
        useEscapeFlag=1;
#endif
      }
    }
  }


#if PLT_CU_ESCAPE_FLAG
  pcCU->setPLTEscapeSubParts(0, useEscapeFlag,0, pcCU->getDepth(0));
#endif
}

Void  TComPrediction::reorderPLT(TComDataCU* pcCU, Pel *pPalette[3], UInt uiNumComp)
{

  UInt uiPLTSizePrev, uiDictMaxSize;
#if PLT_SHARING
  UInt uiPLTUsedSizePrev;
#endif
  Pel * pPalettePrev[3];
  Pel pPaletteTemp[3][MAX_PLT_SIZE];
  ComponentID compBegin = ComponentID(uiNumComp == 2 ? 1 : 0);


  for (UInt comp = compBegin; comp < compBegin + uiNumComp; comp++)
  {
#if PLT_SHARING
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), comp, uiPLTSizePrev, uiPLTUsedSizePrev);
#else
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), comp, uiPLTSizePrev);
#endif
    for (UInt i = 0; i < MAX_PLT_SIZE; i++)
    {
      pPaletteTemp[comp][i] = pPalette[comp][i];
    }
  }

#if PLT_SHARING
  pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), compBegin, uiPLTSizePrev, uiPLTUsedSizePrev);
#else
  pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), compBegin, uiPLTSizePrev);
#endif
  uiDictMaxSize = pcCU->getPLTSize(compBegin, 0);

  UInt uiIdxPrev = 0, uiIdxCurr = 0;
  Bool bReused = false;
  Bool bPredicted[MAX_PLT_SIZE + 1], bReusedPrev[PALETTE_PREDICTION_SIZE + 1];
  memset(bReusedPrev, 0, sizeof(bReusedPrev));
  memset(bPredicted, 0, sizeof(bPredicted));

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

  for (uiIdxPrev = 0; uiIdxPrev < PALETTE_PREDICTION_SIZE; uiIdxPrev++)
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
}

#if CANON_PALETTE_ENCODER
Void  TComPrediction::derivePLT( TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost )
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);

#if PLT_IMPROVE_GEN
  if( bLossless )
  {
    derivePLT( pcCU, Palette, pSrc, uiWidth, uiHeight, uiStride, uiPLTSize );
    return;
  }
#endif

  Int iErrorLimit = bLossless? 0: getPLTErrLimit();

  UInt uiTotalSize = uiHeight*uiWidth;
  SortingElement *psList = new SortingElement [uiTotalSize];
  SortingElement sElement;
  UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElement *pListSort = new SortingElement [MAX_PLT_SIZE+1];
  UInt uiIdx = 0;
  UInt uiPos;

  Int last = -1;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      uiPos = uiY * uiWidth+ uiX;
      sElement.setAll (pSrc[0][uiPos], pSrc[1][uiPos], pSrc[2][uiPos]);

      Int besti = last, bestSAD = last==-1 ? MAX_UINT : psList[last].getSAD(sElement);
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
  uiDictMaxSize=1;
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
          uiDictMaxSize = std::min(uiDictMaxSize+1,(UInt)MAX_PLT_SIZE);
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
#if PLT_IMPROVE_GEN
  Double bitCost = pcCost->getLambda()*24;
#else
  Double bitCost = pcCost->getLambda()*32;
#endif

  for (Int i = 0; i < MAX_PLT_SIZE; i++)
  {
    if( pListSort[i].uiCnt )
    {
      Int iHalf = pListSort[i].uiCnt>>1;
      Palette[0][uiPLTSize] = (pListSort[i].uiSumData[0]+iHalf)/pListSort[i].uiCnt;
      Palette[1][uiPLTSize] = (pListSort[i].uiSumData[1]+iHalf)/pListSort[i].uiCnt;
      Palette[2][uiPLTSize] = (pListSort[i].uiSumData[2]+iHalf)/pListSort[i].uiCnt;

#if PLT_IMPROVE_GEN
      Int best = -1;
#endif

      if( iErrorLimit )
      {
        Double pal[3] = { pListSort[i].uiSumData[0]/(Double)pListSort[i].uiCnt,
                          pListSort[i].uiSumData[1]/(Double)pListSort[i].uiCnt,
                          pListSort[i].uiSumData[2]/(Double)pListSort[i].uiCnt };
#if !PLT_IMPROVE_GEN
        Int best = -1;
#endif
        Double err      = pal[0] - Palette[0][uiPLTSize];
        Double bestCost = err*err;
        err = pal[1] - Palette[1][uiPLTSize]; bestCost += err*err;
        err = pal[2] - Palette[2][uiPLTSize]; bestCost += err*err;
        bestCost = bestCost * pListSort[i].uiCnt + bitCost;

        for(int t=0; t<pcCU->getLastPLTInLcuSizeFinal(0); t++)
        {
          err = pal[0] - pPred[0][t];
          Double cost = err*err;
          err = pal[1] - pPred[1][t]; cost += err*err;
          err = pal[2] - pPred[2][t]; cost += err*err;
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
#if PLT_IMPROVE_GEN
      if( pListSort[i].uiCnt == 1 && best == -1 )
      {
        bDuplicate = true;
      }
      else
#endif
      for( Int t=0; t<uiPLTSize; t++)
      {
        if( Palette[0][uiPLTSize] == Palette[0][t] && Palette[1][uiPLTSize] == Palette[1][t] && Palette[2][uiPLTSize] == Palette[2][t] )
        {
          bDuplicate = true;
          break;
        }
      }
      if( !bDuplicate ) uiPLTSize++;
    }
    else
      break;
  }

#if !PLT_IMPROVE_GEN
  if( uiPLTSize < MAX_PLT_SIZE )
  {
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        uiPos = uiY * uiStride+ uiX;
        Bool found = false;
        for( Int i = uiPLTSize-1; i>=0; i-- )
        {
          UInt err = abs(pSrc[0][uiPos]-Palette[0][i])
                   + abs(pSrc[1][uiPos]-Palette[1][i])
                   + abs(pSrc[2][uiPos]-Palette[2][i]);
          if( err <= 3*iErrorLimit )
          {
            found = true;
            break;
          }
        }

        if( !found )
        {
          Palette[0][uiPLTSize] = pSrc[0][uiPos];
          Palette[1][uiPLTSize] = pSrc[1][uiPos];
          Palette[2][uiPLTSize] = pSrc[2][uiPos];
          uiPLTSize++;
          if( uiPLTSize == MAX_PLT_SIZE ) goto end;
        }
      }
    }
  }
end:
#endif

  delete [] psList;
  delete [] pListSort;
}
#endif //CANON_PALETTE_ENCODER

#if PLT_IMPROVE_GEN
Void TComPrediction::derivePLT(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless ? 0 : getPLTErrLimit();
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
#if PLT_SHARING
  UInt uiPLTUsedSizePrev;
#endif
  Pel *pPalettePrev[3];
  for (UInt comp = 0; comp < 3; comp++)
  {
#if PLT_SHARING
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), comp, uiPLTSizePrev, uiPLTUsedSizePrev);    
#else
    pPalettePrev[comp] = pcCU->getPLTPred(pcCU, pcCU->getZorderIdxInCU(), comp, uiPLTSizePrev);    
#endif
  }

  uiPLTSize = 0;
  for (Int i = 0; i < uiIdx; i++)
  {
    for (Int j = i - 1; j >= 0; j--)
    {
      if (psList[j].uiCnt && psList[i].almostEqualData(psList[j], iErrorLimit))
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
      if (uiPLTSize == MAX_PLT_SIZE)
      {
        break;
      }
    }
  }
}
#elif !CANON_PALETTE_ENCODER
Void  TComPrediction::derivePLT (TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize)
{
  Bool bLossless = pcCU->getCUTransquantBypass(0);
  Int iErrorLimit = bLossless? 0: getPLTErrLimit();

  UInt uiTotalSize = uiHeight*uiWidth;
  SortingElement *psList = new SortingElement [uiTotalSize];
  SortingElement sElement;
  UInt uiDictMaxSize = MAX_PLT_SIZE;
  SortingElement *pListSort = new SortingElement [MAX_PLT_SIZE+1];
  UInt uiIdx = 0;
  UInt uiPos;
  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      uiPos = uiY * uiWidth+ uiX;
      sElement.setAll (pSrc[0][uiPos], pSrc[1][uiPos], pSrc[2][uiPos]);
      Int i = 0;
      for ( i = uiIdx-1; i>=0; i--)
      { 
        if (psList[i].almostEqualData(sElement, iErrorLimit))
        {

          psList[i].uiCnt++;

          break;
        }
      }
      if (i == -1)
      {
        psList[uiIdx].copyDataFrom(sElement);

        psList[uiIdx].uiCnt++;

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
  for (Int i = 0; i < uiTotalSize; i++)
  {
    pListSort[uiDictMaxSize].copyAllFrom (psList[i]);
    for (Int j = uiDictMaxSize; j > 0; j--) 
    {
      if (pListSort[j].uiCnt > pListSort[j-1].uiCnt)
      {
        sElement.copyAllFrom (pListSort[j-1]) ;
        pListSort[j-1].copyAllFrom (pListSort[j]) ;
        pListSort[j].copyAllFrom (sElement) ;
      }
      else
      {
        break;
      }
    }
  }

  uiPLTSize = MAX_PLT_SIZE;
  for (Int i = 0; i < MAX_PLT_SIZE; i++)
  {
    Palette[0][i] = pListSort[i].uiData[0];
    Palette[1][i] = pListSort[i].uiData[1];
    Palette[2][i] = pListSort[i].uiData[2];
    if (pListSort[i].uiCnt == 0  )
    {
      uiPLTSize = i;
      break;
    }
  }

  delete [] psList;
  delete [] pListSort;


}
#endif
Bool  TComPrediction::CalcPixelPredLuma(TComDataCU* pcCU, Pel* pOrg, Pel *pPalette, Pel* pValue, Pel * pEscapeFlag,   Pel*paRecoValue, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiIdx )
{
  Bool bLossless = pcCU->getCUTransquantBypass (0);
  const QpParam cQP(*pcCU, ComponentID(0));
  Int iQP = cQP.Qp;
  Int iQPrem = iQP%6;
  Int iQPper = iQP/6;
  Int quantiserScale =  g_quantScales[iQPrem];
  const Int quantiserRightShift = QUANT_SHIFT + iQPper;
  const Int rightShiftOffset = 1<< (quantiserRightShift-1);
  const Int InvquantiserRightShift = (IQUANT_SHIFT - iQPper);
  Int iAdd =  InvquantiserRightShift == 0? 0: 1 << (InvquantiserRightShift - 1);
  UInt uiY, uiX;
  uiY = uiIdx/uiWidth;
  uiX = uiIdx%uiWidth;

  UInt uiYIdxRaster = uiY*uiStrideOrg + uiX;
  UInt uiMaxBit[3];
  pcCU->xCalcMaxBits(pcCU, uiMaxBit);

  if (bLossless)
  {
    paRecoValue[uiYIdxRaster] = pValue[uiYIdxRaster] = pOrg[uiYIdxRaster];
  }
  else
  {
    pValue[uiYIdxRaster] = Pel(ClipBD<Int>((pOrg[uiYIdxRaster] * quantiserScale + rightShiftOffset) >> quantiserRightShift, uiMaxBit[0]));
    paRecoValue[uiYIdxRaster] = (pValue[uiYIdxRaster] * g_invQuantScales[iQPrem] + iAdd) >> InvquantiserRightShift;
    paRecoValue[uiYIdxRaster] = Pel(ClipBD<Int>(paRecoValue[uiYIdxRaster], g_bitDepth[0]));
  }

  return false; //do not use pixel prediction
}

Bool  TComPrediction::CalcPixelPredChroma(TComDataCU* pcCU, Pel* pOrg [2], Pel *pPalette[2], Pel* pValue[2], Pel * pEscapeFlag,   Pel*paRecoValue[2], UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiIdx )
{

  Bool bLossless = pcCU->getCUTransquantBypass (0);
  const QpParam cQP(*pcCU, ComponentID(1));//chroma
  Int iQP = cQP.Qp;
  Int iQPrem = iQP%6;
  Int iQPper = iQP/6;
  Int quantiserScale =  g_quantScales[iQPrem];
  const Int quantiserRightShift = QUANT_SHIFT + iQPper;
  const Int rightShiftOffset = 1<< (quantiserRightShift-1);
  const Int InvquantiserRightShift = (IQUANT_SHIFT - iQPper);
  Int iAdd =  InvquantiserRightShift == 0? 0: 1 << (InvquantiserRightShift - 1);
  UInt uiY, uiX;
  uiY = uiIdx/uiWidth;
  uiX = uiIdx%uiWidth;

  UInt uiYIdxRaster    = uiY*uiStrideOrg + uiX;
  UInt uiMaxBit[3];
  pcCU->xCalcMaxBits(pcCU, uiMaxBit);

  if (bLossless)
  {
    for (int i = 0; i < 2; i++)
    {
      pValue[i][uiYIdxRaster] = pOrg[i][uiYIdxRaster];
      paRecoValue[i][uiYIdxRaster] = pOrg[i][uiYIdxRaster];
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      pValue[i][uiYIdxRaster] = Pel(ClipBD<Int>((pOrg[i][uiYIdxRaster] * quantiserScale + rightShiftOffset) >> quantiserRightShift, uiMaxBit[1]));
      paRecoValue[i][uiYIdxRaster] = (pValue[i][uiYIdxRaster] * g_invQuantScales[iQPrem] + iAdd) >> InvquantiserRightShift;
      paRecoValue[i][uiYIdxRaster] = Pel(ClipBD<Int>(paRecoValue[i][uiYIdxRaster], g_bitDepth[1]));
    }
  }

  return false; //do not use pixel prediction
}

Bool TComPrediction:: CalcPixelPred(TComDataCU* pcCU, Pel* pOrg [3], Pel *pPalette[3], Pel* pValue, Pel * pEscapeFlag,  Pel*paPixelValue[3], Pel * paRecoValue[3], UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiIdx )
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
#if PLT_IDX_ADAPT_SCAN
  UInt uiScanIdx = uiY * uiStrideOrg + uiX;
  UInt uiYIdxRaster = pcCU->getPLTScanRotationModeFlag(0)? (uiX * uiStrideOrg + uiY) : (uiY * uiStrideOrg + uiX);
#else
  UInt uiYIdxRaster = uiY * uiStrideOrg + uiX;
#endif
  UInt uiMaxBit[3];
  pcCU->xCalcMaxBits(pcCU, uiMaxBit);
  if (bLossless)
  {
    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch ++)
    {
#if PLT_IDX_ADAPT_SCAN
      paPixelValue[ch][uiScanIdx] =  pOrg[ch][uiYIdxRaster];
#else
      paPixelValue[ch][uiYIdxRaster] =  pOrg[ch][uiYIdxRaster];
#endif
      paRecoValue[ch][uiYIdxRaster] = pOrg[ch][uiYIdxRaster];
    }
  }
  else
  {
    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch ++)
    {
#if PLT_IDX_ADAPT_SCAN
      paPixelValue[ch][uiScanIdx] = Pel(ClipBD<Int>((pOrg[ch][uiYIdxRaster] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch], uiMaxBit[ch]));
      paRecoValue[ch][uiYIdxRaster] = (paPixelValue[ch] [uiScanIdx]*g_invQuantScales[iQPrem[ch]] + iAdd[ch])>>InvquantiserRightShift[ch];
#else
      paPixelValue[ch][uiYIdxRaster] = Pel(ClipBD<Int>((pOrg[ch][uiYIdxRaster] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch], uiMaxBit[ch]));
      paRecoValue[ch][uiYIdxRaster] = (paPixelValue[ch] [uiYIdxRaster]*g_invQuantScales[iQPrem[ch]] + iAdd[ch])>>InvquantiserRightShift[ch];
#endif      
      paRecoValue[ch][uiYIdxRaster] = Pel(ClipBD<Int>(paRecoValue[ch][uiYIdxRaster], g_bitDepth[ch? 1:0]));
    }
  }

  return false; //do not use pixel prediction
}

Bool TComPrediction::CalRun(Pel* pValue, Pel * pEscapeFlag, UInt uiStartPos, UInt uiTotal, UInt &uiRun, Bool bChroma)
{
  UInt uiIdx = uiStartPos;
  Pel *pcIndexBlock = bChroma ? m_cIndexBlockChroma : m_cIndexBlock;
  while (uiIdx < uiTotal)
  {
    UInt uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan) 
    pValue[uiTraIdx] = pcIndexBlock[uiTraIdx] < 0 ? pcIndexBlock[uiTraIdx] + MAX_PLT_SIZE : pcIndexBlock[uiTraIdx];
    Bool bMismatch = (pcIndexBlock[uiTraIdx] < 0);
    if (bMismatch && uiIdx == uiStartPos)
    {
      return false;
    }
    if ((uiIdx && uiIdx > uiStartPos && pEscapeFlag[getIdxScanPos(uiIdx - 1)] == 0 && pValue[uiTraIdx] == pValue[getIdxScanPos(uiIdx - 1)]) && bMismatch == 0)
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

Bool  TComPrediction::CalCopy(Pel* pValue, Pel *pEscapeFlag,UInt uiWidth, UInt uiStartPos, UInt uiTotal, UInt &uiRun, Bool bChroma)
{
  UInt uiIdx = uiStartPos;
  UInt uiY = 0;
  Bool valid = false;
  Pel *pcIndexBlock = bChroma ? m_cIndexBlockChroma : m_cIndexBlock;
  UInt uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

  while (uiIdx < uiTotal)
  {
    uiY = uiTraIdx / uiWidth;
    if (uiY == 0)
    {
      return false;
    }
    UInt uiStride = uiWidth;
    uiTraIdx = getIdxScanPos(uiIdx);  //unified position variable (raster scan)

    pValue[uiTraIdx] = pcIndexBlock[uiTraIdx] < 0 ? pcIndexBlock[uiTraIdx] + MAX_PLT_SIZE : pcIndexBlock[uiTraIdx];
    Bool bMismatch = (pcIndexBlock[uiTraIdx] < 0);
    if (uiRun == 0 && bMismatch)
    {
      return false;
    }
    if ((pValue[uiTraIdx] == pValue[uiTraIdx - uiStride]) && (pEscapeFlag[uiTraIdx - uiStride] == 0) && (bMismatch == 0))
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

inline UInt TComPrediction::getIdxScanPos( UInt uiIdx )
{
#if PLT_IDX_ADAPT_SCAN
  return m_puiScanOrder[uiIdx];
#else
  return uiIdx;
#endif
}

#if PLT_IDX_ADAPT_SCAN
Void  TComPrediction::rotationScan( Pel* pLevel, UInt uiWidth, UInt uiHeight, Bool isInverse )
{
  Pel tmpLevel;
  UInt uiPos = 0;
  UInt* puiScanOrder = g_scanOrder[SCAN_UNGROUPED][SCAN_VER][g_aucConvertToBit[uiWidth]+2][g_aucConvertToBit[uiHeight]+2];

  for (UInt j=1; j<uiHeight; j++)
  {
    uiPos += j;
    for (UInt i=j; i<uiWidth; i++)
    {
      tmpLevel = pLevel[uiPos];
      pLevel[uiPos] = pLevel[puiScanOrder[uiPos]];
      pLevel[puiScanOrder[uiPos]] = tmpLevel;
      uiPos++;
    }
  }
}
#endif

#endif

//! \}
