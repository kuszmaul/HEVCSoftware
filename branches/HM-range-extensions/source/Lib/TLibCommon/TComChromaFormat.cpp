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


#include "TComChromaFormat.h"
#include "TComPic.h"
#include "TComDataCU.h"
#include "TComTrQuant.h"

//----------------------------------------------------------------------------------------------------------------------

static UInt emptySearch[]={INVALID_MODE_IDX, STOPCHROMASEARCH_MODE_IDX };
#if !REMOVE_LMCHROMA
static UInt dm_lm_search[]={DM_CHROMA_IDX, LM_CHROMA_IDX, STOPCHROMASEARCH_MODE_IDX };
#endif

UInt *getCombinedSearchChromaModeList(TComDataCU *pcCU, const UInt uiAbsPartIdx, const Bool bLumaOnly, UInt searchArray[NUM_CHROMA_MODE+3])
{
  if (fullRDSearchOverAllComponents() && !bLumaOnly && (getChromasCorrespondingPULumaIdx(uiAbsPartIdx, pcCU->getPic()->getChromaFormat()) == uiAbsPartIdx))
  {
    if (reducedIntraChromaModes())
    {
#if REMOVE_LMCHROMA
      return emptySearch;
#else
      if (pcCU->getSlice()->getSPS()->getUseLMChroma()) return dm_lm_search;
      else                                              return emptySearch;
#endif
    }
    else
    {
      pcCU->getAllowedChromaDir(uiAbsPartIdx, searchArray);
      searchArray[NUM_CHROMA_MODE] = STOPCHROMASEARCH_MODE_IDX;
      return searchArray;
    }
  }
  else
  {
    return emptySearch;
  }
}


//----------------------------------------------------------------------------------------------------------------------

Int getMDDTmode(const ComponentID compID, class TComDataCU* pcCU, const UInt uiAbsPartIdx)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return ((compID==COMPONENT_Y || (pcCU->getPic()->getChromaFormat()==CHROMA_444 && ToolOptionList::EnableMDDTFor444Chroma.getInt()!=0))
          && pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA)
          ? pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx )
          : REG_DCT;
#elif (ECF__ENABLE_MDDT_FOR_444_CHROMA == 1)
  return ((compID==COMPONENT_Y || pcCU->getPic()->getChromaFormat()==CHROMA_444) && pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA)
          ? pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx )
          : REG_DCT;
#else
  return ((compID==COMPONENT_Y) && pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA)
          ? pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx )
          : REG_DCT;
#endif
}


//----------------------------------------------------------------------------------------------------------------------

MDCSMode getMDCSMode(const UInt width, const UInt height, const ComponentID component, const ChromaFormat format)
{
  //------------------

  //check that the TU is not too big

  UInt maximumWidth  = 0;
  UInt maximumHeight = 0;

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (ToolOptionList::NonSubsampledChromaUseLumaMDCSSizeLimits.getInt() != 0)
  {
    maximumWidth  = ((getComponentScaleX(component, format) == 0) ? (ToolOptionList::LumaMDCSMaximumWidth .getInt()) : (ToolOptionList::ChromaMDCSMaximumWidth .getInt()));
    maximumHeight = ((getComponentScaleY(component, format) == 0) ? (ToolOptionList::LumaMDCSMaximumHeight.getInt()) : (ToolOptionList::ChromaMDCSMaximumHeight.getInt()));
  }
  else
  {
    maximumWidth  = ((isLuma(component))                          ? (ToolOptionList::LumaMDCSMaximumWidth .getInt()) : (ToolOptionList::ChromaMDCSMaximumWidth .getInt()));
    maximumHeight = ((isLuma(component))                          ? (ToolOptionList::LumaMDCSMaximumHeight.getInt()) : (ToolOptionList::ChromaMDCSMaximumHeight.getInt()));
  }
#elif (ECF__NON_SUBSAMPLED_CHROMA_USE_LUMA_MDCS_SIZE_LIMITS == 1)
  maximumWidth  = ((getComponentScaleX(component, format) == 0) ? (ECF__LUMA_MDCS_MAXIMUM_WIDTH ) : (ECF__CHROMA_MDCS_MAXIMUM_WIDTH ));
  maximumHeight = ((getComponentScaleY(component, format) == 0) ? (ECF__LUMA_MDCS_MAXIMUM_HEIGHT) : (ECF__CHROMA_MDCS_MAXIMUM_HEIGHT));
#elif (ECF__NON_SUBSAMPLED_CHROMA_USE_LUMA_MDCS_SIZE_LIMITS == 0)
  maximumWidth  = ((isLuma(component))                          ? (ECF__LUMA_MDCS_MAXIMUM_WIDTH ) : (ECF__CHROMA_MDCS_MAXIMUM_WIDTH ));
  maximumHeight = ((isLuma(component))                          ? (ECF__LUMA_MDCS_MAXIMUM_HEIGHT) : (ECF__CHROMA_MDCS_MAXIMUM_HEIGHT));
#endif

  if ((width > maximumWidth) || (height > maximumHeight)) return MDCS_DISABLED;

  //------------------

  //return the appropriate mode setting

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  const UInt MDCSModeIndex = ((isLuma(component)) ? (ToolOptionList::LumaMDCSMode.getInt()) : (ToolOptionList::ChromaMDCSMode.getInt()));
#else
  const UInt MDCSModeIndex = ((isLuma(component)) ? (ECF__LUMA_MDCS_MODE                  ) : (ECF__CHROMA_MDCS_MODE                  ));
#endif

  switch (MDCSModeIndex)
  {
    case 3: return MDCS_BOTH_DIRECTIONS; break;
    case 2: return MDCS_VERTICAL_ONLY;   break;
    case 1: return MDCS_HORIZONTAL_ONLY; break;
    default: break;
  }

  //------------------

  return MDCS_DISABLED;
}


//----------------------------------------------------------------------------------------------------------------------

Void setQPforQuant(       QpParam      &result,
                    const Int           qpy,
                    const ChannelType   chType,
                    const Int           qpBdOffset,
                    const Int           chromaQPOffset,
                    const ChromaFormat  chFmt,
                    const Bool          useTransformSkip )
{
  Int baseQp      = MAX_INT;
  Int adjustedQp  = MAX_INT;
  Int qpRemOffset = 0;

  if(isLuma(chType))
  {
    baseQp     = qpy + qpBdOffset;
    adjustedQp = baseQp;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }

    adjustedQp = baseQp;

    //------------------------------------------------

    //adjustment for chroma 4:2:2
    
    if ((chFmt == CHROMA_422) && !useTransformSkip)
    {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      switch (ToolOptionList::Chroma422QuantiserAdjustment.getInt())
      {
        case 1: //+3 method
          switch (ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt())
          {
            case 1:                          adjustedQp  += 3; break; //Qp modification method
            case 2:                          qpRemOffset += 3; break; //Table method
            default: break;
          }
          break; 

        case 2: //-3 method
          switch (ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt())
          {
            case 1: assert(adjustedQp >= 3); adjustedQp  -= 3; break; //Qp modification method
            case 2:                          qpRemOffset -= 3; break; //Table method
            default: break;
          }
          break; 

        default: break;
      }
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1) //+3 method
  #if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 1) //Qp modification method
      adjustedQp  += 3;
  #elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2) //Table method
      qpRemOffset += 3;
  #endif
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2) //-3 method
  #if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 1) //Qp modification method
      adjustedQp  -= 3;
  #elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2) //Table method
      qpRemOffset -= 3;
  #endif
#endif
    }

    //------------------------------------------------
  }

  if ((adjustedQp == baseQp) && (qpRemOffset == 0))
  {
    result.setBothQps   (QpParam::QpData(baseQp,     (baseQp     / 6),  (baseQp     % 6)               ));
  }
  else
  {
    result.setBaseQp    (QpParam::QpData(baseQp,     (baseQp     / 6),  (baseQp     % 6)               ));
    result.setAdjustedQp(QpParam::QpData(adjustedQp, (adjustedQp / 6), ((adjustedQp % 6) + qpRemOffset)));
  }
}


//----------------------------------------------------------------------------------------------------------------------

Void getTUEntropyCodingParameters(      TUEntropyCodingParameters &result,
                                  const UInt                       uiScanIdx,
                                  const UInt                       width,
                                  const UInt                       height,
                                  const ComponentID                component,
                                  const ChromaFormat               format)
{
  //------------------------------------------------

  //set the local parameters

  const UInt        log2BlockWidth    = g_aucConvertToBit[width]  + 2;
  const UInt        log2BlockHeight   = g_aucConvertToBit[height] + 2;
  const Bool        doubleGroupHeight = doubleHeightCoefficientGroups(component, format);
  const UInt        log2GroupSize     = (doubleGroupHeight ? (MLS_CG_SIZE + 1) : MLS_CG_SIZE);
  const ChannelType channelType       = toChannelType(component);

  result.scanType = ((uiScanIdx == SCAN_ZIGZAG) ? SCAN_DIAG : COEFF_SCAN_TYPE(uiScanIdx));
  
  //------------------------------------------------

  //set the group layout

#if !REMOVAL_8x2_2x8_CG
  if( result.scanType == SCAN_HOR )  
  {
    result.log2GroupWidth  = std::min<UInt>(log2BlockWidth,   log2GroupSize);
    result.log2GroupHeight = std::min<UInt>(log2BlockHeight, (log2GroupSize - result.log2GroupWidth));
  }
  else if( result.scanType == SCAN_VER )
  {
    result.log2GroupHeight = std::min<UInt>(log2BlockHeight,  log2GroupSize);
    result.log2GroupWidth  = std::min<UInt>(log2BlockWidth,  (log2GroupSize - result.log2GroupHeight));
  }
  else
#endif
  {
    result.log2GroupWidth  = std::min<UInt>(log2BlockWidth,  ( log2GroupSize      >> 1));   //width rounds down and height rounds up so that, when using non-square
    result.log2GroupHeight = std::min<UInt>(log2BlockHeight, ((log2GroupSize + 1) >> 1));   //coefficient groups, the groups will be double-high rather than double-wide
  }

  result.widthInGroups  = width  >> result.log2GroupWidth;
  result.heightInGroups = height >> result.log2GroupHeight;

  //------------------------------------------------

  //set the scan orders

  const UInt log2WidthInGroups  = g_aucConvertToBit[result.widthInGroups  * 4];
  const UInt log2HeightInGroups = g_aucConvertToBit[result.heightInGroups * 4];

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
#if REMOVAL_8x2_2x8_CG
  const UInt groupType          = doubleGroupHeight ? SCAN_GROUPED_4x8 : SCAN_GROUPED_4x4;
#else
  const UInt groupType          = ((result.scanType == SCAN_HOR) || (result.scanType == SCAN_VER)) ? SCAN_UNGROUPED : (doubleGroupHeight ? SCAN_GROUPED_4x8 : SCAN_GROUPED_4x4);
#endif
#else
#if REMOVAL_8x2_2x8_CG
  const UInt groupType          = SCAN_GROUPED_4x4;
#else
  const UInt groupType          = ((result.scanType == SCAN_HOR) || (result.scanType == SCAN_VER)) ? SCAN_UNGROUPED : SCAN_GROUPED_4x4;
#endif
#endif

  result.scan   = g_scanOrder[ groupType      ][ result.scanType ][ log2BlockWidth    ][ log2BlockHeight    ];
  result.scanCG = g_scanOrder[ SCAN_UNGROUPED ][ result.scanType ][ log2WidthInGroups ][ log2HeightInGroups ];

  //------------------------------------------------

  //set the significance map context selection parameters

  if (((width == 4) && (height == 4)) || (fixed422SignificanceMapContext() && (((width == 4) && (height == 8)) || ((width == 8) && (height == 4)))))
  {
    result.firstSignificanceMapContext        = significanceMapContextSetStart[channelType][CONTEXT_TYPE_4x4];
    result.useFixedGridSignificanceMapContext = true;
    if ((width != height) && separateDC422SignificanceMapContext())
    {
      if      (height == 8) result.fixedGridContextParameters.initialise(ctxIndMap4x8, 2, 3, log2BlockWidth, log2BlockHeight);
      else if (width  == 8) result.fixedGridContextParameters.initialise(ctxIndMap8x4, 3, 2, log2BlockWidth, log2BlockHeight);
    }
    else result.fixedGridContextParameters.initialise(ctxIndMap4x4, 2, 2, log2BlockWidth, log2BlockHeight);
  }
  else if (((width == 8) && (height == 8)) || (fixed422SignificanceMapContext() && (((width == 8) && (height == 16)) || ((width == 16) && (height == 8)))))
  {
    result.firstSignificanceMapContext        = significanceMapContextSetStart[channelType][CONTEXT_TYPE_8x8];
#if REMOVAL_8x2_2x8_CG
    result.useFixedGridSignificanceMapContext = false;
    if (result.scanType != SCAN_DIAG) result.firstSignificanceMapContext += nonDiagonalScan8x8ContextOffset[channelType];
#else
    result.useFixedGridSignificanceMapContext = true;
    if ((width != height) && separateDC422SignificanceMapContext())
    {
      if      (height == 16) result.fixedGridContextParameters.initialise(ctxIndMap4x8, 2, 3, log2BlockWidth, log2BlockHeight);
      else if (width  == 16) result.fixedGridContextParameters.initialise(ctxIndMap8x4, 3, 2, log2BlockWidth, log2BlockHeight);
    }
    else result.fixedGridContextParameters.initialise(ctxIndMap4x4, 2, 2, log2BlockWidth, log2BlockHeight);
#endif
  }
  else
  {
    result.firstSignificanceMapContext        = significanceMapContextSetStart[channelType][CONTEXT_TYPE_NxN];
    result.useFixedGridSignificanceMapContext = false;
  }

  //------------------------------------------------

  //set the neighbourhood context thresholds

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
  if (!result.useFixedGridSignificanceMapContext)
  {
    //we must check all these conditions - doubleGroupHeight may not result in actual double-size groups
    //due to clipping and doubleGroupHeight is also not the only way to get non-square groups because of MDCS
    if (doubleGroupHeight && (result.log2GroupWidth != result.log2GroupHeight))
    {
#if REMOVAL_8x2_2x8_CG
      result.neighbourhoodContextParameters.pattern00Context1Threshold = NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x8;
      result.neighbourhoodContextParameters.pattern00Context2Threshold = NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x8;
#else
      result.neighbourhoodContextParameters.pattern00ContextThreshold = NEIGHBOURHOOD_00_CONTEXT_THRESHOLD_4x8;
      result.neighbourhoodContextParameters.pattern11ContextThreshold = NEIGHBOURHOOD_11_CONTEXT_THRESHOLD_4x8;
#endif
    }
    else
    {
#if REMOVAL_8x2_2x8_CG
      result.neighbourhoodContextParameters.pattern00Context1Threshold = NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4;
      result.neighbourhoodContextParameters.pattern00Context2Threshold = NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4;
#else
      result.neighbourhoodContextParameters.pattern00ContextThreshold = NEIGHBOURHOOD_00_CONTEXT_THRESHOLD_4x4;
      result.neighbourhoodContextParameters.pattern11ContextThreshold = NEIGHBOURHOOD_11_CONTEXT_THRESHOLD_4x4;
#endif
    }
  }
#endif

  //------------------------------------------------
}


//----------------------------------------------------------------------------------------------------------------------
