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

#ifndef __TCOMCHROMAFORMAT__
#define __TCOMCHROMAFORMAT__

#include "CommonDef.h"
#include "TComRectangle.h"
#include "ContextTables.h"
#include "TComRom.h"
#include <iostream>
#include <vector>
#include <assert.h>
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "Debug.h"
#endif

//======================================================================================================================
//Chroma format utility functions  =====================================================================================
//======================================================================================================================

class TComDataCU;


static inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
static inline Bool        isLuma                    (const ComponentID id)                         { return (id==COMPONENT_Y);                                          }
static inline Bool        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
static inline Bool        isChroma                  (const ComponentID id)                         { return (id!=COMPONENT_Y);                                          }
static inline Bool        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);                                    }
static inline UInt        getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;                  }
static inline UInt        getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1;                  }
static inline UInt        getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);               }
static inline UInt        getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);               }
static inline UInt        getNumberValidChannelTypes(const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }
static inline UInt        getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
static inline Bool        isChromaEnabled           (const ChromaFormat fmt)                       { return  fmt!=CHROMA_400;                                           }


//------------------------------------------------

static inline UInt getTotalSamples(const UInt width, const UInt height, const ChromaFormat format)
{
  const UInt samplesPerChannel = width * height;

  switch (format)
  {
    case CHROMA_400: return  samplesPerChannel;           break;
    case CHROMA_420: return (samplesPerChannel * 3) >> 1; break;
    case CHROMA_422: return  samplesPerChannel * 2;       break;
    case CHROMA_444: return  samplesPerChannel * 3;       break;
    default:
      std::cerr << "ERROR: Unrecognised chroma format in getTotalSamples()" << std::endl;
      exit(1);
      break;
  }

  return MAX_UINT;
}


//------------------------------------------------

static inline Bool allFormatsUse420TUTreeStructure()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::AllChromaFormatsUseSameTUStructureAs420.getInt() != 0);
#elif (ECF__ALL_CHROMA_FORMATS_USE_SAME_TU_STRUCTURE_AS_420 == 1)
  return true;
#elif (ECF__ALL_CHROMA_FORMATS_USE_SAME_TU_STRUCTURE_AS_420 == 0)
  return false;
#endif
}


//------------------------------------------------

//returns true if it is possible for chroma to split down to the same level as luma
//(i.e. it will never have to "step-up" such that a chroma TU can cover multiple luma TUs)

static inline Bool chromaTUCanSplitDownToMinimumLumaSize(const ChromaFormat chFmt)
{
  //     (      this part checks if chroma can split for 4x4 luma      )    (  this checks if min luma size is 4x4  )
  return ((chFmt == CHROMA_444) && (!allFormatsUse420TUTreeStructure())) || ((g_uiMaxCUWidth >> g_uiMaxCUDepth) >= 8);
}


//------------------------------------------------

// In HM, a CU only has one chroma intra prediction direction, that corresponds to the top left luma intra prediction
// even if the NxN PU split occurs when 4 sub-TUs exist for chroma.
// Use this function to allow NxN PU splitting for chroma.

static inline Bool enable4ChromaPUsInIntraNxNCU(const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  switch (ToolOptionList::IntraNxNCUChromaPUSplitMode.getInt())
  {
    case 2: return                          chromaTUCanSplitDownToMinimumLumaSize(chFmt); break;
    case 1: return (chFmt == CHROMA_444) && chromaTUCanSplitDownToMinimumLumaSize(chFmt); break;
    default: break;
  }
  return false;
#elif (ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE == 2)
  return chromaTUCanSplitDownToMinimumLumaSize(chFmt);
#elif (ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE == 1)
  return (chFmt == CHROMA_444) && chromaTUCanSplitDownToMinimumLumaSize(chFmt);
#elif (ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool doubleHeightCoefficientGroups(const ComponentID compID, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return isChroma(compID) && (chFmt == CHROMA_422) && (ToolOptionList::DoubleHeightCoefficientGroups422.getInt() != 0);
#elif (ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422 == 1)
  return isChroma(compID) && (chFmt == CHROMA_422);
#elif (ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422 == 0)
  return false;
#endif
}


//------------------------------------------------

//returns the part index of the luma region that is co-located with the specified chroma region

static inline UInt getChromasCorrespondingPULumaIdx(const UInt lumaLCUIdx, const ChromaFormat chFmt)
{
#ifdef ECF__CHROMA_NxN_PU_CAN_HAVE_4_PARTS
  return enable4ChromaPUsInIntraNxNCU(chFmt) ? lumaLCUIdx : lumaLCUIdx & (~((1<<(2*g_uiAddCUDepth))-1)); //(lumaLCUIdx/numParts)*numParts;
#else
  return lumaLCUIdx & (~((1<<(2*g_uiAddCUDepth))-1));                                            //(lumaLCUIdx/numParts)*numParts;
#endif
}


//======================================================================================================================
//Intra prediction  ====================================================================================================
//======================================================================================================================

static inline Bool reducedIntraChromaModes()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::ReducedChromaIntraModeSet.getInt() != 0);
#elif (ECF__REDUCED_CHROMA_INTRA_MODE_SET == 1)
  return true;
#elif (ECF__REDUCED_CHROMA_INTRA_MODE_SET == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool combinedLumaChromaIntraSearch()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::CombinedLumaChromaIntraModeSearch.getInt() != 0);
#elif (ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH == 1)
  return true;
#elif (ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool initalDMChromaPreEst()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::EncoderInitialIntraModePreEstDMChroma.getInt() != 0);
#elif (ECF__ENCODER_INITIAL_INTRA_MODE_PREEST_DMCHROMA == 1)
  return true;
#elif (ECF__ENCODER_INITIAL_INTRA_MODE_PREEST_DMCHROMA == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool fastSearchOverAllComponents()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::EncoderFastIntraModeSearchOverAllComponents.getInt() != 0);
#elif (ECF__ENCODER_FAST_INTRA_MODE_SEARCH_OVER_ALL_COMPONENTS == 1)
  return true;
#elif (ECF__ENCODER_FAST_INTRA_MODE_SEARCH_OVER_ALL_COMPONENTS == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool fullRDSearchOverAllComponents()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::EncoderFullRateDistortionSearchOverAllComponents.getInt() != 0);
#elif (ECF__ENCODER_FULL_RATE_DISTORTION_SEARCH_OVER_ALL_COMPONENTS == 1)
  return true;
#elif (ECF__ENCODER_FULL_RATE_DISTORTION_SEARCH_OVER_ALL_COMPONENTS == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool additionalTrialEncodeChromaIntraSearch()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::AdditionalTrialEncodeChromaIntraModeSearch.getInt() != 0);
#elif (ECF__ADDITIONAL_TRIAL_ENCODE_CHROMA_INTRA_MODE_SEARCH == 1)
  return true;
#elif (ECF__ADDITIONAL_TRIAL_ENCODE_CHROMA_INTRA_MODE_SEARCH == 0)
  return false;
#endif
}


//------------------------------------------------

UInt* getCombinedSearchChromaModeList(TComDataCU *pcCU, const UInt uiAbsPartIdx, const Bool bLumaOnly, UInt searchArray[NUM_CHROMA_MODE+3]);

//------------------------------------------------

static inline Bool filterIntraReferenceSamples (const ChannelType chType, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return isLuma(chType) || (chFmt!=CHROMA_420 && ToolOptionList::ChromaIntraReferenceSampleFiltering.getInt()!=0);
#elif (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 1) || (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 2)
  return isLuma(chType) || (chFmt != CHROMA_420);
#elif (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 0)
  return isLuma(chType);
#endif
}


//------------------------------------------------

// this is modulated by the above filterIntraReferenceSamples
static inline Bool applyFilteredIntraReferenceSamples(const ChannelType chType, const ChromaFormat chFmt, const Int where) /*0=DC, 1=VERTICAL, 2=HORIZONTAL*/
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return isLuma(chType) || chFmt==CHROMA_444 || (chFmt==CHROMA_422 && (where==1 || ToolOptionList::ChromaIntraReferenceSampleFiltering.getInt()==2));
#elif (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 2)
  return isLuma(chType) || (chFmt != CHROMA_420);
#elif (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 1)
  return isLuma(chType) || chFmt==CHROMA_444 || (chFmt==CHROMA_422 && (where==1));
#elif (ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING == 0)
  return isLuma(chType);
#endif
}


//------------------------------------------------

static inline Bool nonScaledIntraChroma422(const ChannelType chType, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (isChroma(chType) && (chFmt == CHROMA_422) && (ToolOptionList::Chroma422IntraAngleScaling.getInt() == 0));
#elif (ECF__CHROMA_422_INTRA_ANGLE_SCALING == 1)
  return false;
#elif (ECF__CHROMA_422_INTRA_ANGLE_SCALING == 0)
  return (isChroma(chType) && (chFmt == CHROMA_422));
#endif
}


//------------------------------------------------

static inline Bool doubleWeightIntraDCAboveSamples(const ChannelType chType, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (isChroma(chType) && (chFmt == CHROMA_422) && (ToolOptionList::Chroma422IntraDCDoubleWeightAboveSamples.getInt() != 0));
#elif (ECF__CHROMA_422_INTRA_DC_DOUBLE_WEIGHT_ABOVE_SAMPLES == 1)
  return (isChroma(chType) && (chFmt == CHROMA_422));
#elif (ECF__CHROMA_422_INTRA_DC_DOUBLE_WEIGHT_ABOVE_SAMPLES == 0)
  return false;
#endif
}


//------------------------------------------------

static inline Bool intraPlanarSingleStageCalculation(const ChannelType chType, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (isChroma(chType) && (chFmt == CHROMA_422) && (ToolOptionList::Chroma422IntraPlanarSingleStageCalculation.getInt() != 0));
#elif (ECF__CHROMA_422_INTRA_PLANAR_SINGLE_STAGE_CALCULATION == 1)
  return (isChroma(chType) && (chFmt == CHROMA_422));
#elif (ECF__CHROMA_422_INTRA_PLANAR_SINGLE_STAGE_CALCULATION == 0)
  return false;
#endif
}


//------------------------------------------------

static inline FilterMode getIntraEdgeFilterMode(const ChannelType type, const ChromaFormat fmt)
{
  if (isLuma(type)) return FILTER_BOTH_DIRECTIONS;

  switch (fmt)
  {
    case CHROMA_444:
      {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        if (ToolOptionList::SetIntraChromaEdgeFilter444.getInt() != 0) return FILTER_BOTH_DIRECTIONS;
#elif (ECF__SET_INTRA_CHROMA_EDGE_FILTER_444 == 1)
        return FILTER_BOTH_DIRECTIONS;
#endif //ECF__SET_INTRA_CHROMA_EDGE_FILTER_444 == 0 falls through to return FILTER_DISABLED
      }
      break;
      
    case CHROMA_422:
      {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        switch (ToolOptionList::SetIntraChromaEdgeFilter422.getInt())
        {
          case 2:  return FILTER_BOTH_DIRECTIONS; break;
          case 1:  return FILTER_VERTICAL_ONLY;   break;
          default: break;
        }
#elif (ECF__SET_INTRA_CHROMA_EDGE_FILTER_422 == 2)
        return FILTER_BOTH_DIRECTIONS;
#elif (ECF__SET_INTRA_CHROMA_EDGE_FILTER_422 == 1)
        return FILTER_VERTICAL_ONLY;
#endif //ECF__SET_INTRA_CHROMA_EDGE_FILTER_422 == 0 falls through to return FILTER_DISABLED
      }
      break;

    default: break;
  }

  return FILTER_DISABLED;
}


//------------------------------------------------

static inline FilterMode getIntraDCFilterMode(const ChannelType type, const ChromaFormat fmt)
{
  if (isLuma(type)) return FILTER_BOTH_DIRECTIONS;

  switch (fmt)
  {
    case CHROMA_444:
      {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        if (ToolOptionList::SetIntraChromaDCFilter444.getInt() != 0) return FILTER_BOTH_DIRECTIONS;
#elif (ECF__SET_INTRA_CHROMA_DC_FILTER_444 == 1)
        return FILTER_BOTH_DIRECTIONS;
#endif //ECF__SET_INTRA_CHROMA_DC_FILTER_444 == 0 falls through to return FILTER_DISABLED
      }
      break;
      
    case CHROMA_422:
      {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        switch (ToolOptionList::SetIntraChromaDCFilter422.getInt())
        {
          case 2:  return FILTER_BOTH_DIRECTIONS; break;
          case 1:  return FILTER_VERTICAL_ONLY;   break;
          default: break;
        }
#elif (ECF__SET_INTRA_CHROMA_DC_FILTER_422 == 2)
        return FILTER_BOTH_DIRECTIONS;
#elif (ECF__SET_INTRA_CHROMA_DC_FILTER_422 == 1)
        return FILTER_VERTICAL_ONLY;
#endif //ECF__SET_INTRA_CHROMA_DC_FILTER_422 == 0 falls through to return FILTER_DISABLED
      }
      break;

    default: break;
  }

  return FILTER_DISABLED;
}


//------------------------------------------------

static inline Bool getLMChromaSamplesFrom2ndLeftCol(const Bool LMmode, const ChromaFormat chFmt)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return LMmode && (ToolOptionList::Get444LMChromaReferenceSamplesFrom1stColumn.getInt()==0 || chFmt!=CHROMA_444);
#elif (ECF__GET_444_LMCHROMA_REFERENCE_SAMPLES_FROM_1ST_COLUMN == 1)
  return LMmode && (chFmt != CHROMA_444);
#elif (ECF__GET_444_LMCHROMA_REFERENCE_SAMPLES_FROM_1ST_COLUMN == 0)
  return LMmode;
#endif
}


//======================================================================================================================
//Inter prediction  ====================================================================================================
//======================================================================================================================

static inline Bool useLumaInterpFilter(const ComponentID compID, const ChromaFormat fmt, const UInt dir /*0=H, 1=V*/)
{
  if (isLuma(compID)) return true;

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST

  return (ToolOptionList::UseLumaFilterForChromaQuarterSampleInterpolation.getInt() != 0) && ((fmt == CHROMA_444) || ((fmt == CHROMA_422) && (dir == 1))); //quarter-sample chrominance

#elif (ECF__USE_LUMA_FILTER_FOR_CHROMA_QUARTER_SAMPLE_INTERPOLATION == 1)
  return (fmt == CHROMA_444) || ((fmt == CHROMA_422) && (dir == 1)); //quarter-sample chrominance
#elif (ECF__USE_LUMA_FILTER_FOR_CHROMA_QUARTER_SAMPLE_INTERPOLATION == 0)
  return false;
#endif
}


//======================================================================================================================
//Transform and Quantisation  ==========================================================================================
//======================================================================================================================

Int getMDDTmode (const ComponentID compID, class TComDataCU* pcCU, const UInt uiAbsPartIdx); //in TComChromaFormat.cpp


//------------------------------------------------

static inline Bool TUCompRectHasAssociatedTransformSkipFlag(const TComRectangle &rectSamples)
{
#if !INTER_TRANSFORMSKIP
  return (rectSamples.width <= MAX_TS_WIDTH); // NOTE ECF - with intra-only TS, only width is checked. Allows 4x8 (for 4:2:2) and 4x4 only.
#else
#if REMOVE_NSQT
  return (rectSamples.width <= MAX_TS_WIDTH); // NOTE ECF - with NSQT disabled, only width is checked. Allows 4x8 (for 4:2:2) and 4x4 only.
#else
  return (rectSamples.width <= MAX_TS_WIDTH && rectSamples.height<=MAX_TS_HEIGHT); // NOTE ECF - allow 4x8 (for 4:2:2) and 4x4 only
#endif
#endif
}


//------------------------------------------------

static inline Bool singleTransformSkipFlag(const ChromaFormat format)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return ((format == CHROMA_444) && (ToolOptionList::SingleTransformSkipFlagForAllChannels444.getInt() != 0));
#elif (ECF__SINGLE_TRANSFORM_SKIP_FLAG_FOR_ALL_CHANNELS_444 == 1)
  return (format == CHROMA_444);
#elif (ECF__SINGLE_TRANSFORM_SKIP_FLAG_FOR_ALL_CHANNELS_444 == 0)
  return false;
#endif
}


//------------------------------------------------

Void setQPforQuant(       class QpParam      &result,
                    const       Int           qpy,
                    const       ChannelType   chType,
                    const       Int           qpBdOffset,
                    const       Int           chromaQPOffset,
                    const       ChromaFormat  chFmt,
                    const       Bool          useTransformSkip );


//------------------------------------------------

//when using the table-based -3 method, the arrays are defined with indices 0->8 to represent tables with indices -3->5
//this offset is needed to allow tables to be selected correctly

static inline Int getQpRemTableIndexOffset()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return ((ToolOptionList::Chroma422QuantiserAdjustment.getInt() == 2) && (ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt() == 2)) ? 3 : 0;
#elif ((ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2) && (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2))
  return 3;
#else
  return 0;
#endif
}


//------------------------------------------------

static inline Int getQuantScaling(const Int qp_rem, const ChromaFormat format)
{
  const Int tableIndex = qp_rem + getQpRemTableIndexOffset();

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if ((ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt() == 2) && (format == CHROMA_422))
  {
    switch (ToolOptionList::Chroma422QuantiserAdjustment.getInt())
    {
      case 1:  return g_quantScalesInc[tableIndex]; break; //+3 method
      case 2:  return g_quantScalesDec[tableIndex]; break; //-3 method
      default: break;
    }
  }
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2)
#if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1)
  if (format == CHROMA_422) return g_quantScalesInc[tableIndex];
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2)
  if (format == CHROMA_422) return g_quantScalesDec[tableIndex];
#endif
#endif

  return g_quantScales[tableIndex];
}


//------------------------------------------------

static inline Int getInverseQuantScaling(const Int qp_rem, const ChromaFormat format)
{
  const Int tableIndex = qp_rem + getQpRemTableIndexOffset();

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if ((ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt() == 2) && (format == CHROMA_422))
  {
    switch (ToolOptionList::Chroma422QuantiserAdjustment.getInt())
    {
      case 1:  return g_invQuantScalesInc[tableIndex]; break; //+3 method
      case 2:  return g_invQuantScalesDec[tableIndex]; break; //-3 method
      default: break;
    }
  }
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2)
#if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1)
  if (format == CHROMA_422) return g_invQuantScalesInc[tableIndex];
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2)
  if (format == CHROMA_422) return g_invQuantScalesDec[tableIndex];
#endif
#endif

  return g_invQuantScales[tableIndex];
}


//------------------------------------------------

static inline ErrorScaleAdjustmentMode getErrorScaleAdjustmentMode(const ComponentID compID, const ChromaFormat chFmt)
{
  return (isChroma(compID) && (chFmt == CHROMA_422)) ? ERROR_SCALE_ADJUSTMENT_MODE_422 : ERROR_SCALE_ADJUSTMENT_MODE_NONE;
}


//------------------------------------------------

static inline Bool roundTransformShiftUp(const ComponentID compID, const ChromaFormat chFmt, const Bool useTransformSkip)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (isChroma(compID) && (chFmt == CHROMA_422) && (!useTransformSkip) && (ToolOptionList::Chroma422QuantiserAdjustment.getInt() == 2));
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2)
  return (isChroma(compID) && (chFmt == CHROMA_422) && (!useTransformSkip));
#elif ((ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1) || (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 0))
  return false;
#endif
}


//------------------------------------------------

// NOTE: ECF - Represents scaling through forward transform, although this is not exact for 422 with TransformSkip enabled.
static inline Int getTransformShift(const UInt uiLog2TrSize)
{
#if FULL_NBIT
  return MAX_TR_DYNAMIC_RANGE - g_uiBitDepth - uiLog2TrSize;
#else
  return MAX_TR_DYNAMIC_RANGE - (g_uiBitDepth + g_uiBitIncrement) - uiLog2TrSize;
#endif
}


//------------------------------------------------

static inline Int getScaledChromaQP(Int unscaledChromaQP, const ChromaFormat chFmt)
{
  const UInt scalingTableIndex = Clip3(0, (chromaQPMappingTableSize - 1), unscaledChromaQP);

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST

  if (ToolOptionList::AdditionalChromaQpMappingTables.getInt() != 0)
  {
    if      (chFmt==CHROMA_422) return g_aucChromaScale422[ scalingTableIndex ];
    else if (chFmt==CHROMA_444) return g_aucChromaScale444[ scalingTableIndex ];
  }
  return g_aucChromaScale[ scalingTableIndex ];

#elif (ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES == 1)

  if      (chFmt==CHROMA_422) return g_aucChromaScale422[ scalingTableIndex ];
  else if (chFmt==CHROMA_444) return g_aucChromaScale444[ scalingTableIndex ];
  else                        return g_aucChromaScale   [ scalingTableIndex ];

#elif (ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES == 0)

  return g_aucChromaScale[ scalingTableIndex ];

#endif
}


//------------------------------------------------

static inline Void getAdditionalQuantiserMultiplyAndShift(Int &multiplier, Int &shift, const ComponentID compID, const ChromaFormat chFmt, const Bool useTransformSkip)
{
  multiplier = 1;
  shift      = 0;

  if (isChroma(compID) && (chFmt == CHROMA_422) && !useTransformSkip)
  {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
    if (ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt() == 0)
    {
      switch (ToolOptionList::Chroma422QuantiserAdjustment.getInt())
      {
        case 1: multiplier = INVSQRT2; shift = INVSQRT2_SHIFT; break; // / sqrt(2) method
        case 2: multiplier = SQRT2;    shift = SQRT2_SHIFT;    break; // * sqrt(2) method
        default: break;
      }
    }
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 0) //if using constant multiplication method
#if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1) // / sqrt(2) method
    multiplier = INVSQRT2;
    shift      = INVSQRT2_SHIFT;
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2) // * sqrt(2) method
    multiplier = SQRT2;
    shift      = SQRT2_SHIFT;
#endif
#endif
  }
}


//------------------------------------------------

static inline Void getAdditionalInverseQuantiserMultiplyAndShift(Int &multiplier, Int &shift, const ComponentID compID, const ChromaFormat chFmt, const Bool useTransformSkip)
{
  multiplier = 1;
  shift      = 0;

  if (isChroma(compID) && (chFmt == CHROMA_422) && !useTransformSkip)
  {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
    if (ToolOptionList::Chroma422QuantiserAdjustmentMethod.getInt() == 0)
    {
      switch (ToolOptionList::Chroma422QuantiserAdjustment.getInt())
      {
        case 1: multiplier = SQRT2;    shift = SQRT2_SHIFT;    break; // / sqrt(2) method
        case 2: multiplier = INVSQRT2; shift = INVSQRT2_SHIFT; break; // * sqrt(2) method
        default: break;
      }
    }
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 0) //if using constant multiplication method
#if   (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 1) // / sqrt(2) method
    multiplier = SQRT2;
    shift      = SQRT2_SHIFT;
#elif (ECF__CHROMA_422_QUANTISER_ADJUSTMENT == 2) // * sqrt(2) method
    multiplier = INVSQRT2;
    shift      = INVSQRT2_SHIFT;
#endif
#endif
  }
}


//======================================================================================================================
//Scaling lists  =======================================================================================================
//======================================================================================================================

#if !REMOVE_NSQT
static inline ScalingListDIR getScalingListDIR(const UInt width, const UInt height, const ChromaFormat fmt, const ComponentID compID)
{
  const UInt csx=getComponentScaleX(compID, fmt);
  const UInt csy=getComponentScaleY(compID, fmt);
  if ((width<<csx) == (height<<csy))      return SCALING_LIST_SQT;
  else if ((width<<csx) < (height<<csy))  return SCALING_LIST_VER;
  else                                    return SCALING_LIST_HOR;
}
#endif

//------------------------------------------------

// for a given TU size, there is one list per intra/inter & channel combination (i.e. 6 per TU size), apart from 32x32,
// because 32x32 Cb/Cr TUs don't exist in 4:2:0
static inline Int getScalingListType(const Bool isIntra, const UInt log2TUSize, const ComponentID compID
#if !REMOVE_NSQT
    , const ScalingListDIR scalingList
#endif
                                     )
{
#if ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA
  return (isIntra ? 0 : MAX_NUM_COMPONENT) + compID;
#else
  const Int base=(isIntra ? 0 : MAX_NUM_COMPONENT);
#if REMOVE_NSQT
  const Int numForAdjSizeID=g_scalingListNum[log2TUSize-2];
#else
  const Int numForAdjSizeID=g_scalingListNum[log2TUSize-2+(scalingList!=SCALING_LIST_SQT?1:0)];
#endif
  return (numForAdjSizeID!=SCALING_LIST_NUM) ? base : base+compID;
#endif
}


//------------------------------------------------

static inline UInt getScalingListCoeffIdx(const ChromaFormat chFmt, const ComponentID compID, const UInt blkPos, const UInt tuWidth, const UInt tuHeight)
{
  if ( tuWidth==tuHeight || isLuma(compID) || chFmt!=CHROMA_422  )
    return blkPos;
  else if (tuWidth<tuHeight)
    return (blkPos << 1);
  else
    return ( (blkPos & (~(tuWidth-1)))<<1) + (blkPos & (tuWidth-1));
}


//======================================================================================================================
//TU scanning  =========================================================================================================
//======================================================================================================================

MDCSMode getMDCSMode(const UInt width, const UInt height, const ComponentID component, const ChromaFormat format);

//------------------------------------------------

static inline UInt getMDCSAngleLimit(const ComponentID component)
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return ((isLuma(component)) ? (ToolOptionList::LumaMDCSAngleLimit.getInt()) : (ToolOptionList::ChromaMDCSAngleLimit.getInt()));
#else
  return ((isLuma(component)) ? (ECF__LUMA_MDCS_ANGLE_LIMIT) : (ECF__CHROMA_MDCS_ANGLE_LIMIT));
#endif
}


//======================================================================================================================
//Context variable selection  ==========================================================================================
//======================================================================================================================

//context variable source tables

#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
static const UInt significanceMapContextStartTable[MAX_NUM_COMPONENT]    = {FIRST_SIG_FLAG_CTX_LUMA, FIRST_SIG_FLAG_CTX_CB, FIRST_SIG_FLAG_CTX_CR};
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
static const UInt significanceMapContextStartTable[MAX_NUM_CHANNEL_TYPE] = {FIRST_SIG_FLAG_CTX_LUMA, FIRST_SIG_FLAG_CTX_CHROMA};
#endif

#if   (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 2)
static const UInt contextSetStartTable[MAX_NUM_COMPONENT]    = {FIRST_CTX_SET_LUMA, FIRST_CTX_SET_CB, FIRST_CTX_SET_CR};
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 1)
static const UInt contextSetStartTable[MAX_NUM_CHANNEL_TYPE] = {FIRST_CTX_SET_LUMA, FIRST_CTX_SET_CHROMA};
#endif

#if   (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 2)
static const UInt CBFContextStartTable[MAX_NUM_COMPONENT]    = {FIRST_CBF_CTX_LUMA, FIRST_CBF_CTX_CB, FIRST_CBF_CTX_CR};
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 1)
static const UInt CBFContextStartTable[MAX_NUM_CHANNEL_TYPE] = {FIRST_CBF_CTX_LUMA, FIRST_CBF_CTX_CHROMA};
#endif


//------------------------------------------------

static inline Bool useTransformDepthForCbfCtxSelection(const ChromaFormat fmt, const ChannelType chType)
{
#if (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 0)
  return false;
#elif ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (isChroma(chType) && ((fmt != CHROMA_444) || (ToolOptionList::UseTransformDepthFor444ChromaCBFContextSelection.getInt() != 0)));
#elif (ECF__USE_TRANSFORM_DEPTH_FOR_444_CHROMA_CBF_CONTEXT_SELECTION == 1)
  return isChroma(chType);
#elif (ECF__USE_TRANSFORM_DEPTH_FOR_444_CHROMA_CBF_CONTEXT_SELECTION == 0)
  return (isChroma(chType) && (fmt != CHROMA_444));
#endif
}


//------------------------------------------------

//Function for last-significant-coefficient context selection parameters

static inline Void getLastSignificantContextParameters (const ComponentID  component,
                                                        const Int          width,
                                                        const Int          height,
                                                              Int         &result_offsetX,
                                                              Int         &result_offsetY,
                                                              Int         &result_shiftX,
                                                              Int         &result_shiftY)
{
  const UInt convertedWidth  = g_aucConvertToBit[width];
  const UInt convertedHeight = g_aucConvertToBit[height];

#ifdef ECF__CHROMA_LAST_POSITION_CONTEXT_SAME_AS_LUMA
  result_offsetX = ((convertedWidth  * 3) + ((convertedWidth  + 1) >> 2));
  result_offsetY = ((convertedHeight * 3) + ((convertedHeight + 1) >> 2));
  result_shiftX  = ((convertedWidth  + 3) >> 2);
  result_shiftY  = ((convertedHeight + 3) >> 2);
#else
  result_offsetX = (isChroma(component)) ? 0               : ((convertedWidth  * 3) + ((convertedWidth  + 1) >> 2));
  result_offsetY = (isChroma(component)) ? 0               : ((convertedHeight * 3) + ((convertedHeight + 1) >> 2));
  result_shiftX  = (isChroma(component)) ? convertedWidth  : ((convertedWidth  + 3) >> 2);
  result_shiftY  = (isChroma(component)) ? convertedHeight : ((convertedHeight + 3) >> 2);
#endif
}


//------------------------------------------------

//Function for significance map context index offset selection

static inline UInt getSignificanceMapContextOffset (const ComponentID component)
{
#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
  return significanceMapContextStartTable[component];
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
  return significanceMapContextStartTable[toChannelType(component)];
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)
  return FIRST_SIG_FLAG_CTX_LUMA;
#endif
}


//------------------------------------------------

//When deriving patternSigCtx for significance map context selection, if one neighbour
//group is available and the other is not, rather than assume 0 for the unavailable group,
//assume the same significance as the available group

static inline Bool patternSigCtxCopyMissingGroupFromAvailableGroup()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::PatternSigCtxMissingGroupsSameAsAvailableGroups.getInt() != 0);
#elif (ECF__PATTERNSIGCTX_MISSING_GROUPS_SAME_AS_AVAILABLE_GROUPS == 1)
  return true;
#elif (ECF__PATTERNSIGCTX_MISSING_GROUPS_SAME_AS_AVAILABLE_GROUPS == 0)
  return false;
#endif
}


//------------------------------------------------

//Enable fixed context variable mapping grid for 8x4, 4x8, 8x16 and 16x8

static inline Bool fixed422SignificanceMapContext()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::Chroma422SignificanceMapContextGrid.getInt() != 0);
#elif ((ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 1) || (ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 2))
  return true;
#elif (ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 0)
  return false;
#endif
}


//------------------------------------------------

//Enable fixed context variable mapping grid for 8x4, 4x8, 8x16 and 16x8

static inline Bool separateDC422SignificanceMapContext()
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  return (ToolOptionList::Chroma422SignificanceMapContextGrid.getInt() > 1);
#elif (ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 2)
  return true;
#elif ((ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 1) || (ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID == 0))
  return false;
#endif
}


//------------------------------------------------

// Function for greater-than-one map/greater-than-two map context set selection

static inline UInt getContextSetIndex (const ComponentID  component,
                                       const UInt         subsetIndex,
                                       const Bool         foundACoefficientGreaterThan1)
{
  //------------------

#if   (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 2)
  UInt contextSetIndex = contextSetStartTable[component];
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 1)
  UInt contextSetIndex = contextSetStartTable[toChannelType(component)];
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 0)
  UInt contextSetIndex = FIRST_CTX_SET_LUMA;
#endif

  //------------------

#ifdef ECF__CHROMA_C1_C2_CONTEXT_SAME_AS_LUMA
  if                       (subsetIndex > 0)  contextSetIndex += 2;
#else
  if (isLuma(component) && (subsetIndex > 0)) contextSetIndex += 2;
#endif

  //------------------

  return ((foundACoefficientGreaterThan1) ? (contextSetIndex + 1) : (contextSetIndex));
}


//------------------------------------------------

//Function for CBF context index offset

static inline UInt getCBFContextOffset (const ComponentID component)
{
#if   (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 2)
  return CBFContextStartTable[component];
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 1)
  return CBFContextStartTable[toChannelType(component)];
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 0)
  return FIRST_CBF_CTX_LUMA;
#endif
}


//======================================================================================================================
//Entropy coding parameters ============================================================================================
//======================================================================================================================

Void getTUEntropyCodingParameters(      TUEntropyCodingParameters &result,
                                  const UInt                       uiScanIdx,
                                  const UInt                       width,
                                  const UInt                       height,
                                  const ComponentID                component,
                                  const ChromaFormat               format);


//======================================================================================================================
//End  =================================================================================================================
//======================================================================================================================

#endif
