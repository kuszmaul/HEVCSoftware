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

/** \file     Debug.cpp
    \brief    Defines types and objects for environment-variable-based debugging and feature control
*/

#include "Debug.h"
#include <algorithm>
#include "TComDataCU.h"
#include "TComPic.h"
#include "TComYuv.h"

static const UInt settingNameWidth  = 61;
static const UInt settingHelpWidth  = 84;
static const UInt settingValueWidth = 3;

// --------------------------------------------------------------------------------------------------------------------- //

//EnvVar definition

std::list<std::pair<std::string, std::string> > &EnvVar::getEnvVarList()
{
  static std::list<std::pair<std::string, std::string> > varInfoList;
  return varInfoList;
}

std::list<EnvVar*> &EnvVar::getEnvVarInUse()
{
  static std::list<EnvVar*> varInUseList;
  return varInUseList;
}

static inline Void printPair(const std::pair<std::string, std::string> &p)
{
  if (p.second=="")
  {
    std::cout << "\n" << std::setw(settingNameWidth) << p.first << "\n" << std::endl;
  }
  else
  {
    std::cout << std::setw(settingNameWidth) << p.first << ":   " << p.second << "\n" << std::endl;
  }
}

static inline Void printVal(const EnvVar* env)
{
  std::cout << std::setw(settingNameWidth) << env->getName() << " = " << std::setw(settingValueWidth) << env->getInt() << " (string = " << std::setw(15) << env->getString() << ")" << std::endl;
}

//static inline bool sameEnvName( const std::pair<std::string, std::string> &a,
//                                const std::pair<std::string, std::string> &b )
//{
//  // only check env name
//  return (a.first==b.first);
//}

Void EnvVar::printEnvVar()
{
//  getEnvVarList().unique(sameEnvName);
  if (getEnvVarList().size()!=0)
  {
    std::cout << "--- Environment variables:\n" << std::endl;
    for_each(getEnvVarList().begin(), getEnvVarList().end(), printPair);
  }
  std::cout << std::endl;
}

Void EnvVar::printEnvVarInUse()
{
  if (getEnvVarInUse().size()!=0)
  {
    std::cout << "ECF Environment variables set as follows: \n" << std::endl;
    for_each(getEnvVarInUse().begin(), getEnvVarInUse().end(), printVal);
  }
  std::cout << std::endl;
}

EnvVar::EnvVar(const std::string &sName, const std::string &sDefault, const std::string &sHelp) :
                                                                m_sName(sName),
                                                                m_sHelp(sHelp),
                                                                m_sVal(),
                                                                m_dVal(0),
                                                                m_iVal(0),
                                                                m_bSet(false)
{
  if (getenv(m_sName.c_str()))
  {
    m_sVal = getenv(m_sName.c_str());
    m_bSet = true;
    getEnvVarInUse().push_back(this);
  }
  else m_sVal = sDefault;

  m_dVal = strtod(m_sVal.c_str(), 0);
  m_iVal = int(m_dVal);

  getEnvVarList().push_back( std::pair<std::string, std::string>(m_sName, indentNewLines(lineWrap(splitOnSettings(m_sHelp), settingHelpWidth), (settingNameWidth + 4))) );
}


// --------------------------------------------------------------------------------------------------------------------- //

// Debug environment variables:

EnvVar Debug("-- Debugging","","");

EnvVar DebugOptionList::DebugSBAC           ("DEBUG_SBAC",              "0", "Output debug data from SBAC entropy coder (coefficient data etc.)");
EnvVar DebugOptionList::DebugRQT            ("DEBUG_RQT",               "0", "Output RQT debug data from entropy coder"                         );
EnvVar DebugOptionList::DebugPred           ("DEBUG_PRED",              "0", "Output prediction debug"                                          );
EnvVar DebugOptionList::ForceLumaMode       ("FORCE_LUMA_MODE",         "0", "Force a particular intra direction for Luma (0-34)"               );
EnvVar DebugOptionList::ForceChromaMode     ("FORCE_CHROMA_MODE",       "0", "Force a particular intra direction for chroma (0-5)"              );
EnvVar DebugOptionList::CopyLumaToChroma444 ("COPY_LUMA_TO_CHROMA_444", "0", "If using 444, copy luma channel to both chroma channels"          );
EnvVar DebugOptionList::SwapCbCrOnLoading   ("SWAP_CB_CR_ON_LOADING",   "0", "Swaps Cb and Cr channels on loading"                              );


// Tool setting environment variables:

EnvVar Tools("--     Tools","","");

EnvVar ToolOptionList::AllChromaFormatsUseSameTUStructureAs420          ("ECF__ALL_CHROMA_FORMATS_USE_SAME_TU_STRUCTURE_AS_420",          "0", "0 (default) = Allow chroma TU tree to split down to the minimum possible TU size, 1 = Prevent chroma TU splitting wherever an equivalent 4:2:0 chroma TU could not split (e.g. prevent splitting of chroma TUs wherever luma splits down to 4x4)"                                                              );
EnvVar ToolOptionList::IntraNxNCUChromaPUSplitMode                      ("ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE",                        "0", "0 (default) = An intra-NxN-split CU always has only one chroma PU, 1 = In 4:4:4, an intra-NxN-split CU can have four chroma PUs (subject to limitations on minimum TU size etc.), 2 = As 1, but for any chroma format (not just 4:4:4)"                                                                        );
EnvVar ToolOptionList::DoubleHeightCoefficientGroups422                 ("ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422",                     "0", "0 (default) = use standard size square coefficient groups for all formats, 1 = use double-height groups for 4:2:2"                                                                                                                                                                                             );
EnvVar ToolOptionList::ReducedChromaIntraModeSet                        ("ECF__REDUCED_CHROMA_INTRA_MODE_SET",                            "0", "0 (default) = Allow chroma to select a different intra prediction mode to luma, 1 = Always use DM_Chroma or LM_Chroma (when enbled)"                                                                                                                                                                           );
EnvVar ToolOptionList::CombinedLumaChromaIntraModeSearch                ("ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH",                   "0", "0 (default) = When processing the intra prediction mode search that defines the TU tree, only take luma into account, 1 = Also take chroma into account"                                                                                                                                                       );
EnvVar ToolOptionList::EncoderInitialIntraModePreEstDMChroma            ("ECF__ENCODER_INITIAL_INTRA_MODE_PREEST_DMCHROMA",               "0", "[NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Use pre-est to estimate initial chroma intra prediction mode, 1 = Set initial chroma intra prediciton mode to DM_CHROMA"                                                                                                        );
EnvVar ToolOptionList::EncoderFastIntraModeSearchOverAllComponents      ("ECF__ENCODER_FAST_INTRA_MODE_SEARCH_OVER_ALL_COMPONENTS",       "0", "[NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Fast encoder intra mode search using luma only, 1 = Fast encoder intra mode search using all components"                                                                                                                        );
EnvVar ToolOptionList::EncoderFullRateDistortionSearchOverAllComponents ("ECF__ENCODER_FULL_RATE_DISTORTION_SEARCH_OVER_ALL_COMPONENTS",  "0", "[NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Full rate-distortion intra mode search using luma only, 1 = Full rate-distortion intra mode search also tests all allowed chroma intra modes"                                                                                   );
EnvVar ToolOptionList::AdditionalTrialEncodeChromaIntraModeSearch       ("ECF__ADDITIONAL_TRIAL_ENCODE_CHROMA_INTRA_MODE_SEARCH",         "1", "[NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 = When using combined luma & chroma intra search, skip the trial-encode to define the final chroma intra mode, 1 (default) = Enable trial-encode (overwriting the pre-estimated chroma intra mode)"                                         );
EnvVar ToolOptionList::ChromaIntraReferenceSampleFiltering              ("ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING",                  "0", "0 (default) = No reference sample filtering for chroma (in any format), 1 = Apply filter vertically for 4:2:2 and in both directions for 4:4:4, 2 = Apply filter in both directions for 4:2:2 and 4:4:4"                                                                                                       );
EnvVar ToolOptionList::Get444LMChromaReferenceSamplesFrom1stColumn      ("ECF__GET_444_LMCHROMA_REFERENCE_SAMPLES_FROM_1ST_COLUMN",       "1", "0 = Get reference samples for LM_CHROMA from 2nd column to the left of current TU in all formats, 1 (default) = In 4:4:4, get reference samples from 1st column to the left instead"                                                                                                                           );
EnvVar ToolOptionList::Chroma422IntraAngleScaling                       ("ECF__CHROMA_422_INTRA_ANGLE_SCALING",                           "1", "0 = When generating angular intra predictions for a chroma 4:2:2 TU, intra modes map to the same angles as for square TUs, 1 (default) = scale the angles according to the TU's aspect ratio (i.e. the angle is halved for vertical modes and doubled for horizontal modes)"                                   );
EnvVar ToolOptionList::Chroma422IntraDCDoubleWeightAboveSamples         ("ECF__CHROMA_422_INTRA_DC_DOUBLE_WEIGHT_ABOVE_SAMPLES",          "0", "0 (default) = When generating a DC intra prediction for a chroma 4:2:2 TU, weight each above sample the same as a left sample, 1 = double the weighting of the above samples (i.e. weight each above sample equivalent to two left samples)"                                                                   );
EnvVar ToolOptionList::Chroma422IntraPlanarSingleStageCalculation       ("ECF__CHROMA_422_INTRA_PLANAR_SINGLE_STAGE_CALCULATION",         "0", "0 (default) = When generating planar intra prediction for a chroma 4:2:2 TU, use intermediate stages, 1 = combine all stages into a single calculation"                                                                                                                                                        );
EnvVar ToolOptionList::SetIntraChromaEdgeFilter422                      ("ECF__SET_INTRA_CHROMA_EDGE_FILTER_422",                         "0", "0 (default) = Disable intra edge filtering for chroma 4:2:2, 1 = Enable filtering in vertical direction only, 2 = Enable filtering in both horizontal and vertical directions"                                                                                                                                 );
EnvVar ToolOptionList::SetIntraChromaDCFilter422                        ("ECF__SET_INTRA_CHROMA_DC_FILTER_422",                           "0", "0 (default) = Disable intra DC filtering for chroma 4:2:2, 1 = Enable filtering in vertical direction only, 2 = Enable filtering in both horizontal and vertical directions"                                                                                                                                   );
EnvVar ToolOptionList::SetIntraChromaEdgeFilter444                      ("ECF__SET_INTRA_CHROMA_EDGE_FILTER_444",                         "0", "0 (default) = Disable intra edge filtering for chroma 4:4:4, 1 = Enable filtering in both horizontal and vertical directions"                                                                                                                                                                                  );
EnvVar ToolOptionList::SetIntraChromaDCFilter444                        ("ECF__SET_INTRA_CHROMA_DC_FILTER_444",                           "0", "0 (default) = Disable intra edge filtering for chroma 4:4:4, 1 = Enable filtering in both horizontal and vertical directions"                                                                                                                                                                                  );
EnvVar ToolOptionList::UseLumaFilterForChromaQuarterSampleInterpolation ("ECF__USE_LUMA_FILTER_FOR_CHROMA_QUARTER_SAMPLE_INTERPOLATION",  "0", "0 (default) = Use chroma filter for all chroma interpolation, 1 = Use luma filter wherever quarter-sample interpolation is required (4:2:2 vertical, 4:4:4 both directions)"                                                                                                                                   );
EnvVar ToolOptionList::EnableMDDTFor444Chroma                           ("ECF__ENABLE_MDDT_FOR_444_CHROMA",                               "0", "0 (default) = Use MDDT for luminance only in all formats, 1 = In 4:4:4, also allow MDDT for chrominance TUs"                                                                                                                                                                                                   );
EnvVar ToolOptionList::SingleTransformSkipFlagForAllChannels444         ("ECF__SINGLE_TRANSFORM_SKIP_FLAG_FOR_ALL_CHANNELS_444",          "0", "0 (default) = Always code a transform skip flag for each TU on each channel, 1 = In 4:4:4, code a transform skip flag only for luminance TUs, with corresponding chrominance TUs also using its value"                                                                                                         );
EnvVar ToolOptionList::Chroma422QuantiserAdjustment                     ("ECF__CHROMA_422_QUANTISER_ADJUSTMENT",                          "1", "0 = No quantiser modification for 4:2:2 TUs (shift in transform is rounded down), 1 (default) = Use rounded-down shift in transform and introduce an additional factor of sqrt(2) into the quantisation to normalise, 2 = Use rounded-up shift in transform and additional quantisation factor of 1/(sqrt(2))" );
EnvVar ToolOptionList::Chroma422QuantiserAdjustmentMethod               ("ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD",                   "1", "[NO EFFECT IF ECF__CHROMA_422_QUANTISER_ADJUSTMENT IS 0]  0 = Directly divide/multiply coefficients by sqrt(2), 1 (default) = Modify QP by +/- 3 to effect division/multiplication by sqrt(2), 2 = Modify QP_rem by +/- 3 and use extended 9-element quantisation coefficient tables"                          );
EnvVar ToolOptionList::AdditionalChromaQpMappingTables                  ("ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES",                      "0", "0 (default) = Use same g_aucChromaScale tables for mapping chroma QP as 4:2:0, 1 = Use alternative tables for 4:2:2 and 4:4:4 that tend towards the behaviour of luma"                                                                                                                                         );
EnvVar ToolOptionList::UseTransformDepthFor444ChromaCBFContextSelection ("ECF__USE_TRANSFORM_DEPTH_FOR_444_CHROMA_CBF_CONTEXT_SELECTION", "1", "0 = 4:4:4 Chrominance CBFs use same method as luminance to select context variables, 1 (default) = 4:4:4 Chrominance CBFs use transform depth to select context variables (as in 4:2:0)"                                                                                                                       );
EnvVar ToolOptionList::Chroma422SignificanceMapContextGrid              ("ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID",                 "0", "[AFFECTS 4x8, 8x4, 8x16 and 16x8 TUs] 0 (default) = Use neighbourhood method for significance map context selection, 1 = Use position-repeated versions of the 4x4/8x8 context grids, 2 = As 1, but without re-using the DC context variable for 4x8/8x4"                                                      );
EnvVar ToolOptionList::PatternSigCtxMissingGroupsSameAsAvailableGroups  ("ECF__PATTERNSIGCTX_MISSING_GROUPS_SAME_AS_AVAILABLE_GROUPS",    "0", "0 (default) = When deriving patternSigCtx for significance map context selection, assume 0 for unavailable groups, 1 = If one neighbour group is available and the other is not, assume the same significance as the available group for both groups"                                                          );
EnvVar ToolOptionList::LumaMDCSMode                                     ("ECF__LUMA_MDCS_MODE",                                           "3", "0 = MDCS disabled for luminance, 1 = Horizontal scan only, 2 = Vertical scan only, 3 (default) = Full MDCS (horizontal and vertical scans enabled)"                                                                                                                                                            );
EnvVar ToolOptionList::LumaMDCSAngleLimit                               ("ECF__LUMA_MDCS_ANGLE_LIMIT",                                    "4", "(default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc..."                                                                                                                                                                                                 );
EnvVar ToolOptionList::LumaMDCSMaximumWidth                             ("ECF__LUMA_MDCS_MAXIMUM_WIDTH",                                  "8", "(default 8) Luminance TUs with width greater than this can only use diagonal scan"                                                                                                                                                                                                                             );
EnvVar ToolOptionList::LumaMDCSMaximumHeight                            ("ECF__LUMA_MDCS_MAXIMUM_HEIGHT",                                 "8", "(default 8) Luminance TUs with height greater than this can only use diagonal scan"                                                                                                                                                                                                                            );
EnvVar ToolOptionList::ChromaMDCSMode                                   ("ECF__CHROMA_MDCS_MODE",                                         "3", "0 = MDCS disabled for chrominance, 1 = Horizontal scan only, 2 = Vertical scan only, 3 (default) = Full MDCS (horizontal and vertical scans enabled)"                                                                                                                                                          );
EnvVar ToolOptionList::ChromaMDCSAngleLimit                             ("ECF__CHROMA_MDCS_ANGLE_LIMIT",                                  "4", "(default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc..."                                                                                                                                                                                                 );
EnvVar ToolOptionList::ChromaMDCSMaximumWidth                           ("ECF__CHROMA_MDCS_MAXIMUM_WIDTH",                                "4", "(default 4) Chrominance TUs with width greater than this can only use diagonal scan"                                                                                                                                                                                                                           );
EnvVar ToolOptionList::ChromaMDCSMaximumHeight                          ("ECF__CHROMA_MDCS_MAXIMUM_HEIGHT",                               "4", "(default 4) Chrominance TUs with height greater than this can only use diagonal scan"                                                                                                                                                                                                                          );
EnvVar ToolOptionList::NonSubsampledChromaUseLumaMDCSSizeLimits         ("ECF__NON_SUBSAMPLED_CHROMA_USE_LUMA_MDCS_SIZE_LIMITS",          "1", "0 = Always use chrominance size limits when determining if a chroma TU is too large to use MDCS, 1 (default) = Non-subsampled chrominance axes (vertical for 4:2:2, both for 4:4:4) use the luminance maximum width/height to determine if MDCS should be enabled"                                             );


// --------------------------------------------------------------------------------------------------------------------- //

//macro value printing function

Void printECFMacroSettings()
{
  std::cout << "ECF Non-environment-variable-controlled macros set as follows: \n" << std::endl;

  //------------------------------------------------

  //setting macros

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  PRINT_CONSTANT(ECF__ALL_CHROMA_FORMATS_USE_SAME_TU_STRUCTURE_AS_420,          settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE,                        settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422,                     settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__REDUCED_CHROMA_INTRA_MODE_SET,                            settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH,                   settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ENCODER_INITIAL_INTRA_MODE_PREEST_DMCHROMA,               settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ENCODER_FAST_INTRA_MODE_SEARCH_OVER_ALL_COMPONENTS,       settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ENCODER_FULL_RATE_DISTORTION_SEARCH_OVER_ALL_COMPONENTS,  settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ADDITIONAL_TRIAL_ENCODE_CHROMA_INTRA_MODE_SEARCH,         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING,                  settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__GET_444_LMCHROMA_REFERENCE_SAMPLES_FROM_1ST_COLUMN,       settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_INTRA_ANGLE_SCALING,                           settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_INTRA_DC_DOUBLE_WEIGHT_ABOVE_SAMPLES,          settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_INTRA_PLANAR_SINGLE_STAGE_CALCULATION,         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__SET_INTRA_CHROMA_EDGE_FILTER_422,                         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__SET_INTRA_CHROMA_DC_FILTER_422,                           settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__SET_INTRA_CHROMA_EDGE_FILTER_444,                         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__SET_INTRA_CHROMA_DC_FILTER_444,                           settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__USE_LUMA_FILTER_FOR_CHROMA_QUARTER_SAMPLE_INTERPOLATION,  settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ENABLE_MDDT_FOR_444_CHROMA,                               settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__SINGLE_TRANSFORM_SKIP_FLAG_FOR_ALL_CHANNELS_444,          settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_QUANTISER_ADJUSTMENT,                          settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD,                   settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES,                      settingNameWidth, settingValueWidth);
#endif
  PRINT_CONSTANT(ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA,              settingNameWidth, settingValueWidth);
#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  PRINT_CONSTANT(ECF__USE_TRANSFORM_DEPTH_FOR_444_CHROMA_CBF_CONTEXT_SELECTION, settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID,                 settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__PATTERNSIGCTX_MISSING_GROUPS_SAME_AS_AVAILABLE_GROUPS,    settingNameWidth, settingValueWidth);
#endif
  //These settings cannot be defined using environment variables because they are used to set the size of static const arrays
  PRINT_CONSTANT(ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION,              settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION,                 settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION,                         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CBF_CONTEXT_CHANNEL_SEPARATION,                           settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT,                 settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__EXTENDED_CHROMA_LAST_POSITION_CONTEXT,                    settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__EXTENDED_CHROMA_C1_C2_CONTEXT,                            settingNameWidth, settingValueWidth);
#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  PRINT_CONSTANT(ECF__LUMA_MDCS_MODE,                                           settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__LUMA_MDCS_ANGLE_LIMIT,                                    settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__LUMA_MDCS_MAXIMUM_WIDTH,                                  settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__LUMA_MDCS_MAXIMUM_HEIGHT,                                 settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_MDCS_MODE,                                         settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_MDCS_ANGLE_LIMIT,                                  settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_MDCS_MAXIMUM_WIDTH,                                settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__CHROMA_MDCS_MAXIMUM_HEIGHT,                               settingNameWidth, settingValueWidth);
  PRINT_CONSTANT(ECF__NON_SUBSAMPLED_CHROMA_USE_LUMA_MDCS_SIZE_LIMITS,          settingNameWidth, settingValueWidth);
#endif

  std::cout << std::endl;

  //------------------------------------------------

  //derived macros

#ifdef ECF__CHROMA_NxN_PU_CAN_HAVE_4_PARTS
  std::cout << std::setw(settingNameWidth) << "ECF__CHROMA_NxN_PU_CAN_HAVE_4_PARTS" << " is defined" << std::endl;
#endif

#ifdef ECF__NON_SCALED_INTRA_CHROMA_422_ENABLED
  std::cout << std::setw(settingNameWidth) << "ECF__NON_SCALED_INTRA_CHROMA_422_ENABLED" << " is defined" << std::endl;
#endif

#ifdef ECF__EXTENDED_QP_TABLES
  std::cout << std::setw(settingNameWidth) << "ECF__EXTENDED_QP_TABLES" << " is defined" << std::endl;
#endif

#ifdef ECF__MULTIPLE_CHROMA_QP_MAPPING_TABLES
  std::cout << std::setw(settingNameWidth) << "ECF__MULTIPLE_CHROMA_QP_MAPPING_TABLES" << " is defined" << std::endl;
#endif

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
  std::cout << std::setw(settingNameWidth) << "ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS" << " is defined" << std::endl;
#endif

#ifdef ECF__CHROMA_SIGNIFICANCE_MAP_CONTEXT_SAME_AS_LUMA
  std::cout << std::setw(settingNameWidth) << "ECF__CHROMA_SIGNIFICANCE_MAP_CONTEXT_SAME_AS_LUMA" << " is defined" << std::endl;
#endif

#ifdef ECF__CHROMA_LAST_POSITION_CONTEXT_SAME_AS_LUMA
  std::cout << std::setw(settingNameWidth) << "ECF__CHROMA_LAST_POSITION_CONTEXT_SAME_AS_LUMA" << " is defined" << std::endl;
#endif

#ifdef ECF__CHROMA_C1_C2_CONTEXT_SAME_AS_LUMA
  std::cout << std::setw(settingNameWidth) << "ECF__CHROMA_C1_C2_CONTEXT_SAME_AS_LUMA" << " is defined" << std::endl;
#endif

  //------------------------------------------------

  std::cout << std::endl;
}


// --------------------------------------------------------------------------------------------------------------------- //

//Debugging

UInt  g_debugCounter  = 0;
Bool  g_printDebug    = false;
Void* g_debugAddr     = NULL;

Void printSBACCoeffData(  const UInt          lastX,
                          const UInt          lastY,
                          const UInt          width,
                          const UInt          height,
                          const UInt          chan,
                          const UInt          absPart,
                          const UInt          scanIdx,
                          const TCoeff *const pCoeff,
                          const Bool          finalEncode
                        )
{
  if (DebugOptionList::DebugSBAC.getInt()!=0 && finalEncode)
  {
    std::cout << "Size: " << width << "x" << height << ", Last X/Y: (" << lastX << ", " << lastY << "), absPartIdx: " << absPart << ", scanIdx: " << scanIdx << ", chan: " << chan << std::endl;
    for (int i=0; i<width*height; i++)
    {
      std::cout << std::setw(3) << pCoeff[i];// + dcVal;
      if (i%width == width-1) std::cout << std::endl;
      else                    std::cout << ",";
    }
    std::cout << std::endl;
  }
}

Void printCbfArray( TComDataCU* pcCU  )
{
  const UInt CUSizeInParts = pcCU->getWidth(0)/4;
  const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();
  for (UInt ch=0; ch<numValidComp; ch++)
  {
    const ComponentID compID=ComponentID(ch);
    printf("channel: %d\n", ch);
    for (Int y=0; y<CUSizeInParts; y++)
    {
      for (Int x=0; x<CUSizeInParts; x++)
      {
        printf(x+1==CUSizeInParts?"%3d\n":"%3d, ", pcCU->getCbf(compID)[g_auiRasterToZscan[y*CUSizeInParts + x]]);
      }
    }
  }
}


// --------------------------------------------------------------------------------------------------------------------- //

//String manipulation functions for aligning and wrapping printed text


std::string splitOnSettings(const std::string &input)
{
  std::string result = input;

  std::string::size_type searchFromPosition = 0;

  while (searchFromPosition < result.length())
  {
    //find the " = " that is used to define each setting
    std::string::size_type equalsPosition = result.find(" = ", searchFromPosition);

    if (equalsPosition == std::string::npos) break;

    //then find the end of the numeric characters
    std::string::size_type splitPosition = result.find_last_of("1234567890", equalsPosition);

    //then find the last space before the first numeric character...
    if (splitPosition != std::string::npos) splitPosition = result.find_last_of(' ', splitPosition);

    //...and replace it with a new line
    if (splitPosition != std::string::npos) result.replace(splitPosition, 1, 1, '\n');

    //start the next search from the end of the " = " string
    searchFromPosition = (equalsPosition + 3);
  }

  return result;
}


std::string lineWrap(const std::string &input, const UInt maximumLineLength)
{
  if (maximumLineLength == 0) return input;
  std::string result = input;

  std::string::size_type lineStartPosition = result.find_first_not_of(' '); //don't wrap any leading spaces in the string

  while (lineStartPosition != std::string::npos)
  {
    //------------------------------------------------

    const std::string::size_type searchFromPosition = lineStartPosition + maximumLineLength;

    if (searchFromPosition >= result.length()) break;

    //------------------------------------------------

    //first check to see if there is another new line character before the maximum line length
    //we can't use find for this unfortunately because it doesn't take both a beginning and an end for its search range
    std::string::size_type nextLineStartPosition = std::string::npos;
    for (std::string::size_type currentPosition = lineStartPosition; currentPosition <= searchFromPosition; currentPosition++)
    {
      if (result[currentPosition] == '\n') { nextLineStartPosition = currentPosition + 1; break; }
    }

    //------------------------------------------------

    //if there ia another new line character before the maximum line length, we need to start this loop again from that position
    if (nextLineStartPosition != std::string::npos) lineStartPosition = nextLineStartPosition;
    else
    {
      std::string::size_type spacePosition = std::string::npos;
    
      //search backwards for the last space character (must use signed int because lineStartPosition can be 0)
      for (Int currentPosition = Int(searchFromPosition); currentPosition >= Int(lineStartPosition); currentPosition--)
      {
        if (result[currentPosition] == ' ') { spacePosition = currentPosition; break; }
      }

      //if we didn't find a space searching backwards, we must hyphenate
      if (spacePosition == std::string::npos)
      {
        result.insert(searchFromPosition, "-\n");
        lineStartPosition = searchFromPosition + 2; //make sure the next search ignores the hyphen
      }
      else //if we found a space to split on, replace it with a new line character
      {
        result.replace(spacePosition, 1, 1, '\n');
        lineStartPosition = spacePosition + 1;
      }
    }

    //------------------------------------------------
  }

  return result;
}


std::string indentNewLines(const std::string &input, const UInt indentBy)
{
  std::string result = input;

  const std::string indentString(indentBy, ' ');
  std::string::size_type offset = 0;

  while ((offset = result.find('\n', offset)) != std::string::npos)
  {
    if ((++offset) >= result.length()) break; //increment offset so we don't find the same \n again and do no indentation at the end
    result.insert(offset, indentString);
  }
        
  return result;
}


// --------------------------------------------------------------------------------------------------------------------- //


Void printBlockToStream( std::ostream &ss, const char *pLinePrefix, TComYuv &src, const UInt numSubBlocksAcross, const UInt numSubBlocksUp, const UInt defWidth )
{
  const UInt numValidComp=src.getNumberValidComponents();

  for (UInt ch=0; ch<numValidComp ; ch++)
  {
    const ComponentID compID = ComponentID(ch);
    const UInt width  = src.getWidth(compID);
    const UInt height = src.getHeight(compID);
    const UInt stride = src.getStride(compID);
    const Pel* blkSrc = src.getAddr(compID);
    const UInt subBlockWidth=width/numSubBlocksAcross;
    const UInt subBlockHeight=height/numSubBlocksUp;

    ss << pLinePrefix << " compID: " << compID << "\n";
    for (UInt y=0; y<height; y++)
    {
      if ((y%subBlockHeight)==0 && y!=0)
        ss << pLinePrefix << '\n';

      ss << pLinePrefix;
      for (UInt x=0; x<width; x++)
      {
        if ((x%subBlockWidth)==0 && x!=0)
          ss << std::setw(defWidth+2) << "";

        ss << std::setw(defWidth) << blkSrc[y*stride + x] << ' ';
      }
      ss << '\n';
    }
    ss << pLinePrefix << " --- \n";
  }
}
