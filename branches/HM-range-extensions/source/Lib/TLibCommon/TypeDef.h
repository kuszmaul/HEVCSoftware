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

/** \file     TypeDef.h
    \brief    Define basic types, new types and enumerations
*/

#ifndef _TYPEDEF__
#define _TYPEDEF__

#include <vector>

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// #define DEBUG_STRING              // enable to print out final decision debug info at encoder and decoder:
// #define DEBUG_ENCODER_SEARCH_BINS // enable to print out each bin as it is coded during encoder search

#ifdef DEBUG_STRING
  #define DEBUG_STRING_PASS_INTO(name) , name
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp) , (exp==0)?0:name
  #define DEBUG_STRING_FN_DECLARE(name) , std::string &name
  #define DEBUG_STRING_FN_DECLAREP(name) , std::string *name
  #define DEBUG_STRING_NEW(name) std::string name;
  #define DEBUG_STRING_OUTPUT(os, name) os << name;
  #define DEBUG_STRING_APPEND(str1, str2) str1+=str2;
  #define DEBUG_STRING_SWAP(str1, str2) str1.swap(str2);
  #define DEBUG_STRING_CHANNEL_CONDITION(compID) (compID != COMPONENT_Y)
  #define DEBUG_INTRA_REF_SAMPLES     0
  #define DEBUG_RD_COST_INTRA         0
  #define DEBUG_INTRA_CODING_TU       0
  #define DEBUG_INTRA_CODING_INV_TRAN 0
  #define DEBUG_INTER_CODING_INV_TRAN 0
  #define DEBUG_INTER_CODING_PRED     0
  #define DEBUG_INTER_CODING_RESI     0
  #define DEBUG_INTER_CODING_RECON    0
  #include <sstream>
  #include <iomanip>
#else
  #define DEBUG_STRING_PASS_INTO(name)
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp)
  #define DEBUG_STRING_FN_DECLARE(name)
  #define DEBUG_STRING_FN_DECLAREP(name)
  #define DEBUG_STRING_NEW(name)
  #define DEBUG_STRING_OUTPUT(os, name)
  #define DEBUG_STRING_APPEND(str1, str2)
  #define DEBUG_STRING_SWAP(srt1, str2)
  #define DEBUG_STRING_CHANNEL_CONDITION(compID)
#endif


// ====================================================================================================================
// Tool Switches
// ====================================================================================================================

#define SAO_LUM_CHROMA_ONOFF_FLAGS       1  ///< J0087: slice-level independent luma/chroma SAO on/off flag 
#define LTRP_IN_SPS                      1  ///< J0116: Include support for signalling LTRP LSBs in the SPS, and index them in the slice header.
#define CHROMA_QP_EXTENSION              1  ///< J0342: Extend mapping table from luma QP to chroma QP, introduce slice-level chroma offsets, apply limits on offset values
#define SIMPLE_LUMA_CBF_CTX_DERIVATION   1  ///< J0303: simplified luma_CBF context derivation

#define COEF_REMAIN_BIN_REDUCTION        3 ///< J0142: Maximum codeword length of coeff_abs_level_remaining reduced to 32.
                                           ///< COEF_REMAIN_BIN_REDUCTION is also used to indicate the level at which the VLC 
                                           ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_EG                     1 ///< J0089: Bin reduction for delta QP coding
#if (CU_DQP_TU_EG)
#define CU_DQP_TU_CMAX 5 //max number bins for truncated unary
#define CU_DQP_EG_k 0 //expgolomb order
#endif

#define NAL_UNIT_HEADER                  1  ///< J0550: Define nal_unit_header() method
#define REMOVE_NAL_REF_FLAG              1  ///< J0550: Remove nal_ref_flag, and allocate extra bit to reserved bits, and re-order syntax to put reserved bits after nal_unit_type
#define TEMPORAL_ID_PLUS1                1  ///< J0550: Signal temporal_id_plus1 instead of temporal_id in NAL unit, and change reserved_one_5bits
                                            ///<        value to zero
#define REFERENCE_PICTURE_DEFN           1  ///< J0118: Reflect change of defn. of referece picture in semantics of delta_poc_msb_present_flag
#define MOVE_LOOP_FILTER_SLICES_FLAG     1  ///< J0288: Move seq_loop_filter_across_slices_enabled_flag from SPS to PPS
#define SPLICING_FRIENDLY_PARAMS         1  ///< J0108: Remove rap_pic_id and move no_output_prior_pic_flag

#define  SKIP_FLAG                       1  ///< J0336: store skip flag

#define PPS_TS_FLAG                      1  ///< J0184: move transform_skip_enabled_flag from SPS to PPS
#if PPS_TS_FLAG
#define TS_FLAT_QUANTIZATION_MATRIX      1  ///< I0408: set default quantization matrix to be flat if TS is enabled in PPS
#endif
#define INTER_TRANSFORMSKIP              1  ///< J0237: inter transform skipping (inter-TS)
#define INTRA_TRANSFORMSKIP_FAST         1  ///< J0572: fast encoding for intra transform skipping

#define REMOVAL_8x2_2x8_CG               1  ///< J0256: removal of 8x2 / 2x8 coefficient groups
#define REF_IDX_BYPASS                   1  ///< J0098: bypass coding starting from the second bin for reference index

#define RECALCULATE_QP_ACCORDING_LAMBDA  1  ///< J0242: recalculate QP value according to lambda value
#define TU_ZERO_CBF_RDO                  1  ///< J0241: take the bits to represent zero cbf into consideration when doing TU RDO
#define REMOVE_NUM_GREATER1              1  ///< J0408: numGreater1 removal and ctxset decision with c1 

#define INTRA_TRANS_SIMP                 1  ///< J0035: Use DST for 4x4 luma intra TU's (regardless of the intra prediction direction)

#define J0234_INTER_RPS_SIMPL            1  ///< J0234: Do not signal delta_idx_minus1 when building the RPS-list in SPS
#define NUM_WP_LIMIT                     1  ///< J0571: number of total signalled weight flags <=24
#define DISALLOW_BIPRED_IN_8x4_4x8PUS    1  ///< J0086: disallow bi-pred for 8x4 and 4x8 inter PUs
#define SAO_SINGLE_MERGE                 1  ///< J0355: Single SAO merge flag for all color components (per Left and Up merge)
#define SAO_TYPE_SHARING                 1  ///< J0045: SAO types, merge left/up flags are shared between Cr and Cb
#define SAO_TYPE_CODING                  1  ///< J0268: SAO type signalling using 1 ctx on/off flag + 1 bp BO/EO flag + 2 bp bins for EO class
#define SAO_MERGE_ONE_CTX                1  ///< J0041: SAO merge left/up flags share the same ctx
#define SAO_ABS_BY_PASS                  1  ///< J0043: by pass coding for SAO magnitudes 
#define SAO_LCU_BOUNDARY                 1  ///< J0139: SAO parameter estimation using non-deblocked pixels for LCU bottom and right boundary areas
#define MODIFIED_CROSS_SLICE             1  ///< J0266: SAO slice boundary control for GDR
#define CU_DQP_ENABLE_FLAG               1  ///< J0220: cu_qp_delta_enabled_flag in PPS
#define REMOVE_ZIGZAG_SCAN               1  ///< J0150: removal of zigzag scan

#define TRANS_SPLIT_FLAG_CTX_REDUCTION   1  ///< J0133: Reduce the context number of transform split flag to 3

#define WP_PARAM_RANGE_LIMIT             1  ///< J0221: Range limit of delta_weight and delta_offset for chroma.
#define J0260 1 ///< Fix in rate control equations

#define SLICE_HEADER_EXTENSION           1  ///< II0235: Slice header extension mechanism

#define REMOVE_NSQT                      1 ///< Disable NSQT-related code
#define REMOVE_LMCHROMA                  1 ///< Disable LM_Chroma-related code
#define REMOVE_FGS                       1 ///< Disable fine-granularity slices code
#define REMOVE_ALF                       1 ///< Disable ALF-related code
#define REMOVE_APS                       1 ///< Disable APS-related code

#define PREVREFPIC_DEFN                  0  ///< J0248: Shall be set equal to 0! (prevRefPic definition reverted to CD definition)
#define BYTE_ALIGNMENT                   1  ///< I0330: Add byte_alignment() procedure to end of slice header

#define SBH_THRESHOLD                    4  ///< I0156: value of the fixed SBH controlling threshold
  
#define SEQUENCE_LEVEL_LOSSLESS          0  ///< H0530: used only for sequence or frame-level lossless coding

#define DISABLING_CLIP_FOR_BIPREDME      1  ///< Ticket #175
  
#define C1FLAG_NUMBER                    8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER                    1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5 

#define REMOVE_SAO_LCU_ENC_CONSTRAINTS_3 1  ///< disable the encoder constraint that conditionally disable SAO for chroma for entire slice in interleaved mode

#define SAO_SKIP_RIGHT                   1  ///< H1101: disallow using unavailable pixel during RDO

#define SAO_ENCODING_CHOICE              1  ///< I0184: picture early termination
#define PICTURE_SAO_RDO_FIX              0  ///< J0097: picture-based SAO optimization fix
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                0.75
#define SAO_ENCODING_CHOICE_CHROMA       1 ///< J0044: picture early termination Luma and Chroma are handled separatenly
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA         0.5
#endif
#endif

#define MAX_NUM_SAO_OFFSETS              4

#define MAX_NUM_SPS                     32
#define MAX_NUM_PPS                    256
#define MAX_NUM_APS                     32         //< !!!KS: number not defined in WD yet

#define MRG_MAX_NUM_CANDS_SIGNALED       5   //<G091: value of maxNumMergeCand signaled in slice header 

#define WEIGHTED_CHROMA_DISTORTION       1   ///< F386: weighting of chroma for RDO
#define RDOQ_CHROMA_LAMBDA               1   ///< F386: weighting of chroma for RDOQ
#define ALF_CHROMA_LAMBDA                1   ///< F386: weighting of chroma for ALF
#define SAO_CHROMA_LAMBDA                1   ///< F386: weighting of chroma for SAO

#define MIN_SCAN_POS_CROSS               4

#define FAST_BIT_EST                     1   ///< G763: Table-based bit estimation for CABAC

#define MLS_GRP_NUM                     64     ///< G644 : Max number of coefficient groups, max(16, 64)
#define MLS_CG_SIZE                      4      ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION            1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                  7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                     30     ///< G382: max coefficient level in statistics collection
#endif

#if REMOVE_NSQT
#define NS_HAD                           0
#else
#define NS_HAD                           1
#endif

#define APS_BITS_FOR_SAO_BYTE_LENGTH    12           
#define APS_BITS_FOR_ALF_BYTE_LENGTH     8

#define HHI_RQT_INTRA_SPEEDUP            1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD        0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR           4

#define SCAN_SET_SIZE                   16
#define LOG2_SCAN_SET_SIZE               4

#define FAST_UDI_MAX_RDMODE_NUM         35          ///< maximum number of RD comparison in fast-UDI estimation loop 

#define ZERO_MVD_EST                     0           ///< Zero Mvd Estimation in normal mode

#define NUM_INTRA_MODE                  36
#if !REMOVE_LM_CHROMA
#define LM_CHROMA_IDX                   35
#endif

#define IBDI_DISTORTION                  0           ///< enable/disable SSE modification when IBDI is used (JCTVC-D152)
#define FIXED_ROUNDING_FRAME_MEMORY      0           ///< enable/disable fixed rounding to 8-bitdepth of frame memory when IBDI is used  

#define WRITE_BACK                       1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                   1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                   0           ///< Enable/disable the printing of bits used to send the RPS.
                                                    // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                    // this should be done with encoder only decision
                                                    // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX                       0
#define VER_IDX                         26                    // index for intra VERTICAL   mode
#define HOR_IDX                         10                    // index for intra HORIZONTAL mode
#define DC_IDX                           1                    // index for intra DC mode
#if REMOVE_LMCHROMA
#define NUM_CHROMA_MODE                  5                    // total number of chroma modes
#else
#define NUM_CHROMA_MODE                  6                    // total number of chroma modes
#endif
#define DM_CHROMA_IDX                   36                    // chroma mode index for derived from luma intra mode
#define INVALID_MODE_IDX                (NUM_INTRA_MODE+1)    // value used to indicate an invalid intra mode
#define STOPCHROMASEARCH_MODE_IDX       (INVALID_MODE_IDX+1)  // value used to signal the end of a chroma mode search



#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS             0           ///< Disable counting dQP bits in RDO-based mode decision

#define FULL_NBIT 0 ///< When enabled, does not use g_uiBitIncrement anymore to support > 8 bit data

#define AD_HOC_SLICES_FIXED_NUMBER_OF_LCU_IN_SLICE      1          ///< OPTION IDENTIFIER. mode==1 -> Limit maximum number of largest coding tree blocks in a slice
#define AD_HOC_SLICES_FIXED_NUMBER_OF_BYTES_IN_SLICE    2          ///< OPTION IDENTIFIER. mode==2 -> Limit maximum number of bins/bits in a slice
#define AD_HOC_SLICES_FIXED_NUMBER_OF_TILES_IN_SLICE    3

#define DEPENDENT_SLICES                 1 ///< JCTVC-I0229
// Dependent slice options
#define SHARP_FIXED_NUMBER_OF_LCU_IN_DEPENDENT_SLICE            1          ///< OPTION IDENTIFIER. Limit maximum number of largest coding tree blocks in an dependent slice
#define SHARP_MULTIPLE_CONSTRAINT_BASED_DEPENDENT_SLICE         2          ///< OPTION IDENTIFIER. Limit maximum number of bins/bits in an dependent slice
#if DEPENDENT_SLICES
#define FIXED_NUMBER_OF_TILES_IN_DEPENDENT_SLICE                3          // JCTVC-I0229
#endif

#define LOG2_MAX_NUM_COLUMNS_MINUS1        7
#define LOG2_MAX_NUM_ROWS_MINUS1           7
#define LOG2_MAX_COLUMN_WIDTH              13
#define LOG2_MAX_ROW_HEIGHT                13

#define MATRIX_MULT                            0 // Brute force matrix multiplication instead of partial butterfly

#define REG_DCT 65535

#define AMP_SAD                                1 ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                        1 ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                                1 ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define SCALING_LIST_OUTPUT_RESULT             0 //JCTVC-G880/JCTVC-G1016 quantization matrices
                                             
#define CABAC_INIT_PRESENT_FLAG                1

#define LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS   4 // NOTE: ECF - new definition
#define CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS 8 // NOTE: ECF - new definition


// ====================================================================================================================
// VPS constants
// ====================================================================================================================

#define MAX_LAYER_NUM                         10
#define MAX_NUM_VPS                           16


// ====================================================================================================================
// ECF control settings
// ====================================================================================================================

//------------------------------------------------
// Enable environment variables
//------------------------------------------------

#define ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST                              0 ///< When enabled, allows control of ECF modifications via environment variables
#define ECF__PRINT_MACRO_VALUES                                               1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

//------------------------------------------------
// Block Structure
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__ALL_CHROMA_FORMATS_USE_SAME_TU_STRUCTURE_AS_420                  0 ///< 0 (default) = Allow chroma TU tree to split down to the minimum possible TU size, 1 = Prevent chroma TU splitting wherever an equivalent 4:2:0 chroma TU could not split (e.g. prevent splitting of chroma TUs wherever luma splits down to 4x4)
  #define ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE                                0 ///< 0 (default) = An intra-NxN-split CU always has only one chroma PU, 1 = In 4:4:4, an intra-NxN-split CU can have four chroma PUs (subject to limitations on minimum TU size etc.), 2 = As 1, but for any chroma format (not just 4:4:4)
  #define ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422                             0 ///< 0 (default) = use standard size square coefficient groups for all formats, 1 = use double-height groups for 4:2:2
#endif

//------------------------------------------------
// Intra Prediction
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__REDUCED_CHROMA_INTRA_MODE_SET                                    0 ///< 0 (default) = Allow chroma to select a different intra prediction mode to luma, 1 = Always use DM_Chroma or LM_Chroma (when enbled)
  
  #define ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH                           0 ///< 0 (default) = When processing the intra prediction mode search that defines the TU tree, only take luma into account, 1 = Also take chroma into account
  #define ECF__ENCODER_INITIAL_INTRA_MODE_PREEST_DMCHROMA                       0 ///< [NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Use pre-est to estimate initial chroma intra prediction mode, 1 = Set initial chroma intra prediciton mode to DM_CHROMA
  #define ECF__ENCODER_FAST_INTRA_MODE_SEARCH_OVER_ALL_COMPONENTS               0 ///< [NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Fast encoder intra mode search using luma only, 1 = Fast encoder intra mode search using all components
  #define ECF__ENCODER_FULL_RATE_DISTORTION_SEARCH_OVER_ALL_COMPONENTS          0 ///< [NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 (default) = Full rate-distortion intra mode search using luma only, 1 = Full rate-distortion intra mode search also tests all allowed chroma intra modes
  #define ECF__ADDITIONAL_TRIAL_ENCODE_CHROMA_INTRA_MODE_SEARCH                 1 ///< [NO EFFECT IF ECF__COMBINED_LUMA_CHROMA_INTRA_MODE_SEARCH IS 0]  0 = When using combined luma & chroma intra search, skip the trial-encode to define the final chroma intra mode, 1 (default) = Enable trial-encode (overwriting the pre-estimated chroma intra mode)

  #define ECF__CHROMA_INTRA_REFERENCE_SAMPLE_FILTERING                          0 ///< 0 (default) = No reference sample filtering for chroma (in any format), 1 = Apply filter vertically for 4:2:2 and in both directions for 4:4:4, 2 = Apply filter in both directions for 4:2:2 and 4:4:4
  #define ECF__GET_444_LMCHROMA_REFERENCE_SAMPLES_FROM_1ST_COLUMN               1 ///< 0 = Get reference samples for LM_CHROMA from 2nd column to the left of current TU in all formats, 1 (default) = In 4:4:4, get reference samples from 1st column to the left instead

  #define ECF__CHROMA_422_INTRA_ANGLE_SCALING                                   1 ///< 0 = When generating angular intra predictions for a chroma 4:2:2 TU, intra modes map to the same angles as for square TUs, 1 (default) = scale the angles according to the TU's aspect ratio (i.e. the angle is halved for vertical modes and doubled for horizontal modes)
  #define ECF__CHROMA_422_INTRA_DC_DOUBLE_WEIGHT_ABOVE_SAMPLES                  0 ///< 0 (default) = When generating DC intra prediction for a chroma 4:2:2 TU, weight each above sample the same as a left sample, 1 = double the weighting of the above samples (i.e. weight each above sample equivalent to two left samples)
  #define ECF__CHROMA_422_INTRA_PLANAR_SINGLE_STAGE_CALCULATION                 0 ///< 0 (default) = When generating planar intra prediction for a chroma 4:2:2 TU, use intermediate stages, 1 = combine all stages into a single calculation

  #define ECF__SET_INTRA_CHROMA_EDGE_FILTER_422                                 0 ///< 0 (default) = Disable intra edge filtering for chroma 4:2:2, 1 = Enable filtering in vertical direction only, 2 = Enable filtering in both horizontal and vertical directions
  #define ECF__SET_INTRA_CHROMA_DC_FILTER_422                                   0 ///< 0 (default) = Disable intra DC filtering for chroma 4:2:2, 1 = Enable filtering in vertical direction only, 2 = Enable filtering in both horizontal and vertical directions
  
  #define ECF__SET_INTRA_CHROMA_EDGE_FILTER_444                                 0 ///< 0 (default) = Disable intra edge filtering for chroma 4:4:4, 1 = Enable filtering in both horizontal and vertical directions
  #define ECF__SET_INTRA_CHROMA_DC_FILTER_444                                   0 ///< 0 (default) = Disable intra DC filtering for chroma 4:4:4, 1 = Enable filtering in both horizontal and vertical directions
#endif

//------------------------------------------------
// Inter Prediction
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__USE_LUMA_FILTER_FOR_CHROMA_QUARTER_SAMPLE_INTERPOLATION          0 ///< 0 (default) = Use chroma filter for all chroma interpolation, 1 = Use luma filter wherever quarter-sample interpolation is required (4:2:2 vertical, 4:4:4 both directions)
#endif

//------------------------------------------------
// Transform and Quantisation
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__ENABLE_MDDT_FOR_444_CHROMA                                       0 ///< 0 (default) = Use MDDT for luminance only in all formats, 1 = In 4:4:4, also allow MDDT for chrominance TUs
  #define ECF__SINGLE_TRANSFORM_SKIP_FLAG_FOR_ALL_CHANNELS_444                  0 ///< 0 (default) = Always code a transform skip flag for each TU on each channel, 1 = In 4:4:4, code a transform skip flag only for luminance TUs, with corresponding chrominance TUs also using its value
  #define ECF__CHROMA_422_QUANTISER_ADJUSTMENT                                  1 ///< 0 = No quantiser modification for 4:2:2 TUs (shift in transform is rounded down), 1 (default) = Use rounded-down shift in transform and introduce an additional factor of sqrt(2) into the quantisation to normalise, 2 = Use rounded-up shift in transform and additional quantisation factor of 1/(sqrt(2))
  #define ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD                           1 ///< [NO EFFECT IF ECF__CHROMA_422_QUANTISER_ADJUSTMENT IS 0]  0 = Directly divide/multiply coefficients by sqrt(2), 1 (default) = Modify QP by +/- 3 to effect division/multiplication by sqrt(2), 2 = Modify QP_rem by +/- 3 and use extended 9-element quantisation coefficient tables
  #define ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES                              0 ///< 0 (default) = Use same g_aucChromaScale tables for mapping chroma QP as 4:2:0, 1 = Use alternative tables for 4:2:2 and 4:4:4 that tend towards the behaviour of luma
#endif

//------------------

//These settings cannot be defined using environment variables because they are used to set the size of static const arrays

#define ECF__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA                      0 ///< 0 (default) = Chroma shares the Luma 32x32 ScalingList (ensures compatibility with existing scaling list definition files). 1 = Chroma channels have their own 32x32 ScalingList

//------------------------------------------------
// Context Variable Selection
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__USE_TRANSFORM_DEPTH_FOR_444_CHROMA_CBF_CONTEXT_SELECTION         1 ///< 0 = 4:4:4 Chrominance CBFs use same method as luminance to select context variables, 1 (default) = 4:4:4 Chrominance CBFs use transform depth to select context variables (as in 4:2:0)
  #define ECF__CHROMA_422_SIGNIFICANCE_MAP_CONTEXT_GRID                         0 ///< [AFFECTS 4x8, 8x4, 8x16 and 16x8 TUs] 0 (default) = Use neighbourhood method for significance map context selection, 1 = Use position-repeated versions of the 4x4/8x8 context grids, 2 = As 1, but without re-using the DC context variable for 4x8/8x4
  #define ECF__PATTERNSIGCTX_MISSING_GROUPS_SAME_AS_AVAILABLE_GROUPS            0 ///< 0 (default) = When deriving patternSigCtx for significance map context selection, assume 0 for unavailable groups, 1 = If one neighbour group is available and the other is not, assume the same significance as the available group for both groups
#endif

//------------------

//These settings cannot be defined using environment variables because they are used to set the size of static const arrays

#define ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION                      1 ///< 0 = All channels use the same context variables (using luma selection method), 1 (default) = Luminance separate from chrominance, 2 = All channels separate from each other
#define ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION                         1 ///< 0 = All channels use the same context variables (using luma selection method), 1 (default) = Luminance separate from chrominance, 2 = All channels separate from each other
#define ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION                                 1 ///< 0 = All channels use the same context variables (using luma selection method), 1 (default) = Luminance separate from chrominance, 2 = All channels separate from each other
#define ECF__CBF_CONTEXT_CHANNEL_SEPARATION                                   1 ///< 0 = All channels use the same context variables (using luma selection method), 1 (default) = Luminance separate from chrominance, 2 = All channels separate from each other

#define ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT                         0 ///< [NO EFFECT (ALWAYS ON) IF ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION IS 0]  0 (default) = Chrominance uses standard context variables and selection method, 1 = Chrominance uses the same number of context variables and selection method as luminance
#define ECF__EXTENDED_CHROMA_LAST_POSITION_CONTEXT                            0 ///< [NO EFFECT (ALWAYS ON) IF ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION IS 0   ]  0 (default) = Chrominance uses standard context variables and selection method, 1 = Chrominance uses the same number of context variables and selection method as luminance
#define ECF__EXTENDED_CHROMA_C1_C2_CONTEXT                                    0 ///< [NO EFFECT (ALWAYS ON) IF ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION IS 0           ]  0 (default) = Chrominance uses standard context variables and selection method, 1 = Chrominance uses the same number of context variables and selection method as luminance

//------------------------------------------------
// MDCS
//------------------------------------------------

#if (ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST == 0)
  #define ECF__LUMA_MDCS_MODE                                                   3 ///< 0 = MDCS disabled for luminance, 1 = Horizontal scan only, 2 = Vertical scan only, 3 (default) = Full MDCS (horizontal and vertical scans enabled)
  #define ECF__LUMA_MDCS_ANGLE_LIMIT                                            4 ///< (default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
  #define ECF__LUMA_MDCS_MAXIMUM_WIDTH                                          8 ///< (default 8) Luminance TUs with width greater than this can only use diagonal scan
  #define ECF__LUMA_MDCS_MAXIMUM_HEIGHT                                         8 ///< (default 8) Luminance TUs with height greater than this can only use diagonal scan

  #define ECF__CHROMA_MDCS_MODE                                                 3 ///< 0 = MDCS disabled for chrominance, 1 = Horizontal scan only, 2 = Vertical scan only, 3 (default) = Full MDCS (horizontal and vertical scans enabled)
  #define ECF__CHROMA_MDCS_ANGLE_LIMIT                                          4 ///< (default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
  #define ECF__CHROMA_MDCS_MAXIMUM_WIDTH                                        4 ///< (default 4) Chrominance TUs with width greater than this can only use diagonal scan
  #define ECF__CHROMA_MDCS_MAXIMUM_HEIGHT                                       4 ///< (default 4) Chrominance TUs with height greater than this can only use diagonal scan

  #define ECF__NON_SUBSAMPLED_CHROMA_USE_LUMA_MDCS_SIZE_LIMITS                  1 ///< 0 = Always use chrominance size limits when determining if a chroma TU is too large to use MDCS, 1 (default) = Non-subsampled chrominance axes (vertical for 4:2:2, both for 4:4:4) use the luminance maximum width/height to determine if MDCS should be enabled
#endif

//------------------------------------------------
// Backwards-compatibility
//------------------------------------------------

#define ECF__BACKWARDS_COMPATIBILITY_HM                                       0 ///< Maintain backwards compatibility with HM for certain non-standard test configuration settings
#define ECF__BACKWARDS_COMPATIBILITY_HM_TRANSQUANTBYPASS                      0 ///< Maintain backwards compatibility with HM's transquant lossless encoding methods

//------------------------------------------------
// Error checking
//------------------------------------------------

#if ((ECF__CHROMA_422_QUANTISER_ADJUSTMENT < 0) || (ECF__CHROMA_422_QUANTISER_ADJUSTMENT > 2))
#error 4:2:2 QP mode must be 0, 1 or 2
#endif

//------------------------------------------------
// Derived Macros
//------------------------------------------------

#if ((ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST != 0) || (ECF__INTRA_NxN_CU_CHROMA_PU_SPLIT_MODE != 0))
#define ECF__CHROMA_NxN_PU_CAN_HAVE_4_PARTS
#endif

#if ((ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST != 0) || (ECF__CHROMA_422_INTRA_ANGLE_SCALING == 0))
#define ECF__NON_SCALED_INTRA_CHROMA_422_ENABLED
#endif

#if ((ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST != 0) || ((ECF__CHROMA_422_QUANTISER_ADJUSTMENT != 0) && (ECF__CHROMA_422_QUANTISER_ADJUSTMENT_METHOD == 2)))
#define ECF__EXTENDED_QP_TABLES
#endif

#if ((ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST != 0) || (ECF__ADDITIONAL_CHROMA_QP_MAPPING_TABLES == 1))
#define ECF__MULTIPLE_CHROMA_QP_MAPPING_TABLES
#endif

#if ((ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST != 0) || (ECF__DOUBLE_HEIGHT_COEFFICIENT_GROUPS_422 != 0))
#define ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
#endif

//if using fully unified or extended context variables, always use the luminance selection methods

#if ((ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0) || (ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT == 1))
#define ECF__CHROMA_SIGNIFICANCE_MAP_CONTEXT_SAME_AS_LUMA
#endif

#if ((ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 0) || (ECF__EXTENDED_CHROMA_LAST_POSITION_CONTEXT == 1))
#define ECF__CHROMA_LAST_POSITION_CONTEXT_SAME_AS_LUMA
#endif

#if ((ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 0) || (ECF__EXTENDED_CHROMA_C1_C2_CONTEXT == 1))
#define ECF__CHROMA_C1_C2_CONTEXT_SAME_AS_LUMA
#endif


// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

typedef       char                Char;
typedef       unsigned char       UChar;
typedef       short               Short;
typedef       unsigned short      UShort;
typedef       int                 Int;
typedef       unsigned int        UInt;
typedef       double              Double;


// ====================================================================================================================
// 64-bit integer type
// ====================================================================================================================

#ifdef _MSC_VER
typedef       __int64             Int64;

#if _MSC_VER <= 1200 // MS VC6
typedef       __int64             UInt64;   // MS VC6 does not support unsigned __int64 to double conversion
#else
typedef       unsigned __int64    UInt64;
#endif

#else

typedef       long long           Int64;
typedef       unsigned long long  UInt64;

#endif


// ====================================================================================================================
// Enumeration
// ====================================================================================================================

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

enum ComponentID
{
  COMPONENT_Y       = 0,
  COMPONENT_Cb      = 1,
  COMPONENT_Cr      = 2,
  MAX_NUM_COMPONENT = 3
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported partition shape
enum PartSize
{
  SIZE_2Nx2N           = 0,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN            = 1,           ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N            = 2,           ///< symmetric motion partition,   Nx2N
  SIZE_NxN             = 3,           ///< symmetric motion partition,   Nx N
  SIZE_2NxnU           = 4,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD           = 5,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N           = 6,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N           = 7,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
  NUMBER_OF_PART_SIZES = 8
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  NUMBER_OF_PREDICTION_MODES = 2
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0 = 0,   ///< reference list 0
  REF_PIC_LIST_1 = 1,   ///< reference list 1
  REF_PIC_LIST_C = 2,   ///< combined reference list for uni-prediction in B-Slices
  REF_PIC_LIST_X = 100  ///< special mark
};

static const UInt NUM_REF_PIC_LIST_01  = 2; // NOTE: ECF - new definition
static const UInt NUM_REF_PIC_LIST_01C = 3; // NOTE: ECF - new definition

/// distortion function index
enum DFunc
{
  DF_DEFAULT         = 0,
  DF_SSE             = 1,      ///< general size SSE
  DF_SSE4            = 2,      ///<   4xM SSE
  DF_SSE8            = 3,      ///<   8xM SSE
  DF_SSE16           = 4,      ///<  16xM SSE
  DF_SSE32           = 5,      ///<  32xM SSE
  DF_SSE64           = 6,      ///<  64xM SSE
  DF_SSE16N          = 7,      ///< 16NxM SSE
  
  DF_SAD             = 8,      ///< general size SAD
  DF_SAD4            = 9,      ///<   4xM SAD
  DF_SAD8            = 10,     ///<   8xM SAD
  DF_SAD16           = 11,     ///<  16xM SAD
  DF_SAD32           = 12,     ///<  32xM SAD
  DF_SAD64           = 13,     ///<  64xM SAD
  DF_SAD16N          = 14,     ///< 16NxM SAD
  
  DF_SADS            = 15,     ///< general size SAD with step
  DF_SADS4           = 16,     ///<   4xM SAD with step
  DF_SADS8           = 17,     ///<   8xM SAD with step
  DF_SADS16          = 18,     ///<  16xM SAD with step
  DF_SADS32          = 19,     ///<  32xM SAD with step
  DF_SADS64          = 20,     ///<  64xM SAD with step
  DF_SADS16N         = 21,     ///< 16NxM SAD with step
  
  DF_HADS            = 22,     ///< general size Hadamard with step
  DF_HADS4           = 23,     ///<   4xM HAD with step
  DF_HADS8           = 24,     ///<   8xM HAD with step
  DF_HADS16          = 25,     ///<  16xM HAD with step
  DF_HADS32          = 26,     ///<  32xM HAD with step
  DF_HADS64          = 27,     ///<  64xM HAD with step
  DF_HADS16N         = 28,     ///< 16NxM HAD with step
  
#if AMP_SAD
  DF_SAD12           = 43,
  DF_SAD24           = 44,
  DF_SAD48           = 45,

  DF_SADS12          = 46,
  DF_SADS24          = 47,
  DF_SADS48          = 48,

  DF_SSE_FRAME       = 50,     ///< Frame-based SSE
  DF_TOTAL_FUNCTIONS = 64
#else
  DF_SSE_FRAME       = 32,     ///< Frame-based SSE
  DF_TOTAL_FUNCTIONS = 33
#endif
};

/// index for SBAC based RD optimization
enum CI_IDX
{
  CI_CURR_BEST = 0,     ///< best mode index
  CI_NEXT_BEST,         ///< next best index
  CI_TEMP_BEST,         ///< temporal index
  CI_CHROMA_INTRA,      ///< chroma intra index
  CI_QT_TRAFO_TEST,
  CI_QT_TRAFO_ROOT,
  CI_NUM,               ///< total number
};

/// motion vector predictor direction used in AMVP
enum MVP_DIR
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

/// motion vector prediction mode used in AMVP
enum AMVP_MODE
{
  AM_NONE = 0,          ///< no AMVP mode
  AM_EXPL,              ///< explicit signalling of motion vector index
};

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_ZIGZAG = 0,      ///< typical zigzag scan
  SCAN_HOR    = 1,      ///< horizontal first scan
  SCAN_VER    = 2,      ///< vertical first scan
  SCAN_DIAG   = 3,      ///< up-right diagonal scan
  SCAN_NUMBER_OF_TYPES = 4
};

enum COEFF_SCAN_GROUP_TYPE
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
  SCAN_GROUPED_4x8 = 2,
  SCAN_NUMBER_OF_GROUP_TYPES = 3
#else
  SCAN_NUMBER_OF_GROUP_TYPES = 2
#endif
};

enum SignificanceMapContextType
{
  CONTEXT_TYPE_4x4 = 0,
  CONTEXT_TYPE_8x8 = 1,
  CONTEXT_TYPE_NxN = 2,
  CONTEXT_NUMBER_OF_TYPES = 3
};

#if !REMOVE_NSQT
/// scaling list types
enum ScalingListDIR
{
  SCALING_LIST_SQT = 0,
  SCALING_LIST_VER,
  SCALING_LIST_HOR,
  SCALING_LIST_DIR_NUM
};
#endif

enum ScalingListSize
{
  SCALING_LIST_4x4 = 0,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_SIZE_NUM
};

enum ErrorScaleAdjustmentMode
{
  ERROR_SCALE_ADJUSTMENT_MODE_NONE       = 0,
  ERROR_SCALE_ADJUSTMENT_MODE_422        = 1,
  NUMBER_OF_ERROR_SCALE_ADJUSTMENT_MODES = 2
};

///MDCS modes
enum MDCSMode
{
  MDCS_DISABLED        = 0,
  MDCS_HORIZONTAL_ONLY = 1,
  MDCS_VERTICAL_ONLY   = 2,
  MDCS_BOTH_DIRECTIONS = 3,
  MDCS_NUMBER_OF_MODES = 4
};

///Filter modes
enum FilterMode
{
  FILTER_DISABLED        = 0,
  FILTER_HORIZONTAL_ONLY = 1,
  FILTER_VERTICAL_ONLY   = 2,
  FILTER_BOTH_DIRECTIONS = 3,
  FILTER_NUMBER_OF_MODES = 4
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

typedef       Short           Pel;               ///< pixel type
typedef       Int             TCoeff;            ///< transform coefficient
typedef       Short           TMatrixCoeff;      ///< transform matrix coefficient
typedef       Short           TFilterCoeff;      ///< filter coefficient
typedef       Int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       UInt            Intermediate_UInt; ///< used as intermediate value in calculations

/// parameters for adaptive loop filter
class TComPicSym;

#define NUM_DOWN_PART 4

enum SAOTypeLen
{
  SAO_EO_LEN    = 4, 
  SAO_BO_LEN    = 4,
  SAO_MAX_BO_CLASSES = 32
};

enum SAOType
{
  SAO_EO_0 = 0, 
  SAO_EO_1,
  SAO_EO_2, 
  SAO_EO_3,
  SAO_BO,
  MAX_NUM_SAO_TYPE
};

typedef struct _SaoQTPart
{
  Int         iBestType;
  Int         iLength;
#if SAO_TYPE_CODING
  Int         subTypeIdx;                 ///< indicates EO class or BO band position
#else
  Int         bandPosition;
#endif
  Int         iOffset[MAX_NUM_SAO_OFFSETS];
  Int         StartCUX;
  Int         StartCUY;
  Int         EndCUX;
  Int         EndCUY;

  Int         PartIdx;
  Int         PartLevel;
  Int         PartCol;
  Int         PartRow;

  Int         DownPartsIdx[NUM_DOWN_PART];
  Int         UpPartIdx;

  Bool        bSplit;

  //---- encoder only start -----//
  Bool        bProcessed;
  Double      dMinCost;
  Int64       iMinDist;
  Int         iMinRate;
  //---- encoder only end -----//
} SAOQTPart;

typedef struct _SaoLcuParam
{
  Bool       mergeUpFlag;
  Bool       mergeLeftFlag;
  Int        typeIdx;
#if SAO_TYPE_CODING
  Int        subTypeIdx;                  ///< indicates EO class or BO band position
#else
  Int        bandPosition;
#endif
  Int        offset[MAX_NUM_SAO_OFFSETS];
  Int        partIdx;
  Int        partIdxTmp;
  Int        length;
} SaoLcuParam;

struct SAOParam
{
#if SAO_TYPE_SHARING
  Bool         bSaoFlag[MAX_NUM_CHANNEL_TYPE];
#else
  Bool         bSaoFlag[MAX_NUM_COMPONENT];
#endif
  SAOQTPart*   psSaoPart[MAX_NUM_COMPONENT];
  Int          iMaxSplitLevel;
  Int          iNumClass[MAX_NUM_SAO_TYPE];
  Bool         oneUnitFlag[MAX_NUM_COMPONENT];
  SaoLcuParam* saoLcuParam[MAX_NUM_COMPONENT];
  Int          numCuInHeight;
  Int          numCuInWidth;
  ~SAOParam();
};

#if !REMOVE_ALF
struct ALFParam
{
  Int alf_flag;                           ///< indicates use of ALF
  Int num_coeff;                          ///< number of filter coefficients
  Int filter_shape;
  Int *filterPattern;
  Int startSecondFilter;
  Int filters_per_group;
  Int **coeffmulti;
  ComponentID componentID;

  //constructor, operator
  ALFParam():componentID(MAX_NUM_COMPONENT)  {}
  ALFParam(ComponentID cID)              {create(cID);}
  ALFParam(const ALFParam& src)          {*this = src;}
  ~ALFParam(){destroy();}
  const ALFParam& operator= (const ALFParam& src);
private:
  Void create(ComponentID cID);
  Void destroy();
  Void copy(const ALFParam& src);
};
#endif

/// parameters for deblocking filter
typedef struct _LFCUParam
{
  Bool bInternalEdge;                     ///< indicates internal edge
  Bool bLeftEdge;                         ///< indicates left edge
  Bool bTopEdge;                          ///< indicates top edge
} LFCUParam;



//TU settings for entropy encoding
struct TUEntropyCodingParameters
{
  const UInt            *scan;
  const UInt            *scanCG;
        COEFF_SCAN_TYPE  scanType;
        UInt             log2GroupWidth;
        UInt             log2GroupHeight;
        UInt             widthInGroups;
        UInt             heightInGroups;
        UInt             firstSignificanceMapContext;
        Bool             useFixedGridSignificanceMapContext;

        //------------------
        
        struct FixedGridContextParameters
        {
          const UInt *grid;
                UInt  stride;
                UInt  widthScale;
                UInt  heightScale;

          Void initialise(const UInt *gridSource, const UInt log2GridWidth, const UInt log2GridHeight, const UInt log2TUWidth, const UInt log2TUHeight)
          {
            grid        = gridSource;
            stride      = 1 << log2GridWidth;
            widthScale  = log2TUWidth  - log2GridWidth;
            heightScale = log2TUHeight - log2GridHeight;
          }
        }
        fixedGridContextParameters;

        //------------------

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
        struct NeighbourhoodContextParameters
        {
#if REMOVAL_8x2_2x8_CG
          UInt pattern00Context1Threshold;
          UInt pattern00Context2Threshold;
#else
          UInt pattern00ContextThreshold;
          UInt pattern11ContextThreshold;
#endif
        }
        neighbourhoodContextParameters;
#endif

        //------------------
};


struct TComDigest
{
  std::vector<unsigned char> hash;

  bool operator==(const TComDigest &other) const
  {
    if (other.hash.size() != hash.size()) return false;
    for(UInt i=0; i<UInt(hash.size()); i++)
      if (other.hash[i] != hash[i]) return false;
    return true;
  }

  bool operator!=(const TComDigest &other) const
  {
    return !(*this == other);
  }
};

//! \}

#endif


