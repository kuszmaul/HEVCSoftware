/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
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

// #define DEBUG_STRING                 // enable to print out final decision debug info at encoder and decoder:
// #define DEBUG_ENCODER_SEARCH_BINS    // enable to print out each bin as it is coded during encoder search
// #define DEBUG_CABAC_BINS             // enable to print out each bin as it is coded during final encode and decode
// #define DEBUG_INTRA_SEARCH_COSTS     // enable to print out the cost for each mode during encoder search
// #define DEBUG_TRANSFORM_AND_QUANTISE // enable to print out each TU as it passes through the transform-quantise-dequantise-inverseTransform process

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

#define L0363_DU_BIT_RATE                                 1 ///< L0363: add bit_rate_du_value_minus1 to HRD parameters
#define L0328_SPLICING                                    1 ///< L0328: splicing support in HRD
#define L0044_DU_DPB_OUTPUT_DELAY_HRD                     1 ///< L0044: Include dpb_output_delay_du_length_minus1 in hrd_parameters(), dpb_output_du_delay in
                                                            ///<        picture timing SEI and DU information SEI
#define L0045_PERSISTENCE_FLAGS                           1      ///< L0045: Replace "repetition_period" syntax elements in SEI with "persistence_flag"
#define L0045_NON_NESTED_SEI_RESTRICTIONS                 1 ///< L0045; Include restriction on the order of APS and non-nested BP, PT and DU info SEI messages
#define L0044_CPB_DPB_DELAY_OFFSET                        1  ///< L0044: Include syntax elements cpb_delay_offset and dpb_delay_offset in the BP SEI message
#define L0047_APS_FLAGS                                   1  ///< L0047: Include full_random_access_flag and no_param_set_update_flag in the active parameter set SEI message
#define L0043_TIMING_INFO                                 1  ///< L0043: Timing information is signalled in VUI outside hrd_parameters()
#define L0046_RENAME_PROG_SRC_IDC                         1  ///< L0046: Rename progressive_source_idc to source_scan_type
#define L0045_CONDITION_SIGNALLING                        1  ///< L0045: Condition the signaling of some syntax elements in picture timing SEI message
#define L0043_MSS_IDC                                     1
#define L0116_ENTRY_POINT                                 1
#define L0363_MORE_BITS                                   1
#define L0363_MVP_POC                                     1
#define L0363_BYTE_ALIGN                                  1
#define L0363_SEI_ALLOW_SUFFIX                            1
#define L0323_LIMIT_DEFAULT_LIST_SIZE                     1
#define L0046_CONSTRAINT_FLAGS                            1
#define L0255_MOVE_PPS_FLAGS                              1  ///< move some flags to earlier positions in the PPS
#define L0444_FPA_TYPE                                    1  ///< allow only FPA types 3, 4 and 5
#define L0372                                             1
#define SIGNAL_BITRATE_PICRATE_IN_VPS                     0  ///< K0125: Signal bit_rate and pic_rate in VPS
#define L0232_RD_PENALTY                                  1  ///< L0232: RD-penalty for 32x32 TU for intra in non-intra slices

#define MAX_VPS_NUM_HRD_PARAMETERS                        1
#define MAX_VPS_OP_SETS_PLUS1                          1024
#define MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1          1

#define RATE_CONTROL_LAMBDA_DOMAIN                        1  ///< JCTVC-K0103, rate control by R-lambda model
#define L0033_RC_BUGFIX                                   1  ///< JCTVC-L0033, bug fix for R-lambda model based rate control

#define MAXIMUM_INTRA_FILTERED_WIDTH                     16
#define MAXIMUM_INTRA_FILTERED_HEIGHT                    16

#define MAX_CPB_CNT                                      32 ///< Upper bound of (cpb_cnt_minus1 + 1)
#define MAX_NUM_LAYER_IDS                                64

#define COEF_REMAIN_BIN_REDUCTION                         3 ///< indicates the level at which the VLC
                                                            ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_CMAX                                    5 ///< max number bins for truncated unary
#define CU_DQP_EG_k                                       0 ///< expgolomb order

#define SBH_THRESHOLD                                     4  ///< I0156: value of the fixed SBH controlling threshold

#define SEQUENCE_LEVEL_LOSSLESS                           0  ///< H0530: used only for sequence or frame-level lossless coding

#define DISABLING_CLIP_FOR_BIPREDME                       1  ///< Ticket #175

#define C1FLAG_NUMBER                                     8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER                                     1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5

#define REMOVE_SAO_LCU_ENC_CONSTRAINTS_3                  1  ///< disable the encoder constraint that conditionally disable SAO for chroma for entire slice in interleaved mode

#define REMOVE_SINGLE_SEI_EXTENSION_FLAGS                 1 ///< remove display orientation SEI extension flag (there is a generic SEI extension mechanism now) 

#define SAO_ENCODING_CHOICE                               1  ///< I0184: picture early termination
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                                 0.75
#define SAO_ENCODING_CHOICE_CHROMA                        1 ///< J0044: picture early termination Luma and Chroma are handled separately
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA                          0.5
#endif
#endif

#define MAX_NUM_SAO_OFFSETS                               4

#define MAX_NUM_VPS                                      16
#define MAX_NUM_SPS                                      16
#define MAX_NUM_PPS                                      64

#define WEIGHTED_CHROMA_DISTORTION                        1   ///< F386: weighting of chroma for RDO
#define RDOQ_CHROMA_LAMBDA                                1   ///< F386: weighting of chroma for RDOQ
#define SAO_CHROMA_LAMBDA                                 1   ///< F386: weighting of chroma for SAO

#define MIN_SCAN_POS_CROSS                                4

#define FAST_BIT_EST                                      1   ///< G763: Table-based bit estimation for CABAC

#define MLS_GRP_NUM                                      64     ///< G644 : Max number of coefficient groups, max(16, 64)
#define MLS_CG_LOG2_WIDTH                                 2
#define MLS_CG_LOG2_HEIGHT                                2
#define MLS_CG_SIZE                                     (MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT)  ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION                             1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                                   7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                                      30     ///< G382: max coefficient level in statistics collection
#endif

#define NS_HAD                                            0

#define HHI_RQT_INTRA_SPEEDUP                             1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD                         0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR                            4

#define SCAN_SET_SIZE                                    16
#define LOG2_SCAN_SET_SIZE                                4

#define FAST_UDI_MAX_RDMODE_NUM                          35          ///< maximum number of RD comparison in fast-UDI estimation loop

#define ZERO_MVD_EST                                      0           ///< Zero Mvd Estimation in normal mode

#define NUM_INTRA_MODE                                   36

#define WRITE_BACK                                        1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                                    1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                                    0           ///< Enable/disable the printing of bits used to send the RPS.
                                                                        // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                                        // this should be done with encoder only decision
                                                                        // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX                                        0
#define VER_IDX                                          26                    // index for intra VERTICAL   mode
#define HOR_IDX                                          10                    // index for intra HORIZONTAL mode
#define DC_IDX                                            1                    // index for intra DC mode
#define NUM_CHROMA_MODE                                   5                    // total number of chroma modes
#define DM_CHROMA_IDX                                    36                    // chroma mode index for derived from luma intra mode
#define INVALID_MODE_IDX                                 (NUM_INTRA_MODE+1)    // value used to indicate an invalid intra mode
#define STOPCHROMASEARCH_MODE_IDX                        (INVALID_MODE_IDX+1)  // value used to signal the end of a chroma mode search

#define MDCS_MODE                       MDCS_BOTH_DIRECTIONS        ///< Name taken from definition of MDCSMode enumeration below
#define MDCS_ANGLE_LIMIT                                  4         ///< (default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
#define MDCS_MAXIMUM_WIDTH                                8         ///< (default 8) (measured in pixels) TUs with width greater than this can only use diagonal scan
#define MDCS_MAXIMUM_HEIGHT                               8         ///< (default 8) (measured in pixels) TUs with height greater than this can only use diagonal scan

#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS                              0           ///< Disable counting dQP bits in RDO-based mode decision

#define FULL_NBIT                                         0           ///< When enabled, does not use g_uiBitIncrement anymore to support > 8 bit data
#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x)  0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#define LOG2_MAX_NUM_COLUMNS_MINUS1                       7
#define LOG2_MAX_NUM_ROWS_MINUS1                          7
#define LOG2_MAX_COLUMN_WIDTH                            13
#define LOG2_MAX_ROW_HEIGHT                              13

#define MATRIX_MULT                                       0 // Brute force matrix multiplication instead of partial butterfly

#define REG_DCT 65535

#define AMP_SAD                                           1 ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                                   1 ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                                           1 ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define SCALING_LIST_OUTPUT_RESULT                        0 //JCTVC-G880/JCTVC-G1016 quantization matrices

#define CABAC_INIT_PRESENT_FLAG                           1

#define LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS    4 // NOTE: RExt - new definition
#define CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS  8 // NOTE: RExt - new definition


// ====================================================================================================================
// RExt control settings
// ====================================================================================================================

//------------------------------------------------
// Enable environment variables
//------------------------------------------------

#define RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST                              0 ///< When enabled, allows control of RExt modifications via environment variables
#define RExt__PRINT_MACRO_VALUES                                               1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

//------------------------------------------------
// Processing controls
//------------------------------------------------

#define RExt__COLOUR_SPACE_CONVERSIONS                                         1 ///< 0 = disable colour space conversion, 1 (default) = enable colour space conversions for the source, reconstruction and decoded images

#define RExt__SQUARE_TRANSFORM_CHROMA_422                                      1 ///< 0 = allow rectangular transforms for chroma 4:2:2, 1 (default) = split rectangular TUs into square sub-TUs prior to prediction for intra and transform for inter

#define RExt__INCREASE_NUMBER_OF_SCALING_LISTS_FOR_CHROMA                      0 ///< 0 (default) = Chroma shares the Luma 32x32 ScalingList (ensures compatibility with existing scaling list definition files). 1 = Chroma channels have their own 32x32 ScalingList

// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                                     0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

//------------------------------------------------
// Backwards-compatibility
//------------------------------------------------

#define RExt__BACKWARDS_COMPATIBILITY_HM_TRANSQUANTBYPASS                      0 ///< Maintain backwards compatibility with HM's transquant lossless encoding methods

// NOTE: RExt - Compatibility defaults chosen so that simulations run with the common test conditions do not differ with HM.
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_986                            1 ///< Maintain backwards compatibility with HM for ticket 986 (encodeQtCbfZero called with inconsistent depths)
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_987                            0 ///< Maintain backwards compatibility with HM for ticket 987 (SAO mixing quadtree indices and components)
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_990                            0 ///< Maintain backwards compatibility with HM for ticket 990 (RDOQ_CHROMA_LAMBDA interaction with TComTrQuant)
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_990_SAO                        1 ///< Maintain backwards compatibility with HM for ticket 990 (RDOQ_CHROMA_LAMBDA interaction with TComTrQuant - SAO interaction subclause). Ticket 993 compatibility must disabled.
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_992                            0 ///< Maintain backwards compatibility with HM for ticket 992 (MAX_CU_SIZE)
#define RExt__BACKWARDS_COMPATIBILITY_HM_TICKET_993                            0 ///< Maintain backwards compatibility with HM for ticket 993 (TEncSAO m_dLambdaLuma/Chroma member variables not required)

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
typedef       float               Float;


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

#if RExt__COLOUR_SPACE_CONVERSIONS
enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};
#endif

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

static const UInt NUM_REF_PIC_LIST_01  = 2; // NOTE: RExt - new definition
static const UInt NUM_REF_PIC_LIST_01C = 3; // NOTE: RExt - new definition

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

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
  SCAN_HOR  = 1,        ///< horizontal first scan
  SCAN_VER  = 2,        ///< vertical first scan
  SCAN_NUMBER_OF_TYPES = 3
};

enum COEFF_SCAN_GROUP_TYPE
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum SignificanceMapContextType
{
  CONTEXT_TYPE_4x4 = 0,
  CONTEXT_TYPE_8x8 = 1,
  CONTEXT_TYPE_NxN = 2,
  CONTEXT_NUMBER_OF_TYPES = 3
};

enum ScalingListSize
{
  SCALING_LIST_4x4 = 0,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_SIZE_NUM
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

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_LCU    = 1,          ///< Limit maximum number of largest coding tree blocks in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
};

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

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
  };
}


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

typedef struct _SaoQTPart
{
  Int         iBestType;
  Int         iLength;
  Int         subTypeIdx;                 ///< indicates EO class or BO band position
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
  Int        subTypeIdx;                  ///< indicates EO class or BO band position
  Int        offset[MAX_NUM_SAO_OFFSETS];
  Int        partIdx;
  Int        partIdxTmp;
  Int        length;
} SaoLcuParam;

struct SAOParam
{
  Bool         bSaoFlag[MAX_NUM_CHANNEL_TYPE];
  SAOQTPart*   psSaoPart[MAX_NUM_COMPONENT];
  Int          iMaxSplitLevel;
  Bool         oneUnitFlag[MAX_NUM_COMPONENT];
  SaoLcuParam* saoLcuParam[MAX_NUM_COMPONENT];
  Int          numCuInHeight;
  Int          numCuInWidth;
  ~SAOParam();
};


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
        UInt             widthInGroups;
        UInt             heightInGroups;
        UInt             firstSignificanceMapContext;
};


struct TComDigest
{
  std::vector<UChar> hash;

  Bool operator==(const TComDigest &other) const
  {
    if (other.hash.size() != hash.size()) return false;
    for(UInt i=0; i<UInt(hash.size()); i++)
      if (other.hash[i] != hash[i]) return false;
    return true;
  }

  Bool operator!=(const TComDigest &other) const
  {
    return !(*this == other);
  }
};

//! \}

#endif


