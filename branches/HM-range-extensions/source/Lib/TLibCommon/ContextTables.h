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

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_NUM_CTX_MOD            1024       ///< maximum number of supported contexts

#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended

#define NUM_ALF_CTRL_FLAG_CTX         1       ///< number of context models for ALF control flag
#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_CU_AMP_CTX                1       ///< number of context models for partition size (AMP)
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#define NUM_ADI_CTX                   1       ///< number of context models for intra prediction

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)

#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction

#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference

#if REF_IDX_BYPASS
#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#else
#define NUM_REF_NO_CTX                4       ///< number of context models for reference index
#endif
#if TRANS_SPLIT_FLAG_CTX_REDUCTION
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#else
#define NUM_TRANS_SUBDIV_FLAG_CTX     10      ///< number of context models for transform subdivision flags
#endif
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE

//--------------------------------------------------------------------------------------------------

// context size definitions for significance map
#if REMOVAL_8x2_2x8_CG
#define NUM_SIG_FLAG_CTX_LUMA        27      ///< number of context models for luma sig flag
#else
#define NUM_SIG_FLAG_CTX_LUMA        24      ///< number of context models for luma sig flag
#endif

//------------------

#ifdef ECF__CHROMA_SIGNIFICANCE_MAP_CONTEXT_SAME_AS_LUMA

#define NUM_SIG_FLAG_CTX_CHROMA        NUM_SIG_FLAG_CTX_LUMA  ///< number of context models for chroma sig flag

//                                                                                                   |--Luma---|  |-Chroma--|
#if REMOVAL_8x2_2x8_CG
static const UInt significanceMapContextSetStart [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 21}, {0,  9, 21} };
static const UInt significanceMapContextSetSize  [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9, 12,  6}, {9, 12,  6} };
static const UInt nonDiagonalScan8x8ContextOffset[MAX_NUM_CHANNEL_TYPE]                          = {  6,           6          };
#else
static const UInt significanceMapContextSetStart [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 18}, {0,  9, 18} };
static const UInt significanceMapContextSetSize  [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9,  9,  6}, {9,  9,  6} };
#endif
static const UInt notFirstGroupNeighbourhoodContextOffset[MAX_NUM_CHANNEL_TYPE]                  = {  3,           3          };

#else

#if REMOVAL_8x2_2x8_CG

#define NUM_SIG_FLAG_CTX_CHROMA        15                     ///< number of context models for chroma sig flag

//                                                                                                   |--Luma---|  |-Chroma--|
static const UInt significanceMapContextSetStart [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 21}, {0,  9, 12} };
static const UInt significanceMapContextSetSize  [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9, 12,  6}, {9,  3,  3} };
static const UInt nonDiagonalScan8x8ContextOffset[MAX_NUM_CHANNEL_TYPE]                          = {  6,           0          };

#else

#define NUM_SIG_FLAG_CTX_CHROMA        21                     ///< number of context models for chroma sig flag

//                                                                                                   |--Luma---|  |-Chroma--|
static const UInt significanceMapContextSetStart [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 18}, {0,  9, 18} };
static const UInt significanceMapContextSetSize  [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9,  9,  6}, {9,  9,  3} };
#endif
static const UInt notFirstGroupNeighbourhoodContextOffset[MAX_NUM_CHANNEL_TYPE]                  = {  3,           0          };

#endif

//------------------

#if REMOVAL_8x2_2x8_CG

#define NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4  3
#define NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4  1

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
#define NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x8  4
#define NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x8  2
#endif

#else

#define NEIGHBOURHOOD_00_CONTEXT_THRESHOLD_4x4    6
#define NEIGHBOURHOOD_11_CONTEXT_THRESHOLD_4x4   13

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
#define NEIGHBOURHOOD_00_CONTEXT_THRESHOLD_4x8   10
#define NEIGHBOURHOOD_11_CONTEXT_THRESHOLD_4x8   26
#endif

#endif

//------------------

#define FIRST_SIG_FLAG_CTX_LUMA                   0

#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)

#define FIRST_SIG_FLAG_CTX_CB         (FIRST_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_LUMA)
#define FIRST_SIG_FLAG_CTX_CR         (FIRST_SIG_FLAG_CTX_CB + NUM_SIG_FLAG_CTX_CHROMA)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA + (2 * NUM_SIG_FLAG_CTX_CHROMA)) ///< number of context models for sig flag

#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)

#define FIRST_SIG_FLAG_CTX_CHROMA     (FIRST_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_LUMA)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_CHROMA)       ///< number of context models for sig flag

#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA)

#endif

//--------------------------------------------------------------------------------------------------

// context size definitions for last significant coefficient position

#define NUM_CTX_LAST_FLAG_SETS        (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION + 1)

#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position

//--------------------------------------------------------------------------------------------------

// context size definitions for greater-than-one and greater-than-two maps

#define NUM_ONE_FLAG_CTX_PER_SET       4      ///< number of context models for greater than 1 flag in a set
#define NUM_ABS_FLAG_CTX_PER_SET       1      ///< number of context models for greater than 2 flag in a set

//------------------

#define NUM_CTX_SETS_LUMA              4      ///< number of context model sets for luminance
#ifdef ECF__CHROMA_C1_C2_CONTEXT_SAME_AS_LUMA
#define NUM_CTX_SETS_CHROMA            4      ///< number of context model sets for combined chrominance
#else
#define NUM_CTX_SETS_CHROMA            2      ///< number of context model sets for combined chrominance
#endif

#define FIRST_CTX_SET_LUMA             0      ///< index of first luminance context set

//------------------

#define NUM_ONE_FLAG_CTX_LUMA         (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA       (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 1 flag of chroma

#define NUM_ABS_FLAG_CTX_LUMA         (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA       (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 2 flag of chroma

//------------------

#if   (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 2)

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA + (2 * NUM_ONE_FLAG_CTX_CHROMA))  ///< number of context models for greater than 1 flag
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA + (2 * NUM_ABS_FLAG_CTX_CHROMA))  ///< number of context models for greater than 2 flag

#define FIRST_CTX_SET_CB              (FIRST_CTX_SET_LUMA + NUM_CTX_SETS_LUMA)                 ///< index of first chrominance context set
#define FIRST_CTX_SET_CR              (FIRST_CTX_SET_CB + NUM_CTX_SETS_CHROMA)                 ///< index of first chrominance context set

#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 1)

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA + NUM_ONE_FLAG_CTX_CHROMA)        ///< number of context models for greater than 1 flag 
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA + NUM_ABS_FLAG_CTX_CHROMA)        ///< number of context models for greater than 2 flag

#define FIRST_CTX_SET_CHROMA          (FIRST_CTX_SET_LUMA + NUM_CTX_SETS_LUMA)                 ///< index of first chrominance context set

#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 0)

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA)                                  ///< number of context models for greater than 1 flag 
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA)                                  ///< number of context models for greater than 2 flag

#endif

//--------------------------------------------------------------------------------------------------

// context size definitions for CBF

#define NUM_QT_CBF_CTX_SETS           (ECF__CBF_CONTEXT_CHANNEL_SEPARATION + 1)

#define NUM_QT_CBF_CTX_PER_SET        5       ///< number of context models for QT CBF

#define FIRST_CBF_CTX_LUMA            0       ///< index of first luminance CBF context


#if   (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 2)

#define FIRST_CBF_CTX_CB              (FIRST_CBF_CTX_LUMA + NUM_QT_CBF_CTX_PER_SET)  ///< index of first Cb CBF context
#define FIRST_CBF_CTX_CR              (FIRST_CBF_CTX_CB   + NUM_QT_CBF_CTX_PER_SET)  ///< index of first Cr CBF context

#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 1)

#define FIRST_CBF_CTX_CHROMA          (FIRST_CBF_CTX_LUMA + NUM_QT_CBF_CTX_PER_SET)  ///< index of first chrominance CBF context

#endif

//--------------------------------------------------------------------------------------------------

#define NUM_MVP_IDX_CTX               2       ///< number of context models for MVP index

#define NUM_ALF_FLAG_CTX              1       ///< number of context models for ALF flag
#define NUM_ALF_UVLC_CTX              2       ///< number of context models for ALF UVLC (filter length)
#define NUM_ALF_SVLC_CTX              3       ///< number of context models for ALF SVLC (filter coeff.)

#if !SAO_ABS_BY_PASS
#define NUM_SAO_UVLC_CTX              2       ///< number of context models for SAO UVLC
#endif
#if SAO_MERGE_ONE_CTX
#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#else
#if SAO_SINGLE_MERGE
#define NUM_SAO_MERGE_LEFT_FLAG_CTX   1       ///< number of context models for SAO Merge-Left flag
#else
#define NUM_SAO_MERGE_LEFT_FLAG_CTX   3       ///< number of context models for AO SVLC (filter coeff.)
#endif
#define NUM_SAO_MERGE_UP_FLAG_CTX     1       ///< number of context models for AO SVLC (filter coeff.)
#endif
#if SAO_TYPE_CODING
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index
#else
#define NUM_SAO_TYPE_IDX_CTX          2       ///< number of context models for AO SVLC (filter coeff.)
#endif

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping 

#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1 

#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'

// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
INIT_CU_TRANSQUANT_BYPASS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
{
  { 154 }, 
  { 154 }, 
  { 154 }, 
};

// initial probability for split flag
static const UChar 
INIT_SPLIT_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SPLIT_FLAG_CTX] =
{
  { 107,  139,  126, },
  { 107,  139,  126, }, 
  { 139,  141,  157, }, 
};

static const UChar 
INIT_SKIP_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SKIP_FLAG_CTX] =
{
  { 197,  185,  201, }, 
  { 197,  185,  201, }, 
  { CNU,  CNU,  CNU, }, 
};

static const UChar 
INIT_ALF_CTRL_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ALF_CTRL_FLAG_CTX] =
{
  { 102, }, 
  { 102, }, 
  { 118, }, 
};

static const UChar 
INIT_MERGE_FLAG_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_FLAG_EXT_CTX] =
{
  { 154, }, 
  { 110, }, 
  { CNU, }, 
};

static const UChar 
INIT_MERGE_IDX_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_IDX_EXT_CTX] =
{
  { 137, }, 
  { 122, }, 
  { CNU, }, 
};

static const UChar 
INIT_PART_SIZE[NUMBER_OF_SLICE_TYPES][NUM_PART_SIZE_CTX] =
{
  { 154,  139,  CNU,  CNU, }, 
  { 154,  139,  CNU,  CNU, }, 
  { 184,  CNU,  CNU,  CNU, }, 
};

static const UChar 
INIT_CU_AMP_POS[NUMBER_OF_SLICE_TYPES][NUM_CU_AMP_CTX] =
{
  { 154, }, 
  { 154, }, 
  { CNU, }, 
};

static const UChar 
INIT_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_PRED_MODE_CTX] =
{
  { 134, }, 
  { 149, }, 
  { CNU, }, 
};

static const UChar 
INIT_INTRA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_ADI_CTX] =
{
  { 183, }, 
  { 154, }, 
  { 184, }, 
};

static const UChar 
INIT_CHROMA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_PRED_CTX] =
{
  { 152,  139, }, 
  { 152,  139, }, 
  {  63,  139, }, 
};

static const UChar 
INIT_INTER_DIR[NUMBER_OF_SLICE_TYPES][NUM_INTER_DIR_CTX] =
{
  {  95,   79,   63,   31,  31, }, 
  {  95,   79,   63,   31,  31, }, 
  { CNU,  CNU,  CNU,  CNU, CNU, }, 
};

static const UChar 
INIT_MVD[NUMBER_OF_SLICE_TYPES][NUM_MV_RES_CTX] =
{
  { 169,  198, }, 
  { 140,  198, }, 
  { CNU,  CNU, }, 
};

#if REF_IDX_BYPASS
static const UChar
INIT_REF_PIC[NUMBER_OF_SLICE_TYPES][NUM_REF_NO_CTX] =
{
  { 153,  153 }, 
  { 153,  153 }, 
  { CNU,  CNU }, 
};
#else
static const UChar 
INIT_REF_PIC[NUMBER_OF_SLICE_TYPES][NUM_REF_NO_CTX] =
{
  { 153,  153,  168,  CNU, }, 
  { 153,  153,  139,  CNU, }, 
  { CNU,  CNU,  CNU,  CNU, }, 
};
#endif

static const UChar 
INIT_DQP[NUMBER_OF_SLICE_TYPES][NUM_DELTA_QP_CTX] =
{
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
  { 154,  154,  154, }, 
};

//--------------------------------------------------------------------------------------------------

//Initialisation for CBF

//                                 |---------Luminance---------|
#define BSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define PSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define ISLICE_LUMA_CBF_CONTEXT     111,  141,  CNU,  CNU,  CNU
//                                 |--------Chrominance--------|
#define BSLICE_CHROMA_CBF_CONTEXT   149,   92,  167,  CNU,  CNU
#define PSLICE_CHROMA_CBF_CONTEXT   149,  107,  167,  CNU,  CNU
#define ISLICE_CHROMA_CBF_CONTEXT    94,  138,  182,  CNU,  CNU


static const UChar 
INIT_QT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_CBF_CTX_SETS * NUM_QT_CBF_CTX_PER_SET] =
{
#if   (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 2)
  { BSLICE_LUMA_CBF_CONTEXT, BSLICE_CHROMA_CBF_CONTEXT, BSLICE_CHROMA_CBF_CONTEXT },
  { PSLICE_LUMA_CBF_CONTEXT, PSLICE_CHROMA_CBF_CONTEXT, PSLICE_CHROMA_CBF_CONTEXT },
  { ISLICE_LUMA_CBF_CONTEXT, ISLICE_CHROMA_CBF_CONTEXT, ISLICE_CHROMA_CBF_CONTEXT },
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 1)
  { BSLICE_LUMA_CBF_CONTEXT, BSLICE_CHROMA_CBF_CONTEXT },
  { PSLICE_LUMA_CBF_CONTEXT, PSLICE_CHROMA_CBF_CONTEXT },
  { ISLICE_LUMA_CBF_CONTEXT, ISLICE_CHROMA_CBF_CONTEXT },
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 0)
  { BSLICE_LUMA_CBF_CONTEXT },
  { PSLICE_LUMA_CBF_CONTEXT },
  { ISLICE_LUMA_CBF_CONTEXT },
#endif
};


//--------------------------------------------------------------------------------------------------

static const UChar 
INIT_QT_ROOT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_ROOT_CBF_CTX] =
{
  {  79, }, 
  {  79, }, 
  { CNU, }, 
};


//--------------------------------------------------------------------------------------------------

//Initialisation for last-significant-position

//                                           |------------------------------Luminance----------------------------------|
#define BSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79
#define PSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94
#define ISLICE_LUMA_LAST_POSITION_CONTEXT     110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79
//                                           |------------------------------Chrominance--------------------------------|
#define BSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  93, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79
#define PSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123, 108, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94
#define ISLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  63, 110, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79


static const UChar 
INIT_LAST[NUMBER_OF_SLICE_TYPES][NUM_CTX_LAST_FLAG_SETS * NUM_CTX_LAST_FLAG_XY] =
{
#if   (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 2)
  { BSLICE_LUMA_LAST_POSITION_CONTEXT, BSLICE_CHROMA_LAST_POSITION_CONTEXT, BSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { PSLICE_LUMA_LAST_POSITION_CONTEXT, PSLICE_CHROMA_LAST_POSITION_CONTEXT, PSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { ISLICE_LUMA_LAST_POSITION_CONTEXT, ISLICE_CHROMA_LAST_POSITION_CONTEXT, ISLICE_CHROMA_LAST_POSITION_CONTEXT },
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 1)
  { BSLICE_LUMA_LAST_POSITION_CONTEXT, BSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { PSLICE_LUMA_LAST_POSITION_CONTEXT, PSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { ISLICE_LUMA_LAST_POSITION_CONTEXT, ISLICE_CHROMA_LAST_POSITION_CONTEXT },
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 0)
  { BSLICE_LUMA_LAST_POSITION_CONTEXT },
  { PSLICE_LUMA_LAST_POSITION_CONTEXT },
  { ISLICE_LUMA_LAST_POSITION_CONTEXT },
#endif
};


//--------------------------------------------------------------------------------------------------

static const UChar 
INIT_SIG_CG_FLAG[NUMBER_OF_SLICE_TYPES][2 * NUM_SIG_CG_FLAG_CTX] =
{
  { 121,  140,  
    61,  154, 
  }, 
  { 121,  140, 
    61,  154, 
  }, 
  {  91,  171,  
    134,  141, 
  }, 
};


//--------------------------------------------------------------------------------------------------

//Initialisation for significance map

#if REMOVAL_8x2_2x8_CG

  //                                          |-DC-|  |-----------------4x4------------------|  |------8x8 Diagonal Scan------|  |----8x8 Non-Diagonal Scan----|  |-NxN First group-|  |-NxN Other group-|
  //                                          |    |  |                                      |  |-First Group-| |-Other Group-|  |-First Group-| |-Other Group-|  |                 |  |                 |
  #define BSLICE_LUMA_SIGNIFICANCE_CONTEXT     170,    154, 139, 153, 139, 123, 123,  63, 124,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154
  #define PSLICE_LUMA_SIGNIFICANCE_CONTEXT     155,    154, 139, 153, 139, 123, 123,  63, 153,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154
  #define ISLICE_LUMA_SIGNIFICANCE_CONTEXT     111,    111, 125, 110, 110,  94, 124, 108, 124,   107, 125, 141,  179, 153, 125,   107, 125, 141,  179, 153, 125,   107,   125,   141,   179,   153,   125

  #if   (ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT == 1)
    //                                          |-DC-|  |-----------------4x4------------------|  |------8x8 Diagonal Scan------|  |----8x8 Non-Diagonal Scan----|  |-NxN First group-|  |-NxN Other group-|
    //                                          |    |  |                                      |  |-First Group-| |-Other Group-|  |-First Group-| |-Other Group-|  |                 |  |                 |
    #define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   151, 183, 140,  151, 183, 140,   151, 183, 140,  151, 183, 140,   151,   183,   140,   151,   183,   140
    #define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   151, 183, 140,  151, 183, 140,   151, 183, 140,  151, 183, 140,   151,   183,   140,   151,   183,   140
    #define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   136, 139, 111,  136, 139, 111,   136, 139, 111,  136, 139, 111,   136,   139,   111,   136,   139,   111
  #elif (ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT == 0)
    //                                          |-DC-|  |-----------------4x4------------------|  |-8x8 Any group-|  |-NxN Any group-|
    #define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   151,  183,  140,   151,  183,  140
    #define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   151,  183,  140,   151,  183,  140
    #define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   136,  139,  111,   136,  139,  111
  #endif

#else

  //                                          |-DC-|  |-----------------4x4------------------|  |---------------------8x8-------------------|  |-NxN First group-|  |-NxN Other group-|
  #define BSLICE_LUMA_SIGNIFICANCE_CONTEXT     170,    154, 139, 153, 139, 123, 123,  63, 124,   153, 153, 152, 152, 152, 137, 152, 137, 137,   166,   183,   140,   136,   153,   154
  #define PSLICE_LUMA_SIGNIFICANCE_CONTEXT     155,    154, 139, 153, 139, 123, 123,  63, 153,   153, 153, 152, 152, 152, 137, 152, 137, 122,   166,   183,   140,   136,   153,   154
  #define ISLICE_LUMA_SIGNIFICANCE_CONTEXT     111,    111, 125, 110, 110,  94, 124, 108, 124,   139, 139, 139, 168, 124, 138, 124, 138, 107,   107,   125,   141,   179,   153,   125

  #if   (ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT == 1)
    //                                          |-DC-|  |-----------------4x4------------------|  |---------------------8x8-------------------|  |-NxN First group-|  |-NxN Other group-|
    #define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   153, 167, 136, 121, 122, 136, 121, 122,  91,   151,   183,   140,   151,   183,   140
    #define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   153, 167, 136, 149, 107, 136, 121, 122,  91,   151,   183,   140,   151,   183,   140
    #define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   182, 137, 149, 192, 152, 224, 136,  31, 136,   136,   139,   111,   136,   139,   111
  #elif (ECF__EXTENDED_CHROMA_SIGNIFICANCE_MAP_CONTEXT == 0)
    //                                          |-DC-|  |-----------------4x4------------------|  |---------------------8x8-------------------|  |-NxN Any group-|
    #define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   153, 167, 136, 121, 122, 136, 121, 122,  91,   151,  183,  140
    #define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   153, 167, 136, 149, 107, 136, 121, 122,  91,   151,  183,  140
    #define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   182, 137, 149, 192, 152, 224, 136,  31, 136,   136,  139,  111
  #endif

#endif

//------------------------------------------------

static const UChar 
INIT_SIG_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SIG_FLAG_CTX] =
{
#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
  { BSLICE_LUMA_SIGNIFICANCE_CONTEXT, BSLICE_CHROMA_SIGNIFICANCE_CONTEXT, BSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { PSLICE_LUMA_SIGNIFICANCE_CONTEXT, PSLICE_CHROMA_SIGNIFICANCE_CONTEXT, PSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { ISLICE_LUMA_SIGNIFICANCE_CONTEXT, ISLICE_CHROMA_SIGNIFICANCE_CONTEXT, ISLICE_CHROMA_SIGNIFICANCE_CONTEXT },
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
  { BSLICE_LUMA_SIGNIFICANCE_CONTEXT, BSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { PSLICE_LUMA_SIGNIFICANCE_CONTEXT, PSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { ISLICE_LUMA_SIGNIFICANCE_CONTEXT, ISLICE_CHROMA_SIGNIFICANCE_CONTEXT },
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)
  { BSLICE_LUMA_SIGNIFICANCE_CONTEXT },
  { PSLICE_LUMA_SIGNIFICANCE_CONTEXT },
  { ISLICE_LUMA_SIGNIFICANCE_CONTEXT },
#endif
};


//--------------------------------------------------------------------------------------------------

//Initialisation for greater-than-one flags and greater-than-two flags

//                                 |------Set 0-------| |------Set 1-------| |------Set 2-------| |------Set 3-------|
#define BSLICE_LUMA_ONE_CONTEXT     154, 196, 167, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 122
#define PSLICE_LUMA_ONE_CONTEXT     154, 196, 196, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 137
#define ISLICE_LUMA_ONE_CONTEXT     140,  92, 137, 138,  140, 152, 138, 139,  153,  74, 149,  92,  139, 107, 122, 152

#define BSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 107
#define PSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 122
#define ISLICE_LUMA_ABS_CONTEXT     138,                 153,                 136,                 167

#if   (ECF__EXTENDED_CHROMA_C1_C2_CONTEXT == 1)
  //                                 |------Set 4-------| |------Set 5-------| |------Set 6-------| |------Set 7-------|
  #define BSLICE_CHROMA_ONE_CONTEXT   169, 208, 166, 167,  154, 152, 167, 182,  169, 208, 166, 167,  154, 152, 167, 182
  #define PSLICE_CHROMA_ONE_CONTEXT   169, 194, 166, 167,  154, 167, 137, 182,  169, 194, 166, 167,  154, 167, 137, 182
  #define ISLICE_CHROMA_ONE_CONTEXT   140, 179, 166, 182,  140, 227, 122, 197,  140, 179, 166, 182,  140, 227, 122, 197

  #define BSLICE_CHROMA_ABS_CONTEXT   107,                 167,                 107,                 167
  #define PSLICE_CHROMA_ABS_CONTEXT   107,                 167,                 107,                 167
  #define ISLICE_CHROMA_ABS_CONTEXT   152,                 152,                 152,                 152

#elif (ECF__EXTENDED_CHROMA_C1_C2_CONTEXT == 0)
  //                                 |------Set 4-------| |------Set 5-------|
  #define BSLICE_CHROMA_ONE_CONTEXT   169, 208, 166, 167,  154, 152, 167, 182
  #define PSLICE_CHROMA_ONE_CONTEXT   169, 194, 166, 167,  154, 167, 137, 182
  #define ISLICE_CHROMA_ONE_CONTEXT   140, 179, 166, 182,  140, 227, 122, 197

  #define BSLICE_CHROMA_ABS_CONTEXT   107,                 167
  #define PSLICE_CHROMA_ABS_CONTEXT   107,                 167
  #define ISLICE_CHROMA_ABS_CONTEXT   152,                 152

#endif

//------------------------------------------------

static const UChar 
INIT_ONE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ONE_FLAG_CTX] =
{
#if   (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 2)
  { BSLICE_LUMA_ONE_CONTEXT, BSLICE_CHROMA_ONE_CONTEXT, BSLICE_CHROMA_ONE_CONTEXT },
  { PSLICE_LUMA_ONE_CONTEXT, PSLICE_CHROMA_ONE_CONTEXT, PSLICE_CHROMA_ONE_CONTEXT },
  { ISLICE_LUMA_ONE_CONTEXT, ISLICE_CHROMA_ONE_CONTEXT, ISLICE_CHROMA_ONE_CONTEXT },
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 1)
  { BSLICE_LUMA_ONE_CONTEXT, BSLICE_CHROMA_ONE_CONTEXT },
  { PSLICE_LUMA_ONE_CONTEXT, PSLICE_CHROMA_ONE_CONTEXT },
  { ISLICE_LUMA_ONE_CONTEXT, ISLICE_CHROMA_ONE_CONTEXT },
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 0)
  { BSLICE_LUMA_ONE_CONTEXT },
  { PSLICE_LUMA_ONE_CONTEXT },
  { ISLICE_LUMA_ONE_CONTEXT },
#endif
};

static const UChar 
INIT_ABS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ABS_FLAG_CTX] =
{
#if   (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 2)
  { BSLICE_LUMA_ABS_CONTEXT, BSLICE_CHROMA_ABS_CONTEXT, BSLICE_CHROMA_ABS_CONTEXT },
  { PSLICE_LUMA_ABS_CONTEXT, PSLICE_CHROMA_ABS_CONTEXT, PSLICE_CHROMA_ABS_CONTEXT },
  { ISLICE_LUMA_ABS_CONTEXT, ISLICE_CHROMA_ABS_CONTEXT, ISLICE_CHROMA_ABS_CONTEXT },
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 1)
  { BSLICE_LUMA_ABS_CONTEXT, BSLICE_CHROMA_ABS_CONTEXT },
  { PSLICE_LUMA_ABS_CONTEXT, PSLICE_CHROMA_ABS_CONTEXT },
  { ISLICE_LUMA_ABS_CONTEXT, ISLICE_CHROMA_ABS_CONTEXT },
#elif (ECF__C1_C2_CONTEXT_CHANNEL_SEPARATION == 0)
  { BSLICE_LUMA_ABS_CONTEXT },
  { PSLICE_LUMA_ABS_CONTEXT },
  { ISLICE_LUMA_ABS_CONTEXT },
#endif
};


//--------------------------------------------------------------------------------------------------

static const UChar 
INIT_MVP_IDX[NUMBER_OF_SLICE_TYPES][NUM_MVP_IDX_CTX] =
{
  { 168,  CNU, }, 
  { 168,  CNU, }, 
  { CNU,  CNU, }, 
};

static const UChar 
INIT_ALF_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ALF_FLAG_CTX] =
{
  { 153, }, 
  { 153, }, 
  { 153, }, 
};

static const UChar 
INIT_ALF_UVLC[NUMBER_OF_SLICE_TYPES][NUM_ALF_UVLC_CTX] =
{
  { 154,  154, }, 
  { 154,  154, }, 
  { 140,  154, }, 
};

static const UChar 
INIT_ALF_SVLC[NUMBER_OF_SLICE_TYPES][NUM_ALF_SVLC_CTX] =
{
  { 141,  154,  159, }, 
  { 141,  154,  189, }, 
  { 187,  154,  159, }, 
};

#if !SAO_ABS_BY_PASS
static const UChar 
INIT_SAO_UVLC[NUMBER_OF_SLICE_TYPES][NUM_SAO_UVLC_CTX] =
{
  { 200,  140, }, 
  { 185,  140, }, 
  { 143,  140, }, 
};
#endif
#if SAO_MERGE_ONE_CTX
static const UChar 
INIT_SAO_MERGE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SAO_MERGE_FLAG_CTX] =
{
  { 153,  }, 
  { 153,  }, 
  { 153,  }, 
};
#else
static const UChar 
INIT_SAO_MERGE_LEFT_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SAO_MERGE_LEFT_FLAG_CTX] =
{
#if SAO_SINGLE_MERGE
  { 153, }, 
  { 153, }, 
  { 153, }, 
#else
  { 153,  153,  153, }, 
  { 153,  153,  153, }, 
  { 153,  153,  153, }, 
#endif
};

static const UChar 
INIT_SAO_MERGE_UP_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SAO_MERGE_UP_FLAG_CTX] =
{
  { 153, }, 
  { 153, }, 
  { 175, }, 
};
#endif

static const UChar 
INIT_SAO_TYPE_IDX[NUMBER_OF_SLICE_TYPES][NUM_SAO_TYPE_IDX_CTX] =
{
#if SAO_TYPE_CODING
  { 200, }, 
  { 185, }, 
  { 160, }, 
#else
  { 200,  140, }, 
  { 185,  140, }, 
  { 160,  140, }, 
#endif
};

#if TRANS_SPLIT_FLAG_CTX_REDUCTION
static const UChar
INIT_TRANS_SUBDIV_FLAG[NUMBER_OF_SLICE_TYPES][NUM_TRANS_SUBDIV_FLAG_CTX] =
{
  { 153,  138,  138, },
  { 124,  138,   94, },
  { 224,  167,  122, },
};
#else
static const UChar 
INIT_TRANS_SUBDIV_FLAG[NUMBER_OF_SLICE_TYPES][NUM_TRANS_SUBDIV_FLAG_CTX] =
{
{ CNU,  153,  138,  138,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
{ CNU,  124,  138,   94,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
{ CNU,  224,  167,  122,  CNU,  CNU,  CNU,  CNU,  CNU,  CNU, }, 
};
#endif

static const UChar
INIT_TRANSFORMSKIP_FLAG[NUMBER_OF_SLICE_TYPES][2*NUM_TRANSFORMSKIP_FLAG_CTX] =
{
  { 139,  139}, 
  { 139,  139}, 
  { 139,  139}, 
};

//! \}

#endif
