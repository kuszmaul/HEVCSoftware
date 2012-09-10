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

/** \file     TEncSbac.cpp
    \brief    SBAC encoder class
*/

#include "TEncTop.h"
#include "TEncSbac.h"
#include "TLibCommon/TComTU.h"

#include <map>
#include <algorithm>

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
#endif


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSbac::TEncSbac()
// new structure here
: m_pcBitIf                      ( NULL )
, m_pcSlice                      ( NULL )
, m_pcBinIf                      ( NULL )
, m_uiCoeffCost                  ( 0 )
, m_numContextModels             ( 0 )
, m_cCUSplitFlagSCModel          ( 1,             1,                      NUM_SPLIT_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSkipFlagSCModel           ( 1,             1,                      NUM_SKIP_FLAG_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeFlagExtSCModel       ( 1,             1,                      NUM_MERGE_FLAG_EXT_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel        ( 1,             1,                      NUM_MERGE_IDX_EXT_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPartSizeSCModel           ( 1,             1,                      NUM_PART_SIZE_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel           ( 1,             1,                      NUM_PRED_MODE_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAlfCtrlFlagSCModel        ( 1,             1,                      NUM_ALF_CTRL_FLAG_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUIntraPredSCModel          ( 1,             1,                      NUM_ADI_CTX                      , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUChromaPredSCModel         ( 1,             1,                      NUM_CHROMA_PRED_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUDeltaQpSCModel            ( 1,             1,                      NUM_DELTA_QP_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUInterDirSCModel           ( 1,             1,                      NUM_INTER_DIR_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCURefPicSCModel             ( 1,             1,                      NUM_REF_NO_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMvdSCModel                ( 1,             1,                      NUM_MV_RES_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtCbfSCModel              ( 1,             NUM_QT_CBF_CTX_SETS,    NUM_QT_CBF_CTX_PER_SET           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUTransSubdivFlagSCModel    ( 1,             1,                      NUM_TRANS_SUBDIV_FLAG_CTX        , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtRootCbfSCModel          ( 1,             1,                      NUM_QT_ROOT_CBF_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigCoeffGroupSCModel      ( 1,             2,                      NUM_SIG_CG_FLAG_CTX              , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigSCModel                ( 1,             1,                      NUM_SIG_FLAG_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastX                  ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastY                  ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUOneSCModel                ( 1,             1,                      NUM_ONE_FLAG_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAbsSCModel                ( 1,             1,                      NUM_ABS_FLAG_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cMVPIdxSCModel               ( 1,             1,                      NUM_MVP_IDX_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFFlagSCModel              ( 1,             1,                      NUM_ALF_FLAG_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFUvlcSCModel              ( 1,             1,                      NUM_ALF_UVLC_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cALFSvlcSCModel              ( 1,             1,                      NUM_ALF_SVLC_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAMPSCModel                ( 1,             1,                      NUM_CU_AMP_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
#if !SAO_ABS_BY_PASS
, m_cSaoUvlcSCModel              ( 1,             1,                      NUM_SAO_UVLC_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
#endif
#if SAO_MERGE_ONE_CTX
, m_cSaoMergeSCModel             ( 1,             1,                      NUM_SAO_MERGE_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
#else
, m_cSaoMergeLeftSCModel         ( 1,             1,                      NUM_SAO_MERGE_LEFT_FLAG_CTX      , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeUpSCModel           ( 1,             1,                      NUM_SAO_MERGE_UP_FLAG_CTX        , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_cSaoTypeIdxSCModel           ( 1,             1,                      NUM_SAO_TYPE_IDX_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel        ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_TRANSFORMSKIP_FLAG_CTX       , m_contextModels + m_numContextModels, m_numContextModels)
, m_CUTransquantBypassFlagSCModel( 1,             1,                      NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX, m_contextModels + m_numContextModels, m_numContextModels)
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
#if !REMOVE_FGS
  m_iSliceGranularity = 0;
#endif
}

TEncSbac::~TEncSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSbac::resetEntropy           ()
{
  Int  iQp              = m_pcSlice->getSliceQp();
  SliceType eSliceType  = m_pcSlice->getSliceType();

  Int  encCABACTableIdx = m_pcSlice->getPPS()->getEncCABACTableIdx();
  if (!m_pcSlice->isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && m_pcSlice->getPPS()->getCabacInitPresentFlag())
  {
    eSliceType = (SliceType) encCABACTableIdx;
  }
  
  m_cCUSplitFlagSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SKIP_FLAG );
  m_cCUAlfCtrlFlagSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_ALF_CTRL_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_MERGE_FLAG_EXT);
  m_cCUMergeIdxExtSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_MERGE_IDX_EXT);
  m_cCUPartSizeSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_PART_SIZE );
  m_cCUAMPSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_CU_AMP_POS );
  m_cCUPredModeSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer            ( eSliceType, iQp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                  ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                  ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer               ( eSliceType, iQp, (UChar*)INIT_MVP_IDX );
  m_cALFFlagSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_FLAG );
  m_cALFUvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_UVLC );
  m_cALFSvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_SVLC );
  m_cCUTransSubdivFlagSCModel.initBuffer    ( eSliceType, iQp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
#if !SAO_ABS_BY_PASS
  m_cSaoUvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_SAO_UVLC );
#endif
#if SAO_MERGE_ONE_CTX
  m_cSaoMergeSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_FLAG );
#else
  m_cSaoMergeLeftSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_LEFT_FLAG );
  m_cSaoMergeUpSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_UP_FLAG );
#endif
  m_cSaoTypeIdxSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cTransformSkipSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer( eSliceType, iQp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );

  // new structure
  m_uiLastQp = iQp;
  
  m_pcBinIf->start();
  
  return;
}

/** The function does the following:
 * If current slice type is P/B then it determines the distance of initialisation type 1 and 2 from the current CABAC states and
 * stores the index of the closest table.  This index is used for the next P/B slice when cabac_init_present_flag is true.
 */
Void TEncSbac::determineCabacInitIdx()
{
  Int  qp              = m_pcSlice->getSliceQp();

  if (!m_pcSlice->isIntra())
  {
    SliceType aSliceTypeChoices[] = {B_SLICE, P_SLICE};

    UInt bestCost             = MAX_UINT;
    SliceType bestSliceType   = aSliceTypeChoices[0];
    for (UInt idx=0; idx<2; idx++)
    {
      UInt curCost          = 0;
      SliceType curSliceType  = aSliceTypeChoices[idx];

      curCost  = m_cCUSplitFlagSCModel.calcCost          ( curSliceType, qp, (UChar*)INIT_SPLIT_FLAG );
      curCost += m_cCUSkipFlagSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_SKIP_FLAG );
      curCost += m_cCUAlfCtrlFlagSCModel.calcCost        ( curSliceType, qp, (UChar*)INIT_ALF_CTRL_FLAG );
      curCost += m_cCUMergeFlagExtSCModel.calcCost       ( curSliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT);
      curCost += m_cCUMergeIdxExtSCModel.calcCost        ( curSliceType, qp, (UChar*)INIT_MERGE_IDX_EXT);
      curCost += m_cCUPartSizeSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_PART_SIZE );
      curCost += m_cCUAMPSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_CU_AMP_POS );
      curCost += m_cCUPredModeSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_PRED_MODE );
      curCost += m_cCUIntraPredSCModel.calcCost          ( curSliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
      curCost += m_cCUChromaPredSCModel.calcCost         ( curSliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
      curCost += m_cCUInterDirSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_INTER_DIR );
      curCost += m_cCUMvdSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_MVD );
      curCost += m_cCURefPicSCModel.calcCost             ( curSliceType, qp, (UChar*)INIT_REF_PIC );
      curCost += m_cCUDeltaQpSCModel.calcCost            ( curSliceType, qp, (UChar*)INIT_DQP );
      curCost += m_cCUQtCbfSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_QT_CBF );
      curCost += m_cCUQtRootCbfSCModel.calcCost          ( curSliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
      curCost += m_cCUSigCoeffGroupSCModel.calcCost      ( curSliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
      curCost += m_cCUSigSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_SIG_FLAG );
      curCost += m_cCuCtxLastX.calcCost                  ( curSliceType, qp, (UChar*)INIT_LAST );
      curCost += m_cCuCtxLastY.calcCost                  ( curSliceType, qp, (UChar*)INIT_LAST );
      curCost += m_cCUOneSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_ONE_FLAG );
      curCost += m_cCUAbsSCModel.calcCost                ( curSliceType, qp, (UChar*)INIT_ABS_FLAG );
      curCost += m_cMVPIdxSCModel.calcCost               ( curSliceType, qp, (UChar*)INIT_MVP_IDX );
      curCost += m_cALFFlagSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_ALF_FLAG );
      curCost += m_cALFUvlcSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_ALF_UVLC );
      curCost += m_cALFSvlcSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_ALF_SVLC );
      curCost += m_cCUTransSubdivFlagSCModel.calcCost    ( curSliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
#if !SAO_ABS_BY_PASS
      curCost += m_cSaoUvlcSCModel.calcCost              ( curSliceType, qp, (UChar*)INIT_SAO_UVLC );
#endif
#if SAO_MERGE_ONE_CTX
      curCost += m_cSaoMergeSCModel.calcCost             ( curSliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
#else
      curCost += m_cSaoMergeLeftSCModel.calcCost         ( curSliceType, qp, (UChar*)INIT_SAO_MERGE_LEFT_FLAG );
      curCost += m_cSaoMergeUpSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_SAO_MERGE_UP_FLAG );
#endif
      curCost += m_cSaoTypeIdxSCModel.calcCost           ( curSliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );
      curCost += m_cTransformSkipSCModel.calcCost        ( curSliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
      curCost += m_CUTransquantBypassFlagSCModel.calcCost( curSliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );

      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    m_pcSlice->getPPS()->setEncCABACTableIdx( bestSliceType );
  }
  else
  {
    m_pcSlice->getPPS()->setEncCABACTableIdx( I_SLICE );
  }
}


/** The function does the followng: Write out terminate bit. Flush CABAC. Intialize CABAC states. Start CABAC.
 */
Void TEncSbac::updateContextTables( SliceType eSliceType, Int iQp, Bool bExecuteFinish )
{
  m_pcBinIf->encodeBinTrm(1);
  if (bExecuteFinish) m_pcBinIf->finish();
  m_cCUSplitFlagSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SKIP_FLAG );
  m_cCUAlfCtrlFlagSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_ALF_CTRL_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_MERGE_FLAG_EXT);
  m_cCUMergeIdxExtSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_MERGE_IDX_EXT);
  m_cCUPartSizeSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_PART_SIZE );
  m_cCUAMPSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_CU_AMP_POS );
  m_cCUPredModeSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer            ( eSliceType, iQp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                  ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                  ( eSliceType, iQp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                ( eSliceType, iQp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer               ( eSliceType, iQp, (UChar*)INIT_MVP_IDX );
  m_cALFFlagSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_FLAG );
  m_cALFUvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_UVLC );
  m_cALFSvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_ALF_SVLC );
  m_cCUTransSubdivFlagSCModel.initBuffer    ( eSliceType, iQp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
#if !SAO_ABS_BY_PASS
  m_cSaoUvlcSCModel.initBuffer              ( eSliceType, iQp, (UChar*)INIT_SAO_UVLC );
#endif
#if SAO_MERGE_ONE_CTX
  m_cSaoMergeSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_FLAG );
#else
  m_cSaoMergeLeftSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_LEFT_FLAG );
  m_cSaoMergeUpSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_UP_FLAG );
#endif
  m_cSaoTypeIdxSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cTransformSkipSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer( eSliceType, iQp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );

  m_pcBinIf->start();
}

void TEncSbac::codeSEI(const SEI&)
{
  assert(0);
}

Void TEncSbac::codeVPS( TComVPS* pcVPS )
{
  assert (0);
  return;
}

Void TEncSbac::codeSPS( TComSPS* pcSPS )
{
  assert (0);
  return;
}

Void TEncSbac::codePPS( TComPPS* pcPPS )
{
  assert (0);
  return;
}

Void TEncSbac::codeSliceHeader( TComSlice* pcSlice )
{
  assert (0);
  return;
}

Void TEncSbac::codeTilesWPPEntryPoint( TComSlice* pSlice )
{
  assert (0);
  return;
}

Void TEncSbac::codeTerminatingBit( UInt uilsLast )
{
  m_pcBinIf->encodeBinTrm( uilsLast );
}

Void TEncSbac::codeSliceFinish()
{
  m_pcBinIf->finish();
}

Void TEncSbac::codeFlush()
{
  m_pcBinIf->flush();
}

Void TEncSbac::encodeStart()
{
  m_pcBinIf->start();
}

Void TEncSbac::xWriteUnarySymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset )
{
  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[0] );
  
  if( 0 == uiSymbol)
  {
    return;
  }
  
  while( uiSymbol-- )
  {
    m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ iOffset ] );
  }
  
  return;
}

Void TEncSbac::xWriteUnaryMaxSymbol( UInt uiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
{
  if (uiMaxSymbol == 0)
  {
    return;
  }
  
  m_pcBinIf->encodeBin( uiSymbol ? 1 : 0, pcSCModel[ 0 ] );
  
  if ( uiSymbol == 0 )
  {
    return;
  }
  
  Bool bCodeLast = ( uiMaxSymbol > uiSymbol );
  
  while( --uiSymbol )
  {
    m_pcBinIf->encodeBin( 1, pcSCModel[ iOffset ] );
  }
  if( bCodeLast )
  {
    m_pcBinIf->encodeBin( 0, pcSCModel[ iOffset ] );
  }
  
  return;
}

Void TEncSbac::xWriteEpExGolomb( UInt uiSymbol, UInt uiCount )
{
  UInt bins = 0;
  Int numBins = 0;
  
  while( uiSymbol >= (UInt)(1<<uiCount) )
  {
    bins = 2 * bins + 1;
    numBins++;
    uiSymbol -= 1 << uiCount;
    uiCount  ++;
  }
  bins = 2 * bins + 0;
  numBins++;
  
  bins = (bins << uiCount) | uiSymbol;
  numBins += uiCount;
  
  assert( numBins <= 32 );
  m_pcBinIf->encodeBinsEP( bins, numBins );
}


/** Coding of coeff_abs_level_minus3
 * \param uiSymbol value of coeff_abs_level_minus3
 * \param ruiGoRiceParam reference to Rice parameter
 * \returns Void
 */
Void TEncSbac::xWriteCoefRemainExGolomb ( UInt symbol, UInt &rParam )
{
  Int codeNumber  = (Int)symbol;
  UInt length;
#if COEF_REMAIN_BIN_REDUCTION
  if (codeNumber < (COEF_REMAIN_BIN_REDUCTION << rParam))
#else
  if (codeNumber < (8 << rParam))
#endif
  {
    length = codeNumber>>rParam;
    m_pcBinIf->encodeBinsEP( (1<<(length+1))-2 , length+1);
    m_pcBinIf->encodeBinsEP((codeNumber%(1<<rParam)),rParam);
  }
  else
  {
    length = rParam;
#if COEF_REMAIN_BIN_REDUCTION
    codeNumber  = codeNumber - ( COEF_REMAIN_BIN_REDUCTION << rParam);
#else
    codeNumber  = codeNumber - ( 8 << rParam);
#endif 
    while (codeNumber >= (1<<length))
    {
      codeNumber -=  (1<<(length++));    
    }
#if COEF_REMAIN_BIN_REDUCTION
    m_pcBinIf->encodeBinsEP((1<<(COEF_REMAIN_BIN_REDUCTION+length+1-rParam))-2,COEF_REMAIN_BIN_REDUCTION+length+1-rParam);
#else
    m_pcBinIf->encodeBinsEP((1<<(8+length+1-rParam))-2,8+length+1-rParam);
#endif
    m_pcBinIf->encodeBinsEP(codeNumber,length);
  }
}

// SBAC RD
Void  TEncSbac::load ( TEncSbac* pSrc)
{
  this->xCopyFrom(pSrc);
}

Void  TEncSbac::loadIntraDirMode( TEncSbac* pSrc, const ChannelType chType )
{
  m_pcBinIf->copyState( pSrc->m_pcBinIf );
  if (isLuma(chType))
    this->m_cCUIntraPredSCModel      .copyFrom( &pSrc->m_cCUIntraPredSCModel       );
  else
    this->m_cCUChromaPredSCModel     .copyFrom( &pSrc->m_cCUChromaPredSCModel      );
}


Void  TEncSbac::store( TEncSbac* pDest)
{
  pDest->xCopyFrom( this );
}


Void TEncSbac::xCopyFrom( TEncSbac* pSrc )
{
  m_pcBinIf->copyState( pSrc->m_pcBinIf );
  
  this->m_uiCoeffCost = pSrc->m_uiCoeffCost;
  this->m_uiLastQp    = pSrc->m_uiLastQp;
  
  memcpy( m_contextModels, pSrc->m_contextModels, m_numContextModels * sizeof( ContextModel ) );
}

Void TEncSbac::codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iSymbol = pcCU->getMVPIdx(eRefList, uiAbsPartIdx);
  Int iNum = AMVP_MAX_NUM_CANDS;

  xWriteUnaryMaxSymbol(iSymbol, m_cMVPIdxSCModel.get(0), 1, iNum-1);
}

Void TEncSbac::codePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  PartSize eSize         = pcCU->getPartitionSize( uiAbsPartIdx );
  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      m_pcBinIf->encodeBin( eSize == SIZE_2Nx2N? 1 : 0, m_cCUPartSizeSCModel.get( 0, 0, 0 ) );
    }
    return;
  }
  
  switch(eSize)
  {
    case SIZE_2Nx2N:
    {
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      break;
    }
    case SIZE_2NxN:
    case SIZE_2NxnU:
    case SIZE_2NxnD:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 1) );
      if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
      {
        if (eSize == SIZE_2NxN)
        {
          m_pcBinIf->encodeBin(1, m_cCUAMPSCModel.get( 0, 0, 0 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUAMPSCModel.get( 0, 0, 0 ));          
          m_pcBinIf->encodeBinEP((eSize == SIZE_2NxnU? 0: 1));
        }
      }
      break;
    }
    case SIZE_Nx2N:
    case SIZE_nLx2N:
    case SIZE_nRx2N:
    {
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
      m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );

      if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && !( pcCU->getWidth(uiAbsPartIdx) == 8 && pcCU->getHeight(uiAbsPartIdx) == 8 ) )
      {
        m_pcBinIf->encodeBin( 1, m_cCUPartSizeSCModel.get( 0, 0, 2) );
      }

      if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
      {
        if (eSize == SIZE_Nx2N)
        {
          m_pcBinIf->encodeBin(1, m_cCUAMPSCModel.get( 0, 0, 0 ));
        }
        else
        {
          m_pcBinIf->encodeBin(0, m_cCUAMPSCModel.get( 0, 0, 0 ));
          m_pcBinIf->encodeBinEP((eSize == SIZE_nLx2N? 0: 1));
        }
      }
      break;
    }
    case SIZE_NxN:
    {
      if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && !( pcCU->getWidth(uiAbsPartIdx) == 8 && pcCU->getHeight(uiAbsPartIdx) == 8 ) )
      {
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 0) );
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 1) );
        m_pcBinIf->encodeBin( 0, m_cCUPartSizeSCModel.get( 0, 0, 2) );
      }
      break;
    }
    default:
    {
      assert(0);
      break;
    }
  }
}

/** code prediction mode
 * \param pcCU
 * \param uiAbsPartIdx  
 * \returns Void
 */
Void TEncSbac::codePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  Int iPredMode = pcCU->getPredictionMode( uiAbsPartIdx );
  m_pcBinIf->encodeBin( iPredMode == MODE_INTER ? 0 : 1, m_cCUPredModeSCModel.get( 0, 0, 0 ) );
}

Void TEncSbac::codeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiSymbol = pcCU->getCUTransquantBypass(uiAbsPartIdx);
  m_pcBinIf->encodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) );
}

/** code skip flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \returns Void
 */
Void TEncSbac::codeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // get context function is here
  UInt uiSymbol = pcCU->isSkipped( uiAbsPartIdx ) ? 1 : 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx ) ;
  m_pcBinIf->encodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");
}

/** code merge flag
 * \param pcCU
 * \param uiAbsPartIdx 
 * \returns Void
 */
Void TEncSbac::codeMergeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  const UInt uiSymbol = pcCU->getMergeFlag( uiAbsPartIdx ) ? 1 : 0;
  m_pcBinIf->encodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

/** code merge index
 * \param pcCU
 * \param uiAbsPartIdx 
 * \returns Void
 */
Void TEncSbac::codeMergeIndex( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiNumCand = MRG_MAX_NUM_CANDS;
  UInt uiUnaryIdx = pcCU->getMergeIndex( uiAbsPartIdx );
  uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( UInt ui = 0; ui < uiNumCand - 1; ++ui )
    {
      const UInt uiSymbol = ui == uiUnaryIdx ? 0 : 1;
      if ( ui==0 )
      {
        m_pcBinIf->encodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) );
      }
      else
      {
        m_pcBinIf->encodeBinEP( uiSymbol );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tparseMergeIndex()" );
  DTRACE_CABAC_T( "\tuiMRGIdx= " );
  DTRACE_CABAC_V( pcCU->getMergeIndex( uiAbsPartIdx ) );
  DTRACE_CABAC_T( "\n" );
}

Void TEncSbac::codeSplitFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
    return;
  
  UInt uiCtx           = pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth );
  UInt uiCurrSplitFlag = ( pcCU->getDepth( uiAbsPartIdx ) > uiDepth ) ? 1 : 0;
  
  assert( uiCtx < 3 );
  m_pcBinIf->encodeBin( uiCurrSplitFlag, m_cCUSplitFlagSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  return;
}

Void TEncSbac::codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx )
{
  m_pcBinIf->encodeBin( uiSymbol, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\n" )
}


Void TEncSbac::codeIntraDirLumaAng( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiple)
{
  UInt dir[4],j;
  Int preds[4][NUM_MOST_PROBABLE_MODES] = {{-1, -1, -1},{-1, -1, -1},{-1, -1, -1},{-1, -1, -1}};
  Int predNum[4], predIdx[4] ={ -1,-1,-1,-1};
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = isMultiple?(mode==SIZE_NxN?4:1):1;
  UInt partOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  for (j=0;j<partNum;j++)
  {
    dir[j] = pcCU->getIntraDir( CHANNEL_TYPE_LUMA, absPartIdx+partOffset*j );
    predNum[j] = pcCU->getIntraDirPredictor(absPartIdx+partOffset*j, preds[j], COMPONENT_Y);
    for(UInt i = 0; i < predNum[j]; i++)
    {
      if(dir[j] == preds[j][i])
      {
        predIdx[j] = i;
      }
    }
    m_pcBinIf->encodeBin((predIdx[j] != -1)? 1 : 0, m_cCUIntraPredSCModel.get( 0, 0, 0 ) );
  }  
  for (j=0;j<partNum;j++)
  {
    if(predIdx[j] != -1)
    {
      m_pcBinIf->encodeBinEP( predIdx[j] ? 1 : 0 );
      if (predIdx[j])
      {
        m_pcBinIf->encodeBinEP( predIdx[j]-1 );
      }
    }
    else
    {
      assert(predNum[j]>=3); // It is currently always 3!
      if (preds[j][0] > preds[j][1])
      { 
        std::swap(preds[j][0], preds[j][1]); 
      }
      if (preds[j][0] > preds[j][2])
      {
        std::swap(preds[j][0], preds[j][2]);
      }
      if (preds[j][1] > preds[j][2])
      {
        std::swap(preds[j][1], preds[j][2]);
      }
      for(Int i = (predNum[j] - 1); i >= 0; i--)
      {
        dir[j] = dir[j] > preds[j][i] ? dir[j] - 1 : dir[j];
      }
      m_pcBinIf->encodeBinsEP( dir[j], 5 );
    }
  }
  return;
}

Void TEncSbac::codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiIntraDirChroma = pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx );

  if (!reducedIntraChromaModes())
  {
    if( uiIntraDirChroma == DM_CHROMA_IDX ) 
    {
      m_pcBinIf->encodeBin( 0, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );
    }
#if !REMOVE_LMCHROMA
    else if( uiIntraDirChroma == LM_CHROMA_IDX )
    {
      m_pcBinIf->encodeBin( 1, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );
      m_pcBinIf->encodeBin( 0, m_cCUChromaPredSCModel.get( 0, 0, 1 ) );
    }
#endif
    else
    { 
      m_pcBinIf->encodeBin( 1, m_cCUChromaPredSCModel.get( 0, 0, 0 ) );

#if !REMOVE_LMCHROMA
      if (pcCU->getSlice()->getSPS()->getUseLMChroma())
      {
        m_pcBinIf->encodeBin( 1, m_cCUChromaPredSCModel.get( 0, 0, 1 ));
      }
#endif

      UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
      pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );

#if REMOVE_LMCHROMA
      for( Int i = 0; i < NUM_CHROMA_MODE - 1; i++ )
#else
      for( Int i = 0; i < NUM_CHROMA_MODE - 2; i++ )
#endif
      {
        if( uiIntraDirChroma == uiAllowedChromaDir[i] )
        {
          uiIntraDirChroma = i;
          break;
        }
      }

      m_pcBinIf->encodeBinsEP( uiIntraDirChroma, 2 );
    }
  }
#if !REMOVE_LMCHROMA
  else
  {
    if (pcCU->getSlice()->getSPS()->getUseLMChroma())
      m_pcBinIf->encodeBin( ((uiIntraDirChroma == LM_CHROMA_IDX) ? 1 : 0), m_cCUChromaPredSCModel.get( 0, 0, 0 ) );
  }
#endif

  return;
}

Void TEncSbac::codeInterDir( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  const UInt uiInterDir = pcCU->getInterDir( uiAbsPartIdx ) - 1;
  const UInt uiCtx      = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx    = m_cCUInterDirSCModel.get( 0 );

#if DISALLOW_BIPRED_IN_8x4_4x8PUS
  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
  {
#endif
    m_pcBinIf->encodeBin( uiInterDir == 2 ? 1 : 0, *( pCtx + uiCtx ) );
#if DISALLOW_BIPRED_IN_8x4_4x8PUS
  }
#endif

  if (uiInterDir < 2)
  {
    m_pcBinIf->encodeBin( uiInterDir, *( pCtx + 4 ) );
  }

  return;
}

Void TEncSbac::codeRefFrmIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  Int iRefFrame = pcCU->getCUMvField( eRefList )->getRefIdx( uiAbsPartIdx );
  ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
  m_pcBinIf->encodeBin( ( iRefFrame == 0 ? 0 : 1 ), *pCtx );
    
  if( iRefFrame > 0 )
  {
#if REF_IDX_BYPASS
    UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
    pCtx++;
    iRefFrame--;
    for( UInt ui = 0; ui < uiRefNum; ++ui )
    {
      const UInt uiSymbol = ui == iRefFrame ? 0 : 1;
      if( ui == 0 )
      {
        m_pcBinIf->encodeBin( uiSymbol, *pCtx );       
      }
      else
      {
        m_pcBinIf->encodeBinEP( uiSymbol );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
#else
    xWriteUnaryMaxSymbol( iRefFrame - 1, pCtx + 1, 1, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );
#endif
  }
  return;
}

Void TEncSbac::codeMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    return;
  }

  const TComCUMvField* pcCUMvField = pcCU->getCUMvField( eRefList );
  const Int iHor = pcCUMvField->getMvd( uiAbsPartIdx ).getHor();
  const Int iVer = pcCUMvField->getMvd( uiAbsPartIdx ).getVer();
  ContextModel* pCtx = m_cCUMvdSCModel.get( 0 );

  m_pcBinIf->encodeBin( iHor != 0 ? 1 : 0, *pCtx );
  m_pcBinIf->encodeBin( iVer != 0 ? 1 : 0, *pCtx );

  const Bool bHorAbsGr0 = iHor != 0;
  const Bool bVerAbsGr0 = iVer != 0;
  const UInt uiHorAbs   = 0 > iHor ? -iHor : iHor;
  const UInt uiVerAbs   = 0 > iVer ? -iVer : iVer;
  pCtx++;

  if( bHorAbsGr0 )
  {
    m_pcBinIf->encodeBin( uiHorAbs > 1 ? 1 : 0, *pCtx );
  }

  if( bVerAbsGr0 )
  {
    m_pcBinIf->encodeBin( uiVerAbs > 1 ? 1 : 0, *pCtx );
  }

  if( bHorAbsGr0 )
  {
    if( uiHorAbs > 1 )
    {
      xWriteEpExGolomb( uiHorAbs-2, 1 );
    }

    m_pcBinIf->encodeBinEP( 0 > iHor ? 1 : 0 );
  }

  if( bVerAbsGr0 )
  {
    if( uiVerAbs > 1 )
    {
      xWriteEpExGolomb( uiVerAbs-2, 1 );
    }

    m_pcBinIf->encodeBinEP( 0 > iVer ? 1 : 0 );
  }
  
  return;
}

Void TEncSbac::codeDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  Int iDQp  = pcCU->getQP( uiAbsPartIdx ) - pcCU->getRefQP( uiAbsPartIdx );
  
  Int qpBdOffsetY =  pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  iDQp = (iDQp + 78 + qpBdOffsetY + (qpBdOffsetY/2)) % (52 + qpBdOffsetY) - 26 - (qpBdOffsetY/2);

#if CU_DQP_TU_EG
  UInt uiAbsDQp = (UInt)((iDQp > 0)? iDQp  : (-iDQp));
  UInt TUValue = min((Int)uiAbsDQp, CU_DQP_TU_CMAX);
  xWriteUnaryMaxSymbol( TUValue, &m_cCUDeltaQpSCModel.get( 0, 0, 0 ), 1, CU_DQP_TU_CMAX);
  if( uiAbsDQp >= CU_DQP_TU_CMAX )
  {
    xWriteEpExGolomb( uiAbsDQp - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }

  if ( uiAbsDQp > 0)
  {
    UInt uiSign = (iDQp > 0 ? 0 : 1);
    m_pcBinIf->encodeBinEP(uiSign);
  }
#else
  if ( iDQp == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cCUDeltaQpSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cCUDeltaQpSCModel.get( 0, 0, 0 ) );
   
    UInt uiSign = (iDQp > 0 ? 0 : 1);

    m_pcBinIf->encodeBinEP(uiSign);

    assert(iDQp >= -(26+(qpBdOffsetY/2)));
    assert(iDQp <=  (25+(qpBdOffsetY/2)));

    UInt uiMaxAbsDQpMinus1 = 24 + (qpBdOffsetY/2) + (uiSign);
    UInt uiAbsDQpMinus1 = (UInt)((iDQp > 0)? iDQp  : (-iDQp)) - 1;
    xWriteUnaryMaxSymbol( uiAbsDQpMinus1, &m_cCUDeltaQpSCModel.get( 0, 0, 1 ), 1, uiMaxAbsDQpMinus1);
  }
#endif
  
  return;
}


Void TEncSbac::codeQtCbf( TComTU &rTu, const ComponentID compID )
{
  TComDataCU* pcCU = rTu.getCU();

  UInt uiCbf = pcCU->getCbf     ( rTu.GetAbsPartIdxTU(), compID, rTu.GetTransformDepthRel() );
  UInt uiCtx = pcCU->getCtxQtCbf( rTu, toChannelType(compID), false );

#if   (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 2)
  const UInt contextSet = compID;
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 1)
  const UInt contextSet = toChannelType(compID);
#elif (ECF__CBF_CONTEXT_CHANNEL_SEPARATION == 0)
  const UInt contextSet = CHANNEL_TYPE_LUMA;
#endif

  m_pcBinIf->encodeBin( uiCbf , m_cCUQtCbfSCModel.get( 0, contextSet, uiCtx ) );


  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiCbf )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( compID )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( rTu.GetAbsPartIdxTU(compID) )
  DTRACE_CABAC_T( "\n" )
}


void TEncSbac::codeTransformSkipFlags (TComTU &rTu, ComponentID component )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }

#if !INTER_TRANSFORMSKIP
  if(!pcCU->isIntra(uiAbsPartIdx))
  {
    return;
  }
#endif


  if (!TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(component)))
  {
    return;
  }

  if (isChroma(component) && singleTransformSkipFlag(rTu.GetChromaFormat()))
  {
    return;
  }

  UInt useTansformSkip = pcCU->getTransformSkip( uiAbsPartIdx,component);
  m_pcBinIf->encodeBin( useTansformSkip, m_cTransformSkipSCModel.get( 0, toChannelType(component), 0 ) );

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T("\tparseTransformSkip()");
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( useTansformSkip )
  DTRACE_CABAC_T( "\tAddr=" )
  DTRACE_CABAC_V( pcCU->getAddr() )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( component )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
}


/** Code I_PCM information. 
 * \param pcCU pointer to CU
 * \param uiAbsPartIdx CU index
 * \param numIPCM the number of succesive IPCM blocks with the same size 
 * \param firstIPCMFlag 
 * \returns Void
 */
Void TEncSbac::codeIPCMInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, Int numIPCM, Bool firstIPCMFlag)
{
  UInt uiIPCM = (pcCU->getIPCMFlag(uiAbsPartIdx) == true)? 1 : 0;

  Bool writePCMSampleFlag = pcCU->getIPCMFlag(uiAbsPartIdx);

  if( uiIPCM == 0 || firstIPCMFlag)
  {
    m_pcBinIf->encodeBinTrm (uiIPCM);

    if ( firstIPCMFlag )
    {
      m_pcBinIf->encodeNumSubseqIPCM( numIPCM - 1 );
      m_pcBinIf->encodePCMAlignBits();
    }
  }

  if (writePCMSampleFlag)
  {
    const UInt minCoeffSizeY = pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getMinCUHeight();
    const UInt offsetY       = minCoeffSizeY * uiAbsPartIdx;
    for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compID = ComponentID(ch);
      const UInt offset = offsetY >> (pcCU->getPic()->getComponentScaleX(compID) + pcCU->getPic()->getComponentScaleY(compID));
      Pel * pPCMSample  = pcCU->getPCMSample(compID) + offset;
      const UInt width  = pcCU->getWidth (uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleX(compID);
      const UInt height = pcCU->getHeight(uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleY(compID);
      const UInt sampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
      for (UInt y=0; y<height; y++)
      {
        for (UInt x=0; x<width; x++)
        {
          UInt sample = pPCMSample[x];
          m_pcBinIf->xWritePCMCode(sample, sampleBits);
        }
        pPCMSample += width;
      }
    }

    numIPCM--;
    if(numIPCM == 0)
    {
      m_pcBinIf->resetBac();
    }
  }
}

Void TEncSbac::codeQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiCbf = pcCU->getQtRootCbf( uiAbsPartIdx );
  UInt uiCtx = 0;
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiCbf )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )
}

#if TU_ZERO_CBF_RDO
Void TEncSbac::codeQtCbfZero( TComTU & rTu, const ChannelType chType, const Bool useAdjustedDepth )
{
  // this function is only used to estimate the bits when cbf is 0
  // and will never be called when writing the bistream. do not need to write log
  UInt uiCbf = 0;
  UInt uiCtx = rTu.getCU()->getCtxQtCbf( rTu, chType, useAdjustedDepth );
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtCbfSCModel.get( 0, chType, uiCtx ) );
}

Void TEncSbac::codeQtRootCbfZero( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // this function is only used to estimate the bits when cbf is 0
  // and will never be called when writing the bistream. do not need to write log
  UInt uiCbf = 0;
  UInt uiCtx = 0;
  m_pcBinIf->encodeBin( uiCbf , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) );
}
#endif

/** Encode (X,Y) position of the last significant coefficient
 * \param uiPosX X component of last coefficient
 * \param uiPosY Y component of last coefficient
 * \param width  Block width
 * \param height Block height
 * \param eTType plane type / luminance or chrominance
 * \param uiScanIdx scan type (zig-zag, hor, ver)
 * This method encodes the X and Y component within a block of the last significant coefficient.
 */
Void TEncSbac::codeLastSignificantXY( UInt uiPosX, UInt uiPosY, Int width, Int height, ComponentID component, UInt uiScanIdx )
{  
  // swap
  if( uiScanIdx == SCAN_VER )
  {
    swap( uiPosX, uiPosY );
    swap( width,  height );
  }

  UInt uiCtxLast;
  UInt uiGroupIdxX    = g_uiGroupIdx[ uiPosX ];
  UInt uiGroupIdxY    = g_uiGroupIdx[ uiPosY ];

#if   (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 2)
  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, component );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, component );
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 1)
  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, toChannelType(component) );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, toChannelType(component) );
#elif (ECF__LAST_POSITION_CONTEXT_CHANNEL_SEPARATION == 0)
  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, CHANNEL_TYPE_LUMA );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, CHANNEL_TYPE_LUMA );
#endif

  Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
  getLastSignificantContextParameters(component, width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

  //------------------

  // posX

  for( uiCtxLast = 0; uiCtxLast < uiGroupIdxX; uiCtxLast++ )
  {
    m_pcBinIf->encodeBin( 1, *( pCtxX + blkSizeOffsetX + (uiCtxLast >>shiftX) ) );
  }
  if( uiGroupIdxX < g_uiGroupIdx[ width - 1 ])
  {
    m_pcBinIf->encodeBin( 0, *( pCtxX + blkSizeOffsetX + (uiCtxLast >>shiftX) ) );
  }

  // posY

  for( uiCtxLast = 0; uiCtxLast < uiGroupIdxY; uiCtxLast++ )
  {
    m_pcBinIf->encodeBin( 1, *( pCtxY + blkSizeOffsetY + (uiCtxLast >>shiftY) ) );
  }
  if( uiGroupIdxY < g_uiGroupIdx[ height - 1 ])
  {
    m_pcBinIf->encodeBin( 0, *( pCtxY + blkSizeOffsetY + (uiCtxLast >>shiftY) ) );
  }

  // EP-coded part

  if ( uiGroupIdxX > 3 )
  {      
    UInt uiCount = ( uiGroupIdxX - 2 ) >> 1;
    uiPosX       = uiPosX - g_uiMinInGroup[ uiGroupIdxX ];
    for (Int i = uiCount - 1 ; i >= 0; i-- )
    {
      m_pcBinIf->encodeBinEP( ( uiPosX >> i ) & 1 );
    }
  }
  if ( uiGroupIdxY > 3 )
  {      
    UInt uiCount = ( uiGroupIdxY - 2 ) >> 1;
    uiPosY       = uiPosY - g_uiMinInGroup[ uiGroupIdxY ];
    for ( Int i = uiCount - 1 ; i >= 0; i-- )
    {
      m_pcBinIf->encodeBinEP( ( uiPosY >> i ) & 1 );
    }
  }
}


Void TEncSbac::codeCoeffNxN( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  const TComRectangle &tuRect=rTu.getRect(compID);
  const UInt uiWidth=tuRect.width;
  const UInt uiHeight=tuRect.height;

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
  DTRACE_CABAC_V( compID )
  DTRACE_CABAC_T( "\twidth=" )
  DTRACE_CABAC_V( uiWidth )
  DTRACE_CABAC_T( "\theight=" )
  DTRACE_CABAC_V( uiHeight )
  DTRACE_CABAC_T( "\tdepth=" )
// NOTE: ECF - the following commented debug lines have been changed to make consistent with HM
//  DTRACE_CABAC_V( rTu.GetTransformDepthTotalAdj(compID) )
  DTRACE_CABAC_V( rTu.GetTransformDepthTotal() )
  DTRACE_CABAC_T( "\tabspartidx=" )
//  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\ttoCU-X=" )
  DTRACE_CABAC_V( pcCU->getCUPelX() )
  DTRACE_CABAC_T( "\ttoCU-Y=" )
  DTRACE_CABAC_V( pcCU->getCUPelY() )
  DTRACE_CABAC_T( "\tCU-addr=" )
  DTRACE_CABAC_V(  pcCU->getAddr() )
  DTRACE_CABAC_T( "\tinCU-X=" )
//  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_V( g_auiRasterToPelX[ g_auiZscanToRaster[rTu.GetAbsPartIdxTU(compID)] ] )
  DTRACE_CABAC_T( "\tinCU-Y=" )
// DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ] )
  DTRACE_CABAC_V( g_auiRasterToPelY[ g_auiZscanToRaster[rTu.GetAbsPartIdxTU(compID)] ] )
  DTRACE_CABAC_T( "\tpredmode=" )
  DTRACE_CABAC_V(  pcCU->getPredictionMode( uiAbsPartIdx ) )
  DTRACE_CABAC_T( "\n" )

  //--------------------------------------------------------------------------------------------------

  if( uiWidth > m_pcSlice->getSPS()->getMaxTrSize() )
  {
    std::cerr << "ERROR: codeCoeffNxN was passed a TU with dimensions larger than the maximum allowed size" << std::endl;
    assert(false);
    exit(1);
  }
  
  // compute number of significant coefficients
  UInt uiNumSig = TEncEntropy::countNonZeroCoeffs(pcCoef, uiWidth * uiHeight);
  
  if ( uiNumSig == 0 ) return;

  //--------------------------------------------------------------------------------------------------

  //set parameters

  const ChannelType  chType            = toChannelType(compID);
  const UInt         uiLog2BlockWidth  = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt         uiLog2BlockHeight = g_aucConvertToBit[ uiHeight ] + 2;

  Bool beValid; 
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    beValid = false;
  }
  else 
  {
    beValid = pcCU->getSlice()->getPPS()->getSignHideFlag() > 0;
  }
  
  //select scans
  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, uiHeight, compID), uiWidth, uiHeight, compID, rTu.GetChromaFormat());

  //--------------------------------------------------------------------------------------------------

#if PPS_TS_FLAG
  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
#else
  if(pcCU->getSlice()->getSPS()->getUseTransformSkip())
#endif
  {
    codeTransformSkipFlags(rTu, compID);
  }

  //--------------------------------------------------------------------------------------------------

  //----- encode significance map -----

  // Find position of last coefficient
  Int scanPosLast = -1;
  Int posLast;


  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];

  memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );
  do
  {
    posLast = codingParameters.scan[ ++scanPosLast ];

    if( pcCoef[ posLast ] != 0 )
    {
      // get L1 sig map
      UInt uiPosY   = posLast >> uiLog2BlockWidth;
      UInt uiPosX   = posLast - ( uiPosY << uiLog2BlockWidth );

      UInt uiBlkIdx = (codingParameters.widthInGroups * (uiPosY >> codingParameters.log2GroupHeight)) + (uiPosX >> codingParameters.log2GroupWidth);
      uiSigCoeffGroupFlag[ uiBlkIdx ] = 1;

      uiNumSig--;
    }
  }
  while ( uiNumSig > 0 );

  // Code position of last coefficient
  Int posLastY = posLast >> uiLog2BlockWidth;
  Int posLastX = posLast - ( posLastY << uiLog2BlockWidth );
  codeLastSignificantXY(posLastX, posLastY, uiWidth, uiHeight, compID, codingParameters.scanType);

  //===== code significance flag =====
  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, chType );
  ContextModel * const baseCtx = m_cCUSigSCModel.get( 0, 0 ) + getSignificanceMapContextOffset(compID);

  const UInt log2GroupSize = codingParameters.log2GroupWidth + codingParameters.log2GroupHeight;
  const Int  iLastScanSet  = scanPosLast >> log2GroupSize;

#if !REMOVE_NUM_GREATER1
  UInt uiNumOne            = 0;
#endif
  UInt c1                  = 1;
  UInt uiGoRiceParam       = 0;
  Int  iScanPosSig         = scanPosLast;

  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int numNonZero  = 0;
    Int  iSubPos    = iSubSet << log2GroupSize;
    uiGoRiceParam   = 0;
    UInt coeffSigns = 0;

#ifdef ECF__EXTENDED_SIZE_COEFFICIENT_GROUPS
    Int absCoeff[2 << MLS_CG_SIZE];
#else
    Int absCoeff[1 << MLS_CG_SIZE];
#endif

    Int lastNZPosInCG  = -1;
    Int firstNZPosInCG = 1 << log2GroupSize;

    if( iScanPosSig == scanPosLast )
    {
      absCoeff[ 0 ] = abs( pcCoef[ posLast ] );
      coeffSigns    = ( pcCoef[ posLast ] < 0 );
      numNonZero    = 1;
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
    }

    // encode significant_coeffgroup_flag
    Int iCGBlkPos = codingParameters.scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / codingParameters.widthInGroups;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * codingParameters.widthInGroups);

    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
    else
    {
      UInt uiSigCoeffGroup   = (uiSigCoeffGroupFlag[ iCGBlkPos ] != 0);
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );
      m_pcBinIf->encodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] );
    }
      
    // encode significant_coeff_flag
    if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
    {
      Int patternSigCtx = 0;
      if (!codingParameters.useFixedGridSignificanceMapContext)
      {
        patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);
      }

      UInt uiBlkPos, uiSig, uiCtxSig;
      for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
      {
        uiBlkPos  = codingParameters.scan[ iScanPosSig ]; 
        uiSig     = (pcCoef[ uiBlkPos ] != 0);
        if( iScanPosSig > iSubPos || iSubSet == 0 || numNonZero )
        {
          uiCtxSig  = TComTrQuant::getSigCtxInc( patternSigCtx, codingParameters, iScanPosSig, uiLog2BlockWidth, uiLog2BlockHeight, chType );
          m_pcBinIf->encodeBin( uiSig, baseCtx[ uiCtxSig ] );
        }
        if( uiSig )
        {
          absCoeff[ numNonZero ] = abs( pcCoef[ uiBlkPos ] );
          coeffSigns = 2 * coeffSigns + ( pcCoef[ uiBlkPos ] < 0 );
          numNonZero++;
          if( lastNZPosInCG == -1 )
          {
            lastNZPosInCG = iScanPosSig;
          }
          firstNZPosInCG = iScanPosSig;
        }
      }
    }
    else
    {
      iScanPosSig = iSubPos - 1;
    }

    if( numNonZero > 0 )
    {
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );

#if REMOVE_NUM_GREATER1
      const UInt uiCtxSet = getContextSetIndex(compID, iSubSet, (c1 == 0));
#else
      const UInt uiCtxSet = getContextSetIndex(compID, iSubSet, (uiNumOne > 0));
      uiNumOne >>= 1;
#endif
      c1 = 1;
      
      ContextModel *baseCtxMod = m_cCUOneSCModel.get( 0, 0 ) + (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet);
      
      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;
      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        UInt uiSymbol = absCoeff[ idx ] > 1;
        m_pcBinIf->encodeBin( uiSymbol, baseCtxMod[c1] );
        if( uiSymbol )
        {
          c1 = 0;

          if (firstC2FlagIdx == -1)
          {
            firstC2FlagIdx = idx;
          }
        }
        else if( (c1 < 3) && (c1 > 0) )
        {
          c1++;
        }
      }
      
      if (c1 == 0)
      {
        baseCtxMod = m_cCUAbsSCModel.get( 0, 0 ) + (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet);
        if ( firstC2FlagIdx != -1)
        {
          UInt symbol = absCoeff[ firstC2FlagIdx ] > 2;
          m_pcBinIf->encodeBin( symbol, baseCtxMod[0] );
        }
      }
      if( beValid && signHidden )
      {
        m_pcBinIf->encodeBinsEP( (coeffSigns >> 1), numNonZero-1 );
      }
      else
      {
        m_pcBinIf->encodeBinsEP( coeffSigns, numNonZero );
      }
      
      Int iFirstCoeff2 = 1;    
      if (c1 == 0 || numNonZero > C1FLAG_NUMBER)
      {
        for ( Int idx = 0; idx < numNonZero; idx++ )
        {
          UInt baseLevel  = (idx < C1FLAG_NUMBER)? (2 + iFirstCoeff2 ) : 1;

          if( absCoeff[ idx ] >= baseLevel)
          {
            xWriteCoefRemainExGolomb( absCoeff[ idx ] - baseLevel, uiGoRiceParam );

            if(absCoeff[idx] > 3*(1<<uiGoRiceParam))
            {
               uiGoRiceParam = min<UInt>(uiGoRiceParam+ 1, 4);
            }
          }
          if(absCoeff[ idx ] >= 2)  
          {
            iFirstCoeff2 = 0;
#if !REMOVE_NUM_GREATER1
            uiNumOne++;
#endif
          }
        }        
      }
    }
#if !REMOVE_NUM_GREATER1
    else
    {
      uiNumOne >>= 1;
    }
#endif
  }
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  printSBACCoeffData(posLastX, posLastY, uiWidth, uiHeight, compID, uiAbsPartIdx, codingParameters.scanType, pcCoef, g_bFinalEncode);
#endif

  return;
}

#if !REMOVE_ALF
Void TEncSbac::codeAlfCtrlFlag( ComponentID component, UInt code )
{
  m_pcBinIf->encodeBin( code, m_cCUAlfCtrlFlagSCModel.get( 0, 0, 0) );

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "codeAlfCtrlFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( code )
  DTRACE_CABAC_T( "\tcompIdx=" )
  DTRACE_CABAC_V( component )
  DTRACE_CABAC_T( "\n" )
}
#endif

/** code SAO offset sign
 * \param code sign value
 */
Void TEncSbac::codeSAOSign( UInt code )
{
  m_pcBinIf->encodeBinEP( code );
}

Void TEncSbac::codeSaoMaxUvlc    ( UInt code, UInt maxSymbol )
{
  if (maxSymbol == 0)
  {
    return;
  }

  Int i;
  Bool bCodeLast = ( maxSymbol > code );

  if ( code == 0 )
  {
#if SAO_ABS_BY_PASS
    m_pcBinIf->encodeBinEP( 0 );
#else
    m_pcBinIf->encodeBin( 0, m_cSaoUvlcSCModel.get( 0, 0, 0 ) );
#endif
  }
  else
  {
#if SAO_ABS_BY_PASS
    m_pcBinIf->encodeBinEP( 1 );
#else
    m_pcBinIf->encodeBin( 1, m_cSaoUvlcSCModel.get( 0, 0, 0 ) );
#endif
    for ( i=0; i<code-1; i++ )
    {
#if SAO_ABS_BY_PASS
      m_pcBinIf->encodeBinEP( 1 );
#else
      m_pcBinIf->encodeBin( 1, m_cSaoUvlcSCModel.get( 0, 0, 1 ) );
#endif
    }
    if( bCodeLast )
    {
#if SAO_ABS_BY_PASS
      m_pcBinIf->encodeBinEP( 0 );
#else
      m_pcBinIf->encodeBin( 0, m_cSaoUvlcSCModel.get( 0, 0, 1 ) );
#endif
    }
  }
}


#if SAO_TYPE_CODING
/** Code SAO EO class or BO band position 
 * \param uiLength
 * \param uiCode
 */
Void TEncSbac::codeSaoUflc       ( UInt uiLength, UInt uiCode )
{
  m_pcBinIf->encodeBinsEP ( uiCode, uiLength );
}
#else
/** Code SAO band position 
 * \param uiCode
 */
Void TEncSbac::codeSaoUflc       ( UInt uiCode )
{
  m_pcBinIf->encodeBinsEP ( uiCode, 5 );
}
#endif

#if SAO_MERGE_ONE_CTX
/** Code SAO merge flags
 * \param uiCode
 * \param uiCompIdx
 */
Void TEncSbac::codeSaoMerge       ( UInt uiCode )
{
  m_pcBinIf->encodeBin(((uiCode == 0) ? 0 : 1),  m_cSaoMergeSCModel.get( 0, 0, 0 ));
}
#else
/** Code SAO merge left flag 
 * \param uiCode
 * \param uiCompIdx
 */
Void TEncSbac::codeSaoMergeLeft       ( UInt uiCode, UInt uiCompIdx )
{
#if SAO_SINGLE_MERGE
  m_pcBinIf->encodeBin(((uiCode == 0) ? 0 : 1),  m_cSaoMergeLeftSCModel.get( 0, 0, 0 ));
#else
  m_pcBinIf->encodeBin(((uiCode == 0) ? 0 : 1),  m_cSaoMergeLeftSCModel.get( 0, 0, uiCompIdx ));
#endif
}
/** Code SAO merge up flag 
 * \param uiCode
 */
Void TEncSbac::codeSaoMergeUp       ( UInt uiCode)
{
  m_pcBinIf->encodeBin(((uiCode == 0) ? 0 : 1),  m_cSaoMergeUpSCModel.get( 0, 0, 0 ));
}
#endif

/** Code SAO type index 
 * \param uiCode
 */
Void TEncSbac::codeSaoTypeIdx       ( UInt uiCode)
{
#if SAO_TYPE_CODING
  if (uiCode == 0)
  {
    m_pcBinIf->encodeBin( 0, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
    m_pcBinIf->encodeBinEP( uiCode <= 4 ? 1 : 0 );
  }
#else
  Int i;
  if ( uiCode == 0 )
  {
    m_pcBinIf->encodeBin( 0, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
  }
  else
  {
    m_pcBinIf->encodeBin( 1, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) );
    for ( i=0; i<uiCode-1; i++ )
    {
      m_pcBinIf->encodeBin( 1, m_cSaoTypeIdxSCModel.get( 0, 0, 1 ) );
    }
    m_pcBinIf->encodeBin( 0, m_cSaoTypeIdxSCModel.get( 0, 0, 1 ) );
  }
#endif
}
/*!
 ****************************************************************************
 * \brief
 *   estimate bit cost for CBP, significant map and significant coefficients
 ****************************************************************************
 */
Void TEncSbac::estBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  estCBFBit( pcEstBitsSbac, 0, chType );

  estSignificantCoeffGroupMapBit( pcEstBitsSbac, 0, chType );
  
  // encode significance map
  estSignificantMapBit( pcEstBitsSbac, width, height, chType );

  // encode last significant position
  estLastSignificantPositionBit( pcEstBitsSbac, width, height, chType );
  
  // encode significant coefficients
  estSignificantCoefficientsBit( pcEstBitsSbac, 0, chType );
}

/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost for each CBP bit
 ****************************************************************************
 */
Void TEncSbac::estCBFBit( estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, ChannelType chType )
{
  ContextModel *pCtx = m_cCUQtCbfSCModel.get( 0 );

  for( UInt uiCtxInc = 0; uiCtxInc < (NUM_QT_CBF_CTX_SETS * NUM_QT_CBF_CTX_PER_SET); uiCtxInc++ )
  {
    pcEstBitsSbac->blockCbpBits[ uiCtxInc ][ 0 ] = pCtx[ uiCtxInc ].getEntropyBits( 0 );
    pcEstBitsSbac->blockCbpBits[ uiCtxInc ][ 1 ] = pCtx[ uiCtxInc ].getEntropyBits( 1 );
  }

  pCtx = m_cCUQtRootCbfSCModel.get( 0 );
  
  for( UInt uiCtxInc = 0; uiCtxInc < 4; uiCtxInc++ )
  {
    pcEstBitsSbac->blockRootCbpBits[ uiCtxInc ][ 0 ] = pCtx[ uiCtxInc ].getEntropyBits( 0 );
    pcEstBitsSbac->blockRootCbpBits[ uiCtxInc ][ 1 ] = pCtx[ uiCtxInc ].getEntropyBits( 1 );
  }
}


/*!
 ****************************************************************************
 * \brief
 *    estimate SAMBAC bit cost for significant coefficient group map
 ****************************************************************************
 */
Void TEncSbac::estSignificantCoeffGroupMapBit( estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, ChannelType chType )
{
  Int firstCtx = 0, numCtx = NUM_SIG_CG_FLAG_CTX;

  for ( Int ctxIdx = firstCtx; ctxIdx < firstCtx + numCtx; ctxIdx++ )
  {
    for( UInt uiBin = 0; uiBin < 2; uiBin++ )
    {
      pcEstBitsSbac->significantCoeffGroupBits[ ctxIdx ][ uiBin ] = m_cCUSigCoeffGroupSCModel.get(  0, chType, ctxIdx ).getEntropyBits( uiBin );
    }
  }
}


/*!
 ****************************************************************************
 * \brief
 *    estimate SAMBAC bit cost for significant coefficient map
 ****************************************************************************
 */
Void TEncSbac::estSignificantMapBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  //--------------------------------------------------------------------------------------------------

  //set up the number of channels and context variables

#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cr));
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)
  const UInt firstComponent = COMPONENT_Y;
  const UInt lastComponent  = COMPONENT_Y;
#endif

  //----------------------------------------------------------

  Int firstCtx = MAX_INT;
  Int numCtx   = MAX_INT;

  if      (((width == 4) && (height == 4)) || (fixed422SignificanceMapContext() && (((width == 4) && (height == 8)) || ((width == 8) && (height == 4)))))
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_4x4];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_4x4];
  }
  else if (((width == 8) && (height == 8)) || (fixed422SignificanceMapContext() && (((width == 8) && (height == 16)) || ((width == 16) && (height == 8)))))
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_8x8];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_8x8];
  }
  else
  {
    firstCtx = significanceMapContextSetStart[chType][CONTEXT_TYPE_NxN];
    numCtx   = significanceMapContextSetSize [chType][CONTEXT_TYPE_NxN];
  }

  //--------------------------------------------------------------------------------------------------

  //fill the data for the significace map

  for (UInt component = firstComponent; component <= lastComponent; component++)
  {
    const UInt contextOffset = getSignificanceMapContextOffset(ComponentID(component));

    if (firstCtx > 0)
    {
      for( UInt bin = 0; bin < 2; bin++ ) //always get the DC
      {
        pcEstBitsSbac->significantBits[ contextOffset ][ bin ] = m_cCUSigSCModel.get( 0, 0, contextOffset ).getEntropyBits( bin );
      }
    }

    for ( Int ctxIdx = firstCtx; ctxIdx < firstCtx + numCtx; ctxIdx++ )
    {
      for( UInt uiBin = 0; uiBin < 2; uiBin++ )
      {
        pcEstBitsSbac->significantBits[ contextOffset + ctxIdx ][ uiBin ] = m_cCUSigSCModel.get(  0, 0, (contextOffset + ctxIdx) ).getEntropyBits( uiBin );
      }
    }
  }

  //--------------------------------------------------------------------------------------------------
}


/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost of significant coefficient
 ****************************************************************************
 */

Void TEncSbac::estLastSignificantPositionBit( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType )
{
  //--------------------------------------------------------------------------------------------------.

  //set up the number of channels

#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cr));
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
  const UInt firstComponent = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
  const UInt lastComponent  = ((isLuma(chType)) ? (COMPONENT_Y) : (COMPONENT_Cb));
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)
  const UInt firstComponent = COMPONENT_Y;
  const UInt lastComponent  = COMPONENT_Y;
#endif

  //--------------------------------------------------------------------------------------------------

  //fill the data for the last-significant-coefficient position
  
  for (UInt componentIndex = firstComponent; componentIndex <= lastComponent; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    Int iBitsX = 0, iBitsY = 0;

    Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
    getLastSignificantContextParameters(ComponentID(component), width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

    Int ctx;

#if   (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 2)
    ContextModel *const pCtxX          = m_cCuCtxLastX.get( 0, component );
    ContextModel *const pCtxY          = m_cCuCtxLastY.get( 0, component );
    Int          *const lastXBitsArray = pcEstBitsSbac->lastXBits[component];
    Int          *const lastYBitsArray = pcEstBitsSbac->lastYBits[component];
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 1)
    const ChannelType channelType = toChannelType(ComponentID(component));

    ContextModel *const pCtxX = m_cCuCtxLastX.get( 0, channelType );
    ContextModel *const pCtxY = m_cCuCtxLastY.get( 0, channelType );
    Int          *const lastXBitsArray = pcEstBitsSbac->lastXBits[channelType];
    Int          *const lastYBitsArray = pcEstBitsSbac->lastYBits[channelType];
#elif (ECF__SIGNIFICANCE_MAP_CONTEXT_CHANNEL_SEPARATION == 0)
    ContextModel *const pCtxX          = m_cCuCtxLastX.get( 0, CHANNEL_TYPE_LUMA );
    ContextModel *const pCtxY          = m_cCuCtxLastY.get( 0, CHANNEL_TYPE_LUMA );
    Int          *const lastXBitsArray = pcEstBitsSbac->lastXBits;
    Int          *const lastYBitsArray = pcEstBitsSbac->lastYBits;
#endif

    //------------------------------------------------

    //X-coordinate

    for (ctx = 0; ctx < g_uiGroupIdx[ width - 1 ]; ctx++)
    {
      Int ctxOffset = blkSizeOffsetX + (ctx >>shiftX);
      lastXBitsArray[ ctx ] = iBitsX + pCtxX[ ctxOffset ].getEntropyBits( 0 );
      iBitsX += pCtxX[ ctxOffset ].getEntropyBits( 1 );
    }
  
    lastXBitsArray[ctx] = iBitsX;

    //------------------------------------------------

    //Y-coordinate

    for (ctx = 0; ctx < g_uiGroupIdx[ height - 1 ]; ctx++)
    {
      Int ctxOffset = blkSizeOffsetY + (ctx >>shiftY);
      lastYBitsArray[ ctx ] = iBitsY + pCtxY[ ctxOffset ].getEntropyBits( 0 );
      iBitsY += pCtxY[ ctxOffset ].getEntropyBits( 1 );
    }

    lastYBitsArray[ctx] = iBitsY;

  } //end of component loop

  //--------------------------------------------------------------------------------------------------
}


/*!
 ****************************************************************************
 * \brief
 *    estimate bit cost of significant coefficient
 ****************************************************************************
 */
Void TEncSbac::estSignificantCoefficientsBit( estBitsSbacStruct* pcEstBitsSbac, UInt uiCTXIdx, ChannelType chType )
{
  ContextModel *ctxOne = m_cCUOneSCModel.get(0, 0);
  ContextModel *ctxAbs = m_cCUAbsSCModel.get(0, 0);
  
  const UInt oneStartIndex = ((isLuma(chType)) ? (0)                     : (NUM_ONE_FLAG_CTX_LUMA));
  const UInt oneStopIndex  = ((isLuma(chType)) ? (NUM_ONE_FLAG_CTX_LUMA) : (NUM_ONE_FLAG_CTX));
  const UInt absStartIndex = ((isLuma(chType)) ? (0)                     : (NUM_ABS_FLAG_CTX_LUMA));
  const UInt absStopIndex  = ((isLuma(chType)) ? (NUM_ABS_FLAG_CTX_LUMA) : (NUM_ABS_FLAG_CTX));

  for (Int ctxIdx = oneStartIndex; ctxIdx < oneStopIndex; ctxIdx++)
  {
    pcEstBitsSbac->m_greaterOneBits[ ctxIdx ][ 0 ] = ctxOne[ ctxIdx ].getEntropyBits( 0 );
    pcEstBitsSbac->m_greaterOneBits[ ctxIdx ][ 1 ] = ctxOne[ ctxIdx ].getEntropyBits( 1 );    
  }

  for (Int ctxIdx = absStartIndex; ctxIdx < absStopIndex; ctxIdx++)
  {
    pcEstBitsSbac->m_levelAbsBits[ ctxIdx ][ 0 ] = ctxAbs[ ctxIdx ].getEntropyBits( 0 );
    pcEstBitsSbac->m_levelAbsBits[ ctxIdx ][ 1 ] = ctxAbs[ ctxIdx ].getEntropyBits( 1 );    
  }
}

/**
 - Initialize our context information from the nominated source.
 .
 \param pSrc From where to copy context information.
 */
Void TEncSbac::xCopyContextsFrom( TEncSbac* pSrc )
{  
  memcpy(m_contextModels, pSrc->m_contextModels, m_numContextModels*sizeof(m_contextModels[0]));
}

Void  TEncSbac::loadContexts ( TEncSbac* pScr)
{
  this->xCopyContextsFrom(pScr);
}
//! \}
