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

/** \file     TDecSbac.cpp
    \brief    Context-adaptive entropy decoder class
*/

#include "TDecSbac.h"
#include "TLibCommon/TComTU.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
//
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a) , a
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a)
#endif

//! \ingroup TLibDecoder
//! \{

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSbac::TDecSbac()
// new structure here
: m_pcBitstream                  ( 0 )
, m_pcTDecBinIf                  ( NULL )
, m_numContextModels             ( 0 )
, m_cCUSplitFlagSCModel          ( 1,             1,                      NUM_SPLIT_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSkipFlagSCModel           ( 1,             1,                      NUM_SKIP_FLAG_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeFlagExtSCModel       ( 1,             1,                      NUM_MERGE_FLAG_EXT_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel        ( 1,             1,                      NUM_MERGE_IDX_EXT_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPartSizeSCModel           ( 1,             1,                      NUM_PART_SIZE_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel           ( 1,             1,                      NUM_PRED_MODE_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
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
, m_cCUAMPSCModel                ( 1,             1,                      NUM_CU_AMP_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeSCModel             ( 1,             1,                      NUM_SAO_MERGE_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoTypeIdxSCModel           ( 1,             1,                      NUM_SAO_TYPE_IDX_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel        ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_TRANSFORMSKIP_FLAG_CTX       , m_contextModels + m_numContextModels, m_numContextModels)
, m_CUTransquantBypassFlagSCModel( 1,             1,                      NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX, m_contextModels + m_numContextModels, m_numContextModels)
#if RExt__NRCE2_RESIDUAL_DPCM
, m_interRdpcmFlagSCModel        ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_INTER_RDPCM_FLAG_CTX         , m_contextModels + m_numContextModels, m_numContextModels)
, m_interRdpcmDirSCModel         ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_INTER_RDPCM_DIR_CTX          , m_contextModels + m_numContextModels, m_numContextModels)
#endif
, m_cIntraBCPredFlagSCModel      (1,              1,                      NUM_INTRABC_PRED_CTX             , m_contextModels + m_numContextModels, m_numContextModels)
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
}

TDecSbac::~TDecSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecSbac::resetEntropy(TComSlice* pSlice)
{
  SliceType sliceType  = pSlice->getSliceType();
  Int       qp         = pSlice->getSliceQp();

  if (pSlice->getPPS()->getCabacInitPresentFlag() && pSlice->getCabacInitFlag())
  {
    switch (sliceType)
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      assert(0);
      break;
    }
  }

  m_cCUSplitFlagSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_SKIP_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer       ( sliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_MERGE_IDX_EXT );
  m_cCUPartSizeSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_PART_SIZE );
  m_cCUAMPSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_CU_AMP_POS );
  m_cCUPredModeSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer         ( sliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                  ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                  ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer               ( sliceType, qp, (UChar*)INIT_MVP_IDX );
  m_cSaoMergeSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer    ( sliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer( sliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
#if RExt__NRCE2_RESIDUAL_DPCM
  m_interRdpcmFlagSCModel.initBuffer        ( sliceType, qp, (UChar*)INIT_INTER_RDPCM_FLAG);
  m_interRdpcmDirSCModel.initBuffer         ( sliceType, qp, (UChar*)INIT_INTER_RDPCM_DIR);
#endif
  m_cIntraBCPredFlagSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_INTRABC_PRED_FLAG );

  m_uiLastDQpNonZero  = 0;

  // new structure
  m_uiLastQp          = qp;

  m_pcTDecBinIf->start();
}

/** The function does the following: Read out terminate bit. Flush CABAC. Byte-align for next tile.
 *  Intialize CABAC states. Start CABAC.
 */
Void TDecSbac::updateContextTables( SliceType eSliceType, Int iQp )
{
  UInt uiBit;
  m_pcTDecBinIf->decodeBinTrm(uiBit);
  assert(uiBit == 1); // end_of_sub_stream_one_bit must be equal to 1
  m_pcTDecBinIf->finish();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(STATS__TRAILING_BITS, m_pcBitstream->readOutTrailingBits(),0);
#else
  m_pcBitstream->readOutTrailingBits();
#endif
  m_cCUSplitFlagSCModel.initBuffer          ( eSliceType, iQp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SKIP_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer       ( eSliceType, iQp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_MERGE_IDX_EXT );
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
  m_cSaoMergeSCModel.initBuffer             ( eSliceType, iQp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer           ( eSliceType, iQp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer    ( eSliceType, iQp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer( eSliceType, iQp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
#if RExt__NRCE2_RESIDUAL_DPCM
  m_interRdpcmFlagSCModel.initBuffer        ( eSliceType, iQp, (UChar*)INIT_INTER_RDPCM_FLAG );
  m_interRdpcmDirSCModel.initBuffer         ( eSliceType, iQp, (UChar*)INIT_INTER_RDPCM_DIR );
#endif
  m_cIntraBCPredFlagSCModel.initBuffer      ( eSliceType, iQp, (UChar*)INIT_INTRABC_PRED_FLAG );

  m_pcTDecBinIf->start();
}

Void TDecSbac::parseTerminatingBit( UInt& ruiBit )
{
  m_pcTDecBinIf->decodeBinTrm( ruiBit );
  if ( ruiBit == 1 )
  {
    m_pcTDecBinIf->finish();
  }
}


#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
#endif
{
  if (uiMaxSymbol == 0)
  {
    ruiSymbol = 0;
    return;
  }

  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );

  if( ruiSymbol == 0 || uiMaxSymbol == 1 )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol++;
  }
  while( uiCont && ( uiSymbol < uiMaxSymbol - 1 ) );

  if( uiCont && ( uiSymbol == uiMaxSymbol - 1 ) )
  {
    uiSymbol++;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
#endif
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;

  while( uiBit )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += uiBit << uiCount++;
  }

  if ( --uiCount )
  {
    UInt bins;
    m_pcTDecBinIf->decodeBinsEP( bins, uiCount RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += bins;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset )
#endif
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));

  if( !ruiSymbol )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    uiSymbol++;
  }
  while( uiCont );

  ruiSymbol = uiSymbol;
}


/** Parsing of coeff_abs_level_remaing
 * \param ruiSymbol reference to coeff_abs_level_remaing
 * \param ruiParam reference to parameter
 * \returns Void
 */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam )
#endif
{

  UInt prefix   = 0;
  UInt codeWord = 0;
  do
  {
    prefix++;
    m_pcTDecBinIf->decodeBinEP( codeWord RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
  }
  while( codeWord);

  codeWord  = 1 - codeWord;
  prefix -= codeWord;
  codeWord=0;

  if (prefix < COEF_REMAIN_BIN_REDUCTION )
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (prefix<<rParam) + codeWord;
  }
  else
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,prefix-COEF_REMAIN_BIN_REDUCTION+rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (((1<<(prefix-COEF_REMAIN_BIN_REDUCTION))+COEF_REMAIN_BIN_REDUCTION-1)<<rParam)+codeWord;
  }
}


/** Parse I_PCM information.
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 *
 * If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
 */
Void TDecSbac::parseIPCMInfo ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBinTrm(uiSymbol);

  if (uiSymbol == 1)
  {
    Bool bIpcmFlag = true;

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setTrIdxSubParts     ( 0, uiAbsPartIdx, uiDepth );
    pcCU->setIPCMFlagSubParts  ( bIpcmFlag, uiAbsPartIdx, uiDepth );

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
          UInt sample;
          m_pcTDecBinIf->xReadPCMCode(sampleBits, sample);
          pPCMSample[x] = sample;
        }
        pPCMSample += width;
      }
    }

    m_pcTDecBinIf->start();
  }
}

Void TDecSbac::parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__TQ_BYPASS_FLAG) );
  pcCU->setCUTransquantBypassSubParts(uiSymbol ? true : false, uiAbsPartIdx, uiDepth);
}

/** parse skip flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SKIP_FLAG) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  if( uiSymbol )
  {
    pcCU->setSkipFlagSubParts( true,        uiAbsPartIdx, uiDepth );
    pcCU->setPredModeSubParts( MODE_INTER,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setMergeFlagSubParts( true , uiAbsPartIdx, 0, uiDepth );
  }
}

Void TDecSbac::parseIntraBCFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{  
  UInt uiSymbol = 0;

  UInt uiCtxIntraBC = pcCU->getCtxIntraBCFlag( uiAbsPartIdx ) ;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cIntraBCPredFlagSCModel.get( 0, 0, uiCtxIntraBC ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTRA_BLOCK_COPY_VECTOR)); 
  
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tIntraBCFlag" );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  if ( uiSymbol )
  {
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setPredModeSubParts( MODE_INTRABC, uiAbsPartIdx, uiDepth );
    pcCU->setTrIdxSubParts( 0, uiAbsPartIdx, uiDepth );    
    pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, DC_IDX, uiAbsPartIdx, uiDepth);
    pcCU->setIntraDirSubParts ( CHANNEL_TYPE_CHROMA, DC_IDX, uiAbsPartIdx, uiDepth);
  }
}

Void TDecSbac::parseIntraBC ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{  
  Int mvx = 0, mvy = 0;
  
  parseMvd(pcCU, uiAbsPartIdx, uiPartIdx, uiDepth, REF_PIC_LIST_INTRABC);

  mvx = pcCU->getCUMvField(REF_PIC_LIST_INTRABC)->getMvd(uiAbsPartIdx).getHor();
  mvy = pcCU->getCUMvField(REF_PIC_LIST_INTRABC)->getMvd(uiAbsPartIdx).getVer();

  const TComMv cMv(mvx, mvy );

  pcCU->getCUMvField( REF_PIC_LIST_INTRABC )->setAllMv( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiDepth, uiPartIdx );
}


/** parse merge flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPUIdx
 * \returns Void
 */
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_FLAG) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex )
{
  UInt uiUnaryIdx = 0;
  UInt uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( ; uiUnaryIdx < uiNumCand - 1; ++uiUnaryIdx )
    {
      UInt uiSymbol = 0;
      if ( uiUnaryIdx==0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  ruiMergeIndex = uiUnaryIdx;

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseMergeIndex()" )
  DTRACE_CABAC_T( "\tuiMRGIdx= " )
  DTRACE_CABAC_V( ruiMergeIndex )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseMVPIdx      ( Int& riMVPIdx )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cMVPIdxSCModel.get(0), 1, AMVP_MAX_NUM_CANDS-1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVP_IDX) );
  riMVPIdx = uiSymbol;
}

Void TDecSbac::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__SPLIT_FLAG, g_aucConvertToBit[g_uiMaxCUWidth>>uiDepth]+2);
#endif

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSplitFlagSCModel.get( 0, 0, pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth ) ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return;
}

/** parse partition size
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol, uiMode = 0;
  PartSize eMode;

  assert( ! pcCU->isIntraBC( uiAbsPartIdx ) );

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__PART_SIZE, g_aucConvertToBit[g_uiMaxCUWidth>>uiDepth]+2);
#endif

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    uiSymbol = 1;
    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    }
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
    UInt uiTrLevel = 0;
    UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
    UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
    uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;
    if( eMode == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
  else
  {
    UInt uiMaxNumBits = 2;

    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && !( (g_uiMaxCUWidth>>uiDepth) == 8 && (g_uiMaxCUHeight>>uiDepth) == 8 ) )
    {
      uiMaxNumBits ++;
    }

    for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, ui) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if ( uiSymbol )
      {
        break;
      }
      uiMode++;
    }
    eMode = (PartSize) uiMode;
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
    {
      if (eMode == SIZE_2NxN)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUAMPSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUAMPSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
  }
  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
}

/** parse prediction mode
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__PRED_MODE) );
  iPredMode += uiSymbol;
  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt absPartIdx, UInt depth )
{
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = mode==SIZE_NxN?4:1;
  UInt partOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  UInt mpmPred[4],symbol;
  Int j,intraPredMode;
  if (mode==SIZE_NxN)
  {
    depth++;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[g_uiMaxCUWidth>>depth]+2, CHANNEL_TYPE_LUMA);
#endif
  for (j=0;j<partNum;j++)
  {
    m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    mpmPred[j] = symbol;
  }
  for (j=0;j<partNum;j++)
  {
    Int preds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};
    Int predNum = pcCU->getIntraDirPredictor(absPartIdx+partOffset*j, preds, COMPONENT_Y);
    if (mpmPred[j])
    {
      m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if (symbol)
      {
        m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        symbol++;
      }
      intraPredMode = preds[symbol];
    }
    else
    {
      m_pcTDecBinIf->decodeBinsEP( symbol, 5 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      intraPredMode = symbol;

      //postponed sorting of MPMs (only in remaining branch)
      assert(predNum>=3); // It is currently always 3!
      if (preds[0] > preds[1])
      {
        std::swap(preds[0], preds[1]);
      }
      if (preds[0] > preds[2])
      {
        std::swap(preds[0], preds[2]);
      }
      if (preds[1] > preds[2])
      {
        std::swap(preds[1], preds[2]);
      }
      for ( Int i = 0; i < predNum; i++ )
      {
        intraPredMode += ( intraPredMode >= preds[i] );
      }
    }
    pcCU->setIntraDirSubParts(CHANNEL_TYPE_LUMA, (UChar)intraPredMode, absPartIdx+partOffset*j, depth );
  }
}


Void TDecSbac::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[g_uiMaxCUWidth>>uiDepth]+2, CHANNEL_TYPE_CHROMA);
#endif

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  if( uiSymbol == 0 )
  {
    uiSymbol = DM_CHROMA_IDX;
  }
  else
  {
    UInt uiIPredMode;
    m_pcTDecBinIf->decodeBinsEP( uiIPredMode, 2 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
    pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );
    uiSymbol = uiAllowedChromaDir[ uiIPredMode ];
  }

  pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiSymbol, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx = m_cCUInterDirSCModel.get( 0 );

  uiSymbol = 0;
  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + uiCtx ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
  }

  if( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + 4 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
    assert(uiSymbol == 0 || uiSymbol == 1);
  }

  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecSbac::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList )
{
  UInt uiSymbol;

  ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
  m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );

  if( uiSymbol )
  {
    UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
    pCtx++;
    UInt ui;
    for( ui = 0; ui < uiRefNum; ++ui )
    {
      if( ui == 0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
    uiSymbol = ui + 1;
  }
  riRefFrmIdx = uiSymbol;

  return;
}

Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  UInt uiHorAbs;
  UInt uiVerAbs;
  UInt uiHorSign = 0;
  UInt uiVerSign = 0;
  ContextModel *pCtx = m_cCUMvdSCModel.get( 0 );

  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    uiHorAbs=0;
    uiVerAbs=0;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiHorAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
    m_pcTDecBinIf->decodeBin( uiVerAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );

    const Bool bHorAbsGr0 = uiHorAbs != 0;
    const Bool bVerAbsGr0 = uiVerAbs != 0;
    pCtx++;

    if( bHorAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiHorAbs += uiSymbol;
    }

    if( bVerAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiVerAbs += uiSymbol;
    }

    if( bHorAbsGr0 )
    {
      if( 2 == uiHorAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiHorAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiHorSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

    if( bVerAbsGr0 )
    {
      if( 2 == uiVerAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiVerAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiVerSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

  }

  const TComMv cMv( uiHorSign ? -Int( uiHorAbs ): uiHorAbs, uiVerSign ? -Int( uiVerAbs ) : uiVerAbs );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiDepth, uiPartIdx );
  return;
}


Void TDecSbac::parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize )
{
  m_pcTDecBinIf->decodeBin( ruiSubdivFlag, m_cCUTransSubdivFlagSCModel.get( 0, 0, uiLog2TransformBlockSize )
      RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__TRANSFORM_SUBDIV_FLAG, 5-uiLog2TransformBlockSize))
                          );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseTransformSubdivFlag()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( ruiSubdivFlag )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiLog2TransformBlockSize )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseQtRootCbf( UInt uiAbsPartIdx, UInt& uiQtRootCbf )
{
  UInt uiSymbol;
  const UInt uiCtx = 0;
  m_pcTDecBinIf->decodeBin( uiSymbol , m_cCUQtRootCbfSCModel.get( 0, 0, uiCtx ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__QT_ROOT_CBF) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseQtRootCbf()" )
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( uiSymbol )
  DTRACE_CABAC_T( "\tctx=" )
  DTRACE_CABAC_V( uiCtx )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )

  uiQtRootCbf = uiSymbol;
}

Void TDecSbac::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  Int qp;
  UInt uiDQp;
  Int  iDQp;

  UInt uiSymbol;

  xReadUnaryMaxSymbol (uiDQp,  &m_cCUDeltaQpSCModel.get( 0, 0, 0 ), 1, CU_DQP_TU_CMAX RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP) );

  if( uiDQp >= CU_DQP_TU_CMAX)
  {
    xReadEpExGolomb( uiSymbol, CU_DQP_EG_k RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP));
    uiDQp+=uiSymbol;
  }

  if ( uiDQp > 0 )
  {
    UInt uiSign;
    Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
    m_pcTDecBinIf->decodeBinEP(uiSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__DELTA_QP_EP));
    iDQp = uiDQp;
    if(uiSign)
    {
      iDQp = -iDQp;
    }
    qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+qpBdOffsetY)) - qpBdOffsetY;
  }
  else
  {
    qp = pcCU->getRefQP(uiAbsPartIdx);
  }

  pcCU->setQPSubParts(qp, uiAbsPartIdx, uiDepth);
  pcCU->setCodedQP(qp);
}


#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
Void TDecSbac::parseQtCbf( TComTU &rTu, const ComponentID compID, const Bool lowestLevel )
#else
Void TDecSbac::parseQtCbf( TComTU &rTu, const ComponentID compID )
#endif
{
  TComDataCU* pcCU = rTu.getCU();

  const UInt absPartIdx       = rTu.GetAbsPartIdxTU(compID);
  const UInt TUDepth          = rTu.GetTransformDepthRel();
  const UInt uiCtx            = pcCU->getCtxQtCbf( rTu, toChannelType(compID) );
  const UInt contextSet       = toChannelType(compID);

#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  const UInt width            = rTu.getRect(compID).width;
  const UInt height           = rTu.getRect(compID).height;
  const Bool canQuadSplit     = (width >= (MIN_TU_SIZE * 2)) && (height >= (MIN_TU_SIZE * 2));
  const UInt coveredPartIdxes = rTu.GetAbsPartIdxNumParts(compID);

  //NOTE: RExt - since the CBF for chroma is coded at the highest level possible, if sub-TUs are
  //             to be coded for a 4x8 chroma TU, their CBFs must be coded at the highest 4x8 level
  //             (i.e. where luma TUs are 8x8 rather than 4x4)
  //    ___ ___
  //   |   |   | <- 4 x (8x8 luma + 4x8 4:2:2 chroma)
  //   |___|___|    each quadrant has its own chroma CBF
  //   |   |   | _ _ _ _
  //   |___|___|        |
  //   <--16--->        V
  //                   _ _
  //                  |_|_| <- 4 x 4x4 luma + 1 x 4x8 4:2:2 chroma
  //                  |_|_|    no chroma CBF is coded - instead the parent CBF is inherited
  //                  <-8->    if sub-TUs are present, their CBFs had to be coded at the parent level

  const UInt lowestTUDepth = TUDepth + ((!lowestLevel && !canQuadSplit) ? 1 : 0); //unsplittable TUs inherit their parent's CBF
        UInt lowestTUCBF   = 0;

  if ((width != height) && (lowestLevel || !canQuadSplit)) //if sub-TUs are present
  {
    const UInt subTUDepth        = lowestTUDepth + 1;
    const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

    UInt combinedSubTUCBF = 0;

    for (UInt subTU = 0; subTU < 2; subTU++)
    {
      UInt uiCbf = MAX_UINT;
      m_pcTDecBinIf->decodeBin(uiCbf, m_cCUQtCbfSCModel.get(0, contextSet, uiCtx) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__QT_CBF, g_aucConvertToBit[rTu.getRect(compID).width]+2, compID)));
      
      const UInt subTUAbsPartIdx = absPartIdx + (subTU * partIdxesPerSubTU);
      pcCU->setCbfPartRange((uiCbf << subTUDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU);
      combinedSubTUCBF |= uiCbf;

      DTRACE_CABAC_VL( g_nSymbolCounter++ )
      DTRACE_CABAC_T( "\tparseQtCbf()" )
      DTRACE_CABAC_T( "\tsub-TU=" )
      DTRACE_CABAC_V( subTU )
      DTRACE_CABAC_T( "\tsymbol=" )
      DTRACE_CABAC_V( uiCbf )
      DTRACE_CABAC_T( "\tctx=" )
      DTRACE_CABAC_V( uiCtx )
      DTRACE_CABAC_T( "\tetype=" )
      DTRACE_CABAC_V( compID )
      DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
      DTRACE_CABAC_V( subTUAbsPartIdx )
      DTRACE_CABAC_T( "\n" )
    }

    //propagate the sub-TU CBF up to the lowest TU level
    if (combinedSubTUCBF != 0)
    {
      pcCU->bitwiseOrCbfPartRange((combinedSubTUCBF << lowestTUDepth), compID, absPartIdx, coveredPartIdxes);
      lowestTUCBF = combinedSubTUCBF;
    }
  }
  else
  {
#endif
    UInt uiCbf = MAX_UINT;
    m_pcTDecBinIf->decodeBin(uiCbf, m_cCUQtCbfSCModel.get(0, contextSet, uiCtx) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__QT_CBF, g_aucConvertToBit[rTu.getRect(compID).width]+2, compID)));

#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
    pcCU->setCbfSubParts((uiCbf << lowestTUDepth), compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#else
    pcCU->setCbfSubParts((uiCbf << TUDepth), compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif

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

#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
    lowestTUCBF = uiCbf;
  }

  //propagate the lowest level CBF up to the current level
  if (lowestTUCBF != 0)
  {
    for (UInt depth = TUDepth; depth < lowestTUDepth; depth++)
    {
      pcCU->bitwiseOrCbfPartRange((lowestTUCBF << depth), compID, absPartIdx, coveredPartIdxes);
    }
  }
#endif
}


void TDecSbac::parseTransformSkipFlags (TComTU &rTu, ComponentID component)
{
  TComDataCU* pcCU=rTu.getCU();
  UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU(component);

  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    return;
  }

  if (!TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(component), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize()))
  {
    return;
  }

  UInt useTransformSkip;

  m_pcTDecBinIf->decodeBin( useTransformSkip , m_cTransformSkipSCModel.get( 0, toChannelType(component), 0 )
      RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(TComCodingStatisticsClassType(STATS__CABAC_BITS__TRANSFORM_SKIP_FLAGS, component))
                          );

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T("\tparseTransformSkip()");
  DTRACE_CABAC_T( "\tsymbol=" )
  DTRACE_CABAC_V( useTransformSkip )
  DTRACE_CABAC_T( "\tAddr=" )
  DTRACE_CABAC_V( pcCU->getAddr() )
  DTRACE_CABAC_T( "\tetype=" )
  DTRACE_CABAC_V( component )
  DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_T( "\n" )

#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
  pcCU->setTransformSkipPartRange( useTransformSkip, component, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(component));
#else
  pcCU->setTransformSkipSubParts( useTransformSkip, component, uiAbsPartIdx, rTu.GetTransformDepthTotalAdj(component));
#endif
}


/** Parse (X,Y) position of the last significant coefficient
 * \param uiPosLastX reference to X component of last coefficient
 * \param uiPosLastY reference to Y component of last coefficient
 * \param width  Block width
 * \param height Block height
 * \param eTType plane type / luminance or chrominance
 * \param uiScanIdx scan type (zig-zag, hor, ver)
 *
 * This method decodes the X and Y component within a block of the last significant coefficient.
 */
Void TDecSbac::parseLastSignificantXY( UInt& uiPosLastX, UInt& uiPosLastY, Int width, Int height, ComponentID component, UInt uiScanIdx )
{
  UInt uiLast;

  ContextModel *pCtxX = m_cCuCtxLastX.get( 0, toChannelType(component) );
  ContextModel *pCtxY = m_cCuCtxLastY.get( 0, toChannelType(component) );

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__LAST_SIG_X_Y, g_aucConvertToBit[width]+2, component);
#endif


  if ( uiScanIdx == SCAN_VER )
  {
    swap( width, height );
  }

  Int blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY;
  getLastSignificantContextParameters(component, width, height, blkSizeOffsetX, blkSizeOffsetY, shiftX, shiftY);

  //------------------

  // posX

  for( uiPosLastX = 0; uiPosLastX < g_uiGroupIdx[ width - 1 ]; uiPosLastX++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxX + blkSizeOffsetX + (uiPosLastX >>shiftX) ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if( !uiLast )
    {
      break;
    }
  }

  // posY

  for( uiPosLastY = 0; uiPosLastY < g_uiGroupIdx[ height - 1 ]; uiPosLastY++ )
  {
    m_pcTDecBinIf->decodeBin( uiLast, *( pCtxY + blkSizeOffsetY + (uiPosLastY >>shiftY)) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if( !uiLast )
    {
      break;
    }
  }

  // EP-coded part

  if ( uiPosLastX > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastX - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      uiTemp += uiLast << i;
    }
    uiPosLastX = g_uiMinInGroup[ uiPosLastX ] + uiTemp;
  }
  if ( uiPosLastY > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( uiPosLastY - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      m_pcTDecBinIf->decodeBinEP( uiLast RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      uiTemp += uiLast << i;
    }
    uiPosLastY = g_uiMinInGroup[ uiPosLastY ] + uiTemp;
  }

  if( uiScanIdx == SCAN_VER )
  {
    swap( uiPosLastX, uiPosLastY );
  }
}

Void TDecSbac::parseCoeffNxN(  TComTU &rTu, ComponentID compID )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &rRect=rTu.getRect(compID);
  const UInt uiWidth=rRect.width;
  const UInt uiHeight=rRect.height;
  TCoeff* pcCoef=(pcCU->getCoeff(compID)+rTu.getCoefficientOffset(compID));

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseCoeffNxN()\teType=" )
  DTRACE_CABAC_V( compID )
  DTRACE_CABAC_T( "\twidth=" )
  DTRACE_CABAC_V( uiWidth )
  DTRACE_CABAC_T( "\theight=" )
  DTRACE_CABAC_V( uiHeight )
  DTRACE_CABAC_T( "\tdepth=" )
//  DTRACE_CABAC_V( rTu.GetTransformDepthTotalAdj(compID) )
  DTRACE_CABAC_V( rTu.GetTransformDepthTotal() )
  DTRACE_CABAC_T( "\tabspartidx=" )
//  DTRACE_CABAC_V( uiAbsPartIdx )
  DTRACE_CABAC_V( rTu.GetAbsPartIdxTU(compID) )
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

  if( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    std::cerr << "ERROR: parseCoeffNxN was passed a TU with dimensions larger than the maximum allowed size" << std::endl;
    assert(false);
    exit(1);
  }

  //--------------------------------------------------------------------------------------------------

  //set parameters

  const ChannelType  chType            = toChannelType(compID);
  const UInt         uiLog2BlockWidth  = g_aucConvertToBit[ uiWidth  ] + 2;
  const UInt         uiLog2BlockHeight = g_aucConvertToBit[ uiHeight ] + 2;
  const UInt         uiMaxNumCoeff     = uiWidth * uiHeight;
  const UInt         uiMaxNumCoeffM1   = uiMaxNumCoeff - 1;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatisticsClassType ctype_group(STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_map(STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_gt1(STATS__CABAC_BITS__GT1_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_gt2(STATS__CABAC_BITS__GT2_FLAG, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_signs(STATS__CABAC_BITS__SIGN_BIT, uiLog2BlockWidth, compID);
  TComCodingStatisticsClassType ctype_escs(STATS__CABAC_BITS__ESCAPE_BITS, uiLog2BlockWidth, compID);
#endif

  Bool beValid;
  if (pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    beValid = false;
#if RDPCM_INTER_LOSSLESS
    if((!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx))
      parseInterRdpcmMode(rTu, compID);
#endif
  }
  else
  {
    beValid = pcCU->getSlice()->getPPS()->getSignHideFlag() > 0;
  }

  UInt absSum = 0;

  //--------------------------------------------------------------------------------------------------

  if(pcCU->getSlice()->getPPS()->getUseTransformSkip())
  {
    parseTransformSkipFlags(rTu, compID);
#if RDPCM_INTER_LOSSY
    //  This TU has coefficients and is transform skipped. Check whether is inter coded and if yes decode the inter RDPCM mode
    if(pcCU->getTransformSkip(uiAbsPartIdx, compID) && (!pcCU->isIntra(uiAbsPartIdx)) && pcCU->isRDPCMEnabled(uiAbsPartIdx) )
    {
      parseInterRdpcmMode(rTu, compID);
      if(pcCU->getInterRdpcmMode(compID, uiAbsPartIdx) > DPCM_OFF)
      {
        //  Sign data hiding is avoided for horizontal and vertical RDPCM modes
        beValid = false;
      }
    }
#endif
  }

#if RExt__NRCE2_RESIDUAL_DPCM
  Int uiIntraMode = -1;
  const Bool       bIsLuma = isLuma(compID);
  Int isIntra = pcCU->isIntra(uiAbsPartIdx) ? 1 : 0;
  if ( isIntra && pcCU->isRDPCMEnabled(uiAbsPartIdx) )
  {
    uiIntraMode = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
    uiIntraMode = (uiIntraMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, rTu.GetChromaFormat())) : uiIntraMode;
    uiIntraMode = ((rTu.GetChromaFormat() == CHROMA_422) && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiIntraMode] : uiIntraMode;

    Bool transformSkip = pcCU->getTransformSkip( uiAbsPartIdx,compID);
    Bool rdpcm_lossy = ( transformSkip /*&& isIntra*/ && ( (uiIntraMode == HOR_IDX) || (uiIntraMode == VER_IDX) ) );
    if ( rdpcm_lossy )
    {
      beValid = false;
    }
  }
#endif

  //--------------------------------------------------------------------------------------------------

  //select scans
  TUEntropyCodingParameters codingParameters;
  getTUEntropyCodingParameters(codingParameters, rTu, compID);

  //===== decode last significant =====
  UInt uiPosLastX, uiPosLastY;
  parseLastSignificantXY( uiPosLastX, uiPosLastY, uiWidth, uiHeight, compID, codingParameters.scanType );
  UInt uiBlkPosLast      = uiPosLastX + (uiPosLastY<<uiLog2BlockWidth);
  pcCoef[ uiBlkPosLast ] = 1;

  //===== decode significance flags =====
  UInt uiScanPosLast;
  for( uiScanPosLast = 0; uiScanPosLast < uiMaxNumCoeffM1; uiScanPosLast++ )
  {
    UInt uiBlkPos = codingParameters.scan[ uiScanPosLast ];
    if( uiBlkPosLast == uiBlkPos )
    {
      break;
    }
  }

  ContextModel * const baseCoeffGroupCtx = m_cCUSigCoeffGroupSCModel.get( 0, isChroma(chType) );
  ContextModel * const baseCtx = m_cCUSigSCModel.get( 0, 0 ) + getSignificanceMapContextOffset(compID);

  const Int  iLastScanSet  = uiScanPosLast >> MLS_CG_SIZE;
  UInt c1                  = 1;
  UInt uiGoRiceParam       = 0;


  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];
  memset( uiSigCoeffGroupFlag, 0, sizeof(UInt) * MLS_GRP_NUM );

  Int  iScanPosSig             = (Int) uiScanPosLast;
  for( Int iSubSet = iLastScanSet; iSubSet >= 0; iSubSet-- )
  {
    Int  iSubPos   = iSubSet << MLS_CG_SIZE;
    uiGoRiceParam  = 0;
    Int numNonZero = 0;

    Int lastNZPosInCG  = -1;
    Int firstNZPosInCG = 1 << MLS_CG_SIZE;

    Int pos[1 << MLS_CG_SIZE];

    if( iScanPosSig == (Int) uiScanPosLast )
    {
      lastNZPosInCG  = iScanPosSig;
      firstNZPosInCG = iScanPosSig;
      iScanPosSig--;
      pos[ numNonZero ] = uiBlkPosLast;
      numNonZero = 1;
    }

    // decode significant_coeffgroup_flag
    Int iCGBlkPos = codingParameters.scanCG[ iSubSet ];
    Int iCGPosY   = iCGBlkPos / codingParameters.widthInGroups;
    Int iCGPosX   = iCGBlkPos - (iCGPosY * codingParameters.widthInGroups);

    if( iSubSet == iLastScanSet || iSubSet == 0)
    {
      uiSigCoeffGroupFlag[ iCGBlkPos ] = 1;
    }
    else
    {
      UInt uiSigCoeffGroup;
      UInt uiCtxSig  = TComTrQuant::getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );
      m_pcTDecBinIf->decodeBin( uiSigCoeffGroup, baseCoeffGroupCtx[ uiCtxSig ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_group) );
      uiSigCoeffGroupFlag[ iCGBlkPos ] = uiSigCoeffGroup;
    }

    // decode significant_coeff_flag
    const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, iCGPosX, iCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);

    UInt uiBlkPos, uiSig, uiCtxSig;
    for( ; iScanPosSig >= iSubPos; iScanPosSig-- )
    {
      uiBlkPos  = codingParameters.scan[ iScanPosSig ];
      uiSig     = 0;

      if( uiSigCoeffGroupFlag[ iCGBlkPos ] )
      {
        if( iScanPosSig > iSubPos || iSubSet == 0  || numNonZero )
        {
          uiCtxSig  = TComTrQuant::getSigCtxInc( patternSigCtx, codingParameters, iScanPosSig, uiLog2BlockWidth, uiLog2BlockHeight, chType );
          m_pcTDecBinIf->decodeBin( uiSig, baseCtx[ uiCtxSig ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_map) );
        }
        else
        {
          uiSig = 1;
        }
      }
      pcCoef[ uiBlkPos ] = uiSig;
      if( uiSig )
      {
        pos[ numNonZero ] = uiBlkPos;
        numNonZero ++;
        if( lastNZPosInCG == -1 )
        {
          lastNZPosInCG = iScanPosSig;
        }
        firstNZPosInCG = iScanPosSig;
      }
    }

    if( numNonZero > 0 )
    {
      Bool signHidden = ( lastNZPosInCG - firstNZPosInCG >= SBH_THRESHOLD );

      absSum = 0;

      const UInt uiCtxSet = getContextSetIndex(compID, iSubSet, (c1 == 0));
      c1 = 1;
      UInt uiBin;

      ContextModel *baseCtxMod = m_cCUOneSCModel.get( 0, 0 ) + (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet);

      Int absCoeff[1 << MLS_CG_SIZE];

      for ( Int i = 0; i < numNonZero; i++) absCoeff[i] = 1;
      Int numC1Flag = min(numNonZero, C1FLAG_NUMBER);
      Int firstC2FlagIdx = -1;

      for( Int idx = 0; idx < numC1Flag; idx++ )
      {
        m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[c1] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_gt1) );
        if( uiBin == 1 )
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
        absCoeff[ idx ] = uiBin + 1;
      }

      if (c1 == 0)
      {
        baseCtxMod = m_cCUAbsSCModel.get( 0, 0 ) + (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet);
        if ( firstC2FlagIdx != -1)
        {
          m_pcTDecBinIf->decodeBin( uiBin, baseCtxMod[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_gt2) );
          absCoeff[ firstC2FlagIdx ] = uiBin + 2;
        }
      }

      UInt coeffSigns;
      if ( signHidden && beValid )
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero-1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_signs) );
        coeffSigns <<= 32 - (numNonZero-1);
      }
      else
      {
        m_pcTDecBinIf->decodeBinsEP( coeffSigns, numNonZero RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_signs) );
        coeffSigns <<= 32 - numNonZero;
      }

      Int iFirstCoeff2 = 1;
      if (c1 == 0 || numNonZero > C1FLAG_NUMBER)
      {
        for( Int idx = 0; idx < numNonZero; idx++ )
        {
          UInt baseLevel  = (idx < C1FLAG_NUMBER)? (2 + iFirstCoeff2) : 1;

          if( absCoeff[ idx ] == baseLevel)
          {
            UInt uiLevel;
            xReadCoefRemainExGolomb( uiLevel, uiGoRiceParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype_escs) );

            absCoeff[ idx ] = uiLevel + baseLevel;

            if(absCoeff[idx]>3*(1<<uiGoRiceParam))
            {
              uiGoRiceParam = min<UInt>(uiGoRiceParam+ 1, 4);
            }
          }

          if(absCoeff[ idx ] >= 2)
          {
            iFirstCoeff2 = 0;
          }
        }
      }

      for( Int idx = 0; idx < numNonZero; idx++ )
      {
        Int blkPos = pos[ idx ];
        // Signs applied later.
        pcCoef[ blkPos ] = absCoeff[ idx ];
        absSum += absCoeff[ idx ];

        if ( idx == numNonZero-1 && signHidden && beValid )
        {
          // Infer sign of 1st element.
          if (absSum&0x1) pcCoef[ blkPos ] = -pcCoef[ blkPos ];
        }
        else
        {
          Int sign = static_cast<Int>( coeffSigns ) >> 31;
          pcCoef[ blkPos ] = ( pcCoef[ blkPos ] ^ sign ) - sign;
          coeffSigns <<= 1;
        }
      }
    }
  }

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  printSBACCoeffData(uiPosLastX, uiPosLastY, uiWidth, uiHeight, compID, uiAbsPartIdx, codingParameters.scanType, pcCoef);
#endif

  return;
}

Void TDecSbac::parseSaoMaxUvlc ( UInt& val, UInt maxSymbol )
{
  if (maxSymbol == 0)
  {
    val = 0;
    return;
  }

  UInt code;
  Int  i;
  m_pcTDecBinIf->decodeBinEP( code RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  if ( code == 0 )
  {
    val = 0;
    return;
  }

  i=1;
  while (1)
  {
    m_pcTDecBinIf->decodeBinEP( code RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
    if ( code == 0 )
    {
      break;
    }
    i++;
    if (i == maxSymbol)
    {
      break;
    }
  }

  val = i;
}

Void TDecSbac::parseSaoUflc (UInt uiLength, UInt&  riVal)
{
  m_pcTDecBinIf->decodeBinsEP ( riVal, uiLength RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
}

Void TDecSbac::parseSaoMerge (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoMergeSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  ruiVal = (Int)uiCode;
}

Void TDecSbac::parseSaoTypeIdx (UInt&  ruiVal)
{
  UInt uiCode;
  m_pcTDecBinIf->decodeBin( uiCode, m_cSaoTypeIdxSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
  if (uiCode == 0)
  {
    ruiVal = 0;
  }
  else
  {
    m_pcTDecBinIf->decodeBinEP( uiCode RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
    if (uiCode == 0)
    {
#if HM_CLEANUP_SAO
      ruiVal = 1;
#else
      ruiVal = 5;
#endif
    }
    else
    {
#if HM_CLEANUP_SAO
      ruiVal = 2;
#else
      ruiVal = 1;
#endif
    }
  }
}

#if HM_CLEANUP_SAO

Void TDecSbac::parseSaoSign(UInt& val)
{
  m_pcTDecBinIf->decodeBinEP ( val RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) ); 
}

Void TDecSbac::parseSAOBlkParam (SAOBlkParam& saoBlkParam
                                , Bool* sliceEnabled
                                , Bool leftMergeAvail
                                , Bool aboveMergeAvail
                                )
{
  UInt uiSymbol;

  Bool isLeftMerge = false;
  Bool isAboveMerge= false;

  if(leftMergeAvail)
  {
    parseSaoMerge(uiSymbol); //sao_merge_left_flag
    isLeftMerge = (uiSymbol?true:false);
  }

  if( aboveMergeAvail && !isLeftMerge)
  {
    parseSaoMerge(uiSymbol); //sao_merge_up_flag
    isAboveMerge = (uiSymbol?true:false);
  }

  if(isLeftMerge || isAboveMerge) //merge mode
  {
    for (UInt componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
    {
      saoBlkParam[componentIndex].modeIdc = SAO_MODE_MERGE;
      saoBlkParam[componentIndex].typeIdc = (isLeftMerge)?SAO_MERGE_LEFT:SAO_MERGE_ABOVE;
    }
  }
  else //new or off mode
  {    
    for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      SAOOffset& ctbParam = saoBlkParam[compIdx];

      if(!sliceEnabled[compIdx])
      {
        //off
        ctbParam.modeIdc = SAO_MODE_OFF;
        continue;
      }

      //type
      if(compIdx == COMPONENT_Y || compIdx == COMPONENT_Cb)
      {
        parseSaoTypeIdx(uiSymbol); //sao_type_idx_luma or sao_type_idx_chroma

        assert(uiSymbol ==0 || uiSymbol ==1 || uiSymbol ==2);

        if(uiSymbol ==0) //OFF
        {
          ctbParam.modeIdc = SAO_MODE_OFF;
        }
        else if(uiSymbol == 1) //BO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_BO;
        }
        else //2, EO
        {
          ctbParam.modeIdc = SAO_MODE_NEW;
          ctbParam.typeIdc = SAO_TYPE_START_EO;
        }

      }
      else //Cr, follow Cb SAO type
      {
        ctbParam.modeIdc = saoBlkParam[COMPONENT_Cb].modeIdc;
        ctbParam.typeIdc = saoBlkParam[COMPONENT_Cb].typeIdc;
      }

      if(ctbParam.modeIdc == SAO_MODE_NEW)
      {
        Int offset[4];
        for(Int i=0; i< 4; i++)
        {
          parseSaoMaxUvlc(uiSymbol,  g_saoMaxOffsetQVal[compIdx] ); //sao_offset_abs
          offset[i] = (Int)uiSymbol;
        }

        if(ctbParam.typeIdc == SAO_TYPE_START_BO)
        {
          for(Int i=0; i< 4; i++)
          {
            if(offset[i] != 0)
            {
              parseSaoSign(uiSymbol); //sao_offset_sign
              if(uiSymbol)
              {
                offset[i] = -offset[i];
              }
            }
          }
          parseSaoUflc(NUM_SAO_BO_CLASSES_LOG2, uiSymbol ); //sao_band_position
          ctbParam.typeAuxInfo = uiSymbol;
        
          for(Int i=0; i<4; i++)
          {
            ctbParam.offset[(ctbParam.typeAuxInfo+i)%MAX_NUM_SAO_CLASSES] = offset[i];
          }      
        
        }
        else //EO
        {
          ctbParam.typeAuxInfo = 0;

          if(compIdx == COMPONENT_Y || compIdx == COMPONENT_Cb)
          {
            parseSaoUflc(NUM_SAO_EO_TYPES_LOG2, uiSymbol ); //sao_eo_class_luma or sao_eo_class_chroma
            ctbParam.typeIdc += uiSymbol;
          }
          else
          {
            ctbParam.typeIdc = saoBlkParam[COMPONENT_Cb].typeIdc;
          }
          ctbParam.offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
          ctbParam.offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
          ctbParam.offset[SAO_CLASS_EO_PLAIN      ] = 0;
          ctbParam.offset[SAO_CLASS_EO_HALF_PEAK  ] = -offset[2];
          ctbParam.offset[SAO_CLASS_EO_FULL_PEAK  ] = -offset[3];
        }
      }
    }
  }
}

#else

inline Void copySaoOneLcuParam(SaoLcuParam* psDst,  SaoLcuParam* psSrc)
{
  Int i;
  psDst->partIdx = psSrc->partIdx;
  psDst->typeIdx    = psSrc->typeIdx;
  if (psDst->typeIdx != -1)
  {
    psDst->subTypeIdx = psSrc->subTypeIdx ;
    psDst->length  = psSrc->length;
    for (i=0;i<psDst->length;i++)
    {
      psDst->offset[i] = psSrc->offset[i];
    }
  }
  else
  {
    psDst->length  = 0;
    for (i=0;i<SAO_BO_LEN;i++)
    {
      psDst->offset[i] = 0;
    }
  }
}

Void TDecSbac::parseSaoOffset(SaoLcuParam* psSaoLcuParam, const ComponentID compIdx)
{
  UInt uiSymbol;

  static Int iTypeLength[MAX_NUM_SAO_TYPE] =
  {
    SAO_EO_LEN,
    SAO_EO_LEN,
    SAO_EO_LEN,
    SAO_EO_LEN,
    SAO_BO_LEN
  };

  if (compIdx==COMPONENT_Cr)
  {
    uiSymbol = (UInt)( psSaoLcuParam->typeIdx + 1);
  }
  else
  {
    parseSaoTypeIdx(uiSymbol);
  }

  psSaoLcuParam->typeIdx = (Int)uiSymbol - 1;
  if (uiSymbol)
  {
    psSaoLcuParam->length = iTypeLength[psSaoLcuParam->typeIdx];
    Int offsetTh = 1 << min(g_bitDepth[toChannelType(compIdx)] - 5,5);

    if( psSaoLcuParam->typeIdx == SAO_BO )
    {
      for(Int i=0; i< psSaoLcuParam->length; i++)
      {
        parseSaoMaxUvlc(uiSymbol, offsetTh -1 );
        psSaoLcuParam->offset[i] = uiSymbol;
      }

      for(Int i=0; i< psSaoLcuParam->length; i++)
      {
        if (psSaoLcuParam->offset[i] != 0)
        {
          m_pcTDecBinIf->decodeBinEP ( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SAO) );
          if (uiSymbol)
          {
            psSaoLcuParam->offset[i] = -psSaoLcuParam->offset[i] ;
          }
        }
      }
      parseSaoUflc(5, uiSymbol );
      psSaoLcuParam->subTypeIdx = uiSymbol;
    }
    else if( psSaoLcuParam->typeIdx < 4 )
    {
      parseSaoMaxUvlc(uiSymbol, offsetTh -1 ); psSaoLcuParam->offset[0] = uiSymbol;
      parseSaoMaxUvlc(uiSymbol, offsetTh -1 ); psSaoLcuParam->offset[1] = uiSymbol;
      parseSaoMaxUvlc(uiSymbol, offsetTh -1 ); psSaoLcuParam->offset[2] = -(Int)uiSymbol;
      parseSaoMaxUvlc(uiSymbol, offsetTh -1 ); psSaoLcuParam->offset[3] = -(Int)uiSymbol;

      if (compIdx != COMPONENT_Cr)
      {
        parseSaoUflc(2, uiSymbol );
        psSaoLcuParam->subTypeIdx = uiSymbol;
        psSaoLcuParam->typeIdx += psSaoLcuParam->subTypeIdx;
      }
    }
  }
  else
  {
    psSaoLcuParam->length = 0;
  }
}

Void TDecSbac::parseSaoOneLcuInterleaving(Int rx, Int ry, SAOParam* pSaoParam, TComDataCU* pcCU, Int iCUAddrInSlice, Int iCUAddrUpInSlice, Int allowMergeLeft, Int allowMergeUp)
{
  Int iAddr = pcCU->getAddr();
  UInt uiSymbol;
  const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();
  for (Int iCompIdx=0; iCompIdx<numValidComp; iCompIdx++)
  {
    pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeUpFlag    = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeLeftFlag  = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].subTypeIdx     = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].typeIdx        = -1;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].offset[0]      = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].offset[1]      = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].offset[2]      = 0;
    pSaoParam->saoLcuParam[iCompIdx][iAddr].offset[3]      = 0;
  }

  const Bool bChroma = isChromaEnabled(pcCU->getPic()->getChromaFormat());

  if (pSaoParam->bSaoFlag[CHANNEL_TYPE_LUMA] || (bChroma && pSaoParam->bSaoFlag[CHANNEL_TYPE_CHROMA]) )
  {
    if (rx>0 && iCUAddrInSlice!=0 && allowMergeLeft)
    {
      parseSaoMerge(uiSymbol);
      pSaoParam->saoLcuParam[0][iAddr].mergeLeftFlag = (Bool)uiSymbol;
    }
    if (pSaoParam->saoLcuParam[0][iAddr].mergeLeftFlag==0)
    {
      if ((ry > 0) && (iCUAddrUpInSlice>=0) && allowMergeUp)
      {
        parseSaoMerge(uiSymbol);
        pSaoParam->saoLcuParam[0][iAddr].mergeUpFlag = (Bool)uiSymbol;
      }
    }
  }

  for (Int iCompIdx=0; iCompIdx<numValidComp; iCompIdx++)
  {
    if (pSaoParam->bSaoFlag[toChannelType(ComponentID(iCompIdx))])
    {
      if (rx>0 && iCUAddrInSlice!=0 && allowMergeLeft)
      {
        pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeLeftFlag = pSaoParam->saoLcuParam[0][iAddr].mergeLeftFlag;
      }
      else
      {
        pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeLeftFlag = 0;
      }

      if (pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeLeftFlag==0)
      {
        if ((ry > 0) && (iCUAddrUpInSlice>=0) && allowMergeUp)
        {
          pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeUpFlag = pSaoParam->saoLcuParam[0][iAddr].mergeUpFlag;
        }
        else
        {
          pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeUpFlag = 0;
        }
        if (!pSaoParam->saoLcuParam[iCompIdx][iAddr].mergeUpFlag)
        {
          pSaoParam->saoLcuParam[2][iAddr].typeIdx = pSaoParam->saoLcuParam[1][iAddr].typeIdx;
          parseSaoOffset(&(pSaoParam->saoLcuParam[iCompIdx][iAddr]), ComponentID(iCompIdx));
        }
        else
        {
          copySaoOneLcuParam(&pSaoParam->saoLcuParam[iCompIdx][iAddr], &pSaoParam->saoLcuParam[iCompIdx][iAddr-pSaoParam->numCuInWidth]);
        }
      }
      else
      {
        copySaoOneLcuParam(&pSaoParam->saoLcuParam[iCompIdx][iAddr],  &pSaoParam->saoLcuParam[iCompIdx][iAddr-1]);
      }
    }
    else
    {
      pSaoParam->saoLcuParam[iCompIdx][iAddr].typeIdx = -1;
      pSaoParam->saoLcuParam[iCompIdx][iAddr].subTypeIdx = 0;
    }
  }
}

#endif

/**
 - Initialize our contexts from the nominated source.
 .
 \param pSrc Contexts to be copied.
 */
Void TDecSbac::xCopyContextsFrom( TDecSbac* pSrc )
{
  memcpy(m_contextModels, pSrc->m_contextModels, m_numContextModels*sizeof(m_contextModels[0]));
}

Void TDecSbac::xCopyFrom( TDecSbac* pSrc )
{
  m_pcTDecBinIf->copyState( pSrc->m_pcTDecBinIf );

  m_uiLastQp           = pSrc->m_uiLastQp;
  xCopyContextsFrom( pSrc );

}

Void TDecSbac::load ( TDecSbac* pScr )
{
  xCopyFrom(pScr);
}

Void TDecSbac::loadContexts ( TDecSbac* pScr )
{
  xCopyContextsFrom(pScr);
}

#if RExt__NRCE2_RESIDUAL_DPCM
/** Performs CABAC decoding of the inter RDPCM mode
 * \param rTu current TU data structure
 * \param compID component identifier
 */
Void TDecSbac::parseInterRdpcmMode( TComTU &rTu, ComponentID compID )
{
  TComDataCU* cu = rTu.getCU();
  const UInt absPartIdx=rTu.GetAbsPartIdxTU(compID);
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt tuHeight = g_aucConvertToBit[rect.height];
  const UInt tuWidth  = g_aucConvertToBit[rect.width];
  UInt code = 0;

  assert(tuHeight == tuWidth);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__INTER_RDPCM_BITS, g_aucConvertToBit[g_uiMaxCUWidth>>rTu.GetTransformDepthTotal()]+2);
#endif

  m_pcTDecBinIf->decodeBin(code, m_interRdpcmFlagSCModel.get (0, toChannelType(compID), 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
  if(code == 0)
  {
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
    cu->setInterRdpcmModePartRange( DPCM_OFF, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
#else
    cu->setInterRdpcmModeSubParts( DPCM_OFF, compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif
  }
  else
  {
    m_pcTDecBinIf->decodeBin(code, m_interRdpcmDirSCModel.get (0, toChannelType(compID), 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
    if(code == 0)
    {
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
      cu->setInterRdpcmModePartRange( DPCM_HOR, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
#else
      cu->setInterRdpcmModeSubParts( DPCM_HOR, compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif
    }
    else
    {
#if (RExt__SQUARE_TRANSFORM_CHROMA_422 != 0)
      cu->setInterRdpcmModePartRange( DPCM_VER, compID, absPartIdx, rTu.GetAbsPartIdxNumParts(compID));
#else
      cu->setInterRdpcmModeSubParts( DPCM_VER, compID, absPartIdx, rTu.GetTransformDepthTotalAdj(compID));
#endif
    }
  }
}
#endif


//! \}
