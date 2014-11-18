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

/** \file     TEncSearch.h
    \brief    encoder search class (header)
*/

#ifndef __TENCSEARCH__
#define __TENCSEARCH__

// Include files
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TLibCommon/TComPattern.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComRectangle.h"
#include "TEncEntropy.h"
#include "TEncSbac.h"
#include "TEncCfg.h"
#include "TLibCommon/TComHash.h"

// ====================================================================================================================
// Constants 
// ====================================================================================================================

#if SCM_S0067_ENCODER_IMPROVEMENTS
#define CHROMA_REFINEMENT_CANDIDATES  8
#else
#define CHROMA_REFINEMENT_CANDIDATES  4
#endif

//! \ingroup TLibEncoder
//! \{

class TEncCu;

#define INTRABC_HASH_DEPTH                     1  ////< Currently used only for 8x8
#define INTRABC_HASH_TABLESIZE                (1 << 16)

struct IntraBCHashNode
{
  Int pos_X;
  Int pos_Y;
  IntraBCHashNode * next;
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const UInt MAX_NUM_REF_LIST_ADAPT_SR=2;
static const UInt MAX_IDX_ADAPT_SR=33;
static const UInt NUM_MV_PREDICTORS=3;

/// encoder search class
class TEncSearch : public TComPrediction
{
private:
  TCoeff**        m_ppcQTTempCoeff[MAX_NUM_COMPONENT /* 0->Y, 1->Cb, 2->Cr*/];
  TCoeff*         m_pcQTTempCoeff[MAX_NUM_COMPONENT];
#if ADAPTIVE_QP_SELECTION
  TCoeff**        m_ppcQTTempArlCoeff[MAX_NUM_COMPONENT];
  TCoeff*         m_pcQTTempArlCoeff[MAX_NUM_COMPONENT];
#endif
  UChar*          m_puhQTTempTrIdx;
  UChar*          m_puhQTTempCbf[MAX_NUM_COMPONENT];

  TComYuv*        m_pcQTTempTComYuv;
  TComYuv         m_tmpYuvPred; // To be used in xGetInterPredictionError() to avoid constant memory allocation/deallocation

  Char*           m_phQTTempCrossComponentPredictionAlpha[MAX_NUM_COMPONENT];
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_COMPONENT];
  TCoeff*         m_pcQTTempTUCoeff[MAX_NUM_COMPONENT];
  UChar*          m_puhQTTempTransformSkipFlag[MAX_NUM_COMPONENT];
  TComYuv         m_pcQTTempTransformSkipTComYuv;
#if ADAPTIVE_QP_SELECTION
  TCoeff*         m_ppcQTTempTUArlCoeff[MAX_NUM_COMPONENT];
#endif

  IntraBCHashNode***      m_pcIntraBCHashTable;                 ///< The hash table used for Intra BC search

protected:
  // interface to option
  TEncCfg*        m_pcEncCfg;

  // interface to classes
  TComTrQuant*    m_pcTrQuant;
  TComRdCost*     m_pcRdCost;
  TEncEntropy*    m_pcEntropyCoder;

  // ME parameters
  Int             m_iSearchRange;
  Int             m_bipredSearchRange; // Search range for bi-prediction
  Int             m_iFastSearch;
  Int             m_aaiAdaptSR[MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];
  TComMv          m_cSrchRngLT;
  TComMv          m_cSrchRngRB;
  TComMv          m_acMvPredictors[NUM_MV_PREDICTORS]; // Left, Above, AboveRight. enum MVP_DIR first NUM_MV_PREDICTORS entries are suitable for accessing.

  // RD computation
  TEncSbac***     m_pppcRDSbacCoder;
  TEncSbac*       m_pcRDGoOnSbacCoder;
  DistParam       m_cDistParam;

  // Misc.
  Pel*            m_pTempPel;
  const UInt*     m_puiDFilter;
  Int             m_iMaxDeltaQP;

  // AMVP cost computation
  // UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS];
  UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds

  RefPicList      m_currRefPicList;
  Int             m_currRefPicIndex;
  Bool            m_bSkipFracME;
  TComMv          m_integerMv2Nx2N[NUM_REF_PIC_LIST_01][MAX_NUM_REF];
#if SCM_S0067_ENCODER_IMPROVEMENTS
  TComMv          m_acBVs[SCM_S0067_NUM_CANDIDATES];
  UInt            m_uiNumBVs, m_uiNumBV16s;
  Distortion      m_lastCandCost;
#endif
  Bool            m_bBestScanRotationMode;
  Pel*            m_paOriginalLevel;
  Pel*            m_paBestLevel[MAX_NUM_COMPONENT];
  UChar*          m_paBestSPoint;
  TCoeff*         m_paBestRun;
#if SCM_S0258_PLT_ESCAPE_SIG
  UChar*          m_paBestEscapeFlag;
#endif
public:
  TEncSearch();
  virtual ~TEncSearch();

  TComYuv* getTmpYuvPred() {return &m_tmpYuvPred;}

  Void init(  TEncCfg*      pcEncCfg,
            TComTrQuant*  pcTrQuant,
            Int           iSearchRange,
            Int           bipredSearchRange,
            Int           iFastSearch,
            Int           iMaxDeltaQP,
            TEncEntropy*  pcEntropyCoder,
            TComRdCost*   pcRdCost,
            TEncSbac***   pppcRDSbacCoder,
            TEncSbac*     pcRDGoOnSbacCoder );

protected:

  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion  xPatternRefinement( TComPattern* pcPatternKey,
                                  TComMv baseRefMv,
                                  Int iFrac, TComMv& rcMvFrac, Bool bAllowUseOfHadamard
                                 );

  typedef struct
  {
    Pel*        piRefY;
    Int         iYStride;
    Int         iBestX;
    Int         iBestY;
    UInt        uiBestRound;
    UInt        uiBestDistance;
    Distortion  uiBestSad;
    UChar       ucPointNr;
  } IntTZSearchStruct;

  // sub-functions for ME
  __inline Void xTZSearchHelp         ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance );
  __inline Void xTZ2PointSearch       ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB );
  __inline Void xTZ8PointSquareSearch ( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist );
  __inline Void xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStrukt, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist, Bool bSkipLeftDist2 = false, Bool bSkipTopDist2 = false );

  Void xGetInterPredictionError( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Distortion& ruiSAD, Bool Hadamard );

public:
  Void  preestChromaPredMode    ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv );

  Void  estIntraPredQT          ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv,
                                  Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                  Distortion& ruiDistC,
                                  Bool        bLumaOnly
                                  DEBUG_STRING_FN_DECLARE(sDebug));

  Void  estIntraPredChromaQT    ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv,
                                  Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                  Distortion  uiPreCalcDistC
                                  DEBUG_STRING_FN_DECLARE(sDebug));

  Void  estIntraPredQTCT        ( TComDataCU* pcCU,
                                   TComYuv*    pcOrgYuv,
                                   TComYuv*    pcPredYuv,
                                   TComYuv*    pcResiYuv,
                                   TComYuv*    pcRecoYuv
                                   DEBUG_STRING_FN_DECLARE(sDebug)
                                 );

  Void  estIntraPredQTWithModeReuse ( TComDataCU* pcCU,
                                      TComYuv*    pcOrgYuv,
                                      TComYuv*    pcPredYuv,
                                      TComYuv*    pcResiYuv,
                                      TComYuv*    pcRecoYuv,
                                      Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      Distortion& ruiDistC,
                                      Bool        bLumaOnly
                                    );
  Void  estIntraPredChromaQTWithModeReuse ( TComDataCU* pcCU,
                                            TComYuv*    pcOrgYuv,
                                            TComYuv*    pcPredYuv,
                                            TComYuv*    pcResiYuv,
                                            TComYuv*    pcRecoYuv,
                                            Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                            Distortion  uiPreCalcDistC
                                          );

  /// encoder estimation - inter prediction (non-skip)
  Void predInterSearch          ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv
                                  DEBUG_STRING_FN_DECLARE(sDebug),
                                  Bool        bUseRes = false
#if AMP_MRG
                                 ,Bool        bUseMRG = false
#endif
                                );

  Bool predIntraBCSearch        ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*&   rpcPredYuv,
                                  TComYuv*&   rpcResiYuv,
                                  TComYuv*&   rpcRecoYuv
                                  DEBUG_STRING_FN_DECLARE(sDebug),
                                  Bool        bUse1DSearchFor8x8,
                                  Bool        bUseRes
#if SCM_S0067_ENCODER_IMPROVEMENTS
                                  , Bool      testOnlyPred
#endif
                                );

  Void xIntraBlockCopyEstimation( TComDataCU*  pcCU,
                                  TComYuv*     pcYuvOrg,
                                  Int          iPartIdx,
                                  TComMv*      pcMvPred,
                                  TComMv&      rcMv,
                                  Distortion&  ruiCost,
                                  Bool         bUse1DSearchFor8x8
#if SCM_S0067_ENCODER_IMPROVEMENTS
                                  , Bool       testOnlyPred
#endif
                                );

  Void addToSortList            ( list<BlockHash>& listBlockHash,
                                  list<Int>& listCost,
                                  Int cost,
                                  const BlockHash& blockHash
                                );

  Distortion getSAD             ( Pel* pRef,
                                  Int refStride,
                                  Pel* pCurr,
                                  Int currStride,
                                  Int width,
                                  Int height
                                );

  Bool predInterHashSearch      ( TComDataCU* pcCU,
                                  TComYuv* pcOrg,
                                  TComYuv*& rpcPredYuv,
                                  Bool& isPerfectMatch
                                );

  Bool xHashInterEstimation     ( TComDataCU* pcCU,
                                  Int width,
                                  Int height,
                                  RefPicList& bestRefPicList,
                                  Int& bestRefIndex,
                                  TComMv& bestMv,
                                  TComMv& bestMvd,
                                  Int& bestMVPIndex,
                                  Bool& isPerfectMatch
                                 );
  
  Int  xHashInterPredME         ( TComDataCU* pcCU,
                                  Int width,
                                  Int height,
                                  RefPicList currRefPicList,
                                  Int currRefPicIndex,
                                  TComMv bestMv[5]
                                );

  Void selectMatchesInter       ( TComDataCU* pcCU,
                                  const MapIterator& itBegin,
                                  Int count,
                                  list<BlockHash>& vecBlockHash,
                                  const BlockHash& currBlockHash
                                 );

  Void xSetIntraSearchRange     ( TComDataCU*   pcCU,
                                  TComMv&       cMvPred,
                                  UInt          uiPartAddr,
                                  Int           iRoiWidth,
                                  Int           iRoiHeight,
                                  TComMv&       rcMvSrchRngLT,
                                  TComMv&       rcMvSrchRngRB );

  Bool xCIPIntraSearchPruning(    TComDataCU*   pcCU,
                                  Int           relX,
                                  Int           relY,
                                  Int           roiWidth,
                                  Int           roiHeight);

#if SCM_S0220_IBC_PRED_CONSTRAINT
  Bool isValidIntraBCSearchArea(  TComDataCU*   pcCU,
#if SCM_420_IBC_BUGFIX
                                  Int           iPartIdx,
#endif
                                  Int           predX,
                                  Int           ROIStartX,
                                  Int           predY,
                                  Int           ROIStartY,
                                  Int           roiWidth,
                                  Int           roiHeight,
                                  Int           uiPartOffset)
  {
    const Int uiMaxCuWidth   = pcCU->getSlice()->getSPS()->getMaxCUWidth();
    const Int uiMaxCuHeight  = pcCU->getSlice()->getSPS()->getMaxCUHeight();
    const Int  cuPelX        = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiPartOffset ] ];
    const Int  cuPelY        = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiPartOffset ] ];
    Int uiRefCuX             = (cuPelX + predX + roiWidth  - 1)/uiMaxCuWidth;
    Int uiRefCuY             = (cuPelY + predY + roiHeight - 1)/uiMaxCuHeight;
    Int uiCuPelX             = (cuPelX / uiMaxCuWidth);
    Int uiCuPelY             = (cuPelY / uiMaxCuHeight);
    
    if(((Int)(uiRefCuX - uiCuPelX) > (Int)((uiCuPelY - uiRefCuY))))
    {
      return false;
    }

#if SCM_420_IBC_BUGFIX
    // check boundary
    if ( pcCU->getWidth( 0 ) == 8 && pcCU->getPartitionSize( 0 ) != SIZE_2Nx2N && pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() != CHROMA_444 )
    {
      if ( pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() == CHROMA_420 )
      {
        if ( (pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 3) ||
             (pcCU->getPartitionSize( 0 ) == SIZE_2NxN && iPartIdx == 1) ||
             (pcCU->getPartitionSize( 0 ) == SIZE_Nx2N && iPartIdx == 1) )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 8 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
      }
      else if ( pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() == CHROMA_422 )
      {
        if ( pcCU->getPartitionSize( 0 ) == SIZE_Nx2N && iPartIdx == 1 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 8 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
        else if ( pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 1 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 4 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
        else if ( pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 3 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + 4 + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 4 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
      }
    }
#endif

    return (!pcCU->getSlice()->getPPS()->getConstrainedIntraPred())        ||
           (pcCU->getSlice()->getSliceType() == I_SLICE)                   ||
           xCIPIntraSearchPruning(pcCU, predX + ROIStartX, predY + ROIStartY, roiWidth, roiHeight);
  }
#else
  Bool isValidIntraBCSearchArea(  TComDataCU*   pcCU,
#if SCM_420_IBC_BUGFIX
                                  Int           iPartIdx,
                                  Int           predX,
                                  Int           ROIStartX,
                                  Int           predY,
                                  Int           ROIStartY,
#else
                                  Int           relX,
                                  Int           relY,
#endif
                                  Int           roiWidth,
                                  Int           roiHeight)
  {
#if SCM_420_IBC_BUGFIX
    // check boundary
    if ( pcCU->getWidth( 0 ) == 8 && pcCU->getPartitionSize( 0 ) != SIZE_2Nx2N && pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() != CHROMA_444 )
    {
      if ( pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() == CHROMA_420 )
      {
        if ( (pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 3) ||
             (pcCU->getPartitionSize( 0 ) == SIZE_2NxN && iPartIdx == 1) ||
             (pcCU->getPartitionSize( 0 ) == SIZE_Nx2N && iPartIdx == 1) )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 8 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
      }
      else if ( pcCU->getSlice()->getPic()->getPicYuvOrg()->getChromaFormat() == CHROMA_422 )
      {
        if ( pcCU->getPartitionSize( 0 ) == SIZE_Nx2N && iPartIdx == 1 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 8 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
        else if ( pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 1 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 4 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
        else if ( pcCU->getPartitionSize( 0 ) == SIZE_NxN && iPartIdx == 3 )
        {
          Int cuStartX = pcCU->getCUPelX();
          Int cuStartY = pcCU->getCUPelY();
          Int refStartX = cuStartX + predX;
          Int refStartY = cuStartY + 4 + predY;
          Int refEndX = refStartX + 8 - 1;
          Int refEndY = refStartY + 4 - 1;
          if ( refStartX < 0 || refStartY < 0 ||
               refEndX >= pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ||
               refEndY >= pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() )
          {
            return false;
          }
        }
      }
    }
#endif

    return (!pcCU->getSlice()->getPPS()->getConstrainedIntraPred())        ||
           (pcCU->getSlice()->getSliceType() == I_SLICE)                   ||
#if SCM_420_IBC_BUGFIX
           xCIPIntraSearchPruning(pcCU, predX + ROIStartX, predY + ROIStartY, roiWidth, roiHeight);
#else
           xCIPIntraSearchPruning(pcCU, relX, relY, roiWidth, roiHeight);
#endif
  }
#endif 

  Void xIntraBCSearchMVCandUpdate(Distortion uiSad, Int x, Int y, Distortion* uiSadBestCand, TComMv* cMVCand);
  
  Int xIntraBCSearchMVChromaRefine( TComDataCU *pcCU,
                                    Int         iRoiWidth,
                                    Int         iRoiHeight,
                                    Int         cuPelX,
                                    Int         cuPelY,
                                    Distortion* uiSadBestCand, 
                                    TComMv*     cMVCand, 
                                    UInt        uiPartAddr);

  Void xIntraPatternSearch      ( TComDataCU*  pcCU,
#if SCM_420_IBC_BUGFIX
                                  Int          iPartIdx,
#endif
                                  UInt         uiPartAddr,
                                  TComPattern* pcPatternKey,
                                  Pel*         piRefY,
                                  Int          iRefStride,
                                  TComMv*      pcMvSrchRngLT,
                                  TComMv*      pcMvSrchRngRB,
                                  TComMv&      rcMv,
                                  Distortion&  ruiSAD,
                                  Int          iRoiWidth,
                                  Int          iRoiHeight,
                                  TComMv*      mvPreds, 
                                  Bool         bUse1DSearchFor8x8
#if SCM_S0067_ENCODER_IMPROVEMENTS
                                  , Bool       testOnlyPred
#endif
                                );

  /// encode residual and compute rd-cost for inter mode
  Void encodeResAndCalcRdInterCU( TComDataCU* pcCU,
                                  TComYuv*    pcYuvOrg,
                                  TComYuv*    pcYuvPred,
                                  TComYuv*    pcYuvResi,
                                  TComYuv*    pcYuvResiBest,
                                  TComYuv*    pcYuvRec,
                                  Bool        bSkipRes,
                                  TComYuv*    pcYuvNoCorrResi
                                  DEBUG_STRING_FN_DECLARE(sDebug) );

  /// set ME search range
  Void setAdaptiveSearchRange   ( Int iDir, Int iRefIdx, Int iSearchRange) { assert(iDir < MAX_NUM_REF_LIST_ADAPT_SR && iRefIdx<Int(MAX_IDX_ADAPT_SR)); m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }

  Void xEncPCM    (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPCM, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, const ComponentID compID );
  Void IPCMSearch (TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* rpcPredYuv, TComYuv* rpcResiYuv, TComYuv* rpcRecoYuv );
  Void PLTSearch  (TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv,TComYuv *& rpcResiBestYuv, TComYuv*& rpcRecoYuv, Bool bCheckPLTSharingMode);
  Void deriveRunAndCalcBits( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcRecoYuv, UInt& uiMinBits, Bool bReset, PLTScanMode pltScanMode);

  Int xIntraBCHashTableIndex  ( TComDataCU* pcCU,
                                Int pos_X,
                                Int pos_Y,
                                Int width,
                                Int height,
                                Bool isRec
                              );

  Void xIntraBCHashSearch     ( TComDataCU* pcCU,
                                TComYuv* pcYuvOrg,
                                Int iPartIdx,
                                TComMv* pcMvPred,
                                TComMv& rcMv,
                                UInt uiIntraBCECost
                              );

  Void xIntraBCHashTableUpdate( TComDataCU* pcCU,
                                Bool isRec
                              );

  Void xClearIntraBCHashTable();

  Void setHashLinklist        ( IntraBCHashNode*& HashLinklist,
                                UInt uiDepth,
                                UInt uiHashIdx
                              );

  IntraBCHashNode* getHashLinklist( UInt uiDepth, Int iHashIdx ) { return m_pcIntraBCHashTable[uiDepth][iHashIdx]; }


protected:

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  Void  xEncSubdivCbfQT           ( TComTU      &rTu,
                                    Bool         bLuma,
                                    Bool         bChroma );

  Void  xEncCoeffQT               ( TComTU &rTu,
                                    ComponentID  component,
                                    Bool         bRealCoeff );
  Void  xEncIntraHeader           ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLuma,
                                    Bool         bChroma );
  UInt  xGetIntraBitsQT           ( TComTU &rTu,
                                    Bool         bLuma,
                                    Bool         bChroma,
                                    Bool         bRealCoeff );

  UInt  xGetIntraBitsQTChroma    ( TComTU &rTu,
                                   ComponentID compID,
                                   Bool          bRealCoeff );

  Void  xIntraCodingTUBlock       (       TComYuv*      pcOrgYuv,
                                          TComYuv*      pcPredYuv,
                                          TComYuv*      pcResiYuv,
                                          Pel           resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    const Bool          checkCrossCPrediction,
                                          Distortion&   ruiDist,
                                    const ComponentID   compID,
                                          TComTU        &rTu
                                    DEBUG_STRING_FN_DECLARE(sTest)
                                         ,Int           default0Save1Load2 = 0
                                   );

  Void  xIntraCodingTUBlockCSC    (       TComYuv*      pcResiYuv,
                                          Pel           resiLuma[MAX_CU_SIZE * MAX_CU_SIZE],
                                    const Bool          checkDecorrelation,
                                    const ComponentID   compID,
                                          TComTU        &rTu,
                                          QpParam       &cQP
                                          DEBUG_STRING_FN_DECLARE(sDebug)
                                  );

  Void  xRecurIntraCodingQT       ( Bool        bLumaOnly,
                                    TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDistY,
                                    Distortion& ruiDistC,
#if HHI_RQT_INTRA_SPEEDUP
                                   Bool         bCheckFirst,
#endif
                                   Double&      dRDCost,
                                   TComTU      &rTu
                                   DEBUG_STRING_FN_DECLARE(sDebug));

  Void  xSetIntraResultQT         ( Bool         bLumaOnly,
                                    TComYuv*     pcRecoYuv,
                                    TComTU &rTu);

  Void xStoreCrossComponentPredictionResult  (       Pel    *pResiLuma,
                                               const Pel    *pBestLuma,
                                                     TComTU &rTu,
                                               const Int     xOffset,
                                               const Int     yOffset,
                                               const Int     strideResi,
                                               const Int     strideBest );

  Char xCalcCrossComponentPredictionAlpha    (       TComTU &rTu,
                                               const ComponentID compID,
                                               const Pel*        piResiL,
                                               const Pel*        piResiC,
                                               const Int         width,
                                               const Int         height,
                                               const Int         strideL,
                                               const Int         strideC );

  Void  xRecurIntraChromaCodingQT ( TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDist,
                                    TComTU      &rTu
                                    DEBUG_STRING_FN_DECLARE(sDebug));

  Void  xRecurIntraCodingQTCSC    ( TComYuv*     pcOrgYuv,
                                    TComYuv*     pcPredYuv,
                                    TComYuv*     pcResiYuv,
                                    Distortion&  uiPUDistY,
                                    Distortion&  uiPUDistC,
                                    Double&      dPUCost,
                                    TComTU&      rTu,
                                    Bool         bTestMaxTUSize
                                    DEBUG_STRING_FN_DECLARE(sDebug)
                                  );

  Void  xSetIntraResultChromaQT   ( TComYuv*    pcRecoYuv, TComTU &rTu);

  Void  xStoreIntraResultQT       ( const ComponentID first, const ComponentID lastIncl, TComTU &rTu);
  Void  xLoadIntraResultQT        ( const ComponentID first, const ComponentID lastIncl, TComTU &rTu);


  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  Void xEstimateMvPredAMVP        ( TComDataCU* pcCU,
                                    TComYuv*    pcOrgYuv,
                                    UInt        uiPartIdx,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    TComMv&     rcMvPred,
                                    Bool        bFilled = false
                                  , Distortion* puiDistBiP = NULL
                                     );

  Void xCheckBestMVP              ( TComDataCU* pcCU,
                                    RefPicList  eRefPicList,
                                    TComMv      cMv,
                                    TComMv&     rcMvPred,
                                    Int&        riMVPIdx,
                                    UInt&       ruiBits,
                                    Distortion& ruiCost );

  Distortion xGetTemplateCost    ( TComDataCU*  pcCU,
                                    UInt        uiPartIdx,
                                    UInt        uiPartAddr,
                                    TComYuv*    pcOrgYuv,
                                    TComYuv*    pcTemplateCand,
                                    TComMv      cMvCand,
                                    Int         iMVPIdx,
                                    Int         iMVPNum,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    Int         iSizeX,
                                    Int         iSizeY
                                   );


  Void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  UInt xGetMvpIdxBits             ( Int iIdx, Int iNum );
  Void xGetBlkBits                ( PartSize  eCUMode, Bool bPSlice, Int iPartIdx,  UInt uiLastMode, UInt uiBlkBit[3]);

  Void xMergeEstimation           ( TComDataCU*  pcCU,
                                    TComYuv*     pcYuvOrg,
                                    Int          iPartIdx,
                                    UInt&        uiInterDir,
                                    TComMvField* pacMvField,
                                    UInt&        uiMergeIndex,
                                    Distortion&  ruiCost,
                                    TComMvField* cMvFieldNeighbours,
                                    UChar*       uhInterDirNeighbours,
                                    Int&         numValidMergeCand
                                   );

  Void xRestrictBipredMergeCand   ( TComDataCU*     pcCU,
                                    UInt            puIdx,
                                    TComMvField*    mvFieldNeighbours,
                                    UChar*          interDirNeighbours,
                                    Int             numValidMergeCand );


  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  Void xMotionEstimation          ( TComDataCU*  pcCU,
                                    TComYuv*     pcYuvOrg,
                                    Int          iPartIdx,
                                    RefPicList   eRefPicList,
                                    TComMv*      pcMvPred,
                                    Int          iRefIdxPred,
                                    TComMv&      rcMv,
                                    UInt&        ruiBits,
                                    Distortion&  ruiCost,
                                    Bool         bBi = false  );

  Void xTZSearch                  ( TComDataCU*  pcCU,
                                    TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvSrchRngLT,
                                    TComMv*      pcMvSrchRngRB,
                                    TComMv&      rcMv,
                                    Distortion&  ruiSAD,
                                    const TComMv *pIntegerMv2Nx2NPred
                                    );

  Void xTZSearchSelective         ( TComDataCU*  pcCU,
                                    TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvSrchRngLT,
                                    TComMv*      pcMvSrchRngRB,
                                    TComMv&      rcMv,
                                    Distortion&  ruiSAD,
                                    const TComMv *pIntegerMv2Nx2NPred
                                    );

  Void xSetSearchRange            ( TComDataCU*  pcCU,
                                    TComMv&      cMvPred,
                                    Int          iSrchRng,
                                    TComMv&      rcMvSrchRngLT,
                                    TComMv&      rcMvSrchRngRB );

  Void xPatternSearchFast         ( TComDataCU*  pcCU,
                                    TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvSrchRngLT,
                                    TComMv*      pcMvSrchRngRB,
                                    TComMv&      rcMv,
                                    Distortion&  ruiSAD,
                                    const TComMv* pIntegerMv2Nx2NPred
                                  );

  Void xPatternSearch             ( TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvSrchRngLT,
                                    TComMv*      pcMvSrchRngRB,
                                    TComMv&      rcMv,
                                    Distortion&  ruiSAD );

  Void xPatternSearchFracDIF      (
                                    Bool         bIsLosslessCoded,
#if SCM_S0085_ADAPTIVE_MV_RESOLUTION
                                    TComDataCU*  pcCU,
#endif
                                    TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvInt,
                                    TComMv&      rcMvHalf,
                                    TComMv&      rcMvQter,
                                    Distortion&  ruiCost,
                                    Bool         biPred
                                   );

  Void xExtDIFUpSamplingH( TComPattern* pcPattern, Bool biPred  );
  Void xExtDIFUpSamplingQ( TComPattern* pcPatternKey, TComMv halfPelRef, Bool biPred );

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------


  Void xEncodeResidualQT( const ComponentID compID, TComTU &rTu );
  Void xEstimateResidualQT( TComYuv* pcResi, Double &rdCost, UInt &ruiBits, Distortion &ruiDist, Distortion *puiZeroDist, TComTU &rTu DEBUG_STRING_FN_DECLARE(sDebug), TComYuv* pcOrgResi = NULL);
  Void xSetResidualQTData( TComYuv* pcResi, Bool bSpatial, TComTU &rTu  );

  UInt  xModeBitsIntra ( TComDataCU* pcCU, UInt uiMode, UInt uiPartOffset, UInt uiDepth, UInt uiInitTrDepth, const ChannelType compID );
  UInt  xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList );

  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

  Void xAddSymbolBitsInter       ( TComDataCU*   pcCU,
                                   UInt          uiQp,
                                   UInt          uiTrMode,
                                   UInt&         ruiBits);

  Void  setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur );
  inline  Void  setDistParamComp( ComponentID compIdx )  { m_cDistParam.compIdx = compIdx; }

#if SCM_S0156_PLT_ENC_RDO
  Void   xDeriveRun (TComDataCU* pcCU, Pel* pOrg [3],  Pel *pPalette [3],  Pel* pValue, UChar* pSPoint, Pel *pRecoValue[], Pel *pPixelRec[], TCoeff* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize);
  Double xGetRunBits(TComDataCU* pcCU, Pel *pValue, UInt uiStartPos, UInt uiRun, PLTRunMode cPltRunMode);
#endif
};// END CLASS DEFINITION TEncSearch

//! \}

#endif // __TENCSEARCH__
