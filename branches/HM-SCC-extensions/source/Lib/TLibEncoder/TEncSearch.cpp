/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TEncSearch.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/Debug.h"
#include <math.h>
#include <limits>


//! \ingroup TLibEncoder
//! \{

static const TComMv s_acMvRefineH[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const TComMv s_acMvRefineQ[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const UInt s_auiDFilter[9] =
{
  0, 1, 0,
  2, 3, 2,
  0, 1, 0
};

static Void offsetSubTUCBFs(TComTU &rTu, const ComponentID compID)
{
        TComDataCU *pcCU              = rTu.getCU();
  const UInt        uiTrDepth         = rTu.GetTransformDepthRel();
  const UInt        uiAbsPartIdx      = rTu.GetAbsPartIdxTU(compID);
  const UInt        partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

  //move the CBFs down a level and set the parent CBF

  UChar subTUCBF[2];
  UChar combinedSubTUCBF = 0;

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);

    subTUCBF[subTU]   = pcCU->getCbf(subTUAbsPartIdx, compID, uiTrDepth);
    combinedSubTUCBF |= subTUCBF[subTU];
  }

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);
    const UChar compositeCBF = (subTUCBF[subTU] << 1) | combinedSubTUCBF;

    pcCU->setCbfPartRange((compositeCBF << uiTrDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU);
  }
}


TEncSearch::TEncSearch()
: m_puhQTTempTrIdx(NULL)
, m_pcQTTempTComYuv(NULL)
, m_pcEncCfg (NULL)
, m_pcTrQuant (NULL)
, m_pcRdCost (NULL)
, m_pcEntropyCoder (NULL)
, m_iSearchRange (0)
, m_bipredSearchRange (0)
, m_iFastSearch (0)
, m_pppcRDSbacCoder (NULL)
, m_pcRDGoOnSbacCoder (NULL)
, m_pTempPel (NULL)
, m_puiDFilter (NULL)
, m_isInitialized (false)
{
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    m_ppcQTTempCoeff[ch]                           = NULL;
    m_pcQTTempCoeff[ch]                            = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]                        = NULL;
    m_pcQTTempArlCoeff[ch]                         = NULL;
#endif
    m_puhQTTempCbf[ch]                             = NULL;
    m_phQTTempCrossComponentPredictionAlpha[ch]    = NULL;
    m_pSharedPredTransformSkip[ch]                 = NULL;
    m_pcQTTempTUCoeff[ch]                          = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = NULL;
#endif
    m_puhQTTempTransformSkipFlag[ch]               = NULL;
#if SCM_U0106_ACT_TU_SIG
    m_pcACTTempTUCoeff[ch]                         = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcACTTempTUArlCoeff[ch]                     = NULL;
#endif
#endif
  }

  m_pcIntraBCHashTable = NULL;
  m_pcIntraBCHashTable = new IntraBCHashNode**[INTRABC_HASH_DEPTH];
  for(int i = 0; i < INTRABC_HASH_DEPTH; i++)
  {
    m_pcIntraBCHashTable[i] = new IntraBCHashNode*[INTRABC_HASH_TABLESIZE];
    memset(m_pcIntraBCHashTable[i], 0, INTRABC_HASH_TABLESIZE * sizeof(IntraBCHashNode*));
  }

#if SCM_U0106_ACT_TU_SIG
  m_pcQTTempTComYuvCS                              = NULL;
  m_pcNoCorrYuvTmp                                 = NULL;
  m_puhQTTempACTFlag                               = NULL;
#endif

  m_paOriginalLevel  = (Pel*)xMalloc(Pel , MAX_CU_SIZE * MAX_CU_SIZE);

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt group=0, k=0, j, uiTotal=32*32;

  while (k<uiTotal)
  {
    if (group<2)
    {
      m_runGolombGroups[k]=group+1;
      k++; group++;
      if (k>=uiTotal)
      {
        break;
      }
    }
    else
    {
      for (j=0; j<(1<<(group-1)); j++)
      {
        m_runGolombGroups[k+j]=2*group;
      }
      k+=(1<<(group-1));
      group++;
    }
    if (k>=uiTotal)
    {
      break;
    }
  }
#endif

  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    m_paBestLevel[i]  = (Pel*)xMalloc(Pel , MAX_CU_SIZE * MAX_CU_SIZE);
  }
  m_paBestSPoint = (UChar*)xMalloc(UChar , MAX_CU_SIZE * MAX_CU_SIZE);
  m_paBestRun    = (TCoeff*)xMalloc(TCoeff , MAX_CU_SIZE * MAX_CU_SIZE);
  m_paBestEscapeFlag = (UChar*)xMalloc(UChar , MAX_CU_SIZE * MAX_CU_SIZE);

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    m_paLevelStoreRD[i]  = (Pel*)xMalloc(Pel , MAX_CU_SIZE * MAX_CU_SIZE);
  }
  m_paSPointStoreRD = (UChar*)xMalloc(UChar , MAX_CU_SIZE * MAX_CU_SIZE);
  m_paRunStoreRD    = (TCoeff*)xMalloc(TCoeff , MAX_CU_SIZE * MAX_CU_SIZE);
  m_paEscapeFlagStoreRD = (UChar*)xMalloc(UChar , MAX_CU_SIZE * MAX_CU_SIZE);
#endif
  for (Int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (Int));
  }
  for (Int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (UInt) );
  }

  setWpScalingDistParam( NULL, -1, REF_PIC_LIST_X );
}


Void TEncSearch::destroy()
{
  assert (m_isInitialized);
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  if ( m_pcEncCfg )
  {
    const UInt uiNumLayersAllocated = m_pcEncCfg->getQuadtreeTULog2MaxSize()-m_pcEncCfg->getQuadtreeTULog2MinSize()+1;

    for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for (UInt layer = 0; layer < uiNumLayersAllocated; layer++)
      {
        delete[] m_ppcQTTempCoeff[ch][layer];
#if ADAPTIVE_QP_SELECTION
        delete[] m_ppcQTTempArlCoeff[ch][layer];
#endif
      }
      delete[] m_ppcQTTempCoeff[ch];
      delete[] m_pcQTTempCoeff[ch];
      delete[] m_puhQTTempCbf[ch];
#if ADAPTIVE_QP_SELECTION
      delete[] m_ppcQTTempArlCoeff[ch];
      delete[] m_pcQTTempArlCoeff[ch];
#endif
    }

    for( UInt layer = 0; layer < uiNumLayersAllocated; layer++ )
    {
      m_pcQTTempTComYuv[layer].destroy();
#if SCM_U0106_ACT_TU_SIG
      m_pcQTTempTComYuvCS[layer].destroy();
      m_pcNoCorrYuvTmp[layer].destroy();
#endif
    }
  }

  delete[] m_puhQTTempTrIdx;
  delete[] m_pcQTTempTComYuv;
#if SCM_U0106_ACT_TU_SIG
  delete[] m_puhQTTempACTFlag;
  delete[] m_pcQTTempTComYuvCS;
  delete[] m_pcNoCorrYuvTmp;
#endif

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    delete[] m_pSharedPredTransformSkip[ch];
    delete[] m_pcQTTempTUCoeff[ch];
#if ADAPTIVE_QP_SELECTION
    delete[] m_ppcQTTempTUArlCoeff[ch];
#endif
    delete[] m_phQTTempCrossComponentPredictionAlpha[ch];
    delete[] m_puhQTTempTransformSkipFlag[ch];
#if SCM_U0106_ACT_TU_SIG
    delete[] m_pcACTTempTUCoeff[ch];
#if ADAPTIVE_QP_SELECTION
    delete[] m_ppcACTTempTUArlCoeff[ch];
#endif
#endif
  }
  m_pcQTTempTransformSkipTComYuv.destroy();
#if SCM_U0106_ACT_TU_SIG
  m_pcACTTempTransformSkipTComYuv.destroy();
#endif

  if(m_pcIntraBCHashTable)
  {
    for(int iDepth = 0; iDepth < INTRABC_HASH_DEPTH; iDepth++)
    {
      if(m_pcIntraBCHashTable[iDepth])
      {
        delete[]  m_pcIntraBCHashTable[iDepth];
      }
    }
    delete[] m_pcIntraBCHashTable;
  }

  m_pcIntraBCHashTable = NULL;


  m_tmpYuvPred.destroy();
  if (m_paOriginalLevel)  { xFree(m_paOriginalLevel);  m_paOriginalLevel=NULL;  }
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    if (m_paBestLevel[i])      { xFree(m_paBestLevel[i]);  m_paBestLevel[i]=NULL;  }
  }
  if (m_paBestSPoint)     { xFree(m_paBestSPoint); m_paBestSPoint=NULL; }
  if (m_paBestRun)        { xFree(m_paBestRun);    m_paBestRun=NULL;    }
  if (m_paBestEscapeFlag) { xFree(m_paBestEscapeFlag);    m_paBestEscapeFlag=NULL;    }
  
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    if (m_paLevelStoreRD[i])      { xFree(m_paLevelStoreRD[i]);  m_paLevelStoreRD[i]=NULL;  }
  }
  if (m_paSPointStoreRD)     { xFree(m_paSPointStoreRD); m_paSPointStoreRD=NULL; }
  if (m_paRunStoreRD)        { xFree(m_paRunStoreRD);    m_paRunStoreRD=NULL;    }
  if (m_paEscapeFlagStoreRD) { xFree(m_paEscapeFlagStoreRD);    m_paEscapeFlagStoreRD=NULL;    }
#endif
  
  m_isInitialized = false;
  

  
}

TEncSearch::~TEncSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}




Void TEncSearch::init(TEncCfg*      pcEncCfg,
                      TComTrQuant*  pcTrQuant,
                      Int           iSearchRange,
                      Int           bipredSearchRange,
                      Int           iFastSearch,
                      const UInt    maxCUWidth,
                      const UInt    maxCUHeight,
                      const UInt    maxTotalCUDepth,
                      TEncEntropy*  pcEntropyCoder,
                      TComRdCost*   pcRdCost,
                      TEncSbac*** pppcRDSbacCoder,
                      TEncSbac*   pcRDGoOnSbacCoder
                      )
{
  assert (!m_isInitialized);
  m_pcEncCfg             = pcEncCfg;
  m_pcTrQuant            = pcTrQuant;
  m_iSearchRange         = iSearchRange;
  m_bipredSearchRange    = bipredSearchRange;
  m_iFastSearch          = iFastSearch;
  m_pcEntropyCoder       = pcEntropyCoder;
  m_pcRdCost             = pcRdCost;

  m_pppcRDSbacCoder     = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder   = pcRDGoOnSbacCoder;

  m_uiNumBVs = 0;

  for (UInt iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++)
  {
    for (UInt iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  m_puiDFilter = s_auiDFilter + 4;

  // initialize motion cost
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
      }
    }
  }

  const ChromaFormat cform=pcEncCfg->getChromaFormatIdc();
  initTempBuff(cform);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  initTBCTable(pcEncCfg->getBitDepth(ChannelType(0)));
#endif
  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];

  const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
  const UInt uiNumPartitions = 1<<(maxTotalCUDepth<<1);
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt csx=::getComponentScaleX(ComponentID(ch), cform);
    const UInt csy=::getComponentScaleY(ComponentID(ch), cform);
    m_ppcQTTempCoeff[ch] = new TCoeff* [uiNumLayersToAllocate];
    m_pcQTTempCoeff[ch]   = new TCoeff [(maxCUWidth*maxCUHeight)>>(csx+csy)   ];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]  = new TCoeff*[uiNumLayersToAllocate];
    m_pcQTTempArlCoeff[ch]   = new TCoeff [(maxCUWidth*maxCUHeight)>>(csx+csy)   ];
#endif
    m_puhQTTempCbf[ch] = new UChar  [uiNumPartitions];

    for (UInt layer = 0; layer < uiNumLayersToAllocate; layer++)
    {
      m_ppcQTTempCoeff[ch][layer] = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy)];
#if ADAPTIVE_QP_SELECTION
      m_ppcQTTempArlCoeff[ch][layer]  = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy) ];
#endif
    }

    m_phQTTempCrossComponentPredictionAlpha[ch]    = new Char  [uiNumPartitions];
    m_pSharedPredTransformSkip[ch]                 = new Pel   [MAX_CU_SIZE*MAX_CU_SIZE];
    m_pcQTTempTUCoeff[ch]                          = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#endif
    m_puhQTTempTransformSkipFlag[ch]               = new UChar [uiNumPartitions];
#if SCM_U0106_ACT_TU_SIG
    m_pcACTTempTUCoeff[ch]                         = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#if ADAPTIVE_QP_SELECTION
    m_ppcACTTempTUArlCoeff[ch]                     = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#endif
#endif
  }
  m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
  m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
#if SCM_U0106_ACT_TU_SIG
  m_puhQTTempACTFlag   = new Bool   [uiNumPartitions];
  m_pcQTTempTComYuvCS  = new TComYuv[uiNumLayersToAllocate];
  m_pcNoCorrYuvTmp     = new TComYuv[uiNumLayersToAllocate];
#endif
  for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
  {
    m_pcQTTempTComYuv[ui].create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
#if SCM_U0106_ACT_TU_SIG
    m_pcQTTempTComYuvCS[ui].create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
    m_pcNoCorrYuvTmp[ui].create( (maxCUWidth>>ui), (maxCUHeight>>ui), pcEncCfg->getChromaFormatIdc() );
#endif
  }
  m_pcQTTempTransformSkipTComYuv.create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
#if SCM_U0106_ACT_TU_SIG
  m_pcACTTempTransformSkipTComYuv.create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
#endif
  m_tmpYuvPred.create(MAX_CU_SIZE, MAX_CU_SIZE, pcEncCfg->getChromaFormatIdc());
  m_isInitialized = true;
}

#define TZ_SEARCH_CONFIGURATION                                                                                 \
const Int  iRaster                  = 5;  /* TZ soll von aussen ?ergeben werden */                            \
const Bool bTestOtherPredictedMV    = 0;                                                                      \
const Bool bTestZeroVector          = 1;                                                                      \
const Bool bTestZeroVectorStart     = 0;                                                                      \
const Bool bTestZeroVectorStop      = 0;                                                                      \
const Bool bFirstSearchDiamond      = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bFirstSearchStop         = m_pcEncCfg->getFastMEAssumingSmootherMVEnabled();                       \
const UInt uiFirstSearchRounds      = 3;  /* first search stop X rounds after best match (must be >=1) */     \
const Bool bEnableRasterSearch      = 1;                                                                      \
const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 2 slower ===== */                     \
const Bool bRasterRefinementEnable  = 0;  /* enable either raster refinement or star refinement */            \
const Bool bRasterRefinementDiamond = 0;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */            \
const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bStarRefinementStop      = 0;                                                                      \
const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */  \

#define SEL_SEARCH_CONFIGURATION                                                                                 \
  const Bool bTestOtherPredictedMV    = 0;                                                                       \
  const Bool bTestZeroVector          = 1;                                                                       \
  const Bool bEnableRasterSearch      = 1;                                                                       \
  const Bool bAlwaysRasterSearch      = 0;  /* ===== 1: BETTER but factor 15x slower ===== */                    \
  const Bool bStarRefinementEnable    = 1;  /* enable either star refinement or raster refinement */             \
  const Bool bStarRefinementDiamond   = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */         \
  const Bool bStarRefinementStop      = 0;                                                                       \
  const UInt uiStarRefinementRounds   = 2;  /* star refinement stop X rounds after best match (must be >=1) */   \
  const UInt uiSearchRange            = m_iSearchRange;                                                          \
  const Int  uiSearchRangeInitial     = m_iSearchRange >> 2;                                                     \
  const Int  uiSearchStep             = 4;                                                                       \
  const Int  iMVDistThresh            = 8;                                                                       \

__inline Void TEncSearch::xTZSearchHelp( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  Distortion  uiSad = 0;

  Pel*  piRefSrch;

  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iYStride + iSearchX;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefSrch, rcStruct.iYStride,  m_cDistParam );

  if(m_pcEncCfg->getFastSearch() != SELECTIVE)
  {
    // fast encoder decision: use subsampled SAD when rows > 8 for integer ME
    if ( m_pcEncCfg->getUseFastEnc() )
    {
      if ( m_cDistParam.iRows > 8 )
      {
        m_cDistParam.iSubShift = 1;
      }
    }
  }

  setDistParamComp(COMPONENT_Y);

  // distortion
  m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
  if(m_pcEncCfg->getFastSearch() == SELECTIVE)
  {
    Int isubShift = 0;
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCost( iSearchX, iSearchY );

    if ( m_cDistParam.iRows > 32 )
    {
      m_cDistParam.iSubShift = 4;
    }
    else if ( m_cDistParam.iRows > 16 )
    {
      m_cDistParam.iSubShift = 3;
    }
    else if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 2;
    }
    else
    {
      m_cDistParam.iSubShift = 1;
    }

    Distortion uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
    if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
    {
      uiSad += uiTempSad >>  m_cDistParam.iSubShift;
      while(m_cDistParam.iSubShift > 0)
      {
        isubShift         = m_cDistParam.iSubShift -1;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + (pcPatternKey->getPatternLStride() << isubShift);
        m_cDistParam.pCur = piRefSrch + (rcStruct.iYStride << isubShift);
        uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
        uiSad += uiTempSad >>  m_cDistParam.iSubShift;
        if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
        {
          break;
        }

        m_cDistParam.iSubShift--;
      }

      if(m_cDistParam.iSubShift == 0)
      {
        uiSad += uiBitCost;
        if( uiSad < rcStruct.uiBestSad )
        {
          rcStruct.uiBestSad      = uiSad;
          rcStruct.iBestX         = iSearchX;
          rcStruct.iBestY         = iSearchY;
          rcStruct.uiBestDistance = uiDistance;
          rcStruct.uiBestRound    = 0;
          rcStruct.ucPointNr      = ucPointNr;
        }
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.DistFunc( &m_cDistParam );

    // motion cost
    uiSad += m_pcRdCost->getCost( iSearchX, iSearchY );

    if( uiSad < rcStruct.uiBestSad )
    {
      rcStruct.uiBestSad      = uiSad;
      rcStruct.iBestX         = iSearchX;
      rcStruct.iBestY         = iSearchY;
      rcStruct.uiBestDistance = uiDistance;
      rcStruct.uiBestRound    = 0;
      rcStruct.ucPointNr      = ucPointNr;
    }
  }
}




__inline Void TEncSearch::xTZ2PointSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
    case 1:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY, 0, 2 );
      }
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
    }
      break;
    case 2:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 3:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
    }
      break;
    case 4:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 5:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 6:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY , 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    case 7:
    {
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 8:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    default:
    {
      assert( false );
    }
      break;
  } // switch( rcStruct.ucPointNr )
}




__inline Void TEncSearch::xTZ8PointSquareSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= iSrchRngVerTop ) // check top
  {
    if ( iLeft >= iSrchRngHorLeft ) // check top left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= iSrchRngHorRight ) // check top right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= iSrchRngHorLeft ) // check middle left
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= iSrchRngHorRight ) // check middle right
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= iSrchRngVerBottom ) // check bottom
  {
    if ( iLeft >= iSrchRngHorLeft ) // check bottom left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= iSrchRngHorRight ) // check bottom right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}



__inline Void TEncSearch::xTZ8PointDiamondSearch( TComPattern* pcPatternKey, IntTZSearchStruct& rcStruct, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist, Bool bSkipLeftDist2, Bool bSkipTopDist2 )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert ( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if( bSkipLeftDist2 || bSkipTopDist2 )
    assert( iDist == 2 );

  if ( iDist == 1 ) // iDist == 1
  {
    if ( iTop >= iSrchRngVerTop ) // check top
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
    }
    if ( iLeft >= iSrchRngHorLeft ) // check middle left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= iSrchRngHorRight ) // check middle right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= iSrchRngVerBottom ) // check bottom
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
    }
  }
  else // if (iDist != 1)
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        if( !bSkipTopDist2 )
         xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        if( !bSkipLeftDist2 )
         xTZSearchHelp( pcPatternKey, rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop && !bSkipTopDist2 ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= iSrchRngVerTop ) // check half top
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= iSrchRngHorLeft && !bSkipLeftDist2 ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= iSrchRngVerBottom ) // check half bottom
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          Int iPosYT = iTop    + ((iDist>>2) * index);
          Int iPosYB = iBottom - ((iDist>>2) * index);
          Int iPosXL = iStartX - ((iDist>>2) * index);
          Int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= iSrchRngVerTop ) // check top
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= iSrchRngVerBottom ) // check bottom
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}





//<--

Distortion TEncSearch::xPatternRefinement( TComPattern* pcPatternKey,
                                           TComMv baseRefMv,
                                           Int iFrac, TComMv& rcMvFrac,
                                           Bool bAllowUseOfHadamard
                                         )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  UInt        uiDirecBest = 0;

  Pel*  piRefPos;
  Int iRefStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);

  m_pcRdCost->setDistParam( pcPatternKey, m_filteredBlock[0][0].getAddr(COMPONENT_Y), iRefStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);

  for (UInt i = 0; i < 9; i++)
  {
    if ( m_bSkipFracME && i > 0 )
    {
      break;
    }
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    Int horVal = cMvTest.getHor() * iFrac;
    Int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[ verVal & 3 ][ horVal & 3 ].getAddr(COMPONENT_Y);
    if ( horVal == 2 && ( verVal & 1 ) == 0 )
    {
      piRefPos += 1;
    }
    if ( ( horVal & 1 ) == 0 && verVal == 2 )
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;

    setDistParamComp(COMPONENT_Y);

    m_cDistParam.pCur = piRefPos;
    m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
    uiDist += m_pcRdCost->getCost( cMvTest.getHor(), cMvTest.getVer() );

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}



Void
TEncSearch::xEncSubdivCbfQT(TComTU      &rTu,
                            Bool         bLuma,
                            Bool         bChroma )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx         = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth            = rTu.GetTransformDepthRel();
  const UInt uiTrMode             = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt uiSubdiv             = ( uiTrMode > uiTrDepth ? 1 : 0 );
  const UInt uiLog2LumaTrafoSize  = rTu.GetLog2LumaTrSize();

  if( pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_NxN && uiTrDepth == 0 )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2LumaTrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
    if( bLuma )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, 5 - uiLog2LumaTrafoSize );
    }
  }

  if ( bChroma )
  {
    const UInt numberValidComponents = getNumberValidComponents(rTu.GetChromaFormat());
    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      if( rTu.ProcessingAllQuadrants(compID) && (uiTrDepth==0 || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth-1 ) ))
      {
        m_pcEntropyCoder->encodeQtCbf(rTu, compID, (uiSubdiv == 0));
      }
    }
  }

  if( uiSubdiv )
  {
    TComTURecurse tuRecurse(rTu, false);
    do
    {
      xEncSubdivCbfQT( tuRecurse, bLuma, bChroma );
    } while (tuRecurse.nextSection(rTu));
  }
  else
  {
    //===== Cbfs =====
    if( bLuma )
    {
      m_pcEntropyCoder->encodeQtCbf( rTu, COMPONENT_Y, true );
    }
  }
}




Void
TEncSearch::xEncCoeffQT(TComTU &rTu,
                        const ComponentID  component,
                        Bool         bRealCoeff )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();

  const UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );

  if( uiSubdiv )
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xEncCoeffQT( tuRecurseChild, component, bRealCoeff );
    } while (tuRecurseChild.nextSection(rTu) );
  }
  else if (rTu.ProcessComponentSection(component))
  {
    //===== coefficients =====
    const UInt  uiLog2TrafoSize = rTu.GetLog2LumaTrSize();
    UInt    uiCoeffOffset   = rTu.getCoefficientOffset(component);
    UInt    uiQTLayer       = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize;
    TCoeff* pcCoeff         = bRealCoeff ? pcCU->getCoeff(component) : m_ppcQTTempCoeff[component][uiQTLayer];

    if (isChroma(component) && (pcCU->getCbf( rTu.GetAbsPartIdxTU(), COMPONENT_Y, uiTrMode ) != 0) && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
    {
      m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, component );
    }

    m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeff+uiCoeffOffset, component );
  }
}

#if SCM_U0106_ACT_TU_SIG
Void TEncSearch::xEncColorTransformFlagQT( TComTU &rTu )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();

  const UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );

  if( uiSubdiv )
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xEncColorTransformFlagQT( tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu) );
  }
  else
  {
    //===== color transform flag =====
    Bool hasCodedCoeff = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y,  uiTrDepth) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, uiTrDepth) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, uiTrDepth));
    if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && pcCU->hasAssociatedACTFlag(uiAbsPartIdx) && hasCodedCoeff )
    {
      m_pcEntropyCoder->m_pcEntropyCoderIf->codeColourTransformFlag( pcCU, uiAbsPartIdx );
    }
  }
}
#endif

Void
TEncSearch::xEncIntraHeader( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  if( bLuma )
  {
    // CU header
    if( uiAbsPartIdx == 0 )
    {
      if( !pcCU->getSlice()->isIntra() )
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, 0, true );
        }
        m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
        m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
      }
      else // encodePredMode has already done it
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
        }
        m_pcEntropyCoder->encodePLTModeInfo(pcCU, 0, true);
      }

      if (pcCU->getPLTModeFlag(0))
      {
        return;
      }
      m_pcEntropyCoder  ->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );

      if (pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_2Nx2N )
      {
        m_pcEntropyCoder->encodeIPCMInfo( pcCU, 0, true );

        if ( pcCU->getIPCMFlag (0))
        {
          return;
        }
      }
    }
    // luma prediction mode
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N )
    {
      if (uiAbsPartIdx==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, 0 );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      if (uiTrDepth>0 && (uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
      }
    }
  }

  if( bChroma )
  {
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N || !enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()))
    {
      if(uiAbsPartIdx==0)
      {
         m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      assert(uiTrDepth>0);
      if ((uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
  }
}




UInt
TEncSearch::xGetIntraBitsQT(TComTU &rTu,
                            Bool         bLuma,
                            Bool         bChroma,
                            Bool         bRealCoeff /* just for test */ )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  xEncSubdivCbfQT ( rTu, bLuma, bChroma );

#if SCM_U0106_ACT_TU_SIG
  if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && bLuma && bChroma )
  {
    xEncColorTransformFlagQT(rTu);
  }
#endif

  if( bLuma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Y,      bRealCoeff );
  }
  if( bChroma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Cb,  bRealCoeff );
    xEncCoeffQT   ( rTu, COMPONENT_Cr,  bRealCoeff );
  }
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  return uiBits;
}

UInt TEncSearch::xGetIntraBitsQTChroma(TComTU &rTu,
                                       ComponentID compID,
                                       Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  xEncCoeffQT   ( rTu, compID,  bRealCoeff );
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}

Void TEncSearch::xIntraCodingTUBlock(       TComYuv*    pcOrgYuv,
                                            TComYuv*    pcPredYuv,
                                            TComYuv*    pcResiYuv,
                                            Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      const Bool        checkCrossCPrediction,
                                            Distortion& ruiDist,
                                      const ComponentID compID,
                                            TComTU&     rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug)
                                           ,Int         default0Save1Load2
                                     )
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }
  const Bool           bIsLuma          = isLuma(compID);
  const TComRectangle &rect             = rTu.getRect(compID);
        TComDataCU    *pcCU             = rTu.getCU();
  const UInt           uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const TComSPS       &sps              = *(pcCU->getSlice()->getSPS());

  const UInt           uiTrDepth        = rTu.GetTransformDepthRelAdj(compID);
  const UInt           uiFullDepth      = rTu.GetTransformDepthTotal();
  const UInt           uiLog2TrSize     = rTu.GetLog2LumaTrSize();
  const ChromaFormat   chFmt            = pcOrgYuv->getChromaFormat();
  const ChannelType    chType           = toChannelType(compID);
  const Int            bitDepth         = sps.getBitDepth(chType);

  const UInt           uiWidth          = rect.width;
  const UInt           uiHeight         = rect.height;
  const UInt           uiStride         = pcOrgYuv ->getStride (compID);
        Pel           *piOrg            = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
        Pel           *piPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piResi           = pcResiYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piReco           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const UInt           uiQTLayer        = sps.getQuadtreeTULog2MaxSize() - uiLog2TrSize;
        Pel           *piRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
  const UInt           uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
  const UInt           uiZOrder         = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
        Pel           *piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
        UInt           uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );
        TCoeff        *pcCoeff          = m_ppcQTTempCoeff[compID][uiQTLayer] + rTu.getCoefficientOffset(compID);
        Bool           useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);

#if ADAPTIVE_QP_SELECTION
        TCoeff        *pcArlCoeff       = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#endif

  const UInt           uiChPredMode     = pcCU->getIntraDir( chType, uiAbsPartIdx );
  const UInt           partsPerMinCU    = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
  const UInt           uiChCodedMode    = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
  const UInt           uiChFinalMode    = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  const Int            blkX                                 = g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            blkY                                 = g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            bufferOffset                         = blkX + (blkY * MAX_CU_SIZE);
        Pel  *const    encoderLumaResidual                  = resiLuma[RESIDUAL_ENCODER_SIDE ] + bufferOffset;
        Pel  *const    reconstructedLumaResidual            = resiLuma[RESIDUAL_RECONSTRUCTED] + bufferOffset;
  const Bool           bUseCrossCPrediction                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX) && checkCrossCPrediction;
  const Bool           bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
        Pel *const     lumaResidualForEstimate              = bUseReconstructedResidualForEstimate ? reconstructedLumaResidual : encoderLumaResidual;

#if DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
#endif

#if SCM_U0106_ACT_TU_SIG
  assert(!pcCU->getColourTransform(uiAbsPartIdx));
  QpParam cQP(*pcCU, compID, uiAbsPartIdx);
#else
  QpParam cQP(*pcCU, compID);
#endif

  //===== init availability pattern =====
  DEBUG_STRING_NEW(sTemp)

#if !DEBUG_STRING
  if( default0Save1Load2 != 2 )
#endif
  {
    const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag());

    initIntraPatternChType( rTu, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sDebug) );

    //===== get prediction signal =====
    predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bUseFilteredPredictions );

    // save prediction
    if( default0Save1Load2 == 1 )
    {
      Pel*  pPred   = piPred;
      Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
      Int k = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pPredBuf[ k ++ ] = pPred[ uiX ];
        }
        pPred += uiStride;
      }
    }
  }
#if !DEBUG_STRING
  else
  {
    // load prediction
    Pel*  pPred   = piPred;
    Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
    Int k = 0;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pPred[ uiX ] = pPredBuf[ k ++ ];
      }
      pPred += uiStride;
    }
  }
#endif

  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;

    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }

      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }

  if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
  {
    if (bUseCrossCPrediction)
    {
      if (xCalcCrossComponentPredictionAlpha( rTu, compID, lumaResidualForEstimate, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride ) == 0)
      {
        return;
      }
      TComTrQuant::crossComponentPrediction ( rTu, compID, reconstructedLumaResidual, piResi, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride, uiStride, false );
    }
    else if (isLuma(compID) && !bUseReconstructedResidualForEstimate)
    {
      xStoreCrossComponentPredictionResult( encoderLumaResidual, piResi, rTu, 0, 0, MAX_CU_SIZE, uiStride );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  if( useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ() )
  {
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiHeight, chType );
  }

  //--- transform and quantization ---
  TCoeff uiAbsSum = 0;
  if (bIsLuma)
  {
    pcCU       ->setTrIdxSubParts ( uiTrDepth, uiAbsPartIdx, uiFullDepth );
  }

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda     (compID);
#endif

  m_pcTrQuant->transformNxN     ( rTu, compID, piResi, uiStride, pcCoeff,
#if ADAPTIVE_QP_SELECTION
    pcArlCoeff,
#endif
    uiAbsSum, cQP
    );

  //--- inverse transform ---

#if DEBUG_STRING
  if ( (uiAbsSum > 0) || (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) )
#else
  if ( uiAbsSum > 0 )
#endif
  {
    m_pcTrQuant->invTransformNxN ( rTu, compID, piResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sDebug, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask)) );
  }
  else
  {
    Pel* pResi = piResi;
    memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      memset( pResi, 0, sizeof( Pel ) * uiWidth );
      pResi += uiStride;
    }
  }


  //===== reconstruction =====
  {
    Pel* pPred      = piPred;
    Pel* pResi      = piResi;
    Pel* pReco      = piReco;
    Pel* pRecQt     = piRecQt;
    Pel* pRecIPred  = piRecIPred;

    if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
    {
      if (bUseCrossCPrediction)
      {
        TComTrQuant::crossComponentPrediction( rTu, compID, reconstructedLumaResidual, piResi, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride, uiStride, true );
      }
      else if (isLuma(compID))
      {
        xStoreCrossComponentPredictionResult( reconstructedLumaResidual, piResi, rTu, 0, 0, MAX_CU_SIZE, uiStride );
      }
    }

 #if DEBUG_STRING
    std::stringstream ss(stringstream::out);
    const Bool bDebugPred=((DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
    const Bool bDebugResi=((DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
    const Bool bDebugReco=((DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));

    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "###: " << "CompID: " << compID << " pred mode (ch/fin): " << uiChPredMode << "/" << uiChFinalMode << " absPartIdx: " << rTu.GetAbsPartIdxTU() << "\n";
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        ss << "###: ";
        if (bDebugPred)
        {
          ss << " - pred: ";
          for( UInt uiX = 0; uiX < uiWidth; uiX++ )
          {
            ss << pPred[ uiX ] << ", ";
          }
        }
        if (bDebugResi)
        {
          ss << " - resi: ";
        }
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          if (bDebugResi)
          {
            ss << pResi[ uiX ] << ", ";
          }
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), bitDepth ));
          pRecQt   [ uiX ] = pReco[ uiX ];
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        if (bDebugReco)
        {
          ss << " - reco: ";
          for( UInt uiX = 0; uiX < uiWidth; uiX++ )
          {
            ss << pReco[ uiX ] << ", ";
          }
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
        ss << "\n";
      }
      DEBUG_STRING_APPEND(sDebug, ss.str())
    }
    else
#endif
    {

      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), bitDepth ));
          pRecQt   [ uiX ] = pReco[ uiX ];
          pRecIPred[ uiX ] = pReco[ uiX ];
        }
        pPred     += uiStride;
        pResi     += uiStride;
        pReco     += uiStride;
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }

  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart( bitDepth, piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight, compID );
}

Void
TEncSearch::xIntraCodingTUBlockCSC(       TComYuv*    pcResiYuv,
                                          Pel         reconResiLuma[MAX_CU_SIZE * MAX_CU_SIZE],
                                    const Bool        checkCrossCPrediction,
                                    const ComponentID compID,
                                          TComTU&     rTu,
                                          QpParam&    cQP
                                          DEBUG_STRING_FN_DECLARE(sDebug)
                                  )
{
  assert( rTu.ProcessComponentSection(compID) );

  TComDataCU*         pcCU              = rTu.getCU();
  const TComRectangle &rect             = rTu.getRect(compID);
  const UInt          uiLog2TrSize      = rTu.GetLog2LumaTrSize();
  const UInt          uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
  const UInt          uiQTLayer         = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  const ChromaFormat  chFmt             = rTu.GetChromaFormat();
  const ChannelType   chType            = toChannelType(compID);
  assert( chFmt == CHROMA_444 );

  const UInt          uiWidth           = rect.width;
  const UInt          uiHeight          = rect.height;
  Pel*                piResi            = pcResiYuv->getAddr( compID, uiAbsPartIdx );
  Pel*                piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
  Pel                 codedResi[MAX_CU_SIZE * MAX_CU_SIZE];
  const UInt          uiStride          = pcResiYuv->getStride (compID);
  const UInt          uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
  const UInt          uiCodedResiStride = MAX_CU_SIZE;

  const UInt          uiChPredMode         = pcCU->getIntraDir( chType, uiAbsPartIdx );
  const Bool          bUseCrossCPrediction = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX) && checkCrossCPrediction;
  const Int           blkX                 = g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int           blkY                 = g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int           bufferOffset         = blkX + (blkY * MAX_CU_SIZE);
  Pel  *const reconstructedLumaResidual    = reconResiLuma + bufferOffset;

#if DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
#endif

  TCoeff*       pcCoeff           = m_ppcQTTempCoeff[compID][uiQTLayer] + rTu.getCoefficientOffset(compID);
#if ADAPTIVE_QP_SELECTION
  TCoeff*       pcArlCoeff        = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#endif
  Bool          useTransformSkip  = pcCU->getTransformSkip(uiAbsPartIdx, compID);

  // get residual
  Pel*  pcCodedResi = codedResi;
  Pel*  pcResi      = piResi;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for ( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      pcCodedResi[uiX] = pcResi[uiX];
    }

    pcResi      += uiStride;
    pcCodedResi += uiCodedResiStride; 
  }

  if ( pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && bUseCrossCPrediction )
  {
    TComTrQuant::crossComponentPrediction( rTu, compID, reconstructedLumaResidual, codedResi, codedResi, uiWidth, uiHeight, MAX_CU_SIZE, uiCodedResiStride, uiCodedResiStride, false );
  }

  //--- transform and quantization ---
  if( useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ() ) 
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiHeight, chType );

  //--- transform and quantization ---
  TCoeff uiAbsSum = 0;

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda     (compID);
#endif

  m_pcTrQuant->transformNxN     ( rTu, compID, codedResi, uiCodedResiStride, pcCoeff,
#if ADAPTIVE_QP_SELECTION
                                  pcArlCoeff,
#endif
                                  uiAbsSum, cQP
                                );

  //--- inverse transform ---
  DEBUG_STRING_NEW(sTemp)

#if DEBUG_STRING
  if ( (uiAbsSum > 0) || (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) )
#else
  if ( uiAbsSum > 0 )
#endif
  {
    pcCodedResi = codedResi;
    m_pcTrQuant->invTransformNxN ( rTu, compID, pcCodedResi, uiCodedResiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sDebug, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask))  );
  }
  else
  {
    pcCodedResi = codedResi;
    memset( pcCoeff, 0, sizeof( TCoeff ) * uiWidth * uiHeight );
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      memset( pcCodedResi, 0, sizeof( Pel ) * uiWidth );
      pcCodedResi += uiCodedResiStride;
    }
  }

  //--- residue reconstruction ---
  if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
  {
    if ( bUseCrossCPrediction )
    {
      TComTrQuant::crossComponentPrediction( rTu, compID, reconstructedLumaResidual, codedResi, codedResi, uiWidth, uiHeight, MAX_CU_SIZE, uiCodedResiStride, uiCodedResiStride, true );
    }
    else if ( isLuma( compID ) )
    {
      xStoreCrossComponentPredictionResult( reconstructedLumaResidual, codedResi, rTu, 0, 0, MAX_CU_SIZE, uiCodedResiStride );
    }
  }

  Pel* pCodedResi   = codedResi;
  Pel* pRecQt       = piRecQt;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for ( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      pRecQt[uiX] = pCodedResi[uiX];  //recon rsidue
    }

    pCodedResi += uiCodedResiStride;
    pRecQt     += uiRecQtStride;
  }
}





Void
TEncSearch::xRecurIntraCodingLumaQT(TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDistY,
#if HHI_RQT_INTRA_SPEEDUP
                                    Bool        bCheckFirst,
#endif
                                    Double&     dRDCost,
                                    TComTU&     rTu
                                    DEBUG_STRING_FN_DECLARE(sDebug))
{
  TComDataCU   *pcCU          = rTu.getCU();
  const UInt    uiAbsPartIdx  = rTu.GetAbsPartIdxTU();
  const UInt    uiFullDepth   = rTu.GetTransformDepthTotal();
  const UInt    uiTrDepth     = rTu.GetTransformDepthRel();
  const UInt    uiLog2TrSize  = rTu.GetLog2LumaTrSize();
        Bool    bCheckFull    = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
        Bool    bCheckSplit   = ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );

        Pel     resiLumaSplit [NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];
        Pel     resiLumaSingle[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

        Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
        for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
        {
          bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
        }

        bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

#if HHI_RQT_INTRA_SPEEDUP
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // don't check split if TU size is less or equal to max TU size
  Bool noSplitIntraMaxTuSize = bCheckFull;
  if(m_pcEncCfg->getRDpenalty() && ! isIntraSlice)
  {
    // in addition don't check split if TU size is less or equal to 16x16 TU size for non-intra slice
    noSplitIntraMaxTuSize = ( uiLog2TrSize  <= min(maxTuSize,4) );

    // if maximum RD-penalty don't check TU size 32x32
    if(m_pcEncCfg->getRDpenalty()==2)
    {
      bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
    }
  }
  if( bCheckFirst && noSplitIntraMaxTuSize )

  {
    bCheckSplit = false;
  }
#else
  Int maxTuSize = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize();
  Int isIntraSlice = (pcCU->getSlice()->getSliceType() == I_SLICE);
  // if maximum RD-penalty don't check TU size 32x32
  if((m_pcEncCfg->getRDpenalty()==2)  && !isIntraSlice)
  {
    bCheckFull    = ( uiLog2TrSize  <= min(maxTuSize,4));
  }
#endif

  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && bCheckFull)
  {
    bCheckSplit = false;
  }

  Double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  UInt       uiSingleCbfLuma                    = 0;
  Bool       checkTransformSkip  = pcCU->getSlice()->getPPS()->getUseTransformSkip();
  Int        bestModeId[MAX_NUM_COMPONENT] = { 0, 0, 0};
  checkTransformSkip           &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());
  checkTransformSkip           &= (!pcCU->getCUTransquantBypass(0));

  assert (rTu.ProcessComponentSection(COMPONENT_Y));
  const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(COMPONENT_Y);

  if ( m_pcEncCfg->getUseTransformSkipFast() )
  {
    checkTransformSkip       &= (pcCU->getPartitionSize(uiAbsPartIdx)==SIZE_NxN);
  }

  if( bCheckFull )
  {
    if(checkTransformSkip == true)
    {
      //----- store original entropy coding status -----
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

      Distortion singleDistTmpLuma                    = 0;
      UInt       singleCbfTmpLuma                     = 0;
      Double     singleCostTmp                        = 0;
      Int        firstCheckId                         = 0;

      for(Int modeId = firstCheckId; modeId < 2; modeId ++)
      {
        DEBUG_STRING_NEW(sModeString)
        Int  default0Save1Load2 = 0;
        singleDistTmpLuma=0;
        if(modeId == firstCheckId)
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }


        pcCU->setTransformSkipSubParts ( modeId, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
        xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, singleDistTmpLuma, COMPONENT_Y, rTu DEBUG_STRING_PASS_INTO(sModeString), default0Save1Load2 );

        singleCbfTmpLuma = pcCU->getCbf( uiAbsPartIdx, COMPONENT_Y, uiTrDepth );

        //----- determine rate and r-d cost -----
        if(modeId == 1 && singleCbfTmpLuma == 0)
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          UInt uiSingleBits = xGetIntraBitsQT( rTu, true, false, false );
          singleCostTmp     = m_pcRdCost->calcRdCost( uiSingleBits, singleDistTmpLuma );
        }
        if(singleCostTmp < dSingleCost)
        {
          DEBUG_STRING_SWAP(sDebug, sModeString)
          dSingleCost   = singleCostTmp;
          uiSingleDistLuma = singleDistTmpLuma;
          uiSingleCbfLuma = singleCbfTmpLuma;

          bestModeId[COMPONENT_Y] = modeId;
          if(bestModeId[COMPONENT_Y] == firstCheckId)
          {
            xStoreIntraResultQT(COMPONENT_Y, rTu );
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
          }

          if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
          {
            const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
            const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
            for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
            {
              if (bMaintainResidual[storedResidualIndex])
              {
                xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSingle[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
              }
            }
          }
        }
        if (modeId == firstCheckId)
        {
          m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
        }
      }

      pcCU ->setTransformSkipSubParts ( bestModeId[COMPONENT_Y], COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );

      if(bestModeId[COMPONENT_Y] == firstCheckId)
      {
        xLoadIntraResultQT(COMPONENT_Y, rTu );
        pcCU->setCbfSubParts  ( uiSingleCbfLuma << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, rTu.GetTransformDepthTotalAdj(COMPONENT_Y) );

        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      }
    }
    else
    {
      //----- store original entropy coding status -----
      if( bCheckSplit )
      {
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      }
      //----- code luma/chroma block with given intra prediction mode and store Cbf-----
      dSingleCost   = 0.0;

      pcCU ->setTransformSkipSubParts ( 0, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
      xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, uiSingleDistLuma, COMPONENT_Y, rTu DEBUG_STRING_PASS_INTO(sDebug));

      if( bCheckSplit )
      {
        uiSingleCbfLuma = pcCU->getCbf( uiAbsPartIdx, COMPONENT_Y, uiTrDepth );
      }
      //----- determine rate and r-d cost -----
      UInt uiSingleBits = xGetIntraBitsQT( rTu, true, false, false );

      if(m_pcEncCfg->getRDpenalty() && (uiLog2TrSize==5) && !isIntraSlice)
      {
        uiSingleBits=uiSingleBits*4;
      }

      dSingleCost       = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDistLuma );

      if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
      {
        const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
        const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
        for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
        {
          if (bMaintainResidual[storedResidualIndex])
          {
            xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSingle[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
          }
        }
      }
    }
  }

  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    else
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    //----- code splitted block -----
    Double     dSplitCost      = 0.0;
    Distortion uiSplitDistLuma = 0;
    UInt       uiSplitCbfLuma  = 0;

    TComTURecurse tuRecurseChild(rTu, false);
    DEBUG_STRING_NEW(sSplit)
    do
    {
      DEBUG_STRING_NEW(sChild)
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDistLuma, bCheckFirst, dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#else
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDistLuma, dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#endif
      DEBUG_STRING_APPEND(sSplit, sChild)
      uiSplitCbfLuma |= pcCU->getCbf( tuRecurseChild.GetAbsPartIdxTU(), COMPONENT_Y, tuRecurseChild.GetTransformDepthRel() );
    } while (tuRecurseChild.nextSection(rTu) );

    UInt    uiPartsDiv     = rTu.GetAbsPartIdxNumParts();
    {
      if (uiSplitCbfLuma)
      {
        const UInt flag=1<<uiTrDepth;
        UChar *pBase=pcCU->getCbf( COMPONENT_Y );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }
    //----- restore context states -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    
    //----- determine rate and r-d cost -----
    UInt uiSplitBits = xGetIntraBitsQT( rTu, true, false, false );
    dSplitCost       = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDistLuma );

    //===== compare and set best =====
    if( dSplitCost < dSingleCost )
    {
      //--- update cost ---
      DEBUG_STRING_SWAP(sSplit, sDebug)
      ruiDistY += uiSplitDistLuma;
      dRDCost  += dSplitCost;

      if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
      {
        const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
        const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
        for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
        {
          if (bMaintainResidual[storedResidualIndex])
          {
            xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaSplit[storedResidualIndex], rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
          }
        }
      }

      return;
    }

    //----- set entropy coding status -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

    //--- set transform index and Cbf values ---
    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );
    const TComRectangle &tuRect=rTu.getRect(COMPONENT_Y);
    pcCU->setCbfSubParts  ( uiSingleCbfLuma << uiTrDepth, COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );
    pcCU ->setTransformSkipSubParts  ( bestModeId[COMPONENT_Y], COMPONENT_Y, uiAbsPartIdx, totalAdjustedDepthChan );

    //--- set reconstruction for next intra prediction blocks ---
    const UInt  uiQTLayer   = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    const UInt  uiZOrder    = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
    const UInt  uiWidth     = tuRect.width;
    const UInt  uiHeight    = tuRect.height;
    Pel*  piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( COMPONENT_Y, uiAbsPartIdx );
    UInt  uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( COMPONENT_Y );
    Pel*  piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), uiZOrder );
    UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ( COMPONENT_Y );

    for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        piDes[ uiX ] = piSrc[ uiX ];
      }
    }
  }
  ruiDistY += uiSingleDistLuma;
  dRDCost  += dSingleCost;
}


Void
TEncSearch::xSetIntraResultLumaQT(TComYuv* pcRecoYuv, TComTU &rTu)
{
  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiTrDepth    = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    //===== copy transform coefficients =====

    const TComRectangle &tuRect=rTu.getRect(COMPONENT_Y);
    const UInt coeffOffset = rTu.getCoefficientOffset(COMPONENT_Y);
    const UInt numCoeffInBlock = tuRect.width * tuRect.height;

    if (numCoeffInBlock!=0)
    {
      const TCoeff* srcCoeff = m_ppcQTTempCoeff[COMPONENT_Y][uiQTLayer] + coeffOffset;
      TCoeff* destCoeff      = pcCU->getCoeff(COMPONENT_Y) + coeffOffset;
      ::memcpy( destCoeff, srcCoeff, sizeof(TCoeff)*numCoeffInBlock );
#if ADAPTIVE_QP_SELECTION
      const TCoeff* srcArlCoeff = m_ppcQTTempArlCoeff[COMPONENT_Y][ uiQTLayer ] + coeffOffset;
      TCoeff* destArlCoeff      = pcCU->getArlCoeff (COMPONENT_Y)               + coeffOffset;
      ::memcpy( destArlCoeff, srcArlCoeff, sizeof( TCoeff ) * numCoeffInBlock );
#endif
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Y, pcRecoYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
    }

  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetIntraResultLumaQT( pcRecoYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}

#if SCM_U0106_ACT_TU_SIG
Void
TEncSearch::xStoreIntraResultQT(const ComponentID compID, TComTU &rTu, Bool bACTCache )
#else
Void
TEncSearch::xStoreIntraResultQT(const ComponentID compID, TComTU &rTu )
#endif
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if ( compID==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    if (rTu.ProcessComponentSection(compID))
    {
      const TComRectangle &tuRect=rTu.getRect(compID);

      //===== copy transform coefficients =====
      const UInt uiNumCoeff    = tuRect.width * tuRect.height;
      TCoeff* pcCoeffSrc = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#if SCM_U0106_ACT_TU_SIG
      TCoeff* pcCoeffDst = (!bACTCache)? m_pcQTTempTUCoeff[compID]: m_pcACTTempTUCoeff[compID];
#else
      TCoeff* pcCoeffDst = m_pcQTTempTUCoeff[compID];
#endif

      ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffSrc = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#if SCM_U0106_ACT_TU_SIG
      TCoeff* pcArlCoeffDst = (!bACTCache)? m_ppcQTTempTUArlCoeff[compID]: m_ppcACTTempTUArlCoeff[compID];
#else
      TCoeff* pcArlCoeffDst = m_ppcQTTempTUArlCoeff[compID];
#endif
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
      //===== copy reconstruction =====
#if SCM_U0106_ACT_TU_SIG
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( compID, ((!bACTCache)? &m_pcQTTempTransformSkipTComYuv: &m_pcACTTempTransformSkipTComYuv), uiAbsPartIdx, tuRect.width, tuRect.height );
#else
      m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( compID, &m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
#endif
    }
  }
}

#if SCM_U0106_ACT_TU_SIG
Void TEncSearch::xLoadIntraResultQT(const ComponentID compID, TComTU &rTu, Bool bACTCache)
#else
Void
TEncSearch::xLoadIntraResultQT(const ComponentID compID, TComTU &rTu)
#endif
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if ( compID==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    const UInt uiZOrder     = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;

    if (rTu.ProcessComponentSection(compID))
    {
      const TComRectangle &tuRect=rTu.getRect(compID);

      //===== copy transform coefficients =====
      const UInt uiNumCoeff = tuRect.width * tuRect.height;
      TCoeff* pcCoeffDst = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#if SCM_U0106_ACT_TU_SIG
      TCoeff* pcCoeffSrc = (!bACTCache)?m_pcQTTempTUCoeff[compID]: m_pcACTTempTUCoeff[compID];
#else
      TCoeff* pcCoeffSrc = m_pcQTTempTUCoeff[compID];
#endif

      ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffDst = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#if SCM_U0106_ACT_TU_SIG
      TCoeff* pcArlCoeffSrc = (!bACTCache)? m_ppcQTTempTUArlCoeff[compID]: m_ppcACTTempTUArlCoeff[compID];
#else
      TCoeff* pcArlCoeffSrc = m_ppcQTTempTUArlCoeff[compID];
#endif
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
      //===== copy reconstruction =====
#if SCM_U0106_ACT_TU_SIG
      if( !bACTCache )
      {
        m_pcQTTempTransformSkipTComYuv.copyPartToPartComponent( compID, &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, tuRect.width, tuRect.height );
      }
      else
      {
        m_pcACTTempTransformSkipTComYuv.copyPartToPartComponent( compID, &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, tuRect.width, tuRect.height );
      }
#else
      m_pcQTTempTransformSkipTComYuv.copyPartToPartComponent( compID, &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, tuRect.width, tuRect.height );
#endif

      Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride (compID);
      Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride  (compID);
      UInt    uiWidth           = tuRect.width;
      UInt    uiHeight          = tuRect.height;
      Pel* pRecQt               = piRecQt;
      Pel* pRecIPred            = piRecIPred;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pRecIPred[ uiX ] = pRecQt   [ uiX ];
        }
        pRecQt    += uiRecQtStride;
        pRecIPred += uiRecIPredStride;
      }
    }
  }
}

Void
TEncSearch::xStoreCrossComponentPredictionResult(       Pel    *pResiDst,
                                                  const Pel    *pResiSrc,
                                                        TComTU &rTu,
                                                  const Int     xOffset,
                                                  const Int     yOffset,
                                                  const Int     strideDst,
                                                  const Int     strideSrc )
{
  const Pel *pSrc = pResiSrc + yOffset * strideSrc + xOffset;
        Pel *pDst = pResiDst + yOffset * strideDst + xOffset;

  for( Int y = 0; y < rTu.getRect( COMPONENT_Y ).height; y++ )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel) * rTu.getRect( COMPONENT_Y ).width );
    pDst += strideDst;
    pSrc += strideSrc;
  }
}

Char
TEncSearch::xCalcCrossComponentPredictionAlpha(       TComTU &rTu,
                                                const ComponentID compID,
                                                const Pel*        piResiL,
                                                const Pel*        piResiC,
                                                const Int         width,
                                                const Int         height,
                                                const Int         strideL,
                                                const Int         strideC )
{
  const Pel *pResiL = piResiL;
  const Pel *pResiC = piResiC;

        TComDataCU *pCU = rTu.getCU();
  const Int  absPartIdx = rTu.GetAbsPartIdxTU( compID );
  const Int diffBitDepth = pCU->getSlice()->getSPS()->getDifferentialLumaChromaBitDepth();

  Char alpha = 0;
  Int SSxy  = 0;
  Int SSxx  = 0;

  for( UInt uiY = 0; uiY < height; uiY++ )
  {
    for( UInt uiX = 0; uiX < width; uiX++ )
    {
      const Pel scaledResiL = rightShift( pResiL[ uiX ], diffBitDepth );
      SSxy += ( scaledResiL * pResiC[ uiX ] );
      SSxx += ( scaledResiL * scaledResiL   );
    }

    pResiL += strideL;
    pResiC += strideC;
  }

  if( SSxx != 0 )
  {
    Double dAlpha = SSxy / Double( SSxx );
    alpha = Char(Clip3<Int>(-16, 16, (Int)(dAlpha * 16)));

    static const Char alphaQuant[17] = {0, 1, 1, 2, 2, 2, 4, 4, 4, 4, 4, 4, 8, 8, 8, 8, 8};

    alpha = (alpha < 0) ? -alphaQuant[Int(-alpha)] : alphaQuant[Int(alpha)];
  }
  pCU->setCrossComponentPredictionAlphaPartRange( alpha, compID, absPartIdx, rTu.GetAbsPartIdxNumParts( compID ) );

  return alpha;
}

Void
TEncSearch::xRecurIntraChromaCodingQT(TComYuv*    pcOrgYuv,
                                      TComYuv*    pcPredYuv,
                                      TComYuv*    pcResiYuv,
                                      Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      Distortion& ruiDist,
                                      TComTU&     rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug))
{
  TComDataCU         *pcCU                  = rTu.getCU();
  const UInt          uiTrDepth             = rTu.GetTransformDepthRel();
  const UInt          uiAbsPartIdx          = rTu.GetAbsPartIdxTU();
  const ChromaFormat  format                = rTu.GetChromaFormat();
  UInt                uiTrMode              = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt          numberValidComponents = getNumberValidComponents(format);

  if(  uiTrMode == uiTrDepth )
  {
    if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      return;
    }

    const UInt uiFullDepth = rTu.GetTransformDepthTotal();

    Bool checkTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip();
    checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Cb), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());

    if ( m_pcEncCfg->getUseTransformSkipFast() )
    {
      checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize());

      if (checkTransformSkip)
      {
        Int nbLumaSkip = 0;
        const UInt maxAbsPartIdxSub=uiAbsPartIdx + (rTu.ProcessingAllQuadrants(COMPONENT_Cb)?1:4);
        for(UInt absPartIdxSub = uiAbsPartIdx; absPartIdxSub < maxAbsPartIdxSub; absPartIdxSub ++)
        {
          nbLumaSkip += pcCU->getTransformSkip(absPartIdxSub, COMPONENT_Y);
        }
        checkTransformSkip &= (nbLumaSkip > 0);
      }
    }


    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      DEBUG_STRING_NEW(sDebugBestMode)

      //use RDO to decide whether Cr/Cb takes TS
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT] );

      const Bool splitIntoSubTUs = rTu.getRect(compID).width != rTu.getRect(compID).height;

      TComTURecurse TUIterator(rTu, false, (splitIntoSubTUs ? TComTU::VERTICAL_SPLIT : TComTU::DONT_SPLIT), true, compID);

      const UInt partIdxesPerSubTU = TUIterator.GetAbsPartIdxNumParts(compID);

      do
      {
        const UInt subTUAbsPartIdx   = TUIterator.GetAbsPartIdxTU(compID);

        Double     dSingleCost               = MAX_DOUBLE;
        Int        bestModeId                = 0;
        Distortion singleDistC               = 0;
        UInt       singleCbfC                = 0;
        Distortion singleDistCTmp            = 0;
        Double     singleCostTmp             = 0;
        UInt       singleCbfCTmp             = 0;
        Char       bestCrossCPredictionAlpha = 0;
        Int        bestTransformSkipMode     = 0;

        const Bool checkCrossComponentPrediction =    (pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, subTUAbsPartIdx) == DM_CHROMA_IDX)
                                                   &&  pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                                   && (pcCU->getCbf(subTUAbsPartIdx,  COMPONENT_Y, uiTrDepth) != 0);

        const Int  crossCPredictionModesToTest = checkCrossComponentPrediction ? 2 : 1;
        const Int  transformSkipModesToTest    = checkTransformSkip            ? 2 : 1;
        const Int  totalModesToTest            = crossCPredictionModesToTest * transformSkipModesToTest;
              Int  currModeId                  = 0;
              Int  default0Save1Load2          = 0;

        for(Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
        {
          for(Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
          {
            pcCU->setCrossComponentPredictionAlphaPartRange(0, compID, subTUAbsPartIdx, partIdxesPerSubTU);
            DEBUG_STRING_NEW(sDebugMode)
            pcCU->setTransformSkipPartRange( transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU );
            currModeId++;

            const Bool isOneMode  = (totalModesToTest == 1);
            const Bool isLastMode = (currModeId == totalModesToTest); // currModeId is indexed from 1

            if (isOneMode)
            {
              default0Save1Load2 = 0;
            }
            else if (!isOneMode && (transformSkipModeId == 0) && (crossCPredictionModeId == 0))
            {
              default0Save1Load2 = 1; //save prediction on first mode
            }
            else
            {
              default0Save1Load2 = 2; //load it on subsequent modes
            }

            singleDistCTmp = 0;

            xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, (crossCPredictionModeId != 0), singleDistCTmp, compID, TUIterator DEBUG_STRING_PASS_INTO(sDebugMode), default0Save1Load2);
            singleCbfCTmp = pcCU->getCbf( subTUAbsPartIdx, compID, uiTrDepth);

            if (  ((crossCPredictionModeId == 1) && (pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) == 0))
               || ((transformSkipModeId    == 1) && (singleCbfCTmp == 0))) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
            {
              singleCostTmp = MAX_DOUBLE;
            }
            else if (!isOneMode)
            {
              UInt bitsTmp = xGetIntraBitsQTChroma( TUIterator, compID, false );
              singleCostTmp  = m_pcRdCost->calcRdCost( bitsTmp, singleDistCTmp);
            }

            if(singleCostTmp < dSingleCost)
            {
              DEBUG_STRING_SWAP(sDebugBestMode, sDebugMode)
              dSingleCost               = singleCostTmp;
              singleDistC               = singleDistCTmp;
              bestCrossCPredictionAlpha = (crossCPredictionModeId != 0) ? pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) : 0;
              bestTransformSkipMode     = transformSkipModeId;
              bestModeId                = currModeId;
              singleCbfC                = singleCbfCTmp;

              if (!isOneMode && !isLastMode)
              {
                xStoreIntraResultQT(compID, TUIterator);
                m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
              }
            }

            if (!isOneMode && !isLastMode)
            {
              m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
            }
          }
        }

        if(bestModeId < totalModesToTest)
        {
          xLoadIntraResultQT(compID, TUIterator);
          pcCU->setCbfPartRange( singleCbfC << uiTrDepth, compID, subTUAbsPartIdx, partIdxesPerSubTU );

          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
        }

        DEBUG_STRING_APPEND(sDebug, sDebugBestMode)
        pcCU ->setTransformSkipPartRange                ( bestTransformSkipMode,     compID, subTUAbsPartIdx, partIdxesPerSubTU );
        pcCU ->setCrossComponentPredictionAlphaPartRange( bestCrossCPredictionAlpha, compID, subTUAbsPartIdx, partIdxesPerSubTU );
        ruiDist += singleDistC;
      } while (TUIterator.nextSection(rTu));

      if (splitIntoSubTUs)
      {
        offsetSubTUCBFs(rTu, compID);
      }
    }
  }
  else
  {
    UInt    uiSplitCbf[MAX_NUM_COMPONENT] = {0,0,0};

    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiTrDepthChild   = tuRecurseChild.GetTransformDepthRel();
    do
    {
      DEBUG_STRING_NEW(sChild)

      xRecurIntraChromaCodingQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, ruiDist, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );

      DEBUG_STRING_APPEND(sDebug, sChild)
      const UInt uiAbsPartIdxSub=tuRecurseChild.GetAbsPartIdxTU();

      for(UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
      {
        uiSplitCbf[ch] |= pcCU->getCbf( uiAbsPartIdxSub, ComponentID(ch), uiTrDepthChild );
      }
    } while ( tuRecurseChild.nextSection(rTu) );


    UInt uiPartsDiv = rTu.GetAbsPartIdxNumParts();
    for(UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      if (uiSplitCbf[ch])
      {
        const UInt flag=1<<uiTrDepth;
        ComponentID compID=ComponentID(ch);
        UChar *pBase=pcCU->getCbf( compID );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }
  }
}

#if SCM_U0106_ACT_TU_SIG
Void
TEncSearch::xRecurIntraCodingQTTUCSC( TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, Distortion& uiPUDistY, Distortion& uiPUDistC, Double& dPUCost, TComTU& rTu, Bool bTestMaxTUSize, ACTRDTestTypes eACTRDTestType DEBUG_STRING_FN_DECLARE(sDebug) )
{
  TComDataCU          *pcCU                 = rTu.getCU();
  const UInt          uiTrDepth             = rTu.GetTransformDepthRel();
  const UInt          uiFullDepth           = rTu.GetTransformDepthTotal();
  const UInt          uiAbsPartIdx          = rTu.GetAbsPartIdxTU();
  const UInt          partIndxNumPerTU      = rTu.GetAbsPartIdxNumParts(COMPONENT_Y);
  const ChromaFormat  chFmt                 = rTu.GetChromaFormat();
  const UInt          uiLog2TrSize          = rTu.GetLog2LumaTrSize();
  const UInt          uiQTLayer             = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  const UInt          numberValidComponents = getNumberValidComponents(chFmt);
  Bool                bCheckFull            = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  Bool                bCheckSplit           = ( bTestMaxTUSize && bCheckFull )? false: ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
  const Bool          extendedPrecision     = rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();;

  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && bCheckFull)
  {
    bCheckSplit = false;
  }

  assert( bCheckFull || bCheckSplit );
  assert( chFmt == CHROMA_444);

  Double              dSingleCost                                 = MAX_DOUBLE;
  Distortion          uiSingleDist[MAX_NUM_CHANNEL_TYPE]          = {0, 0};
  UInt                uiSingleBits                                = 0;

  UInt                uiSingleColorSpaceId                        = 0;
  Double              dSingleColorSpaceCost[2]                        = {MAX_DOUBLE, MAX_DOUBLE};
  Distortion          uiSingleColorSpaceDist[2][MAX_NUM_CHANNEL_TYPE] = {{0, 0}, {0, 0}};
  UInt                uiSingleColorSpaceBits[2]                       = {0, 0};  

  Double              dSingleComponentCost[2][MAX_NUM_COMPONENT]              = {{MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE}, {MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE}};
  Distortion          uiSingleComponentDist[2][MAX_NUM_COMPONENT]             = {{0, 0, 0}, {0, 0, 0}};
  UInt                uiSingleComponentCbf[2][MAX_NUM_COMPONENT]              = {{0, 0, 0}, {0, 0, 0}};
  UInt                uiSingleComponentTransformMode[2][MAX_NUM_COMPONENT]    = {{0, 0, 0}, {0, 0, 0}};
  Char                cSingleComponentPredictionAlpha[2][MAX_NUM_COMPONENT]   = {{0, 0, 0}, {0, 0, 0}};

  UInt uiRelTrDepth = uiTrDepth;
  if( pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN )
  {
    uiRelTrDepth --;
  }
  m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled  = MAX_DOUBLE;
  m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled = MAX_DOUBLE;
  m_sACTRDCostTU[uiRelTrDepth].uiIsCSCEnabled       = 2;

  if( bCheckFull )
  {
    m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );

    //intra prediction
    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID   compID    = ComponentID(ch);
      const ChannelType   chType    = toChannelType(compID);
      const TComRectangle &rect     = rTu.getRect(compID);
      const UInt          uiWidth   = rect.width;
      const UInt          uiHeight  = rect.height;

      const Bool bIsLuma            = isLuma(compID);
      const UInt uiChPredMode       = pcCU->getIntraDir( chType, uiAbsPartIdx );
      const UInt uiChFinalMode      = (uiChPredMode == DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx) : uiChPredMode;

      const Bool bUseFilteredPredictions = TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getSpsRangeExtension().getIntraSmoothingDisabledFlag());
      Pel*       piOrg                   = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
      Pel*       piPred                  = pcPredYuv->getAddr( compID, uiAbsPartIdx );
      Pel*       piResi                  = pcResiYuv->getAddr( compID, uiAbsPartIdx );
      const UInt uiStride                = pcOrgYuv ->getStride (compID);

      DEBUG_STRING_NEW(sTemp)
      initIntraPatternChType( rTu, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sTemp) );
      predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bUseFilteredPredictions );

      Pel* pcOrg  = piOrg;
      Pel* pcPred = piPred;
      Pel* pcResi = piResi;

      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pcResi[ uiX ] = pcOrg[ uiX ] - pcPred[ uiX ];
        }

        pcOrg  += uiStride;
        pcResi += uiStride;
        pcPred += uiStride;
      }
    }

    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID compID = ComponentID(ch); 
      pcResiYuv->copyPartToPartComponent( compID, &(m_pcQTTempTComYuvCS[uiQTLayer]), uiAbsPartIdx, rTu.getRect(compID).width, rTu.getRect(compID).height );
    }

    for(Int colorSpaceId = 0; colorSpaceId < 2; colorSpaceId++)
    {
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

      Bool bCheckACT = m_pcEncCfg->getRGBFormatFlag()? (!colorSpaceId? true: false): (colorSpaceId? true: false);
      assert( pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx ) == DM_CHROMA_IDX );

      if(eACTRDTestType == ACT_TRAN_CLR && !bCheckACT)
      {
        continue;
      }
      if(eACTRDTestType == ACT_ORG_CLR && bCheckACT)
      {
        continue;
      }

      pcCU->setColourTransformSubParts(bCheckACT, uiAbsPartIdx, uiFullDepth);
      if( colorSpaceId )
      {
        for( UInt ch = 0; ch < numberValidComponents; ch++ )
        {
          const ComponentID compID = ComponentID(ch); 
          m_pcQTTempTComYuvCS[uiQTLayer].copyPartToPartComponent( compID, pcResiYuv, uiAbsPartIdx, rTu.getRect(compID).width, rTu.getRect(compID).height );
        }
      }

      if(bCheckACT)
      {
        pcResiYuv->convert(extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, true, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0));
      }

      Char preCalcAlpha[MAX_NUM_COMPONENT] = {0, 0, 0};

      if( pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && !m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() ) 
      {
        for( UInt ch = 0; ch < numberValidComponents; ch++ )
        {
          const ComponentID    compID                        = ComponentID(ch);
          const TComRectangle &tuCompRect                    = rTu.getRect(compID);
          const Pel  *const    lumaResidualForEstimate       = pcResiYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
          const UInt           lumaResidualStrideForEstimate = pcResiYuv->getStride ( COMPONENT_Y );

          const UInt           uiChPredMode                  = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
          const Bool bDecorrelationAvailable                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX);
          if( bDecorrelationAvailable )
          {
            preCalcAlpha[compID] = xCalcCrossComponentPredictionAlpha(rTu,
                                                                      compID,
                                                                      lumaResidualForEstimate,
                                                                      pcResiYuv->getAddr( compID, uiAbsPartIdx ),
                                                                      tuCompRect.width,
                                                                      tuCompRect.height,
                                                                      lumaResidualStrideForEstimate,
                                                                      pcResiYuv->getStride( compID ) );
          }
        }
      }    

      Pel reconResiLuma   [MAX_CU_SIZE * MAX_CU_SIZE];
      Pel reconResiLumaTmp[MAX_CU_SIZE * MAX_CU_SIZE];

      for( UInt ch = 0; ch < numberValidComponents; ch++ )
      {
        Double     dSingleCostTmp  = MAX_DOUBLE;
        UInt       uiSingleBitsTmp = 0;
        Distortion uiSingleDistTmp = 0;
        UInt       uiSingleCbfTmp  = 0;

        const ComponentID    compID     = ComponentID(ch);
        const TComRectangle &tuCompRect = rTu.getRect(compID);
        QpParam cQP(*pcCU, compID, uiAbsPartIdx);

        if( !pcCU->isLosslessCoded(0) && bCheckACT )
        {
          Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
          m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( deltaQP );
          m_pcRdCost->adjustLambdaForColourTrans( deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
        }

        Bool bCheckTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
                                   TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()) &&
                                   (!pcCU->isLosslessCoded(0));
        if( m_pcEncCfg->getUseTransformSkipFast() )
        {
          bCheckTransformSkip &= (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN);
          if(isChroma(compID))
          {
            bCheckTransformSkip &= (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Y) != 0);
          }
        }

        Bool bCheckCrossComponentPrediction =    isChroma(compID)
                                              && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                              && (pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx ) == DM_CHROMA_IDX)
                                              && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrDepth) != 0);

        Char cCalcAlpha = 0;
        if ( bCheckCrossComponentPrediction && m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() )
        {
          cCalcAlpha = xCalcCrossComponentPredictionAlpha(rTu,
                                                          compID,
                                                          reconResiLuma + tuCompRect.y0 * MAX_CU_SIZE + tuCompRect.x0,
                                                          pcResiYuv->getAddr( compID, uiAbsPartIdx ),
                                                          tuCompRect.width,
                                                          tuCompRect.height,
                                                          MAX_CU_SIZE,
                                                          pcResiYuv->getStride( compID ) );
        }
        else if( bCheckCrossComponentPrediction )
        {
          cCalcAlpha = preCalcAlpha[compID];
        }
        bCheckCrossComponentPrediction &= (cCalcAlpha != 0);

        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

        const Int transformSkipModesToTest    = bCheckTransformSkip               ? 2 : 1;
        const Int crossCPredictionModesToTest = bCheckCrossComponentPrediction    ? 2 : 1;

        for(Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
        {
          for(Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
          {
            DEBUG_STRING_NEW(sModeString)
            m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
            m_pcEntropyCoder->resetBits();

            pcCU->setTransformSkipPartRange                ( transformSkipModeId,                          compID, uiAbsPartIdx, partIndxNumPerTU );
            pcCU->setCrossComponentPredictionAlphaPartRange( (crossCPredictionModeId != 0)? cCalcAlpha: 0, compID, uiAbsPartIdx, partIndxNumPerTU );

            xIntraCodingTUBlockCSC( pcResiYuv, (compID == COMPONENT_Y)? reconResiLumaTmp: reconResiLuma, (crossCPredictionModeId != 0), compID, rTu, cQP DEBUG_STRING_PASS_INTO(sModeString) );
            uiSingleCbfTmp = pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth );

            if( (uiSingleCbfTmp == 0) && (transformSkipModeId != 0) )
            {
              dSingleCostTmp = MAX_DOUBLE;
            }
            else
            {
              if( compID == COMPONENT_Y )
              {
                uiSingleBitsTmp = xGetIntraBitsQT( rTu, true, false, false );
              }
              else
              {
                uiSingleBitsTmp = xGetIntraBitsQTChroma( rTu, compID, false );
              }

              //residual distortion in YCgCo domain
              uiSingleDistTmp = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType( compID )], 
                                                         m_pcQTTempTComYuv[uiQTLayer].getAddr(compID, uiAbsPartIdx), 
                                                         m_pcQTTempTComYuv[uiQTLayer].getStride(compID), 
                                                         pcResiYuv->getAddr(compID, uiAbsPartIdx), 
                                                         pcResiYuv->getStride(compID), 
                                                         rTu.getRect(compID).width, 
                                                         rTu.getRect(compID).height, 
                                                         compID);
              dSingleCostTmp = m_pcRdCost->calcRdCost( uiSingleBitsTmp, uiSingleDistTmp );
            }

            if( dSingleCostTmp < dSingleComponentCost[colorSpaceId][compID] )
            {
              dSingleComponentCost[colorSpaceId][compID]             = dSingleCostTmp;
              uiSingleComponentDist[colorSpaceId][compID]            = uiSingleDistTmp;
              uiSingleComponentCbf[colorSpaceId][compID]             = uiSingleCbfTmp;
              cSingleComponentPredictionAlpha[colorSpaceId][compID]  = (crossCPredictionModeId != 0) ? pcCU->getCrossComponentPredictionAlpha(uiAbsPartIdx, compID) : 0;
              uiSingleComponentTransformMode[colorSpaceId][compID]   = transformSkipModeId;

              xStoreIntraResultQT(compID, rTu);

              m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );

              if( compID == COMPONENT_Y )
              {
                const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
                const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
                xStoreCrossComponentPredictionResult(reconResiLuma, reconResiLumaTmp, rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE);
              }
            }
          }
        }

        pcCU->setTransformSkipPartRange(uiSingleComponentTransformMode[colorSpaceId][compID],                      compID, uiAbsPartIdx, partIndxNumPerTU );
        pcCU->setCbfPartRange          ((uiSingleComponentCbf[colorSpaceId][compID] << uiTrDepth),                 compID, uiAbsPartIdx, partIndxNumPerTU );
        pcCU->setCrossComponentPredictionAlphaPartRange(cSingleComponentPredictionAlpha[colorSpaceId][compID],     compID, uiAbsPartIdx, partIndxNumPerTU );
        xLoadIntraResultQT(compID, rTu );

        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );

        if( !pcCU->isLosslessCoded(0) && bCheckACT )
        {
          Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
          m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( - deltaQP );
          m_pcRdCost->adjustLambdaForColourTrans            ( - deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
        }
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
      m_pcEntropyCoder->resetBits();
      uiSingleColorSpaceBits[colorSpaceId] = xGetIntraBitsQT( rTu, true, true, false );

      if( bCheckACT )
      {
        m_pcQTTempTComYuv[ uiQTLayer ].convert(extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0));
      }

      for( UInt ch = 0; ch < numberValidComponents; ch++ )
      {
        ComponentID         compID           = ComponentID(ch);
        const TComRectangle &rect            = rTu.getRect(compID);
        const UInt          uiWidth          = rect.width;
        const UInt          uiHeight         = rect.height;
        Pel*                pcPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
        Pel*                pcResi           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
        Pel*                pcRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
        const UInt          uiStride         = pcPredYuv->getStride (compID);
        const UInt          uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
        const UInt          clipbd           = pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(compID)];

        for( UInt uiY = 0; uiY < uiHeight; uiY++ )
        {
          for( UInt uiX = 0; uiX < uiWidth; uiX++ )
          {
            pcRecQt[ uiX ] = Pel( ClipBD<Int>( Int(pcPred[uiX]) + Int(pcResi[uiX]), clipbd ) );
          }

          pcPred     += uiStride;
          pcResi     += uiRecQtStride;
          pcRecQt    += uiRecQtStride;
        }
      }

      uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_LUMA] = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(COMPONENT_Y)],  
                                                                                         m_pcQTTempTComYuv[uiQTLayer].getAddr(COMPONENT_Y, uiAbsPartIdx), 
                                                                                         m_pcQTTempTComYuv[uiQTLayer].getStride(COMPONENT_Y),
                                                                                         pcOrgYuv->getAddr(COMPONENT_Y, uiAbsPartIdx), 
                                                                                         pcOrgYuv->getStride(COMPONENT_Y),
                                                                                         rTu.getRect(COMPONENT_Y).width,
                                                                                         rTu.getRect(COMPONENT_Y).height,
                                                                                         COMPONENT_Y);
      uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_CHROMA] = 0;
      for(UInt ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
      {
        ComponentID compID = ComponentID(ch);
        uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_CHROMA] += m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(compID)], 
                                                                                              m_pcQTTempTComYuv[uiQTLayer].getAddr(compID, uiAbsPartIdx), 
                                                                                              m_pcQTTempTComYuv[uiQTLayer].getStride(compID), 
                                                                                              pcOrgYuv->getAddr(compID, uiAbsPartIdx), 
                                                                                              pcOrgYuv->getStride(compID), 
                                                                                              rTu.getRect(compID).width, 
                                                                                              rTu.getRect(compID).height, 
                                                                                              compID);
      }

      dSingleColorSpaceCost[colorSpaceId] = m_pcRdCost->calcRdCost( uiSingleColorSpaceBits[colorSpaceId], uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_LUMA] + uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_CHROMA] );

      if(colorSpaceId)
      {
        m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled = dSingleColorSpaceCost[colorSpaceId];
      }
      else
      {
        m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled  = dSingleColorSpaceCost[colorSpaceId];
      }

      if( dSingleColorSpaceCost[colorSpaceId] < dSingleCost )
      {
        dSingleCost                       = dSingleColorSpaceCost[colorSpaceId];
        uiSingleDist[CHANNEL_TYPE_LUMA]   = uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_LUMA];
        uiSingleDist[CHANNEL_TYPE_CHROMA] = uiSingleColorSpaceDist[colorSpaceId][CHANNEL_TYPE_CHROMA];
        uiSingleBits                      = uiSingleColorSpaceBits[colorSpaceId];
        uiSingleColorSpaceId              = colorSpaceId;

        xStoreIntraResultQT(COMPONENT_Y,  rTu, true);
        xStoreIntraResultQT(COMPONENT_Cb, rTu, true);
        xStoreIntraResultQT(COMPONENT_Cr, rTu, true);
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_CHROMA_INTRA ] );  //CI_CHROMA_INTRA is never used
      }

      if( !colorSpaceId && !uiSingleComponentCbf[colorSpaceId][COMPONENT_Y] && !uiSingleComponentCbf[colorSpaceId][COMPONENT_Cb] && !uiSingleComponentCbf[colorSpaceId][COMPONENT_Cr] )  //all cbfs are zero, skip the other color space
      {
        break;
      }
      if( !colorSpaceId && uiRelTrDepth && m_sACTRDCostTU[uiRelTrDepth-1].uiIsCSCEnabled == 1 )
      {
        break;
      }
    }

    Bool bACTEnabled = m_pcEncCfg->getRGBFormatFlag()? (!uiSingleColorSpaceId? true: false): (uiSingleColorSpaceId? true: false);
    pcCU->setColourTransformSubParts(bACTEnabled, uiAbsPartIdx, uiFullDepth);
    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      ComponentID compID = ComponentID(ch);
      pcCU->setTransformSkipPartRange(uiSingleComponentTransformMode[uiSingleColorSpaceId][compID],                      compID, uiAbsPartIdx, partIndxNumPerTU );
      pcCU->setCbfPartRange          ((uiSingleComponentCbf[uiSingleColorSpaceId][compID] << uiTrDepth),                 compID, uiAbsPartIdx, partIndxNumPerTU );
      pcCU->setCrossComponentPredictionAlphaPartRange(cSingleComponentPredictionAlpha[uiSingleColorSpaceId][compID],     compID, uiAbsPartIdx, partIndxNumPerTU );
    }

    xLoadIntraResultQT(COMPONENT_Y,  rTu, true );
    xLoadIntraResultQT(COMPONENT_Cb, rTu, true );
    xLoadIntraResultQT(COMPONENT_Cr, rTu, true );
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_CHROMA_INTRA ] );

    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      ComponentID         compID           = ComponentID(ch);
      const TComRectangle &rect            = rTu.getRect(compID);
      const UInt          uiWidth          = rect.width;
      const UInt          uiHeight         = rect.height;
      const UInt          uiZOrder         = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
      Pel*                pcRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      Pel*                piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      const UInt          uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
      const UInt          uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride (compID);
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piRecIPred[ uiX ] = pcRecQt[ uiX ];
        }  
        pcRecQt    += uiRecQtStride;
        piRecIPred += uiRecIPredStride;
      }
    }
  }

  if( m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled != MAX_DOUBLE && m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled != MAX_DOUBLE )
  {
    if( m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled < m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled )
    {
      m_sACTRDCostTU[uiRelTrDepth].uiIsCSCEnabled = 0;
    }
    else
    {
      m_sACTRDCostTU[uiRelTrDepth].uiIsCSCEnabled = 1;
    }
  }
  else if( m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled == MAX_DOUBLE && m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled == MAX_DOUBLE )
  {
    m_sACTRDCostTU[uiRelTrDepth].uiIsCSCEnabled = 2;
  }
  else if( m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCEnabled != MAX_DOUBLE && m_sACTRDCostTU[uiRelTrDepth].tmpRDCostCSCDisabled == MAX_DOUBLE )
  {
    m_sACTRDCostTU[uiRelTrDepth].uiIsCSCEnabled = 1;
  }
  else
  {
    assert(0);
  }

  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    } 
    else
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }

    Double     dSplitCost                         = 0.0;
    Distortion uiSplitDist[MAX_NUM_CHANNEL_TYPE]  = {0, 0};
    UInt       uiSplitBits                        = 0;
    UInt       uiSplitCbf[MAX_NUM_COMPONENT]      = {0, 0, 0};

    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xRecurIntraCodingQTTUCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiSplitDist[CHANNEL_TYPE_LUMA], uiSplitDist[CHANNEL_TYPE_CHROMA], dSplitCost, tuRecurseChild, bTestMaxTUSize, eACTRDTestType DEBUG_STRING_PASS_INTO(sDebug) );

      for(UInt ch = 0; ch < numberValidComponents; ch++)
      {
        uiSplitCbf[ch] |= pcCU->getCbf( tuRecurseChild.GetAbsPartIdxTU(), ComponentID(ch), tuRecurseChild.GetTransformDepthRel() );
      }
    } while ( tuRecurseChild.nextSection(rTu) );

    UInt uiPartsDiv = rTu.GetAbsPartIdxNumParts();
    for(UInt ch = 0; ch < numberValidComponents; ch++)
    {
      if (uiSplitCbf[ch])
      {
        const UInt        flag   = 1 << uiTrDepth;
        const ComponentID compID = ComponentID(ch);
        UChar             *pBase = pcCU->getCbf( compID );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }

    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    uiSplitBits += xGetIntraBitsQT( rTu, true, true, false );
    dSplitCost  = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDist[CHANNEL_TYPE_LUMA] + uiSplitDist[CHANNEL_TYPE_CHROMA] );

    if( dSplitCost < dSingleCost )
    {
      uiPUDistY += uiSplitDist[CHANNEL_TYPE_LUMA];
      uiPUDistC += uiSplitDist[CHANNEL_TYPE_CHROMA];
      dPUCost   += dSplitCost;
      return;
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );
    Bool bACTEnabled = m_pcEncCfg->getRGBFormatFlag()? (!uiSingleColorSpaceId? true: false): (uiSingleColorSpaceId? true: false);
    pcCU->setColourTransformSubParts(bACTEnabled, uiAbsPartIdx, uiFullDepth);

    for(UInt ch = 0; ch < numberValidComponents; ch++)
    {
      const ComponentID   compID            = ComponentID(ch);
      const TComRectangle &tuRect           = rTu.getRect(compID);
      const UInt          subTUAbsPartIdx   = rTu.GetAbsPartIdxTU(compID);
      const UInt          partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID);
      assert(subTUAbsPartIdx == uiAbsPartIdx);

      pcCU->setTransformSkipPartRange(uiSingleComponentTransformMode[uiSingleColorSpaceId][compID], compID, subTUAbsPartIdx, partIdxesPerSubTU );
      pcCU->setCbfPartRange( (uiSingleComponentCbf[uiSingleColorSpaceId][compID] << uiTrDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU );
      pcCU->setCrossComponentPredictionAlphaPartRange(cSingleComponentPredictionAlpha[uiSingleColorSpaceId][compID], compID, subTUAbsPartIdx, partIdxesPerSubTU );

      //--- set reconstruction for next intra TU
      const UInt  uiZOrder    = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
      const UInt  uiWidth     = tuRect.width;
      const UInt  uiHeight    = tuRect.height;
      Pel*        piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt        uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( compID );
      Pel*        piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      UInt        uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );

      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }
  }
  uiPUDistY += uiSingleDist[CHANNEL_TYPE_LUMA];
  uiPUDistC += uiSingleDist[CHANNEL_TYPE_CHROMA];
  dPUCost   += dSingleCost;
}
#endif

Void
TEncSearch::xRecurIntraCodingQTCSC( TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, Distortion& uiPUDistY, Distortion& uiPUDistC, Double& dPUCost, TComTU& rTu, Bool bTestMaxTUSize DEBUG_STRING_FN_DECLARE(sDebug) )
{
  TComDataCU          *pcCU                 = rTu.getCU();
  const UInt          uiTrDepth             = rTu.GetTransformDepthRel();
  const UInt          uiFullDepth           = rTu.GetTransformDepthTotal();
  const UInt          uiAbsPartIdx          = rTu.GetAbsPartIdxTU();
  const UInt          partIndxNumPerTU      = rTu.GetAbsPartIdxNumParts(COMPONENT_Y);
  const ChromaFormat  chFmt                 = rTu.GetChromaFormat();
  const UInt          uiLog2TrSize          = rTu.GetLog2LumaTrSize();
  const UInt          uiQTLayer             = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
  const UInt          numberValidComponents = getNumberValidComponents(chFmt);
  Bool                bCheckFull            = ( uiLog2TrSize  <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  Bool                bCheckSplit           = ( bTestMaxTUSize && bCheckFull )? false: ( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
  const Bool          extendedPrecision     = rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();

#if SCM_U0095_FAST_INTRA_ACT
  UInt uiDepth = pcCU->getDepth(uiAbsPartIdx);
#endif
  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && bCheckFull)
  {
    bCheckSplit = false;
  }
#if SCM_U0095_FAST_INTRA_ACT
  else if ( m_pcEncCfg->getNoTUSplitIntraACTEnabled() && uiDepth <= 1 && bCheckFull )
  {
    bCheckSplit = false;
  }
#endif

  assert( bCheckFull || bCheckSplit );
  assert( chFmt == CHROMA_444);

  Double              dSingleCost                                 = MAX_DOUBLE;
  Distortion          uiSingleDist[MAX_NUM_CHANNEL_TYPE]          = {0, 0};
  UInt                uiSingleBits                                = 0;

  Double              dSingleComponentCost[MAX_NUM_COMPONENT]              = {MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE};
  UInt                uiSingleComponentCbf[MAX_NUM_COMPONENT]              = {0, 0, 0};
  UInt                uiSingleComponentTransformMode[MAX_NUM_COMPONENT]    = {0, 0, 0};
  Char                cSingleComponentPredictionAlpha[MAX_NUM_COMPONENT]  = {0, 0, 0};

  if( bCheckFull )
  {
    m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );

    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );

    //intra prediction
    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID   compID    = ComponentID(ch);
      const ChannelType   chType    = toChannelType(compID);
      const TComRectangle &rect     = rTu.getRect(compID);
      const UInt          uiWidth   = rect.width;
      const UInt          uiHeight  = rect.height;

      const Bool bIsLuma            = isLuma(compID);
      const UInt uiChPredMode       = pcCU->getIntraDir( chType, uiAbsPartIdx );
      const UInt uiChFinalMode      = (uiChPredMode == DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx) : uiChPredMode;

      const Bool bUseFilteredPredictions = TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getSpsRangeExtension().getIntraSmoothingDisabledFlag());
      Pel*       piOrg                   = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
      Pel*       piPred                  = pcPredYuv->getAddr( compID, uiAbsPartIdx );
      Pel*       piResi                  = pcResiYuv->getAddr( compID, uiAbsPartIdx );
      const UInt uiStride                = pcOrgYuv ->getStride (compID);

      DEBUG_STRING_NEW(sTemp)
      initIntraPatternChType( rTu, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sTemp) );
      predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bUseFilteredPredictions );

      Pel* pcOrg  = piOrg;
      Pel* pcPred = piPred;
      Pel* pcResi = piResi;

      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for ( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pcResi[uiX] = pcOrg[uiX] - pcPred[uiX];
        }

        pcOrg  += uiStride;
        pcResi += uiStride;
        pcPred += uiStride; 
      }
    }

    pcResiYuv->convert(extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, true, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0));
    Char preCalcAlpha[MAX_NUM_COMPONENT] = {0, 0, 0};

    if( pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && !m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() ) 
    {
      for( UInt ch = 0; ch < numberValidComponents; ch++ )
      {
        const ComponentID    compID                        = ComponentID(ch);
        const TComRectangle &tuCompRect                    = rTu.getRect(compID);
        const Pel  *const    lumaResidualForEstimate       = pcResiYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
        const UInt           lumaResidualStrideForEstimate = pcResiYuv->getStride ( COMPONENT_Y );

        const UInt           uiChPredMode                  = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
        const Bool bDecorrelationAvailable                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX);
        if( bDecorrelationAvailable )
        {
          preCalcAlpha[compID] = xCalcCrossComponentPredictionAlpha(rTu,
            compID,
            lumaResidualForEstimate,
            pcResiYuv->getAddr( compID, uiAbsPartIdx ),
            tuCompRect.width,
            tuCompRect.height,
            lumaResidualStrideForEstimate,
            pcResiYuv->getStride( compID )
            );
        }
      }
    }

    Pel reconResiLuma   [MAX_CU_SIZE * MAX_CU_SIZE];
    Pel reconResiLumaTmp[MAX_CU_SIZE * MAX_CU_SIZE];

    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      Double     dSingleCostTmp  = MAX_DOUBLE;
      UInt       uiSingleBitsTmp = 0;
      Distortion uiSingleDistTmp = 0;
      UInt       uiSingleCbfTmp  = 0;

      const ComponentID    compID     = ComponentID(ch);
      const TComRectangle &tuCompRect = rTu.getRect(compID);
#if SCM_U0106_ACT_TU_SIG
      assert(pcCU->getColourTransform(uiAbsPartIdx));
      QpParam cQP(*pcCU, compID, uiAbsPartIdx);
#else
      QpParam cQP(*pcCU, compID);
#endif
      if(!pcCU->isLosslessCoded(0))
      {
        Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
        m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( deltaQP );
        m_pcRdCost->adjustLambdaForColourTrans( deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
      }

      Bool bCheckTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
        TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()) &&
        (!pcCU->isLosslessCoded(0));
      if( m_pcEncCfg->getUseTransformSkipFast() )
      {
        bCheckTransformSkip &= (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN);
        if ( isChroma( compID ) )
        {
          bCheckTransformSkip &= (pcCU->getTransformSkip( uiAbsPartIdx, COMPONENT_Y ) != 0);
        }
      }

      Bool bCheckCrossComponentPrediction =    isChroma(compID)
        && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
        && (pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx ) == DM_CHROMA_IDX)
        && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrDepth) != 0);

      Char cCalcAlpha = 0;
      if ( bCheckCrossComponentPrediction && m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() )
      {
        cCalcAlpha = xCalcCrossComponentPredictionAlpha(rTu,
          compID,
          reconResiLuma + tuCompRect.y0 * MAX_CU_SIZE + tuCompRect.x0,
          pcResiYuv->getAddr( compID, uiAbsPartIdx),
          tuCompRect.width,
          tuCompRect.height,
          MAX_CU_SIZE,
          pcResiYuv->getStride( compID )
          );
      }
      else if ( bCheckCrossComponentPrediction )
      {
        cCalcAlpha = preCalcAlpha[compID];
      }
      bCheckCrossComponentPrediction &= (cCalcAlpha != 0);

      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

      const Int transformSkipModesToTest    = bCheckTransformSkip               ? 2 : 1;
      const Int crossCPredictionModesToTest = bCheckCrossComponentPrediction    ? 2 : 1;

      for ( Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++ )
      {
        for ( Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++ )
        {
          DEBUG_STRING_NEW(sModeString)
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_TEST] );
          m_pcEntropyCoder->resetBits();

          pcCU->setTransformSkipPartRange( transformSkipModeId, compID, uiAbsPartIdx, partIndxNumPerTU );
          pcCU->setCrossComponentPredictionAlphaPartRange( (crossCPredictionModeId != 0) ? cCalcAlpha : 0, compID, uiAbsPartIdx, partIndxNumPerTU );

          xIntraCodingTUBlockCSC( pcResiYuv, (compID == COMPONENT_Y) ? reconResiLumaTmp : reconResiLuma, (crossCPredictionModeId != 0), compID, rTu, cQP DEBUG_STRING_PASS_INTO(sModeString) );
          uiSingleCbfTmp = pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth );

          if ( (uiSingleCbfTmp == 0) && (transformSkipModeId != 0) )
          {
            dSingleCostTmp = MAX_DOUBLE;
          }
          else
          {
            if ( compID == COMPONENT_Y )
            {
              uiSingleBitsTmp = xGetIntraBitsQT( rTu, true, false, false );
            }
            else
            {
              uiSingleBitsTmp = xGetIntraBitsQTChroma( rTu, compID, false );
            }

            //residual distortion in YCgCo domain
            uiSingleDistTmp = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType( compID )],
                                                       m_pcQTTempTComYuv[uiQTLayer].getAddr( compID, uiAbsPartIdx ),
                                                       m_pcQTTempTComYuv[uiQTLayer].getStride( compID ),
                                                       pcResiYuv->getAddr( compID, uiAbsPartIdx ),
                                                       pcResiYuv->getStride( compID ),
                                                       rTu.getRect( compID ).width,
                                                       rTu.getRect( compID ).height,
                                                       compID );
            dSingleCostTmp = m_pcRdCost->calcRdCost( uiSingleBitsTmp, uiSingleDistTmp );
          }

          if ( dSingleCostTmp < dSingleComponentCost[compID] )
          {
            dSingleComponentCost[compID]             = dSingleCostTmp;
            uiSingleComponentCbf[compID]             = uiSingleCbfTmp;
            cSingleComponentPredictionAlpha[compID]  = (crossCPredictionModeId != 0) ? pcCU->getCrossComponentPredictionAlpha( uiAbsPartIdx, compID ) : 0;
            uiSingleComponentTransformMode[compID]   = transformSkipModeId;

            xStoreIntraResultQT( compID, rTu );

            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_TEMP_BEST] );

            if ( compID == COMPONENT_Y )
            {
              const Int xOffset = rTu.getRect( COMPONENT_Y ).x0;
              const Int yOffset = rTu.getRect( COMPONENT_Y ).y0;
              xStoreCrossComponentPredictionResult( reconResiLuma, reconResiLumaTmp, rTu, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
            }
          }
        }
      }

      pcCU->setTransformSkipPartRange(uiSingleComponentTransformMode[compID],                      compID, uiAbsPartIdx, partIndxNumPerTU );
      pcCU->setCbfPartRange          ((uiSingleComponentCbf[compID] << uiTrDepth),                 compID, uiAbsPartIdx, partIndxNumPerTU );
      pcCU->setCrossComponentPredictionAlphaPartRange(cSingleComponentPredictionAlpha[compID],     compID, uiAbsPartIdx, partIndxNumPerTU );

      xLoadIntraResultQT(compID, rTu );

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      if(!pcCU->isLosslessCoded(0))
      {
        Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
        m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( - deltaQP );
        m_pcRdCost->adjustLambdaForColourTrans            ( - deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
      }
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    uiSingleBits = xGetIntraBitsQT( rTu, true, true, false );

    m_pcQTTempTComYuv[ uiQTLayer ].convert(extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0));

    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      ComponentID         compID           = ComponentID(ch);
      const TComRectangle &rect            = rTu.getRect(compID);
      const UInt          uiWidth          = rect.width;
      const UInt          uiHeight         = rect.height;
      const UInt          uiZOrder         = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
      Pel*                pcPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
      Pel*                pcResi           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      Pel*                pcRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      Pel*                piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      const UInt          uiStride         = pcPredYuv->getStride (compID);
      const UInt          uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
      const UInt          uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride (compID);
      const UInt          clipbd           = pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(compID)];

      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pcRecQt[ uiX ] = Pel( ClipBD<Int>( Int(pcPred[uiX]) + Int(pcResi[uiX]), clipbd ) );
          piRecIPred[ uiX ] = pcRecQt[ uiX ];
        }

        pcPred     += uiStride;
        pcResi     += uiRecQtStride;
        pcRecQt    += uiRecQtStride;
        piRecIPred += uiRecIPredStride;
      }
    }

    uiSingleDist[CHANNEL_TYPE_LUMA] = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(COMPONENT_Y)],  
      m_pcQTTempTComYuv[uiQTLayer].getAddr(COMPONENT_Y, uiAbsPartIdx), 
      m_pcQTTempTComYuv[uiQTLayer].getStride(COMPONENT_Y),
      pcOrgYuv->getAddr(COMPONENT_Y, uiAbsPartIdx), 
      pcOrgYuv->getStride(COMPONENT_Y),
      rTu.getRect(COMPONENT_Y).width,
      rTu.getRect(COMPONENT_Y).height,
      COMPONENT_Y);
    uiSingleDist[CHANNEL_TYPE_CHROMA] = 0;
    for(UInt ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
    {
      ComponentID compID = ComponentID(ch);
      uiSingleDist[CHANNEL_TYPE_CHROMA] += m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(compID)], 
        m_pcQTTempTComYuv[uiQTLayer].getAddr(compID, uiAbsPartIdx), 
        m_pcQTTempTComYuv[uiQTLayer].getStride(compID), 
        pcOrgYuv->getAddr(compID, uiAbsPartIdx), 
        pcOrgYuv->getStride(compID), 
        rTu.getRect(compID).width, 
        rTu.getRect(compID).height, 
        compID);
    }

    dSingleCost = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist[CHANNEL_TYPE_LUMA] + uiSingleDist[CHANNEL_TYPE_CHROMA] );
  }

  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    else
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiFullDepth][CI_QT_TRAFO_ROOT] );
    }

    Double     dSplitCost                         = 0.0;
    Distortion uiSplitDist[MAX_NUM_CHANNEL_TYPE]  = {0, 0};
    UInt       uiSplitBits                        = 0;
    UInt       uiSplitCbf[MAX_NUM_COMPONENT]      = {0, 0, 0};

    TComTURecurse tuRecurseChild(rTu, false);
    do 
    {
      xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiSplitDist[CHANNEL_TYPE_LUMA], uiSplitDist[CHANNEL_TYPE_CHROMA], dSplitCost, tuRecurseChild, bTestMaxTUSize DEBUG_STRING_PASS_INTO(sDebug) );

      for(UInt ch = 0; ch < numberValidComponents; ch++) 
        uiSplitCbf[ch] |= pcCU->getCbf( tuRecurseChild.GetAbsPartIdxTU(), ComponentID(ch), tuRecurseChild.GetTransformDepthRel() );
    } while ( tuRecurseChild.nextSection(rTu) );

    UInt uiPartsDiv = rTu.GetAbsPartIdxNumParts();
    for(UInt ch = 0; ch < numberValidComponents; ch++)
    {
      if (uiSplitCbf[ch])
      {
        const UInt        flag   = 1 << uiTrDepth;
        const ComponentID compID = ComponentID(ch);
        UChar             *pBase = pcCU->getCbf( compID );
        for ( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[uiAbsPartIdx + uiOffs] |= flag;
        }
      }
    }

    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    uiSplitBits += xGetIntraBitsQT( rTu, true, true, false );
    dSplitCost  = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDist[CHANNEL_TYPE_LUMA] + uiSplitDist[CHANNEL_TYPE_CHROMA] );

    if( dSplitCost < dSingleCost )
    {
      uiPUDistY += uiSplitDist[CHANNEL_TYPE_LUMA];
      uiPUDistC += uiSplitDist[CHANNEL_TYPE_CHROMA];
      dPUCost   += dSplitCost;
      return;
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_TEST ] );

    pcCU->setTrIdxSubParts( uiTrDepth, uiAbsPartIdx, uiFullDepth );

    for(UInt ch = 0; ch < numberValidComponents; ch++)
    {
      const ComponentID   compID            = ComponentID(ch);
      const TComRectangle &tuRect           = rTu.getRect(compID);
      const UInt          subTUAbsPartIdx   = rTu.GetAbsPartIdxTU(compID);
      const UInt          partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID);
      assert(subTUAbsPartIdx == uiAbsPartIdx);

      pcCU->setTransformSkipPartRange(uiSingleComponentTransformMode[compID], compID, subTUAbsPartIdx, partIdxesPerSubTU );
      pcCU->setCbfPartRange( (uiSingleComponentCbf[compID] << uiTrDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU );
      pcCU->setCrossComponentPredictionAlphaPartRange(cSingleComponentPredictionAlpha[compID], compID, subTUAbsPartIdx, partIdxesPerSubTU );

      //--- set reconstruction for next intra TU
      const UInt  uiZOrder    = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
      const UInt  uiWidth     = tuRect.width;
      const UInt  uiHeight    = tuRect.height;
      Pel*        piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt        uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( compID );
      Pel*        piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
      UInt        uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );

      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for ( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[uiX] = piSrc[uiX];
        }
      }
    }
  }
  uiPUDistY += uiSingleDist[CHANNEL_TYPE_LUMA];
  uiPUDistC += uiSingleDist[CHANNEL_TYPE_CHROMA];
  dPUCost   += dSingleCost;
}


Void
TEncSearch::xSetIntraResultChromaQT(TComYuv*    pcRecoYuv, TComTU &rTu)
{
  if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
  {
    return;
  }
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth   = rTu.GetTransformDepthRel();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    //===== copy transform coefficients =====
    const TComRectangle &tuRectCb=rTu.getRect(COMPONENT_Cb);
    UInt uiNumCoeffC    = tuRectCb.width*tuRectCb.height;//( pcCU->getSlice()->getSPS()->getMaxCUWidth() * pcCU->getSlice()->getSPS()->getMaxCUHeight() ) >> ( uiFullDepth << 1 );
    const UInt offset = rTu.getCoefficientOffset(COMPONENT_Cb);

    const UInt numberValidComponents = getNumberValidComponents(rTu.GetChromaFormat());
    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID component = ComponentID(ch);
      const TCoeff* src           = m_ppcQTTempCoeff[component][uiQTLayer] + offset;//(uiNumCoeffIncC*uiAbsPartIdx);
      TCoeff* dest                = pcCU->getCoeff(component) + offset;//(uiNumCoeffIncC*uiAbsPartIdx);
      ::memcpy( dest, src, sizeof(TCoeff)*uiNumCoeffC );
#if ADAPTIVE_QP_SELECTION
      TCoeff* pcArlCoeffSrc = m_ppcQTTempArlCoeff[component][ uiQTLayer ] + offset;//( uiNumCoeffIncC * uiAbsPartIdx );
      TCoeff* pcArlCoeffDst = pcCU->getArlCoeff(component)                + offset;//( uiNumCoeffIncC * uiAbsPartIdx );
      ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeffC );
#endif
    }

    //===== copy reconstruction =====

    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Cb, pcRecoYuv, uiAbsPartIdx, tuRectCb.width, tuRectCb.height );
    m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( COMPONENT_Cr, pcRecoYuv, uiAbsPartIdx, tuRectCb.width, tuRectCb.height );
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetIntraResultChromaQT( pcRecoYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}



Void
TEncSearch::estIntraPredLumaQT(TComDataCU* pcCU,
                               TComYuv*    pcOrgYuv,
                               TComYuv*    pcPredYuv,
                               TComYuv*    pcResiYuv,
                               TComYuv*    pcRecoYuv,
                               Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                               DEBUG_STRING_FN_DECLARE(sDebug))
{
  const UInt         uiDepth               = pcCU->getDepth(0);
  const UInt         uiInitTrDepth         = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  const UInt         uiNumPU               = 1<<(2*uiInitTrDepth);
  const UInt         uiQNumParts           = pcCU->getTotalNumPart() >> 2;
  const UInt         uiWidthBit            = pcCU->getIntraSizeIdx(0);
  const ChromaFormat chFmt                 = pcCU->getPic()->getChromaFormat();
  const UInt         numberValidComponents = getNumberValidComponents(chFmt);
  const TComSPS     &sps                   = *(pcCU->getSlice()->getSPS());
  const TComPPS     &pps                   = *(pcCU->getSlice()->getPPS());
        Distortion   uiOverallDistY        = 0;
        UInt         CandNum;
        Double       CandCostList[ FAST_UDI_MAX_RDMODE_NUM ];
        Pel          resiLumaPU[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

        Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
        for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
        {
          bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
        }

        bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantisation divisor is 1.
#if FULL_NBIT
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#else
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (sps.getBitDepth(CHANNEL_TYPE_LUMA) - 8)) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#endif

  //===== set QP and clear Cbf =====
  if ( pps.getUseDQP() == true)
  {
    pcCU->setQPSubParts( pcCU->getQP(0), 0, uiDepth );
  }
  else
  {
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  }

  //===== loop over partitions =====
  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

  do
  {
    const UInt uiPartOffset=tuRecurseWithPU.GetAbsPartIdxTU();
//  for( UInt uiPU = 0, uiPartOffset=0; uiPU < uiNumPU; uiPU++, uiPartOffset += uiQNumParts )
  //{
    //===== init pattern for luma prediction =====
    DEBUG_STRING_NEW(sTemp2)

    //===== determine set of modes to be tested (using prediction signal only) =====
    Int numModesAvailable     = 35; //total number of Intra modes
    UInt uiRdModeList[FAST_UDI_MAX_RDMODE_NUM];
    Int numModesForFullRD = m_pcEncCfg->getFastUDIUseMPMEnabled()?g_aucIntraModeNumFast_UseMPM[ uiWidthBit ] : g_aucIntraModeNumFast_NotUseMPM[ uiWidthBit ];

    // this should always be true
    assert (tuRecurseWithPU.ProcessComponentSection(COMPONENT_Y));
    initIntraPatternChType( tuRecurseWithPU, COMPONENT_Y, true DEBUG_STRING_PASS_INTO(sTemp2) );

    Bool doFastSearch = (numModesForFullRD != numModesAvailable);
    if (doFastSearch)
    {
      assert(numModesForFullRD < numModesAvailable);

      for( Int i=0; i < numModesForFullRD; i++ )
      {
        CandCostList[ i ] = MAX_DOUBLE;
      }
      CandNum = 0;

      const TComRectangle &puRect=tuRecurseWithPU.getRect(COMPONENT_Y);
      const UInt uiAbsPartIdx=tuRecurseWithPU.GetAbsPartIdxTU();

      Pel* piOrg         = pcOrgYuv ->getAddr( COMPONENT_Y, uiAbsPartIdx );
      Pel* piPred        = pcPredYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
      UInt uiStride      = pcPredYuv->getStride( COMPONENT_Y );
      DistParam distParam;
      const Bool bUseHadamard=pcCU->getCUTransquantBypass(0) == 0;
      m_pcRdCost->setDistParam(distParam, sps.getBitDepth(CHANNEL_TYPE_LUMA), piOrg, uiStride, piPred, uiStride, puRect.width, puRect.height, bUseHadamard);
      distParam.bApplyWeight = false;
      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt       uiMode = modeIdx;
        Distortion uiSad  = 0;

        const Bool bUseFilter=TComPrediction::filteringIntraReferenceSamples(COMPONENT_Y, uiMode, puRect.width, puRect.height, chFmt, sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag());

        predIntraAng( COMPONENT_Y, uiMode, piOrg, uiStride, piPred, uiStride, tuRecurseWithPU, bUseFilter, TComPrediction::UseDPCMForFirstPassIntraEstimation(tuRecurseWithPU, uiMode) );

        // use hadamard transform here
        uiSad+=distParam.DistFunc(&distParam);

        UInt   iModeBits = 0;

        // NB xModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
        iModeBits+=xModeBitsIntra( pcCU, uiMode, uiPartOffset, uiDepth, CHANNEL_TYPE_LUMA );

        Double cost      = (Double)uiSad + (Double)iModeBits * sqrtLambdaForFirstPass;

#if DEBUG_INTRA_SEARCH_COSTS
        std::cout << "1st pass mode " << uiMode << " SAD = " << uiSad << ", mode bits = " << iModeBits << ", cost = " << cost << "\n";
#endif

        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

      if (m_pcEncCfg->getFastUDIUseMPMEnabled())
      {
        Int uiPreds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};

        Int iMode = -1;
        pcCU->getIntraDirPredictor( uiPartOffset, uiPreds, COMPONENT_Y, &iMode );

        const Int numCand = ( iMode >= 0 ) ? iMode : Int(NUM_MOST_PROBABLE_MODES);

        for( Int j=0; j < numCand; j++)
        {
          Bool mostProbableModeIncluded = false;
          Int mostProbableMode = uiPreds[j];

          for( Int i=0; i < numModesForFullRD; i++)
          {
            mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i]);
          }
          if (!mostProbableModeIncluded)
          {
            uiRdModeList[numModesForFullRD++] = mostProbableMode;
          }
        }
      }
    }
    else
    {
      for( Int i=0; i < numModesForFullRD; i++)
      {
        uiRdModeList[i] = i;
      }
    }

    //===== check modes (using r-d costs) =====
#if HHI_RQT_INTRA_SPEEDUP_MOD
    UInt   uiSecondBestMode  = MAX_UINT;
    Double dSecondBestPUCost = MAX_DOUBLE;
#endif
    DEBUG_STRING_NEW(sPU)
    UInt       uiBestPUMode  = 0;
    Distortion uiBestPUDistY = 0;
    Double     dBestPUCost   = MAX_DOUBLE;

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
    UInt max=numModesForFullRD;

    if (DebugOptionList::ForceLumaMode.isSet())
    {
      max=0;  // we are forcing a direction, so don't bother with mode check
    }
    for ( UInt uiMode = 0; uiMode < max; uiMode++)
#else
    for( UInt uiMode = 0; uiMode < numModesForFullRD; uiMode++ )
#endif
    {
      // set luma prediction mode
      UInt uiOrgMode = uiRdModeList[uiMode];

      pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );

      DEBUG_STRING_NEW(sMode)
      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      // determine residual for partition
      Distortion uiPUDistY = 0;
      Double     dPUCost   = 0.0;
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, true, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#else
      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#endif

#if DEBUG_INTRA_SEARCH_COSTS
      std::cout << "2nd pass [luma,chroma] mode [" << Int(pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiPartOffset)) << "," << Int(pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiPartOffset)) << "] cost = " << dPUCost << "\n";
#endif

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        DEBUG_STRING_SWAP(sPU, sMode)
#if HHI_RQT_INTRA_SPEEDUP_MOD
        uiSecondBestMode  = uiBestPUMode;
        dSecondBestPUCost = dBestPUCost;
#endif
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        dBestPUCost   = dPUCost;

        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );

        if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
        {
          const Int xOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).x0;
          const Int yOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).y0;
          for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
          {
            if (bMaintainResidual[storedResidualIndex])
            {
              xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaPU[storedResidualIndex], tuRecurseWithPU, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
            }
          }
        }

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();

        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID  ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],  pcCU->getTransformSkip(compID)  + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        }
      }
#if HHI_RQT_INTRA_SPEEDUP_MOD
      else if( dPUCost < dSecondBestPUCost )
      {
        uiSecondBestMode  = uiOrgMode;
        dSecondBestPUCost = dPUCost;
      }
#endif
    } // Mode loop

#if HHI_RQT_INTRA_SPEEDUP
#if HHI_RQT_INTRA_SPEEDUP_MOD
    for( UInt ui =0; ui < 2; ++ui )
#endif
    if(!m_pcEncCfg->getTransquantBypassInferTUSplit() || !pcCU->isLosslessCoded(0))
    {
#if HHI_RQT_INTRA_SPEEDUP_MOD
      UInt uiOrgMode   = ui ? uiSecondBestMode  : uiBestPUMode;
      if( uiOrgMode == MAX_UINT )
      {
        break;
      }
#else
      UInt uiOrgMode = uiBestPUMode;
#endif

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      if (DebugOptionList::ForceLumaMode.isSet())
      {
        uiOrgMode = DebugOptionList::ForceLumaMode.getInt();
      }
#endif

      pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      DEBUG_STRING_NEW(sModeTree)

      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      // determine residual for partition
      Distortion uiPUDistY = 0;
      Double     dPUCost   = 0.0;

      xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, false, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sModeTree));

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        DEBUG_STRING_SWAP(sPU, sModeTree)
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        dBestPUCost   = dPUCost;

        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );

        if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
        {
          const Int xOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).x0;
          const Int yOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).y0;
          for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
          {
            if (bMaintainResidual[storedResidualIndex])
            {
              xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaPU[storedResidualIndex], tuRecurseWithPU, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
            }
          }
        }

        const UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx()       + uiPartOffset, uiQPartNum * sizeof( UChar ) );

        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID  ) + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],  pcCU->getTransformSkip(compID)  + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        }
      }
    } // Mode loop
#endif

    DEBUG_STRING_APPEND(sDebug, sPU)

    //--- update overall distortion ---
    uiOverallDistY += uiBestPUDistY;

    //--- update transform index and cbf ---
    const UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
    ::memcpy( pcCU->getTransformIdx()       + uiPartOffset, m_puhQTTempTrIdx,  uiQPartNum * sizeof( UChar ) );
    for (UInt component = 0; component < numberValidComponents; component++)
    {
      const ComponentID compID = ComponentID(component);
      ::memcpy( pcCU->getCbf( compID  ) + uiPartOffset, m_puhQTTempCbf[compID], uiQPartNum * sizeof( UChar ) );
      ::memcpy( pcCU->getTransformSkip( compID  ) + uiPartOffset, m_puhQTTempTransformSkipFlag[compID ], uiQPartNum * sizeof( UChar ) );
    }

    //--- set reconstruction for next intra prediction blocks ---
    if( !tuRecurseWithPU.IsLastSection() )
    {
      const TComRectangle &puRect=tuRecurseWithPU.getRect(COMPONENT_Y);
      const UInt  uiCompWidth   = puRect.width;
      const UInt  uiCompHeight  = puRect.height;

      const UInt  uiZOrder      = pcCU->getZorderIdxInCtu() + uiPartOffset;
            Pel*  piDes         = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), uiZOrder );
      const UInt  uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride( COMPONENT_Y);
      const Pel*  piSrc         = pcRecoYuv->getAddr( COMPONENT_Y, uiPartOffset );
      const UInt  uiSrcStride   = pcRecoYuv->getStride( COMPONENT_Y);

      for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }

    //=== update PU data ====
    pcCU->setIntraDirSubParts     ( CHANNEL_TYPE_LUMA, uiBestPUMode, uiPartOffset, uiDepth + uiInitTrDepth );
  } while (tuRecurseWithPU.nextSection(tuRecurseCU));


  if( uiNumPU > 1 )
  { // set Cbf for all blocks
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, COMPONENT_Y,  1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Y  )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  //===== reset context models =====
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  //===== set distortion (rate and r-d costs are determined later) =====
  pcCU->getTotalDistortion() = uiOverallDistY;
}

Void
TEncSearch::estIntraPredChromaQT(TComDataCU* pcCU,
                                 TComYuv*    pcOrgYuv,
                                 TComYuv*    pcPredYuv,
                                 TComYuv*    pcResiYuv,
                                 TComYuv*    pcRecoYuv,
                                 Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                 DEBUG_STRING_FN_DECLARE(sDebug))
{
  assert( pcCU->getColourTransform( 0 ) == false );

  const UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;

  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);
  const UInt    uiQNumParts    = tuRecurseWithPU.GetAbsPartIdxNumParts();
  const UInt    uiDepthCU=tuRecurseWithPU.getCUDepth();
  const UInt    numberValidComponents = pcCU->getPic()->getNumberValidComponents();

  do
  {
    UInt       uiBestMode  = 0;
    Distortion uiBestDist  = 0;
    Double     dBestCost   = MAX_DOUBLE;

    //----- init mode list -----
    if (tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      UInt uiModeList[FAST_UDI_MAX_RDMODE_NUM];
      const UInt  uiQPartNum     = uiQNumParts;
      const UInt  uiPartOffset   = tuRecurseWithPU.GetAbsPartIdxTU();
      {
        UInt  uiMinMode = 0;
        UInt  uiMaxMode = NUM_CHROMA_MODE;

        //----- check chroma modes -----
        pcCU->getAllowedChromaDir( uiPartOffset, uiModeList );

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        if (DebugOptionList::ForceChromaMode.isSet())
        {
          uiMinMode=DebugOptionList::ForceChromaMode.getInt();
          if (uiModeList[uiMinMode]==34)
          {
            uiMinMode=4; // if the fixed mode has been renumbered because DM_CHROMA covers it, use DM_CHROMA.
          }
          uiMaxMode=uiMinMode+1;
        }
#endif

        DEBUG_STRING_NEW(sPU)

        for( UInt uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++ )
        {
          //----- restore context models -----
          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
          
          DEBUG_STRING_NEW(sMode)
          //----- chroma coding -----
          Distortion uiDist = 0;
          pcCU->setIntraDirSubParts  ( CHANNEL_TYPE_CHROMA, uiModeList[uiMode], uiPartOffset, uiDepthCU+uiInitTrDepth );
          xRecurIntraChromaCodingQT       ( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, uiDist, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );

          if( pcCU->getSlice()->getPPS()->getUseTransformSkip() )
          {
            m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
          }

          UInt    uiBits = xGetIntraBitsQT( tuRecurseWithPU, false, true, false );
          Double  dCost  = m_pcRdCost->calcRdCost( uiBits, uiDist );

          //----- compare -----
          if( dCost < dBestCost )
          {
            DEBUG_STRING_SWAP(sPU, sMode);
            dBestCost   = dCost;
            uiBestDist  = uiDist;
            uiBestMode  = uiModeList[uiMode];

            xSetIntraResultChromaQT( pcRecoYuv, tuRecurseWithPU );
            for (UInt componentIndex = COMPONENT_Cb; componentIndex < numberValidComponents; componentIndex++)
            {
              const ComponentID compID = ComponentID(componentIndex);
              ::memcpy( m_puhQTTempCbf[compID], pcCU->getCbf( compID )+uiPartOffset, uiQPartNum * sizeof( UChar ) );
              ::memcpy( m_puhQTTempTransformSkipFlag[compID], pcCU->getTransformSkip( compID )+uiPartOffset, uiQPartNum * sizeof( UChar ) );
              ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID], pcCU->getCrossComponentPredictionAlpha(compID)+uiPartOffset, uiQPartNum * sizeof( Char ) );
            }
          }
        }

        DEBUG_STRING_APPEND(sDebug, sPU)

        //----- set data -----
        for (UInt componentIndex = COMPONENT_Cb; componentIndex < numberValidComponents; componentIndex++)
        {
          const ComponentID compID = ComponentID(componentIndex);
          ::memcpy( pcCU->getCbf( compID )+uiPartOffset, m_puhQTTempCbf[compID], uiQPartNum * sizeof( UChar ) );
          ::memcpy( pcCU->getTransformSkip( compID )+uiPartOffset, m_puhQTTempTransformSkipFlag[compID], uiQPartNum * sizeof( UChar ) );
          ::memcpy( pcCU->getCrossComponentPredictionAlpha(compID)+uiPartOffset, m_phQTTempCrossComponentPredictionAlpha[compID], uiQPartNum * sizeof( Char ) );
        }
      }

      if( ! tuRecurseWithPU.IsLastSection() )
      {
        for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
        {
          const ComponentID compID    = ComponentID(ch);
          const TComRectangle &tuRect = tuRecurseWithPU.getRect(compID);
          const UInt  uiCompWidth     = tuRect.width;
          const UInt  uiCompHeight    = tuRect.height;
          const UInt  uiZOrder        = pcCU->getZorderIdxInCtu() + tuRecurseWithPU.GetAbsPartIdxTU();
                Pel*  piDes           = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
          const UInt  uiDesStride     = pcCU->getPic()->getPicYuvRec()->getStride( compID);
          const Pel*  piSrc           = pcRecoYuv->getAddr( compID, uiPartOffset );
          const UInt  uiSrcStride     = pcRecoYuv->getStride( compID);

          for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
          {
            for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
            {
              piDes[ uiX ] = piSrc[ uiX ];
            }
          }
        }
      }

      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestMode, uiPartOffset, uiDepthCU+uiInitTrDepth );
      pcCU->getTotalDistortion      () += uiBestDist;
    }

  } while (tuRecurseWithPU.nextSection(tuRecurseCU));

  //----- restore context models -----

  if( uiInitTrDepth != 0 )
  { // set Cbf for all blocks
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
}


#if SCM_U0106_ACT_TU_SIG
Void
TEncSearch::estIntraPredQTCT( TComDataCU*    pcCU,
                              TComYuv*       pcOrgYuv,
                              TComYuv*       pcPredYuv,
                              TComYuv*       pcResiYuv,
                              TComYuv*       pcRecoYuv,
                              ACTRDTestTypes eACTRDTestType,
                              Bool           bReuseIntraMode
                              DEBUG_STRING_FN_DECLARE(sDebug)
                             )
#else
Void
TEncSearch::estIntraPredQTCT( TComDataCU* pcCU,
                              TComYuv* pcOrgYuv,
                              TComYuv* pcPredYuv,
                              TComYuv* pcResiYuv,
                              TComYuv* pcRecoYuv
                              DEBUG_STRING_FN_DECLARE(sDebug)
                            )
#endif
{
  const UInt         uiDepth               = pcCU->getDepth(0);
  const UInt         uiInitTrDepth         = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  const UInt         uiNumPU               = 1<<(2*uiInitTrDepth);
  const UInt         uiQNumParts           = pcCU->getTotalNumPart() >> 2;
  const ChromaFormat chFmt                 = pcCU->getPic()->getChromaFormat();
  const UInt         uiWidthBit            = pcCU->getIntraSizeIdx(0);
  const UInt         numberValidComponents = getNumberValidComponents(chFmt);
  Distortion         uiOverallDistY        = 0;
  Distortion         uiOverallDistC        = 0;
  const TComSPS     &sps                   = *(pcCU->getSlice()->getSPS());

#if !SCM_U0106_ACT_TU_SIG
  Bool                bReuse               = !m_pcEncCfg->getRGBFormatFlag();
#endif

#if FULL_NBIT
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
    sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0)))
    : m_pcRdCost->getSqrtLambda();
#else
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
    sqrt(0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8)) / 3.0)))
    : m_pcRdCost->getSqrtLambda();
#endif
  
  if ( pcCU->getSlice()->getPPS()->getUseDQP() == true )
  {
    pcCU->setQPSubParts( pcCU->getQP( 0 ), 0, uiDepth );
  }
  else
  {
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  }

  //===== loop over partitions =====
  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

  do
  {
    //luma intra mode selection
    UInt                uiRdModeList[35];
    Double              CandCostList[35];
    Int                 numModesAvailable = 35;
    Int                 numModesForFullRD = m_pcEncCfg->getFastUDIUseMPMEnabled() ? g_aucIntraModeNumFast_UseMPM[ uiWidthBit ] : g_aucIntraModeNumFast_NotUseMPM[ uiWidthBit ];
    const TComRectangle &puRect           = tuRecurseWithPU.getRect(COMPONENT_Y);
    const UInt          uiPartOffset      = tuRecurseWithPU.GetAbsPartIdxTU();
    const UInt          uiLog2TrSize      = tuRecurseWithPU.GetLog2LumaTrSize();
    UInt                CandNum;

    Pel*                piOrg             = pcOrgYuv ->getAddr( COMPONENT_Y, uiPartOffset );
    Pel*                piPred            = pcPredYuv->getAddr( COMPONENT_Y, uiPartOffset );
    UInt                uiStride          = pcPredYuv->getStride( COMPONENT_Y );

    DEBUG_STRING_NEW(sTemp2)

    for ( Int i=0; i < numModesForFullRD; i++ )
    {
      CandCostList[i] = MAX_DOUBLE;
    }
    CandNum = 0;

    initIntraPatternChType( tuRecurseWithPU, COMPONENT_Y, true DEBUG_STRING_PASS_INTO(sTemp2) );

#if SCM_U0106_ACT_TU_SIG
    if(bReuseIntraMode)
#else
    if(bReuse)
#endif
    {
      uiRdModeList [0] =tuRecurseWithPU.getCU()->getIntraDir( CHANNEL_TYPE_LUMA, tuRecurseWithPU.GetAbsPartIdxTU() );
      numModesForFullRD = 1;
    }
    else
    {
      DistParam distParam;
      const Bool bUseHadamard=pcCU->getCUTransquantBypass(0) == 0;
      m_pcRdCost->setDistParam(distParam, sps.getBitDepth(CHANNEL_TYPE_LUMA), piOrg, uiStride, piPred, uiStride, puRect.width, puRect.height, bUseHadamard);
      distParam.bApplyWeight = false;
      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt       uiMode = modeIdx;
        Distortion uiSad  = 0;

        const Bool bUseFilter = TComPrediction::filteringIntraReferenceSamples( COMPONENT_Y, uiMode, puRect.width, puRect.height, chFmt, pcCU->getSlice()->getSPS()->getSpsRangeExtension().getIntraSmoothingDisabledFlag() ); 
        predIntraAng( COMPONENT_Y, uiMode, piOrg, uiStride, piPred, uiStride, tuRecurseWithPU, bUseFilter, TComPrediction::UseDPCMForFirstPassIntraEstimation(tuRecurseWithPU, uiMode) );

        //hadamard transform
        uiSad += distParam.DistFunc(&distParam);

        UInt iModeBits = 0;
        iModeBits     += xModeBitsIntra( pcCU, uiMode, uiPartOffset, uiDepth, CHANNEL_TYPE_LUMA );
        Double cost    = (Double)uiSad + (Double)iModeBits * sqrtLambdaForFirstPass;

        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

      Int uiPreds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};
      Int iMode                            = -1;
      pcCU->getIntraDirPredictor( uiPartOffset, uiPreds, COMPONENT_Y, &iMode );
      const Int numCand = ( iMode >= 0 ) ? iMode : Int(NUM_MOST_PROBABLE_MODES);

      for( Int j=0; j < numCand; j++)
      {
        Bool mostProbableModeIncluded = false;
        Int  mostProbableMode         = uiPreds[j];

        for ( Int i=0; i < numModesForFullRD; i++ )
        {
          mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i]);
        }

        if ( !mostProbableModeIncluded )
        {
          uiRdModeList[numModesForFullRD++] = mostProbableMode;
        }
      }
    }
    UInt       uiBestPUMode[MAX_NUM_CHANNEL_TYPE]  = {0, 0};
    Distortion uiBestPUDist[MAX_NUM_CHANNEL_TYPE]  = {0, 0};
    Double     dBestPUCost                         = MAX_DOUBLE;

    //select luma intra mode
    for( UInt uiLumaModeIdx = 0; uiLumaModeIdx < numModesForFullRD; uiLumaModeIdx++ )  //candidate luma intra mode
    {
      pcCU->setIntraDirSubParts( CHANNEL_TYPE_LUMA,   uiRdModeList[uiLumaModeIdx], uiPartOffset, uiDepth + uiInitTrDepth );
      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, DM_CHROMA_IDX,               uiPartOffset, uiDepth + uiInitTrDepth );  //use DM_CHROMA_IDX for chroma intra mode
#if SCM_U0106_ACT_TU_SIG
      if( pcCU->getPartitionSize(0) == SIZE_NxN )
      {
        pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, DM_CHROMA_IDX, 0, uiDepth );
      }
#endif

      DEBUG_STRING_NEW(sMode)
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      Distortion uiPUDistY = 0;
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0;

#if SCM_U0106_ACT_TU_SIG
      if(eACTRDTestType == ACT_TWO_CLR)
      {
        xRecurIntraCodingQTTUCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, true, eACTRDTestType DEBUG_STRING_PASS_INTO(sMode)  );
      }
      else if(eACTRDTestType == ACT_TRAN_CLR)
      {
        pcCU->setColourTransformSubParts(true, 0, uiDepth);
        xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, true DEBUG_STRING_PASS_INTO(sMode)  );
      }
      else if(eACTRDTestType == ACT_ORG_CLR)
      {
        xRecurIntraCodingQTTUCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, true, eACTRDTestType DEBUG_STRING_PASS_INTO(sMode)  );
      }
      else
      {
        assert(0);
      }
#else
      xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, true DEBUG_STRING_PASS_INTO(sMode)  );
#endif

      if( dPUCost < dBestPUCost)
      {
        dBestPUCost                       = dPUCost;
        uiBestPUDist[CHANNEL_TYPE_LUMA]   = uiPUDistY;
        uiBestPUDist[CHANNEL_TYPE_CHROMA] = uiPUDistC;
        uiBestPUMode[CHANNEL_TYPE_LUMA]   = uiRdModeList[uiLumaModeIdx];
        uiBestPUMode[CHANNEL_TYPE_CHROMA] = DM_CHROMA_IDX;
        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );
        xSetIntraResultChromaQT( pcRecoYuv, tuRecurseWithPU );

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx() + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#if SCM_U0106_ACT_TU_SIG
        ::memcpy( m_puhQTTempACTFlag, pcCU->getColourTransform() + uiPartOffset, uiQPartNum * sizeof(Bool) );
#endif
        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID],                             pcCU->getCbf( compID  )                           + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],               pcCU->getTransformSkip(compID)                    + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID],    pcCU->getCrossComponentPredictionAlpha(compID)    + uiPartOffset, uiQPartNum * sizeof( Char  ) );
        }
      }
    }

#if SCM_U0095_FAST_INTRA_ACT
    if ((!m_pcEncCfg->getTransquantBypassInferTUSplit() || !pcCU->isLosslessCoded(0)) && (uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiPartOffset)) && ((!m_pcEncCfg->getNoTUSplitIntraACTEnabled()) || (m_pcEncCfg->getNoTUSplitIntraACTEnabled() && uiDepth > 1)))
#else
    if((!m_pcEncCfg->getTransquantBypassInferTUSplit() || !pcCU->isLosslessCoded(0)) && (uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiPartOffset)) )
#endif
    {
      Distortion uiPUDistY = 0;
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0;

      pcCU->setIntraDirSubParts( CHANNEL_TYPE_LUMA,   uiBestPUMode[CHANNEL_TYPE_LUMA],   uiPartOffset, uiDepth + uiInitTrDepth );
      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestPUMode[CHANNEL_TYPE_CHROMA], uiPartOffset, uiDepth + uiInitTrDepth );

      DEBUG_STRING_NEW(sMode)
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
#if SCM_U0106_ACT_TU_SIG
      if(eACTRDTestType == ACT_TWO_CLR)
      {
        xRecurIntraCodingQTTUCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, false, eACTRDTestType DEBUG_STRING_PASS_INTO(sMode) );
      }
      else if(eACTRDTestType == ACT_TRAN_CLR)
      {
        pcCU->setColourTransformSubParts(true, 0, uiDepth);
        xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, false DEBUG_STRING_PASS_INTO(sMode) ); 
      }
      else if(eACTRDTestType == ACT_ORG_CLR)
      {
        xRecurIntraCodingQTTUCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, false, eACTRDTestType DEBUG_STRING_PASS_INTO(sMode) );
      }
      else
      {
        assert(0);
      }
#else
      xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, false DEBUG_STRING_PASS_INTO(sMode) );
#endif

      if( dPUCost < dBestPUCost)
      {
        dBestPUCost                       = dPUCost;
        uiBestPUDist[CHANNEL_TYPE_LUMA]   = uiPUDistY;
        uiBestPUDist[CHANNEL_TYPE_CHROMA] = uiPUDistC;
        xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );
        xSetIntraResultChromaQT( pcRecoYuv, tuRecurseWithPU );

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx() + uiPartOffset, uiQPartNum * sizeof( UChar ) );
#if SCM_U0106_ACT_TU_SIG
        ::memcpy( m_puhQTTempACTFlag, pcCU->getColourTransform() + uiPartOffset, uiQPartNum * sizeof(Bool) );
#endif
        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID],                             pcCU->getCbf( compID  )                           + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],               pcCU->getTransformSkip(compID)                    + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID],    pcCU->getCrossComponentPredictionAlpha(compID)    + uiPartOffset, uiQPartNum * sizeof( Char  ) );
        }
      }
    }

    uiOverallDistY += uiBestPUDist[CHANNEL_TYPE_LUMA];
    uiOverallDistC += uiBestPUDist[CHANNEL_TYPE_CHROMA];

    pcCU->setIntraDirSubParts( CHANNEL_TYPE_LUMA,   uiBestPUMode[CHANNEL_TYPE_LUMA],   uiPartOffset, uiDepth + uiInitTrDepth );
    pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestPUMode[CHANNEL_TYPE_CHROMA], uiPartOffset, uiDepth + uiInitTrDepth );
    const UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
    ::memcpy( pcCU->getTransformIdx() + uiPartOffset, m_puhQTTempTrIdx, uiQPartNum * sizeof( UChar ) );
#if SCM_U0106_ACT_TU_SIG
    ::memcpy( pcCU->getColourTransform() + uiPartOffset, m_puhQTTempACTFlag, uiQPartNum * sizeof(Bool) );
#endif
    for (UInt component = 0; component < numberValidComponents; component++)
    {
      const ComponentID compID = ComponentID(component);
      ::memcpy( pcCU->getCbf( compID  )                           + uiPartOffset, m_puhQTTempCbf[compID],                             uiQPartNum * sizeof( UChar ) );
      ::memcpy( pcCU->getTransformSkip( compID  )                 + uiPartOffset, m_puhQTTempTransformSkipFlag[compID ],              uiQPartNum * sizeof( UChar ) );
      ::memcpy( pcCU->getCrossComponentPredictionAlpha(compID)    + uiPartOffset, m_phQTTempCrossComponentPredictionAlpha[compID],    uiQPartNum * sizeof( Char  ) );
    }

    if( !tuRecurseWithPU.IsLastSection() )
    {
      assert( tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA) );

      const UInt numChannelToProcess = getNumberValidComponents(pcCU->getPic()->getChromaFormat());

      for (UInt ch=0; ch < numChannelToProcess; ch++)
      {
        const ComponentID compID     = ComponentID(ch);
        const TComRectangle &puCRect = tuRecurseWithPU.getRect(compID);
        const UInt  uiCompWidth      = puCRect.width;
        const UInt  uiCompHeight     = puCRect.height;

        const UInt  uiZOrder      = pcCU->getZorderIdxInCtu() + uiPartOffset;
        Pel*  piDes               = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
        const UInt  uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride( compID);
        const Pel*  piSrc         = pcRecoYuv->getAddr( compID, uiPartOffset );
        const UInt  uiSrcStride   = pcRecoYuv->getStride( compID);

        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for ( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[uiX] = piSrc[uiX];
          }
        }
      }
    }

  } while ( tuRecurseWithPU.nextSection(tuRecurseCU) );

  if( uiNumPU > 1 )
  {
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;

    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, COMPONENT_Y,  1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }

    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Y  )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;
}

Void
TEncSearch::estIntraPredLumaQTWithModeReuse(TComDataCU* pcCU,
                                            TComYuv*    pcOrgYuv,
                                            TComYuv*    pcPredYuv,
                                            TComYuv*    pcResiYuv,
                                            TComYuv*    pcRecoYuv,
                                            Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                           )
{
  const UInt         uiDepth               = pcCU->getDepth(0);
  const UInt         uiInitTrDepth         = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  const UInt         uiNumPU               = 1<<(2*uiInitTrDepth);
  const UInt         uiQNumParts           = pcCU->getTotalNumPart() >> 2;
  Distortion         uiOverallDistY        = 0;
  Distortion         uiOverallDistC        = 0;
  Pel                resiLumaPU[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

  Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
  for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
  {
    bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
  }

  bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

  //===== set QP and clear Cbf =====
  if ( pcCU->getSlice()->getPPS()->getUseDQP() == true)
  {
    pcCU->setQPSubParts( pcCU->getQP(0), 0, uiDepth );
  }
  else
  {
    pcCU->setQPSubParts( pcCU->getSlice()->getSliceQp(), 0, uiDepth );
  }

  //===== loop over partitions =====
  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

  do
  {
    const UInt uiPartOffset  = tuRecurseWithPU.GetAbsPartIdxTU();
    Distortion uiBestPUDistY = 0;
    Double     dBestPUCost  = 0;

    DEBUG_STRING_NEW(sMode)
    // set context models
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

    xRecurIntraCodingLumaQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiBestPUDistY, true, dBestPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
    xSetIntraResultLumaQT( pcRecoYuv, tuRecurseWithPU );

    if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
    {
      const Int xOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).x0;
      const Int yOffset = tuRecurseWithPU.getRect( COMPONENT_Y ).y0;
      for (UInt storedResidualIndex = 0; storedResidualIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; storedResidualIndex++)
      {
        if (bMaintainResidual[storedResidualIndex])
        {
          xStoreCrossComponentPredictionResult(resiLuma[storedResidualIndex], resiLumaPU[storedResidualIndex], tuRecurseWithPU, xOffset, yOffset, MAX_CU_SIZE, MAX_CU_SIZE );
        }
      }
    }

    uiOverallDistY += uiBestPUDistY;

    //--- set reconstruction for next intra prediction blocks ---
    if( !tuRecurseWithPU.IsLastSection() )
    {
      const UInt numChannelToProcess = 1;

      for (UInt ch=0; ch<numChannelToProcess; ch++)
      {
        const ComponentID compID    = ComponentID(ch);
        const TComRectangle &puRect = tuRecurseWithPU.getRect(compID);
        const UInt  uiCompWidth     = puRect.width;
        const UInt  uiCompHeight    = puRect.height;

        const UInt  uiZOrder      = pcCU->getZorderIdxInCtu() + uiPartOffset;
        Pel*        piDes         = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
        const UInt  uiDesStride   = pcCU->getPic()->getPicYuvRec()->getStride( compID);
        const Pel*  piSrc         = pcRecoYuv->getAddr( compID, uiPartOffset );
        const UInt  uiSrcStride   = pcRecoYuv->getStride( compID);

        for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
        {
          for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
          {
            piDes[ uiX ] = piSrc[ uiX ];
          }
        }
      }
    }
  } while (tuRecurseWithPU.nextSection(tuRecurseCU));


  if( uiNumPU > 1 )
  { // set Cbf for all blocks
    UInt uiCombCbfY = 0;
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfY |= pcCU->getCbf( uiPartIdx, COMPONENT_Y,  1 );
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Y  )[ uiOffs ] |= uiCombCbfY;
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }

  //===== reset context models =====
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  //===== set distortion (rate and r-d costs are determined later) =====
  assert(uiOverallDistC == 0);
  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;
}

Void
TEncSearch::estIntraPredChromaQTWithModeReuse(TComDataCU* pcCU,
                                              TComYuv*    pcOrgYuv,
                                              TComYuv*    pcPredYuv,
                                              TComYuv*    pcResiYuv,
                                              TComYuv*    pcRecoYuv,
                                              Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                             )
{
  const UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;

  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);
  const UInt    uiQNumParts           = tuRecurseWithPU.GetAbsPartIdxNumParts();
  const UInt    uiDepthCU             = tuRecurseWithPU.getCUDepth();
  const UInt    numberValidComponents = pcCU->getPic()->getNumberValidComponents();

  do
  {
    //----- init mode list -----
    if (tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      const UInt  uiPartOffset   = tuRecurseWithPU.GetAbsPartIdxTU();

      assert( pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiPartOffset ) == DM_CHROMA_IDX );  //YCgCo space only test DM_CHROMA_IDX

      //----- restore context models -----
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );

      DEBUG_STRING_NEW(sMode)
      //----- chroma coding -----
      Distortion uiDist = 0;
      xRecurIntraChromaCodingQT( pcOrgYuv, pcPredYuv, pcResiYuv, resiLuma, uiDist, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );

      xSetIntraResultChromaQT( pcRecoYuv, tuRecurseWithPU );

      if( ! tuRecurseWithPU.IsLastSection() )
      {
        for (UInt ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
        {
          const ComponentID compID    = ComponentID(ch);
          const TComRectangle &tuRect = tuRecurseWithPU.getRect(compID);
          const UInt  uiCompWidth     = tuRect.width;
          const UInt  uiCompHeight    = tuRect.height;
          const UInt  uiZOrder        = pcCU->getZorderIdxInCtu() + tuRecurseWithPU.GetAbsPartIdxTU();
          Pel*  piDes                 = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
          const UInt  uiDesStride     = pcCU->getPic()->getPicYuvRec()->getStride( compID);
          const Pel*  piSrc           = pcRecoYuv->getAddr( compID, uiPartOffset );
          const UInt  uiSrcStride     = pcRecoYuv->getStride( compID);

          for( UInt uiY = 0; uiY < uiCompHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
          {
            for( UInt uiX = 0; uiX < uiCompWidth; uiX++ )
            {
              piDes[ uiX ] = piSrc[ uiX ];
            }
          }
        }
      }

      pcCU->getTotalDistortion() += uiDist;
    }

  } while (tuRecurseWithPU.nextSection(tuRecurseCU));

  //----- restore context models -----

  if( uiInitTrDepth != 0 )
  { // set Cbf for all blocks
    UInt uiCombCbfU = 0;
    UInt uiCombCbfV = 0;
    UInt uiPartIdx  = 0;
    for( UInt uiPart = 0; uiPart < 4; uiPart++, uiPartIdx += uiQNumParts )
    {
      uiCombCbfU |= pcCU->getCbf( uiPartIdx, COMPONENT_Cb, 1 );
      uiCombCbfV |= pcCU->getCbf( uiPartIdx, COMPONENT_Cr, 1 );
    }
    for( UInt uiOffs = 0; uiOffs < 4 * uiQNumParts; uiOffs++ )
    {
      pcCU->getCbf( COMPONENT_Cb )[ uiOffs ] |= uiCombCbfU;
      pcCU->getCbf( COMPONENT_Cr )[ uiOffs ] |= uiCombCbfV;
    }
  }
  
  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepthCU][CI_CURR_BEST] );
}


/** Function for encoding and reconstructing luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param pOrg pointer to original sample arrays
 * \param pPCM pointer to PCM code arrays
 * \param pPred pointer to prediction signal arrays
 * \param pResi pointer to residual signal arrays
 * \param pReco pointer to reconstructed sample arrays
 * \param uiStride stride of the original/prediction/residual sample arrays
 * \param uiWidth block width
 * \param uiHeight block height
 * \param compID texture component type
 */
Void TEncSearch::xEncPCM (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* pOrg, Pel* pPCM, Pel* pPred, Pel* pResi, Pel* pReco, UInt uiStride, UInt uiWidth, UInt uiHeight, const ComponentID compID )
{
  const UInt uiReconStride   = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  const UInt uiPCMBitDepth   = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
  const Int  channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
  Pel* pRecoPic = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiAbsPartIdx);

  const Int pcmShiftRight=(channelBitDepth - Int(uiPCMBitDepth));

  assert(pcmShiftRight >= 0);

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      // Reset pred and residual
      pPred[uiX] = 0;
      pResi[uiX] = 0;
      // Encode
      pPCM[uiX] = (pOrg[uiX]>>pcmShiftRight);
      // Reconstruction
      pReco   [uiX] = (pPCM[uiX]<<(pcmShiftRight));
      pRecoPic[uiX] = pReco[uiX];
    }
    pPred += uiStride;
    pResi += uiStride;
    pPCM += uiWidth;
    pOrg += uiStride;
    pReco += uiStride;
    pRecoPic += uiReconStride;
  }
}

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
UInt TEncSearch::PLTSearch(TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv *& rpcResiBestYuv, TComYuv*& rpcRecoYuv, Bool forcePLTPrediction,
  UInt uiIterNumber, UInt *pltSizeCurrIter)
#else
Void TEncSearch::PLTSearch(TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv *& rpcResiBestYuv, TComYuv*& rpcRecoYuv, Bool forcePLTPrediction)
#endif
{
  UInt  uiDepth      = pcCU->getDepth(0);
  Distortion  uiDistortion = 0;
  Pel *paOrig[3], *paPalette[3];
  TCoeff *pRun;
  UChar *paSPoint[3];
  Pel *pPixelValue[3];
  UInt uiScaleX = pcCU->getPic()->getComponentScaleX(COMPONENT_Cb);
  UInt uiScaleY = pcCU->getPic()->getComponentScaleY(COMPONENT_Cb);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt testMode=0;
  UInt uiPLTIdx = 0;
#endif
  for (UInt ch = 0; ch < 3; ch++)
  {
    paOrig[ch] = pcOrgYuv->getAddr((ComponentID)ch, 0);
    paPalette[ch] = pcCU->getPLT(ch, 0);
  }

  pRun = pcCU->getRun(COMPONENT_Y);
  paSPoint[0] = pcCU->getSPoint(COMPONENT_Y);
  UChar* pEscapeFlag = pcCU->getEscapeFlag(COMPONENT_Y);
#if !SCM_U0096_PLT_ENCODER_IMPROVEMENT
  Int iPLTErrLimit = g_uhPLTQuant[Int(pcCU->getQP(0))];
  setPLTErrLimit(iPLTErrLimit);
#endif
 
  UInt uiPLTSize = 1;
#if !SCM_U0096_PLT_ENCODER_IMPROVEMENT
  if( pcCU->getCUTransquantBypass(0) )
  {
    derivePLTLossless(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, forcePLTPrediction);
  }
  else if( forcePLTPrediction )
  {
    derivePLTLossyForcePrediction(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
  }
  else
  {
    derivePLTLossy(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
  }
#else
  if (uiIterNumber < MAX_PLT_ITER)
  {
    if( pcCU->getCUTransquantBypass(0) )
    {
      derivePLTLossless(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, forcePLTPrediction);
    }
    else if( forcePLTPrediction )
    {
      derivePLTLossyForcePrediction(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
    }
    else
    {
      derivePLTLossy(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
    }
  }
  else
  {
    uiPLTSize = m_prevPltSize[uiIterNumber - MAX_PLT_ITER];
    for (UInt ch = 0; ch < 3; ch++)
    {
      for ( uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
      {
        paPalette[ch][uiPLTIdx] = m_prevPlt[uiIterNumber - MAX_PLT_ITER][ch][uiPLTIdx];
      }
    }
    derivePLTLossyIterative(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
  }
  *pltSizeCurrIter=uiPLTSize;
#endif

  pcCU->setPLTSizeSubParts(0, uiPLTSize, 0, pcCU->getDepth(0));
  pcCU->setPLTSizeSubParts(1, uiPLTSize, 0, pcCU->getDepth(0));
  pcCU->setPLTSizeSubParts(2, uiPLTSize, 0, pcCU->getDepth(0));
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  reorderPLT(pcCU, paPalette, 3);

  if (uiIterNumber==0)
  {
    m_forcePltSize=uiPLTSize;
    for (UInt ch = 0; ch < 3; ch++)
    {
      for ( uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
      {
        m_forcePlt[ch][uiPLTIdx]=paPalette[ch][uiPLTIdx];
      }
    }
  }
  else if (uiIterNumber==1)
  {
    UInt samePLT=0;
    if (uiPLTSize==m_forcePltSize)
    {
      samePLT=1;
      for (UInt ch = 0; ch < 3; ch++)
      {
        for ( uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
        {
          if (paPalette[ch][uiPLTIdx]!=m_forcePlt[ch][uiPLTIdx])
          {
            samePLT=0;
          }
        }
      }
    }
    if (samePLT)
    {
      pcCU->getTotalCost()=MAX_DOUBLE;
      return(0);
    }
  }
#else
  reorderPLT(pcCU, paPalette, 3);
#endif

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt calcErroBits = uiIterNumber < MAX_PLT_ITER ? 1 : 0;
  preCalcPLTIndexRD(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost, calcErroBits);
#else
  preCalcPLTIndex(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), uiPLTSize);
#endif

#if !SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt uiPLTIdx = 0;
#endif
  for (UInt ch = 0; ch < 3; ch++)
  {
    for ( uiPLTIdx = 0; uiPLTIdx < pcCU->getSlice()->getSPS()->getSpsScreenExtension().getPLTMaxSize(); uiPLTIdx++)
    {
      pcCU->setPLTSubParts(ch,  paPalette[ch][uiPLTIdx], uiPLTIdx, 0, pcCU->getDepth(0));
    }
    pPixelValue[ch] = pcCU->getLevel(ComponentID (ch));
  }

  UInt uiWidth  = pcCU->getWidth(0);
  UInt uiHeight = pcCU->getHeight(0);
  UInt uiStride = rpcPredYuv->getStride(ComponentID(0));

  memcpy(m_paOriginalLevel, m_cIndexBlock, sizeof(Pel) * (uiWidth * uiHeight));

  UInt uiBits      = MAX_UINT;
  m_bBestScanRotationMode = 0;

  deriveRunAndCalcBits(pcCU, pcOrgYuv, rpcRecoYuv, uiBits, true,  PLT_SCAN_HORTRAV);
  if ( (pcCU->getPLTSize( COMPONENT_Y, 0 ) + pcCU->getPLTEscape( COMPONENT_Y, 0 )) > 1 )
  {
    deriveRunAndCalcBits( pcCU, pcOrgYuv, rpcRecoYuv, uiBits, false, PLT_SCAN_VERTRAV );
  }

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt errorOrig, errorNew;
  if (uiPLTSize > 2)
  {
    UInt uiTotalPixel = uiWidth * uiHeight, uiTotalPixelC = (uiWidth>>uiScaleX) * (uiHeight>>uiScaleY);

    for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      if ( ch == 0 )
      {
        memcpy(m_paLevelStoreRD[ch], m_paBestLevel[ch], sizeof(Pel) * uiTotalPixel);
      }
      else
      {
        memcpy(m_paLevelStoreRD[ch], m_paBestLevel[ch], sizeof(Pel) * uiTotalPixelC);
      }
    }
    memcpy(m_paSPointStoreRD, m_paBestSPoint, sizeof(UChar) * uiTotalPixel);
    memcpy(m_paRunStoreRD, m_paBestRun, sizeof(TCoeff) * uiTotalPixel);
    memcpy(m_paEscapeFlagStoreRD, m_paBestEscapeFlag, sizeof(UChar) * uiTotalPixel );

    memcpy(m_pltInfoStoreRD, m_pltInfoBest, sizeof(m_pltInfo));

    m_SPointRD     = m_paBestSPoint;
    m_EscapeFlagRD = m_paBestEscapeFlag;
    m_RunRD        = m_paBestRun;
    m_LevelRD[0]   = m_paBestLevel[0];
    m_LevelRD[1]   = m_paBestLevel[1];
    m_LevelRD[2]   = m_paBestLevel[2];

    preCalcRDMerge(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost, &errorOrig, &errorNew, calcErroBits);
  }
#endif

  pcCU->setPLTScanRotationModeFlagSubParts( m_bBestScanRotationMode, 0, uiDepth );
  for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    if ( ch == 0 )
    {
      memcpy( pPixelValue[ch], m_paBestLevel[ch], sizeof( Pel ) * uiWidth * uiHeight );
    }
    else
    {
      memcpy( pPixelValue[ch], m_paBestLevel[ch], sizeof( Pel ) * (uiWidth>>uiScaleX) * (uiHeight>>uiScaleY) );
    }
  }
  memcpy(paSPoint[0], m_paBestSPoint, sizeof(UChar) * uiWidth * uiHeight);
  memcpy(pRun,        m_paBestRun,    sizeof(TCoeff) * uiWidth * uiHeight);
  memcpy(pEscapeFlag, m_paBestEscapeFlag,  sizeof(UChar) * uiWidth * uiHeight);

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  if (uiPLTSize > 2 )
  {
      UInt uiBitsRD;

      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
      m_pcEntropyCoder->resetBits();
      xEncIntraHeader(pcCU, uiDepth, 0, true, false);
      uiBitsRD = m_pcEntropyCoder->getNumberOfWrittenBits();

      Double rdOrig, rdNew;

      rdOrig = errorOrig + m_pcRdCost->getLambda()*(Double)uiBits;
      rdNew = errorNew + m_pcRdCost->getLambda()*(Double)uiBitsRD;

      if (rdNew <= rdOrig)
      {
        uiBits = uiBitsRD;
        m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
      }
      else
      {
        for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
        {
          if (ch == 0)
          {
            memcpy(pPixelValue[ch], m_paLevelStoreRD[ch], sizeof(Pel)* uiWidth * uiHeight);
          }
          else
          {
            memcpy(pPixelValue[ch], m_paLevelStoreRD[ch], sizeof(Pel)* (uiWidth >> uiScaleX) * (uiHeight >> uiScaleY));
          }
        }
        memcpy(paSPoint[0], m_paSPointStoreRD, sizeof(UChar)* uiWidth * uiHeight);
        memcpy(pRun, m_paRunStoreRD, sizeof(TCoeff)* uiWidth * uiHeight);
        memcpy(pEscapeFlag, m_paEscapeFlagStoreRD, sizeof(UChar)* uiWidth * uiHeight);
        memcpy(m_pltInfoBest, m_pltInfoStoreRD, sizeof(m_pltInfo));
      }


    if (uiIterNumber<MAX_PLT_ITER) //after cabac loading fix
    {
      testMode = preCalcRD(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost, uiIterNumber);
    }
  }
#endif

  for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID  = ComponentID(ch);
    uiWidth  = pcCU->getWidth(0)  >> pcCU->getPic()->getComponentScaleX(compID);
    uiHeight = pcCU->getHeight(0) >> pcCU->getPic()->getComponentScaleY(compID);
    uiStride = rpcPredYuv->getStride(compID);

    Pel *pOrig    = pcOrgYuv->getAddr  (compID, 0);
    Pel *pResi    = rpcResiYuv->getAddr(compID, 0);
    Pel *pPred    = rpcPredYuv->getAddr(compID, 0);
    Pel *pLevel   = pcCU->getLevel  (COMPONENT_Y);
    Pel *pPalette = pcCU->getPLT   (compID,0);
    Pel *pReco    = rpcRecoYuv->getAddr(compID, 0);
    Pel *pRecoPic = pcCU->getPic()->getPicYuvRec()->getAddr(ComponentID(ch), pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu());
    const UInt uiReconStride = pcCU->getPic()->getPicYuvRec()->getStride(ComponentID(ch));
    if(!m_bBestScanRotationMode)
    {
      UInt uiIdx = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          uiIdx = (uiY << pcCU->getPic()->getComponentScaleY(compID)) * (uiWidth << pcCU->getPic()->getComponentScaleX(compID))
                  +(uiX << pcCU->getPic()->getComponentScaleX(compID));
          if(!pEscapeFlag[uiIdx])
          {
            pPred[uiX] = pPalette[pLevel[uiIdx]];
            pReco[uiX] = pPred[uiX];
          }
          pResi[uiX] = pOrig[uiX] - pPred[uiX];
          pRecoPic[uiX] = pReco[uiX];
        }

        pPred += uiStride;
        pResi += uiStride;
        pOrig += uiStride;
        pReco += uiStride;
        pRecoPic += uiReconStride;
      }
    }
    else
    {
      UInt uiIdx = 0;
      for( UInt uiY = 0; uiY < uiWidth; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiHeight; uiX++ )
        {
          uiIdx = (uiY << pcCU->getPic()->getComponentScaleX(compID)) * (uiHeight << pcCU->getPic()->getComponentScaleY(compID))
                  + (uiX << pcCU->getPic()->getComponentScaleY(compID));
          UInt uiPxlPos = uiX*uiStride+uiY;
          if( !pEscapeFlag[uiIdx] )
          {
            pPred[uiPxlPos] = pPalette[pLevel[uiIdx]];
            pReco[uiPxlPos] = pPred[uiPxlPos];
          }
          pResi[uiPxlPos] = pOrig[uiPxlPos] - pPred[uiPxlPos];
          pRecoPic[uiX*uiReconStride+uiY] = pReco[uiPxlPos];
        }
      }
    }

  }

  for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID  = ComponentID(ch);
    const ChannelType chType = toChannelType(compID);
    uiWidth  = pcCU->getWidth(0)  >> pcCU->getPic()->getComponentScaleX(compID);
    uiHeight = pcCU->getHeight(0) >> pcCU->getPic()->getComponentScaleY(compID);
    uiStride = rpcPredYuv->getStride(compID);

    Pel *pOrig = pcOrgYuv->getAddr(compID, 0);    
    Pel *pReco = rpcRecoYuv->getAddr(compID, 0);
    uiDistortion += m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepth(chType), pReco, uiStride, pOrig, uiStride, uiWidth, uiHeight, compID );
  }

  Double dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;
  pcCU->copyToPic(uiDepth);

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  return(testMode);
#endif
}

Void TEncSearch::deriveRunAndCalcBits(TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcRecoYuv, UInt& uiMinBits, Bool bReset, PLTScanMode pltScanMode)
{
  UInt uiDepth = pcCU->getDepth(0);
  Pel *paOrig[3], *paPalette[3], *paLevel[3];
  TCoeff *pRun;
  UChar *paSPoint[3];
  Pel *pPixelValue[3];
  Pel * pRecoValue[3];

  const UInt uiWidth  = pcCU->getWidth(0);
  const UInt uiHeight = pcCU->getHeight(0);
  const UInt uiTotalPixel = uiWidth * uiHeight;
  UInt uiScaleX = pcCU->getPic()->getComponentScaleX(COMPONENT_Cb);
  UInt uiScaleY = pcCU->getPic()->getComponentScaleY(COMPONENT_Cb);
  const UInt uiTotalPixelC = uiTotalPixel >> uiScaleX >> uiScaleY;
  UInt uiPLTSize = pcCU->getPLTSize(0, 0);

  paLevel[0] = pcCU->getLevel(COMPONENT_Y);
  pRun = pcCU->getRun(COMPONENT_Y);
  paSPoint[0] = pcCU->getSPoint(COMPONENT_Y);
  UChar *pEscapeFlag = pcCU->getEscapeFlag(COMPONENT_Y);

  for (UInt ch = 0; ch < 3; ch++)
  {
    paOrig[ch] = pcOrgYuv->getAddr((ComponentID)ch, 0);
    paPalette[ch] = pcCU->getPLT(ch, 0);
    pPixelValue[ch] = pcCU->getLevel(ComponentID (ch));
    pRecoValue[ch] = pcRecoYuv->getAddr(ComponentID (ch), 0);
  }
  pcCU->setPLTScanRotationModeFlagSubParts(pltScanMode, 0, uiDepth );
  if (pltScanMode == PLT_SCAN_VERTRAV)
  {    
    rotationScan(m_cIndexBlock, uiWidth, uiHeight, false);  
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
    rotationScan(m_cPosBlock, uiWidth, uiHeight, false);
#endif 
  }

  m_puiScanOrder = g_scanOrder[SCAN_UNGROUPED][SCAN_TRAV][g_aucConvertToBit[uiWidth]+2][g_aucConvertToBit[uiHeight]+2];

  xDeriveRun(pcCU, paOrig, paPalette,  paLevel[0], paSPoint[0], pPixelValue, pRecoValue, pRun, uiWidth, uiHeight, pcOrgYuv->getStride(ComponentID(0)), uiPLTSize);

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiDepth, 0, true, false);
  UInt uiTempBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  if (uiMinBits > uiTempBits)
  {
    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
    m_bBestScanRotationMode = pltScanMode;
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
     memcpy(m_pltInfoBest, m_pltInfo, sizeof(m_pltInfo));
     m_pltNoElementsBest = m_pltNoElements;

     memcpy(m_cPosBlockRD, m_cPosBlock, sizeof(m_cPosBlock));
     memcpy(m_cIndexBlockRD, m_cIndexBlock, sizeof(m_cIndexBlock));
#endif
    for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      if ( ch == 0 )
      {
        memcpy(m_paBestLevel[ch], pPixelValue[ch], sizeof(Pel) * uiTotalPixel);
      }
      else
      {
        memcpy(m_paBestLevel[ch], pPixelValue[ch], sizeof(Pel) * uiTotalPixelC);
      }
    }
    memcpy(m_paBestSPoint, paSPoint[0], sizeof(UChar) * uiTotalPixel);
    memcpy(m_paBestRun, pRun, sizeof(TCoeff) * uiTotalPixel);
    memcpy(m_paBestEscapeFlag, pEscapeFlag, sizeof(UChar) * uiTotalPixel );
    uiMinBits = uiTempBits;
  }
  if (bReset)
  {
    memcpy(m_cIndexBlock, m_paOriginalLevel, sizeof(Pel) * uiTotalPixel);
    memset(paSPoint[0], 0, sizeof(UChar) * uiTotalPixel);
    memset(pEscapeFlag, 0, sizeof(UChar) * uiTotalPixel);
  }
}

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
UInt TEncSearch::preCalcRD(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize, TComRdCost *pcCost, UInt uiIterNumber)
{
  UInt uiTotal = uiHeight * uiWidth;
  Bool bEscape = 0;
  UInt uiPLTSizeTemp=uiPLTSize, uiPLTSizeBest=uiPLTSize;
  UInt pltIdxRemove1=0, pltIdxRemove2=0, pltIdxReplacement1 = MAX_PLT_SIZE-1, 
       pltIdxMapping1[MAX_PLT_SIZE], pltIdxMapping2[MAX_PLT_SIZE], removedIndices[MAX_PLT_SIZE], removedIndicesBest[MAX_PLT_SIZE];
  
  UInt64 error = 0;
  Double rdCostNew, rdCostOrig, rdCostDiff = MAX_DOUBLE;
  Int64  iBits=0, runBits=0;

  // Initial RD cost
  memset(removedIndices, 0, sizeof(removedIndices));

  for (UInt uiPLTIdx = 0; uiPLTIdx < MAX_PLT_SIZE; uiPLTIdx++)
  {
    pltIdxMapping1[uiPLTIdx] = uiPLTIdx;
    pltIdxMapping2[uiPLTIdx] = uiPLTIdx;
  }

  error = 0;
  for (UInt uiIdx=0; uiIdx < uiTotal; uiIdx++)
  {
    if (m_cIndexBlockRD[uiIdx] < 0)
    {
      m_cIndexBlockRD[uiIdx] = (MAX_PLT_SIZE-1);
    }
    UInt uiCurPltIdx = pltIdxMapping1[m_cIndexBlockRD[uiIdx]];
    error += m_indError[m_cPosBlockRD[uiIdx]][uiCurPltIdx];
  }


  for (UInt noElement = 0; noElement < m_pltNoElementsBest; noElement++)
  {
    if (m_pltInfoBest[noElement].pltMode == PLT_RUN_LEFT && m_pltInfoBest[noElement].index < (MAX_PLT_SIZE-1))
    {
      iBits += m_pltInfoBest[noElement].bitsInd;
    }
  }

  rdCostOrig = pcCost->getLambda()*(Double)(iBits>>15) + error;

  // Initial error calculation
  Int64 errorDiffPltIndArray[MAX_PLT_SIZE][MAX_PLT_SIZE];
  Int64 bestIndToRemoveArray[MAX_PLT_SIZE][2];

  for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
  {
    memset(errorDiffPltIndArray[uiPLTIdx], 0, MAX_PLT_SIZE*sizeof(UInt64));
  }

  //Calculate all the SSE if index A is mapped to index B
  for (UInt uiIdx=0; uiIdx < uiTotal; uiIdx++)
  {
    UInt uiOrgIdx = m_cIndexBlockRD[uiIdx]; //Escape already converted
    UInt* indError = m_indError[m_cPosBlockRD[uiIdx]];
    Int64* errDiffArray = errorDiffPltIndArray[uiOrgIdx];
    
    for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
    {
      errDiffArray[uiPLTIdx] += indError[uiPLTIdx];
    }
    errDiffArray[MAX_PLT_SIZE - 1] += indError[MAX_PLT_SIZE - 1];
  }

  //select the best mapped index for each index
  for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
  {
    Int64* bestIdxRemove = bestIndToRemoveArray[uiPLTIdx];
    Int64* errDiffArray = errorDiffPltIndArray[uiPLTIdx];

    bestIdxRemove[0] = MAX_PLT_SIZE-1;
    bestIdxRemove[1] = errDiffArray[MAX_PLT_SIZE-1] - errDiffArray[uiPLTIdx];

    for (UInt uiTmpIdx = 0; uiTmpIdx < uiPLTSize; uiTmpIdx++)
    {
      Int64 curError = errDiffArray[uiTmpIdx] - errDiffArray[uiPLTIdx];

      if (uiPLTIdx != uiTmpIdx && (curError < bestIdxRemove[1] ||
        (curError == bestIdxRemove[1] && uiTmpIdx < bestIdxRemove[0])))
      {
        bestIdxRemove[1] = curError;
        bestIdxRemove[0] = uiTmpIdx;
      }
    }
  }

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
  m_pcRDGoOnSbacCoder->saveRestorePltCtx(1);

  //Remove the unused index after the mapping
  while (uiPLTSizeTemp > 2)
  {
    Int64 minError = MAX_INT64;
    UInt uiPLTCnt = 0;
    while (uiPLTCnt < uiPLTSize)
    {
      if (removedIndices[uiPLTCnt] == 0)
      {
        if (bestIndToRemoveArray[uiPLTCnt][1] < minError)
        {
          minError           = bestIndToRemoveArray[uiPLTCnt][1];
          pltIdxRemove1      = uiPLTCnt;
          pltIdxReplacement1 = UInt(bestIndToRemoveArray[uiPLTCnt][0]);
        }
      }
      uiPLTCnt++;
    }
      
    // Merge removed index pltIdxRemove1 with pltIdxReplacement1
    if (pltIdxReplacement1 != (MAX_PLT_SIZE-1))
    {
      for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
      {
        if (removedIndices[uiPLTIdx] == 0)
        {
          errorDiffPltIndArray[pltIdxReplacement1][uiPLTIdx] += errorDiffPltIndArray[pltIdxRemove1][uiPLTIdx];
        }
      }
      errorDiffPltIndArray[pltIdxReplacement1][MAX_PLT_SIZE-1] += errorDiffPltIndArray[pltIdxRemove1][MAX_PLT_SIZE-1];
    }
    removedIndices[pltIdxRemove1]=1;

    // Find another min index for pltIdxReplacement1 and indices for which pltIdxRemove1 was chosen
    for (UInt uiPLTIdxOrg = 0; uiPLTIdxOrg < uiPLTSize; uiPLTIdxOrg++)
    {
      if ((removedIndices[uiPLTIdxOrg] == 0 && bestIndToRemoveArray[uiPLTIdxOrg][0] == pltIdxRemove1) || uiPLTIdxOrg == pltIdxReplacement1)
      {
        Int64* bestIdxRemove = bestIndToRemoveArray[uiPLTIdxOrg];
        Int64* errDiffArray = errorDiffPltIndArray[uiPLTIdxOrg];

        bestIdxRemove[0] = MAX_PLT_SIZE-1;
        bestIdxRemove[1] = errDiffArray[MAX_PLT_SIZE - 1] - errDiffArray[uiPLTIdxOrg];

        for (UInt uiPLTIdxTmp = 0; uiPLTIdxTmp < uiPLTSize; uiPLTIdxTmp++)
        {
          if (removedIndices[uiPLTIdxTmp] == 0)
          {
            Int64 curError = errDiffArray[uiPLTIdxTmp] - errDiffArray[uiPLTIdxOrg];

            if (uiPLTIdxTmp != uiPLTIdxOrg && (curError < bestIdxRemove[1] ||
              (curError == bestIdxRemove[1] && uiPLTIdxTmp<bestIdxRemove[0])))
            {
              bestIndToRemoveArray[uiPLTIdxOrg][1] = curError;
              bestIndToRemoveArray[uiPLTIdxOrg][0] = uiPLTIdxTmp;
            }
          }
        }

      }

    }

    uiPLTSizeTemp--;

    // Remapping
    for (UInt uiPLTIdx = 0; uiPLTIdx<uiPLTSize; uiPLTIdx++)
    {
      if (uiPLTIdx == pltIdxRemove1)
      {
        removedIndices[uiPLTIdx] = 1;
      }
      if (pltIdxMapping1[uiPLTIdx] == pltIdxRemove1)
      {
        pltIdxMapping1[uiPLTIdx] = pltIdxReplacement1;
      }
    }

    pltIdxRemove2 = pltIdxMapping2[pltIdxRemove1];
    UInt pltIdxReplacement2 = pltIdxMapping2[pltIdxReplacement1];

    for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
    {
      if (pltIdxMapping2[uiPLTIdx] == pltIdxRemove2)
      {
        pltIdxMapping2[uiPLTIdx] = pltIdxReplacement2;
      }
      if (pltIdxMapping2[uiPLTIdx] > pltIdxRemove2 && pltIdxMapping2[uiPLTIdx] < (MAX_PLT_SIZE - 1))
      {
        pltIdxMapping2[uiPLTIdx]--;
      }
    }

    // 
    
    if (pltIdxReplacement1 == (MAX_PLT_SIZE-1))
    {
      bEscape = 1;
    }
    UInt uiIndexMaxSize = uiPLTSizeTemp;
    if (pcCU->getPLTEscape(COMPONENT_Y, 0) || bEscape == 1)
    {
      uiIndexMaxSize++;
    }

    // New RD cost
    error=0;
    for (UInt uiIdx = 0; uiIdx < uiTotal; uiIdx++)
    {
      UInt uiCurPLTIdx = pltIdxMapping1[m_cIndexBlockRD[uiIdx]];
      error += m_indError[m_cPosBlockRD[uiIdx]][uiCurPLTIdx];
    }

    UInt currIndex, nextIndex, noSameIndices, predIndex, run=0;

    iBits = 0; 
    UInt noElement = 0;
    while (noElement < m_pltNoElementsBest)
    {
      noSameIndices = 0;

      currIndex = pltIdxMapping2[m_pltInfoBest[noElement].index];
      run = m_pltInfoBest[noElement].run;
      runBits = m_pltInfoBest[noElement].bitsRun;

      if (m_pltInfoBest[noElement].pltMode == PLT_RUN_LEFT && currIndex < (MAX_PLT_SIZE-1))
      {
        UInt uiIdx=1; 
        while((noElement + uiIdx) < m_pltNoElementsBest)
        {
          nextIndex = pltIdxMapping2[m_pltInfoBest[noElement+1].index];
          if (m_pltInfoBest[noElement+uiIdx].pltMode == PLT_RUN_LEFT && nextIndex < (MAX_PLT_SIZE-1) && nextIndex == currIndex)
          {
            run     += (m_pltInfoBest[noElement+uiIdx].run+1);
            runBits += m_pltInfoBest[noElement+uiIdx].bitsRun;
            uiIdx++;
          }
          else
          {
            break;
          }
        }
        noSameIndices = uiIdx-1;

        if (noElement > 0)
        {
          predIndex = pltIdxMapping2[m_pltInfoBest[noElement].indexPred];
          if (currIndex >= predIndex && currIndex > 0)
          {
            currIndex--;
          }
          iBits+=m_truncBinBits[currIndex][uiIndexMaxSize-1]<<15;
        }
        else
        {
          iBits+=m_truncBinBits[currIndex][uiIndexMaxSize]<<15;
        }
        

        if (noSameIndices>0)
        {
          m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
          m_pcRDGoOnSbacCoder->resetBits();
          UInt64 initialBits = m_pcRDGoOnSbacCoder->getNumPartialBits();
          m_pcRDGoOnSbacCoder->encodeRun(run, PLT_RUN_LEFT, currIndex, uiTotal - m_pltInfoBest[noElement].position - 1);
          iBits += (m_pcRDGoOnSbacCoder->getNumPartialBits() - runBits - initialBits);
        }

      }
      noElement += (1 + noSameIndices);
    }

    rdCostNew = pcCost->getLambda()*(Double)(iBits>>15)+error;
    
    if ((rdCostNew - rdCostOrig) < rdCostDiff)
    {
      rdCostDiff = (rdCostNew-rdCostOrig);
      uiPLTSizeBest = uiPLTSizeTemp;
      memcpy(removedIndicesBest, removedIndices, sizeof(removedIndices));
    }
  }

  UInt testReducedInd=1;


  m_prevPltSize[uiIterNumber]=uiPLTSizeBest;
 
  UInt uiPLTCnt=0;
  for (UInt uiPLTIdx = 0; uiPLTIdx < uiPLTSize; uiPLTIdx++)
  {
    if (removedIndicesBest[uiPLTIdx] == 0)
    {
      for (UInt ch = 0; ch < 3; ch++)
      {
        m_prevPlt[uiIterNumber][ch][uiPLTCnt] = Palette[ch][uiPLTIdx];
      }
      uiPLTCnt++;
    }
  }

  return(testReducedInd);
}

UInt TEncSearch::calcPltIndexPredAndBits(Int iMaxSymbol, UInt uiIdxStart, UInt uiWidth, UInt *predIndex, UInt *currIndex)
{
  UInt uiTraIdx, indexBits;

  *predIndex=iMaxSymbol - 1;
  if(uiIdxStart)
  {
    uiTraIdx=m_puiScanOrder[uiIdxStart];
    UInt uiTraIdxLeft = m_puiScanOrder[uiIdxStart - 1];
    if (m_SPointRD[uiTraIdxLeft] == PLT_RUN_LEFT)  ///< copy left
    {
      *predIndex = m_cIndexBlockRD[uiTraIdxLeft];
      if(m_cIndexBlockRD[uiTraIdxLeft]==(MAX_PLT_SIZE-1))
      {
        *predIndex = iMaxSymbol - 1;
      }
    }
    else
    {
      *predIndex = m_cIndexBlockRD[uiTraIdx - uiWidth];
      if(m_cIndexBlockRD[uiTraIdx - uiWidth]==(MAX_PLT_SIZE-1))
      {
        *predIndex = iMaxSymbol - 1;
      }
    }
    iMaxSymbol--;
  }

  if ((*currIndex)>(*predIndex))
  {
    (*currIndex)--;
  }
  indexBits=m_truncBinBits[*currIndex][iMaxSymbol];

  return(indexBits);
}

UInt TEncSearch::calcPLTStartCopy(UInt positionInit, UInt positionCurrSegment, UInt uiWidth)
{
  UInt positionStart;

  positionStart=positionInit;
  while (positionStart>(positionCurrSegment+1) && positionStart>uiWidth)
  {

    UInt uiTraIdx = m_puiScanOrder[positionStart-1]; 
    if (m_cIndexBlockRD[uiTraIdx]==m_cIndexBlockRD[uiTraIdx - uiWidth])
    {
      positionStart--;
    }
    else
    {
      break;
    }
  }

  return(positionStart);
}

UInt TEncSearch::calcPltErrorCopy(UInt uiIdxStart, UInt run, UInt uiWidth, UInt *merge)
{
  UInt error=0;
  UInt uiIdx, uiTraIdx, uiPLTIdx;
  *merge=1;

  for (uiIdx=uiIdxStart; uiIdx<=(uiIdxStart+run); uiIdx++)
  {
    uiTraIdx = m_puiScanOrder[uiIdx]; 

    uiPLTIdx=m_cIndexBlockRD[uiTraIdx - uiWidth];
    if (uiPLTIdx==(MAX_PLT_SIZE-1))
    {
      *merge=0;
      break;
    }
    error+=m_indError[m_cPosBlockRD[uiTraIdx]][uiPLTIdx];
  }

  return(error);
}

UInt64 TEncSearch::calcPltErrorLevel(Int idxStart, UInt run, UInt uiPLTIdx)
{
  UInt64 error = 0;

  for (Int idx = idxStart + run; idx >= idxStart; idx--)
  {
    error += m_indError[m_cPosBlockRD[m_puiScanOrder[idx]]][uiPLTIdx];
  }

  return(error);
}

Void TEncSearch::modifyPltSegment(UInt uiWidth, UInt uiIdxStart, UInt pltMode, UInt pltIdx, UInt run)
{
  for (UInt uiIdx=uiIdxStart; uiIdx<=(uiIdxStart+run); uiIdx++){
    UInt uiTraIdx = m_puiScanOrder[uiIdx]; 

    m_SPointRD[uiTraIdx]=pltMode;

    if (pltMode==PLT_RUN_LEFT)
    {
      m_cIndexBlockRD[uiTraIdx]=pltIdx;
    }
    else
    {
      m_cIndexBlockRD[uiTraIdx]=m_cIndexBlockRD[uiTraIdx - uiWidth];
    }
  }
}

UInt TEncSearch::findPltSegment(pltInfoStruct *pltElement, TComDataCU* pcCU, UInt uiIdxStart, UInt uiIndexMaxSize, UInt uiWidth, UInt uiTotal, UInt copyPixels[],
  Int restrictLevelRun, UInt calcErrBits)
{
  UInt uiTraIdxStart=m_puiScanOrder[uiIdxStart], uiIdx, uiTraIdx, pltMode, predIndex=0, run, currIndex=0;
  UInt64 indexBits=0, runBits, sPointBits;
  Int iMaxSymbol; 
  if (m_SPointRD[uiTraIdxStart]==PLT_RUN_LEFT)
  {
    pltMode=PLT_RUN_LEFT;
    iMaxSymbol=uiIndexMaxSize;

    currIndex=m_cIndexBlockRD[uiTraIdxStart];

    UInt startIndex = currIndex;

    if(m_cIndexBlockRD[uiTraIdxStart]==(MAX_PLT_SIZE-1))
    {
      currIndex = iMaxSymbol - 1;
      pltElement->index=(MAX_PLT_SIZE-1);
    }
    else
    {
      pltElement->index=currIndex;
    }

    if (restrictLevelRun==-1)
    {
      run=0;
      uiIdx=uiIdxStart;
      while (uiIdx<(uiTotal-1))
      {
        uiIdx++;
        uiTraIdx=m_puiScanOrder[uiIdx];

        if (m_cIndexBlockRD[uiTraIdx] == startIndex)
        {
          run++;
        }
        else
        {
          break;
        }
      }
    }
    else if (restrictLevelRun==-2)
    {
      run=0;
      uiIdx=uiIdxStart;
      while (uiIdx<(uiTotal-1))
      {
        uiIdx++;
        uiTraIdx=m_puiScanOrder[uiIdx];

        if (m_cIndexBlockRD[uiTraIdx] == startIndex && m_SPointRD[uiTraIdx] == PLT_RUN_LEFT)
        {
          run++;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      run=restrictLevelRun;
    }
    if (calcErrBits)
    {
      indexBits=calcPltIndexPredAndBits(iMaxSymbol, uiIdxStart, uiWidth, &predIndex, &currIndex);
    }
  }
  else
  {

    pltMode = PLT_RUN_ABOVE;
    pltElement->index = 0;

    run = 0;
    uiIdx = uiIdxStart;
    uiTraIdx = m_puiScanOrder[uiIdx];
    copyPixels[uiTraIdx - uiWidth] = 1;

    m_cIndexBlockRD[uiTraIdx] = m_cIndexBlockRD[uiTraIdx - uiWidth];
    while (uiIdx < (uiTotal-1))
    {
      uiIdx++;
      uiTraIdx=m_puiScanOrder[uiIdx];
      if (m_cIndexBlockRD[uiTraIdx]==m_cIndexBlockRD[uiTraIdx - uiWidth])
      {
        copyPixels[uiTraIdx - uiWidth]=1;
        run++;
      }
      else
      {
        break;
      }
    }
  }


  pltElement->position = uiIdxStart;
  pltElement->pltMode = pltMode;
  pltElement->run = run;

  for (uiIdx = uiIdxStart; uiIdx <= (uiIdxStart + run); uiIdx++)
  {
    uiTraIdx = m_puiScanOrder[uiIdx];

    m_SPointRD[uiTraIdx] = pltMode;
    m_RunRD[uiTraIdx] = run;

    if (m_EscapeFlagRD[uiTraIdx] == 0)
    {
      m_LevelRD[0][uiTraIdx] = m_cIndexBlockRD[uiTraIdx];
    }
  }

  if (calcErrBits)
  {

    UInt escape = 0, usedForCopy = 0;
    UInt64 error = 0;

    for (uiIdx=uiIdxStart; uiIdx<=(uiIdxStart+run); uiIdx++)
    {
      uiTraIdx = m_puiScanOrder[uiIdx]; 
      Pel index = m_cIndexBlockRD[uiTraIdx];
      error += m_indError[m_cPosBlockRD[uiTraIdx]][index];
      if (copyPixels[uiTraIdx])
      {
        usedForCopy = 1;
      }

      UInt escFlagOrig=m_EscapeFlagRD[uiTraIdx];
      UInt escFlagNew = index == (MAX_PLT_SIZE - 1) ? 1 : 0;
      assert(escFlagOrig == escFlagNew);

      if (escFlagNew == 1)
      {
        escape = 1;
      }
    }

    m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
    m_pcRDGoOnSbacCoder->resetBits();
    UInt64 initialBits = m_pcRDGoOnSbacCoder->getNumPartialBits();
    m_pcRDGoOnSbacCoder->encodeSPointRD(uiIdxStart, uiWidth, m_SPointRD, pltMode, m_puiScanOrder);
    sPointBits=m_pcRDGoOnSbacCoder->getNumPartialBits()-initialBits;
    m_pcRDGoOnSbacCoder->encodeRun(run, pltMode, currIndex, uiTotal - uiIdxStart - 1);


    runBits=m_pcRDGoOnSbacCoder->getNumPartialBits()-sPointBits-initialBits;


    pltElement->position = uiIdxStart;
    pltElement->pltMode = pltMode;
    pltElement->run=run;
    pltElement->indexPred = predIndex;
    pltElement->bitsInd=indexBits<<15;
    pltElement->bitsRun = runBits;
    pltElement->bitsAll=runBits+sPointBits+(indexBits<<15);
    pltElement->error = Double(error);
    pltElement->escape = escape;
    pltElement->usedForCopy = usedForCopy;
  }

  return(run);
}

Void TEncSearch::preCalcRDMerge(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize, TComRdCost *pcCost, 
  UInt *errorOrig, UInt *errorNew, UInt calcErrBits)
{
  UInt uiIdx, uiIdxStart, uiTraIdx, noElement, run, uiTotal = uiHeight * uiWidth,
     merge, forceMerge;

  UInt predIndex = 0;
  Double error = 0;

  Double rdCostOrig, rdCostBestMode;
  UInt copyPixels[32*32];
  Int modMode;

  UInt modRunMode, modRunCurrentBest=0, uiModPositionNextBest=0, modRunNextBest=0;
  Double rdCostModRun, rdCostModRunMin;
  UInt pltMode=PLT_RUN_LEFT, pltModeMerge=PLT_RUN_LEFT, uiIdxStartMerge, mergeCurrIndex=0, currIndex, mergePLTIdx=0;
  UInt64 indexBits;
  Double rdCostMerge=MAX_DOUBLE, errorMin = MAX_DOUBLE;
  Int iMaxSymbol;

  UInt uiPLTIdx = 0;

  UInt uiPLTIdxStart, uiPLTIdxEnd; 


  pltInfoStruct* currentPLTElement = &m_currentPLTElement;
  pltInfoStruct* nextPLTElement = &m_nextPLTElement;
  pltInfoStruct *tempPLTElement;

  m_puiScanOrder = g_scanOrder[SCAN_UNGROUPED][SCAN_TRAV][g_aucConvertToBit[uiWidth]+2][g_aucConvertToBit[uiHeight]+2];

  UInt64 errOrig = 0;


  for (uiIdx=0; uiIdx<uiTotal; uiIdx++)
  {
    uiTraIdx = m_puiScanOrder[uiIdx]; 
    if (m_EscapeFlagRD[uiTraIdx])
    {
      errOrig += m_indError[m_cPosBlockRD[uiTraIdx]][MAX_PLT_SIZE];
    }
    else
    {
      uiPLTIdx = m_cIndexBlockRD[uiTraIdx];
      errOrig += m_indError[m_cPosBlockRD[uiTraIdx]][uiPLTIdx];
    }
  }

  *errorOrig = UInt(errOrig);

  UInt uiIndexMaxSize = uiPLTSize;
  if( pcCU->getPLTEscape(COMPONENT_Y, 0) )
  {
    uiIndexMaxSize++;
  }

  memset(copyPixels, 0, sizeof(copyPixels));

  for (noElement=0; noElement<m_pltNoElementsBest; noElement++)
  {
    if (m_pltInfoBest[noElement].pltMode==PLT_RUN_ABOVE)
    {
      UInt end = m_pltInfoBest[noElement].position + m_pltInfoBest[noElement].run;
      for (uiIdx = m_pltInfoBest[noElement].position; uiIdx <= end; uiIdx++)
      {
        uiTraIdx = m_puiScanOrder[uiIdx]; 
        copyPixels[uiTraIdx - uiWidth]=1;

      }
    }
  }

  for (uiIdx=0; uiIdx<uiTotal; uiIdx++)
  {
    if (m_cIndexBlockRD[uiIdx]<0)
    {
      m_cIndexBlockRD[uiIdx]=(MAX_PLT_SIZE-1);
    }
  }

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
  m_pcRDGoOnSbacCoder->saveRestorePltCtx(1);

  uiIdxStart=0; 
  findPltSegment(currentPLTElement, pcCU, uiIdxStart, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, -1, 1);
  uiIdxStart=currentPLTElement->position+(currentPLTElement->run+1);

  while (uiIdxStart < uiTotal)
  {
    findPltSegment(nextPLTElement, pcCU, uiIdxStart, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, -1, 1);
    modMode = 0; merge = 0; forceMerge = 0;  
    if (currentPLTElement->escape == 0 && nextPLTElement->escape == 0)
    {
      run=currentPLTElement->run + nextPLTElement->run + 1;
      
      rdCostOrig=pcCost->getLambda()*(Double)((currentPLTElement->bitsAll+nextPLTElement->bitsAll)>>15)+
        currentPLTElement->error+nextPLTElement->error;
      rdCostBestMode = rdCostOrig;

      if (currentPLTElement->pltMode == nextPLTElement->pltMode && (currentPLTElement->pltMode == PLT_RUN_ABOVE || currentPLTElement->index == nextPLTElement->index))
      {
        forceMerge = 1;
      }
     

      modRunMode=0;
      if (currentPLTElement->pltMode==PLT_RUN_LEFT && nextPLTElement->pltMode==PLT_RUN_ABOVE && forceMerge==0)
      {
        UInt uiModPositionNext, runCurrent, runNext, tempIndex, groupInit, groupNew;
        UInt64 modRunBits;

        rdCostModRunMin=MAX_DOUBLE;
        Int startCopy=calcPLTStartCopy(nextPLTElement->position, currentPLTElement->position, uiWidth);
        runCurrent=startCopy-currentPLTElement->position-1;
        runNext=run-runCurrent-1;

        groupInit=m_runGolombGroups[runNext];

        for (uiModPositionNext=startCopy; uiModPositionNext<nextPLTElement->position; uiModPositionNext++)
        {
          runCurrent=uiModPositionNext-currentPLTElement->position-1;
          runNext=run-runCurrent-1;

          groupNew=m_runGolombGroups[runNext];

          if (uiModPositionNext==startCopy || groupNew<groupInit)
          {
            groupInit=(groupNew<groupInit)? groupNew : groupInit;
            modRunMode=1;

            m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
            m_pcRDGoOnSbacCoder->resetBits();
            UInt64 initialBits=m_pcRDGoOnSbacCoder->getNumPartialBits();
            m_pcRDGoOnSbacCoder->encodeSPointRD(currentPLTElement->position, uiWidth, m_SPointRD, PLT_RUN_LEFT, m_puiScanOrder);
            tempIndex=currentPLTElement->index > currentPLTElement->indexPred ? currentPLTElement->index-1 : currentPLTElement->index ;
            m_pcRDGoOnSbacCoder->encodeRun(runCurrent, PLT_RUN_LEFT, tempIndex, uiTotal - currentPLTElement->position - 1);

            // Next plt segment
            m_pcRDGoOnSbacCoder->encodeSPointRD(uiModPositionNext, uiWidth, m_SPointRD, PLT_RUN_ABOVE, m_puiScanOrder);

            m_pcRDGoOnSbacCoder->encodeRun(runNext, PLT_RUN_ABOVE, tempIndex, uiTotal - uiModPositionNext - 1);

            modRunBits=m_pcRDGoOnSbacCoder->getNumPartialBits()+currentPLTElement->bitsInd-initialBits;
            rdCostModRun=pcCost->getLambda()*(Double)(modRunBits>>15)+currentPLTElement->error+nextPLTElement->error;


            if (rdCostModRun<rdCostModRunMin)
            {
              rdCostModRunMin=rdCostModRun;
              uiModPositionNextBest=uiModPositionNext;
              modRunCurrentBest=runCurrent;
              modRunNextBest=runNext;
            }
          }
        }
      }

      if (modRunMode==1 && rdCostModRunMin<=rdCostBestMode)
      {
        rdCostBestMode=rdCostModRunMin;
        modMode=1;
      }


      if (!pcCU->getCUTransquantBypass(0))
      {
        uiIdxStartMerge = currentPLTElement->position;
        predIndex = currentPLTElement->indexPred;

        UInt testLevel=0;
        if (currentPLTElement->pltMode==PLT_RUN_LEFT)
        {
          if (currentPLTElement->pltMode==PLT_RUN_LEFT && nextPLTElement->pltMode==PLT_RUN_LEFT && (currentPLTElement->usedForCopy==0 || nextPLTElement->usedForCopy==0))
          { 
            testLevel=1;
          }
          if (nextPLTElement->pltMode==PLT_RUN_ABOVE && nextPLTElement->usedForCopy==0)
          { 
            testLevel=1;
          }
          if (forceMerge==1)
          {
            testLevel=1;
          }
        }

        UInt testCopy=(currentPLTElement->position>=uiWidth) ? 1 : 0;
        if (currentPLTElement->pltMode==PLT_RUN_LEFT && currentPLTElement->usedForCopy==1)
        {
          testCopy=0;
        }
        if (nextPLTElement->pltMode==PLT_RUN_LEFT && nextPLTElement->usedForCopy==1)
        {
          testCopy=0;
        }
 
        if (testLevel)
        { 
          pltMode = PLT_RUN_LEFT;
          iMaxSymbol = (uiIdxStartMerge > 0) ? uiIndexMaxSize - 1 : uiIndexMaxSize;
          uiPLTIdxStart = 0, uiPLTIdxEnd = uiPLTSize - 1;

          if (currentPLTElement->usedForCopy == 1)
          {
            uiPLTIdxStart = currentPLTElement->index;
            uiPLTIdxEnd = currentPLTElement->index;
          }
          else if (nextPLTElement->usedForCopy == 1)
          {
            uiPLTIdxStart = nextPLTElement->index;
            uiPLTIdxEnd = nextPLTElement->index;
          }

          errorMin = MAX_DOUBLE;
          for (uiPLTIdx = uiPLTIdxStart; uiPLTIdx <= uiPLTIdxEnd; uiPLTIdx++)
          {
            if (uiPLTIdx != predIndex)
            { // do not allow copy mode
              merge = 1;
              currIndex = uiPLTIdx > predIndex ? uiPLTIdx - 1 : uiPLTIdx;

              indexBits = m_truncBinBits[currIndex][iMaxSymbol];
              error = pcCost->getLambda()*indexBits; // error includes index
              error += calcPltErrorLevel(uiIdxStartMerge, run, uiPLTIdx);

              if (error<errorMin)
              {
                errorMin = error;
                mergePLTIdx = uiPLTIdx;
                mergeCurrIndex = currIndex;
              }
            }
          }

          if (merge==1)
          {
            m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
            m_pcRDGoOnSbacCoder->resetBits();
            UInt64 initialBits=m_pcRDGoOnSbacCoder->getNumPartialBits();
            m_pcRDGoOnSbacCoder->encodeSPointRD(uiIdxStartMerge, uiWidth, m_SPointRD, pltMode, m_puiScanOrder);
            m_pcRDGoOnSbacCoder->encodeRun(run, pltMode, mergeCurrIndex, uiTotal - uiIdxStartMerge - 1);

            rdCostMerge=pcCost->getLambda()*(Double)((m_pcRDGoOnSbacCoder->getNumPartialBits()-initialBits)>>15)+errorMin;
          }

          if ((merge==1 && rdCostMerge<=rdCostBestMode) || forceMerge==1)
          {
            rdCostBestMode=rdCostMerge;
            modMode=2;
            pltModeMerge=PLT_RUN_LEFT;
          }
        }


        if (testCopy) 
        { 
          pltMode=PLT_RUN_ABOVE;
          errorMin=calcPltErrorCopy(uiIdxStartMerge, run, uiWidth, &merge);

          if (currentPLTElement->pltMode==PLT_RUN_ABOVE && nextPLTElement->pltMode==PLT_RUN_ABOVE)
          {
            merge=1;
          }


        if (merge == 1)
        {
          m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
          m_pcRDGoOnSbacCoder->resetBits();
          UInt64 initialBits = m_pcRDGoOnSbacCoder->getNumPartialBits();
          m_pcRDGoOnSbacCoder->encodeSPointRD(uiIdxStartMerge, uiWidth, m_SPointRD, pltMode, m_puiScanOrder);
          m_pcRDGoOnSbacCoder->encodeRun(run, pltMode, mergeCurrIndex, uiTotal - uiIdxStartMerge - 1);
          rdCostMerge = pcCost->getLambda()*(Double)((m_pcRDGoOnSbacCoder->getNumPartialBits() - initialBits) >> 15) + errorMin;
        }

          if (merge==1 && rdCostMerge<=rdCostBestMode)
          {
            rdCostBestMode=rdCostMerge;
            modMode=2;
            pltModeMerge=PLT_RUN_ABOVE;
          }
        }


      }

    }

    if (modMode==0)
    {
      tempPLTElement=currentPLTElement;
      currentPLTElement=nextPLTElement;
      nextPLTElement=tempPLTElement;
    }
    else
    {
      if (modMode==1) 
      {
        modifyPltSegment(uiWidth, uiModPositionNextBest, nextPLTElement->pltMode, nextPLTElement->index, modRunNextBest);
        findPltSegment(currentPLTElement, pcCU, currentPLTElement->position, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, modRunCurrentBest, 1);
        findPltSegment(currentPLTElement, pcCU, uiModPositionNextBest, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, -1, 1);
      }
      if (modMode==2)
      {
        modifyPltSegment(uiWidth, currentPLTElement->position, pltModeMerge, mergePLTIdx, currentPLTElement->run);
        modifyPltSegment(uiWidth, nextPLTElement->position, pltModeMerge, mergePLTIdx, nextPLTElement->run);
        findPltSegment(currentPLTElement, pcCU, currentPLTElement->position, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, -1, 1);
      }
    }

    uiIdxStart=currentPLTElement->position+(currentPLTElement->run+1);
  }

  uiIdxStart=0; noElement=0;
  while (uiIdxStart<uiTotal)
  {
    findPltSegment(m_pltInfoBest + noElement, pcCU, uiIdxStart, uiIndexMaxSize, uiWidth, uiTotal, copyPixels, -2, calcErrBits);
    uiIdxStart=m_pltInfoBest[noElement].position+(m_pltInfoBest[noElement].run+1);
    noElement++;
  }

  m_pltNoElementsBest=noElement;


  UInt64 errNew = 0;

  for (uiIdx=0; uiIdx<uiTotal; uiIdx++)
  {
    uiTraIdx = m_puiScanOrder[uiIdx]; 
    if (m_EscapeFlagRD[uiTraIdx])
    {
      errNew += m_indError[m_cPosBlockRD[uiTraIdx]][MAX_PLT_SIZE];
    }
    else
    {
      uiPLTIdx=m_cIndexBlockRD[uiTraIdx];
      errNew += m_indError[m_cPosBlockRD[uiTraIdx]][uiPLTIdx];
    }
  }

  *errorNew = UInt(errNew);
}   
#endif

Void TEncSearch::xDeriveRun(TComDataCU* pcCU, Pel* pOrg[3],  Pel *pPalette[3],  Pel* pValue, UChar* pSPoint,
  Pel** paPixelValue, Pel ** paRecoValue, TCoeff* pRun,
  UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize)
{
  UInt uiTotal = uiHeight * uiWidth, uiIdx = 0;
  UInt uiStartPos = 0,  uiRun = 0, uiCopyRun = 0;
  Int iTemp = 0;
  UInt uiTraIdx;  //unified position variable (raster scan)
  Pel *pcIndexBlock = m_cIndexBlock;

  UChar *pEscapeFlag  = pcCU->getEscapeFlag(COMPONENT_Y);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt noElements=0;
  UInt64 allBitsCopy, allBitsIndex, indexBits, runBitsIndex, runBitsCopy;

  UInt uiIndexMaxSize = pcCU->getPLTSize(COMPONENT_Y, 0);
  if( pcCU->getPLTEscape(COMPONENT_Y, 0) )
  {
    uiIndexMaxSize++;
  }

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
  m_pcRDGoOnSbacCoder->saveRestorePltCtx(1);
#endif

  //Test Run
  while (uiIdx < uiTotal)
  {
    uiStartPos = uiIdx;
    Double dAveBitsPerPix[NUM_PLT_RUN];

    uiRun = 0;
    Bool RunValid = calLeftRun(pcCU, pValue, pSPoint, uiStartPos, uiTotal, uiRun, pEscapeFlag);

    if(RunValid)
    {
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
      dAveBitsPerPix[PLT_RUN_LEFT] = xGetRunBits(pcCU, pValue, uiStartPos, (uiRun + 1), PLT_RUN_LEFT, &allBitsIndex, &indexBits, &runBitsIndex);
#else
      dAveBitsPerPix[PLT_RUN_LEFT] = xGetRunBits(pcCU, pValue, uiStartPos, (uiRun + 1), PLT_RUN_LEFT);
#endif
    }
    else
    {
      dAveBitsPerPix[PLT_RUN_LEFT] = std::numeric_limits<double>::max();
    }

    uiCopyRun = 0;
    Bool CopyValid = calAboveRun(pcCU, pValue, pSPoint, uiWidth, uiStartPos, uiTotal, uiCopyRun, pEscapeFlag);

    if(CopyValid)
    {
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
      dAveBitsPerPix[PLT_RUN_ABOVE] = xGetRunBits(pcCU, pValue, uiStartPos, uiCopyRun, PLT_RUN_ABOVE, &allBitsCopy, &indexBits, &runBitsCopy);
#else
      dAveBitsPerPix[PLT_RUN_ABOVE] = xGetRunBits(pcCU, pValue, uiStartPos, uiCopyRun, PLT_RUN_ABOVE);
#endif
    }
    else
    {
      dAveBitsPerPix[PLT_RUN_ABOVE] = std::numeric_limits<double>::max();
    }

    uiTraIdx = m_puiScanOrder[uiIdx];    //unified position variable (raster scan)

    assert(RunValid || CopyValid);

    {
      if( dAveBitsPerPix[PLT_RUN_ABOVE] <= dAveBitsPerPix[PLT_RUN_LEFT] )
      {
        pSPoint[uiTraIdx] = PLT_RUN_ABOVE;
        pRun[uiTraIdx]  = uiCopyRun-1;

        pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
        Double error=0;
        m_pltInfo[noElements].index    = 0;
        m_pltInfo[noElements].position = uiIdx;
        m_pltInfo[noElements].pltMode  = pSPoint[uiTraIdx];
        m_pltInfo[noElements].run      = pRun[uiTraIdx];
        m_pltInfo[noElements].bitsRun  = runBitsCopy;
        m_pltInfo[noElements].bitsInd  = 0;
        m_pltInfo[noElements].bitsAll  = allBitsCopy;
#endif

        if( pEscapeFlag[uiTraIdx] )
        {
          calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx); 
        }
        uiIdx++;

        iTemp = uiCopyRun - 1;
        while (iTemp > 0)
        {
          uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)

          pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

          if( pEscapeFlag[uiTraIdx] )
          {
            calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
          }

          pSPoint[uiTraIdx] = PLT_RUN_ABOVE;
          uiIdx++;

          iTemp--;
        }
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
        m_pltInfo[noElements].error = error;
        noElements++;
#endif
      }
      else
      {
        pSPoint[uiTraIdx] = PLT_RUN_LEFT;
        pRun[uiTraIdx] = uiRun;

        pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
        Double error=0;
        m_pltInfo[noElements].position = uiIdx;
        m_pltInfo[noElements].pltMode  = pSPoint[uiTraIdx];
        m_pltInfo[noElements].run      = pRun[uiTraIdx];
        m_pltInfo[noElements].bitsInd  = indexBits;
        m_pltInfo[noElements].bitsRun  = runBitsIndex;
        m_pltInfo[noElements].bitsAll  = allBitsIndex;
        
        m_pltInfo[noElements].index    = pEscapeFlag[uiTraIdx] ? (MAX_PLT_SIZE - 1) : pValue[uiTraIdx];
       

        if (uiIdx>0)
        {
          UInt uiTraIdxLeft = m_puiScanOrder[uiIdx - 1];
          if (pSPoint[uiTraIdxLeft] == PLT_RUN_LEFT)  ///< copy left
          {
            m_pltInfo[noElements].indexPred = pValue[uiTraIdxLeft];
            if( pEscapeFlag[uiTraIdxLeft] )
            {
              m_pltInfo[noElements].indexPred = uiIndexMaxSize - 1;
            }
          }
          else
          {
            m_pltInfo[noElements].indexPred = pValue[uiTraIdx - uiWidth];
            if( pEscapeFlag[uiTraIdx - uiWidth] )
            {
              m_pltInfo[noElements].indexPred = uiIndexMaxSize - 1;
            }
          }
        }
#endif

        if( pEscapeFlag[uiTraIdx] )
        {
          calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
        }

        uiIdx++;

        iTemp = uiRun;
        while (iTemp > 0)
        {
          uiTraIdx = m_puiScanOrder[uiIdx];  //unified position variable (raster scan)

          pEscapeFlag[uiTraIdx] = ( pcIndexBlock[uiTraIdx] < 0 );

          if( pEscapeFlag[uiTraIdx] )
          {
            calcPixelPred(pcCU, pOrg, pPalette, pValue, paPixelValue, paRecoValue, uiWidth, uiHeight, uiStrideOrg, uiTraIdx);
          }

          pSPoint[uiTraIdx] = PLT_RUN_LEFT;
          uiIdx++;
          iTemp--;
        }
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
        m_pltInfo[noElements].error=error;
        noElements++;
#endif
      }
    }
  }
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  m_pltNoElements = noElements;
#endif

  assert (uiIdx == uiTotal);
}

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
Double TEncSearch::xGetRunBits(TComDataCU* pcCU, Pel *pValue, UInt uiStartPos, UInt uiRun, PLTRunMode cPltRunMode, UInt64 *allBits, UInt64 *indexBits, UInt64 *runBits)
#else
Double TEncSearch::xGetRunBits(TComDataCU* pcCU, Pel *pValue, UInt uiStartPos, UInt uiRun, PLTRunMode cPltRunMode)
#endif
{
#if!SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt uiDepth      = pcCU->getDepth(0);
#endif
  UInt uiWidth      = pcCU->getWidth(0);
  UInt uiHeight = pcCU->getHeight(0);
  UInt uiTotal = uiWidth * uiHeight;
  UInt siCurLevel = 0;
  UInt uiIndexMaxSize = pcCU->getPLTSize(COMPONENT_Y, 0);
  if( pcCU->getPLTEscape(COMPONENT_Y, 0) )
  {
    uiIndexMaxSize++;
  }

  UChar* pSPoint     = pcCU->getSPoint(COMPONENT_Y);

  UInt   uiTraIdx    = m_puiScanOrder[uiStartPos];
  UInt   uiRealLevel = pValue[uiTraIdx];
  UChar* pEscapeFlag = pcCU->getEscapeFlag(COMPONENT_Y);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  m_pcRDGoOnSbacCoder->saveRestorePltCtx(0);
#else
  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
#endif
  m_pcEntropyCoder->resetBits();

  m_pcRDGoOnSbacCoder->encodeSPoint(pcCU, 0, uiStartPos, uiWidth, pSPoint, m_puiScanOrder);


#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt64 sPointBits=m_pcRDGoOnSbacCoder->getNumPartialBits();
#endif

  assert(uiRun >= 1);
  switch(cPltRunMode)
  {
  case PLT_RUN_LEFT:
    if( pEscapeFlag[uiTraIdx] )
    {
      pValue[uiTraIdx] = uiIndexMaxSize - 1;
    }

    siCurLevel = m_pcRDGoOnSbacCoder->writePLTIndex(uiStartPos, pValue, uiIndexMaxSize, pSPoint, uiWidth, pEscapeFlag);

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
    *indexBits = m_pcRDGoOnSbacCoder->getNumPartialBits() - sPointBits;
#endif
    if( pEscapeFlag[uiTraIdx] )
    {
      pValue[uiTraIdx] = uiRealLevel;
    }
    m_pcRDGoOnSbacCoder->encodeRun((uiRun - 1), PLT_RUN_LEFT, siCurLevel, uiTotal - uiStartPos - 1);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
    * runBits = m_pcRDGoOnSbacCoder->getNumPartialBits() - sPointBits - (*indexBits);
#endif

    break;
  case PLT_RUN_ABOVE:
    m_pcRDGoOnSbacCoder->encodeRun((uiRun - 1), PLT_RUN_ABOVE, siCurLevel, uiTotal - uiStartPos - 1);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
    * runBits = m_pcRDGoOnSbacCoder->getNumPartialBits() - sPointBits;
#endif

    break;
  default:
    assert(0);
  }

  UInt uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  *allBits=m_pcRDGoOnSbacCoder->getNumPartialBits();
#endif
  Double dCostPerPixel = uiBits * 1.0 / uiRun;  
  return dCostPerPixel;
}

//!  Function for PCM mode estimation.
Void TEncSearch::IPCMSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv )
{
  UInt        uiDepth      = pcCU->getDepth(0);
  const UInt  uiDistortion = 0;
  UInt        uiBits;

  Double dCost;

  for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID  = ComponentID(ch);
    const UInt width  = pcCU->getWidth(0)  >> pcCU->getPic()->getComponentScaleX(compID);
    const UInt height = pcCU->getHeight(0) >> pcCU->getPic()->getComponentScaleY(compID);
    const UInt stride = pcPredYuv->getStride(compID);

    Pel * pOrig    = pcOrgYuv->getAddr  (compID, 0, width);
    Pel * pResi    = pcResiYuv->getAddr(compID, 0, width);
    Pel * pPred    = pcPredYuv->getAddr(compID, 0, width);
    Pel * pReco    = pcRecoYuv->getAddr(compID, 0, width);
    Pel * pPCM     = pcCU->getPCMSample (compID);

    xEncPCM ( pcCU, 0, pOrig, pPCM, pPred, pResi, pReco, stride, width, height, compID );

  }

  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiDepth, 0, true, false);
  uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;

  pcCU->copyToPic(uiDepth);
}




Void TEncSearch::xGetInterPredictionError( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Distortion& ruiErr, Bool /*bHadamard*/ )
{
  motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );

  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;
  pcCU->getPartIndexAndSize( iPartIdx, uiAbsPartIdx, iWidth, iHeight );

  DistParam cDistParam;

  cDistParam.bApplyWeight = false;


  m_pcRdCost->setDistParam( cDistParam, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA),
                            pcYuvOrg->getAddr( COMPONENT_Y, uiAbsPartIdx ), pcYuvOrg->getStride(COMPONENT_Y),
                            m_tmpYuvPred .getAddr( COMPONENT_Y, uiAbsPartIdx ), m_tmpYuvPred.getStride(COMPONENT_Y),
                            iWidth, iHeight, m_pcEncCfg->getUseHADME() && (pcCU->getCUTransquantBypass(uiAbsPartIdx) == 0) );

  ruiErr = cDistParam.DistFunc( &cDistParam );
}

//! estimation of best merge coding
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPUIdx, UInt& uiInterDir, TComMvField* pacMvField, UInt& uiMergeIndex, Distortion& ruiCost, TComMvField* cMvFieldNeighbours, UChar* uhInterDirNeighbours, Int& numValidMergeCand, Int iCostCalcType )
{
  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;

  pcCU->getPartIndexAndSize( iPUIdx, uiAbsPartIdx, iWidth, iHeight );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );

  PartSize partSize = pcCU->getPartitionSize( 0 );
  if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
  {
    if ( iPUIdx == 0 )
    {
      pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth ); // temporarily set
      pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
      pcCU->setPartSizeSubParts( partSize, 0, uiDepth ); // restore
    }
  }
  else
  {
    pcCU->getInterMergeCandidates( uiAbsPartIdx, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
  }

  xRestrictBipredMergeCand( pcCU, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );

  ruiCost = std::numeric_limits<Distortion>::max();
  if ( iCostCalcType )
  {
    m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( uiAbsPartIdx ) );
  }
  
  for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
  {
    if ( (uhInterDirNeighbours[uiMergeCand] == 1 || uhInterDirNeighbours[uiMergeCand] == 3) && pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, cMvFieldNeighbours[uiMergeCand<<1].getRefIdx() )->getPOC() == pcCU->getSlice()->getPOC() )
    {
      continue;
    }
    Distortion uiCostCand = std::numeric_limits<Distortion>::max();
    UInt       uiBitsCand = 0;

    PartSize ePartSize = pcCU->getPartitionSize( 0 );

    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );

    if ( !iCostCalcType )
    {
      xGetInterPredictionError( pcCU, pcYuvOrg, iPUIdx, uiCostCand, m_pcEncCfg->getUseHADME() );
    }
    else
    {
      motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPUIdx );

      uiCostCand = 0;
      for (Int ch = COMPONENT_Y; ch < (iCostCalcType==1?pcCU->getPic()->getNumberValidComponents(): COMPONENT_Y+1); ch++)
      {
        Int iTempWidth = iWidth >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));
        Int iTempHeight = iHeight >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));
        Int iRefStride = m_tmpYuvPred.getStride(ComponentID(ch));
        Int iOrgStride = pcYuvOrg->getStride(ComponentID(ch));
        Pel *pRef =  m_tmpYuvPred.getAddr( ComponentID(ch), uiAbsPartIdx);
        Pel *pOrg = pcYuvOrg->getAddr( ComponentID(ch), uiAbsPartIdx);

        uiCostCand += getSAD( pRef, iRefStride, pOrg, iOrgStride, iTempWidth, iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );

        if ( uiCostCand >= ruiCost )
        {
          break;
        }
      }
    }

    uiBitsCand = uiMergeCand + 1;
    if (uiMergeCand == m_pcEncCfg->getMaxNumMergeCand() -1)
    {
        uiBitsCand--;
    }
    uiCostCand = uiCostCand + m_pcRdCost->getCost( uiBitsCand );
    if ( uiCostCand < ruiCost )
    {
      ruiCost = uiCostCand;
      pacMvField[0] = cMvFieldNeighbours[0 + 2*uiMergeCand];
      pacMvField[1] = cMvFieldNeighbours[1 + 2*uiMergeCand];
      uiInterDir = uhInterDirNeighbours[uiMergeCand];
      uiMergeIndex = uiMergeCand;
    }
  }
}

/** convert bi-pred merge candidates to uni-pred
 * \param pcCU
 * \param puIdx
 * \param mvFieldNeighbours
 * \param interDirNeighbours
 * \param numValidMergeCand
 * \returns Void
 */
Void TEncSearch::xRestrictBipredMergeCand( TComDataCU* pcCU, UInt puIdx, TComMvField* mvFieldNeighbours, UChar* interDirNeighbours, Int numValidMergeCand )
{
  if ( pcCU->isBipredRestriction(puIdx) )
  {
    for( UInt mergeCand = 0; mergeCand < numValidMergeCand; ++mergeCand )
    {
      if ( interDirNeighbours[mergeCand] == 3 )
      {
        interDirNeighbours[mergeCand] = 1;
        mvFieldNeighbours[(mergeCand << 1) + 1].setMvField(TComMv(0,0), -1);
      }
    }
  }
}

//! search of the best candidate for inter prediction
#if AMP_MRG
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseRes, Bool bUseMRG, TComMv* iMVCandList )
#else
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* pcPredYuv, TComYuv* pcResiYuv, TComYuv* pcRecoYuv, Bool bUseRes, TComMv* iMVCandList )
#endif
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].clear();
  }
  m_cYuvPredTemp.clear();
  pcPredYuv->clear();

  if ( !bUseRes )
  {
    pcResiYuv->clear();
  }

  pcRecoYuv->clear();

  TComMv       cMvSrchRngLT;
  TComMv       cMvSrchRngRB;

  TComMv       cMvZero;
  TComMv       TempMv; //kolya

  TComMv       cMv[2];
  TComMv       cMvBi[2];
  TComMv       cMvTemp[2][33];

  Int          iNumPart    = pcCU->getNumPartitions();
  Int          iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;

  TComMv       cMvPred[2][33];

  TComMv       cMvPredBi[2][33];
  Int          aaiMvpIdxBi[2][33];

  Int          aaiMvpIdx[2][33];
  Int          aaiMvpNum[2][33];

  AMVPInfo     aacAMVPInfo[2][33];

  Int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  Int          iRefIdxBi[2];

  UInt         uiPartAddr;
  Int          iRoiWidth, iRoiHeight;

  UInt         uiMbBits[3] = {1, 1, 0};

  UInt         uiLastMode = 0;
  Int          iRefStart, iRefEnd;

  PartSize     ePartSize = pcCU->getPartitionSize( 0 );

  Int          bestBiPRefIdxL1 = 0;
  Int          bestBiPMvpL1 = 0;
  Distortion   biPDistTemp = std::numeric_limits<Distortion>::max();

  TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0 ;

  for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )
  {
    Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion   uiCostBi  =   std::numeric_limits<Distortion>::max();
    Distortion   uiCostTemp;

    UInt         uiBits[3];
    UInt         uiBitsTemp;
    Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (Int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    UInt         uiBitsTempL0[MAX_NUM_REF];

    TComMv       mvValidList1;
    Int          refIdxValidList1 = 0;
    UInt         bitsValidList1 = MAX_UINT;
    Distortion   costValidList1 = std::numeric_limits<Distortion>::max();

    xGetBlkBits( ePartSize, pcCU->getSlice()->isInterP(), iPartIdx, uiLastMode, uiMbBits);

    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

#if AMP_MRG
    Bool bTestNormalMC = true;

    if ( bUseMRG && pcCU->getWidth( 0 ) > 8 && iNumPart == 2 )
    {
      bTestNormalMC = false;
    }

    if (bTestNormalMC)
    {
#endif

    //  Uni-directional prediction
    for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
    {
      RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      Int refPicNumber = pcCU->getSlice()->getNumRefIdx( eRefPicList );
#if SCM_U0083_U0079_IBC_SIGNAL_PPS
      if ( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#else
      if ( pcCU->getSlice()->getSPS()->getSpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#endif
      {
        refPicNumber--;
      }
      for ( Int iRefIdxTemp = 0; iRefIdxTemp < refPicNumber; iRefIdxTemp++ )
      {
        uiBitsTemp = uiMbBits[iRefList];
        if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 )
          {
            uiBitsTemp--;
          }
        }
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp);
        aaiMvpIdx[iRefList][iRefIdxTemp] = pcCU->getMVPIdx(eRefPicList, uiPartAddr);
        aaiMvpNum[iRefList][iRefIdxTemp] = pcCU->getMVPNum(eRefPicList, uiPartAddr);

        if(pcCU->getSlice()->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
        {
          bestBiPDist = biPDistTemp;
          bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
          bestBiPRefIdxL1 = iRefIdxTemp;
        }

        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

        if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )    // list 1
        {
          if ( pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
          {
            cMvTemp[1][iRefIdxTemp] = cMvTemp[0][pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            uiCostTemp = uiCostTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )];
            /*first subtract the bit-rate part of the cost of the other list*/
            uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp )] );
            /*correct the bit-rate part of the current ref*/
            m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
            uiBitsTemp += m_pcRdCost->getBits( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer() );
            /*calculate the correct cost*/
            uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
          }
          else
          {
            xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
          }
        }
        else
        {
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
        }
        xCopyAMVPInfo(pcCU->getCUMvField(eRefPicList)->getAMVPInfo(), &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
        xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

        if ( iRefList <= 1 && iRefIdxTemp <= 1 && (ePartSize == SIZE_2NxN || ePartSize == SIZE_Nx2N) && pcCU->getWidth( 0 ) <= 16 )
        {
          iMVCandList[4*iRefList + 2*iRefIdxTemp + iPartIdx] = cMvTemp[iRefList][iRefIdxTemp];
        }
        if ( iRefList == 0 )
        {
          uiCostTempL0[iRefIdxTemp] = uiCostTemp;
          uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
        }
        if ( uiCostTemp < uiCost[iRefList] )
        {
          uiCost[iRefList] = uiCostTemp;
          uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

          // set motion
          cMv[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
          iRefIdx[iRefList] = iRefIdxTemp;
        }

        if ( iRefList == 1 && uiCostTemp < costValidList1 && pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
        {
          costValidList1 = uiCostTemp;
          bitsValidList1 = uiBitsTemp;

          // set motion
          mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
          refIdxValidList1 = iRefIdxTemp;
        }
      }
    }

    //  Bi-directional prediction
    if ( (pcCU->getSlice()->isInterB()) && (pcCU->isBipredRestriction(iPartIdx) == false) )
    {

      cMvBi[0] = cMv[0];            cMvBi[1] = cMv[1];
      iRefIdxBi[0] = iRefIdx[0];    iRefIdxBi[1] = iRefIdx[1];

      ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
      ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

      UInt uiMotBits[2];

      if(pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
        pcCU->setMVPIdxSubParts( bestBiPMvpL1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
        cMvPredBi[1][bestBiPRefIdxL1]   = pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo()->m_acMvCand[bestBiPMvpL1];

        cMvBi[1] = cMvPredBi[1][bestBiPRefIdxL1];
        iRefIdxBi[1] = bestBiPRefIdxL1;
#if SCM_AMVR_UNIFICATION
        if( pcCU->getSlice()->getUseIntegerMv() )
        {
          cMvBi[1] = (cMvBi[1]>>2)<<2;          
        }
#endif
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        TComYuv* pcYuvPred = &m_acYuvPred[REF_PIC_LIST_1];
        motionCompensation( pcCU, pcYuvPred, REF_PIC_LIST_1, iPartIdx );

        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiMbBits[1];

        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiMotBits[1] += bestBiPRefIdxL1+1;
          if ( bestBiPRefIdxL1 == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 )
          {
            uiMotBits[1]--;
          }
        }

        uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

        cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
      }
      else
      {
        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiBits[1] - uiMbBits[1];
        uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
      }

      // 4-times iteration (default)
      Int iNumIter = 4;

      // fast encoder setting: only one iteration
      if ( m_pcEncCfg->getUseFastEnc() || pcCU->getSlice()->getMvdL1ZeroFlag())
      {
        iNumIter = 1;
      }

      for ( Int iIter = 0; iIter < iNumIter; iIter++ )
      {
        Int         iRefList    = iIter % 2;

        if ( m_pcEncCfg->getUseFastEnc() )
        {
          if( uiCost[0] <= uiCost[1] )
          {
            iRefList = 1;
          }
          else
          {
            iRefList = 0;
          }
        }
        else if ( iIter == 0 )
        {
          iRefList = 0;
        }
        if ( iIter == 0 && !pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllMv( cMv[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField(RefPicList(1-iRefList))->setAllRefIdx( iRefIdx[1-iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
          TComYuv*  pcYuvPred = &m_acYuvPred[1-iRefList];
          motionCompensation ( pcCU, pcYuvPred, RefPicList(1-iRefList), iPartIdx );
        }

        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

        if(pcCU->getSlice()->getMvdL1ZeroFlag())
        {
          iRefList = 0;
          eRefPicList = REF_PIC_LIST_0;
        }

        Bool bChanged = false;

        iRefStart = 0;
        iRefEnd   = pcCU->getSlice()->getNumRefIdx(eRefPicList)-1;
#if SCM_U0083_U0079_IBC_SIGNAL_PPS
        if ( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#else
        if ( pcCU->getSlice()->getSPS()->getSpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#endif
        {
          iRefEnd--;
        }

        for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
          if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 )
            {
              uiBitsTemp--;
            }
          }
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
          // call ME
          xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, true );

          xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], pcCU->getCUMvField(eRefPicList)->getAMVPInfo());
          xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

          if ( uiCostTemp < uiCostBi )
          {
            bChanged = true;

            cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdxBi[iRefList] = iRefIdxTemp;

            uiCostBi            = uiCostTemp;
            uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
            uiBits[2]           = uiBitsTemp;

            if(iNumIter!=1)
            {
              //  Set motion
              pcCU->getCUMvField( eRefPicList )->setAllMv( cMvBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );
              pcCU->getCUMvField( eRefPicList )->setAllRefIdx( iRefIdxBi[iRefList], ePartSize, uiPartAddr, 0, iPartIdx );

              TComYuv* pcYuvPred = &m_acYuvPred[iRefList];
              motionCompensation( pcCU, pcYuvPred, eRefPicList, iPartIdx );
            }
          }
        } // for loop-iRefIdxTemp

        if ( !bChanged )
        {
          if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] )
          {
            xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], pcCU->getCUMvField(REF_PIC_LIST_0)->getAMVPInfo());
            xCheckBestMVP(pcCU, REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi);
            if(!pcCU->getSlice()->getMvdL1ZeroFlag())
            {
              xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], pcCU->getCUMvField(REF_PIC_LIST_1)->getAMVPInfo());
              xCheckBestMVP(pcCU, REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi);
            }
          }
          break;
        }
      } // for loop-iter
    } // if (B_SLICE)

#if AMP_MRG
    } //end if bTestNormalMC
#endif
    //  Clear Motion Field
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,       ePartSize, uiPartAddr, 0, iPartIdx );

    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

    UInt uiMEBits = 0;
    // Set Motion Field_
    cMv[1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits[1] = bitsValidList1;
    uiCost[1] = costValidList1;

#if AMP_MRG
    if (bTestNormalMC)
    {
#endif
    if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
    {
      uiLastMode = 2;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMvBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdxBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );

#if SCM_AMVR_UNIFICATION
      if( pcCU->getSlice()->getUseIntegerMv() )
      {       
        TempMv = (cMvBi[0]>>2) - (cMvPredBi[0][iRefIdxBi[0]]>>2);        
      }
      else
      {
        TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
      }
#else
      TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
#endif 
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

#if SCM_AMVR_UNIFICATION
      if( pcCU->getSlice()->getUseIntegerMv() )
      {     
        TempMv = (cMvBi[1]>>2) - (cMvPredBi[1][iRefIdxBi[1]]>>2);
      }
      else
      {
        TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
      }
#else
      TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
#endif 
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdxBi[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPIdxSubParts( aaiMvpIdxBi[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdxBi[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[2];
    }
    else if ( uiCost[0] <= uiCost[1] )
    {
      uiLastMode = 0;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMv[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdx[0], ePartSize, uiPartAddr, 0, iPartIdx );
#if SCM_AMVR_UNIFICATION
      if( pcCU->getSlice()->getUseIntegerMv() )
      {        
        TempMv = (cMv[0]>>2) - (cMvPred[0][iRefIdx[0]]>>2);
      }
      else
      {
        TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
      }
#else
      TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
#endif 
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdx[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[0][iRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[0];
    }
    else
    {
      uiLastMode = 1;
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMv[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdx[1], ePartSize, uiPartAddr, 0, iPartIdx );
#if SCM_AMVR_UNIFICATION
      if( pcCU->getSlice()->getUseIntegerMv() )
      {        
        TempMv = (cMv[1]>>2) - (cMvPred[1][iRefIdx[1]]>>2);        
      }
      else
      {
        TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
      }      
#else
      TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
#endif 
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

      pcCU->setMVPIdxSubParts( aaiMvpIdx[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      pcCU->setMVPNumSubParts( aaiMvpNum[1][iRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));

      uiMEBits = uiBits[1];
    }
#if AMP_MRG
    } // end if bTestNormalMC
#endif

    if ( pcCU->getPartitionSize( uiPartAddr ) != SIZE_2Nx2N )
    {
      UInt uiMRGInterDir = 0;
      TComMvField cMRGMvField[2];
      UInt uiMRGIndex = 0;

      UInt uiMEInterDir = 0;
      TComMvField cMEMvField[2];

      m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

#if AMP_MRG
      // calculate ME cost
      Distortion uiMEError = std::numeric_limits<Distortion>::max();
      Distortion uiMECost  = std::numeric_limits<Distortion>::max();

      if (bTestNormalMC)
      {
        xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
        uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
      }
#else
      // calculate ME cost
      Distortion uiMEError = std::numeric_limits<Distortion>::max();
      xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiMEError, m_pcEncCfg->getUseHADME() );
      Distortion uiMECost = uiMEError + m_pcRdCost->getCost( uiMEBits );
#endif
      // save ME result.
      uiMEInterDir = pcCU->getInterDir( uiPartAddr );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_0, cMEMvField[0] );
      pcCU->getMvField( pcCU, uiPartAddr, REF_PIC_LIST_1, cMEMvField[1] );

      // find Merge result
      Distortion uiMRGCost = std::numeric_limits<Distortion>::max();
#if SCM_AMVR_UNIFICATION
      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
#endif
      xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField, uiMRGIndex, uiMRGCost, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand);

      if ( uiMRGCost < uiMECost )
      {
        // set Merge result
        pcCU->setMergeFlagSubParts ( true,          uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setMergeIndexSubParts( uiMRGIndex,    uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setInterDirSubParts  ( uiMRGInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvd    ( cMvZero,            ePartSize, uiPartAddr, 0, iPartIdx );

        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
      }
      else
      {
        // set ME result
        pcCU->setMergeFlagSubParts( false,        uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->setInterDirSubParts ( uiMEInterDir, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMEMvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMEMvField[1], ePartSize, uiPartAddr, 0, iPartIdx );
      }
    }

    //  MC
    motionCompensation ( pcCU, pcPredYuv, REF_PIC_LIST_X, iPartIdx );

  } //  end of for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )

  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  return;
}

Bool TEncSearch::isBlockVectorValid( Int xPos, Int yPos, Int width, Int height, TComDataCU *pcCU, UInt uiAbsPartIdx,
                                     Int xStartInCU, Int yStartInCU, Int xBv, Int yBv, Int ctuSize )
{
  static const Int s_floorLog2[65] =
  {
    -1, 0, 1, 1, 2, 2, 2, 2, 3, 3,
     3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
     4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
     4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
     5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
     5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
     5, 5, 5, 5, 6
  };

  Int ctuSizeLog2 = s_floorLog2[ctuSize];

#if SCM_IBC_CR_INTERPOLATION_ENABLE
  Int interpolationSamplesX = (pcCU->getPic()->getChromaFormat() == CHROMA_422 || pcCU->getPic()->getChromaFormat() == CHROMA_420) ?((xBv&0x1)<< 1) : 0;
  Int interpolationSamplesY = (pcCU->getPic()->getChromaFormat() == CHROMA_420) ? ((yBv&0x1)<< 1) : 0;
  Int refRightX  = xPos + xBv + width - 1 + interpolationSamplesX;
  Int refBottomY = yPos + yBv + height - 1 + interpolationSamplesY;  
#else
  Int refRightX = xPos + xBv + width - 1;   // including
  Int refBottomY = yPos + yBv + height - 1; // including
#endif 
  Int picWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  Int picHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

#if SCM_IBC_CR_INTERPOLATION_ENABLE
  if ( (xPos + xBv - interpolationSamplesX) < 0 )
#else
  if ( xPos + xBv < 0 )
#endif 
  {
    return false;
  }
  if ( refRightX >= picWidth )
  {
    return false;
  }
#if SCM_IBC_CR_INTERPOLATION_ENABLE
  if ( (yPos + yBv - interpolationSamplesY) < 0 )
#else
  if ( yPos + yBv < 0 )
#endif 
  {
    return false;
  }
  if ( refBottomY >= picHeight )
  {
    return false;
  }

#if SCM_IBC_CR_INTERPOLATION_ENABLE
  if ( (xBv + width + interpolationSamplesX) > 0 && (yBv + height+interpolationSamplesY) > 0 )
#else
  if ( xBv + width > 0 && yBv + height > 0 )
#endif 
  {
    return false;
  }

  TComSlice *pcSlice = pcCU->getSlice();
  if( pcSlice->getSliceMode() )
  {
    TComPicSym *pcSym = pcCU->getPic()->getPicSym();
    Int      ctuX = (xPos + xBv) / ctuSize;
    Int      ctuY = (yPos + yBv) / ctuSize;
    UInt   refCtu = ctuX + pcSym->getFrameWidthInCtus()*ctuY;
    UInt startCtu = /*pcCU->getSlice()->getSliceSegmentCurStartCtuTsAddr();*/ pcSym->getCtuTsToRsAddrMap( pcCU->getSlice()->getSliceSegmentCurStartCtuTsAddr() );
    if (refCtu < startCtu) return false;
  }

  if ( refBottomY>>ctuSizeLog2 < yPos>>ctuSizeLog2 )
  {
    Int uiRefCuX   = refRightX/ctuSize;
    Int uiRefCuY   = refBottomY/ctuSize;
    Int uiCuPelX   = xPos / ctuSize;
    Int uiCuPelY   = yPos / ctuSize;

    if(((Int)(uiRefCuX - uiCuPelX) > (Int)((uiCuPelY - uiRefCuY))))
    {
      return false;
    }
    else
    {
#if SCM_FIX_TICKET_1401
      if(!pcCU->getSlice()->getPPS()->getConstrainedIntraPred() || xCIPIBCSearchPruning(pcCU, xPos + xBv, yPos + yBv, width, height))
      {
        return true;
      }
      else
      {
        return false;
      }
#else
      return true;
#endif
    }
  }

  if ( refBottomY>>ctuSizeLog2 > yPos>>ctuSizeLog2 )
  {
    return false;
  }

  // in the same CTU line
  if ( refRightX>>ctuSizeLog2 < xPos>>ctuSizeLog2 )
  {
#if SCM_FIX_TICKET_1401
    if(!pcCU->getSlice()->getPPS()->getConstrainedIntraPred() || xCIPIBCSearchPruning(pcCU, xPos + xBv, yPos + yBv, width, height))
    {
      return true;
    }
    else
    {
      return false;
    }
#else
    return true;
#endif
  }
  if ( refRightX>>ctuSizeLog2 > xPos>>ctuSizeLog2 )
  {
    return false;
  }
  
  // same CTU
  Int mask = 1<<ctuSizeLog2;
  mask -= 1;
  Int rasterCurr = ( ( ((yPos&mask) - yStartInCU)>>2 ) << (ctuSizeLog2-2) ) + ( ((xPos&mask) - xStartInCU)>>2 );
  Int rasterRef  = ( ( (refBottomY&mask)>>2 ) << (ctuSizeLog2-2) ) + ( (refRightX&mask)>>2 );

  if ( g_auiRasterToZscan[rasterRef] >= g_auiRasterToZscan[rasterCurr] )
  {
    return false;
  }

#if SCM_FIX_TICKET_1401
  if(!pcCU->getSlice()->getPPS()->getConstrainedIntraPred() || xCIPIBCSearchPruning(pcCU, xPos + xBv, yPos + yBv, width, height))
  {
    return true;
  }
  else
  {
    return false;
  }
#else
  return true;
#endif
}

// based on predInterSearch()
Bool TEncSearch::predIntraBCSearch( TComDataCU * pcCU,
                                    TComYuv    * pcOrgYuv,
                                    TComYuv    *&rpcPredYuv,
                                    TComYuv    *&rpcResiYuv,
                                    TComYuv    *&rpcRecoYuv
                                    DEBUG_STRING_FN_DECLARE(sDebug),
                                    Bool         bUse1DSearchFor8x8,
                                    Bool         bUseRes
                                  , Bool         testOnlyPred
                                    )
{
  rpcPredYuv->clear();
  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }
  rpcRecoYuv->clear();

  PartSize     ePartSize  = pcCU->getPartitionSize( 0 );

#if SCM_S0067_MAX_CAND_SIZE
  if (m_pcEncCfg->getUseIntraBlockCopyFastSearch() && (pcCU->getWidth(0) > SCM_S0067_MAX_CAND_SIZE))
#else
  if (m_pcEncCfg->getUseIntraBlockCopyFastSearch() && (pcCU->getWidth(0) > 16))
#endif
    return false;

  const Int iNumPart = pcCU->getNumPartitions();
  Distortion uiTotalCost = 0;
  for( Int iPartIdx = 0; iPartIdx < iNumPart; ++iPartIdx )
  {
    Int width, height;
    UInt uiPartAddr = 0;
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, width, height );

    TComMvField cMEMvField;
    Distortion  uiCost;

    TComMv      cMv, cMvd, cMvPred[2];
    AMVPInfo currAMVPInfo;
    pcCU->fillMvpCand( iPartIdx, uiPartAddr, REF_PIC_LIST_0, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, &currAMVPInfo );
    cMvPred[0].set( currAMVPInfo.m_acMvCand[0].getHor() >> 2, currAMVPInfo.m_acMvCand[0].getVer() >> 2);
    cMvPred[1].set( currAMVPInfo.m_acMvCand[1].getHor() >> 2, currAMVPInfo.m_acMvCand[1].getVer() >> 2);

    xIntraBlockCopyEstimation ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, uiCost, bUse1DSearchFor8x8, testOnlyPred );

    if( m_pcEncCfg->getUseHashBasedIntraBCSearch()
      && pcCU->getWidth(0) == 8
      && !testOnlyPred
      && ePartSize == SIZE_2Nx2N
      && (m_pcEncCfg->getUseHashBasedIntraBCSearch()
        || pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCSearchWidthInCTUs() ) != pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() ))
      )
    {
      Distortion uiIntraBCECost = uiCost;
      xIntraBCHashSearch ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, (UInt)uiIntraBCECost);
      uiCost = std::min(uiIntraBCECost, uiCost);
    }
    uiTotalCost += uiCost;
    // choose one MVP and compare with merge mode
    // no valid intra BV
    if ( cMv.getHor() == 0 && cMv.getVer() == 0 )
    {
      if( testOnlyPred ) m_lastCandCost = MAX_UINT;
      return false;
    }

    m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( 0 ) );
    m_pcRdCost->setCostScale( 0 );

    UInt uiDepth = pcCU->getDepth( 0 );
    Int bitsAMVPBest, bitsAMVPTemp,                bitsMergeTemp;
    Int distAMVPBest,                              distMergeTemp;
    Int costAMVPBest,               costMergeBest, costMergeTemp;
    bitsAMVPBest = MAX_INT;
    costAMVPBest = MAX_INT;
    costMergeBest = MAX_INT;
    Int mvpIdxBest = 0;
    Int mvpIdxTemp;
    Int mrgIdxBest = -1;
    Int mrgIdxTemp = -1;
    Int xCUStart = pcCU->getCUPelX();
    Int yCUStart = pcCU->getCUPelY();
    Int xStartInCU, yStartInCU;
    pcCU->getStartPosition( iPartIdx, xStartInCU, yStartInCU );
#if SCM_IBC_CR_INTERPOLATION_ENABLE==0
    Int xStartInPic = xCUStart+xStartInCU;
    Int yStartInPic = yCUStart+yStartInCU;
#endif 
    Pel* pCurrStart;
    Pel* pCurr;
#if SCM_IBC_CR_INTERPOLATION_ENABLE==0
    Pel* pRefStart;
#endif 
    Pel* pRef;
    Int currStride, refStride;
    distAMVPBest = 0;

#if SCM_IBC_CR_INTERPOLATION_ENABLE
    TComMv cMvQuaterPixl = cMv;
    cMvQuaterPixl <<= 2;
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMvQuaterPixl, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
    pcCU->setInterDirSubParts( 1 + REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
    motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );    
#endif

    for (UInt ch = COMPONENT_Y; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      Int iTempHeight = height >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));
      Int iTempWidth = width >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));

      pCurrStart = pcOrgYuv->getAddr  ( ComponentID(ch), uiPartAddr );
      currStride = pcOrgYuv->getStride( ComponentID(ch) );
      pCurr = pCurrStart;
#if SCM_IBC_CR_INTERPOLATION_ENABLE 
      ComponentID compID = (ComponentID)ch;
      pRef = m_tmpYuvPred.getAddr( compID, uiPartAddr);
      refStride = m_tmpYuvPred.getStride(compID);
#else
      pRefStart = pcCU->getPic()->getPicYuvRec()->getAddr( ComponentID(ch) );
      refStride = pcCU->getPic()->getPicYuvRec()->getStride( ComponentID(ch) );
      pRef = pRefStart + ((yStartInPic+ cMv.getVer())>> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch)))*refStride + ((xStartInPic + cMv.getHor())>> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch)));
#endif 
      distAMVPBest += getSAD( pRef, refStride, pCurr, currStride, iTempWidth, iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );
    }

    cMvPred[0].setHor( currAMVPInfo.m_acMvCand[0].getHor() >> 2);
    cMvPred[0].setVer( currAMVPInfo.m_acMvCand[0].getVer() >> 2);
    cMvPred[1].setHor( currAMVPInfo.m_acMvCand[1].getHor() >> 2);
    cMvPred[1].setVer( currAMVPInfo.m_acMvCand[1].getVer() >> 2);

    for ( mvpIdxTemp=0; mvpIdxTemp<currAMVPInfo.iN; mvpIdxTemp++ )
    {
      m_pcRdCost->setPredictor( cMvPred[mvpIdxTemp] );
      bitsAMVPTemp = m_pcRdCost->getBits( cMv.getHor(), cMv.getVer() );
      if ( bitsAMVPTemp < bitsAMVPBest )
      {
        bitsAMVPBest = bitsAMVPTemp;
        mvpIdxBest = mvpIdxTemp;
      }
    }
    bitsAMVPBest++; // for MVP Index bits
    costAMVPBest = distAMVPBest + m_pcRdCost->getCost( bitsAMVPBest );

    TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;

    if ( ePartSize != SIZE_2Nx2N )
    {
      if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && ePartSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
      {
        pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
        if ( iPartIdx == 0 )
        {
          pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
        }
        pcCU->setPartSizeSubParts( ePartSize, 0, uiDepth );
      }
      else
      {
        pcCU->getInterMergeCandidates( uiPartAddr, iPartIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
      }

      xRestrictBipredMergeCand( pcCU, iPartIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );

      for ( mrgIdxTemp = 0; mrgIdxTemp < numValidMergeCand; mrgIdxTemp++ )
      {
        if ( uhInterDirNeighbours[mrgIdxTemp] != 1 )
        {
          continue;
        }

        if ( pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, cMvFieldNeighbours[mrgIdxTemp<<1].getRefIdx() )->getPOC() != pcCU->getSlice()->getPOC() )
        {
          continue;
        }

        if ( !isBlockVectorValid( xCUStart+xStartInCU, yCUStart+yStartInCU, width, height, pcCU, uiPartAddr,
          xStartInCU, yStartInCU, (cMvFieldNeighbours[mrgIdxTemp<<1].getHor() >> 2), (cMvFieldNeighbours[mrgIdxTemp<<1].getVer()>>2), pcCU->getSlice()->getSPS()->getMaxCUWidth() ) )
        {
          continue;
        }
        bitsMergeTemp = mrgIdxTemp == m_pcEncCfg->getMaxNumMergeCand() ? mrgIdxTemp : mrgIdxTemp+1;

        distMergeTemp = 0;

#if SCM_IBC_CR_INTERPOLATION_ENABLE
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMvFieldNeighbours[mrgIdxTemp<<1].getMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->setInterDirSubParts( 1 + REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
        motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );
#endif

        for (UInt ch = COMPONENT_Y; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
        {
          Int iTempHeight = height >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));
          Int iTempWidth = width >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));

          pCurrStart = pcOrgYuv->getAddr  ( ComponentID(ch), uiPartAddr );
          currStride = pcOrgYuv->getStride( ComponentID(ch) );
          pCurr = pCurrStart;

#if SCM_IBC_CR_INTERPOLATION_ENABLE
          ComponentID compID = (ComponentID)ch;
          pRef = m_tmpYuvPred.getAddr( compID, uiPartAddr);
          refStride = m_tmpYuvPred.getStride(compID);
#else
          pRefStart = pcCU->getPic()->getPicYuvRec()->getAddr( ComponentID(ch) );
          refStride = pcCU->getPic()->getPicYuvRec()->getStride( ComponentID(ch) );
          pRef = pRefStart + ((yStartInPic+ (cMvFieldNeighbours[mrgIdxTemp<<1].getVer()>>2))>> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch)))*refStride + ((xStartInPic+ (cMvFieldNeighbours[mrgIdxTemp<<1].getHor()>>2))>>pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch)));      
#endif 
          distMergeTemp += getSAD( pRef, refStride, pCurr, currStride, iTempWidth, iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );
        }
        costMergeTemp = distMergeTemp + m_pcRdCost->getCost( bitsMergeTemp );

        if ( costMergeTemp < costMergeBest )
        {
          costMergeBest = costMergeTemp;
          mrgIdxBest = mrgIdxTemp;
        }
      }
    }

    if ( costAMVPBest < costMergeBest )
    {
      TComMv zeroMv;
      TComMv mv( (cMv.getHor()<<2), (cMv.getVer()<<2) );
      TComMvField mvField[2];
      mvField[0].setMvField( mv, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1 );   // the current picture is at the last position of list0
      mvField[1].setMvField( zeroMv, -1 );

      pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( mvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( mvField[1], ePartSize, uiPartAddr, 0, iPartIdx );

      TComMv mvd( cMv.getHor() - (currAMVPInfo.m_acMvCand[mvpIdxBest].getHor() >> 2), cMv.getVer() - (currAMVPInfo.m_acMvCand[mvpIdxBest].getVer()>>2) );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( mvd,    ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( zeroMv, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( mvpIdxBest, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
    }
    else
    {
      TComMv zeroMv;
      TComMv mv( cMvFieldNeighbours[mrgIdxBest<<1].getHor(), cMvFieldNeighbours[mrgIdxBest<<1].getVer() );
      TComMvField mvField[2];
      mvField[0].setMvField( mv, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1 );   // the current picture is at the last position of list0
      mvField[1].setMvField( zeroMv, -1 );

      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeIndexSubParts( mrgIdxBest, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( mvField[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( mvField[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( zeroMv, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( zeroMv, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
    }

  }

  Distortion abortThreshold = pcCU->getWidth(0)*pcCU->getHeight(0)*2;
  if( testOnlyPred )
  {
    if( iNumPart==1 && uiTotalCost > abortThreshold )
    {
      m_lastCandCost = MAX_UINT;
      return false;
    }
    m_lastCandCost = uiTotalCost;
  }
  else if( uiTotalCost < abortThreshold && 3*uiTotalCost>>2 >= m_lastCandCost )
  {
    return false;
  }

  // motion compensation
  if( !pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() || !m_pcEncCfg->getRGBFormatFlag() || (pcCU->getCUTransquantBypass(0) && (pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) != pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA))) )
  {
    motionCompensation ( pcCU, rpcPredYuv );
  }
  return true;
}

// based on predInterSearch()
Bool TEncSearch::predMixedIntraBCInterSearch( TComDataCU * pcCU,
                                              TComYuv    * pcOrgYuv,
                                              TComYuv    *&rpcPredYuv,
                                              TComYuv    *&rpcResiYuv,
                                              TComYuv    *&rpcRecoYuv
                                              DEBUG_STRING_FN_DECLARE( sDebug ),
                                              TComMv*      iMvCandList,
                                              Bool         bUseRes
                                              )
{
  rpcPredYuv->clear();
  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }
  rpcRecoYuv->clear();

  PartSize     ePartSize  = pcCU->getPartitionSize( 0 );

  UInt uiDepth = pcCU->getDepth( 0 );
  Int iNumComb = 2;
  Int iNumPart = 2;
  //  PredMode pMixedMode[2][2] = {{MODE_INTRABC, MODE_INTER},{MODE_INTER, MODE_INTRABC}};
  Distortion  uiCost[2]={ 0, 0 };
  Distortion uiMaxCost = std::numeric_limits<Distortion>::max();

  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

  TComMvField cMvFieldNeighbours[2][MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
  UChar uhInterDirNeighbours[2][MRG_MAX_NUM_CANDS];
  Int numValidMergeCand[2] ={ MRG_MAX_NUM_CANDS, MRG_MAX_NUM_CANDS };
  Int iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;
  TComMv       cMvZero( 0, 0 );

  TComMv  cMvPredCand[2][2];
  Int iIBCValidFlag = 0;
  Int iBestIBCMvpIdx[2] ={ 0, 0 };
  Int iBestInterMvpIdx[2] ={ 0, 0 };
  Int iBestInterDir[2] ={ 0, 0 };
  Int iBestRefIdx[2] ={ 0, 0 };
  bool isMergeMode[2]={ false, false };
  bool isIBCMergeMode[2]={ false, false };
  TComMvField cMRGMvField[2][2];
  TComMvField cMRGMvFieldIBC[2][2];

  for ( Int iCombo = 0; iCombo < iNumComb; iCombo++ ) // number of combination
  {

    for ( Int iPartIdx = 0; iPartIdx < iNumPart; ++iPartIdx )
    {
      Int iDummyWidth, iDummyHeight;
      UInt uiPartAddr = 0;
      pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth, iDummyHeight );

      //  Search key pattern initialization
      pcPatternKey->initPattern( pcOrgYuv->getAddr( COMPONENT_Y, uiPartAddr ), iDummyWidth, iDummyHeight, pcOrgYuv->getStride( COMPONENT_Y ), pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

      // disable weighted prediction
      setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );


      TComMv mvPred[2]; // put result of AMVP into mvPred
      TComMv bvPred[2]; // put result of BVP into bvPred
      Distortion   biPDistTemp = std::numeric_limits<Distortion>::max();
      if ( (iCombo==0 && iPartIdx==0) || (iCombo==1 && iPartIdx==1) ) // intraBC
      {
        TComMv cMv = iMvCandList[8+iPartIdx];
        if ( cMv.getHor() == 0 && cMv.getVer() == 0 )
        {
          uiCost[iCombo] = uiMaxCost;
          iIBCValidFlag++;
          break;
        }
        m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( uiPartAddr ) );
        m_pcRdCost->setCostScale( 0 );
        //Pel*        piRefY      = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
        //Int         iRefStride  = pcCU->getPic()->getPicYuvRec()->getStride( COMPONENT_Y );
        //  xEstimateMvPredAMVPIBC( pcCU, pcOrgYuv, iPartIdx, REF_PIC_LIST_INTRABC, 0, mvPred, false, &biPDistTemp);
        AMVPInfo currAMVPInfo;
        //  pcCU->fillMvpCand( iPartIdx, uiPartAddr, REF_PIC_LIST_0, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, &currAMVPInfo );
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, REF_PIC_LIST_0, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, bvPred[0], false, &biPDistTemp );
        //  Int iMvpIdx;// = pcCU->getMVPIdx(eRefPicList, uiPartAddr);

        bvPred[0] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getAMVPInfo()->m_acMvCand[0];
        bvPred[1] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getAMVPInfo()->m_acMvCand[1];
        bvPred[0] >>= 2;
        bvPred[1] >>= 2;

        /////////////////////////////////////////////////////////////
        // ibc merge
        // choose one MVP and compare with merge mode

        m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( 0 ) );
        m_pcRdCost->setCostScale( 0 );


        //  UInt uiDepth = pcCU->getDepth( 0 );
        Int bitsAMVPBest, bitsAMVPTemp, bitsMergeTemp;
        Int distAMVPBest, distMergeTemp;
        Int costAMVPBest, costMergeBest, costMergeTemp;
        bitsAMVPBest = MAX_INT;
        costAMVPBest = MAX_INT;
        costMergeBest = MAX_INT;
        Int mvpIdxBest = 0;
        Int mvpIdxTemp;
        Int mrgIdxBest = -1;
        Int mrgIdxTemp = -1;
        Int xCUStart = pcCU->getCUPelX();
        Int yCUStart = pcCU->getCUPelY();
        Int xStartInCU, yStartInCU;
        pcCU->getStartPosition( iPartIdx, xStartInCU, yStartInCU );
#if SCM_IBC_CR_INTERPOLATION_ENABLE==0
        Int xStartInPic = xCUStart+xStartInCU;
        Int yStartInPic = yCUStart+yStartInCU;
#endif 
        Pel* pCurrStart;
        Int currStride;
        Pel* pCurr;
#if SCM_IBC_CR_INTERPOLATION_ENABLE==0
        Pel* pRefStart;
#endif 
        Int refStride;
        Pel* pRef;
        distAMVPBest = 0;

#if SCM_IBC_CR_INTERPOLATION_ENABLE
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMv, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
        pcCU->setInterDirSubParts( 1 + REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
        motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );
#endif

        for (UInt ch = COMPONENT_Y; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
        {
          Int iTempWidth = iDummyWidth >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));
          Int iTempHeight = iDummyHeight >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));

          pCurrStart = pcOrgYuv->getAddr( ComponentID(ch), uiPartAddr);
          currStride = pcOrgYuv->getStride( ComponentID(ch) );
          pCurr = pCurrStart;

#if SCM_IBC_CR_INTERPOLATION_ENABLE
          ComponentID compID = (ComponentID)ch;
          pRef = m_tmpYuvPred.getAddr( compID, uiPartAddr);
          refStride = m_tmpYuvPred.getStride(compID);
#else
          pRefStart = pcCU->getPic()->getPicYuvRec()->getAddr( ComponentID(ch) );
          refStride = pcCU->getPic()->getPicYuvRec()->getStride( ComponentID(ch) );
          pRef = pRefStart + ((yStartInPic+(cMv.getVer()>>2))>>pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch)))*refStride + ((xStartInPic+(cMv.getHor()>>2)) >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch)));
#endif 
          distAMVPBest += getSAD( pRef, refStride, pCurr, currStride, iTempWidth, iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );
        }

        for ( mvpIdxTemp=0; mvpIdxTemp<AMVP_MAX_NUM_CANDS; mvpIdxTemp++ )
        {
          m_pcRdCost->setPredictor( bvPred[mvpIdxTemp] );
          bitsAMVPTemp = m_pcRdCost->getBits( (cMv.getHor() >> 2), (cMv.getVer() >> 2) );
          if ( bitsAMVPTemp < bitsAMVPBest )
          {
            bitsAMVPBest = bitsAMVPTemp;
            mvpIdxBest = mvpIdxTemp;
          }
        }


        bitsAMVPBest++; // for MVP Index bits
        costAMVPBest = distAMVPBest + m_pcRdCost->getCost( bitsAMVPBest );

        TComMvField cMvFieldNeighboursIBC[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
        UChar uhInterDirNeighboursIBC[MRG_MAX_NUM_CANDS];
        Int numValidMergeCandIBC = 0;

        if ( ePartSize != SIZE_2Nx2N )
        {
          if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && ePartSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
          {
            pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
            if ( iPartIdx == 0 )
            {
              pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighboursIBC, uhInterDirNeighboursIBC, numValidMergeCandIBC );
            }
            pcCU->setPartSizeSubParts( ePartSize, 0, uiDepth );
          }
          else
          {
            pcCU->getInterMergeCandidates( uiPartAddr, iPartIdx, cMvFieldNeighboursIBC, uhInterDirNeighboursIBC, numValidMergeCandIBC );
          }

          xRestrictBipredMergeCand( pcCU, iPartIdx, cMvFieldNeighboursIBC, uhInterDirNeighboursIBC, numValidMergeCandIBC );

          for ( mrgIdxTemp = 0; mrgIdxTemp < numValidMergeCandIBC; mrgIdxTemp++ )
          {
            if ( uhInterDirNeighboursIBC[mrgIdxTemp] != 1 )
            {
              continue;
            }

            if ( pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, cMvFieldNeighboursIBC[mrgIdxTemp<<1].getRefIdx() )->getPOC() != pcCU->getSlice()->getPOC() )
            {
              continue;
            }

            if ( !isBlockVectorValid( xCUStart+xStartInCU, yCUStart+yStartInCU, iDummyWidth, iDummyHeight, pcCU, uiPartAddr,
              xStartInCU, yStartInCU, (cMvFieldNeighboursIBC[mrgIdxTemp<<1].getHor() >> 2), (cMvFieldNeighboursIBC[mrgIdxTemp<<1].getVer() >> 2), pcCU->getSlice()->getSPS()->getMaxCUWidth() ) )
            {
              continue;
            }

            bitsMergeTemp = mrgIdxTemp == m_pcEncCfg->getMaxNumMergeCand() ? mrgIdxTemp : mrgIdxTemp+1;

            distMergeTemp = 0;
#if SCM_IBC_CR_INTERPOLATION_ENABLE
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMvFieldNeighboursIBC[mrgIdxTemp<<1].getMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(), pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, pcCU->getPartitionSize(0), uiPartAddr, 0, iPartIdx );
            pcCU->setInterDirSubParts( 1 + REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(0) );
            motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );
#endif
            for (UInt ch = COMPONENT_Y; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
            {          
              Int iTempWidth = iDummyWidth >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));
              Int iTempHeight = iDummyHeight >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));

              pCurrStart = pcOrgYuv->getAddr( ComponentID(ch), uiPartAddr);
              currStride = pcOrgYuv->getStride( ComponentID(ch) );
              pCurr = pCurrStart;
#if SCM_IBC_CR_INTERPOLATION_ENABLE
              ComponentID compID = (ComponentID)ch;
              pRef = m_tmpYuvPred.getAddr( compID, uiPartAddr);
              refStride = m_tmpYuvPred.getStride(compID);
#else
              pRefStart = pcCU->getPic()->getPicYuvRec()->getAddr( ComponentID(ch) );
              refStride = pcCU->getPic()->getPicYuvRec()->getStride( ComponentID(ch) );
              pRef = pRefStart + ((yStartInPic+(cMvFieldNeighboursIBC[mrgIdxTemp<<1].getVer() >> 2))>>pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch)))*refStride + ((xStartInPic+(cMvFieldNeighboursIBC[mrgIdxTemp<<1].getHor() >> 2))>>pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch)));
#endif 
              distMergeTemp += getSAD( pRef, refStride, pCurr, currStride, iTempWidth , iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );
            }

            costMergeTemp = distMergeTemp + m_pcRdCost->getCost( bitsMergeTemp );

            if ( costMergeTemp < costMergeBest )
            {
              costMergeBest = costMergeTemp;
              mrgIdxBest = mrgIdxTemp;

            }
          }
        }

        if ( costMergeBest < costAMVPBest )
        {
          uiCost[iCombo] += costMergeBest;
          isIBCMergeMode[iCombo] = true;
          iBestIBCMvpIdx[iCombo] = mrgIdxBest;

          TComMvField mvField[2];
          TComMv mv( cMvFieldNeighboursIBC[mrgIdxBest<<1].getHor(), cMvFieldNeighboursIBC[mrgIdxBest<<1].getVer() );
          mvField[0].setMvField( mv, pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1 );   // the current picture is at the last position of list0
          mvField[1].setMvField( cMvZero, -1 );
          cMRGMvFieldIBC[iCombo][0] = mvField[0];
          cMRGMvFieldIBC[iCombo][1] = mvField[1];
        }
        else
        {
          uiCost[iCombo] += costAMVPBest;
          isIBCMergeMode[iCombo] = false;
          iBestIBCMvpIdx[iCombo] = mvpIdxBest;
          cMvPredCand[iCombo][iPartIdx].setHor( bvPred[mvpIdxBest].getHor() << 2);
          cMvPredCand[iCombo][iPartIdx].setVer( bvPred[mvpIdxBest].getVer() << 2);
        }

        pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
        if ( isIBCMergeMode[iCombo] )
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMRGMvFieldIBC[iCombo][0].getMv(), ePartSize, uiPartAddr, 0, iPartIdx );
        else
        {
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( iMvCandList[8+iPartIdx], ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
        }
        // ibc merge
        /////////////////////////////////////////////////////////////
      }
      else // is inter PU
      {
        Distortion  uiCostInterTemp = 0;
        Distortion  uiCostInterBest = std::numeric_limits<Distortion>::max();
        m_pcRdCost->setCostScale( 0 );
        //  Uni-directional prediction
        //  Int iNumPredDir = 1;
        for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
        {
          RefPicList  eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
          //  UInt uiNumRef = (pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 > 1) ? 2: 1;
          UInt uiNumRef = iRefList ? ((pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 1) ? 2 : 1) : ((pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 )-1 > 1) ? 2 : 1);
          for ( Int iRefIdx = 0; iRefIdx < uiNumRef; iRefIdx++ )
          {
            TComMv cMv = iMvCandList[4*iRefList + 2*iRefIdx + iPartIdx];

            xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdx, mvPred[0], false, &biPDistTemp );
            Int iMvpIdx;// = pcCU->getMVPIdx(eRefPicList, uiPartAddr);

            Distortion  uiTempCost0 = 0;
            Distortion  uiTempCost1 = 0;
            mvPred[0] = pcCU->getCUMvField( eRefPicList )->getAMVPInfo()->m_acMvCand[0];
            mvPred[1] = pcCU->getCUMvField( eRefPicList )->getAMVPInfo()->m_acMvCand[1];

            m_pcRdCost->setPredictor( mvPred[0] );
            uiTempCost0 = m_pcRdCost->getCost( cMv.getHor(), cMv.getVer() );
            m_pcRdCost->setPredictor( mvPred[1] );
            uiTempCost1 = m_pcRdCost->getCost( cMv.getHor(), cMv.getVer() );
            if ( uiTempCost1 < uiTempCost0 )
            {
              iMvpIdx = 1;
            }
            else
            {
              iMvpIdx = 0;
            }

            UInt uiBitsTemp = m_auiMVPIdxCost[iMvpIdx][AMVP_MAX_NUM_CANDS];

            m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( uiPartAddr ) );
            m_pcRdCost->setPredictor( mvPred[iMvpIdx] );
#if SCM_AMVR_UNIFICATION 
            if( pcCU->getSlice()->getUseIntegerMv() )
            {
              pcCU->getCUMvField( eRefPicList )->setAllMv( (cMv>>2)<<2, ePartSize, uiPartAddr, 0, iPartIdx );
            }
            else
            {
              pcCU->getCUMvField( eRefPicList )->setAllMv( cMv, ePartSize, uiPartAddr, 0, iPartIdx );
            }
#else
            pcCU->getCUMvField( eRefPicList )->setAllMv( cMv, ePartSize, uiPartAddr, 0, iPartIdx );
#endif 
            pcCU->getCUMvField( eRefPicList )->setAllRefIdx( iRefIdx, ePartSize, uiPartAddr, 0, iPartIdx );
            pcCU->setInterDirSubParts( 1 + iRefList, uiPartAddr, iPartIdx, uiDepth );

#if SCM_T0227_INTER_SEARCH_YUV
            motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );
            uiCostInterTemp = 0;
#if SCM_T0227_INTER_SEARCH_YUV==1
            for (Int ch = COMPONENT_Y; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
#else
            for (Int ch = COMPONENT_Y; ch < (COMPONENT_Y+1); ch++)
#endif
            {
              Int iTempWidth = iDummyWidth >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(ComponentID(ch));
              Int iTempHeight = iDummyHeight >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(ComponentID(ch));

              Int iRefStride = m_tmpYuvPred.getStride(ComponentID(ch));
              Int iOrgStride = pcOrgYuv->getStride(ComponentID(ch));
              Pel *pRef =  m_tmpYuvPred.getAddr( ComponentID(ch), uiPartAddr);
              Pel *pOrg = pcOrgYuv->getAddr( ComponentID(ch), uiPartAddr);
              //calculate the dist;
              uiCostInterTemp += getSAD( pRef, iRefStride, pOrg, iOrgStride, iTempWidth, iTempHeight, pcCU->getSlice()->getSPS()->getBitDepths() );

              if(uiCostInterTemp >= uiCostInterBest)
                break;
            }
#else
            xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiCostInterTemp, m_pcEncCfg->getUseHADME() );
#endif
            pcCU->getCUMvField( eRefPicList )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
            //  m_pcRdCost->setCostScale  ( 2 );
            uiCostInterTemp += m_pcRdCost->getCost( cMv.getHor(), cMv.getVer() );
            uiCostInterTemp += m_pcRdCost->getCost( uiBitsTemp );

            if ( uiCostInterTemp < uiCostInterBest )
            {
              uiCostInterBest = uiCostInterTemp;
              iBestInterMvpIdx[iCombo] = iMvpIdx;
              iBestInterDir[iCombo] = iRefList;
              iBestRefIdx[iCombo] = iRefIdx;
              cMvPredCand[iCombo][iPartIdx] = mvPred[iMvpIdx];
            }
          }
        } // end RefIdx and RefList search

        UInt uiMRGInterDir = 0;
        UInt uiMRGIndex = 0;

        // find Merge result
        Distortion uiMRGCost = std::numeric_limits<Distortion>::max();
#if SCM_AMVR_UNIFICATION
        pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
#endif

#if SCM_T0227_INTER_SEARCH_YUV==1
        xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField[iCombo], uiMRGIndex, uiMRGCost, cMvFieldNeighbours[iCombo], uhInterDirNeighbours[iCombo], numValidMergeCand[iCombo], 1 );
#elif SCM_T0227_INTER_SEARCH_YUV==2
        xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField[iCombo], uiMRGIndex, uiMRGCost, cMvFieldNeighbours[iCombo], uhInterDirNeighbours[iCombo], numValidMergeCand[iCombo], 2 );
#else
        xMergeEstimation( pcCU, pcOrgYuv, iPartIdx, uiMRGInterDir, cMRGMvField[iCombo], uiMRGIndex, uiMRGCost, cMvFieldNeighbours[iCombo], uhInterDirNeighbours[iCombo], numValidMergeCand[iCombo] );
#endif
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );

        if ( uiMRGCost < uiCostInterBest )

        {
          uiCostInterBest = uiMRGCost;
          isMergeMode[iCombo] = true;
          iBestInterMvpIdx[iCombo] = uiMRGIndex;
          iBestInterDir[iCombo]= uiMRGInterDir;

        }

        uiCost[iCombo] += uiCostInterBest;
        if ( isMergeMode[iCombo] )
        {
          pcCU->setInterDirSubParts( iBestInterDir[iCombo], uiPartAddr, iPartIdx, uiDepth );
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[iCombo][0], ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[iCombo][1], ePartSize, uiPartAddr, 0, iPartIdx );
        }
        else
        {
          Int iRefListOpt = iBestInterDir[iCombo];
          Int iRefIdxOpt = iBestRefIdx[iCombo];
          RefPicList  eRefPicListOpt = (iRefListOpt ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
#if SCM_AMVR_UNIFICATION
          if( pcCU->getSlice()->getUseIntegerMv() )
          {            
            pcCU->getCUMvField( eRefPicListOpt )->setAllMv( (iMvCandList[iPartIdx + 2*iRefIdxOpt + 4*iRefListOpt]>>2)<<2, ePartSize, uiPartAddr, 0, iPartIdx );            
          }
          else
          {
            pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[iPartIdx + 2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
          }
#else
          pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[iPartIdx + 2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
#endif 
          pcCU->getCUMvField( eRefPicListOpt )->setAllRefIdx( iRefIdxOpt, ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->getCUMvField( (RefPicList)(1-iRefListOpt) )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
          pcCU->setInterDirSubParts( 1 + iRefListOpt, uiPartAddr, iPartIdx, uiDepth );
          pcCU->setMVPIdxSubParts( iBestInterMvpIdx[iCombo], eRefPicListOpt, uiPartAddr, iPartIdx, uiDepth );
        }


      }
    } // for ipartIdx
  } // for iCombo

  if ( iIBCValidFlag > 1 )
    return false;

  TComMv cMvd;
  TComMv cMVFinal;
  if ( uiCost[0] <= uiCost[1] )

  {
    Int iDummyWidth1, iDummyHeight1;
    UInt uiPartAddr = 0;
    UInt iPartIdx = 0;
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth1, iDummyHeight1 );

    if ( isIBCMergeMode[0] )
    {

      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeIndexSubParts( iBestIBCMvpIdx[0], uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvFieldIBC[0][0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvFieldIBC[0][1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );

    }
    else
    {
      pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, uiDepth );
      cMvd.setHor( (iMvCandList[8].getHor() - cMvPredCand[0][0].getHor()) >> 2 );
      cMvd.setVer( (iMvCandList[8].getVer() - cMvPredCand[0][0].getVer()) >> 2 );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( iMvCandList[8], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( iBestIBCMvpIdx[0], REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
    }
    iPartIdx = 1;
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth1, iDummyHeight1 );


    if ( isMergeMode[0] )
    {
      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeIndexSubParts( iBestInterMvpIdx[0], uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( iBestInterDir[0], uiPartAddr, iPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[0][0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[0][1], ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
    }
    else
    {
      Int iRefListOpt = iBestInterDir[0];
      Int iRefIdxOpt = iBestRefIdx[0];
      RefPicList  eRefPicListOpt = (iRefListOpt ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
#if SCM_AMVR_UNIFICATION            
      if( pcCU->getSlice()->getUseIntegerMv() )
      {            
        cMvd.setHor( (iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getHor() >> 2) - (cMvPredCand[0][1].getHor()>>2) );
        cMvd.setVer( (iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getVer() >> 2) - (cMvPredCand[0][1].getVer()>>2) );
        pcCU->getCUMvField( eRefPicListOpt )->setAllMv( (iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt]>>2)<<2, ePartSize, uiPartAddr, 0, iPartIdx );
      }
      else
      {
        cMvd.setHor( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getHor() - cMvPredCand[0][1].getHor() );
        cMvd.setVer( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getVer() - cMvPredCand[0][1].getVer() );
        pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
      }
#else
      cMvd.setHor( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getHor() - cMvPredCand[0][1].getHor() );
      cMvd.setVer( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt].getVer() - cMvPredCand[0][1].getVer() );
      pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[1 + 2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
#endif 
      pcCU->getCUMvField( eRefPicListOpt )->setAllMvd( cMvd, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( eRefPicListOpt )->setAllRefIdx( iRefIdxOpt, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( (RefPicList)(1-iRefListOpt) )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 1 + iRefListOpt, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( iBestInterMvpIdx[0], eRefPicListOpt, uiPartAddr, iPartIdx, uiDepth );
    }
  }
  else
  {
    Int iDummyWidth2, iDummyHeight2;
    UInt uiPartAddr = 0;
    UInt iPartIdx = 0;

    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth2, iDummyHeight2 );

    if ( isMergeMode[1] )
    {
      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeIndexSubParts( iBestInterMvpIdx[1], uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( iBestInterDir[1], uiPartAddr, iPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvField[1][0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvField[1][1], ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );

      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );
    }
    else
    {
      Int iRefListOpt = iBestInterDir[1];
      Int iRefIdxOpt = iBestRefIdx[1];
      RefPicList  eRefPicListOpt = (iRefListOpt ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
#if SCM_AMVR_UNIFICATION            
      if( pcCU->getSlice()->getUseIntegerMv() )
      {            
        cMvd.setHor( (iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getHor()>>2) - (cMvPredCand[1][0].getHor()>>2) );
        cMvd.setVer( (iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getVer()>>2) - (cMvPredCand[1][0].getVer()>>2) );
        pcCU->getCUMvField( eRefPicListOpt )->setAllMv( (iMvCandList[2*iRefIdxOpt + 4*iRefListOpt]>>2)<<2, ePartSize, uiPartAddr, 0, iPartIdx );
      }
      else
      {
        cMvd.setHor( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getHor() - cMvPredCand[1][0].getHor() );
        cMvd.setVer( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getVer() - cMvPredCand[1][0].getVer() );
        pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
      }      
#else
      cMvd.setHor( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getHor() - cMvPredCand[1][0].getHor() );
      cMvd.setVer( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt].getVer() - cMvPredCand[1][0].getVer() );
      pcCU->getCUMvField( eRefPicListOpt )->setAllMv( iMvCandList[2*iRefIdxOpt + 4*iRefListOpt], ePartSize, uiPartAddr, 0, iPartIdx );
#endif 
      pcCU->getCUMvField( eRefPicListOpt )->setAllMvd( cMvd, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( eRefPicListOpt )->setAllRefIdx( iRefIdxOpt, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( (RefPicList)(1-iRefListOpt) )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 1 + iRefListOpt, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( iBestInterMvpIdx[1], eRefPicListOpt, uiPartAddr, iPartIdx, uiDepth );
    }

    iPartIdx = 1;
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth2, iDummyHeight2 );

    if ( isIBCMergeMode[1] )
    {
      pcCU->setMergeFlagSubParts( true, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMergeIndexSubParts( iBestIBCMvpIdx[1], uiPartAddr, iPartIdx, uiDepth );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMRGMvFieldIBC[1][0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMRGMvFieldIBC[1][1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, uiDepth );

    }
    else
    {
      pcCU->setMergeFlagSubParts( false, uiPartAddr, iPartIdx, uiDepth );
      cMvd.setHor( (iMvCandList[9].getHor() - cMvPredCand[1][1].getHor())>>2 );
      cMvd.setVer( (iMvCandList[9].getVer() - cMvPredCand[1][1].getVer())>>2 );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( iMvCandList[9], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setMVPIdxSubParts( iBestIBCMvpIdx[1], REF_PIC_LIST_0, uiPartAddr, iPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, uiDepth );  // list 0 prediction
    }

  }
  motionCompensation( pcCU, rpcPredYuv );

  return true;
}

Void TEncSearch::addToSortList( list<BlockHash>& listBlockHash, list<Int>& listCost, Int cost, const BlockHash& blockHash )
{
  assert( listBlockHash.size() == listCost.size() );
  list<BlockHash>::iterator itBlockHash = listBlockHash.begin();
  list<Int>::iterator itCost = listCost.begin();

  while ( itCost != listCost.end() )
  {
    if ( cost < (*itCost) )
    {
      listCost.insert( itCost, cost );
      listBlockHash.insert( itBlockHash, blockHash );
      return;
    }

    ++itCost;
    ++itBlockHash;
  }

  listCost.push_back( cost );
  listBlockHash.push_back( blockHash );
}

Distortion TEncSearch::getSAD( Pel* pRef, Int refStride, Pel* pCurr, Int currStride, Int width, Int height, const BitDepths& bitDepths )
{
  Distortion dist = 0;

  for ( Int i=0; i<height; i++ )
  {
    for ( Int j=0; j<width; j++ )
    {
      dist += abs( pRef[j] - pCurr[j] );
    }
    pRef += refStride;
    pCurr += currStride;
  }

  if ( bitDepths.recon[CHANNEL_TYPE_LUMA] == 8 )
  {
    return dist;
  }
  else
  {
    Int shift = DISTORTION_PRECISION_ADJUSTMENT( bitDepths.recon[CHANNEL_TYPE_LUMA] - 8 );
    return dist >> shift;
  }
  return 0;
}

Bool TEncSearch::predInterHashSearch( TComDataCU* pcCU, TComYuv* pcOrg, TComYuv*& rpcPredYuv, Bool& isPerfectMatch )
{
  rpcPredYuv->clear();
  TComMvField  cMEMvField;
  TComMv       bestMv, bestMvd;
  RefPicList   bestRefPicList;
  Int          bestRefIndex;
  Int          bestMVPIndex;
  Int          iPartIdx   = 0;
  Int          uiPartAddr = 0;
  PartSize     ePartSize  = pcCU->getPartitionSize( 0 );

  //  Clear Motion Field
  TComMv cMvZero( 0, 0 );
  pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
  pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( TComMvField(), ePartSize, uiPartAddr, 0, iPartIdx );
  pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );
  pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvZero, ePartSize, uiPartAddr, 0, iPartIdx );

  pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
  pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
  pcCU->setMVPIdxSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );
  pcCU->setMVPNumSubParts( -1, REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );

  if ( xHashInterEstimation( pcCU, pcCU->getWidth( 0 ), pcCU->getHeight( 0 ), bestRefPicList, bestRefIndex, bestMv, bestMvd, bestMVPIndex, isPerfectMatch ) )
  {
    pcCU->getCUMvField( bestRefPicList )->setAllMv( bestMv, ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField( bestRefPicList )->setAllRefIdx( bestRefIndex, ePartSize, uiPartAddr, 0, iPartIdx );
    pcCU->getCUMvField( bestRefPicList )->setAllMvd( bestMvd, ePartSize, uiPartAddr, 0, iPartIdx );

    pcCU->setInterDirSubParts( static_cast<Int>(bestRefPicList) + 1, uiPartAddr, iPartIdx, pcCU->getDepth( 0 ) );
    pcCU->setMVPIdxSubParts( bestMVPIndex, bestRefPicList, uiPartAddr, iPartIdx, pcCU->getDepth( uiPartAddr ) );

    motionCompensation ( pcCU, rpcPredYuv, REF_PIC_LIST_X, iPartIdx );
    return true;
  }
  else
  {
    return false;
  }

  assert( 0 );
  return true;
}

Void TEncSearch::selectMatchesInter( TComDataCU* pcCU, const MapIterator& itBegin, Int count, list<BlockHash>& listBlockHash, const BlockHash& currBlockHash )
{
  const Int maxReturnNumber = 5;

  listBlockHash.clear();
  list<Int> listCost;
  listCost.clear();

  MapIterator it = itBegin;
  for ( Int i=0; i<count; i++, it++ )
  {
    if ( (*it).hashValue2 != currBlockHash.hashValue2 )  // check having the same second hash values, otherwise, not matched
    {
      continue;
    }

    // as exactly matched, only calculate bits
    Int currCost = TComRdCost::xGetExpGolombNumberOfBits( (*it).x - currBlockHash.x ) +
                   TComRdCost::xGetExpGolombNumberOfBits( (*it).y - currBlockHash.y );

    if ( listBlockHash.size() < maxReturnNumber )
    {
      addToSortList( listBlockHash, listCost, currCost, (*it) );
    }
    else if ( !listCost.empty() && currCost < listCost.back( ) )
    {
      listCost.pop_back( );
      listBlockHash.pop_back();
      addToSortList( listBlockHash, listCost, currCost, (*it) );
    }
  }
}


Int TEncSearch::xHashInterPredME( TComDataCU* pcCU, Int width, Int height, RefPicList currRefPicList, Int currRefPicIndex, TComMv bestMv[5] )
{
  TComPic* pcPic = pcCU->getPic();
  Int xPos = pcCU->getCUPelX();
  Int yPos = pcCU->getCUPelY();
  UInt hashValue1;
  UInt hashValue2;

  if ( !TComHash::getBlockHashValue( pcPic->getPicYuvOrg(), width, height, xPos, yPos, pcCU->getSlice()->getSPS()->getBitDepths(), hashValue1, hashValue2 ) )
  {
    return 0;
  }


  BlockHash currBlockHash;
  currBlockHash.x = xPos;
  currBlockHash.y = yPos;
  currBlockHash.hashValue2 = hashValue2;

  Int count = static_cast<Int>(pcCU->getSlice()->getRefPic( currRefPicList, currRefPicIndex )->getHashMap()->count( hashValue1 ));
  if ( count == 0 )
  {
    return 0;
  }

  list<BlockHash> listBlockHash;
  selectMatchesInter( pcCU, pcCU->getSlice()->getRefPic( currRefPicList, currRefPicIndex )->getHashMap()->getFirstIterator( hashValue1 ), count, listBlockHash, currBlockHash );

  if ( listBlockHash.empty() )
  {
    return 0;
  }

  Int totalSize = 0;
  list<BlockHash>::iterator it = listBlockHash.begin();
  for ( Int i=0; i<5 && i<listBlockHash.size(); i++, it++ )
  {
    bestMv[i].set( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
    totalSize++;
  }

  return totalSize;
}

Bool TEncSearch::xHashInterEstimation( TComDataCU* pcCU, Int width, Int height, RefPicList& bestRefPicList, Int& bestRefIndex, TComMv& bestMv, TComMv& bestMvd, Int& bestMVPIndex, Bool& isPerfectMatch )
{
  TComPic* pcPic = pcCU->getPic();
  Int xPos = pcCU->getCUPelX();
  Int yPos = pcCU->getCUPelY();
  UInt hashValue1;
  UInt hashValue2;
  Int bestCost = MAX_INT;

  if ( !TComHash::getBlockHashValue( pcPic->getPicYuvOrg(), width, height, xPos, yPos, pcCU->getSlice()->getSPS()->getBitDepths(), hashValue1, hashValue2 ) )
  {
    return false;
  }

  BlockHash currBlockHash;
  currBlockHash.x = xPos;
  currBlockHash.y = yPos;
  currBlockHash.hashValue2 = hashValue2;

  Pel* pCurrStart = pcCU->getPic()->getPicYuvOrg()->getAddr( COMPONENT_Y );
  Int currStride = pcCU->getPic()->getPicYuvOrg()->getStride( COMPONENT_Y );
  Pel* pCurr = pCurrStart + (currBlockHash.y)*currStride + (currBlockHash.x);

  Int iNumPredDir = pcCU->getSlice()->isInterP() ? 1 : 2;
  for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
  {
    RefPicList eRefPicList = (iRefList==0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    Int refPicNumber = pcCU->getSlice()->getNumRefIdx( eRefPicList );
#if SCM_U0083_U0079_IBC_SIGNAL_PPS
    if ( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#else
    if ( pcCU->getSlice()->getSPS()->getSpsScreenExtension().getUseIntraBlockCopy() && eRefPicList == REF_PIC_LIST_0 )
#endif
    {
      refPicNumber--;
    }
    for ( Int iRefIdx = 0; iRefIdx < refPicNumber; iRefIdx++ )
    {
      Int bitsOnRefIdx = iRefIdx+1;
      if ( iRefIdx+1 == pcCU->getSlice()->getNumRefIdx( eRefPicList ) )
      {
        bitsOnRefIdx--;
      }
      
      if ( iRefList == 0 || pcCU->getSlice()->getList1IdxToList0Idx( iRefIdx ) < 0 )
      {
        Int count = static_cast<Int>( pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getHashMap()->count( hashValue1 ) );
        if ( count == 0 )
        {
          continue;
        }

        list<BlockHash> listBlockHash;
        selectMatchesInter( pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getHashMap()->getFirstIterator( hashValue1 ), count, listBlockHash, currBlockHash );

        if ( listBlockHash.empty() )
        {
          continue;
        }

        AMVPInfo currAMVPInfo;
        pcCU->fillMvpCand( 0, 0, eRefPicList, iRefIdx, &currAMVPInfo );

        Pel* pRefStart = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec()->getAddr( COMPONENT_Y );
        Int refStride = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec()->getStride( COMPONENT_Y );

        m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass( 0 ) );
        m_pcRdCost->setCostScale( 2 );
        if ( pcCU->getSlice()->getUseIntegerMv() )
        {
          m_pcRdCost->setCostScale( 0 );
        }
        list<BlockHash>::iterator it;
        for ( it = listBlockHash.begin(); it != listBlockHash.end(); ++it )
        {
          Int currMVPIdx = 0;
          if ( currAMVPInfo.iN > 1 )
          {
            m_pcRdCost->setPredictor( currAMVPInfo.m_acMvCand[0] );
            Int bitsMVP0 = m_pcRdCost->getBits( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
            m_pcRdCost->setPredictor( currAMVPInfo.m_acMvCand[1] );
            Int bitsMVP1 = m_pcRdCost->getBits( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
            if ( bitsMVP1 < bitsMVP0 )
            {
              currMVPIdx = 1;
            }
          }
          m_pcRdCost->setPredictor( currAMVPInfo.m_acMvCand[currMVPIdx] );

          Pel* pRef = pRefStart + (*it).y*refStride + (*it).x;
          Distortion currSad = getSAD( pRef, refStride, pCurr, currStride, width, height, pcCU->getSlice()->getSPS()->getBitDepths() );
          Int bits = bitsOnRefIdx + m_pcRdCost->getBits( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
          Distortion currCost = currSad + m_pcRdCost->getCost( bits );

          if ( !isPerfectMatch && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N )
          {
            if ( pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getSlice( 0 )->getSliceQp() <= pcCU->getSlice()->getSliceQp() )
            {
              isPerfectMatch = true;
            }
          }

          if ( currCost < bestCost )
          {
            bestCost = (Int)currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex = iRefIdx;
            bestMv.set( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
            bestMv <<= 2;
#if !SCM_AMVR_UNIFICATION
            if ( pcCU->getSlice()->getUseIntegerMv() )
            {
              bestMv >>= 2;
            }
#endif 
#if SCM_AMVR_UNIFICATION
            if( pcCU->getSlice()->getUseIntegerMv() )
            {
              bestMvd.set( (bestMv.getHor()>>2) - (currAMVPInfo.m_acMvCand[currMVPIdx].getHor()>>2), (bestMv.getVer()>>2) - (currAMVPInfo.m_acMvCand[currMVPIdx].getVer()>>2) );
            }
            else
            {
               bestMvd.set( bestMv.getHor() - currAMVPInfo.m_acMvCand[currMVPIdx].getHor(), bestMv.getVer() - currAMVPInfo.m_acMvCand[currMVPIdx].getVer() );
            }
#else
            bestMvd.set( bestMv.getHor() - currAMVPInfo.m_acMvCand[currMVPIdx].getHor(), bestMv.getVer() - currAMVPInfo.m_acMvCand[currMVPIdx].getVer() );
#endif
            bestMVPIndex = currMVPIdx;
          }
        }
      }
    }
  }

  return (bestCost < MAX_INT);
}

// based on xMotionEstimation
Void TEncSearch::xIntraBlockCopyEstimation( TComDataCU *pcCU,
                                            TComYuv    *pcYuvOrg,
                                            Int         iPartIdx,
                                            TComMv     *pcMvPred,
                                            TComMv     &rcMv,
                                            Distortion &ruiCost,
                                            Bool        bUse1DSearchFor8x8
                                          , Bool        testOnlyPred
                                           )
{
  UInt          uiPartAddr;
  Int           iRoiWidth;
  Int           iRoiHeight;

  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;

  TComYuv*      pcYuv = pcYuvOrg;

  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
                             iRoiWidth,
                             iRoiHeight,
                             pcYuv->getStride(COMPONENT_Y),
                             pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

  Pel*        piRefY      = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartAddr );
  Int         iRefStride  = pcCU->getPic()->getPicYuvRec()->getStride(COMPONENT_Y);

  TComMv      cMvPred = *pcMvPred;

  // assume that intra BV is integer-pel precision
  xSetIntraSearchRange   ( pcCU, cMvPred, uiPartAddr, iRoiWidth, iRoiHeight, cMvSrchRngLT, cMvSrchRngRB );

  // disable weighted prediction
  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
  m_pcRdCost->setPredictors(pcMvPred);
  m_pcRdCost->setCostScale  ( 0 );

  //  Do integer search
  xIntraPatternSearch      ( pcCU, iPartIdx, uiPartAddr, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, iRoiWidth, iRoiHeight, pcMvPred, bUse1DSearchFor8x8, testOnlyPred );
  //printf("ruiCost = %d\n", ruiCost);
}

// based on xSetSearchRange
Void TEncSearch::xSetIntraSearchRange ( TComDataCU* pcCU, TComMv& cMvPred, UInt uiPartAddr, Int iRoiWidth, Int iRoiHeight, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB )
{
  TComMv cTmpMvPred = cMvPred;
  pcCU->clipMv( cTmpMvPred );

  Int srLeft, srRight, srTop, srBottom;

  const UInt lcuWidth = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt cuPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiPartAddr ] ]; //NOTE: RExt - This variable (and its counterpart below) refer to the PU, not the CU - change these names
  const UInt lcuHeight = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  const UInt cuPelY    = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiPartAddr ] ];

  const Int iPicWidth  = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  const Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

#if SCM_T0056_IBC_VALIDATE_TILES
  const UInt curTileIdx = pcCU->getPic()->getPicSym()->getTileIdxMap( pcCU->getCtuRsAddr() );
  TComTile* curTile = pcCU->getPic()->getPicSym()->getTComTile( curTileIdx );

  const Int tileAreaRight  = (curTile->getRightEdgePosInCtus() + 1) * lcuWidth;
  const Int tileAreaBottom = (curTile->getBottomEdgePosInCtus() + 1) * lcuHeight;

  const Int tileAreaLeft   = tileAreaRight - curTile->getTileWidthInCtus() * lcuWidth;
  const Int tileAreaTop    = tileAreaBottom - curTile->getTileHeightInCtus() * lcuHeight;
#endif

  if((pcCU->getWidth(0) == 16) && (pcCU->getPartitionSize(0) == SIZE_2Nx2N) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
  {
    srLeft  = -1 * cuPelX;
    srTop   = -1 * cuPelY;
    TComSlice *pcSlice = pcCU->getSlice();
    if( pcSlice->getSliceMode() )
    {
      TComPicSym *pcSym = pcCU->getPic()->getPicSym();
      UInt addr = pcSym->getCtuTsToRsAddrMap( pcSlice->getSliceSegmentCurStartCtuTsAddr() );
      srTop += pcSym->getCtu(addr)->getCUPelY();
    }

    srRight = iPicWidth - cuPelX - iRoiWidth;
    srBottom = lcuHeight - cuPelY % lcuHeight - iRoiHeight;
  }
  else
  {
  const UInt uiSearchWidthInCTUs = pcCU->getWidth( 0 ) == 8 ? m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() : m_pcEncCfg->getIntraBCSearchWidthInCTUs();
  Int maxXsr = (cuPelX % lcuWidth) + pcCU->getIntraBCSearchAreaWidth( uiSearchWidthInCTUs );
  Int maxYsr =  cuPelY % lcuHeight;

  const ChromaFormat format = pcCU->getPic()->getChromaFormat();

  if ((format == CHROMA_420) || (format == CHROMA_422)) maxXsr &= ~0x4;
  if ((format == CHROMA_420)                          ) maxYsr &= ~0x4;

  srLeft   = -maxXsr;
  srTop    = -maxYsr;

  srRight = lcuWidth - cuPelX %lcuWidth - iRoiWidth;
  srBottom = lcuHeight - cuPelY % lcuHeight - iRoiHeight;
  }


  if( cuPelX + srRight + iRoiWidth > iPicWidth)
  {
    srRight = iPicWidth%lcuWidth - cuPelX %lcuWidth - iRoiWidth;
  }
  if( cuPelY + srBottom + iRoiHeight > iPicHeight)
  {
    srBottom = iPicHeight%lcuHeight - cuPelY % lcuHeight - iRoiHeight;
  }

#if SCM_T0056_IBC_VALIDATE_TILES
  if( cuPelX + srRight + iRoiWidth > tileAreaRight)
  {
    srRight = tileAreaRight%lcuWidth - cuPelX %lcuWidth - iRoiWidth;
  }
  if( cuPelY + srBottom + iRoiHeight > tileAreaBottom)
  {
    srBottom = tileAreaBottom%lcuHeight - cuPelY % lcuHeight - iRoiHeight;
  }
  if( cuPelX + srLeft < tileAreaLeft)
  {
    srLeft = tileAreaLeft - cuPelX;
  }
  if( cuPelY + srTop < tileAreaTop)
  {
    srTop = tileAreaTop - cuPelY;
  }
#endif

  rcMvSrchRngLT.setHor( srLeft );
  rcMvSrchRngLT.setVer( srTop );
  rcMvSrchRngRB.setHor( srRight );
  rcMvSrchRngRB.setVer( srBottom );

  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );
}

#if SCM_FIX_TICKET_1401
Bool TEncSearch::xCIPIBCSearchPruning( TComDataCU* pcCU, Int refPixlX, Int refPixlY, Int roiWidth, Int roiHeight )
{
  const Int iMaxCuWidth   = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const Int iMaxCuHeight  = pcCU->getSlice()->getSPS()->getMaxCUHeight();

  UInt partNumX = roiWidth/pcCU->getPic()->getMinCUWidth() + (((refPixlX%pcCU->getPic()->getMinCUWidth()) == 0) ? 0:1);
  UInt partNumY = roiHeight/pcCU->getPic()->getMinCUHeight() + (((refPixlY%pcCU->getPic()->getMinCUHeight()) == 0) ? 0:1);


  for(Int partY = 0; partY < partNumY; partY++)
  {
    for(Int partX = 0; partX < partNumX; partX++)
    {
      Int currRefX = refPixlX + partX * pcCU->getPic()->getMinCUWidth();
      Int currRefY = refPixlY + partY * pcCU->getPic()->getMinCUHeight();

      Int currRefCtuX = currRefX/iMaxCuWidth;
      Int currRefCtuY = currRefY/iMaxCuHeight;
      Int currRefCtuRs = currRefCtuY * pcCU->getPic()->getFrameWidthInCtus() + currRefCtuX;

      Int currRefRelX = currRefX%iMaxCuWidth;
      Int currRefRelY = currRefY%iMaxCuHeight;

      TComDataCU* pcCurrRefCU = pcCU->getPic()->getCtu( currRefCtuRs );
      UInt uiAbsPartIdx = g_auiRasterToZscan[currRefRelX/pcCU->getPic()->getMinCUWidth() + (currRefRelY/pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];

      if(pcCurrRefCU->isInter(uiAbsPartIdx))
      {
        Int iRefL0 = pcCurrRefCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiAbsPartIdx);
        Int iRefL1 = pcCurrRefCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiAbsPartIdx);

        if( iRefL0 >= 0 )
        {
          if( pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, iRefL0 )->getPOC() != pcCU->getSlice()->getPOC() )
          {
            return false;
          }
        }
        
        if( iRefL1 >= 0 )
        {
          if( pcCU->getSlice()->getRefPic( REF_PIC_LIST_1, iRefL1 )->getPOC() != pcCU->getSlice()->getPOC() )
          {
            return false;
          }
        }        
      }
    }
  }

  return true;
}
#else
Bool TEncSearch::xCIPIntraSearchPruning( TComDataCU* pcCU, Int relX, Int relY, Int roiWidth, Int roiHeight )
{
  UInt uiAbsPartIdx;
  TComDataCU* pcPredCU;

  UInt partNumX = roiWidth/pcCU->getPic()->getMinCUWidth() + (((relX%pcCU->getPic()->getMinCUWidth()) == 0) ? 0:1);
  UInt partNumY = roiHeight/pcCU->getPic()->getMinCUHeight() + (((relY%pcCU->getPic()->getMinCUHeight()) == 0) ? 0:1);


  for(Int partY = 0; partY < partNumY; partY++)
  {
    for(Int partX = 0; partX < partNumX; partX++)
    {
      Int currX = relX + partX * pcCU->getPic()->getMinCUWidth();
      Int currY = relY + partY * pcCU->getPic()->getMinCUHeight();

      assert(currY >= 0);

      if(currX < 0)
      {
        pcPredCU = pcCU->getCtuLeft();
        currX += pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getNumPartInCtuWidth();
        uiAbsPartIdx = g_auiRasterToZscan[currX/pcCU->getPic()->getMinCUWidth() + (currY/pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
        if(pcPredCU->isInter(uiAbsPartIdx))
          return false;
      }
      else
      {
        pcPredCU = pcCU->getPic()->getCtu( pcCU->getCtuRsAddr() );
        uiAbsPartIdx = g_auiRasterToZscan[currX/pcCU->getPic()->getMinCUWidth() + (currY/pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInCtuWidth()];
        if(pcPredCU->isInter(uiAbsPartIdx))
          return false;
      }
    }
  }

  return true;
}
#endif

Void TEncSearch::xIntraBCSearchMVCandUpdate(Distortion  uiSad, Int x, Int y, Distortion* uiSadBestCand, TComMv* cMVCand)
{
  int j = CHROMA_REFINEMENT_CANDIDATES - 1;

  if(uiSad < uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
  {
    for(int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
    {
      if(uiSad < uiSadBestCand[t])
        j = t;
    }

    for(int k = CHROMA_REFINEMENT_CANDIDATES - 1;k > j; k--)
    {
      uiSadBestCand[k]=uiSadBestCand[k-1];

      cMVCand[k].set(cMVCand[k-1].getHor(),cMVCand[k-1].getVer());
    }
    uiSadBestCand[j]= uiSad;
    cMVCand[j].set(x,y);
  }  
}

Int TEncSearch::xIntraBCSearchMVChromaRefine( TComDataCU* pcCU,
                                              Int         iRoiWidth,
                                              Int         iRoiHeight,
                                              Int         cuPelX,
                                              Int         cuPelY,
                                              Distortion* uiSadBestCand, 
                                              TComMv*     cMVCand, 
                                              UInt        uiPartOffset
#if SCM_IBC_CR_INTERPOLATION_ENABLE
                                             ,Int         iPartIdx
#endif 
                                            )
{
  Int iBestCandIdx = 0;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  Distortion  uiTempSad;

  Pel* pRef;
  Pel* pOrg;
  Int iRefStride, iOrgStride;
  Int iWidth, iHeight;
  Int iMvx, iMvy;

  Int iPicWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

  for(int iCand = 0; iCand < CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    if((!cMVCand[iCand].getHor()) && (!cMVCand[iCand].getVer())) 
      continue;

    if(((Int) (cuPelY + cMVCand[iCand].getVer() + iRoiHeight) >= iPicHeight) || ((cuPelY + cMVCand[iCand].getVer()) < 0))
      continue;

    if(((Int) (cuPelX + cMVCand[iCand].getHor() + iRoiWidth) >= iPicWidth) || ((cuPelX + cMVCand[iCand].getHor()) < 0))
      continue;

    uiTempSad = uiSadBestCand[iCand];
    BitDepths bitDepths = pcCU->getSlice()->getSPS()->getBitDepths();
#if SCM_IBC_CR_INTERPOLATION_ENABLE
    TComMv cMvQuaterPixl = cMVCand[iCand];
    cMvQuaterPixl <<= 2;
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMv( cMvQuaterPixl, pcCU->getPartitionSize(0), uiPartOffset, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) - 1, pcCU->getPartitionSize(0), uiPartOffset, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( TComMv(), pcCU->getPartitionSize(0), uiPartOffset, 0, iPartIdx );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( -1, pcCU->getPartitionSize(0), uiPartOffset, 0, iPartIdx );
    pcCU->setInterDirSubParts( 1 + REF_PIC_LIST_0, uiPartOffset, iPartIdx, pcCU->getDepth(0) );
    motionCompensation( pcCU, &m_tmpYuvPred, REF_PIC_LIST_X, iPartIdx );
#endif
    for (UInt ch = COMPONENT_Cb; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      pRef = pcCU->getPic()->getPicYuvRec()->getAddr(ComponentID(ch), pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartOffset);
      if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && m_pcEncCfg->getRGBFormatFlag() )
      {
        pOrg = pcCU->getPic()->getPicYuvResi()->getAddr(ComponentID(ch), pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartOffset);
      }
      else
      pOrg = pcCU->getPic()->getPicYuvOrg()->getAddr(ComponentID(ch), pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartOffset);
      iRefStride = pcCU->getPic()->getPicYuvRec()->getStride(ComponentID(ch));
      if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && m_pcEncCfg->getRGBFormatFlag() )
      {
        iOrgStride = pcCU->getPic()->getPicYuvResi()->getStride(ComponentID(ch));
      }
      else
      iOrgStride = pcCU->getPic()->getPicYuvOrg()->getStride(ComponentID(ch));
      iWidth = iRoiWidth >> pcCU->getPic()->getComponentScaleX(ComponentID(ch));
      iHeight = iRoiHeight >> pcCU->getPic()->getComponentScaleY(ComponentID(ch));
      iMvx = cMVCand[iCand].getHor() >> pcCU->getPic()->getComponentScaleX(ComponentID(ch));
      iMvy = cMVCand[iCand].getVer() >> pcCU->getPic()->getComponentScaleY(ComponentID(ch));

#if SCM_IBC_CR_INTERPOLATION_ENABLE
      ComponentID compID = (ComponentID)ch;      
      pRef = m_tmpYuvPred.getAddr( compID, uiPartOffset);
      iRefStride = m_tmpYuvPred.getStride(compID);
#else
      pRef = pRef + iMvy * iRefStride + iMvx;
#endif 
      for(int row = 0; row < iHeight; row++)
      {
        for(int col = 0; col < iWidth; col++)
        {
          uiTempSad += ( (abs( pRef[col] - pOrg[col] )) >> (bitDepths.recon[CHANNEL_TYPE_CHROMA]-8) );
        }
        pRef += iRefStride;
        pOrg += iOrgStride;
      }
    }

    if(uiTempSad < uiSadBest)
    {
      uiSadBest = uiTempSad;
      iBestCandIdx = iCand;
    }
  }

  return iBestCandIdx;
}

static UInt MergeCandLists(TComMv *dst, UInt dn, TComMv *src, UInt sn, Bool isSrcQuarPel)
{
  for(UInt cand = 0; cand < sn && dn<SCM_S0067_NUM_CANDIDATES; cand++)
  {
    Bool found = false;
    TComMv TempMv = src[cand];
    if ( !isSrcQuarPel )
    {
      TempMv <<= 2;
    }
    for(int j=0; j<dn; j++)
    {
      if( TempMv == dst[j] )
      {
        found = true;
        break;
      }
    }

    if( !found )
    {
      dst[dn] = TempMv;
      dn++;
    }
  }

  return dn;
}

// based on xPatternSearch
Void TEncSearch::xIntraPatternSearch( TComDataCU  *pcCU,
                                      Int          iPartIdx,
                                      UInt         uiPartAddr,
                                      TComPattern *pcPatternKey,
                                      Pel         *piRefY,
                                      Int          iRefStride,
                                      TComMv      *pcMvSrchRngLT,
                                      TComMv      *pcMvSrchRngRB,
                                      TComMv      &rcMv,
                                      Distortion  &ruiSAD,
                                      Int          iRoiWidth,
                                      Int          iRoiHeight,
                                      TComMv      *mvPred,
                                      Bool         bUse1DSearchFor8x8
                                    , Bool         testOnlyPred
                                      )
{
  const Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  const Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  const Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  const Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  const UInt  lcuWidth          = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt  lcuHeight         = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  const Int   puPelOffsetX      = g_auiRasterToPelX[ g_auiZscanToRaster[ uiPartAddr ] ];
  const Int   puPelOffsetY      = g_auiRasterToPelY[ g_auiZscanToRaster[ uiPartAddr ] ];
  const Int   cuPelX            = pcCU->getCUPelX() + puPelOffsetX;  // Point to the location of PU
  const Int   cuPelY            = pcCU->getCUPelY() + puPelOffsetY;

  Distortion  uiSad;
  Distortion  uiSadBest         = std::numeric_limits<Distortion>::max();
  Int         iBestX            = 0;
  Int         iBestY            = 0;

  Pel*        piRefSrch;

  Int         iBestCandIdx = 0;
  UInt        uiPartOffset = 0;
  Distortion  uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  TComMv      cMVCand[CHROMA_REFINEMENT_CANDIDATES];

  uiPartOffset = uiPartAddr;

  for(int iCand = 0; iCand < CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    uiSadBestCand[iCand] = std::numeric_limits<Distortion>::max();
    cMVCand[iCand].set(0,0);
  }

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  const Int        iRelCUPelX    = cuPelX % lcuWidth;
  const Int        iRelCUPelY    = cuPelY % lcuHeight;

#if SCM_FIX_TICKET_1401
  const Int chromaROIWidthInPixels  = iRoiWidth;
  const Int chromaROIHeightInPixels = iRoiHeight;
#else  
  const ChromaFormat format = pcCU->getPic()->getChromaFormat();
    
  const Int chromaROIWidthInPixels  = (((format == CHROMA_420) || (format == CHROMA_422)) && (iRoiWidth  == 4) && ((iRelCUPelX & 0x4) != 0)) ? (iRoiWidth  * 2) : iRoiWidth;
  const Int chromaROIHeightInPixels = (((format == CHROMA_420)                          ) && (iRoiHeight == 4) && ((iRelCUPelY & 0x4) != 0)) ? (iRoiHeight * 2) : iRoiHeight;
  const Int chromaROIStartXInPixels = iRelCUPelX + iRoiWidth  - chromaROIWidthInPixels;
  const Int chromaROIStartYInPixels = iRelCUPelY + iRoiHeight - chromaROIHeightInPixels;
#endif

#if SCM_T0056_IBC_VALIDATE_TILES
  const UInt curTileIdx = pcCU->getPic()->getPicSym()->getTileIdxMap( pcCU->getCtuRsAddr() );
  TComTile* curTile = pcCU->getPic()->getPicSym()->getTComTile( curTileIdx );

  const Int tileAreaRight  = (curTile->getRightEdgePosInCtus() + 1) * lcuWidth;
  const Int tileAreaBottom = (curTile->getBottomEdgePosInCtus() + 1) * lcuHeight;

  const Int tileAreaLeft   = tileAreaRight - curTile->getTileWidthInCtus() * lcuWidth;
  const Int tileAreaTop    = tileAreaBottom - curTile->getTileHeightInCtus() * lcuHeight;
#endif

  if (m_pcEncCfg->getUseIntraBlockCopyFastSearch())
  {
    setDistParamComp(COMPONENT_Y);
    m_cDistParam.bitDepth  = pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );
    m_cDistParam.iRows     = 4;//to calculate the sad line by line;
    m_cDistParam.iSubShift = 0;

    Distortion uiTempSadBest = 0;

    Int srLeft = iSrchRngHorLeft, srRight = iSrchRngHorRight, srTop = iSrchRngVerTop, srBottom = iSrchRngVerBottom;

    const Int iPicWidth  = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
    const Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

    if(m_pcEncCfg->getUseIntraBCFullFrameSearch() )
    {
      srLeft  = -1 * cuPelX;
      srTop   = -1 * cuPelY;

      srRight  = iPicWidth - cuPelX - iRoiWidth;
      srBottom = lcuHeight - cuPelY % lcuHeight - iRoiHeight;

      if( cuPelX + srRight + iRoiWidth > iPicWidth)
      {
        srRight = iPicWidth%lcuWidth - cuPelX %lcuWidth - iRoiWidth;
      }
      if( cuPelY + srBottom + iRoiHeight > iPicHeight)
      {
        srBottom = iPicHeight%lcuHeight - cuPelY % lcuHeight - iRoiHeight;
      }
#if SCM_T0056_IBC_VALIDATE_TILES
      if( cuPelX + srRight + iRoiWidth > tileAreaRight)
      {
        srRight = tileAreaRight%lcuWidth - cuPelX %lcuWidth - iRoiWidth;
      }
      if( cuPelY + srBottom + iRoiHeight > tileAreaBottom)
      {
        srBottom = tileAreaBottom%lcuHeight - cuPelY % lcuHeight - iRoiHeight;
      }
      if( cuPelX + srLeft < tileAreaLeft)
      {
        srLeft = tileAreaLeft - cuPelX;
      }
      if( cuPelY + srTop < tileAreaTop)
      {
        srTop = tileAreaTop - cuPelY;
      }
#endif
    }

    if(iRoiWidth>8 || iRoiHeight>8)
    {
      m_uiNumBVs = 0;
    }
    else if (iRoiWidth+iRoiHeight==16)
    {
      m_uiNumBVs = m_uiNumBV16s;
    }

    if( testOnlyPred )
    {
      m_uiNumBVs = 0;
    }

    TComMv cMvPredEncOnly[16];
    Int nbPreds = 0;
    pcCU->getIntraBCMVPsEncOnly(uiPartAddr, cMvPredEncOnly, nbPreds, iPartIdx );
    m_uiNumBVs = MergeCandLists(m_acBVs, m_uiNumBVs, cMvPredEncOnly, nbPreds, true);
    for(UInt cand = 0; cand < m_uiNumBVs; cand++)
    {
      Int xPred = m_acBVs[cand].getHor()>>2;
      Int yPred = m_acBVs[cand].getVer()>>2;
      if ( !( xPred==0 && yPred==0)
            && !( (yPred < srTop)  || (yPred > srBottom) )
            && !( (xPred < srLeft) || (xPred > srRight) ) )
      {
        Int iTempY = yPred + iRelCUPelY + iRoiHeight - 1;
        Int iTempX = xPred + iRelCUPelX + iRoiWidth  - 1;
#if SCM_FIX_TICKET_1401
        Bool validCand = isValidIntraBCSearchArea(pcCU, xPred, yPred, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset);
#else
        Bool validCand = isValidIntraBCSearchArea(pcCU, iPartIdx, xPred, chromaROIStartXInPixels, yPred, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset);
#endif

        if((iTempX >= (Int)lcuWidth) && (iTempY >= 0) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
        {
          validCand = false;
        }

        if ((iTempX >= 0) && (iTempY >= 0))
        {
          Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInCtuWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
          Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
          if(iTempZscanIdx >= pcCU->getZorderIdxInCtu())
          {
            validCand = false;
          }
        }

        if( validCand )
        {
          uiSad = m_pcRdCost->getCostMultiplePreds( xPred, yPred);

          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + yPred * iRefStride + r*iRefStride + xPred;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, xPred, yPred, uiSadBestCand, cMVCand);
        }
      }
    }

    iBestX = cMVCand[0].getHor();
    iBestY = cMVCand[0].getVer();
    rcMv.set( iBestX, iBestY );
    uiSadBest = uiSadBestCand[0];

    if( testOnlyPred )
    {
      ruiSAD = uiSadBest;
      return;
    }

    const Int boundY = (0 - iRoiHeight - puPelOffsetY);
    Int lowY = ((pcCU->getPartitionSize(uiPartAddr) == SCM_S0067_IBC_FULL_1D_SEARCH_FOR_PU) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
             ? -cuPelY : max(iSrchRngVerTop, 0 - cuPelY);
    for(Int y = boundY ; y >= lowY ; y-- )
    {
#if SCM_FIX_TICKET_1401
      if ( !isValidIntraBCSearchArea( pcCU, 0, y, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset ) )
#else
      if ( !isValidIntraBCSearchArea( pcCU, iPartIdx, 0, chromaROIStartXInPixels, y, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset ) )
#endif
      {
        continue;
      }

      uiSad = m_pcRdCost->getCostMultiplePreds( 0, y);

      for(int r = 0; r < iRoiHeight; )
      {
        piRefSrch = piRefY + y * iRefStride + r*iRefStride;
        m_cDistParam.pCur = piRefSrch;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

        uiSad += m_cDistParam.DistFunc( &m_cDistParam );

        if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
          break;

        r += 4;
      }

      xIntraBCSearchMVCandUpdate(uiSad, 0, y, uiSadBestCand, cMVCand);
      uiTempSadBest = uiSadBestCand[0];
      if(uiSadBestCand[0] <= 3)
      {
        iBestX = cMVCand[0].getHor();
        iBestY = cMVCand[0].getVer();
        uiSadBest = uiSadBestCand[0];
        rcMv.set( iBestX, iBestY );
        ruiSAD = uiSadBest;
        goto end;
      }
    }

    const Int boundX = ((pcCU->getPartitionSize(uiPartAddr) == SCM_S0067_IBC_FULL_1D_SEARCH_FOR_PU) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
                     ? -cuPelX : max(iSrchRngHorLeft, - cuPelX);
    for(Int x = 0 - iRoiWidth - puPelOffsetX ; x >= boundX ; --x )
    {
#if SCM_FIX_TICKET_1401
      if (!isValidIntraBCSearchArea(pcCU, x, 0, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset))
#else
      if (!isValidIntraBCSearchArea(pcCU, iPartIdx, x, chromaROIStartXInPixels, 0, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset))
#endif
      {
        continue;
      }

      uiSad = m_pcRdCost->getCostMultiplePreds( x, 0);

      for(int r = 0; r < iRoiHeight; )
      {
        piRefSrch = piRefY + r*iRefStride + x;
        m_cDistParam.pCur = piRefSrch;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

        uiSad += m_cDistParam.DistFunc( &m_cDistParam );
        if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
          break;

        r += 4;
      }

      xIntraBCSearchMVCandUpdate(uiSad, x, 0, uiSadBestCand, cMVCand);
      uiTempSadBest = uiSadBestCand[0];
      if(uiSadBestCand[0] <= 3)
      {
        iBestX = cMVCand[0].getHor();
        iBestY = cMVCand[0].getVer();
        uiSadBest = uiSadBestCand[0];
        rcMv.set( iBestX, iBestY );
        ruiSAD = uiSadBest;
        goto end;
      }
    }

    iBestX = cMVCand[0].getHor();
    iBestY = cMVCand[0].getVer();
    uiSadBest = uiSadBestCand[0]; 
    if((!iBestX && !iBestY) || (uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY) <= 32))
    {
      //chroma refine
#if SCM_IBC_CR_INTERPOLATION_ENABLE
      iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else 
      iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
      iBestX       = cMVCand[iBestCandIdx].getHor();
      iBestY       = cMVCand[iBestCandIdx].getVer();
      uiSadBest    = uiSadBestCand[iBestCandIdx]; 
      rcMv.set( iBestX, iBestY );
      ruiSAD       = uiSadBest;
      goto end;
    }


    if( pcCU->getWidth(0) < 16 && !bUse1DSearchFor8x8 )
    {
      for(Int y = max(iSrchRngVerTop, -cuPelY); y <= iSrchRngVerBottom; y +=2)
      {
        if ((y == 0) || ((Int) (cuPelY + y + iRoiHeight) >= iPicHeight)) //NOTE: RExt - is this still necessary?
          continue;

        Int iTempY = y + iRelCUPelY + iRoiHeight - 1;

        for(Int x = max(iSrchRngHorLeft, -cuPelX); x <= iSrchRngHorRight; x++)
        {
          if ((x == 0) || ((Int) (cuPelX + x + iRoiWidth) >= iPicWidth)) //NOTE: RExt - is this still necessary?
            continue;

          Int iTempX = x + iRelCUPelX + iRoiWidth - 1;

          if ((iTempX >= 0) && (iTempY >= 0))
          {
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInCtuWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCtu())
              continue;
          }

#if SCM_FIX_TICKET_1401
          if (!isValidIntraBCSearchArea(pcCU, x, y, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset))
#else
          if (!isValidIntraBCSearchArea(pcCU, iPartIdx, x, chromaROIStartXInPixels, y,  chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset))
#endif
          {
            continue;
          }

          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
        }
      }

      iBestX = cMVCand[0].getHor();
      iBestY = cMVCand[0].getVer();
      uiSadBest = uiSadBestCand[0];
      if(uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY) <= 16)
      {
        //chroma refine
#if SCM_IBC_CR_INTERPOLATION_ENABLE
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
        iBestX       = cMVCand[iBestCandIdx].getHor();
        iBestY       = cMVCand[iBestCandIdx].getVer();
        uiSadBest    = uiSadBestCand[iBestCandIdx]; 
        rcMv.set( iBestX, iBestY );
        ruiSAD       = uiSadBest;
        goto end;
      }


      for(Int y = (max(iSrchRngVerTop, -cuPelY) + 1); y <= iSrchRngVerBottom; y += 2)
      {
        if ((y == 0) || ((Int) (cuPelY + y + iRoiHeight) >= iPicHeight)) //NOTE: RExt - is this still necessary?
          continue;

        Int iTempY = y + iRelCUPelY + iRoiHeight - 1;

        for(Int x = max(iSrchRngHorLeft, -cuPelX); x <= iSrchRngHorRight; x += 2)
        {
          if ((x == 0) || ((Int) (cuPelX + x + iRoiWidth) >= iPicWidth)) //NOTE: RExt - is this still necessary?
            continue;

          Int iTempX = x + iRelCUPelX + iRoiWidth - 1;

          if ((iTempX >= 0) && (iTempY >= 0))
          {
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInCtuWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCtu())
              continue;
          }

#if SCM_FIX_TICKET_1401
          if (!isValidIntraBCSearchArea(pcCU, x, y, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset))
#else
          if (!isValidIntraBCSearchArea(pcCU, iPartIdx, x, chromaROIStartXInPixels, y, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset))
#endif
          {
            continue;
          }

          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
          if(uiSadBestCand[0] <= 5)
          {
            //chroma refine & return
#if SCM_IBC_CR_INTERPOLATION_ENABLE
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
            iBestX       = cMVCand[iBestCandIdx].getHor();
            iBestY       = cMVCand[iBestCandIdx].getVer();
            uiSadBest    = uiSadBestCand[iBestCandIdx]; 
            rcMv.set( iBestX, iBestY );
            ruiSAD       = uiSadBest;
            goto end;
          }
        }
      }

      iBestX = cMVCand[0].getHor();
      iBestY = cMVCand[0].getVer();
      uiSadBest = uiSadBestCand[0];

      if((uiSadBest >= uiTempSadBest) || ((uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY)) <= 32))
      {
        //chroma refine
#if SCM_IBC_CR_INTERPOLATION_ENABLE
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
        iBestX       = cMVCand[iBestCandIdx].getHor();
        iBestY       = cMVCand[iBestCandIdx].getVer();
        uiSadBest    = uiSadBestCand[iBestCandIdx]; 
        rcMv.set( iBestX, iBestY );
        ruiSAD       = uiSadBest;
        goto end;
      }

      uiTempSadBest = uiSadBestCand[0];


      for(Int y = (max(iSrchRngVerTop, -cuPelY) + 1); y <= iSrchRngVerBottom; y += 2)
      {
        if ((y == 0) || ((Int) (cuPelY + y + iRoiHeight) >= iPicHeight)) //NOTE: RExt - is this still necessary?
          continue;

        Int iTempY = y + iRelCUPelY + iRoiHeight - 1;

        for(Int x = (max(iSrchRngHorLeft, -cuPelX) + 1); x <= iSrchRngHorRight; x += 2)
        {

          if ((x == 0) || ((Int) (cuPelX + x + iRoiWidth) >= iPicWidth)) //NOTE: RExt - is this still necessary?
            continue;

          Int iTempX = x + iRelCUPelX + iRoiWidth - 1;

          if ((iTempX >= 0) && (iTempY >= 0))
          {
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInCtuWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCtu())
              continue;
          }

#if SCM_FIX_TICKET_1401
          if (!isValidIntraBCSearchArea(pcCU, x, y, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset))
#else
          if (!isValidIntraBCSearchArea(pcCU, iPartIdx, x, chromaROIStartXInPixels, y, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset))
#endif
          {
            continue;
          }

          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
          if(uiSadBestCand[0] <= 5)
          {
            //chroma refine & return
#if SCM_IBC_CR_INTERPOLATION_ENABLE
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
            iBestX       = cMVCand[iBestCandIdx].getHor();
            iBestY       = cMVCand[iBestCandIdx].getVer();
            uiSadBest    = uiSadBestCand[iBestCandIdx]; 
            rcMv.set( iBestX, iBestY );
            ruiSAD       = uiSadBest;
            goto end;
          }
        }
      }
    }
  }
  else //full search
  {
    setDistParamComp(COMPONENT_Y);
    piRefY += (iSrchRngVerBottom * iRefStride);
    Int iPicWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
    Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

    for(Int y = iSrchRngVerBottom; y >= iSrchRngVerTop; y--)
    {
      if ( ((Int)(cuPelY + y) < 0) || ((Int) (cuPelY + y + iRoiHeight) >= iPicHeight)) //NOTE: RExt - is this still necessary?
      {
        piRefY -= iRefStride;
        continue;
      }

      for(Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
      {

        if (((Int)(cuPelX + x) < 0) || ((Int) (cuPelX + x + iRoiWidth) >= iPicWidth)) //NOTE: RExt - is this still necessary?
        {
          continue;
        }

        Int iTempX = x + iRelCUPelX + iRoiWidth - 1;
        Int iTempY = y + iRelCUPelY + iRoiHeight - 1;
        if ((iTempX >= 0) && (iTempY >= 0))
        {
          Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInCtuWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
          Int iTempZscanIdx  = g_auiRasterToZscan[iTempRasterIdx];
          if(iTempZscanIdx >= pcCU->getZorderIdxInCtu())
            continue;
        }

#if SCM_FIX_TICKET_1401
        if (!isValidIntraBCSearchArea(pcCU, x, y, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartOffset))
#else
        if (!isValidIntraBCSearchArea(pcCU, iPartIdx, x, chromaROIStartXInPixels, y, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartOffset))
#endif
        {
          continue;
        }

        piRefSrch = piRefY + x;
        m_cDistParam.pCur = piRefSrch;

        m_cDistParam.bitDepth = pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );
        uiSad = m_cDistParam.DistFunc( &m_cDistParam );

        uiSad += m_pcRdCost->getCostMultiplePreds( x, y);
        if ( uiSad < uiSadBest )
        {
          uiSadBest = uiSad;
          iBestX    = x;
          iBestY    = y;
        }
      }

      piRefY -= iRefStride;
    }
  }

#if SCM_IBC_CR_INTERPOLATION_ENABLE
  iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset,iPartIdx);
#else
  iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
#endif 
  iBestX       = cMVCand[iBestCandIdx].getHor();
  iBestY       = cMVCand[iBestCandIdx].getVer();
  uiSadBest    = uiSadBestCand[iBestCandIdx];  
  rcMv.set( iBestX, iBestY );
  ruiSAD       = uiSadBest;

end:
  if(iRoiWidth+iRoiHeight > 8)
  {
    m_uiNumBVs = MergeCandLists(m_acBVs, m_uiNumBVs, cMVCand, CHROMA_REFINEMENT_CANDIDATES, false);

    if(iRoiWidth+iRoiHeight==32)
    {
      m_uiNumBV16s = m_uiNumBVs;
    }
  }

  return;
}


Int TEncSearch::xIntraBCHashTableIndex(TComDataCU* pcCU, Int pos_X, Int pos_Y, Int width, Int height, Bool isRec)
{
  TComPicYuv* HashPic;
  Pel*        plane;
  Int         iNumComp    = 1;  
  UInt        uiHashIdx   = 0;  
  UInt        grad        = 0;
  UInt        avgDC1      = 0;
  UInt        avgDC2      = 0;
  UInt        avgDC3      = 0;
  UInt        avgDC4      = 0;
  UInt        gradX       = 0;
  UInt        gradY       = 0;
  Int         iPicWidth   = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  Int         iPicHeight  = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();
  Int         iTotalSamples = width * height;
  if(iNumComp == 3)
  {
    iTotalSamples = getTotalSamples(width, height,pcCU->getSlice()->getSPS()->getChromaFormatIdc());
  }

  assert((pos_X + width) <= iPicWidth);
  assert((pos_Y + height) <= iPicHeight);

  if(isRec)
  {
    HashPic = pcCU->getPic()->getPicYuvRec();
  }
  else
  {
    if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && m_pcEncCfg->getRGBFormatFlag() )
    {
      HashPic = pcCU->getPic()->getPicYuvResi();
    }
    else
    {
      HashPic = pcCU->getPic()->getPicYuvOrg();
    }
  }

  for(Int chan=0; chan < iNumComp; chan++)
  {
    const ComponentID compID=ComponentID(chan);
    Int iBitdepth   = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
    Int cxstride = HashPic->getStride(compID);
    Int cxpos_X = pos_X >> HashPic->getComponentScaleX(compID);
    Int cxpos_Y = pos_Y >> HashPic->getComponentScaleY(compID);
    Int cxwidth = width >> HashPic->getComponentScaleX(compID);
    Int cxheight = height >> HashPic->getComponentScaleY(compID);

    plane = HashPic->getAddr(compID) + cxpos_Y * cxstride + cxpos_X;

    for (UInt y = 1; y < (cxheight >> 1); y++)
    {
      for (UInt x = 1; x < (cxwidth >> 1); x++)
      {
        avgDC1 += (plane[y*cxstride+x] >> (iBitdepth - 8));
        gradX = abs(plane[y*cxstride+x] - plane[y*cxstride+x-1]) >> (iBitdepth - 8);
        gradY = abs(plane[y*cxstride+x] - plane[(y-1)*cxstride+x]) >> (iBitdepth - 8);
        grad += (gradX + gradY) >> 1;
      }
    }

    for (UInt y = (cxheight >> 1); y < cxheight; y++)
    {
      for (UInt x = 1; x < (cxwidth >> 1); x++)
      {
        avgDC2 += (plane[y*cxstride+x] >> (iBitdepth - 8));
        gradX = abs(plane[y*cxstride+x] - plane[y*cxstride+x-1]) >> (iBitdepth - 8);
        gradY = abs(plane[y*cxstride+x] - plane[(y-1)*cxstride+x]) >> (iBitdepth - 8);
        grad += (gradX + gradY) >> 1;
      }
    }

    for (UInt y = 1; y < (cxheight >> 1); y++)
    {
      for (UInt x = (cxwidth >> 1); x < cxwidth; x++)
      {
        avgDC3 += plane[y*cxstride+x] >> (iBitdepth - 8);
        gradX = abs(plane[y*cxstride+x] - plane[y*cxstride+x-1]) >> (iBitdepth - 8);
        gradY = abs(plane[y*cxstride+x] - plane[(y-1)*cxstride+x]) >> (iBitdepth - 8);
        grad += (gradX + gradY) >> 1;
      }
    }

    for (UInt y = (cxheight >> 1); y < cxheight; y++)
    {
      for (UInt x = (cxwidth >> 1); x < cxwidth; x++)
      {
        avgDC4 += plane[y*cxstride+x] >> (iBitdepth - 8);
        gradX = abs(plane[y*cxstride+x] - plane[y*cxstride+x-1]) >> (iBitdepth - 8);
        gradY = abs(plane[y*cxstride+x] - plane[(y-1)*cxstride+x]) >> (iBitdepth - 8);
        grad += (gradX + gradY) >> 1;
      }
    }
  }

  avgDC1 = (avgDC1 << 2)/(iTotalSamples);
  avgDC2 = (avgDC2 << 2)/(iTotalSamples);
  avgDC3 = (avgDC3 << 2)/(iTotalSamples);
  avgDC4 = (avgDC4 << 2)/(iTotalSamples);

  grad = grad/(iTotalSamples);

  if(grad < 5 - (m_pcEncCfg->getRGBFormatFlag()))
  {
    return -1;
  }

  grad   = (grad >> 4) & 0xf; // 4 bits

  avgDC1 = (avgDC1>>5) & 0x7; // 3 bits 
  avgDC2 = (avgDC2>>5) & 0x7;
  avgDC3 = (avgDC3>>5) & 0x7;
  avgDC4 = (avgDC4>>5) & 0x7;

  uiHashIdx = (avgDC1 << 13) + (avgDC2 << 10) + (avgDC3 << 7) + (avgDC4 << 4) + grad;
  
  assert(uiHashIdx <= 0XFFFF);

  return uiHashIdx;
}

Void TEncSearch::xIntraBCHashSearch( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, TComMv* pcMvPred, TComMv& rcMv, UInt uiIntraBCECost)
{
  UInt      uiPartAddr;
  Int       iRoiWidth;
  Int       iRoiHeight;

  TComYuv*    pcYuv = pcYuvOrg;

  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

  Int        iOrgHashIndex;

  Distortion  uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  TComMv      cMVCand[CHROMA_REFINEMENT_CANDIDATES];
  for(int iCand = 0; iCand < CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    uiSadBestCand[iCand] = std::numeric_limits<Distortion>::max();
    cMVCand[iCand].set(0,0);
  }  

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight ); 

  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
    iRoiWidth,
    iRoiHeight,
    pcYuv->getStride(COMPONENT_Y),
    pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

  iOrgHashIndex = xIntraBCHashTableIndex(pcCU, pcCU->getCUPelX(), pcCU->getCUPelY(), iRoiWidth, iRoiHeight, false);

  if(iOrgHashIndex < 0)
  {
    return;
  }

  IntraBCHashNode* HashLinklist = getHashLinklist(0, iOrgHashIndex);  //Intra full frame hash search only for 8x8

  Pel*        piRefY      = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y);
  Int         iRefStride  = pcCU->getPic()->getPicYuvRec()->getStride(COMPONENT_Y);

  // disable weighted prediction
  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
  m_pcRdCost->setPredictor(*pcMvPred);
  m_pcRdCost->setCostScale  ( 0 );
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  setDistParamComp(COMPONENT_Y);
  m_cDistParam.bitDepth  = pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );
  m_cDistParam.iRows     = 4;//to calculate the sad line by line;
  m_cDistParam.iSubShift = 0;

  Int  cuPelX     = pcCU->getCUPelX();
  Int  cuPelY     = pcCU->getCUPelY();
  const UInt uiMaxCuWidth         = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt uiMaxCuHeight        = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  const UInt uiSearchWidth        = m_pcEncCfg->getUseIntraBCFullFrameSearch() ? 0 : pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCSearchWidthInCTUs() );
  const UInt uiNonHashSearchWidth = pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() );
  const UInt uiCtuPelX            = (cuPelX / uiMaxCuWidth) * uiMaxCuWidth;
  const UInt uiCtuPelY            = (cuPelY / uiMaxCuHeight) * uiMaxCuHeight;

  Distortion  uiSad;

  Int         iTempX;
  Int         iTempY;
  Pel*  piRefSrch;

  xIntraBCSearchMVCandUpdate(uiIntraBCECost, rcMv.getHor(), rcMv.getVer(), uiSadBestCand, cMVCand);

#if SCM_T0056_IBC_VALIDATE_TILES
  const Int        iRelCUPelX    = cuPelX % uiMaxCuWidth;
  const Int        iRelCUPelY    = cuPelY % uiMaxCuHeight;
  const ChromaFormat format = pcCU->getPic()->getChromaFormat();
  const Int chromaROIWidthInPixels  = (((format == CHROMA_420) || (format == CHROMA_422)) && (iRoiWidth  == 4) && ((iRelCUPelX & 0x4) != 0)) ? (iRoiWidth  * 2) : iRoiWidth;
  const Int chromaROIHeightInPixels = (((format == CHROMA_420)                          ) && (iRoiHeight == 4) && ((iRelCUPelY & 0x4) != 0)) ? (iRoiHeight * 2) : iRoiHeight;
#if !SCM_FIX_TICKET_1401
  const Int chromaROIStartXInPixels = iRelCUPelX + iRoiWidth  - chromaROIWidthInPixels;
  const Int chromaROIStartYInPixels = iRelCUPelY + iRoiHeight - chromaROIHeightInPixels;
#endif
#endif

  while(HashLinklist)
  {
    iTempX = HashLinklist->pos_X;
    iTempY = HashLinklist->pos_Y;

    if( !m_pcEncCfg->getUseIntraBCFullFrameSearch() )    // if full frame search is disabled, then apply following constraints on search range
    {
      const UInt uiRefCuX    = iTempX/uiMaxCuWidth;
      const UInt uiRefCuY    = iTempY/uiMaxCuHeight;
      const UInt uiRefCuPelX = uiRefCuX*uiMaxCuWidth;
      const UInt uiRefCuPelY = uiRefCuY*uiMaxCuHeight;
      if( uiRefCuPelX+uiSearchWidth < uiCtuPelX          // don't search left area of IntraBlockCopySearchWidth
        || uiRefCuPelX+uiNonHashSearchWidth >= uiCtuPelX // don't search in the area that has been already searched in HashBasedIntraBlockCopySearch
        || uiRefCuPelY != uiCtuPelY )                    // only search current CTU row
      {
        HashLinklist = HashLinklist->next;
        continue;
      }
    }

#if SCM_IBC_CR_INTERPOLATION_ENABLE
    Int xBv = iTempX - cuPelX;
    Int yBv = iTempY - cuPelY;
    if ( !isBlockVectorValid(cuPelX,cuPelY,iRoiWidth,iRoiHeight,pcCU,uiPartAddr,0,0,xBv,yBv,pcCU->getSlice()->getSPS()->getMaxCUWidth()))
    {
      HashLinklist = HashLinklist->next;
      continue;
    }
#else
     Int uiRefCuX   = (iTempX + iRoiWidth  - 1)/uiMaxCuWidth;
     Int uiRefCuY   = (iTempY + iRoiHeight - 1)/uiMaxCuHeight;     
     Int uiCuPelX   = (cuPelX / uiMaxCuWidth);
     Int uiCuPelY   = (cuPelY / uiMaxCuHeight);     

    if(((Int)(uiRefCuX - uiCuPelX) > (Int)((uiCuPelY - uiRefCuY))))
    {
      HashLinklist = HashLinklist->next;
      continue;
    }
#endif 
#if SCM_T0056_IBC_VALIDATE_TILES
    Int xPred = iTempX - cuPelX;
    Int yPred = iTempY - cuPelY;

#if SCM_FIX_TICKET_1401
    Bool validCand = isValidIntraBCSearchArea(pcCU, xPred, yPred, chromaROIWidthInPixels, chromaROIHeightInPixels, uiPartAddr);
#else
    Bool validCand = isValidIntraBCSearchArea(pcCU, iPartIdx, xPred, chromaROIStartXInPixels, yPred, chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels,uiPartAddr);
#endif
    if( !validCand )
    {
      HashLinklist = HashLinklist->next;
      continue;
    }
#endif

    uiSad = 0;//m_pcRdCost->getCost( iTempX - cuPelX, iTempY - cuPelY);

    for(int r = 0; r < iRoiHeight; )
    {
      piRefSrch = piRefY + iTempY * iRefStride + r*iRefStride + iTempX;
      m_cDistParam.pCur = piRefSrch;
      m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

      uiSad += m_cDistParam.DistFunc( &m_cDistParam );
      if(uiSad > uiSadBestCand[CHROMA_REFINEMENT_CANDIDATES-1])
        break;

      r += 4;
    }

    if(uiSad <= 16)
    {
      uiSad += m_pcRdCost->getCostMultiplePreds( iTempX - cuPelX, iTempY - cuPelY);
      xIntraBCSearchMVCandUpdate(uiSad, iTempX - cuPelX, iTempY - cuPelY, uiSadBestCand, cMVCand);
      break;
    }

    uiSad += m_pcRdCost->getCostMultiplePreds( iTempX - cuPelX, iTempY - cuPelY);
    xIntraBCSearchMVCandUpdate(uiSad, iTempX - cuPelX, iTempY - cuPelY, uiSadBestCand, cMVCand);
    HashLinklist = HashLinklist->next;
  }

#if SCM_IBC_CR_INTERPOLATION_ENABLE
  Int  iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, 0,iPartIdx);
#else
  Int iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, 0);
#endif 
  rcMv = cMVCand[iBestCandIdx];

  m_uiNumBVs = MergeCandLists(m_acBVs, m_uiNumBVs, cMVCand, CHROMA_REFINEMENT_CANDIDATES, false);

  return;
}

Void TEncSearch::xIntraBCHashTableUpdate(TComDataCU* pcCU, Bool isRec)
{
  Int         iRoiWidth = 8;
  Int         iRoiHeight = 8;
  Int         cuPelX     = pcCU->getCUPelX();
  Int         cuPelY     = pcCU->getCUPelY();
  Int         iTempX;
  Int         iTempY;
  Int         iPicWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  Int         iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();
  UInt        uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
  UInt        uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();
  Int         iOrgHashIndex;
  IntraBCHashNode* NewHashNode;
  TComPicSym *pcSym = pcCU->getPic()->getPicSym();
  UInt       startCtu = !pcCU->getSlice()->getSliceMode() ? 0
                      : pcSym->getCtuTsToRsAddrMap(pcCU->getSlice()->getSliceSegmentCurStartCtuTsAddr());
  UInt       refY     = pcSym->getCtu(startCtu)->getCUPelY();


#if SCM_T0056_IBC_VALIDATE_TILES
  const UInt lcuWidth = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt lcuHeight = pcCU->getSlice()->getSPS()->getMaxCUHeight();

  const UInt curTileIdx = pcCU->getPic()->getPicSym()->getTileIdxMap( pcCU->getCtuRsAddr() );
  TComTile* curTile = pcCU->getPic()->getPicSym()->getTComTile( curTileIdx );

  const Int tileAreaRight  = (curTile->getRightEdgePosInCtus() + 1) * lcuWidth;
  const Int tileAreaBottom = (curTile->getBottomEdgePosInCtus() + 1) * lcuHeight;

  const Int tileAreaLeft   = tileAreaRight - curTile->getTileWidthInCtus() * lcuWidth;
  const Int tileAreaTop    = tileAreaBottom - curTile->getTileHeightInCtus() * lcuHeight;

  Int j_start = ( tileAreaTop  == cuPelY) ? 7 : 0;
  Int i_start = ( tileAreaLeft == cuPelX) ? 7 : 0;


  for(Int j = j_start; j < uiMaxCuHeight; j++)
  {
    iTempY = cuPelY - iRoiHeight + 1  + j;
    if ( pcCU->getSlice()->getSliceMode() && iTempY < refY )
    {
      continue;
    }
    for(Int i = i_start; i < uiMaxCuWidth; i++)
    {
#else
  for(int j = 0; j < uiMaxCuHeight; j++)
  {
    iTempY = cuPelY - iRoiHeight + 1  + j;
    if ( pcCU->getSlice()->getSliceMode() && iTempY < refY )
    {
      continue;
    }
    for(int i = 0; i < uiMaxCuWidth; i++)
    {
#endif
      iTempX = cuPelX - iRoiWidth + 1 + i;

      if((iTempX < 0) || (iTempY < 0) || ((iTempX + iRoiWidth) >= iPicWidth) || ((iTempY + iRoiHeight) >= iPicHeight))
      {
        continue;
      }
      if( pcCU->getSlice()->getSliceMode() )
      {
        Int      ctuX = iTempX / pcCU->getSlice()->getSPS()->getMaxCUWidth();
        Int      ctuY = iTempY / pcCU->getSlice()->getSPS()->getMaxCUHeight();
        UInt   refCtu = ctuX + pcSym->getFrameWidthInCtus()*ctuY;
        if ( refCtu < startCtu )
        {
          continue;
        }
      }

      iOrgHashIndex = xIntraBCHashTableIndex(pcCU, iTempX, iTempY, iRoiWidth, iRoiHeight, isRec);

      if(iOrgHashIndex < 0)
      {
        continue;
      }

      NewHashNode = new IntraBCHashNode;

      assert(NewHashNode);

      NewHashNode->pos_X = iTempX;
      NewHashNode->pos_Y = iTempY;
      setHashLinklist(NewHashNode, 0, iOrgHashIndex); //Intra full frame hash search only for 8x8
    }
  }
}

Void TEncSearch::xClearIntraBCHashTable()
{
  if(m_pcIntraBCHashTable)
  {
    for(int iDepth = 0; iDepth < INTRABC_HASH_DEPTH; iDepth++)
    {
      if(m_pcIntraBCHashTable[iDepth])
      {
        for(int iIdx = 0; iIdx < INTRABC_HASH_TABLESIZE; iIdx++)
        {
          if(m_pcIntraBCHashTable[iDepth][iIdx] == NULL)
            continue;
          else
          {
            while(m_pcIntraBCHashTable[iDepth][iIdx]->next)
            {
              IntraBCHashNode* TempNode = m_pcIntraBCHashTable[iDepth][iIdx]->next;
              m_pcIntraBCHashTable[iDepth][iIdx]->next = m_pcIntraBCHashTable[iDepth][iIdx]->next->next;

              delete TempNode;
            }

            delete m_pcIntraBCHashTable[iDepth][iIdx];

            m_pcIntraBCHashTable[iDepth][iIdx] = NULL;
          }
        }
      }
    }
  }
}

Void TEncSearch::setHashLinklist(IntraBCHashNode*& HashLinklist, UInt uiDepth, UInt uiHashIdx)
{ 
  HashLinklist->next = m_pcIntraBCHashTable[uiDepth][uiHashIdx];
  m_pcIntraBCHashTable[uiDepth][uiHashIdx] = HashLinklist;  
}


// AMVP
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, Distortion* puiDistBiP )
{
  AMVPInfo*  pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  TComMv     cBestMv;
  Int        iBestIdx   = 0;
  TComMv     cZeroMv;
  TComMv     cMvPred;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  UInt       uiPartAddr = 0;
  Int        iRoiWidth, iRoiHeight;
  Int        i;

  pcCU->getPartIndexAndSize( uiPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );
  // Fill the MV Candidates
  if (!bFilled)
  {
    pcCU->fillMvpCand( uiPartIdx, uiPartAddr, eRefPicList, iRefIdx, pcAMVPInfo );
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->m_acMvCand[0];
  if (pcAMVPInfo->iN <= 1)
  {
    rcMvPred = cBestMv;

    pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));

    if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefPicList==REF_PIC_LIST_1)
    {
      (*puiDistBiP) = xGetTemplateCost( pcCU, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, rcMvPred, 0, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
    }
    return;
  }

  if (bFilled)
  {
    assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
    return;
  }

  m_cYuvPredTemp.clear();
  //-- Check Minimum Cost.
  for ( i = 0 ; i < pcAMVPInfo->iN; i++)
  {
    Distortion uiTmpCost;
    uiTmpCost = xGetTemplateCost( pcCU, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      cBestMv   = pcAMVPInfo->m_acMvCand[i];
      iBestIdx  = i;
      (*puiDistBiP) = uiTmpCost;
    }
  }

  m_cYuvPredTemp.clear();

  // Setting Best MVP
  rcMvPred = cBestMv;
  pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
  return;
}

UInt TEncSearch::xGetMvpIdxBits(Int iIdx, Int iNum)
{
  assert(iIdx >= 0 && iNum >= 0 && iIdx < iNum);

  if (iNum == 1)
  {
    return 0;
  }

  UInt uiLength = 1;
  Int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  Bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

Void TEncSearch::xGetBlkBits( PartSize eCUMode, Bool bPSlice, Int iPartIdx, UInt uiLastMode, UInt uiBlkBit[3])
{
  if ( eCUMode == SIZE_2Nx2N )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else if ( (eCUMode == SIZE_2NxN || eCUMode == SIZE_2NxnU) || eCUMode == SIZE_2NxnD )
  {
    UInt aauiMbBits[2][3][3] = { { {0,0,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7,5,7}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( (eCUMode == SIZE_Nx2N || eCUMode == SIZE_nLx2N) || eCUMode == SIZE_nRx2N )
  {
    UInt aauiMbBits[2][3][3] = { { {0,2,3}, {0,0,0}, {0,0,0} } , { {5,7,7}, {7-2,7-2,9-2}, {9-3,9-3,9-3} } };
    if ( bPSlice )
    {
      uiBlkBit[0] = 3;
      uiBlkBit[1] = 0;
      uiBlkBit[2] = 0;
    }
    else
    {
      ::memcpy( uiBlkBit, aauiMbBits[iPartIdx][uiLastMode], 3*sizeof(UInt) );
    }
  }
  else if ( eCUMode == SIZE_NxN )
  {
    uiBlkBit[0] = (! bPSlice) ? 3 : 1;
    uiBlkBit[1] = 3;
    uiBlkBit[2] = 5;
  }
  else
  {
    printf("Wrong!\n");
    assert( 0 );
  }
}

Void TEncSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}

Void TEncSearch::xCheckBestMVP ( TComDataCU* pcCU, RefPicList eRefPicList, TComMv cMv, TComMv& rcMvPred, Int& riMVPIdx, UInt& ruiBits, Distortion& ruiCost )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();
#if SCM_AMVR_UNIFICATION
  if(!pcCU->getSlice()->getUseIntegerMv() )  
#endif
  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);

  if (pcAMVPInfo->iN < 2)
  {
    return;
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(0) );
  m_pcRdCost->setCostScale ( 0    );

  Int iBestMVPIdx = riMVPIdx;

  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits  = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  Int iBestMvBits = iOrgMvBits;

  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    m_pcRdCost->setPredictor( pcAMVPInfo->m_acMvCand[iMVPIdx] );

    Int iMvBits = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    UInt uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion TEncSearch::xGetTemplateCost( TComDataCU* pcCU,
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
                                         )
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  TComPicYuv* pcPicYuvRef = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec();
#if !SCM_AMVR_UNIFICATION
  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
    cMvCand <<= 2;
  }
#endif
  pcCU->clipMv( cMvCand );

  // prediction pattern
  if ( pcCU->getSlice()->testWeightPred() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, true, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, false, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
  }

  if ( pcCU->getSlice()->testWeightPred() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xWeightedPredictionUni( pcCU, pcTemplateCand, uiPartAddr, iSizeX, iSizeY, eRefPicList, pcTemplateCand, iRefIdx );
  }

  // calc distortion

  uiCost = m_pcRdCost->getDistPart( pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA), pcTemplateCand->getAddr(COMPONENT_Y, uiPartAddr), pcTemplateCand->getStride(COMPONENT_Y), pcOrgYuv->getAddr(COMPONENT_Y, uiPartAddr), pcOrgYuv->getStride(COMPONENT_Y), iSizeX, iSizeY, COMPONENT_Y, DF_SAD );
  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum], uiCost, false, DF_SAD );
  return uiCost;
}




Void TEncSearch::xMotionEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, RefPicList eRefPicList, TComMv* pcMvPred, Int iRefIdxPred, TComMv& rcMv, UInt& ruiBits, Distortion& ruiCost, Bool bBi  )
{
  UInt          uiPartAddr;
  Int           iRoiWidth;
  Int           iRoiHeight;

  TComMv        cMvHalf, cMvQter;
  TComMv        cMvSrchRngLT;
  TComMv        cMvSrchRngRB;

  TComYuv*      pcYuv = pcYuvOrg;

  assert(eRefPicList < MAX_NUM_REF_LIST_ADAPT_SR && iRefIdxPred<Int(MAX_IDX_ADAPT_SR));
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];

  Int           iSrchRng      = ( bBi ? m_bipredSearchRange : m_iSearchRange );
  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

  Double        fWeight       = 1.0;

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  if ( bBi )
  {
    TComYuv*  pcYuvOther = &m_acYuvPred[1-(Int)eRefPicList];
    pcYuv                = &m_cYuvPredTemp;

    pcYuvOrg->copyPartToPartYuv( pcYuv, uiPartAddr, iRoiWidth, iRoiHeight );

    pcYuv->removeHighFreq( pcYuvOther, uiPartAddr, iRoiWidth, iRoiHeight, pcCU->getSlice()->getSPS()->getBitDepths().recon, m_pcEncCfg->getClipForBiPredMeEnabled() );

    fWeight = 0.5;
  }

  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
                             iRoiWidth,
                             iRoiHeight,
                             pcYuv->getStride(COMPONENT_Y),
                             pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

  Pel*        piRefY      = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiPartAddr );
  Int         iRefStride  = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getStride(COMPONENT_Y);

  TComMv      cMvPred = *pcMvPred;

  if ( bBi )
  {
    xSetSearchRange   ( pcCU, rcMv   , iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  }
  else
  {
    xSetSearchRange   ( pcCU, cMvPred, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

  m_pcRdCost->setPredictor  ( *pcMvPred );
  m_pcRdCost->setCostScale  ( 2 );

  setWpScalingDistParam( pcCU, iRefIdxPred, eRefPicList );

  m_currRefPicList = eRefPicList;
  m_currRefPicIndex = iRefIdxPred;
  m_bSkipFracME = false;

  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
#if !SCM_AMVR_UNIFICATION
    m_pcRdCost->setCostScale( 0 );
#endif 
    m_bSkipFracME = true;
  }

  //  Do integer search
  if ( !m_iFastSearch || bBi )
  {
    xPatternSearch      ( pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost );
  }
  else
  {
    rcMv = *pcMvPred;
    const TComMv *pIntegerMv2Nx2NPred=0;
    if (pcCU->getPartitionSize(0) != SIZE_2Nx2N || pcCU->getDepth(0) != 0)
    {
      pIntegerMv2Nx2NPred = &(m_integerMv2Nx2N[eRefPicList][iRefIdxPred]);
    }
    xPatternSearchFast  ( pcCU, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, pIntegerMv2Nx2NPred );
    if (pcCU->getPartitionSize(0) == SIZE_2Nx2N)
    {
      m_integerMv2Nx2N[eRefPicList][iRefIdxPred] = rcMv;
    }
  }

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
  m_pcRdCost->setCostScale ( 1 );

  const Bool bIsLosslessCoded = pcCU->getCUTransquantBypass(uiPartAddr) != 0;
  xPatternSearchFracDIF( bIsLosslessCoded, pcCU, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost );

  m_pcRdCost->setCostScale( 0 );
#if !SCM_AMVR_UNIFICATION
  if ( !pcCU->getSlice()->getUseIntegerMv() )
#endif 
  {
    rcMv <<= 2;
    rcMv += (cMvHalf <<= 1);
    rcMv +=  cMvQter;
  }

  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );

  ruiBits      += uiMvBits;
  ruiCost       = (Distortion)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
}




Void TEncSearch::xSetSearchRange ( TComDataCU* pcCU, TComMv& cMvPred, Int iSrchRng, TComMv& rcMvSrchRngLT, TComMv& rcMvSrchRngRB )
{
  Int  iMvShift = 2;
  TComMv cTmpMvPred = cMvPred;
  pcCU->clipMv( cTmpMvPred );

  rcMvSrchRngLT.setHor( cTmpMvPred.getHor() - (iSrchRng << iMvShift) );
  rcMvSrchRngLT.setVer( cTmpMvPred.getVer() - (iSrchRng << iMvShift) );

  rcMvSrchRngRB.setHor( cTmpMvPred.getHor() + (iSrchRng << iMvShift) );
  rcMvSrchRngRB.setVer( cTmpMvPred.getVer() + (iSrchRng << iMvShift) );
  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );

  rcMvSrchRngLT >>= iMvShift;
  rcMvSrchRngRB >>= iMvShift;
}




Void TEncSearch::xPatternSearch( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, TComMv* pcMvSrchRngLT, TComMv* pcMvSrchRngRB, TComMv& rcMv, Distortion& ruiSAD )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  Distortion  uiSad;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  Int         iBestX = 0;
  Int         iBestY = 0;

  Pel*  piRefSrch;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  // fast encoder decision: use subsampled SAD for integer ME
  if ( m_pcEncCfg->getUseFastEnc() )
  {
    if ( m_cDistParam.iRows > 8 )
    {
      m_cDistParam.iSubShift = 1;
    }
  }

  piRefY += (iSrchRngVerTop * iRefStride);
  for ( Int y = iSrchRngVerTop; y <= iSrchRngVerBottom; y++ )
  {
    for ( Int x = iSrchRngHorLeft; x <= iSrchRngHorRight; x++ )
    {
      //  find min. distortion position
      piRefSrch = piRefY + x;
      m_cDistParam.pCur = piRefSrch;

      setDistParamComp(COMPONENT_Y);

      m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
      uiSad = m_cDistParam.DistFunc( &m_cDistParam );

      // motion cost
      uiSad += m_pcRdCost->getCost( x, y );

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
      }
    }
    piRefY += iRefStride;
  }

  rcMv.set( iBestX, iBestY );

  ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY );
  return;
}



Void TEncSearch::xPatternSearchFast( TComDataCU*   pcCU,
                                     TComPattern*  pcPatternKey,
                                     Pel*          piRefY,
                                     Int           iRefStride,
                                     TComMv*       pcMvSrchRngLT,
                                     TComMv*       pcMvSrchRngRB,
                                     TComMv       &rcMv,
                                     Distortion   &ruiSAD,
                                     const TComMv* pIntegerMv2Nx2NPred )
{
  assert (MD_LEFT < NUM_MV_PREDICTORS);
  pcCU->getMvPredLeft       ( m_acMvPredictors[MD_LEFT] );
  assert (MD_ABOVE < NUM_MV_PREDICTORS);
  pcCU->getMvPredAbove      ( m_acMvPredictors[MD_ABOVE] );
  assert (MD_ABOVE_RIGHT < NUM_MV_PREDICTORS);
  pcCU->getMvPredAboveRight ( m_acMvPredictors[MD_ABOVE_RIGHT] );

  switch ( m_iFastSearch )
  {
    case 1:
      xTZSearch( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
      break;

    case 2:
      xTZSearchSelective( pcCU, pcPatternKey, piRefY, iRefStride, pcMvSrchRngLT, pcMvSrchRngRB, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
      break;
    default:
      break;
  }
}




Void TEncSearch::xTZSearch( TComDataCU*  pcCU,
                            TComPattern* pcPatternKey,
                            Pel*         piRefY,
                            Int          iRefStride,
                            TComMv*      pcMvSrchRngLT,
                            TComMv*      pcMvSrchRngRB,
                            TComMv      &rcMv,
                            Distortion  &ruiSAD,
                            const TComMv* pIntegerMv2Nx2NPred )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  TZ_SEARCH_CONFIGURATION

  UInt uiSearchRange = m_iSearchRange;
#if !SCM_AMVR_UNIFICATION
  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
    rcMv <<= 2;
  }
#endif 
  pcCU->clipMv( rcMv );
  rcMv >>= 2;
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;

  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < NUM_MV_PREDICTORS; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
      cMv >>= 2;
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }

  if (pIntegerMv2Nx2NPred != 0)
  {
    TComMv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    pcCU->clipMv( integerMv2Nx2NPred );
    integerMv2Nx2NPred >>= 2;
    xTZSearchHelp(pcPatternKey, cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

    // reset search range
    TComMv cMvSrchRngLT;
    TComMv cMvSrchRngRB;
    Int iSrchRng = m_iSearchRange;
    TComMv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pcCU, currBestMv, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
    iSrchRngHorLeft   = cMvSrchRngLT.getHor();
    iSrchRngHorRight  = cMvSrchRngRB.getHor();
    iSrchRngVerTop    = cMvSrchRngLT.getVer();
    iSrchRngVerBottom = cMvSrchRngRB.getVer();
  }

  if ( m_pcEncCfg->getUseHashBasedME() && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N )
  {
    TComMv otherMvps[5];
    Int numberOfOtherMvps;
    numberOfOtherMvps = xHashInterPredME( pcCU, pcCU->getWidth( 0 ), pcCU->getHeight( 0 ), m_currRefPicList, m_currRefPicIndex, otherMvps );
    for ( Int i=0; i<numberOfOtherMvps; i++ )
    {
      xTZSearchHelp( pcPatternKey, cStruct, otherMvps[i].getHor(), otherMvps[i].getVer(), 0, 0 );
    }

    if ( numberOfOtherMvps > 0 )
    {
      // write out best match
      rcMv.set( cStruct.iBestX, cStruct.iBestY );
      ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );
      m_bSkipFracME = true;

      return;
    }
  }

  // start search
  Int  iDist = 0;
  Int  iStartX = cStruct.iBestX;
  Int  iStartY = cStruct.iBestY;

  // first search
  for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }
    else
    {
      xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  // test whether zero Mv is a better start point than Median predictor
  if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
    if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
    {
      // test its neighborhood
      for ( iDist = 1; iDist <= (Int)uiSearchRange; iDist*=2 )
      {
        xTZ8PointDiamondSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, 0, 0, iDist );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
  }

  // raster search if distance is too big
  if ( bEnableRasterSearch && ( ((Int)(cStruct.uiBestDistance) > iRaster) || bAlwaysRasterSearch ) )
  {
    cStruct.uiBestDistance = iRaster;
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += iRaster )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += iRaster )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, iRaster );
      }
    }
  }

  // raster refinement
  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // start refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );
}


Void TEncSearch::xTZSearchSelective( TComDataCU*   pcCU,
                                     TComPattern*  pcPatternKey,
                                     Pel*          piRefY,
                                     Int           iRefStride,
                                     TComMv*       pcMvSrchRngLT,
                                     TComMv*       pcMvSrchRngRB,
                                     TComMv       &rcMv,
                                     Distortion   &ruiSAD,
                                     const TComMv* pIntegerMv2Nx2NPred )
{
  SEL_SEARCH_CONFIGURATION

  Int   iSrchRngHorLeft         = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight        = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop          = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom       = pcMvSrchRngRB->getVer();
  Int   iFirstSrchRngHorLeft    = 0;
  Int   iFirstSrchRngHorRight   = 0;
  Int   iFirstSrchRngVerTop     = 0;
  Int   iFirstSrchRngVerBottom  = 0;
  Int   iStartX                 = 0;
  Int   iStartY                 = 0;
  Int   iBestX                  = 0;
  Int   iBestY                  = 0;
  Int   iDist                   = 0;

#if !SCM_AMVR_UNIFICATION
  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
    rcMv <<= 2;
  }
#endif 
  pcCU->clipMv( rcMv );
  rcMv >>= 2;
  // init TZSearchStruct
  IntTZSearchStruct cStruct;
  cStruct.iYStride    = iRefStride;
  cStruct.piRefY      = piRefY;
  cStruct.uiBestSad   = MAX_UINT;
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( pcPatternKey, cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether one of PRED_A, PRED_B, PRED_C MV is better start point than Median predictor
  if ( bTestOtherPredictedMV )
  {
    for ( UInt index = 0; index < NUM_MV_PREDICTORS; index++ )
    {
      TComMv cMv = m_acMvPredictors[index];
      pcCU->clipMv( cMv );
      cMv >>= 2;
      xTZSearchHelp( pcPatternKey, cStruct, cMv.getHor(), cMv.getVer(), 0, 0 );
    }
  }

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( pcPatternKey, cStruct, 0, 0, 0, 0 );
  }

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    TComMv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred <<= 2;
    pcCU->clipMv( integerMv2Nx2NPred );
    integerMv2Nx2NPred >>= 2;
    xTZSearchHelp(pcPatternKey, cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

    // reset search range
    TComMv cMvSrchRngLT;
    TComMv cMvSrchRngRB;
    Int iSrchRng = m_iSearchRange;
    TComMv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pcCU, currBestMv, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
    iSrchRngHorLeft   = cMvSrchRngLT.getHor();
    iSrchRngHorRight  = cMvSrchRngRB.getHor();
    iSrchRngVerTop    = cMvSrchRngLT.getVer();
    iSrchRngVerBottom = cMvSrchRngRB.getVer();
  }

  if ( m_pcEncCfg->getUseHashBasedME() && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N )
  {
    TComMv otherMvps[5];
    Int numberOfOtherMvps;
    numberOfOtherMvps = xHashInterPredME( pcCU, pcCU->getWidth( 0 ), pcCU->getHeight( 0 ), m_currRefPicList, m_currRefPicIndex, otherMvps );
    for ( Int i=0; i<numberOfOtherMvps; i++ )
    {
      xTZSearchHelp( pcPatternKey, cStruct, otherMvps[i].getHor(), otherMvps[i].getVer(), 0, 0 );
    }

    if ( numberOfOtherMvps > 0 )
    {
      // write out best match
      rcMv.set( cStruct.iBestX, cStruct.iBestY );
      ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );
      m_bSkipFracME = true;

      return;
    }
  }
  
  // Initial search
  iBestX = cStruct.iBestX;
  iBestY = cStruct.iBestY; 
  iFirstSrchRngHorLeft    = ((iBestX - uiSearchRangeInitial) > iSrchRngHorLeft)   ? (iBestX - uiSearchRangeInitial) : iSrchRngHorLeft;
  iFirstSrchRngVerTop     = ((iBestY - uiSearchRangeInitial) > iSrchRngVerTop)    ? (iBestY - uiSearchRangeInitial) : iSrchRngVerTop;
  iFirstSrchRngHorRight   = ((iBestX + uiSearchRangeInitial) < iSrchRngHorRight)  ? (iBestX + uiSearchRangeInitial) : iSrchRngHorRight;  
  iFirstSrchRngVerBottom  = ((iBestY + uiSearchRangeInitial) < iSrchRngVerBottom) ? (iBestY + uiSearchRangeInitial) : iSrchRngVerBottom;    

  Bool bFirstVerScan = true;
  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    Bool bFirstHorScan = true;
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, 1 );
      xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, 2, !bFirstHorScan, !bFirstVerScan );
      if( bFirstHorScan )
        bFirstHorScan = false;
    }
    if( bFirstVerScan )
      bFirstVerScan = false;
  }

  Int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = iSrchRngVerTop; iStartY <= iSrchRngVerBottom; iStartY += 1 )
    {
      for ( iStartX = iSrchRngHorLeft; iStartX <= iSrchRngHorRight; iStartX += 1 )
      {
        xTZSearchHelp( pcPatternKey, cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < (Int)uiSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        else
        {
          xTZ8PointSquareSearch  ( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( pcPatternKey, cStruct, pcMvSrchRngLT, pcMvSrchRngRB );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCost( cStruct.iBestX, cStruct.iBestY );

}


Void TEncSearch::xPatternSearchFracDIF(
                                       Bool         bIsLosslessCoded,
                                       TComDataCU*  pcCU,
                                       TComPattern* pcPatternKey,
                                       Pel*         piRefY,
                                       Int          iRefStride,
                                       TComMv*      pcMvInt,
                                       TComMv&      rcMvHalf,
                                       TComMv&      rcMvQter,
                                       Distortion&  ruiCost
                                      )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern(piRefY + iOffset,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride,
                          pcPatternKey->getBitDepthY());

#if !SCM_AMVR_UNIFICATION
  if ( pcCU->getSlice()->getUseIntegerMv() )
  {
    TComMv baseRefMv( 0, 0 );
    rcMvHalf.setZero();
    m_pcRdCost->setCostScale( 0 );
    xExtDIFUpSamplingH( &cPatternRoi );
    rcMvQter = *pcMvInt;    // for mv-cost
    ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
    return;
  }
#endif 

  if ( m_bSkipFracME )
  {
    TComMv baseRefMv( 0, 0 );
    rcMvHalf.setZero();
    m_pcRdCost->setCostScale( 0 );
    xExtDIFUpSamplingH( &cPatternRoi );
    rcMvQter = *pcMvInt;   rcMvQter <<= 2;    // for mv-cost
    ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
    return;
  }


  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi );

  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  TComMv baseRefMv(0, 0);
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 2, rcMvHalf, !bIsLosslessCoded );

  m_pcRdCost->setCostScale( 0 );

  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
}


//! encode residual and calculate rate-distortion for a CU block
#if SCM_U0106_ACT_TU_SIG
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred,
                                            TComYuv* pcYuvResi, TComYuv* pcYuvResiBest, TComYuv* pcYuvRec,
                                            Bool bSkipResidual,
                                            TComYuv* pcYuvNoCorrResi,
                                            ACTRDTestTypes eACTRDTestType
                                            DEBUG_STRING_FN_DECLARE(sDebug) )
#else
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred,
                                            TComYuv* pcYuvResi, TComYuv* pcYuvResiBest, TComYuv* pcYuvRec,
                                            Bool bSkipResidual,
                                            TComYuv* pcYuvNoCorrResi
                                            DEBUG_STRING_FN_DECLARE(sDebug) )
#endif
{
  assert ( !pcCU->isIntra(0) );

  const UInt cuWidthPixels      = pcCU->getWidth ( 0 );
  const UInt cuHeightPixels     = pcCU->getHeight( 0 );
  const Int  numValidComponents = pcCU->getPic()->getNumberValidComponents();
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());

  // The pcCU is not marked as skip-mode at this point, and its m_pcTrCoeff, m_pcArlCoeff, m_puhCbf, m_puhTrIdx will all be 0.
  // due to prior calls to TComDataCU::initEstData(  );
#if SCM_U0106_ACT_TU_SIG
  const Bool iColourTransform = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && ( !pcCU->isLosslessCoded( 0 ) || (sps.getBitDepth( CHANNEL_TYPE_LUMA ) == sps.getBitDepth( CHANNEL_TYPE_CHROMA )) );
#else
  const Bool iColourTransform = pcCU->getColourTransform(0);
#endif
  const Bool extendedPrecision = pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  if ( bSkipResidual ) //  No residual coding : SKIP mode
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );
#if SCM_U0106_ACT_TU_SIG
    pcCU->setColourTransformSubParts(false, 0, pcCU->getDepth(0));
#endif

    pcYuvResi->clear();

    pcYuvPred->copyToPartYuv( pcYuvRec, 0 );
    Distortion distortion = 0;

    for (Int comp=0; comp < numValidComponents; comp++)
    {
      const ComponentID compID=ComponentID(comp);
      const UInt csx=pcYuvOrg->getComponentScaleX(compID);
      const UInt csy=pcYuvOrg->getComponentScaleY(compID);
      distortion += m_pcRdCost->getDistPart( sps.getBitDepth(toChannelType(compID)), pcYuvRec->getAddr(compID), pcYuvRec->getStride(compID), pcYuvOrg->getAddr(compID),
                                               pcYuvOrg->getStride(compID), cuWidthPixels >> csx, cuHeightPixels >> csy, compID);
    }

    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
    m_pcEntropyCoder->resetBits();

    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }

    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    m_pcEntropyCoder->encodeMergeIndex( pcCU, 0, true );

    UInt uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    pcCU->getTotalBits()       = uiBits;
    pcCU->getTotalDistortion() = distortion;
    pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( uiBits, distortion );

    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);

#if DEBUG_STRING
    pcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
    for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
    {
      sDebug+=debug_reorder_data_token[pcCU->isIntraBC(0)?1:0][i];
    }
#endif

    return;
  }

  //  Residual coding.
#if SCM_U0106_ACT_TU_SIG
   pcCU->setSkipFlagSubParts( false, 0, pcCU->getDepth(0) );
#endif
   pcYuvResi->subtract( pcYuvOrg, pcYuvPred, 0, cuWidthPixels );

  TComTURecurse tuLevel0(pcCU, 0);

  Double     nonZeroCost       = 0;
  UInt       nonZeroBits       = 0;
  Distortion nonZeroDistortion = 0;
  Distortion zeroDistortion    = 0;

  if(iColourTransform)
  {
#if SCM_U0106_ACT_TU_SIG
    if(eACTRDTestType == ACT_TWO_CLR || eACTRDTestType == ACT_TRAN_CLR)
    {
#endif
    const UInt uiNumSamplesLuma = cuWidthPixels*cuHeightPixels;
    ::memset( m_pTempPel, 0, sizeof( Pel ) * uiNumSamplesLuma ); 
    zeroDistortion = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ), m_pTempPel, cuWidthPixels, pcYuvResi->getAddr( COMPONENT_Y, 0 ), pcYuvResi->getStride(COMPONENT_Y), cuWidthPixels, cuHeightPixels, COMPONENT_Y ); // initialized with zero residual destortion
    const UInt csx=pcYuvOrg->getComponentScaleX(COMPONENT_Cb);
    const UInt csy=pcYuvOrg->getComponentScaleY(COMPONENT_Cb);
    zeroDistortion += m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_pTempPel, cuWidthPixels >> csx, pcYuvResi->getAddr( COMPONENT_Cb, 0 ), pcYuvResi->getStride(COMPONENT_Cb), cuWidthPixels >> csx, cuHeightPixels >> csy, COMPONENT_Cb ); // initialized with zero residual destortion
    zeroDistortion += m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_pTempPel, cuWidthPixels >> csx, pcYuvResi->getAddr( COMPONENT_Cr, 0 ), pcYuvResi->getStride(COMPONENT_Cr), cuWidthPixels >> csx, cuHeightPixels >> csy, COMPONENT_Cr ); // initialized with zero residual destortion
#if SCM_U0106_ACT_TU_SIG
    }
    if(eACTRDTestType == ACT_TRAN_CLR)
    {
#endif
    pcYuvResi->convert(extendedPrecision, 0, 0, cuWidthPixels, true, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0), pcYuvNoCorrResi);
#if SCM_U0106_ACT_TU_SIG
    }
    else if(eACTRDTestType == ACT_TWO_CLR)
    {
      pcYuvResi->convert(extendedPrecision, 0, 0, cuWidthPixels, true, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0), &(m_pcNoCorrYuvTmp[pcCU->getDepth(0)]));
    }
#endif
  }

  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_CURR_BEST ] );

  if(iColourTransform)
  {
#if SCM_U0106_ACT_TU_SIG
    if(eACTRDTestType == ACT_TWO_CLR)
    {
      xEstimateInterResidualQTTUCSC( pcYuvNoCorrResi, nonZeroCost, nonZeroBits, nonZeroDistortion, tuLevel0, pcYuvResi, eACTRDTestType DEBUG_STRING_PASS_INTO(sDebug) );
    }
    else if(eACTRDTestType == ACT_TRAN_CLR)
    {
      pcCU->setColourTransformSubParts(true, 0 , pcCU->getDepth(0));
      xEstimateInterResidualQT( pcYuvNoCorrResi, nonZeroCost, nonZeroBits, nonZeroDistortion, NULL, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug), pcYuvResi );
    }
    else
    {
      pcCU->setColourTransformSubParts(false, 0, pcCU->getDepth(0));
      xEstimateInterResidualQT( pcYuvResi,  nonZeroCost, nonZeroBits, nonZeroDistortion, &zeroDistortion, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug) );
    }
#else
    xEstimateInterResidualQT( pcYuvNoCorrResi, nonZeroCost, nonZeroBits, nonZeroDistortion, NULL, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug), pcYuvResi );
#endif
  }
  else
  {
#if SCM_U0106_ACT_TU_SIG
    pcCU->setColourTransformSubParts(false, 0, pcCU->getDepth(0));
#endif
    xEstimateInterResidualQT( pcYuvResi,  nonZeroCost, nonZeroBits, nonZeroDistortion, &zeroDistortion, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug) );
  }

  // -------------------------------------------------------
  // set the coefficients in the pcCU, and also calculates the residual data.
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  m_pcEntropyCoder->resetBits();
  m_pcEntropyCoder->encodeQtRootCbfZero( );
  const UInt   zeroResiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  const Double zeroCost     = (pcCU->isLosslessCoded( 0 )) ? (nonZeroCost+1) : (m_pcRdCost->calcRdCost( zeroResiBits, zeroDistortion ));

  if ( zeroCost < nonZeroCost || !pcCU->getQtRootCbf(0) )
  {
    const UInt uiQPartNum = tuLevel0.GetAbsPartIdxNumParts();
    ::memset( pcCU->getTransformIdx()     , 0, uiQPartNum * sizeof(UChar) );
#if SCM_U0106_ACT_TU_SIG
    ::memset( pcCU->getColourTransform()  , false, uiQPartNum * sizeof(Bool) );
#endif
    for (Int comp=0; comp < numValidComponents; comp++)
    {
      const ComponentID component = ComponentID(comp);
      ::memset( pcCU->getCbf( component ) , 0, uiQPartNum * sizeof(UChar) );
      ::memset( pcCU->getCrossComponentPredictionAlpha(component), 0, ( uiQPartNum * sizeof(Char) ) );
    }
    static const UInt useTS[MAX_NUM_COMPONENT]={0,0,0};
    pcCU->setTransformSkipSubParts ( useTS, 0, pcCU->getDepth(0) );
#if DEBUG_STRING
    sDebug.clear();
    for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
    {
      sDebug+=debug_reorder_data_token[pcCU->isIntraBC(0)?1:0][i];
    }
#endif
  }
  else
  {
    xSetInterResidualQTData( NULL, false, tuLevel0); // Call first time to set coefficients.
  }

  // all decisions now made. Fully encode the CU, including the headers:
  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );

  UInt finalBits = 0;
  xAddSymbolBitsInter( pcCU, finalBits );
  // we've now encoded the pcCU, and so have a valid bit cost

  if ( !pcCU->getQtRootCbf( 0 ) )
  {
    pcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
  }
  else
  {
    xSetInterResidualQTData( pcYuvResiBest, true, tuLevel0 ); // else set the residual image data pcYUVResiBest from the various temp images.
  }
  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );

#if SCM_U0106_ACT_TU_SIG
  if(iColourTransform && eACTRDTestType == ACT_TRAN_CLR)
#else
  if(iColourTransform)
#endif
  {
    pcYuvResiBest->convert(extendedPrecision, 0, 0, cuWidthPixels, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(0));
  }

  pcYuvRec->addClip ( pcYuvPred, pcYuvResiBest, 0, cuWidthPixels, sps.getBitDepths() );

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)

  Distortion finalDistortion = 0;
  for(Int comp=0; comp<numValidComponents; comp++)
  {
    const ComponentID compID=ComponentID(comp);
    finalDistortion += m_pcRdCost->getDistPart( sps.getBitDepth(toChannelType(compID)), pcYuvRec->getAddr(compID ), pcYuvRec->getStride(compID ), pcYuvOrg->getAddr(compID ), pcYuvOrg->getStride(compID), cuWidthPixels >> pcYuvOrg->getComponentScaleX(compID), cuHeightPixels >> pcYuvOrg->getComponentScaleY(compID), compID);
  }

  pcCU->getTotalBits()       = finalBits;
  pcCU->getTotalDistortion() = finalDistortion;
  pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( finalBits, finalDistortion );
}

#if SCM_U0106_ACT_TU_SIG
Void TEncSearch::xEstimateInterResidualQTTUCSC( TComYuv        *pcResi,
                                                Double         &rdCost,
                                                UInt           &ruiBits,
                                                Distortion     &ruiDist,
                                                TComTU         &rTu,
                                                TComYuv*       pcOrgResi,
                                                ACTRDTestTypes eACTRDType
                                                DEBUG_STRING_FN_DECLARE(sDebug) )
{
  TComDataCU *pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const UInt uiDepth        = rTu.GetTransformDepthTotal();
  const UInt uiTrMode       = rTu.GetTransformDepthRel();
  const UInt partIdxesPerTU = rTu.GetAbsPartIdxNumParts();
  const UInt numValidComp   = pcCU->getPic()->getNumberValidComponents();
  const Bool extendedPrecision = pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();

  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  UInt SplitFlag = ((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && pcCU->isInter(uiAbsPartIdx) && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ));
  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && (pcCU->getWidth(uiAbsPartIdx) >= 32) && (pcCU->isInter(uiAbsPartIdx) || pcCU->isIntraBC(uiAbsPartIdx)) && (pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N))
  {
    SplitFlag = 1;
  }

  Bool bCheckFull;

  if ( SplitFlag && uiDepth == pcCU->getDepth(uiAbsPartIdx) && ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) ) )
  {
    bCheckFull = false;
  }
  else
  {
    bCheckFull =  ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  }

  Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && (pcCU->isIntraBC(uiAbsPartIdx) || pcCU->isInter(uiAbsPartIdx)) && (pcCU->getWidth(uiAbsPartIdx) >= 32) && bCheckFull)
  {
    bCheckSplit = false;
  }

  assert( bCheckFull || bCheckSplit );
  assert( pcCU->getPic()->getChromaFormat() == CHROMA_444 );

  UInt       uiSingleColorSpaceId      = 0;
  Double     dSingleCost               = MAX_DOUBLE;
  Distortion uiSingleDist              = 0;
  UInt       uiSingleBits              = 0; 

  Double     dSingleColorSpaceCost[2]  = {MAX_DOUBLE, MAX_DOUBLE};
  Distortion uiSingleColorSpaceDist[2] = {0, 0};
  UInt       uiSingleColorSpaceBits[2] = {0, 0};

  Distortion uiSingleDistComp[2][MAX_NUM_COMPONENT]             = { {0,0,0}, {0,0,0} };
  TCoeff     uiAbsSum[2][MAX_NUM_COMPONENT]                     = { {0,0,0}, {0,0,0} };
  UInt       uiBestTransformMode[2][MAX_NUM_COMPONENT]          = { {0,0,0}, {0,0,0} };
  UInt       bestExplicitRdpcmModeUnSplit[2][MAX_NUM_COMPONENT] = { {3,3,3}, {3,3,3} };
  Char       bestCrossCPredictionAlpha[2][MAX_NUM_COMPONENT]    = { {0,0,0}, {0,0,0} };

  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );

  if( bCheckFull )
  {
    Double minCost[MAX_NUM_COMPONENT];
    Bool checkTransformSkip[MAX_NUM_COMPONENT];
    pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

    m_pcEntropyCoder->resetBits();

    memset( m_pTempPel, 0, sizeof( Pel ) * rTu.getRect(COMPONENT_Y).width * rTu.getRect(COMPONENT_Y).height ); // not necessary needed for inside of recursion (only at the beginning)

    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurr[MAX_NUM_COMPONENT];
#if ADAPTIVE_QP_SELECTION
    TCoeff *pcArlCoeffCurr[MAX_NUM_COMPONENT];
#endif

    TCoeff bestColorSpaceCoeff[MAX_NUM_COMPONENT][MAX_TU_SIZE*MAX_TU_SIZE];
    Pel    bestColorSpaceResi [MAX_NUM_COMPONENT][MAX_TU_SIZE*MAX_TU_SIZE];
#if ADAPTIVE_QP_SELECTION
    TCoeff bestColorSpaceArlCoeff[MAX_NUM_COMPONENT][MAX_TU_SIZE*MAX_TU_SIZE];
#endif

    for(Int colorSpaceId = 0; colorSpaceId < 2; colorSpaceId++)
    {
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
      m_pcEntropyCoder->resetBits();

      const Bool iColorTransform = m_pcEncCfg->getRGBFormatFlag()? (!colorSpaceId? true: false): (colorSpaceId? true: false);

      if(eACTRDType == ACT_TRAN_CLR && !iColorTransform)
      {
        continue;
      }
      if(eACTRDType == ACT_ORG_CLR && iColorTransform)
      {
        continue;
      }

      pcCU->setColourTransformSubParts(iColorTransform, uiAbsPartIdx, uiDepth);
      for( UInt ch = 0; ch < numValidComp; ch++ )
      {
        const ComponentID compID = ComponentID(ch);
        if(iColorTransform)
        {
          m_pcNoCorrYuvTmp[pcCU->getDepth(uiAbsPartIdx)].copyPartToPartComponent( compID, pcResi, uiAbsPartIdx, rTu.getRect(compID).width, rTu.getRect(compID).height );
        }
        else
        {
          pcOrgResi->copyPartToPartComponent( compID, pcResi, uiAbsPartIdx, rTu.getRect(compID).width, rTu.getRect(compID).height );
        }
      }

      for(UInt i = 0; i < numValidComp; i++)
      {
        minCost[i] = MAX_DOUBLE;
      }

      Pel crossCPredictedResidualBuffer[ MAX_TU_SIZE * MAX_TU_SIZE ];

      for(UInt i = 0; i < numValidComp; i++)
      {
        checkTransformSkip[i]    = false;
        const ComponentID compID = ComponentID(i);
        const Int channelBitDepth=pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
        pcCoeffCurr[compID]      = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
#if ADAPTIVE_QP_SELECTION
        pcArlCoeffCurr[compID] = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] +  rTu.getCoefficientOffset(compID);
#endif

        if(rTu.ProcessComponentSection(compID))
        {
          QpParam cQP(*pcCU, compID, uiAbsPartIdx);
          if(!pcCU->isLosslessCoded(0) && iColorTransform)
          {
            Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
            m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( deltaQP );
            m_pcRdCost->adjustLambdaForColourTrans( deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
          }

          checkTransformSkip[compID] = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
                                       TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()) &&
                                       (!pcCU->isLosslessCoded(0));

          assert(rTu.getRect(compID).width == rTu.getRect(compID).height);
          const TComRectangle &tuCompRect       = rTu.getRect(compID);
          TCoeff        *currentCoefficients    = pcCoeffCurr[compID];
#if ADAPTIVE_QP_SELECTION
          TCoeff        *currentARLCoefficients = pcArlCoeffCurr[compID];
#endif
          const Bool isCrossCPredictionAvailable =    isChroma(compID)
                                                   && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                                   && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) != 0);

          Char preCalcAlpha = 0;
          const Pel *pLumaResi = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Y, rTu.getRect( COMPONENT_Y ).x0, rTu.getRect( COMPONENT_Y ).y0 );

          if (isCrossCPredictionAvailable)
          {
            const Bool bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
            const Pel  *const lumaResidualForEstimate       = bUseReconstructedResidualForEstimate ? pLumaResi                                                     : pcResi->getAddrPix(COMPONENT_Y, tuCompRect.x0, tuCompRect.y0);
            const UInt        lumaResidualStrideForEstimate = bUseReconstructedResidualForEstimate ? m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y) : pcResi->getStride(COMPONENT_Y);

            preCalcAlpha = xCalcCrossComponentPredictionAlpha(rTu,
                                                              compID,
                                                              lumaResidualForEstimate,
                                                              pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                              tuCompRect.width,
                                                              tuCompRect.height,
                                                              lumaResidualStrideForEstimate,
                                                              pcResi->getStride(compID));
          }

          const Int transformSkipModesToTest    = checkTransformSkip[compID] ? 2 : 1;
          const Int crossCPredictionModesToTest = (preCalcAlpha != 0)        ? 2 : 1; //preCalcAlpha cannot be anything other than 0 if isCrossCPredictionAvailable is false

          const Bool isOneMode                  = (crossCPredictionModesToTest == 1) && (transformSkipModesToTest == 1);

          for (Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
          {
            pcCU->setTransformSkipPartRange(transformSkipModeId, compID, uiAbsPartIdx, partIdxesPerTU);

            for (Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
            {
              const Bool isFirstMode          = (transformSkipModeId == 0) && (crossCPredictionModeId == 0);
              const Bool bUseCrossCPrediction = crossCPredictionModeId != 0;

              m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
              m_pcEntropyCoder->resetBits();

              pcCU->setTransformSkipPartRange(transformSkipModeId, compID, uiAbsPartIdx, partIdxesPerTU);
              pcCU->setCrossComponentPredictionAlphaPartRange((bUseCrossCPrediction ? preCalcAlpha : 0), compID, uiAbsPartIdx, partIdxesPerTU );

              if ((compID != COMPONENT_Cr) && ((transformSkipModeId == 1) ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ()))
              {
                m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, tuCompRect.width, tuCompRect.height, toChannelType(compID));
              }

#if RDOQ_CHROMA_LAMBDA
              m_pcTrQuant->selectLambda(compID);
#endif

              Pel *pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
              UInt resiStride     = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);

              TCoeff bestCoeffComp   [MAX_TU_SIZE*MAX_TU_SIZE];
              Pel    bestResiComp    [MAX_TU_SIZE*MAX_TU_SIZE];

#if ADAPTIVE_QP_SELECTION
              TCoeff bestArlCoeffComp[MAX_TU_SIZE*MAX_TU_SIZE];
#endif
              TCoeff     currAbsSum   = 0;
              UInt       currCompBits = 0;
              Distortion currCompDist = 0;
              Double     currCompCost = 0;
              UInt       nonCoeffBits = 0;
              Distortion nonCoeffDist = 0;
              Double     nonCoeffCost = 0;

              if(!isOneMode && !isFirstMode)
              {
                memcpy(bestCoeffComp,    currentCoefficients,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                memcpy(bestArlCoeffComp, currentARLCoefficients, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                for(Int y = 0; y < tuCompRect.height; y++)
                {
                  memcpy(&bestResiComp[y * tuCompRect.width], (pcResiCurrComp + (y * resiStride)), (sizeof(Pel) * tuCompRect.width));
                }
              }

              if (bUseCrossCPrediction)
              {
                TComTrQuant::crossComponentPrediction(rTu,
                                                      compID,
                                                      pLumaResi,
                                                      pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                      crossCPredictedResidualBuffer,
                                                      tuCompRect.width,
                                                      tuCompRect.height,
                                                      m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                      pcResi->getStride(compID),
                                                      tuCompRect.width,
                                                      false);

                m_pcTrQuant->transformNxN(rTu, compID, crossCPredictedResidualBuffer, tuCompRect.width, currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }
              else
              {
                m_pcTrQuant->transformNxN(rTu, compID, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ), pcResi->getStride(compID), currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }

              if(isFirstMode || (currAbsSum == 0))
              {
                if (bUseCrossCPrediction)
                {
                  TComTrQuant::crossComponentPrediction(rTu,
                                                        compID,
                                                        pLumaResi,
                                                        m_pTempPel,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        tuCompRect.width,
                                                        tuCompRect.height,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                        tuCompRect.width,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        true);

                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride( compID ), pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual distortion
                }  
                else
                {
                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pTempPel, tuCompRect.width, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual distortion
                }

                m_pcEntropyCoder->encodeQtCbfZero( rTu, toChannelType(compID) );

                if ( isCrossCPredictionAvailable )
                {
                  m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, compID );
                }

                nonCoeffBits = m_pcEntropyCoder->getNumberOfWrittenBits();
                nonCoeffCost = m_pcRdCost->calcRdCost( nonCoeffBits, nonCoeffDist );
              }

                if( currAbsSum > 0 ) //if non-zero coefficients are present, a residual needs to be derived for further prediction
                {
                  if (isFirstMode)
                  {
                    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
                    m_pcEntropyCoder->resetBits();
                  }

                  m_pcEntropyCoder->encodeQtCbf( rTu, compID, true );

                  if (isCrossCPredictionAvailable)
                  {
                    m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, compID );
                  }

                  m_pcEntropyCoder->encodeCoeffNxN( rTu, currentCoefficients, compID );
                  currCompBits = m_pcEntropyCoder->getNumberOfWrittenBits();

                  pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 );

                  m_pcTrQuant->invTransformNxN( rTu, compID, pcResiCurrComp, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID), currentCoefficients, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sSingleStringTest, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask)) );

                  if (bUseCrossCPrediction)
                  {
                    TComTrQuant::crossComponentPrediction(rTu,
                                                          compID,
                                                          pLumaResi,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                          tuCompRect.width,
                                                          tuCompRect.height,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                          true);
                  }

                  currCompDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                          pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID),
                                                          tuCompRect.width, tuCompRect.height, compID);

                  currCompCost = m_pcRdCost->calcRdCost(currCompBits, currCompDist);

                  if (pcCU->isLosslessCoded(0))
                  {
                    nonCoeffCost = MAX_DOUBLE;
                  } 
                }  
                else if ((transformSkipModeId == 1) && !bUseCrossCPrediction) //NOTE: RExt - if the CBF (i.e. currAbsSum) is 0, this mode combination gives the same result as when transformSkipModeId = 0 (Not coding-efficiency-affecting - maybe remove test in later revision)
                {
                  currCompCost = MAX_DOUBLE;
                }
                else
                {
                  currCompBits = nonCoeffBits;
                  currCompDist = nonCoeffDist;
                  currCompCost = nonCoeffCost;
                }

                // evaluate
                if ((currCompCost < minCost[compID]) || ((transformSkipModeId == 1) && (currCompCost == minCost[compID])))
                {
                  bestExplicitRdpcmModeUnSplit[colorSpaceId][compID] = pcCU->getExplicitRdpcmMode(compID, uiAbsPartIdx);

                  if(isFirstMode) //check for forced null
                  {
                    if((nonCoeffCost < currCompCost) || (currAbsSum == 0))
                    {
                      memset(currentCoefficients, 0, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));

                      currAbsSum   = 0;
                      currCompBits = nonCoeffBits;
                      currCompDist = nonCoeffDist;
                      currCompCost = nonCoeffCost;
                    }
                  }

                  uiAbsSum                 [colorSpaceId][compID] = currAbsSum;
                  uiSingleDistComp         [colorSpaceId][compID] = currCompDist;
                  minCost                  [compID]               = currCompCost;
                  uiBestTransformMode      [colorSpaceId][compID] = transformSkipModeId;
                  bestCrossCPredictionAlpha[colorSpaceId][compID] = (crossCPredictionModeId == 1) ? pcCU->getCrossComponentPredictionAlpha(uiAbsPartIdx, compID) : 0;

                  if (uiAbsSum[colorSpaceId][compID] == 0)
                  {
                    if (bUseCrossCPrediction)
                    {
                      TComTrQuant::crossComponentPrediction(rTu,
                                                            compID,
                                                            pLumaResi,
                                                            m_pTempPel,
                                                            m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                            tuCompRect.width,
                                                            tuCompRect.height,
                                                            m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                            tuCompRect.width,
                                                            m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                            true);
                    }
                    else
                    {
                      pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
                      const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);
                      for(UInt uiY = 0; uiY < tuCompRect.height; uiY++)
                      {
                        memset(pcResiCurrComp, 0, (sizeof(Pel) * tuCompRect.width));
                        pcResiCurrComp += uiStride;
                      }
                    }
                  }
                }
                else
                {
                  // reset
                  memcpy(currentCoefficients,    bestCoeffComp,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                  memcpy(currentARLCoefficients, bestArlCoeffComp, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                  for (Int y = 0; y < tuCompRect.height; y++)
                  {
                    memcpy((pcResiCurrComp + (y * resiStride)), &bestResiComp[y * tuCompRect.width], (sizeof(Pel) * tuCompRect.width));
                  }
                }  
            }  
          }

          pcCU->setExplicitRdpcmModePartRange            (   bestExplicitRdpcmModeUnSplit[colorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU);
          pcCU->setTransformSkipPartRange                (   uiBestTransformMode         [colorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU );
          pcCU->setCbfPartRange                          ((((uiAbsSum                    [colorSpaceId][compID] > 0) ? 1 : 0) << uiTrMode), compID, uiAbsPartIdx, partIdxesPerTU );
          pcCU->setCrossComponentPredictionAlphaPartRange(   bestCrossCPredictionAlpha   [colorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU );


          if(!pcCU->isLosslessCoded(0) && iColorTransform)
          {
            Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
            m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( - deltaQP );
            m_pcRdCost->adjustLambdaForColourTrans( - deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
          }
        } // processing section
      } // component loop

      if(iColorTransform)
      {
        m_pcQTTempTComYuv[uiQTTempAccessLayer].convert( extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(uiAbsPartIdx) );  //YcgCo -> RGB

        const TComRectangle &tuCompRect = rTu.getRect(COMPONENT_Y);
        uiSingleDistComp[colorSpaceId][COMPONENT_Y ] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                                               pcOrgResi->getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), pcOrgResi->getStride(COMPONENT_Y), tuCompRect.width, tuCompRect.height, COMPONENT_Y );

        const TComRectangle &tuCompRectC = rTu.getRect(COMPONENT_Cb);
        uiSingleDistComp[colorSpaceId][COMPONENT_Cb] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Cb),
                                                                               pcOrgResi->getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cb), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cb );
        uiSingleDistComp[colorSpaceId][COMPONENT_Cr] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Cr),
                                                                               pcOrgResi->getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cr), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cr );
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
      m_pcEntropyCoder->resetBits();

      if( uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
      {
        m_pcEntropyCoder->encodeTransformSubdivFlag( 0, 5 - uiLog2TrSize );
      }

      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        const UInt chOrderChange = ((ch + 1) == numValidComp) ? 0 : (ch + 1);
        const ComponentID compID=ComponentID(chOrderChange);
        if( rTu.ProcessComponentSection(compID) )
        {
          m_pcEntropyCoder->encodeQtCbf( rTu, compID, true );
        }
      }

      if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, uiTrMode)) )
      {
        m_pcEntropyCoder->m_pcEntropyCoderIf->codeColourTransformFlag( pcCU, uiAbsPartIdx );
      }

      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        if (rTu.ProcessComponentSection(compID))
        {
          if(isChroma(compID) && (uiAbsSum[colorSpaceId][COMPONENT_Y] != 0))
          {
            m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, compID );
          }

          m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeffCurr[compID], compID );

          uiSingleColorSpaceDist[colorSpaceId] += uiSingleDistComp[colorSpaceId][compID];
        }
      }

      uiSingleColorSpaceBits[colorSpaceId] = m_pcEntropyCoder->getNumberOfWrittenBits();
      dSingleColorSpaceCost[colorSpaceId]  = m_pcRdCost->calcRdCost( uiSingleColorSpaceBits[colorSpaceId], uiSingleColorSpaceDist[colorSpaceId] );

      if( dSingleColorSpaceCost[colorSpaceId] < dSingleCost )
      {
        dSingleCost          = dSingleColorSpaceCost[colorSpaceId];
        uiSingleDist         = uiSingleColorSpaceDist[colorSpaceId];
        uiSingleBits         = uiSingleColorSpaceBits[colorSpaceId];
        uiSingleColorSpaceId = colorSpaceId;

        for(UInt i = 0; i < numValidComp; i++)
        {
          const ComponentID compID        = ComponentID(i);
          const TComRectangle &tuCompRect = rTu.getRect(compID);

          TCoeff *pcCoeff                 = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
          memcpy(bestColorSpaceCoeff[i],    pcCoeff,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
          TCoeff *pcArlCoeff              = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
          memcpy(bestColorSpaceArlCoeff[i], pcArlCoeff, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif

          Pel *piResi = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
          UInt resiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);
          for(Int y = 0; y < tuCompRect.height; y++)
          {
            memcpy(&(bestColorSpaceResi[i][y * tuCompRect.width]), (piResi + (y * resiStride)), (sizeof(Pel) * tuCompRect.width));
          }
        }
        m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_CHROMA_INTRA ] );  //CI_CHROMA_INTRA is never used
      }

      if( !colorSpaceId && !uiAbsSum[colorSpaceId][COMPONENT_Y] && !uiAbsSum[colorSpaceId][COMPONENT_Cb] && !uiAbsSum[colorSpaceId][COMPONENT_Cr] )  //all cbfs are zero, skip the other color space
        break;
    }

    Bool iColorTransform = m_pcEncCfg->getRGBFormatFlag()? (!uiSingleColorSpaceId? true: false): (uiSingleColorSpaceId? true: false);
    pcCU->setColourTransformSubParts(iColorTransform, uiAbsPartIdx, uiDepth);

    for( UInt ch = 0; ch < numValidComp; ch++ )
    {
      ComponentID compID = ComponentID(ch);
      const TComRectangle &tuCompRect = rTu.getRect(compID);

      pcCU->setExplicitRdpcmModePartRange            (   bestExplicitRdpcmModeUnSplit[uiSingleColorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU );
      pcCU->setTransformSkipPartRange                (   uiBestTransformMode         [uiSingleColorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU );
      pcCU->setCbfPartRange                          ((((uiAbsSum                    [uiSingleColorSpaceId][compID] > 0) ? 1 : 0) << uiTrMode), compID, uiAbsPartIdx, partIdxesPerTU );
      pcCU->setCrossComponentPredictionAlphaPartRange(   bestCrossCPredictionAlpha   [uiSingleColorSpaceId][compID],                            compID, uiAbsPartIdx, partIdxesPerTU );

      TCoeff *pcCoeff                 = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
      memcpy( pcCoeff, bestColorSpaceCoeff[ch],   (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
      TCoeff *pcArlCoeff              = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
      memcpy( pcArlCoeff, bestColorSpaceArlCoeff[ch], (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif

      Pel *piResi = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
      UInt resiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);
      for(Int y = 0; y < tuCompRect.height; y++)
      {
        memcpy((piResi + (y * resiStride)), &(bestColorSpaceResi[ch][y * tuCompRect.width]), (sizeof(Pel) * tuCompRect.width));
      }
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_CHROMA_INTRA ] );
  } // check full

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    Distortion uiSubdivDist = 0;
    UInt       uiSubdivBits = 0;
    Double     dSubdivCost = 0.0;

    //save the non-split CBFs in case we need to restore them later

    UInt bestCBF     [MAX_NUM_COMPONENT];
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);

      if (rTu.ProcessComponentSection(compID))
      {
        bestCBF[compID] = pcCU->getCbf(uiAbsPartIdx, compID, uiTrMode);
      }
    }

    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiQPartNumSubdiv = tuRecurseChild.GetAbsPartIdxNumParts();

      do
      {
        xEstimateInterResidualQTTUCSC( pcResi, dSubdivCost, uiSubdivBits, uiSubdivDist, tuRecurseChild, pcOrgResi, eACTRDType DEBUG_STRING_PASS_INTO(sSplitString));
      }
      while ( tuRecurseChild.nextSection(rTu) ) ;

      UInt uiCbfAny=0;
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        UInt uiYUVCbf = 0;
        for( UInt ui = 0; ui < 4; ++ui )
        {
          uiYUVCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, ComponentID(ch),  uiTrMode + 1 );
        }
        UChar *pBase=pcCU->getCbf( ComponentID(ch) );
        const UInt flags = uiYUVCbf << uiTrMode;
        for( UInt ui = 0; ui < 4 * uiQPartNumSubdiv; ++ui )
        {
          pBase[uiAbsPartIdx + ui] |= flags;
        }
        uiCbfAny |= uiYUVCbf;
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
      m_pcEntropyCoder->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT( MAX_NUM_COMPONENT, rTu );
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        xEncodeInterResidualQT( ComponentID(ch), rTu );
      }

      uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
      dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );

      if (!bCheckFull || (uiCbfAny && (dSubdivCost < dSingleCost)))
      {
        rdCost += dSubdivCost;
        ruiBits += uiSubdivBits;
        ruiDist += uiSubdivDist;
      }
      else
      {
        rdCost  += dSingleCost;
        ruiBits += uiSingleBits;
        ruiDist += uiSingleDist;

        //restore state to unsplit

        pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

        Bool iColorTransform = m_pcEncCfg->getRGBFormatFlag()? (!uiSingleColorSpaceId? true: false): (uiSingleColorSpaceId? true: false);
        pcCU->setColourTransformSubParts(iColorTransform, uiAbsPartIdx, uiDepth);

        for(UInt ch = 0; ch < numValidComp; ch++)
        {
          const ComponentID compID = ComponentID(ch);
            if (rTu.ProcessComponentSection(compID))
            {
              pcCU->setCbfPartRange((bestCBF[compID] << uiTrMode), compID, uiAbsPartIdx, partIdxesPerTU);
              pcCU->setCrossComponentPredictionAlphaPartRange(bestCrossCPredictionAlpha[uiSingleColorSpaceId][compID], compID, uiAbsPartIdx, partIdxesPerTU);
              pcCU->setTransformSkipPartRange(uiBestTransformMode[uiSingleColorSpaceId][compID], compID, uiAbsPartIdx, partIdxesPerTU);
              pcCU->setExplicitRdpcmModePartRange(bestExplicitRdpcmModeUnSplit[uiSingleColorSpaceId][compID], compID, uiAbsPartIdx, partIdxesPerTU);
            }
        }  

        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      }  
  }
  else
  {
    rdCost  += dSingleCost;
    ruiBits += uiSingleBits;
    ruiDist += uiSingleDist;
  }
}
#endif

Void TEncSearch::xEstimateInterResidualQT( TComYuv    *pcResi,
                                           Double     &rdCost,
                                           UInt       &ruiBits,
                                           Distortion &ruiDist,
                                           Distortion *puiZeroDist,
                                           TComTU     &rTu
                                           DEBUG_STRING_FN_DECLARE(sDebug),
                                           TComYuv* pcOrgResi
                                          )
{
  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiDepth      = rTu.GetTransformDepthTotal();
  const UInt uiTrMode     = rTu.GetTransformDepthRel();
  const UInt subTUDepth   = uiTrMode + 1;
  const UInt numValidComp = pcCU->getPic()->getNumberValidComponents();
  const Bool iColourTransform = pcCU->getColourTransform(0);
  const Bool extendedPrecision = pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  DEBUG_STRING_NEW(sSingleStringComp[MAX_NUM_COMPONENT])

  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  UInt SplitFlag = ((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && pcCU->isInter(uiAbsPartIdx) && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ));
#if DEBUG_STRING
  const Bool isIntraBc    = pcCU->isIntraBC(uiAbsPartIdx);
  const Int debugPredModeMask = DebugStringGetPredModeMask(pcCU->getPredictionMode(uiAbsPartIdx));
#endif

  if(m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded(uiAbsPartIdx) && (pcCU->getWidth(uiAbsPartIdx) >= 32) && (pcCU->isInter(uiAbsPartIdx) || pcCU->isIntraBC(uiAbsPartIdx)) && (pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N))
  {
    SplitFlag = 1;
  }

  Bool bCheckFull;

  if ( SplitFlag && uiDepth == pcCU->getDepth(uiAbsPartIdx) && ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) ) )
  {
    bCheckFull = false;
  }
  else
  {
    bCheckFull =  ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  }

  Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
  if ( m_pcEncCfg->getTransquantBypassInferTUSplit() && pcCU->isLosslessCoded( uiAbsPartIdx ) && (pcCU->isIntraBC( uiAbsPartIdx ) || pcCU->isInter( uiAbsPartIdx )) && (pcCU->getWidth( uiAbsPartIdx ) >= 32) && bCheckFull )
  {
    bCheckSplit = false;
  }

  assert( bCheckFull || bCheckSplit );

  // code full block
  Double     dSingleCost = MAX_DOUBLE;
  UInt       uiSingleBits                                                                                                        = 0;
  Distortion uiSingleDistComp            [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  Distortion uiSingleDist                                                                                                        = 0;
  TCoeff     uiAbsSum                    [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  UInt       uiBestTransformMode         [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};
  //  Stores the best explicit RDPCM mode for a TU encoded without split
  UInt       bestExplicitRdpcmModeUnSplit[MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{3,3}, {3,3}, {3,3}};
  Char       bestCrossCPredictionAlpha   [MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/] = {{0,0},{0,0},{0,0}};

  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );

  if( bCheckFull )
  {
    Double minCost[MAX_NUM_COMPONENT][2/*0 = top (or whole TU for non-4:2:2) sub-TU, 1 = bottom sub-TU*/];
    Bool checkTransformSkip[MAX_NUM_COMPONENT];
    pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

    m_pcEntropyCoder->resetBits();

    memset( m_pTempPel, 0, sizeof( Pel ) * rTu.getRect(COMPONENT_Y).width * rTu.getRect(COMPONENT_Y).height ); // not necessary needed for inside of recursion (only at the beginning)

    const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    TCoeff *pcCoeffCurr[MAX_NUM_COMPONENT];
#if ADAPTIVE_QP_SELECTION
    TCoeff *pcArlCoeffCurr[MAX_NUM_COMPONENT];
#endif

    for(UInt i=0; i<numValidComp; i++)
    {
      minCost[i][0] = MAX_DOUBLE;
      minCost[i][1] = MAX_DOUBLE;
    }

    Pel crossCPredictedResidualBuffer[ MAX_TU_SIZE * MAX_TU_SIZE ];

    for(UInt i=0; i<numValidComp; i++)
    {
      checkTransformSkip[i]=false;
      const ComponentID compID=ComponentID(i);
      const Int channelBitDepth=pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
      pcCoeffCurr[compID]    = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
#if ADAPTIVE_QP_SELECTION
      pcArlCoeffCurr[compID] = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] +  rTu.getCoefficientOffset(compID);
#endif

      if(rTu.ProcessComponentSection(compID))
      {
#if SCM_U0106_ACT_TU_SIG
        QpParam cQP(*pcCU, compID, uiAbsPartIdx);
#else
        QpParam cQP(*pcCU, compID);
#endif
        if(!pcCU->isLosslessCoded(0) && iColourTransform)
        {
          Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
          m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( deltaQP );
          m_pcRdCost->adjustLambdaForColourTrans( deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
        }

        checkTransformSkip[compID] = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
                                     TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getPpsRangeExtension().getLog2MaxTransformSkipBlockSize()) &&
                                     (!pcCU->isLosslessCoded(0));

        const Bool splitIntoSubTUs = rTu.getRect(compID).width != rTu.getRect(compID).height;

        TComTURecurse TUIterator(rTu, false, (splitIntoSubTUs ? TComTU::VERTICAL_SPLIT : TComTU::DONT_SPLIT), true, compID);

        const UInt partIdxesPerSubTU = TUIterator.GetAbsPartIdxNumParts(compID);

        do
        {
          const UInt           subTUIndex             = TUIterator.GetSectionNumber();
          const UInt           subTUAbsPartIdx        = TUIterator.GetAbsPartIdxTU(compID);
          const TComRectangle &tuCompRect             = TUIterator.getRect(compID);
          const UInt           subTUBufferOffset      = tuCompRect.width * tuCompRect.height * subTUIndex;

                TCoeff        *currentCoefficients    = pcCoeffCurr[compID] + subTUBufferOffset;
#if ADAPTIVE_QP_SELECTION
                TCoeff        *currentARLCoefficients = pcArlCoeffCurr[compID] + subTUBufferOffset;
#endif
          const Bool isCrossCPredictionAvailable      =    isChroma(compID)
                                                         && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()
                                                         && (pcCU->getCbf(subTUAbsPartIdx, COMPONENT_Y, uiTrMode) != 0);

          Char preCalcAlpha = 0;
          const Pel *pLumaResi = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( COMPONENT_Y, rTu.getRect( COMPONENT_Y ).x0, rTu.getRect( COMPONENT_Y ).y0 );

          if (isCrossCPredictionAvailable)
          {
            const Bool bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
            const Pel  *const lumaResidualForEstimate       = bUseReconstructedResidualForEstimate ? pLumaResi                                                     : pcResi->getAddrPix(COMPONENT_Y, tuCompRect.x0, tuCompRect.y0);
            const UInt        lumaResidualStrideForEstimate = bUseReconstructedResidualForEstimate ? m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y) : pcResi->getStride(COMPONENT_Y);

            preCalcAlpha = xCalcCrossComponentPredictionAlpha(TUIterator,
                                                              compID,
                                                              lumaResidualForEstimate,
                                                              pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                              tuCompRect.width,
                                                              tuCompRect.height,
                                                              lumaResidualStrideForEstimate,
                                                              pcResi->getStride(compID));
          }

          const Int transformSkipModesToTest    = checkTransformSkip[compID] ? 2 : 1;
          const Int crossCPredictionModesToTest = (preCalcAlpha != 0)        ? 2 : 1; // preCalcAlpha cannot be anything other than 0 if isCrossCPredictionAvailable is false

          const Bool isOneMode                  = (crossCPredictionModesToTest == 1) && (transformSkipModesToTest == 1);

          for (Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
          {
            pcCU->setTransformSkipPartRange(transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU);

            for (Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
            {
              const Bool isFirstMode          = (transformSkipModeId == 0) && (crossCPredictionModeId == 0);
              const Bool bUseCrossCPrediction = crossCPredictionModeId != 0;

              m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
              m_pcEntropyCoder->resetBits();

              pcCU->setTransformSkipPartRange(transformSkipModeId, compID, subTUAbsPartIdx, partIdxesPerSubTU);
              pcCU->setCrossComponentPredictionAlphaPartRange((bUseCrossCPrediction ? preCalcAlpha : 0), compID, subTUAbsPartIdx, partIdxesPerSubTU );

              if ((compID != COMPONENT_Cr) && ((transformSkipModeId == 1) ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ()))
              {
                m_pcEntropyCoder->estimateBit(m_pcTrQuant->m_pcEstBitsSbac, tuCompRect.width, tuCompRect.height, toChannelType(compID));
              }

#if RDOQ_CHROMA_LAMBDA
              m_pcTrQuant->selectLambda(compID);
#endif

              Pel *pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
              UInt resiStride     = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);

              TCoeff bestCoeffComp   [MAX_TU_SIZE*MAX_TU_SIZE];
              Pel    bestResiComp    [MAX_TU_SIZE*MAX_TU_SIZE];

#if ADAPTIVE_QP_SELECTION
              TCoeff bestArlCoeffComp[MAX_TU_SIZE*MAX_TU_SIZE];
#endif
              TCoeff     currAbsSum   = 0;
              UInt       currCompBits = 0;
              Distortion currCompDist = 0;
              Double     currCompCost = 0;
              UInt       nonCoeffBits = 0;
              Distortion nonCoeffDist = 0;
              Double     nonCoeffCost = 0;

              if(!isOneMode && !isFirstMode)
              {
                memcpy(bestCoeffComp,    currentCoefficients,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                memcpy(bestArlCoeffComp, currentARLCoefficients, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                for(Int y = 0; y < tuCompRect.height; y++)
                {
                  memcpy(&bestResiComp[y * tuCompRect.width], (pcResiCurrComp + (y * resiStride)), (sizeof(Pel) * tuCompRect.width));
                }
              }

              if (bUseCrossCPrediction)
              {
                TComTrQuant::crossComponentPrediction(TUIterator,
                                                      compID,
                                                      pLumaResi,
                                                      pcResi->getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                      crossCPredictedResidualBuffer,
                                                      tuCompRect.width,
                                                      tuCompRect.height,
                                                      m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                      pcResi->getStride(compID),
                                                      tuCompRect.width,
                                                      false);

                m_pcTrQuant->transformNxN(TUIterator, compID, crossCPredictedResidualBuffer, tuCompRect.width, currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }
              else
              {
                m_pcTrQuant->transformNxN(TUIterator, compID, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ), pcResi->getStride(compID), currentCoefficients,
#if ADAPTIVE_QP_SELECTION
                                          currentARLCoefficients,
#endif
                                          currAbsSum, cQP);
              }

              if(isFirstMode || (currAbsSum == 0))
              {
                if (bUseCrossCPrediction)
                {
                  TComTrQuant::crossComponentPrediction(TUIterator,
                                                        compID,
                                                        pLumaResi,
                                                        m_pTempPel,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        tuCompRect.width,
                                                        tuCompRect.height,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                        tuCompRect.width,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        true);

                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride( compID ), pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual distortion
                }
                else
                {
                  nonCoeffDist = m_pcRdCost->getDistPart( channelBitDepth, m_pTempPel, tuCompRect.width, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                          pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual distortion
                }

                m_pcEntropyCoder->encodeQtCbfZero( TUIterator, toChannelType(compID) );

                if ( isCrossCPredictionAvailable )
                {
                  m_pcEntropyCoder->encodeCrossComponentPrediction( TUIterator, compID );
                }

                nonCoeffBits = m_pcEntropyCoder->getNumberOfWrittenBits();
                nonCoeffCost = m_pcRdCost->calcRdCost( nonCoeffBits, nonCoeffDist );
              }

              if((puiZeroDist != NULL) && isFirstMode)
              {
                *puiZeroDist += nonCoeffDist; // initialized with zero residual distortion
              }

              DEBUG_STRING_NEW(sSingleStringTest)

              if( currAbsSum > 0 ) //if non-zero coefficients are present, a residual needs to be derived for further prediction
              {
                if (isFirstMode)
                {
                  m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
                  m_pcEntropyCoder->resetBits();
                }

                m_pcEntropyCoder->encodeQtCbf( TUIterator, compID, true );

                if (isCrossCPredictionAvailable)
                {
                  m_pcEntropyCoder->encodeCrossComponentPrediction( TUIterator, compID );
                }

                m_pcEntropyCoder->encodeCoeffNxN( TUIterator, currentCoefficients, compID );
                currCompBits = m_pcEntropyCoder->getNumberOfWrittenBits();

                pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 );

                m_pcTrQuant->invTransformNxN( TUIterator, compID, pcResiCurrComp, m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID), currentCoefficients, cQP DEBUG_STRING_PASS_INTO_OPTIONAL(&sSingleStringTest, (DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask)) );

                if (bUseCrossCPrediction)
                {
                  TComTrQuant::crossComponentPrediction(TUIterator,
                                                        compID,
                                                        pLumaResi,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                        tuCompRect.width,
                                                        tuCompRect.height,
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID     ),
                                                        true);
                }

                currCompDist = m_pcRdCost->getDistPart( channelBitDepth, m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        pcResi->getStride(compID),
                                                        tuCompRect.width, tuCompRect.height, compID);

                currCompCost = m_pcRdCost->calcRdCost(currCompBits, currCompDist);
                  
                if (pcCU->isLosslessCoded(0))
                {
                  nonCoeffCost = MAX_DOUBLE;
                }
              }
              else if ((transformSkipModeId == 1) && !bUseCrossCPrediction)
              {
                currCompCost = MAX_DOUBLE;
              }
              else
              {
                currCompBits = nonCoeffBits;
                currCompDist = nonCoeffDist;
                currCompCost = nonCoeffCost;
              }

              // evaluate
              if ((currCompCost < minCost[compID][subTUIndex]) || ((transformSkipModeId == 1) && (currCompCost == minCost[compID][subTUIndex])))
              {
                bestExplicitRdpcmModeUnSplit[compID][subTUIndex] = pcCU->getExplicitRdpcmMode(compID, subTUAbsPartIdx);

                if(isFirstMode) //check for forced null
                {
                  if((nonCoeffCost < currCompCost) || (currAbsSum == 0))
                  {
                    memset(currentCoefficients, 0, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));

                    currAbsSum   = 0;
                    currCompBits = nonCoeffBits;
                    currCompDist = nonCoeffDist;
                    currCompCost = nonCoeffCost;
                  }
                }

#if DEBUG_STRING
                if (currAbsSum > 0)
                {
                  DEBUG_STRING_SWAP(sSingleStringComp[compID], sSingleStringTest)
                }
                else
                {
                  sSingleStringComp[compID].clear();
                }
#endif

                uiAbsSum                 [compID][subTUIndex] = currAbsSum;
                uiSingleDistComp         [compID][subTUIndex] = currCompDist;
                minCost                  [compID][subTUIndex] = currCompCost;
                uiBestTransformMode      [compID][subTUIndex] = transformSkipModeId;
                bestCrossCPredictionAlpha[compID][subTUIndex] = (crossCPredictionModeId == 1) ? pcCU->getCrossComponentPredictionAlpha(subTUAbsPartIdx, compID) : 0;

                if (uiAbsSum[compID][subTUIndex] == 0)
                {
                  if (bUseCrossCPrediction)
                  {
                    TComTrQuant::crossComponentPrediction(TUIterator,
                                                          compID,
                                                          pLumaResi,
                                                          m_pTempPel,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0),
                                                          tuCompRect.width,
                                                          tuCompRect.height,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(COMPONENT_Y),
                                                          tuCompRect.width,
                                                          m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                          true);
                  }
                  else
                  {
                    pcResiCurrComp = m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix(compID, tuCompRect.x0, tuCompRect.y0);
                    const UInt uiStride = m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID);
                    for(UInt uiY = 0; uiY < tuCompRect.height; uiY++)
                    {
                      memset(pcResiCurrComp, 0, (sizeof(Pel) * tuCompRect.width));
                      pcResiCurrComp += uiStride;
                    }
                  }
                }
              }
              else
              {
                // reset
                memcpy(currentCoefficients,    bestCoeffComp,    (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#if ADAPTIVE_QP_SELECTION
                memcpy(currentARLCoefficients, bestArlCoeffComp, (sizeof(TCoeff) * tuCompRect.width * tuCompRect.height));
#endif
                for (Int y = 0; y < tuCompRect.height; y++)
                {
                  memcpy((pcResiCurrComp + (y * resiStride)), &bestResiComp[y * tuCompRect.width], (sizeof(Pel) * tuCompRect.width));
                }
              }
            }
          }

          pcCU->setExplicitRdpcmModePartRange            (   bestExplicitRdpcmModeUnSplit[compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU);
          pcCU->setTransformSkipPartRange                (   uiBestTransformMode         [compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU );
          pcCU->setCbfPartRange                          ((((uiAbsSum                    [compID][subTUIndex] > 0) ? 1 : 0) << uiTrMode), compID, subTUAbsPartIdx, partIdxesPerSubTU );
          pcCU->setCrossComponentPredictionAlphaPartRange(   bestCrossCPredictionAlpha   [compID][subTUIndex],                            compID, subTUAbsPartIdx, partIdxesPerSubTU );
        } while (TUIterator.nextSection(rTu)); //end of sub-TU loop

        if(!pcCU->isLosslessCoded(0) && iColourTransform)
        {
          Int deltaQP = pcCU->getSlice()->getPPS()->getPpsScreenExtension().getActQpOffset(compID) + pcCU->getSlice()->getSliceActQpDelta(compID);
          m_pcTrQuant->adjustBitDepthandLambdaForColourTrans( - deltaQP );
          m_pcRdCost->adjustLambdaForColourTrans( - deltaQP, pcCU->getSlice()->getSPS()->getBitDepths() );
        }
      } // processing section
    } // component loop

    {
      if(iColourTransform)
      {
        const TComRectangle &tuCompRect=rTu.getRect(COMPONENT_Y);
        for(UInt ch = 0; ch < numValidComp; ch++)
        {
          const ComponentID compID=ComponentID(ch);
          const TComRectangle &tuCompRectTmp = rTu.getRect(compID);
          assert(tuCompRectTmp.width == tuCompRectTmp.height);

          m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartComponentMxN(compID, &m_tmpYuvPred, tuCompRectTmp);
        }
        m_tmpYuvPred.convert( extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(uiAbsPartIdx) );  

        uiSingleDistComp[COMPONENT_Y ][0] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ), m_tmpYuvPred.getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), m_tmpYuvPred.getStride(COMPONENT_Y),
          pcOrgResi->getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), pcOrgResi->getStride(COMPONENT_Y), tuCompRect.width, tuCompRect.height, COMPONENT_Y );

        const TComRectangle &tuCompRectC=rTu.getRect(COMPONENT_Cb);
        uiSingleDistComp[COMPONENT_Cb][0] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_tmpYuvPred.getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), m_tmpYuvPred.getStride(COMPONENT_Cb),
          pcOrgResi->getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cb), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cb );

        uiSingleDistComp[COMPONENT_Cr][0] = m_pcRdCost->getDistPart(pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ), m_tmpYuvPred.getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), m_tmpYuvPred.getStride(COMPONENT_Cr),
          pcOrgResi->getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cr), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cr );

        uiSingleDistComp[COMPONENT_Y][1] = uiSingleDistComp[COMPONENT_Cb][1] = uiSingleDistComp[COMPONENT_Cr][1] = 0; 
      }
    }

#if SCM_U0106_ACT_TU_SIG
    if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, uiTrMode)) )
    {
      m_pcEntropyCoder->m_pcEntropyCoderIf->codeColourTransformFlag( pcCU, uiAbsPartIdx );
    }
#endif

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      if (rTu.ProcessComponentSection(compID) && (rTu.getRect(compID).width != rTu.getRect(compID).height))
      {
        offsetSubTUCBFs(rTu, compID); //the CBFs up to now have been defined for two sub-TUs - shift them down a level and replace with the parent level CBF
      }
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    if( uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( 0, 5 - uiLog2TrSize );
    }

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const UInt chOrderChange = ((ch + 1) == numValidComp) ? 0 : (ch + 1);
      const ComponentID compID=ComponentID(chOrderChange);
      if( rTu.ProcessComponentSection(compID) )
      {
        m_pcEntropyCoder->encodeQtCbf( rTu, compID, true );
      }
    }

    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      if (rTu.ProcessComponentSection(compID))
      {
        if(isChroma(compID) && (uiAbsSum[COMPONENT_Y][0] != 0))
        {
          m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, compID );
        }

        m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeffCurr[compID], compID );
        for (UInt subTUIndex = 0; subTUIndex < 2; subTUIndex++)
        {
          uiSingleDist += uiSingleDistComp[compID][subTUIndex];
        }
      }
    }

    uiSingleBits = m_pcEntropyCoder->getNumberOfWrittenBits();

    dSingleCost = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist );
  } // check full

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
      m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    }
    Distortion uiSubdivDist = 0;
    UInt       uiSubdivBits = 0;
    Double     dSubdivCost = 0.0;

    //save the non-split CBFs in case we need to restore them later

    UInt bestCBF     [MAX_NUM_COMPONENT];
    UInt bestsubTUCBF[MAX_NUM_COMPONENT][2];
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);

      if (rTu.ProcessComponentSection(compID))
      {
        bestCBF[compID] = pcCU->getCbf(uiAbsPartIdx, compID, uiTrMode);

        const TComRectangle &tuCompRect = rTu.getRect(compID);
        if (tuCompRect.width != tuCompRect.height)
        {
          const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

          for (UInt subTU = 0; subTU < 2; subTU++)
          {
            bestsubTUCBF[compID][subTU] = pcCU->getCbf ((uiAbsPartIdx + (subTU * partIdxesPerSubTU)), compID, subTUDepth);
          }
        }
      }
    }


    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiQPartNumSubdiv = tuRecurseChild.GetAbsPartIdxNumParts();

    DEBUG_STRING_NEW(sSplitString[MAX_NUM_COMPONENT])

    do
    {
      DEBUG_STRING_NEW(childString)
      xEstimateInterResidualQT( pcResi, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist,  tuRecurseChild DEBUG_STRING_PASS_INTO(childString), pcOrgResi );
#if DEBUG_STRING
      // split the string by component and append to the relevant output (because decoder decodes in channel order, whereas this search searches by TU-order)
      std::size_t lastPos=0;
      const std::size_t endStrng=childString.find(debug_reorder_data_token[isIntraBc?1:0][MAX_NUM_COMPONENT], lastPos);
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        if (lastPos!=std::string::npos && childString.find(debug_reorder_data_inter_token[ch], lastPos)==lastPos)
        {
          lastPos+=strlen(debug_reorder_data_inter_token[ch]); // skip leading string
        }
        if (pos!=std::string::npos && pos>endStrng)
        {
          lastPos=endStrng;
        }
        sSplitString[ch]+=childString.substr(lastPos, (pos==std::string::npos)? std::string::npos : (pos-lastPos) );
        lastPos=pos;
      }
#endif
    } while ( tuRecurseChild.nextSection(rTu) ) ;

    UInt uiCbfAny=0;
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      UInt uiYUVCbf = 0;
      for( UInt ui = 0; ui < 4; ++ui )
      {
        uiYUVCbf |= pcCU->getCbf( uiAbsPartIdx + ui * uiQPartNumSubdiv, ComponentID(ch),  uiTrMode + 1 );
      }
      UChar *pBase=pcCU->getCbf( ComponentID(ch) );
      const UInt flags=uiYUVCbf << uiTrMode;
      for( UInt ui = 0; ui < 4 * uiQPartNumSubdiv; ++ui )
      {
        pBase[uiAbsPartIdx + ui] |= flags;
      }
      uiCbfAny|=uiYUVCbf;
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    // when compID isn't a channel, code Cbfs:
    xEncodeInterResidualQT( MAX_NUM_COMPONENT, rTu );
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      xEncodeInterResidualQT( ComponentID(ch), rTu );
    }

    uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );

    if (!bCheckFull || (uiCbfAny && (dSubdivCost < dSingleCost)))
    {
      rdCost += dSubdivCost;
      ruiBits += uiSubdivBits;
      ruiDist += uiSubdivDist;
#if DEBUG_STRING
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        DEBUG_STRING_APPEND(sDebug, debug_reorder_data_token[isIntraBc?1:0][ch])
        DEBUG_STRING_APPEND(sDebug, sSplitString[ch])
      }
#endif
    }
    else
    {
      rdCost  += dSingleCost;
      ruiBits += uiSingleBits;
      ruiDist += uiSingleDist;

      //restore state to unsplit

      pcCU->setTrIdxSubParts( uiTrMode, uiAbsPartIdx, uiDepth );

      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        const ComponentID compID=ComponentID(ch);

        DEBUG_STRING_APPEND(sDebug, debug_reorder_data_token[isIntraBc?1:0][ch])
        if (rTu.ProcessComponentSection(compID))
        {
          DEBUG_STRING_APPEND(sDebug, sSingleStringComp[compID])

          const Bool splitIntoSubTUs   = rTu.getRect(compID).width != rTu.getRect(compID).height;
          const UInt numberOfSections  = splitIntoSubTUs ? 2 : 1;
          const UInt partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> (splitIntoSubTUs ? 1 : 0);

          for (UInt subTUIndex = 0; subTUIndex < numberOfSections; subTUIndex++)
          {
            const UInt  uisubTUPartIdx = uiAbsPartIdx + (subTUIndex * partIdxesPerSubTU);

            if (splitIntoSubTUs)
            {
              const UChar combinedCBF = (bestsubTUCBF[compID][subTUIndex] << subTUDepth) | (bestCBF[compID] << uiTrMode);
              pcCU->setCbfPartRange(combinedCBF, compID, uisubTUPartIdx, partIdxesPerSubTU);
            }
            else
            {
              pcCU->setCbfPartRange((bestCBF[compID] << uiTrMode), compID, uisubTUPartIdx, partIdxesPerSubTU);
            }

            pcCU->setCrossComponentPredictionAlphaPartRange(bestCrossCPredictionAlpha[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
            pcCU->setTransformSkipPartRange(uiBestTransformMode[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
            pcCU->setExplicitRdpcmModePartRange(bestExplicitRdpcmModeUnSplit[compID][subTUIndex], compID, uisubTUPartIdx, partIdxesPerSubTU);
          }
        }
      }

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][ CI_QT_TRAFO_TEST ] );
    }
  }
  else
  {
    rdCost  += dSingleCost;
    ruiBits += uiSingleBits;
    ruiDist += uiSingleDist;
#if DEBUG_STRING
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      DEBUG_STRING_APPEND(sDebug, debug_reorder_data_token[isIntraBc?1:0][compID])

      if (rTu.ProcessComponentSection(compID))
      {
        DEBUG_STRING_APPEND(sDebug, sSingleStringComp[compID])
      }
    }
#endif
  }
  DEBUG_STRING_APPEND(sDebug, debug_reorder_data_token[isIntraBc?1:0][MAX_NUM_COMPONENT])
}



Void TEncSearch::xEncodeInterResidualQT( const ComponentID compID, TComTU &rTu )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  const UInt uiCurrTrMode = rTu.GetTransformDepthRel();
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );

  const Bool bSubdiv = uiCurrTrMode != uiTrMode;

  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  if (compID==MAX_NUM_COMPONENT)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
    if( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() && uiLog2TrSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
      if((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && (pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N))
      {
        assert(bSubdiv); // Inferred splitting rule - see derivation and use of interSplitFlag in the specification.
      }
      else
      {
        m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, 5 - uiLog2TrSize );
      }
    }

    assert( !pcCU->isIntra(uiAbsPartIdx) );

    const Bool bFirstCbfOfCU = uiCurrTrMode == 0;

    for (UInt ch=COMPONENT_Cb; ch<pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compIdInner=ComponentID(ch);
      if( bFirstCbfOfCU || rTu.ProcessingAllQuadrants(compIdInner) )
      {
        if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode - 1 ) )
        {
          m_pcEntropyCoder->encodeQtCbf( rTu, compIdInner, !bSubdiv );
        }
      }
      else
      {
        assert( pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode ) == pcCU->getCbf( uiAbsPartIdx, compIdInner, uiCurrTrMode - 1 ) );
      }
    }

    if (!bSubdiv)
    {
      m_pcEntropyCoder->encodeQtCbf( rTu, COMPONENT_Y, true );
#if SCM_U0106_ACT_TU_SIG
      if( pcCU->getSlice()->getPPS()->getPpsScreenExtension().getUseColourTrans() && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, uiTrMode) || pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, uiTrMode)) )
      {
        m_pcEntropyCoder->m_pcEntropyCoderIf->codeColourTransformFlag( pcCU, uiAbsPartIdx );
      }
#endif
    }
  }

  if( !bSubdiv )
  {
    if (compID != MAX_NUM_COMPONENT) // we have already coded the CBFs, so now we code coefficients
    {
      if (rTu.ProcessComponentSection(compID))
      {
        if (isChroma(compID) && (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, uiTrMode) != 0))
        {
          m_pcEntropyCoder->encodeCrossComponentPrediction(rTu, compID);
        }

        if (pcCU->getCbf(uiAbsPartIdx, compID, uiTrMode) != 0)
        {
          const UInt uiQTTempAccessLayer = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
          TCoeff *pcCoeffCurr = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
          m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeffCurr, compID );
        }
      }
    }
  }
  else
  {
    if( compID==MAX_NUM_COMPONENT || pcCU->getCbf( uiAbsPartIdx, compID, uiCurrTrMode ) )
    {
      TComTURecurse tuRecurseChild(rTu, false);
      do
      {
        xEncodeInterResidualQT( compID, tuRecurseChild );
      } while (tuRecurseChild.nextSection(rTu));
    }
  }
}




Void TEncSearch::xSetInterResidualQTData( TComYuv* pcResi, Bool bSpatial, TComTU &rTu ) // TODO: turn this into two functions for bSpatial=true and false.
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiCurrTrMode=rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  const TComSPS *sps=pcCU->getSlice()->getSPS();

  if( uiCurrTrMode == uiTrMode )
  {
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTTempAccessLayer = sps->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    if( bSpatial )
    {
      // Data to be copied is in the spatial domain, i.e., inverse-transformed.

      for(UInt i=0; i<pcResi->getNumberValidComponents(); i++)
      {
        const ComponentID compID=ComponentID(i);
        if (rTu.ProcessComponentSection(compID))
        {
          const TComRectangle &rectCompTU(rTu.getRect(compID));
          m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartComponentMxN    ( compID, pcResi, rectCompTU );
        }
      }
    }
    else
    {
      for (UInt ch=0; ch < getNumberValidComponents(sps->getChromaFormatIdc()); ch++)
      {
        const ComponentID compID   = ComponentID(ch);
        if (rTu.ProcessComponentSection(compID))
        {
          const TComRectangle &rectCompTU(rTu.getRect(compID));
          const UInt numCoeffInBlock    = rectCompTU.width * rectCompTU.height;
          const UInt offset             = rTu.getCoefficientOffset(compID);
          TCoeff* dest                  = pcCU->getCoeff(compID)                        + offset;
          const TCoeff* src             = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + offset;
          ::memcpy( dest, src, sizeof(TCoeff)*numCoeffInBlock );

#if ADAPTIVE_QP_SELECTION
          TCoeff* pcArlCoeffSrc            = m_ppcQTTempArlCoeff[compID][uiQTTempAccessLayer] + offset;
          TCoeff* pcArlCoeffDst            = pcCU->getArlCoeff(compID)                        + offset;
          ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * numCoeffInBlock );
#endif
        }
      }
    }
  }
  else
  {

    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetInterResidualQTData( pcResi, bSpatial, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}




UInt TEncSearch::xModeBitsIntra( TComDataCU* pcCU, UInt uiMode, UInt uiPartOffset, UInt uiDepth, const ChannelType chType )
{
  // Reload only contexts required for coding intra mode information
  m_pcRDGoOnSbacCoder->loadIntraDirMode( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST], chType );

  // Temporarily set the intra dir being tested, and only
  // for absPartIdx, since encodeIntraDirModeLuma/Chroma only use
  // the entry at absPartIdx.

  UChar &rIntraDirVal=pcCU->getIntraDir( chType )[uiPartOffset];
  UChar origVal=rIntraDirVal;
  rIntraDirVal = uiMode;
  //pcCU->setIntraDirSubParts ( chType, uiMode, uiPartOffset, uiDepth + uiInitTrDepth );

  m_pcEntropyCoder->resetBits();
  if (isLuma(chType))
  {
    m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPartOffset);
  }
  else
  {
    m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiPartOffset);
  }

  rIntraDirVal = origVal; // restore

  return m_pcEntropyCoder->getNumberOfWrittenBits();
}




UInt TEncSearch::xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList )
{
  UInt i;
  UInt shift=0;

  while ( shift<uiFastCandNum && uiCost<CandCostList[ uiFastCandNum-1-shift ] )
  {
    shift++;
  }

  if( shift!=0 )
  {
    for(i=1; i<shift; i++)
    {
      CandModeList[ uiFastCandNum-i ] = CandModeList[ uiFastCandNum-1-i ];
      CandCostList[ uiFastCandNum-i ] = CandCostList[ uiFastCandNum-1-i ];
    }
    CandModeList[ uiFastCandNum-shift ] = uiMode;
    CandCostList[ uiFastCandNum-shift ] = uiCost;
    return 1;
  }

  return 0;
}





/** add inter-prediction syntax elements for a CU block
 * \param pcCU
 * \param uiQp
 * \param uiTrMode
 * \param ruiBits
 * \returns Void
 */
Void  TEncSearch::xAddSymbolBitsInter( TComDataCU* pcCU, UInt& ruiBits )
{
  if(pcCU->getMergeFlag( 0 ) && pcCU->getPartitionSize( 0 ) == SIZE_2Nx2N && !pcCU->getQtRootCbf( 0 ))
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    m_pcEntropyCoder->resetBits();
    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }
    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    m_pcEntropyCoder->encodeMergeIndex(pcCU, 0, true);

    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    m_pcEntropyCoder->resetBits();

    if(pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }

    m_pcEntropyCoder->encodeSkipFlag ( pcCU, 0, true );
    m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
    m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
    m_pcEntropyCoder->encodePredInfo( pcCU, 0 );

    Bool codeDeltaQp = false;
    Bool codeChromaQpAdj = false;
    m_pcEntropyCoder->encodeCoeff   ( pcCU, 0, pcCU->getDepth(0), codeDeltaQp, codeChromaQpAdj );

    ruiBits += m_pcEntropyCoder->getNumberOfWrittenBits();
  }
}





/**
 * \brief Generate half-sample interpolated block
 *
 * \param pattern Reference picture ROI
 * \param biPred    Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingH( TComPattern* pattern )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();

  Int intStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Int dstStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;
  Int halfFilterSize = (filterSize>>1);
  Pel *srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_filteredBlock[0][0].getChromaFormat();

  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 0, false, chFmt, pattern->getBitDepthY());
  if ( !m_bSkipFracME )
  {
    m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 2, false, chFmt, pattern->getBitDepthY());
  }


  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+0, 0, false, true, chFmt, pattern->getBitDepthY());

  if ( m_bSkipFracME )
  {
    return;
  }

  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2, false, true, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+0, 0, false, true, chFmt, pattern->getBitDepthY());

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[2][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2, false, true, chFmt, pattern->getBitDepthY());
}





/**
 * \brief Generate quarter-sample interpolated blocks
 *
 * \param pattern    Reference picture ROI
 * \param halfPelRef Half-pel mv
 * \param biPred     Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingQ( TComPattern* pattern, TComMv halfPelRef )
{
  Int width      = pattern->getROIYWidth();
  Int height     = pattern->getROIYHeight();
  Int srcStride  = pattern->getPatternLStride();

  Pel *srcPtr;
  Int intStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Int dstStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);
  Pel *intPtr;
  Pel *dstPtr;
  Int filterSize = NTAPS_LUMA;

  Int halfFilterSize = (filterSize>>1);

  Int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_filteredBlock[0][0].getChromaFormat();

  // Horizontal filter 1/4
  srcPtr = pattern->getROIY() - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false, chFmt, pattern->getBitDepthY());

  // Horizontal filter 3/4
  srcPtr = pattern->getROIY() - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false, chFmt, pattern->getBitDepthY());

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][1].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][3].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt, pattern->getBitDepthY());
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt, pattern->getBitDepthY());
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[1][2].getAddr(COMPONENT_Y);
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[3][2].getAddr(COMPONENT_Y);
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][3].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt, pattern->getBitDepthY());

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][3].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt, pattern->getBitDepthY());
}





//! set wp tables
Void  TEncSearch::setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.bApplyWeight = false;
    return;
  }

  TComSlice       *pcSlice  = pcCU->getSlice();
  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.bApplyWeight = ( pcSlice->getSliceType()==P_SLICE && pcSlice->testWeightPred() ) || ( pcSlice->getSliceType()==B_SLICE && pcSlice->testWeightBiPred() ) ;

  if ( !m_cDistParam.bApplyWeight )
  {
    return;
  }

  Int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  Int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcCU, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 )
  {
    wp0 = NULL;
  }
  if ( iRefIdx1 < 0 )
  {
    wp1 = NULL;
  }

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

//! \}
