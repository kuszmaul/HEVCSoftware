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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "TLibCommon/TypeDef.h"
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
  }

  m_pcIntraBCHashTable = NULL;
  m_pcIntraBCHashTable = new IntraBCHashNode**[INTRABC_HASH_DEPTH];
  for(int i = 0; i < INTRABC_HASH_DEPTH; i++)
  {
    m_pcIntraBCHashTable[i] = new IntraBCHashNode*[INTRABC_HASH_TABLESIZE];
    memset(m_pcIntraBCHashTable[i], 0, INTRABC_HASH_TABLESIZE * sizeof(IntraBCHashNode*));
  }

  m_puhQTTempTrIdx                                 = NULL;
  m_pcQTTempTComYuv                                = NULL;
  m_pcEncCfg                                       = NULL;
  m_pcEntropyCoder                                 = NULL;
  m_pTempPel                                       = NULL;
  setWpScalingDistParam( NULL, -1, REF_PIC_LIST_X );

#if SCM__R0348_PALETTE_MODE
  m_paOriginalLevel  = (Pel*)xMalloc(Pel , MAX_CU_SIZE * MAX_CU_SIZE);
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    m_paBestLevel[i]  = (Pel*)xMalloc(Pel , MAX_CU_SIZE * MAX_CU_SIZE);
  }
  m_paBestSPoint = (UChar*)xMalloc(UChar , MAX_CU_SIZE * MAX_CU_SIZE);
  m_paBestRun    = (TCoeff*)xMalloc(TCoeff , MAX_CU_SIZE * MAX_CU_SIZE);
#endif
}




TEncSearch::~TEncSearch()
{
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
    }
  }

  delete[] m_puhQTTempTrIdx;
  delete[] m_pcQTTempTComYuv;

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    delete[] m_pSharedPredTransformSkip[ch];
    delete[] m_pcQTTempTUCoeff[ch];
#if ADAPTIVE_QP_SELECTION
    delete[] m_ppcQTTempTUArlCoeff[ch];
#endif
    delete[] m_phQTTempCrossComponentPredictionAlpha[ch];
    delete[] m_puhQTTempTransformSkipFlag[ch];
  }
  m_pcQTTempTransformSkipTComYuv.destroy();

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
#if SCM__R0348_PALETTE_MODE
  if (m_paOriginalLevel)  { xFree(m_paOriginalLevel);  m_paOriginalLevel=NULL;  }
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    if (m_paBestLevel[i])      { xFree(m_paBestLevel[i]);  m_paBestLevel[i]=NULL;  }
  }
  if (m_paBestSPoint)     { xFree(m_paBestSPoint); m_paBestSPoint=NULL; }
  if (m_paBestRun)        { xFree(m_paBestRun);    m_paBestRun=NULL;    }
#endif
}




Void TEncSearch::init(TEncCfg*      pcEncCfg,
                      TComTrQuant*  pcTrQuant,
                      Int           iSearchRange,
                      Int           bipredSearchRange,
                      Int           iFastSearch,
                      Int           iMaxDeltaQP,
                      TEncEntropy*  pcEntropyCoder,
                      TComRdCost*   pcRdCost,
                      TEncSbac*** pppcRDSbacCoder,
                      TEncSbac*   pcRDGoOnSbacCoder
                      )
{
  m_pcEncCfg             = pcEncCfg;
  m_pcTrQuant            = pcTrQuant;
  m_iSearchRange         = iSearchRange;
  m_bipredSearchRange    = bipredSearchRange;
  m_iFastSearch          = iFastSearch;
  m_iMaxDeltaQP          = iMaxDeltaQP;
  m_pcEntropyCoder       = pcEntropyCoder;
  m_pcRdCost             = pcRdCost;

  m_pppcRDSbacCoder     = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder   = pcRDGoOnSbacCoder;

  for (UInt iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++)
  {
    for (UInt iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  m_puiDFilter = s_auiDFilter + 4;

  // initialize motion cost
#if !FIX203
  m_pcRdCost->initRateDistortionModel( m_iSearchRange << 2 );
#endif

  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      else
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
    }
  }

  const ChromaFormat cform=pcEncCfg->getChromaFormatIdc();
  initTempBuff(cform);

  m_pTempPel = new Pel[g_uiMaxCUWidth*g_uiMaxCUHeight];

  const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
  const UInt uiNumPartitions = 1<<(g_uiMaxCUDepth<<1);
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt csx=::getComponentScaleX(ComponentID(ch), cform);
    const UInt csy=::getComponentScaleY(ComponentID(ch), cform);
    m_ppcQTTempCoeff[ch] = new TCoeff* [uiNumLayersToAllocate];
    m_pcQTTempCoeff[ch]   = new TCoeff [(g_uiMaxCUWidth*g_uiMaxCUHeight)>>(csx+csy)   ];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]  = new TCoeff*[uiNumLayersToAllocate];
    m_pcQTTempArlCoeff[ch]   = new TCoeff [(g_uiMaxCUWidth*g_uiMaxCUHeight)>>(csx+csy)   ];
#endif
    m_puhQTTempCbf[ch] = new UChar  [uiNumPartitions];

    for (UInt layer = 0; layer < uiNumLayersToAllocate; layer++)
    {
      m_ppcQTTempCoeff[ch][layer] = new TCoeff[(g_uiMaxCUWidth*g_uiMaxCUHeight)>>(csx+csy)];
#if ADAPTIVE_QP_SELECTION
      m_ppcQTTempArlCoeff[ch][layer]  = new TCoeff[(g_uiMaxCUWidth*g_uiMaxCUHeight)>>(csx+csy) ];
#endif
    }

    m_phQTTempCrossComponentPredictionAlpha[ch]    = new Char  [uiNumPartitions];
    m_pSharedPredTransformSkip[ch]                 = new Pel   [MAX_CU_SIZE*MAX_CU_SIZE];
    m_pcQTTempTUCoeff[ch]                          = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#endif
    m_puhQTTempTransformSkipFlag[ch]               = new UChar [uiNumPartitions];
  }
  m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
  m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
  for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
  {
    m_pcQTTempTComYuv[ui].create( g_uiMaxCUWidth, g_uiMaxCUHeight, pcEncCfg->getChromaFormatIdc() );
  }
  m_pcQTTempTransformSkipTComYuv.create( g_uiMaxCUWidth, g_uiMaxCUHeight, pcEncCfg->getChromaFormatIdc() );
  m_tmpYuvPred.create(MAX_CU_SIZE, MAX_CU_SIZE, pcEncCfg->getChromaFormatIdc());
}

#if FASTME_SMOOTHER_MV
#define FIRSTSEARCHSTOP     1
#else
#define FIRSTSEARCHSTOP     0
#endif

#define TZ_SEARCH_CONFIGURATION                                                                                 \
const Int  iRaster                  = 5;  /* TZ soll von aussen ?ergeben werden */                            \
const Bool bTestOtherPredictedMV    = 0;                                                                      \
const Bool bTestZeroVector          = 1;                                                                      \
const Bool bTestZeroVectorStart     = 0;                                                                      \
const Bool bTestZeroVectorStop      = 0;                                                                      \
const Bool bFirstSearchDiamond      = 1;  /* 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch */        \
const Bool bFirstSearchStop         = FIRSTSEARCHSTOP;                                                        \
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
  m_cDistParam.bitDepth = g_bitDepth[CHANNEL_TYPE_LUMA];
  if(m_pcEncCfg->getFastSearch() == SELECTIVE)
  {
    Int isubShift = 0;
    // motion cost
    UInt uiBitCost = m_pcRdCost->getCost( iSearchX, iSearchY );

    if ( m_cDistParam.iRows > 32 )
      m_cDistParam.iSubShift = 4;
    else if ( m_cDistParam.iRows > 16 )
      m_cDistParam.iSubShift = 3;
    else if ( m_cDistParam.iRows > 8 )
      m_cDistParam.iSubShift = 2;
    else
      m_cDistParam.iSubShift = 1;

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
          break;

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
    m_cDistParam.bitDepth = g_bitDepth[CHANNEL_TYPE_LUMA];
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
        m_pcEntropyCoder->encodeQtCbf(rTu, compID, (uiSubdiv == 0));
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

    if (isChroma(component) && (pcCU->getCbf( rTu.GetAbsPartIdxTU(), COMPONENT_Y, uiTrMode ) != 0) && pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction() )
    {
      m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, component );
    }

    m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeff+uiCoeffOffset, component );
  }
}




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
#if SCM__R0348_PALETTE_MODE
      else // encodePredMode has already done it
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
        }
        m_pcEntropyCoder->encodePLTModeInfo(pcCU, 0, true);
      }
#endif
      if (pcCU->getSlice()->getSPS()->getUseIntraBlockCopy())
      {
        m_pcEntropyCoder->encodeIntraBCFlag ( pcCU, 0, true );
        if ( pcCU->isIntraBC( 0 ) )
        {
          m_pcEntropyCoder->encodePartSizeIntraBC( pcCU, 0 );
          m_pcEntropyCoder->encodeIntraBC( pcCU, 0 );
          return;
        }
      }
#if SCM__R0348_PALETTE_MODE
      if (pcCU->getPLTModeFlag(0))
      {
        return;
      }
#endif
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
      if (uiTrDepth>0 && (uiAbsPartIdx%uiQNumParts)==0) m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
    }
  }

  if( pcCU->isIntraBC( 0 ) )
  {
    return;
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
  if (!rTu.ProcessComponentSection(compID)) return;
  const Bool       bIsLuma = isLuma(compID);
  const TComRectangle &rect= rTu.getRect(compID);
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  const UInt uiTrDepth=rTu.GetTransformDepthRelAdj(compID);
  const UInt uiFullDepth   = rTu.GetTransformDepthTotal();
  const UInt uiLog2TrSize  = rTu.GetLog2LumaTrSize();
  const ChromaFormat chFmt = pcOrgYuv->getChromaFormat();
  const ChannelType chType = toChannelType(compID);

  const UInt    uiWidth           = rect.width;
  const UInt    uiHeight          = rect.height;
  const UInt    uiStride          = pcOrgYuv ->getStride (compID);
        Pel*    piOrg             = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
        Pel*    piPred            = pcPredYuv->getAddr( compID, uiAbsPartIdx );
        Pel*    piResi            = pcResiYuv->getAddr( compID, uiAbsPartIdx );
        Pel*    piReco            = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const UInt    uiQTLayer           = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
        Pel*    piRecQt           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
  const UInt    uiRecQtStride     = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
  const UInt    uiZOrder            = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
        Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
        UInt    uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );
        TCoeff* pcCoeff           = m_ppcQTTempCoeff[compID][uiQTLayer] + rTu.getCoefficientOffset(compID);
        Bool    useTransformSkip  = pcCU->getTransformSkip(uiAbsPartIdx, compID);

#if ADAPTIVE_QP_SELECTION
        TCoeff*    pcArlCoeff     = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#endif

  const UInt uiChPredMode         = pcCU->getIntraDir( chType, uiAbsPartIdx );
  const UInt uiChCodedMode        = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt)) : uiChPredMode;
  const UInt uiChFinalMode        = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  const Int         blkX                                 = g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int         blkY                                 = g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int         bufferOffset                         = blkX + (blkY * MAX_CU_SIZE);
        Pel  *const encoderLumaResidual                  = resiLuma[RESIDUAL_ENCODER_SIDE ] + bufferOffset;
        Pel  *const reconstructedLumaResidual            = resiLuma[RESDIUAL_RECONSTRUCTED] + bufferOffset;
  const Bool        bUseCrossCPrediction                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX) && checkCrossCPrediction;
  const Bool        bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
        Pel *const  lumaResidualForEstimate              = bUseReconstructedResidualForEstimate ? reconstructedLumaResidual : encoderLumaResidual;

#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
#endif

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  QpParam cQP(*pcCU, compID); 
  if(!pcCU->isLosslessCoded(0) && pcCU->getColorTransform(0))
  {
    cQP.Qp = cQP.Qp + (compID==COMPONENT_Cr ? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
    cQP.per = cQP.Qp/6;
    cQP.rem= cQP.Qp%6;
  }
#endif 

  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;

  DEBUG_STRING_NEW(sTemp)

#ifndef DEBUG_STRING
  if( default0Save1Load2 != 2 )
#endif
  {
    const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

    initAdiPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sDebug) );

    //===== get prediction signal =====
    predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

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
#ifndef DEBUG_STRING
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

  if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
  {
    if (bUseCrossCPrediction)
    {
      if (xCalcCrossComponentPredictionAlpha( rTu, compID, lumaResidualForEstimate, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride ) == 0) return;
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

#if !SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  const QpParam cQP(*pcCU, compID);
#endif

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

#ifdef DEBUG_STRING
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
    const UInt clipbd=g_bitDepth[chType];

    if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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

 #ifdef DEBUG_STRING
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
        if (bDebugResi) ss << " - resi: ";
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          if (bDebugResi) ss << pResi[ uiX ] << ", ";
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), clipbd ));
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
          pReco    [ uiX ] = Pel(ClipBD<Int>( Int(pPred[uiX]) + Int(pResi[uiX]), clipbd ));
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
  ruiDist += m_pcRdCost->getDistPart( g_bitDepth[chType], piReco, uiStride, piOrg, uiStride, uiWidth, uiHeight, compID );
}

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
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

#ifdef DEBUG_STRING
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

  if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction() && bUseCrossCPrediction)
    TComTrQuant::crossComponentPrediction( rTu, compID, reconstructedLumaResidual, codedResi, codedResi, uiWidth, uiHeight, MAX_CU_SIZE, uiCodedResiStride, uiCodedResiStride, false );

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

#ifdef DEBUG_STRING
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
  if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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
#endif





Void
TEncSearch::xRecurIntraCodingQT(Bool        bLumaOnly,
                                TComYuv*    pcOrgYuv,
                                TComYuv*    pcPredYuv,
                                TComYuv*    pcResiYuv,
                                Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                Distortion& ruiDistY,
                                Distortion& ruiDistC,
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
  const UInt    numValidComp  = (bLumaOnly) ? 1 : pcOrgYuv->getNumberValidComponents();

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
  Double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDist[MAX_NUM_CHANNEL_TYPE] = {0,0};
  UInt       uiSingleCbf[MAX_NUM_COMPONENT]     = {0,0,0};
  Bool       checkTransformSkip  = pcCU->getSlice()->getPPS()->getUseTransformSkip();
  Int        bestModeId[MAX_NUM_COMPONENT] = { 0, 0, 0};
  checkTransformSkip           &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize());
  checkTransformSkip           &= (!pcCU->getCUTransquantBypass(0));

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

      Distortion singleDistTmp[MAX_NUM_CHANNEL_TYPE]  = { 0, 0 };
      UInt       singleCbfTmp[MAX_NUM_COMPONENT]      = { 0, 0, 0 };
      Double     singleCostTmp                        = 0;
      Int        firstCheckId                         = 0;

      for(Int modeId = firstCheckId; modeId < 2; modeId ++)
      {
        DEBUG_STRING_NEW(sModeString)
        Int  default0Save1Load2 = 0;
        singleDistTmp[0]=singleDistTmp[1]=0;
        if(modeId == firstCheckId)
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }

        for(UInt ch=COMPONENT_Y; ch<numValidComp; ch++)
        {
          const ComponentID compID = ComponentID(ch);
          if (rTu.ProcessComponentSection(compID))
          {
            const UInt totalAdjustedDepthChan = rTu.GetTransformDepthTotalAdj(compID);
            pcCU->setTransformSkipSubParts ( modeId, compID, uiAbsPartIdx, totalAdjustedDepthChan );

            xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, singleDistTmp[toChannelType(compID)], compID, rTu DEBUG_STRING_PASS_INTO(sModeString), default0Save1Load2 );
          }
          singleCbfTmp[compID] = pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth );
        }
        //----- determine rate and r-d cost -----
        if(modeId == 1 && singleCbfTmp[COMPONENT_Y] == 0)
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          UInt uiSingleBits = xGetIntraBitsQT( rTu, true, !bLumaOnly, false );
          singleCostTmp     = m_pcRdCost->calcRdCost( uiSingleBits, singleDistTmp[CHANNEL_TYPE_LUMA] + singleDistTmp[CHANNEL_TYPE_CHROMA] );
        }
        if(singleCostTmp < dSingleCost)
        {
          DEBUG_STRING_SWAP(sDebug, sModeString)
          dSingleCost   = singleCostTmp;
          uiSingleDist[CHANNEL_TYPE_LUMA] = singleDistTmp[CHANNEL_TYPE_LUMA];
          uiSingleDist[CHANNEL_TYPE_CHROMA] = singleDistTmp[CHANNEL_TYPE_CHROMA];
          for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
            uiSingleCbf[ch] = singleCbfTmp[ch];

          bestModeId[COMPONENT_Y] = modeId;
          if(bestModeId[COMPONENT_Y] == firstCheckId)
          {
            xStoreIntraResultQT(COMPONENT_Y, bLumaOnly?COMPONENT_Y:COMPONENT_Cr, rTu );
            m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
          }

          if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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

      for(UInt ch=COMPONENT_Y; ch<numValidComp; ch++)
      {
        const ComponentID compID=ComponentID(ch);
        if (rTu.ProcessComponentSection(compID))
        {
          const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(compID);
          pcCU ->setTransformSkipSubParts ( bestModeId[COMPONENT_Y], compID, uiAbsPartIdx, totalAdjustedDepthChan );
        }
      }

      if(bestModeId[COMPONENT_Y] == firstCheckId)
      {
        xLoadIntraResultQT(COMPONENT_Y, bLumaOnly?COMPONENT_Y:COMPONENT_Cr, rTu );
        for(UInt ch=COMPONENT_Y; ch< numValidComp; ch++)
        {
          const ComponentID compID=ComponentID(ch);
          if (rTu.ProcessComponentSection(compID))
            pcCU->setCbfSubParts  ( uiSingleCbf[compID] << uiTrDepth, compID, uiAbsPartIdx, rTu.GetTransformDepthTotalAdj(compID) );
        }

        m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      }

      if( !bLumaOnly )
      {
        bestModeId[COMPONENT_Cb] = bestModeId[COMPONENT_Cr] = bestModeId[COMPONENT_Y];
        if (rTu.ProcessComponentSection(COMPONENT_Cb) && bestModeId[COMPONENT_Y] == 1)
        {
          //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          for (UInt ch=COMPONENT_Cb; ch<numValidComp; ch++)
          {
            if (uiSingleCbf[ch] == 0)
            {
              const ComponentID compID=ComponentID(ch);
              const UInt totalAdjustedDepthChan = rTu.GetTransformDepthTotalAdj(compID);
              pcCU ->setTransformSkipSubParts ( 0, compID, uiAbsPartIdx, totalAdjustedDepthChan);
              bestModeId[ch] = 0;
            }
          }
        }
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
      for (UInt ch=COMPONENT_Y; ch<numValidComp; ch++)
      {
        const ComponentID compID = ComponentID(ch);

        if (rTu.ProcessComponentSection(compID))
        {
          const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(compID);
          pcCU ->setTransformSkipSubParts ( 0, compID, uiAbsPartIdx, totalAdjustedDepthChan );
        }

        xIntraCodingTUBlock( pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSingle, false, uiSingleDist[toChannelType(compID)], compID, rTu DEBUG_STRING_PASS_INTO(sDebug));

        if( bCheckSplit )
        {
          uiSingleCbf[compID] = pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth );
        }
      }
      //----- determine rate and r-d cost -----
      UInt uiSingleBits = xGetIntraBitsQT( rTu, true, !bLumaOnly, false );

      if(m_pcEncCfg->getRDpenalty() && (uiLog2TrSize==5) && !isIntraSlice)
      {
        uiSingleBits=uiSingleBits*4;
      }

      dSingleCost       = m_pcRdCost->calcRdCost( uiSingleBits, uiSingleDist[CHANNEL_TYPE_LUMA] + uiSingleDist[CHANNEL_TYPE_CHROMA] );

      if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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
    Double     dSplitCost                         = 0.0;
    Distortion uiSplitDist[MAX_NUM_CHANNEL_TYPE]  = {0,0};
    UInt       uiSplitCbf[MAX_NUM_COMPONENT]      = {0,0,0};

    TComTURecurse tuRecurseChild(rTu, false);
    DEBUG_STRING_NEW(sSplit)
    do
    {
      DEBUG_STRING_NEW(sChild)
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDist[0], uiSplitDist[1], bCheckFirst, dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#else
      xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaSplit, uiSplitDist[0], uiSplitDist[1], dSplitCost, tuRecurseChild DEBUG_STRING_PASS_INTO(sChild) );
#endif
      DEBUG_STRING_APPEND(sSplit, sChild)
      for(UInt ch=0; ch<numValidComp; ch++)
      {
        uiSplitCbf[ch] |= pcCU->getCbf( tuRecurseChild.GetAbsPartIdxTU(), ComponentID(ch), tuRecurseChild.GetTransformDepthRel() );
      }
    } while (tuRecurseChild.nextSection(rTu) );

    UInt    uiPartsDiv     = rTu.GetAbsPartIdxNumParts();
    for(UInt ch=COMPONENT_Y; ch<numValidComp; ch++)
    {
      if (uiSplitCbf[ch])
      {
        const UInt flag=1<<uiTrDepth;
        const ComponentID compID=ComponentID(ch);
        UChar *pBase=pcCU->getCbf( compID );
        for( UInt uiOffs = 0; uiOffs < uiPartsDiv; uiOffs++ )
        {
          pBase[ uiAbsPartIdx + uiOffs ] |= flag;
        }
      }
    }
    //----- restore context states -----
    m_pcRDGoOnSbacCoder->load ( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    
    //----- determine rate and r-d cost -----
    UInt uiSplitBits = xGetIntraBitsQT( rTu, true, !bLumaOnly, false );
    dSplitCost       = m_pcRdCost->calcRdCost( uiSplitBits, uiSplitDist[CHANNEL_TYPE_LUMA] + uiSplitDist[CHANNEL_TYPE_CHROMA] );

    //===== compare and set best =====
    if( dSplitCost < dSingleCost )
    {
      //--- update cost ---
      DEBUG_STRING_SWAP(sSplit, sDebug)
      ruiDistY += uiSplitDist[CHANNEL_TYPE_LUMA];
      ruiDistC += uiSplitDist[CHANNEL_TYPE_CHROMA];
      dRDCost  += dSplitCost;

      if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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
    for(UInt ch=0; ch<numValidComp; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      const TComRectangle &tuRect=rTu.getRect(compID);
      const UInt totalAdjustedDepthChan   = rTu.GetTransformDepthTotalAdj(compID);
      pcCU->setCbfSubParts  ( uiSingleCbf[compID] << uiTrDepth, compID, uiAbsPartIdx, totalAdjustedDepthChan );
      pcCU ->setTransformSkipSubParts  ( bestModeId[compID], compID, uiAbsPartIdx, totalAdjustedDepthChan );

      //--- set reconstruction for next intra prediction blocks ---
      const UInt  uiQTLayer   = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
      const UInt  uiZOrder    = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
      const UInt  uiWidth     = tuRect.width;
      const UInt  uiHeight    = tuRect.height;
      Pel*  piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt  uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( compID );
      Pel*  piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
      UInt  uiDesStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );

      for( UInt uiY = 0; uiY < uiHeight; uiY++, piSrc += uiSrcStride, piDes += uiDesStride )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          piDes[ uiX ] = piSrc[ uiX ];
        }
      }
    }
  }
  ruiDistY += uiSingleDist[CHANNEL_TYPE_LUMA];
  ruiDistC += uiSingleDist[CHANNEL_TYPE_CHROMA];
  dRDCost  += dSingleCost;
}


Void
TEncSearch::xSetIntraResultQT(Bool        bLumaOnly,
                              TComYuv*    pcRecoYuv,
                              TComTU     &rTu)
{
  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiTrDepth    = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if(  uiTrMode == uiTrDepth )
  {
    UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;

    Bool bSkipChroma = !rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA);

    //===== copy transform coefficients =====

    const UInt numChannelsToProcess = (bLumaOnly || bSkipChroma) ? 1 : ::getNumberValidComponents(pcCU->getPic()->getChromaFormat());
    for (UInt ch=0; ch<numChannelsToProcess; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      const TComRectangle &tuRect=rTu.getRect(compID);
      const UInt coeffOffset = rTu.getCoefficientOffset(compID);
      const UInt numCoeffInBlock = tuRect.width * tuRect.height;

      if (numCoeffInBlock!=0)
      {
        const TCoeff* srcCoeff = m_ppcQTTempCoeff[compID][uiQTLayer] + coeffOffset;
        TCoeff* destCoeff      = pcCU->getCoeff(compID) + coeffOffset;
        ::memcpy( destCoeff, srcCoeff, sizeof(TCoeff)*numCoeffInBlock );
#if ADAPTIVE_QP_SELECTION
        const TCoeff* srcArlCoeff = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + coeffOffset;
        TCoeff* destArlCoeff      = pcCU->getArlCoeff (compID)               + coeffOffset;
        ::memcpy( destArlCoeff, srcArlCoeff, sizeof( TCoeff ) * numCoeffInBlock );
#endif
        m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( compID, pcRecoYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
      }
    } // End of channel loop

  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xSetIntraResultQT( bLumaOnly, pcRecoYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}


Void
TEncSearch::xStoreIntraResultQT(const ComponentID first,
                                const ComponentID lastIncl,
                                      TComTU &rTu )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if (  first==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;


    for(UInt compID_=first; compID_<=lastIncl; compID_++)
    {
      ComponentID compID=ComponentID(compID_);
      if (rTu.ProcessComponentSection(compID))
      {
        const TComRectangle &tuRect=rTu.getRect(compID);

        //===== copy transform coefficients =====
        const UInt uiNumCoeff    = tuRect.width * tuRect.height;
        TCoeff* pcCoeffSrc = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
        TCoeff* pcCoeffDst = m_pcQTTempTUCoeff[compID];

        ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
        TCoeff* pcArlCoeffSrc = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
        TCoeff* pcArlCoeffDst = m_ppcQTTempTUArlCoeff[compID];
        ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
        //===== copy reconstruction =====
        m_pcQTTempTComYuv[ uiQTLayer ].copyPartToPartComponent( compID, &m_pcQTTempTransformSkipTComYuv, uiAbsPartIdx, tuRect.width, tuRect.height );
      }
    }
  }
}


Void
TEncSearch::xLoadIntraResultQT(const ComponentID first,
                               const ComponentID lastIncl,
                                     TComTU &rTu)
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiTrDepth = rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if (  first==COMPONENT_Y || uiTrMode == uiTrDepth )
  {
    assert(uiTrMode == uiTrDepth);
    const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();
    const UInt uiQTLayer    = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrSize;
    const UInt uiZOrder     = pcCU->getZorderIdxInCU() + uiAbsPartIdx;

    for(UInt compID_=first; compID_<=lastIncl; compID_++)
    {
      ComponentID compID=ComponentID(compID_);
      if (rTu.ProcessComponentSection(compID))
      {
        const TComRectangle &tuRect=rTu.getRect(compID);

        //===== copy transform coefficients =====
        const UInt uiNumCoeff = tuRect.width * tuRect.height;
        TCoeff* pcCoeffDst = m_ppcQTTempCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
        TCoeff* pcCoeffSrc = m_pcQTTempTUCoeff[compID];

        ::memcpy( pcCoeffDst, pcCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#if ADAPTIVE_QP_SELECTION
        TCoeff* pcArlCoeffDst = m_ppcQTTempArlCoeff[compID] [ uiQTLayer ] + rTu.getCoefficientOffset(compID);
        TCoeff* pcArlCoeffSrc = m_ppcQTTempTUArlCoeff[compID];
        ::memcpy( pcArlCoeffDst, pcArlCoeffSrc, sizeof( TCoeff ) * uiNumCoeff );
#endif
        //===== copy reconstruction =====
        m_pcQTTempTransformSkipTComYuv.copyPartToPartComponent( compID, &m_pcQTTempTComYuv[ uiQTLayer ], uiAbsPartIdx, tuRect.width, tuRect.height );

        Pel*    piRecIPred        = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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
    if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA)) return;

    const UInt uiFullDepth = rTu.GetTransformDepthTotal();

    Bool checkTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip();
    checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Cb), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize());

    if ( m_pcEncCfg->getUseTransformSkipFast() )
    {
      checkTransformSkip &= TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(COMPONENT_Y), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize());

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
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
      if(!pcCU->isLosslessCoded(0) && pcCU->getColorTransform( 0 ))
      {
        m_pcTrQuant->adjustBitDepthandLambdaForColorTrans(ch==COMPONENT_Cr ? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
        m_pcRdCost->adjustLambdaForColorTrans            (ch==COMPONENT_Cr ? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
      }
#endif
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
                                                   &&  pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction()
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
            const Bool isLastMode = (currModeId == totalModesToTest); //NOTE: RExt - currModeId is indexed from 1

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
                xStoreIntraResultQT(compID, compID, TUIterator);
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
          xLoadIntraResultQT(compID, compID, TUIterator);
          pcCU->setCbfPartRange( singleCbfC << uiTrDepth, compID, subTUAbsPartIdx, partIdxesPerSubTU );

          m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
        }

        DEBUG_STRING_APPEND(sDebug, sDebugBestMode)
        pcCU ->setTransformSkipPartRange                ( bestTransformSkipMode,     compID, subTUAbsPartIdx, partIdxesPerSubTU );
        pcCU ->setCrossComponentPredictionAlphaPartRange( bestCrossCPredictionAlpha, compID, subTUAbsPartIdx, partIdxesPerSubTU );
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
        if(!pcCU->getColorTransform( 0 ))
#endif
        ruiDist += singleDistC;
      }
      while (TUIterator.nextSection(rTu));

      if (splitIntoSubTUs) offsetSubTUCBFs(rTu, compID);
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
      if(!pcCU->isLosslessCoded(0) && pcCU->getColorTransform( 0 ))
      {
        m_pcTrQuant->adjustBitDepthandLambdaForColorTrans(compID==COMPONENT_Cr ? - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V:  - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
        m_pcRdCost->adjustLambdaForColorTrans            (compID==COMPONENT_Cr ? - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V:  - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
      }
#endif 
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



#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
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

      Bool       bAboveAvail             = false;
      Bool       bLeftAvail              = false;
      const Bool bUseFilteredPredictions = TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());
      Pel*       piOrg                   = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
      Pel*       piPred                  = pcPredYuv->getAddr( compID, uiAbsPartIdx );
      Pel*       piResi                  = pcResiYuv->getAddr( compID, uiAbsPartIdx );
      const UInt uiStride                = pcOrgYuv ->getStride (compID);

      DEBUG_STRING_NEW(sTemp)
      initAdiPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sTemp) );
      predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

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


    pcResiYuv->convert(rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, true, pcCU->isLosslessCoded(0));

    Char preCalcAlpha[MAX_NUM_COMPONENT] = {0, 0, 0};

    if( pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction() && !m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate() ) 
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
      QpParam cQP(*pcCU, compID);
      if(!pcCU->isLosslessCoded(0) )
      {
        cQP.Qp = cQP.Qp + (compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
        cQP.per = cQP.Qp/6;
        cQP.rem= cQP.Qp%6;
        m_pcTrQuant->adjustBitDepthandLambdaForColorTrans(compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
        m_pcRdCost->adjustLambdaForColorTrans(compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
      }

      Bool bCheckTransformSkip = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
        TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) &&
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
        && pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction()
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
            uiSingleDistTmp = m_pcRdCost->getDistPart( g_bitDepth[toChannelType( compID )],
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

            xStoreIntraResultQT( compID, compID, rTu );

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

      xLoadIntraResultQT(compID, compID, rTu );

      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_TEMP_BEST ] );
      if(!pcCU->isLosslessCoded(0))
      {
        m_pcTrQuant->adjustBitDepthandLambdaForColorTrans( compID==COMPONENT_Cr? -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
        m_pcRdCost->adjustLambdaForColorTrans            ( compID==COMPONENT_Cr? -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
      }
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiFullDepth ][ CI_QT_TRAFO_ROOT ] );
    m_pcEntropyCoder->resetBits();

    uiSingleBits = xGetIntraBitsQT( rTu, true, true, false );


    m_pcQTTempTComYuv[ uiQTLayer ].convert(rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->isLosslessCoded(0));

    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      ComponentID         compID           = ComponentID(ch);
      const TComRectangle &rect            = rTu.getRect(compID);
      const UInt          uiWidth          = rect.width;
      const UInt          uiHeight         = rect.height;
      const UInt          uiZOrder         = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
      Pel*                pcPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
      Pel*                pcResi           = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      Pel*                pcRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      Pel*                piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
      const UInt          uiStride         = pcPredYuv->getStride (compID);
      const UInt          uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
      const UInt          uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride (compID);
      const UInt          clipbd           = g_bitDepth[toChannelType(compID)];

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

    uiSingleDist[CHANNEL_TYPE_LUMA] = m_pcRdCost->getDistPart( g_bitDepth[toChannelType(COMPONENT_Y)],  
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
      uiSingleDist[CHANNEL_TYPE_CHROMA] += m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], 
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
      const UInt  uiZOrder    = pcCU->getZorderIdxInCU() + uiAbsPartIdx;
      const UInt  uiWidth     = tuRect.width;
      const UInt  uiHeight    = tuRect.height;
      Pel*        piSrc       = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
      UInt        uiSrcStride = m_pcQTTempTComYuv[ uiQTLayer ].getStride  ( compID );
      Pel*        piDes       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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
#endif
Void
TEncSearch::xSetIntraResultChromaQT(TComYuv*    pcRecoYuv, TComTU &rTu)
{
  if (!rTu.ProcessChannelSection(CHANNEL_TYPE_CHROMA)) return;
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
TEncSearch::preestChromaPredMode( TComDataCU* pcCU,
                                 TComYuv*    pcOrgYuv,
                                 TComYuv*    pcPredYuv )
{

  //===== loop over partitions =====
  const UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;
  TComTURecurse tuRecurseCU(pcCU, 0);
  TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);
  const ChromaFormat chFmt = tuRecurseWithPU.GetChromaFormat();
  Bool bFilterEnabled=filterIntraReferenceSamples(CHANNEL_TYPE_CHROMA, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

  do
  {
    if (tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
    {
      const TComRectangle &rect=tuRecurseWithPU.getRect(COMPONENT_Cb);
      const UInt  uiWidth     = rect.width;
      const UInt  uiHeight    = rect.height;
      const UInt  partIdx     = tuRecurseWithPU.GetAbsPartIdxCU();
      const UInt  uiStride    = pcOrgYuv ->getStride(COMPONENT_Cb);
      Pel*  piOrgU      = pcOrgYuv ->getAddr ( COMPONENT_Cb, partIdx ); //TODO: RExt - Change this into an array and loop over chroma components below
      Pel*  piOrgV      = pcOrgYuv ->getAddr ( COMPONENT_Cr, partIdx );
      Pel*  piPredU     = pcPredYuv->getAddr ( COMPONENT_Cb, partIdx );
      Pel*  piPredV     = pcPredYuv->getAddr ( COMPONENT_Cr, partIdx );

      //===== init pattern =====
      Bool  bAboveAvail = false;
      Bool  bLeftAvail  = false;
      DEBUG_STRING_NEW(sTemp)
      initAdiPatternChType( tuRecurseWithPU, bAboveAvail, bLeftAvail, COMPONENT_Cb, bFilterEnabled DEBUG_STRING_PASS_INTO(sTemp) );
      initAdiPatternChType( tuRecurseWithPU, bAboveAvail, bLeftAvail, COMPONENT_Cr, bFilterEnabled DEBUG_STRING_PASS_INTO(sTemp) );

      //===== get best prediction modes (using SAD) =====
            UInt        uiMinMode          = 0;
            UInt        uiMaxMode          = 4;
            UInt        uiBestMode         = MAX_UINT;
            Distortion  uiMinSAD           = std::numeric_limits<Distortion>::max();
      const UInt        mappedModeTable[4] = {PLANAR_IDX,DC_IDX,HOR_IDX,VER_IDX};

      DistParam distParamU, distParamV;
      const Bool bUseHadamard=pcCU->getCUTransquantBypass(0) == 0;
      m_pcRdCost->setDistParam(distParamU, g_bitDepth[CHANNEL_TYPE_CHROMA], piOrgU, uiStride, piPredU, uiStride, uiWidth, uiHeight, bUseHadamard);
      m_pcRdCost->setDistParam(distParamV, g_bitDepth[CHANNEL_TYPE_CHROMA], piOrgV, uiStride, piPredV, uiStride, uiWidth, uiHeight, bUseHadamard);
      distParamU.bApplyWeight = false;
      distParamV.bApplyWeight = false;

      for( UInt uiMode_  = uiMinMode; uiMode_ < uiMaxMode; uiMode_++ )
      {
        UInt uiMode=mappedModeTable[uiMode_];
        //--- get prediction ---
        const Bool bUseFilter=TComPrediction::filteringIntraReferenceSamples(COMPONENT_Cb, uiMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

        predIntraAng( COMPONENT_Cb, uiMode, piOrgU, uiStride, piPredU, uiStride, tuRecurseCU, bAboveAvail, bLeftAvail, bUseFilter );
        predIntraAng( COMPONENT_Cr, uiMode, piOrgV, uiStride, piPredV, uiStride, tuRecurseCU, bAboveAvail, bLeftAvail, bUseFilter );

        //--- get SAD ---
        Distortion uiSAD  = distParamU.DistFunc(&distParamU);
        uiSAD            += distParamV.DistFunc(&distParamV);
        //--- check ---
        if( uiSAD < uiMinSAD )
        {
          uiMinSAD   = uiSAD;
          uiBestMode = uiMode;
        }
      }

      //===== set chroma pred mode =====
      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestMode, partIdx, tuRecurseWithPU.getCUDepth() + uiInitTrDepth );
    }
  } while (tuRecurseWithPU.nextSection(tuRecurseCU));
}




Void
TEncSearch::estIntraPredQT(TComDataCU* pcCU,
                           TComYuv*    pcOrgYuv,
                           TComYuv*    pcPredYuv,
                           TComYuv*    pcResiYuv,
                           TComYuv*    pcRecoYuv,
                           Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                           Distortion& ruiDistC,
                           Bool        bLumaOnly
                           DEBUG_STRING_FN_DECLARE(sDebug))
{
  const UInt         uiDepth               = pcCU->getDepth(0);
  const UInt         uiInitTrDepth         = pcCU->getPartitionSize(0) == SIZE_2Nx2N ? 0 : 1;
  const UInt         uiInitTrDepthC        = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;
  const UInt         uiNumPU               = 1<<(2*uiInitTrDepth);
  const UInt         uiQNumParts           = pcCU->getTotalNumPart() >> 2;
  const UInt         uiWidthBit            = pcCU->getIntraSizeIdx(0);
  const ChromaFormat chFmt                 = pcCU->getPic()->getChromaFormat();
  const UInt         numberValidComponents = getNumberValidComponents(chFmt);
        Distortion   uiOverallDistY        = 0;
        Distortion   uiOverallDistC        = 0;
        UInt         CandNum;
        Double       CandCostList[ FAST_UDI_MAX_RDMODE_NUM ];
        Pel          resiLumaPU[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

        Bool    bMaintainResidual[NUMBER_OF_STORED_RESIDUAL_TYPES];
        for (UInt residualTypeIndex = 0; residualTypeIndex < NUMBER_OF_STORED_RESIDUAL_TYPES; residualTypeIndex++)
        {
          bMaintainResidual[residualTypeIndex] = true; //assume true unless specified otherwise
        }

        bMaintainResidual[RESIDUAL_ENCODER_SIDE] = !(m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate());

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
        if(!pcCU->isLosslessCoded(0) && pcCU->getColorTransform( 0 ))
        {
          m_pcTrQuant->adjustBitDepthandLambdaForColorTrans( SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
          m_pcRdCost->adjustLambdaForColorTrans            ( SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
        }
#endif


  //NOTE: RExt - Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantisation divisor is 1.
#if FULL_NBIT
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#else
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
                sqrt(0.57 * pow(2.0, ((RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8)) / 3.0)))
              : m_pcRdCost->getSqrtLambda();
#endif

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
    const UInt uiPartOffset=tuRecurseWithPU.GetAbsPartIdxTU();
//  for( UInt uiPU = 0, uiPartOffset=0; uiPU < uiNumPU; uiPU++, uiPartOffset += uiQNumParts )
  //{
    //===== init pattern for luma prediction =====
    Bool bAboveAvail = false;
    Bool bLeftAvail  = false;
    DEBUG_STRING_NEW(sTemp2)

    //===== determine set of modes to be tested (using prediction signal only) =====
    Int numModesAvailable     = 35; //total number of Intra modes
    UInt uiRdModeList[FAST_UDI_MAX_RDMODE_NUM];
    Int numModesForFullRD = g_aucIntraModeNumFast[ uiWidthBit ];

    if (tuRecurseWithPU.ProcessComponentSection(COMPONENT_Y))
      initAdiPatternChType( tuRecurseWithPU, bAboveAvail, bLeftAvail, COMPONENT_Y, true DEBUG_STRING_PASS_INTO(sTemp2) );

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
      m_pcRdCost->setDistParam(distParam, g_bitDepth[CHANNEL_TYPE_LUMA], piOrg, uiStride, piPred, uiStride, puRect.width, puRect.height, bUseHadamard);
      distParam.bApplyWeight = false;
      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt       uiMode = modeIdx;
        Distortion uiSad  = 0;

        const Bool bUseFilter=TComPrediction::filteringIntraReferenceSamples(COMPONENT_Y, uiMode, puRect.width, puRect.height, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

        predIntraAng( COMPONENT_Y, uiMode, piOrg, uiStride, piPred, uiStride, tuRecurseWithPU, bAboveAvail, bLeftAvail, bUseFilter, TComPrediction::UseDPCMForFirstPassIntraEstimation(tuRecurseWithPU, uiMode) );

        // use hadamard transform here
        uiSad+=distParam.DistFunc(&distParam);

        UInt   iModeBits = 0;

        // NB xModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
        iModeBits+=xModeBitsIntra( pcCU, uiMode, uiPartOffset, uiDepth, uiInitTrDepth, CHANNEL_TYPE_LUMA );

        Double cost      = (Double)uiSad + (Double)iModeBits * sqrtLambdaForFirstPass;

#ifdef DEBUG_INTRA_SEARCH_COSTS
        std::cout << "1st pass mode " << uiMode << " SAD = " << uiSad << ", mode bits = " << iModeBits << ", cost = " << cost << "\n";
#endif

        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

#if FAST_UDI_USE_MPM
      Int uiPreds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};

      Int iMode = -1;
      Int numCand = pcCU->getIntraDirPredictor( uiPartOffset, uiPreds, COMPONENT_Y, &iMode );

      if( iMode >= 0 )
      {
        numCand = iMode;
      }

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
#endif // FAST_UDI_USE_MPM
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
    Distortion uiBestPUDistC = 0;
    Double     dBestPUCost   = MAX_DOUBLE;

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
    UInt max=numModesForFullRD;

    if (DebugOptionList::ForceLumaMode.isSet()) max=0;  // we are forcing a direction, so don't bother with mode check
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
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0.0;
#if HHI_RQT_INTRA_SPEEDUP
      xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, uiPUDistC, true, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#else
      xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
#endif

#ifdef DEBUG_INTRA_SEARCH_COSTS
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
        uiBestPUDistC = uiPUDistC;
        dBestPUCost   = dPUCost;

        xSetIntraResultQT( bLumaOnly, pcRecoYuv, tuRecurseWithPU );

        if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      if (DebugOptionList::ForceLumaMode.isSet())
        uiOrgMode = DebugOptionList::ForceLumaMode.getInt();
#endif

      pcCU->setIntraDirSubParts ( CHANNEL_TYPE_LUMA, uiOrgMode, uiPartOffset, uiDepth + uiInitTrDepth );
      DEBUG_STRING_NEW(sModeTree)

      // set context models
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      // determine residual for partition
      Distortion uiPUDistY = 0;
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0.0;

      xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiPUDistY, uiPUDistC, false, dPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sModeTree));

      // check r-d cost
      if( dPUCost < dBestPUCost )
      {
        DEBUG_STRING_SWAP(sPU, sModeTree)
        uiBestPUMode  = uiOrgMode;
        uiBestPUDistY = uiPUDistY;
        uiBestPUDistC = uiPUDistC;
        dBestPUCost   = dPUCost;

        xSetIntraResultQT( bLumaOnly, pcRecoYuv, tuRecurseWithPU );

        if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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
    uiOverallDistC += uiBestPUDistC;

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
      const Bool bSkipChroma  = tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA);

      const UInt numChannelToProcess = (bLumaOnly || bSkipChroma) ? 1 : getNumberValidComponents(pcCU->getPic()->getChromaFormat());

      for (UInt ch=0; ch<numChannelToProcess; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        const TComRectangle &puRect=tuRecurseWithPU.getRect(compID);
        const UInt  uiCompWidth   = puRect.width;
        const UInt  uiCompHeight  = puRect.height;

        const UInt  uiZOrder      = pcCU->getZorderIdxInCU() + uiPartOffset;
              Pel*  piDes         = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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

    //=== update PU data ====
    pcCU->setIntraDirSubParts     ( CHANNEL_TYPE_LUMA, uiBestPUMode, uiPartOffset, uiDepth + uiInitTrDepth );
    if (!bLumaOnly && getChromasCorrespondingPULumaIdx(uiPartOffset, chFmt)==uiPartOffset)
    {
      UInt chromaDir=pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, getChromasCorrespondingPULumaIdx(uiPartOffset, chFmt));
      if (chromaDir == uiBestPUMode && tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA))
      {
        pcCU->setIntraDirSubParts     ( CHANNEL_TYPE_CHROMA, DM_CHROMA_IDX, getChromasCorrespondingPULumaIdx(uiPartOffset, chFmt), uiDepth + uiInitTrDepthC );
      }
    }
    //pcCU->copyToPic                   ( uiDepth, uiPU, uiInitTrDepth ); // Unnecessary copy?
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
  ruiDistC                   = uiOverallDistC;
  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  if(!pcCU->isLosslessCoded(0) && pcCU->getColorTransform( 0 ))
  {
    m_pcTrQuant->adjustBitDepthandLambdaForColorTrans( -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
    m_pcRdCost->adjustLambdaForColorTrans            ( -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
  }
#endif
}




Void
TEncSearch::estIntraPredChromaQT(TComDataCU* pcCU,
                                 TComYuv*    pcOrgYuv,
                                 TComYuv*    pcPredYuv,
                                 TComYuv*    pcResiYuv,
                                 TComYuv*    pcRecoYuv,
                                 Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                 Distortion  uiPreCalcDistC
                                 DEBUG_STRING_FN_DECLARE(sDebug))
{
  pcCU->getTotalDistortion      () -= uiPreCalcDistC;

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  assert( pcCU->getColorTransform( 0 ) == false );
#endif

  //const UInt    uiDepthCU     = pcCU->getDepth(0);
  const UInt    uiInitTrDepth  = pcCU->getPartitionSize(0) != SIZE_2Nx2N && enable4ChromaPUsInIntraNxNCU(pcOrgYuv->getChromaFormat()) ? 1 : 0;
//  const UInt    uiNumPU        = 1<<(2*uiInitTrDepth);

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

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
        if (DebugOptionList::ForceChromaMode.isSet())
        {
          uiMinMode=DebugOptionList::ForceChromaMode.getInt();
          if (uiModeList[uiMinMode]==34) uiMinMode=5; // if the fixed mode has been renumbered because DM_CHROMA covers it, use DM_CHROMA.
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
          const UInt  uiZOrder        = pcCU->getZorderIdxInCU() + tuRecurseWithPU.GetAbsPartIdxTU();
                Pel*  piDes           = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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



#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
Void
TEncSearch::estIntraPredQTCT( TComDataCU* pcCU,
                              TComYuv* pcOrgYuv,
                              TComYuv* pcPredYuv,
                              TComYuv* pcResiYuv,
                              TComYuv* pcRecoYuv
                              DEBUG_STRING_FN_DECLARE(sDebug)
                            )
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

  Bool                bReuse               = !m_pcEncCfg->getRGBFormatFlag();

#if FULL_NBIT
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
    sqrt(0.57 * pow(2.0, ((RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0)))
    : m_pcRdCost->getSqrtLambda();
#else
  const Double sqrtLambdaForFirstPass= (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING && pcCU->getCUTransquantBypass(0)) ?
    sqrt(0.57 * pow(2.0, ((RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8)) / 3.0)))
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
    Int                 numModesForFullRD = g_aucIntraModeNumFast[ uiWidthBit ];
    const TComRectangle &puRect           = tuRecurseWithPU.getRect(COMPONENT_Y);
    const UInt          uiPartOffset      = tuRecurseWithPU.GetAbsPartIdxTU();
    const UInt          uiLog2TrSize      = tuRecurseWithPU.GetLog2LumaTrSize();
    UInt                CandNum;

    Pel*                piOrg             = pcOrgYuv ->getAddr( COMPONENT_Y, uiPartOffset );
    Pel*                piPred            = pcPredYuv->getAddr( COMPONENT_Y, uiPartOffset );
    UInt                uiStride          = pcPredYuv->getStride( COMPONENT_Y );

    Bool                bAboveAvail       = false;
    Bool                bLeftAvail        = false;
    DEBUG_STRING_NEW(sTemp2)

    for ( Int i=0; i < numModesForFullRD; i++ )
    {
      CandCostList[i] = MAX_DOUBLE;
    }
    CandNum = 0;

    initAdiPatternChType( tuRecurseWithPU, bAboveAvail, bLeftAvail, COMPONENT_Y, true DEBUG_STRING_PASS_INTO(sTemp2) );

    if(bReuse)
    {
      uiRdModeList [0] =tuRecurseWithPU.getCU()->getIntraDir( CHANNEL_TYPE_LUMA, tuRecurseWithPU.GetAbsPartIdxTU() );
      numModesForFullRD = 1;
    }
    else
    {

      for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
      {
        UInt       uiMode = modeIdx;
        Distortion uiSad  = 0;

        const Bool bUseFilter = TComPrediction::filteringIntraReferenceSamples( COMPONENT_Y, uiMode, puRect.width, puRect.height, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing() ); 
        predIntraAng( COMPONENT_Y, uiMode, piOrg, uiStride, piPred, uiStride, tuRecurseWithPU, bAboveAvail, bLeftAvail, bUseFilter, TComPrediction::UseDPCMForFirstPassIntraEstimation(tuRecurseWithPU, uiMode) );

        //hadamard transform
        uiSad += m_pcRdCost->calcHAD( g_bitDepth[toChannelType(COMPONENT_Y)], piOrg, uiStride, piPred, uiStride, puRect.width, puRect.height );

        UInt iModeBits = 0;
        iModeBits     += xModeBitsIntra( pcCU, uiMode, uiPartOffset, uiDepth, uiInitTrDepth, CHANNEL_TYPE_LUMA );
        Double cost    = (Double)uiSad + (Double)iModeBits * sqrtLambdaForFirstPass;

        CandNum += xUpdateCandList( uiMode, cost, numModesForFullRD, uiRdModeList, CandCostList );
      }

      Int uiPreds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};
      Int iMode                            = -1;
      Int numCand                          = pcCU->getIntraDirPredictor( uiPartOffset, uiPreds, COMPONENT_Y, &iMode );

      if ( iMode >= 0 )
      {
        numCand = iMode;
      }

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

      DEBUG_STRING_NEW(sMode)
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

      Distortion uiPUDistY = 0;
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0;

      xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, true DEBUG_STRING_PASS_INTO(sMode)  );

      if( dPUCost < dBestPUCost)
      {
        dBestPUCost                       = dPUCost;
        uiBestPUDist[CHANNEL_TYPE_LUMA]   = uiPUDistY;
        uiBestPUDist[CHANNEL_TYPE_CHROMA] = uiPUDistC;
        uiBestPUMode[CHANNEL_TYPE_LUMA]   = uiRdModeList[uiLumaModeIdx];
        uiBestPUMode[CHANNEL_TYPE_CHROMA] = DM_CHROMA_IDX;
        xSetIntraResultQT( false, pcRecoYuv, tuRecurseWithPU );

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx() + uiPartOffset, uiQPartNum * sizeof( UChar ) );
        for (UInt component = 0; component < numberValidComponents; component++)
        {
          const ComponentID compID = ComponentID(component);
          ::memcpy( m_puhQTTempCbf[compID],                             pcCU->getCbf( compID  )                           + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_puhQTTempTransformSkipFlag[compID],               pcCU->getTransformSkip(compID)                    + uiPartOffset, uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID],    pcCU->getCrossComponentPredictionAlpha(compID)    + uiPartOffset, uiQPartNum * sizeof( Char  ) );
        }
      }
    }

    if( uiLog2TrSize  >  pcCU->getQuadtreeTULog2MinSizeInCU(uiPartOffset) )
    {
      Distortion uiPUDistY = 0;
      Distortion uiPUDistC = 0;
      Double     dPUCost   = 0;

      pcCU->setIntraDirSubParts( CHANNEL_TYPE_LUMA,   uiBestPUMode[CHANNEL_TYPE_LUMA],   uiPartOffset, uiDepth + uiInitTrDepth );
      pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiBestPUMode[CHANNEL_TYPE_CHROMA], uiPartOffset, uiDepth + uiInitTrDepth );

      DEBUG_STRING_NEW(sMode)
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );
      xRecurIntraCodingQTCSC( pcOrgYuv, pcPredYuv, pcResiYuv, uiPUDistY, uiPUDistC, dPUCost, tuRecurseWithPU, false DEBUG_STRING_PASS_INTO(sMode) );

      if( dPUCost < dBestPUCost)
      {
        dBestPUCost                       = dPUCost;
        uiBestPUDist[CHANNEL_TYPE_LUMA]   = uiPUDistY;
        uiBestPUDist[CHANNEL_TYPE_CHROMA] = uiPUDistC;
        xSetIntraResultQT( false, pcRecoYuv, tuRecurseWithPU );

        UInt uiQPartNum = tuRecurseWithPU.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx,  pcCU->getTransformIdx() + uiPartOffset, uiQPartNum * sizeof( UChar ) );
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

        const UInt  uiZOrder      = pcCU->getZorderIdxInCU() + uiPartOffset;
        Pel*  piDes               = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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
TEncSearch::estIntraPredQTWithModeReuse(TComDataCU* pcCU,
                                        TComYuv*    pcOrgYuv,
                                        TComYuv*    pcPredYuv,
                                        TComYuv*    pcResiYuv,
                                        TComYuv*    pcRecoYuv,
                                        Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                        Distortion& ruiDistC,
                                        Bool        bLumaOnly
                                       )
{
  assert(bLumaOnly);
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
    Distortion uiBestPUDistC = 0;
    Double     dBestPUCost  = 0;

    DEBUG_STRING_NEW(sMode)
    // set context models
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST] );

    xRecurIntraCodingQT( bLumaOnly, pcOrgYuv, pcPredYuv, pcResiYuv, resiLumaPU, uiBestPUDistY, uiBestPUDistC, true, dBestPUCost, tuRecurseWithPU DEBUG_STRING_PASS_INTO(sMode) );
    xSetIntraResultQT( bLumaOnly, pcRecoYuv, tuRecurseWithPU );

    if (pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction())
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
    uiOverallDistC += uiBestPUDistC;

    //--- set reconstruction for next intra prediction blocks ---
    if( !tuRecurseWithPU.IsLastSection() )
    {
      const Bool bSkipChroma  = tuRecurseWithPU.ProcessChannelSection(CHANNEL_TYPE_CHROMA);

      const UInt numChannelToProcess = (bLumaOnly || bSkipChroma) ? 1 : getNumberValidComponents(pcCU->getPic()->getChromaFormat());

      for (UInt ch=0; ch<numChannelToProcess; ch++)
      {
        const ComponentID compID    = ComponentID(ch);
        const TComRectangle &puRect = tuRecurseWithPU.getRect(compID);
        const UInt  uiCompWidth     = puRect.width;
        const UInt  uiCompHeight    = puRect.height;

        const UInt  uiZOrder      = pcCU->getZorderIdxInCU() + uiPartOffset;
        Pel*        piDes         = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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
  ruiDistC                   = uiOverallDistC;
  pcCU->getTotalDistortion() = uiOverallDistY + uiOverallDistC;
}

Void
TEncSearch::estIntraPredChromaQTWithModeReuse(TComDataCU* pcCU,
                                              TComYuv*    pcOrgYuv,
                                              TComYuv*    pcPredYuv,
                                              TComYuv*    pcResiYuv,
                                              TComYuv*    pcRecoYuv,
                                              Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                              Distortion  uiPreCalcDistC
                                             )
{
  assert( uiPreCalcDistC == 0 );
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
          const UInt  uiZOrder        = pcCU->getZorderIdxInCU() + tuRecurseWithPU.GetAbsPartIdxTU();
          Pel*  piDes                 = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getAddr(), uiZOrder );
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
#endif
/** Function for encoding and reconstructing luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param piOrg pointer to original sample arrays
 * \param piPCM pointer to PCM code arrays
 * \param piPred pointer to prediction signal arrays
 * \param piResi pointer to residual signal arrays
 * \param piReco pointer to reconstructed sample arrays
 * \param uiStride stride of the original/prediction/residual sample arrays
 * \param uiWidth block width
 * \param uiHeight block height
 * \param ttText texture component type
 * \returns Void
 */
Void TEncSearch::xEncPCM (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* pOrg, Pel* pPCM, Pel* pPred, Pel* pResi, Pel* pReco, UInt uiStride, UInt uiWidth, UInt uiHeight, const ComponentID compID )
{
  const UInt uiReconStride = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  const UInt uiPCMBitDepth = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
  Pel* pRecoPic = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getAddr(), pcCU->getZorderIdxInCU()+uiAbsPartIdx);

  const Int pcmShiftRight=(g_bitDepth[toChannelType(compID)] - Int(uiPCMBitDepth));

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
#if SCM__R0348_PALETTE_MODE
Void TEncSearch::PLTSearch(TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv *& rpcResiBestYuv,
                           TComYuv*& rpcRecoYuv, Bool bCheckPLTSharingMode)
{
  UInt  uiDepth      = pcCU->getDepth(0);
  UInt  uiDistortion = 0;
  Pel *paOrig[3], *paPalette[3];
  TCoeff *pRun;
  UChar *paSPoint[3];
  Pel *pPixelValue[3];
  for (UInt ch = 0; ch < 3; ch++)
  {
    paOrig[ch] = pcOrgYuv->getAddr((ComponentID)ch, 0);
    paPalette[ch] = pcCU->getPLT(ch, 0);
  }

  pRun = pcCU->getRun(COMPONENT_Y);
  paSPoint[0] = pcCU->getSPoint(COMPONENT_Y);
  Int iPLTErrLimit = g_uhPLTQuant[Int(pcCU->getQP(0))];
  setPLTErrLimit(iPLTErrLimit);

  UInt uiPLTSize = 1;
  Bool bPLTSharingModeAvailable = bCheckPLTSharingMode;
  Pel* pPalettePrev[3]          = {NULL, NULL, NULL};
  UInt uiPLTSizePrev            = 1;
  UInt uiPLTUsedSizePrev        = 1;
  if(bPLTSharingModeAvailable)
  {
    for (UInt comp=0; comp < MAX_NUM_COMPONENT; comp++)
    {
      pPalettePrev[comp] = pcCU->getPLTPred (pcCU,  pcCU->getZorderIdxInCU(), comp, uiPLTSizePrev, uiPLTUsedSizePrev);
      assert( pPalettePrev[comp] && !((comp == 0) && (uiPLTSizePrev == 0)));
    }
  }

  pcCU->getPLTPred (pcCU,  pcCU->getZorderIdxInCU(), COMPONENT_Y, uiPLTSizePrev, uiPLTUsedSizePrev);
  pcCU->setPLTSharingFlagSubParts(bPLTSharingModeAvailable, 0, pcCU->getDepth(0));

  if(bPLTSharingModeAvailable)
  {
    uiPLTSize = uiPLTUsedSizePrev;
    for (UInt comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      for (UInt i = 0; i < uiPLTSize; i++)
      {
        paPalette[comp][i] = pPalettePrev[comp][i];
      }
    }
    for(UInt uiIdxPrev = 0; uiIdxPrev < MAX_PLT_PRED_SIZE; uiIdxPrev++)
    {
      if(uiIdxPrev < uiPLTSize)
      {
        for (UInt comp = 0; comp < MAX_NUM_COMPONENT; comp++)
        {
          pcCU->setPrevPLTReusedFlagSubParts(comp, 1, uiIdxPrev, 0, pcCU->getDepth(0));
        }
      }
      else
      {
        for (UInt comp = 0; comp < MAX_NUM_COMPONENT; comp++)
        {
          pcCU->setPrevPLTReusedFlagSubParts(comp, 0, uiIdxPrev, 0, pcCU->getDepth(0));
        }
      }
    }
    pcCU->setPLTSizeSubParts(0, uiPLTSize, 0, pcCU->getDepth(0));
  }
  else
  {
    Bool bLossless = pcCU->getCUTransquantBypass(0);
    if (bLossless)
    {
      derivePLTLossless(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize);
    }
    else
    {
      derivePLTLossy(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), pcCU->getHeight(0), uiPLTSize, m_pcRdCost);
    }
    pcCU->setPLTSizeSubParts(0, uiPLTSize,0, pcCU->getDepth(0));
    reorderPLT (pcCU, paPalette, 3);
  }

  preCalcPLTIndex(pcCU, paPalette, paOrig, pcCU->getWidth(0), pcCU->getHeight(0), uiPLTSize);

  UInt uiPLTIdx = 0;
  for (UInt ch = 0; ch < 3; ch++)
  {
    for ( uiPLTIdx = 0; uiPLTIdx < MAX_PLT_SIZE; uiPLTIdx++)
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
  deriveRunAndCalcBits(pcCU, pcOrgYuv, rpcRecoYuv, uiBits, false,  PLT_SCAN_VERTRAV);

  pcCU->setPLTScanRotationModeFlagSubParts( m_bBestScanRotationMode, 0, uiDepth );
  for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    memcpy(pPixelValue[ch],  m_paBestLevel[ch],  sizeof(Pel) * uiWidth * uiHeight);
  }
  memcpy(paSPoint[0], m_paBestSPoint, sizeof(UChar) * uiWidth * uiHeight);
  memcpy(pRun,        m_paBestRun,    sizeof(TCoeff) * uiWidth * uiHeight);
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
    UChar *pSPoint = pcCU->getSPoint  (COMPONENT_Y);
    Pel *pReco    = rpcRecoYuv->getAddr(compID, 0);
    Pel *pRecoPic = pcCU->getPic()->getPicYuvRec()->getAddr(ComponentID(ch), pcCU->getAddr(), pcCU->getZorderIdxInCU());
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
          if (pSPoint[uiIdx] != PLT_ESCAPE)
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
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          uiIdx = (uiY << pcCU->getPic()->getComponentScaleY(compID)) * (uiWidth << pcCU->getPic()->getComponentScaleX(compID))
                  + (uiX << pcCU->getPic()->getComponentScaleX(compID));
          UInt uiPxlPos = uiX*uiStride+uiY;
          if (PLT_ESCAPE != pSPoint[uiIdx])
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
    uiDistortion += m_pcRdCost->getDistPart( g_bitDepth[chType], pReco, uiStride, pOrig, uiStride, uiWidth, uiHeight, compID );
  }

  Double dCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
  pcCU->getTotalBits()       = uiBits;
  pcCU->getTotalCost()       = dCost;
  pcCU->getTotalDistortion() = uiDistortion;
  pcCU->copyToPic(uiDepth, 0, 0);
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
  UInt uiPLTSize = pcCU->getPLTSize(0, 0);

  paLevel[0] = pcCU->getLevel(COMPONENT_Y);
  pRun = pcCU->getRun(COMPONENT_Y);
  paSPoint[0] = pcCU->getSPoint(COMPONENT_Y);

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
  }

  m_puiScanOrder = g_scanOrder[SCAN_UNGROUPED][SCAN_TRAV][g_aucConvertToBit[uiWidth]+2][g_aucConvertToBit[uiHeight]+2];;
  deriveRun(pcCU, paOrig, paPalette,  paLevel[0], paSPoint[0], pPixelValue, pRecoValue, pRun, uiWidth, uiHeight, pcOrgYuv->getStride(ComponentID(0)), uiPLTSize);

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiDepth, 0, true, false);
  UInt uiTempBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  if (uiMinBits > uiTempBits)
  {
    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
    m_bBestScanRotationMode = pltScanMode;

    for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      memcpy(m_paBestLevel[ch], pPixelValue[ch], sizeof(Pel) * uiTotalPixel);
    }
    memcpy(m_paBestSPoint, paSPoint[0], sizeof(UChar) * uiTotalPixel);
    memcpy(m_paBestRun, pRun, sizeof(TCoeff) * uiTotalPixel);
    
    uiMinBits = uiTempBits;
  }
  if (bReset)
  {
    memcpy(m_cIndexBlock, m_paOriginalLevel, sizeof(Pel) * uiTotalPixel);
    memset(paSPoint[0], 0, sizeof(UChar) * uiTotalPixel);
  }
}
#endif


/**  Function for PCM mode estimation.
 * \param pcCU
 * \param pcOrgYuv
 * \param rpcPredYuv
 * \param rpcResiYuv
 * \param rpcRecoYuv
 * \returns Void
 */
Void TEncSearch::IPCMSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv )
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
    const UInt stride = rpcPredYuv->getStride(compID);

    Pel * pOrig    = pcOrgYuv->getAddr  (compID, 0, width);
    Pel * pResi    = rpcResiYuv->getAddr(compID, 0, width);
    Pel * pPred    = rpcPredYuv->getAddr(compID, 0, width);
    Pel * pReco    = rpcRecoYuv->getAddr(compID, 0, width);
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

  pcCU->copyToPic(uiDepth, 0, 0);
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


  m_pcRdCost->setDistParam( cDistParam, g_bitDepth[CHANNEL_TYPE_LUMA],
                            pcYuvOrg->getAddr( COMPONENT_Y, uiAbsPartIdx ), pcYuvOrg->getStride(COMPONENT_Y),
                            m_tmpYuvPred .getAddr( COMPONENT_Y, uiAbsPartIdx ), m_tmpYuvPred.getStride(COMPONENT_Y),
                            iWidth, iHeight, m_pcEncCfg->getUseHADME() && (pcCU->getCUTransquantBypass(uiAbsPartIdx) == 0) );

  ruiErr = cDistParam.DistFunc( &cDistParam );
}

/** estimation of best merge coding
 * \param pcCU
 * \param pcYuvOrg
 * \param iPUIdx
 * \param uiInterDir
 * \param pacMvField
 * \param uiMergeIndex
 * \param ruiCost
 * \param ruiBits
 * \param puhNeighCands
 * \param bValid
 * \returns Void
 */
Void TEncSearch::xMergeEstimation( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPUIdx, UInt& uiInterDir, TComMvField* pacMvField, UInt& uiMergeIndex, Distortion& ruiCost, TComMvField* cMvFieldNeighbours, UChar* uhInterDirNeighbours, Int& numValidMergeCand )
{
  UInt uiAbsPartIdx = 0;
  Int iWidth = 0;
  Int iHeight = 0;

  pcCU->getPartIndexAndSize( iPUIdx, uiAbsPartIdx, iWidth, iHeight );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );

  PartSize partSize = pcCU->getPartitionSize( 0 );
  if ( pcCU->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() && partSize != SIZE_2Nx2N && pcCU->getWidth( 0 ) <= 8 )
  {
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
    if ( iPUIdx == 0 )
    {
      pcCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
    }
    pcCU->setPartSizeSubParts( partSize, 0, uiDepth );
  }
  else
  {
    pcCU->getInterMergeCandidates( uiAbsPartIdx, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );
  }

  xRestrictBipredMergeCand( pcCU, iPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand );

  ruiCost = std::numeric_limits<Distortion>::max();
  for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
  {
    Distortion uiCostCand = std::numeric_limits<Distortion>::max();
    UInt       uiBitsCand = 0;

    PartSize ePartSize = pcCU->getPartitionSize( 0 );

    pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );
    pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], ePartSize, uiAbsPartIdx, 0, iPUIdx );

    xGetInterPredictionError( pcCU, pcYuvOrg, iPUIdx, uiCostCand, m_pcEncCfg->getUseHADME() );
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

/** search of the best candidate for inter prediction
 * \param pcCU
 * \param pcOrgYuv
 * \param rpcPredYuv
 * \param rpcResiYuv
 * \param rpcRecoYuv
 * \param bUseRes
 * \returns Void
 */
#if AMP_MRG
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseRes, Bool bUseMRG )
#else
Void TEncSearch::predInterSearch( TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv*& rpcPredYuv, TComYuv*& rpcResiYuv, TComYuv*& rpcRecoYuv, Bool bUseRes )
#endif
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].clear();
  }
  m_cYuvPredTemp.clear();
  rpcPredYuv->clear();

  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }

  rpcRecoYuv->clear();

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

#if ZERO_MVD_EST
  Int          aiZeroMvdMvpIdx[2] = {-1, -1};
  Int          aiZeroMvdRefIdx[2] = {0, 0};
  Int          iZeroMvdDir = -1;
#endif

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
#if ZERO_MVD_EST
    Distortion   uiZeroMvdCost = std::numeric_limits<Distortion>::max();
    Distortion   uiZeroMvdCostTemp;
    UInt         uiZeroMvdBitsTemp;
    Distortion   uiZeroMvdDistTemp = std::numeric_limits<Distortion>::max();
    UInt         auiZeroMvdBits[3];
#endif
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

      for ( Int iRefIdxTemp = 0; iRefIdxTemp < pcCU->getSlice()->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
      {
        uiBitsTemp = uiMbBits[iRefList];
        if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
        }
#if ZERO_MVD_EST
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp, &uiZeroMvdDistTemp);
#else
        xEstimateMvPredAMVP( pcCU, pcOrgYuv, iPartIdx, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], false, &biPDistTemp);
#endif
        aaiMvpIdx[iRefList][iRefIdxTemp] = pcCU->getMVPIdx(eRefPicList, uiPartAddr);
        aaiMvpNum[iRefList][iRefIdxTemp] = pcCU->getMVPNum(eRefPicList, uiPartAddr);

        if(pcCU->getSlice()->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
        {
          bestBiPDist = biPDistTemp;
          bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
          bestBiPRefIdxL1 = iRefIdxTemp;
        }

        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
#if ZERO_MVD_EST
        if ( iRefList == 0 || pcCU->getSlice()->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
        {
          uiZeroMvdBitsTemp = uiBitsTemp;
          uiZeroMvdBitsTemp += 2; //zero mvd bits

          m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

          uiZeroMvdCostTemp = uiZeroMvdDistTemp + m_pcRdCost->getCost(uiZeroMvdBitsTemp);

          if (uiZeroMvdCostTemp < uiZeroMvdCost)
          {
            uiZeroMvdCost = uiZeroMvdCostTemp;
            iZeroMvdDir = iRefList + 1;
            aiZeroMvdRefIdx[iRefList] = iRefIdxTemp;
            aiZeroMvdMvpIdx[iRefList] = aaiMvpIdx[iRefList][iRefIdxTemp];
            auiZeroMvdBits[iRefList] = uiZeroMvdBitsTemp;
          }
        }
#endif

#if GPB_SIMPLE_UNI
        if ( iRefList == 1 )    // list 1
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
#else
        xMotionEstimation ( pcCU, pcOrgYuv, iPartIdx, eRefPicList, &cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );
#endif
        xCopyAMVPInfo(pcCU->getCUMvField(eRefPicList)->getAMVPInfo(), &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
        xCheckBestMVP(pcCU, eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

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
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
        TComYuv* pcYuvPred = &m_acYuvPred[REF_PIC_LIST_1];
        motionCompensation( pcCU, pcYuvPred, REF_PIC_LIST_1, iPartIdx );

        uiMotBits[0] = uiBits[0] - uiMbBits[0];
        uiMotBits[1] = uiMbBits[1];

        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiMotBits[1] += bestBiPRefIdxL1+1;
          if ( bestBiPRefIdxL1 == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 ) uiMotBits[1]--;
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

        for ( Int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
        {
          uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
          if ( pcCU->getSlice()->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == pcCU->getSlice()->getNumRefIdx(eRefPicList)-1 ) uiBitsTemp--;
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
#if ZERO_MVD_EST
    if ( (pcCU->getSlice()->isInterB()) && (pcCU->isBipredRestriction(iPartIdx) == false) )
    {
      m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

      for ( Int iL0RefIdxTemp = 0; iL0RefIdxTemp <= pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0)-1; iL0RefIdxTemp++ )
      for ( Int iL1RefIdxTemp = 0; iL1RefIdxTemp <= pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1; iL1RefIdxTemp++ )
      {
        UInt uiRefIdxBitsTemp = 0;
        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0) > 1 )
        {
          uiRefIdxBitsTemp += iL0RefIdxTemp+1;
          if ( iL0RefIdxTemp == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0)-1 ) uiRefIdxBitsTemp--;
        }
        if ( pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1) > 1 )
        {
          uiRefIdxBitsTemp += iL1RefIdxTemp+1;
          if ( iL1RefIdxTemp == pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_1)-1 ) uiRefIdxBitsTemp--;
        }

        Int iL0MVPIdx = 0;
        Int iL1MVPIdx = 0;

        for (iL0MVPIdx = 0; iL0MVPIdx < aaiMvpNum[0][iL0RefIdxTemp]; iL0MVPIdx++)
        {
          for (iL1MVPIdx = 0; iL1MVPIdx < aaiMvpNum[1][iL1RefIdxTemp]; iL1MVPIdx++)
          {
            uiZeroMvdBitsTemp = uiRefIdxBitsTemp;
            uiZeroMvdBitsTemp += uiMbBits[2];
            uiZeroMvdBitsTemp += m_auiMVPIdxCost[iL0MVPIdx][aaiMvpNum[0][iL0RefIdxTemp]] + m_auiMVPIdxCost[iL1MVPIdx][aaiMvpNum[1][iL1RefIdxTemp]];
            uiZeroMvdBitsTemp += 4; //zero mvd for both directions
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( aacAMVPInfo[0][iL0RefIdxTemp].m_acMvCand[iL0MVPIdx], iL0RefIdxTemp, ePartSize, uiPartAddr, iPartIdx, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( aacAMVPInfo[1][iL1RefIdxTemp].m_acMvCand[iL1MVPIdx], iL1RefIdxTemp, ePartSize, uiPartAddr, iPartIdx, 0 );

            xGetInterPredictionError( pcCU, pcOrgYuv, iPartIdx, uiZeroMvdDistTemp, m_pcEncCfg->getUseHADME() );
            uiZeroMvdCostTemp = uiZeroMvdDistTemp + m_pcRdCost->getCost( uiZeroMvdBitsTemp );
            if (uiZeroMvdCostTemp < uiZeroMvdCost)
            {
              uiZeroMvdCost = uiZeroMvdCostTemp;
              iZeroMvdDir = 3;
              aiZeroMvdMvpIdx[0] = iL0MVPIdx;
              aiZeroMvdMvpIdx[1] = iL1MVPIdx;
              aiZeroMvdRefIdx[0] = iL0RefIdxTemp;
              aiZeroMvdRefIdx[1] = iL1RefIdxTemp;
              auiZeroMvdBits[2] = uiZeroMvdBitsTemp;
            }
          }
        }
      }
    }
#endif

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
#if ZERO_MVD_EST
    if (uiZeroMvdCost <= uiCostBi && uiZeroMvdCost <= uiCost[0] && uiZeroMvdCost <= uiCost[1])
    {
      if (iZeroMvdDir == 3)
      {
        uiLastMode = 2;

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( aacAMVPInfo[0][aiZeroMvdRefIdx[0]].m_acMvCand[aiZeroMvdMvpIdx[0]], aiZeroMvdRefIdx[0], ePartSize, uiPartAddr, iPartIdx, 0 );
        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( aacAMVPInfo[1][aiZeroMvdRefIdx[1]].m_acMvCand[aiZeroMvdMvpIdx[1]], aiZeroMvdRefIdx[1], ePartSize, uiPartAddr, iPartIdx, 0 );

        pcCU->setInterDirSubParts( 3, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[0], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[0][aiZeroMvdRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[1], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[1][aiZeroMvdRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[2];
      }
      else if (iZeroMvdDir == 1)
      {
        uiLastMode = 0;

        pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvField( aacAMVPInfo[0][aiZeroMvdRefIdx[0]].m_acMvCand[aiZeroMvdMvpIdx[0]], aiZeroMvdRefIdx[0], ePartSize, uiPartAddr, iPartIdx, 0 );

        pcCU->setInterDirSubParts( 1, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[0], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[0][aiZeroMvdRefIdx[0]], REF_PIC_LIST_0, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[0];
      }
      else if (iZeroMvdDir == 2)
      {
        uiLastMode = 1;

        pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMvField( aacAMVPInfo[1][aiZeroMvdRefIdx[1]].m_acMvCand[aiZeroMvdMvpIdx[1]], aiZeroMvdRefIdx[1], ePartSize, uiPartAddr, iPartIdx, 0 );

        pcCU->setInterDirSubParts( 2, uiPartAddr, iPartIdx, pcCU->getDepth(0) );

        pcCU->setMVPIdxSubParts( aiZeroMvdMvpIdx[1], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        pcCU->setMVPNumSubParts( aaiMvpNum[1][aiZeroMvdRefIdx[1]], REF_PIC_LIST_1, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
        uiMEBits = auiZeroMvdBits[1];
      }
      else
      {
        assert(0);
      }
    }
    else
#endif
    if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
    {
      uiLastMode = 2;
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMv( cMvBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllRefIdx( iRefIdxBi[0], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllMv( cMvBi[1], ePartSize, uiPartAddr, 0, iPartIdx );
      pcCU->getCUMvField(REF_PIC_LIST_1)->setAllRefIdx( iRefIdxBi[1], ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
      pcCU->getCUMvField(REF_PIC_LIST_0)->setAllMvd    ( TempMv,                 ePartSize, uiPartAddr, 0, iPartIdx );

      TempMv = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
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

      TempMv = cMv[0] - cMvPred[0][iRefIdx[0]];
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

      TempMv = cMv[1] - cMvPred[1][iRefIdx[1]];
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
    motionCompensation ( pcCU, rpcPredYuv, REF_PIC_LIST_X, iPartIdx );

  } //  end of for ( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx++ )

  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

  return;
}


// based on predInterSearch()
Bool TEncSearch::predIntraBCSearch( TComDataCU * pcCU,
                                    TComYuv    * pcOrgYuv,
                                    TComYuv    *&rpcPredYuv,
                                    TComYuv    *&rpcResiYuv,
                                    TComYuv    *&rpcRecoYuv
                                    DEBUG_STRING_FN_DECLARE(sDebug),
                                    Bool         bUse1DSearchFor8x8,
                                    Bool         bUseRes )
{
  rpcPredYuv->clear();
  if ( !bUseRes )
  {
    rpcResiYuv->clear();
  }
  rpcRecoYuv->clear();

  PartSize     ePartSize  = pcCU->getPartitionSize( 0 );

  if (m_pcEncCfg->getUseIntraBlockCopyFastSearch() && (pcCU->getWidth(0) > 16))
    return false;

  const Int iNumPart = pcCU->getNumPartitions();
  for( Int iPartIdx = 0; iPartIdx < iNumPart; ++iPartIdx )
  {
    Int iDummyWidth, iDummyHeight;
    UInt uiPartAddr = 0;
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iDummyWidth, iDummyHeight );

    TComMvField cMEMvField;
#if SCM__R0309_INTRABC_BVP
    TComMv    cMv, cMvd, cMvPred[2], MvLast[2];

    for(Int idx = 0; idx < 2; idx++)
    {
      MvLast[idx] = pcCU->getLastIntraBCMv(idx);
    }

    for(Int tmpPartIdx=1; tmpPartIdx<=iPartIdx; tmpPartIdx++)
    {
      TComMv tmpMv = pcCU->getCUMvField(REF_PIC_LIST_INTRABC)->getMv( uiPartAddr - iPartIdx + tmpPartIdx - 1 );
      if( tmpMv != MvLast[0])
      {
        MvLast[1] = MvLast[0];
        MvLast[0] = tmpMv;
      }
    }
    pcCU->getIntraBCMVPs(uiPartAddr, cMvPred, MvLast);
#else
    TComMv      cMv, cMvd, cMvPred;
#endif 
    Distortion  uiCost;
#if !SCM__R0081_CODE_SIMPLIFICATION
    UInt        uiBits = 0;
#endif    

#if SCM__R0081_CODE_SIMPLIFICATION
#if SCM__R0309_INTRABC_BVP
    xIntraBlockCopyEstimation ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, uiCost, bUse1DSearchFor8x8 );
#else
    xIntraBlockCopyEstimation ( pcCU, pcOrgYuv, iPartIdx, &cMvPred, cMv, uiCost, bUse1DSearchFor8x8 );
#endif
#else
#if SCM__R0309_INTRABC_BVP
    xIntraBlockCopyEstimation ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, uiBits, uiCost, bUse1DSearchFor8x8 );
#else
    xIntraBlockCopyEstimation ( pcCU, pcOrgYuv, iPartIdx, &cMvPred, cMv, uiBits, uiCost, bUse1DSearchFor8x8 );
#endif
#endif

#if SCM__FLEXIBLE_INTRABC_SEARCH
    if( m_pcEncCfg->getUseHashBasedIntraBCSearch()
      && pcCU->getWidth(0) == 8
      && ePartSize == SIZE_2Nx2N
      && (m_pcEncCfg->getUseHashBasedIntraBCSearch()
        || pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCSearchWidthInCTUs() ) != pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() ))
      )
#else
    if((pcCU->getWidth(0) == 8) && (ePartSize == SIZE_2Nx2N) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
#endif
    {
#if SCM__R0081_CODE_SIMPLIFICATION
      UInt uiIntraBCECost = uiCost;
#else
#if SCM__R0309_INTRABC_BVP
      UInt uiIntraBCECost = uiCost + m_pcRdCost->getCostMultiplePreds( cMv.getHor(), cMv.getVer());
#else
      UInt uiIntraBCECost = uiCost + m_pcRdCost->getCost( cMv.getHor(), cMv.getVer());
#endif
#endif
#if SCM__R0081_CODE_SIMPLIFICATION
#if SCM__R0309_INTRABC_BVP
      xIntraBCHashSearch ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, uiIntraBCECost);
#else
      xIntraBCHashSearch ( pcCU, pcOrgYuv, iPartIdx, &cMvPred, cMv, uiIntraBCECost);
#endif
#else
#if SCM__R0309_INTRABC_BVP
      xIntraBCHashSearch ( pcCU, pcOrgYuv, iPartIdx, cMvPred, cMv, uiBits, uiCost, uiIntraBCECost);
#else
      xIntraBCHashSearch ( pcCU, pcOrgYuv, iPartIdx, &cMvPred, cMv, uiBits, uiCost, uiIntraBCECost);
#endif
#endif
    }

    // store intra BV in REF_PIC_LIST_0
    cMEMvField.setMvField( cMv, REF_PIC_LIST_INTRABC);
    pcCU->getCUMvField( REF_PIC_LIST_INTRABC )->setAllMvField( cMEMvField, ePartSize, uiPartAddr, 0, iPartIdx );

#if SCM__R0309_INTRABC_BVP
  UInt uiMVPIdx = pcCU->getMVPIdx(REF_PIC_LIST_INTRABC, uiPartAddr);
  cMvd.setHor(cMv.getHor() - cMvPred[uiMVPIdx].getHor());
  cMvd.setVer(cMv.getVer() - cMvPred[uiMVPIdx].getVer());
#else
    cMvd.setHor(cMv.getHor() - cMvPred.getHor());
    cMvd.setVer(cMv.getVer() - cMvPred.getVer());
#endif

    pcCU->getCUMvField(REF_PIC_LIST_INTRABC )->setAllMvd(cMvd, ePartSize, uiPartAddr, 0, iPartIdx);

    // no valid intra BV
    if (cMv.getHor() == 0 && cMv.getVer() == 0)
    {
      return false;
    }
  }

  // motion compensation
#if SCM__R0147_RGB_YUV_RD_ENC
  if( !pcCU->getSlice()->getSPS()->getUseColorTrans() || !m_pcEncCfg->getRGBFormatFlag() )
  {
#endif
  for( Int iPartIdx = 0; iPartIdx < iNumPart; iPartIdx ++ )
  {
    intraBlockCopy ( pcCU, rpcPredYuv, iPartIdx );
  }
#if SCM__R0147_RGB_YUV_RD_ENC
  }
#endif
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

Distortion TEncSearch::getSAD( Pel* pRef, Int refStride, Pel* pCurr, Int currStride, Int width, Int height )
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

  if ( g_bitDepth[CHANNEL_TYPE_LUMA] == 8 )
  {
    return dist;
  }
  else
  {
    Int shift = DISTORTION_PRECISION_ADJUSTMENT( g_bitDepth[CHANNEL_TYPE_LUMA] - 8 );
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
    Int currCost = m_pcRdCost->xGetComponentBits( (*it).x - currBlockHash.x ) +
                   m_pcRdCost->xGetComponentBits( (*it).y - currBlockHash.y );

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

  if ( !TComHash::getBlockHashValue( pcPic->getPicYuvOrg(), width, height, xPos, yPos, hashValue1, hashValue2 ) )
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

  if ( !TComHash::getBlockHashValue( pcPic->getPicYuvOrg(), width, height, xPos, yPos, hashValue1, hashValue2 ) )
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
    for ( Int iRefIdx = 0; iRefIdx < pcCU->getSlice()->getNumRefIdx( eRefPicList ); iRefIdx++ )
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
          Distortion currSad = getSAD( pRef, refStride, pCurr, currStride, width, height );
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
            bestCost = currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex = iRefIdx;
            bestMv.set( (*it).x - currBlockHash.x, (*it).y - currBlockHash.y );
#if SCM__R0102_HASH_ME_FIX
            bestMv <<= 2;
            bestMvd.set( bestMv.getHor() - currAMVPInfo.m_acMvCand[currMVPIdx].getHor(), bestMv.getVer() - currAMVPInfo.m_acMvCand[currMVPIdx].getVer() );
#else
            bestMvd.set( (*it).x - currBlockHash.x - currAMVPInfo.m_acMvCand[currMVPIdx].getHor(), (*it).y - currBlockHash.y - currAMVPInfo.m_acMvCand[currMVPIdx].getVer() );
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
#if !SCM__R0081_CODE_SIMPLIFICATION
                                            UInt       &ruiBits,
#endif 
                                            Distortion &ruiCost,
                                            Bool        bUse1DSearchFor8x8
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

#if !SCM__R0081_CODE_SIMPLIFICATION
  Double        fWeight       = 1.0;
#endif

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight );

  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
                             iRoiWidth,
                             iRoiHeight,
                             pcYuv->getStride(COMPONENT_Y) );

  Pel*        piRefY      = pcCU->getPic()->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
  Int         iRefStride  = pcCU->getPic()->getPicYuvRec()->getStride(COMPONENT_Y);

  TComMv      cMvPred = *pcMvPred;

  // assume that intra BV is integer-pel precision
  xSetIntraSearchRange   ( pcCU, cMvPred, uiPartAddr, iRoiWidth, iRoiHeight, cMvSrchRngLT, cMvSrchRngRB );

  // disable weighted prediction
  setWpScalingDistParam( pcCU, -1, REF_PIC_LIST_X );

#if !SCM__R0309_INTRABC_BVP
  TComMv mvPred = ( iPartIdx == 0 ? pcCU->getLastIntraBCMv() : pcCU->getCUMvField(REF_PIC_LIST_INTRABC)->getMv( uiPartAddr - (pcCU->getTotalNumPart() >> 2) ) );

  if (mvPred.getHor()==0 && mvPred.getVer()==0)
  {
    mvPred.setHor(-Short(pcCU->getWidth(iPartIdx)));
  }
#endif

#if !SCM__R0081_CODE_SIMPLIFICATION
  TComMv        ZeroMv(0,0);
#endif

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
#if SCM__R0309_INTRABC_BVP
  m_pcRdCost->setPredictors(pcMvPred);
#else
  m_pcRdCost->setPredictor(mvPred);
#endif 
  m_pcRdCost->setCostScale  ( 0 );

#if SCM__R0186_INTRABC_BVD
  m_pcEntropyCoder->m_pcEntropyCoderIf->estBvdBin0Cost( m_pcRdCost->getMvdBin0CostPtr());  
#endif

  //  Do integer search
#if SCM__R0309_INTRABC_BVP
  xIntraPatternSearch      ( pcCU, uiPartAddr, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, iRoiWidth, iRoiHeight, pcMvPred, bUse1DSearchFor8x8 );
#else
  xIntraPatternSearch      ( pcCU, uiPartAddr, pcPatternKey, piRefY, iRefStride, &cMvSrchRngLT, &cMvSrchRngRB, rcMv, ruiCost, iRoiWidth, iRoiHeight, mvPred, bUse1DSearchFor8x8 );
  *pcMvPred = mvPred;
#endif 
  //printf("ruiCost = %d\n", ruiCost);

#if SCM__R0309_INTRABC_BVP
  UInt uiMvBits = 0;
  UInt uiMvBitsBest = MAX_UINT;
  UInt predIdxBest = 0;
  for(UInt idx = 0; idx < 2; idx++)
  {
    m_pcRdCost->setPredictor( pcMvPred[idx] );
#if SCM__R0186_INTRABC_BVD
    uiMvBits = m_pcRdCost->getBvBits( rcMv.getHor(), rcMv.getVer() );
#else
    uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );
#endif 

    if( uiMvBits < uiMvBitsBest)
    {
      uiMvBitsBest = uiMvBits;
      predIdxBest = idx;
    }
  }

#if !SCM__R0081_CODE_SIMPLIFICATION
  uiMvBits = uiMvBitsBest;
#endif
  pcCU->setMVPIdxSubParts( predIdxBest, REF_PIC_LIST_INTRABC, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
#else
#if !SCM__R0081_CODE_SIMPLIFICATION
  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );
#endif 
#endif 

#if !SCM__R0081_CODE_SIMPLIFICATION
  ruiBits      += uiMvBits;
  ruiCost       = (Distortion)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
#endif
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

  if((pcCU->getWidth(0) == 16) && (pcCU->getPartitionSize(0) == SIZE_2Nx2N) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
  {
    srLeft  = -1 * cuPelX;
    srTop   = -1 * cuPelY;

    srRight = iPicWidth - cuPelX - iRoiWidth;
    srBottom = lcuHeight - cuPelY % lcuHeight - iRoiHeight;
  }
  else
  {
#if SCM__FLEXIBLE_INTRABC_SEARCH
  const UInt uiSearchWidthInCTUs = pcCU->getWidth( 0 ) == 8 ? m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() : m_pcEncCfg->getIntraBCSearchWidthInCTUs();
  Int maxXsr = (cuPelX % lcuWidth) + pcCU->getIntraBCSearchAreaWidth( uiSearchWidthInCTUs );
#else
  Int maxXsr = (cuPelX % lcuWidth) + pcCU->getIntraBCSearchAreaWidth();
#endif
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

  rcMvSrchRngLT.setHor( srLeft );
  rcMvSrchRngLT.setVer( srTop );
  rcMvSrchRngRB.setHor( srRight );
  rcMvSrchRngRB.setVer( srBottom );

  pcCU->clipMv        ( rcMvSrchRngLT );
  pcCU->clipMv        ( rcMvSrchRngRB );
}



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
        pcPredCU = pcCU->getCULeft();
        currX += pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getNumPartInWidth();
        uiAbsPartIdx = g_auiRasterToZscan[currX/pcCU->getPic()->getMinCUWidth() + (currY/pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInWidth()];
        if(pcPredCU->isInter(uiAbsPartIdx))
          return false;
      }
      else
      {
        pcPredCU = pcCU->getPic()->getCU( pcCU->getAddr() );
        uiAbsPartIdx = g_auiRasterToZscan[currX/pcCU->getPic()->getMinCUWidth() + (currY/pcCU->getPic()->getMinCUHeight())*pcCU->getPic()->getNumPartInWidth()];
        if(pcPredCU->isInter(uiAbsPartIdx))
          return false;
      }
    }
  }

  return true;
}

Void TEncSearch::xIntraBCSearchMVCandUpdate(Distortion  uiSad, Int x, Int y, Distortion* uiSadBestCand, TComMv* cMVCand)
{
  int j = RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1;

  if(uiSad < uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
  {
    for(int t = RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
    {
      if(uiSad < uiSadBestCand[t])
        j = t;
    }

    for(int k = RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1;k > j; k--)
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
                                            )
{
  Int iBestCandIdx = 0;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
#if SCM__R0081_CODE_SIMPLIFICATION
  Distortion  uiTempSad;
#else
  Distortion  uiTempSad, uiUVSad;
#endif 
  Pel* pRef;
  Pel* pOrg;
  Int iRefStride, iOrgStride;
  Int iWidth, iHeight;
  Int iMvx, iMvy;

#if !SCM__R0081_CODE_SIMPLIFICATION
  Double fYWeight, fUVWeight;
#endif 
  Int iPicWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
  Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();

#if !SCM__R0081_CODE_SIMPLIFICATION
  fYWeight =  1.0;
  fUVWeight = 1.0;
#endif 

  for(int iCand = 0; iCand < RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    if((!cMVCand[iCand].getHor()) && (!cMVCand[iCand].getVer())) 
      continue;

    if(((Int) (cuPelY + cMVCand[iCand].getVer() + iRoiHeight) >= iPicHeight) || ((cuPelY + cMVCand[iCand].getVer()) < 0))
      continue;

    if(((Int) (cuPelX + cMVCand[iCand].getHor() + iRoiWidth) >= iPicWidth) || ((cuPelX + cMVCand[iCand].getHor()) < 0))
      continue;

#if SCM__R0081_CODE_SIMPLIFICATION
    uiTempSad = uiSadBestCand[iCand];
#else
#if SCM__R0309_INTRABC_BVP
    Distortion distTemp = m_pcRdCost->getCostMultiplePreds( cMVCand[iCand].getHor(), cMVCand[iCand].getVer());
    uiTempSad = uiSadBestCand[iCand] - distTemp;
    uiTempSad = (UInt)((Double)(uiTempSad) * fYWeight) + distTemp;
#else
    uiTempSad = uiSadBestCand[iCand] - m_pcRdCost->getCost( cMVCand[iCand].getHor(), cMVCand[iCand].getVer());
    uiTempSad = (UInt)((Double)(uiTempSad) * fYWeight) + m_pcRdCost->getCost( cMVCand[iCand].getHor(), cMVCand[iCand].getVer());
#endif 
#endif
    for (UInt ch = COMPONENT_Cb; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
#if !SCM__R0081_CODE_SIMPLIFICATION
      uiUVSad = 0;
#endif 

      pRef = pcCU->getPic()->getPicYuvRec()->getAddr(ComponentID(ch), pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartOffset);
#if SCM__R0147_RGB_YUV_RD_ENC
      if( pcCU->getSlice()->getSPS()->getUseColorTrans () && m_pcEncCfg->getRGBFormatFlag() )
      {
        pOrg = pcCU->getPic()->getPicYuvResi()->getAddr(ComponentID(ch), pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartOffset);
      }
      else
#endif
      pOrg = pcCU->getPic()->getPicYuvOrg()->getAddr(ComponentID(ch), pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartOffset);
      iRefStride = pcCU->getPic()->getPicYuvRec()->getStride(ComponentID(ch));
#if SCM__R0147_RGB_YUV_RD_ENC
      if( pcCU->getSlice()->getSPS()->getUseColorTrans () && m_pcEncCfg->getRGBFormatFlag() )
      {
        iOrgStride = pcCU->getPic()->getPicYuvResi()->getStride(ComponentID(ch));
      }
      else
#endif
      iOrgStride = pcCU->getPic()->getPicYuvOrg()->getStride(ComponentID(ch));
      iWidth = iRoiWidth >> pcCU->getPic()->getComponentScaleX(ComponentID(ch));
      iHeight = iRoiHeight >> pcCU->getPic()->getComponentScaleY(ComponentID(ch));
      iMvx = cMVCand[iCand].getHor() >> pcCU->getPic()->getComponentScaleX(ComponentID(ch));
      iMvy = cMVCand[iCand].getVer() >> pcCU->getPic()->getComponentScaleY(ComponentID(ch));

      pRef = pRef + iMvy * iRefStride + iMvx;

      for(int row = 0; row < iHeight; row++)
      {
        for(int col = 0; col < iWidth; col++)
        {
#if SCM__R0081_CODE_SIMPLIFICATION
          uiTempSad += abs(pRef[col] - pOrg[col]);
#else
          uiUVSad += abs(pRef[row * iRefStride + col] - pOrg[row * iOrgStride + col]);
#endif 
        }
#if SCM__R0081_CODE_SIMPLIFICATION
        pRef += iRefStride;
        pOrg += iOrgStride;
#endif
      }

#if !SCM__R0081_CODE_SIMPLIFICATION
      uiTempSad += (UInt)((Double)(uiUVSad) * fUVWeight);
#endif 
    }

    if(uiTempSad < uiSadBest)
    {
      uiSadBest = uiTempSad;
      iBestCandIdx = iCand;
    }
  }

  return iBestCandIdx;
}


// based on xPatternSearch
Void TEncSearch::xIntraPatternSearch( TComDataCU  *pcCU,
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
#if SCM__R0309_INTRABC_BVP
                                      TComMv      *mvPred,
#else
                                      TComMv      &mvPred,
#endif 
                                      Bool         bUse1DSearchFor8x8
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
  Distortion  uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES];
  TComMv      cMVCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES];

  uiPartOffset = uiPartAddr;

  for(int iCand = 0; iCand < RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    uiSadBestCand[iCand] = std::numeric_limits<Distortion>::max();
    cMVCand[iCand].set(0,0);
  }

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  const Int        iRelCUPelX    = cuPelX % lcuWidth;
  const Int        iRelCUPelY    = cuPelY % lcuHeight;
  
  const ChromaFormat format = pcCU->getPic()->getChromaFormat();
    
  const Int chromaROIWidthInPixels  = (((format == CHROMA_420) || (format == CHROMA_422)) && (iRoiWidth  == 4) && ((iRelCUPelX & 0x4) != 0)) ? (iRoiWidth  * 2) : iRoiWidth;
  const Int chromaROIHeightInPixels = (((format == CHROMA_420)                          ) && (iRoiHeight == 4) && ((iRelCUPelY & 0x4) != 0)) ? (iRoiHeight * 2) : iRoiHeight;
  const Int chromaROIStartXInPixels = iRelCUPelX + iRoiWidth  - chromaROIWidthInPixels;
  const Int chromaROIStartYInPixels = iRelCUPelY + iRoiHeight - chromaROIHeightInPixels;

  if (m_pcEncCfg->getUseIntraBlockCopyFastSearch())
  {
    setDistParamComp(COMPONENT_Y);
    m_cDistParam.bitDepth  = g_bitDepth[CHANNEL_TYPE_LUMA];
    m_cDistParam.iRows     = 4;//to calculate the sad line by line;
    m_cDistParam.iSubShift = 0;

    Distortion uiTempSadBest = 0;

#if SCM__R0081_BUGFIX
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
    }
#endif

#if SCM__R0309_INTRABC_BVP
    for(Int iPred = 0; iPred < 2; iPred++)
    {
      Int xPred = mvPred[iPred].getHor();
      Int yPred = mvPred[iPred].getVer();
#else
    const Int xPred = mvPred.getHor();
    const Int yPred = mvPred.getVer();
#endif

    uiSad = std::numeric_limits<Distortion>::max();

    if ( !( xPred==0 && yPred==0)
#if SCM__R0081_BUGFIX
      && !( (yPred < srTop)  || (yPred > srBottom) )
      && !( (xPred < srLeft) || (xPred > srRight) )
#else
      && !( (yPred < iSrchRngVerTop)  || (yPred > iSrchRngVerBottom) )
      && !( (xPred < iSrchRngHorLeft) || (xPred > iSrchRngHorRight) )
#endif
      )
    {
      Int iTempY = yPred + iRelCUPelY + iRoiHeight - 1;
      Int iTempX = xPred + iRelCUPelX + iRoiWidth  - 1;
      Bool validCand = isValidIntraBCSearchArea(pcCU, xPred + chromaROIStartXInPixels, yPred + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels);
 
#if SCM__R0081_BUGFIX
      if((iTempX >= (Int)lcuWidth) && (iTempY >= 0) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
#else
      if((iTempX >= lcuWidth) && (iTempY >= 0) && m_pcEncCfg->getUseIntraBCFullFrameSearch())
#endif
      {
        validCand = false;
      }

      if ((iTempX >= 0) && (iTempY >= 0))
      {
        Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
        Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
        if(iTempZscanIdx >= pcCU->getZorderIdxInCU())
        {
          validCand = false;
        }
      }

      if( validCand )
      {
#if SCM__R0309_INTRABC_BVP
        uiSad = m_pcRdCost->getCostMultiplePreds( xPred, yPred);
#else
        uiSad = m_pcRdCost->getCost( xPred, yPred);
#endif

        for(int r = 0; r < iRoiHeight; )
        {
          piRefSrch = piRefY + yPred * iRefStride + r*iRefStride + xPred;
          m_cDistParam.pCur = piRefSrch;
          m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

          uiSad += m_cDistParam.DistFunc( &m_cDistParam );

          r += 4;
        }

#if SCM__R0081_BUGFIX
        xIntraBCSearchMVCandUpdate(uiSad, xPred, yPred, uiSadBestCand, cMVCand);
        iBestX = cMVCand[0].getHor();
        iBestY = cMVCand[0].getVer();
        uiSadBest = uiSadBestCand[0];
#else
        if ( uiSad < uiSadBest )
        {
          uiSadBest = uiSad;
          iBestX    = xPred;
          iBestY    = yPred;
        }
#endif
        rcMv.set( iBestX, iBestY );
      }
    }
#if SCM__R0309_INTRABC_BVP
    } // for iPred
#endif

    const Int boundY = (0 - iRoiHeight - puPelOffsetY);
    for(Int y = max(iSrchRngVerTop, 0 - cuPelY) ; y <= boundY ; ++y )
    {
      if (!isValidIntraBCSearchArea(pcCU, 0 + chromaROIStartXInPixels, y + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
        continue;

#if SCM__R0309_INTRABC_BVP
      uiSad = m_pcRdCost->getCostMultiplePreds( 0, y);
#else
      uiSad = m_pcRdCost->getCost( 0, y);
#endif 

      for(int r = 0; r < iRoiHeight; )
      {
        piRefSrch = piRefY + y * iRefStride + r*iRefStride;
        m_cDistParam.pCur = piRefSrch;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

        uiSad += m_cDistParam.DistFunc( &m_cDistParam );

        if(uiSad > uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
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
#if SCM__R0081_CODE_SIMPLIFICATION
        ruiSAD = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
        ruiSAD = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
        ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
        return;
      }
    }

    const Int boundX = max(iSrchRngHorLeft, - cuPelX);
    for(Int x = 0 - iRoiWidth - puPelOffsetX ; x >= boundX ; --x )
    {
      if (!isValidIntraBCSearchArea(pcCU, x + chromaROIStartXInPixels, 0 + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
        continue;

#if SCM__R0309_INTRABC_BVP
      uiSad = m_pcRdCost->getCostMultiplePreds( x, 0);
#else
      uiSad = m_pcRdCost->getCost( x, 0);
#endif 

      for(int r = 0; r < iRoiHeight; )
      {
        piRefSrch = piRefY + r*iRefStride + x;
        m_cDistParam.pCur = piRefSrch;
        m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

        uiSad += m_cDistParam.DistFunc( &m_cDistParam );
        if(uiSad > uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
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
#if SCM__R0081_CODE_SIMPLIFICATION
        ruiSAD = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
        ruiSAD = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
        ruiSAD = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
        return;
      }


    }

    iBestX = cMVCand[0].getHor();
    iBestY = cMVCand[0].getVer();
    uiSadBest = uiSadBestCand[0]; 
#if SCM__R0309_INTRABC_BVP
    if((!iBestX && !iBestY) || (uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY) <= 32))
#else
    if((!iBestX && !iBestY) || (uiSadBest - m_pcRdCost->getCost( iBestX, iBestY) <= 32))
#endif 
    {
      //chroma refine
      iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
      iBestX       = cMVCand[iBestCandIdx].getHor();
      iBestY       = cMVCand[iBestCandIdx].getVer();
      uiSadBest    = uiSadBestCand[iBestCandIdx]; 
      rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
      ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
      ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
      ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
      return; 
    }


    if( pcCU->getWidth(0) < 16 && !bUse1DSearchFor8x8 )
    {
#if !SCM__R0081_BUGFIX
      Int iPicWidth = pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples();
      Int iPicHeight = pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples();
#endif

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
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCU())
              continue;
          }

          if (!isValidIntraBCSearchArea(pcCU, x + chromaROIStartXInPixels, y + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
            continue;

#if SCM__R0309_INTRABC_BVP
          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
#else
          uiSad = m_pcRdCost->getCost( x, y);
#endif
          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
        }
      }

      iBestX = cMVCand[0].getHor();
      iBestY = cMVCand[0].getVer();
      uiSadBest = uiSadBestCand[0];
#if SCM__R0309_INTRABC_BVP
      if(uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY) <= 16)
#else
      if(uiSadBest - m_pcRdCost->getCost( iBestX, iBestY) <= 16)
#endif 
      {
        //chroma refine
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
        iBestX       = cMVCand[iBestCandIdx].getHor();
        iBestY       = cMVCand[iBestCandIdx].getVer();
        uiSadBest    = uiSadBestCand[iBestCandIdx]; 
        rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
        ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
        ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
        ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
        return;
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
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCU())
              continue;
          }

          if (!isValidIntraBCSearchArea(pcCU, x + chromaROIStartXInPixels, y + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
            continue;

#if SCM__R0309_INTRABC_BVP
          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
#else
          uiSad = m_pcRdCost->getCost( x, y);
#endif 

          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
          if(uiSadBestCand[0] <= 5)
          {
            //chroma refine & return
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
            iBestX       = cMVCand[iBestCandIdx].getHor();
            iBestY       = cMVCand[iBestCandIdx].getVer();
            uiSadBest    = uiSadBestCand[iBestCandIdx]; 
            rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
            ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
            ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
            ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
            return;
          }
        }
      }

      iBestX = cMVCand[0].getHor();
      iBestY = cMVCand[0].getVer();
      uiSadBest = uiSadBestCand[0];

#if SCM__R0309_INTRABC_BVP
      if((uiSadBest >= uiTempSadBest) || ((uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY)) <= 32))
#else
      if((uiSadBest >= uiTempSadBest) || ((uiSadBest - m_pcRdCost->getCost( iBestX, iBestY)) <= 32))
#endif 
      {
        //chroma refine
        iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
        iBestX       = cMVCand[iBestCandIdx].getHor();
        iBestY       = cMVCand[iBestCandIdx].getVer();
        uiSadBest    = uiSadBestCand[iBestCandIdx]; 
        rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
        ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
        ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
        ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
        return; 
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
            Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
            Int iTempZscanIdx = g_auiRasterToZscan[iTempRasterIdx];
            if(iTempZscanIdx >= pcCU->getZorderIdxInCU())
              continue;
          }

          if (!isValidIntraBCSearchArea(pcCU, x + chromaROIStartXInPixels, y + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
            continue;

#if SCM__R0309_INTRABC_BVP
          uiSad = m_pcRdCost->getCostMultiplePreds( x, y);
#else
          uiSad = m_pcRdCost->getCost( x, y);
#endif

          for(int r = 0; r < iRoiHeight; )
          {
            piRefSrch = piRefY + y * iRefStride + r*iRefStride + x;
            m_cDistParam.pCur = piRefSrch;
            m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

            uiSad += m_cDistParam.DistFunc( &m_cDistParam );
            if(uiSad > uiSadBestCand[RExt__Q0175_CHROMA_REFINEMENT_CANDIDATES - 1])
              break;

            r += 4;
          }

          xIntraBCSearchMVCandUpdate(uiSad, x, y, uiSadBestCand, cMVCand);
          if(uiSadBestCand[0] <= 5)
          {
            //chroma refine & return
            iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
            iBestX       = cMVCand[iBestCandIdx].getHor();
            iBestY       = cMVCand[iBestCandIdx].getVer();
            uiSadBest    = uiSadBestCand[iBestCandIdx]; 
            rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
            ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
            ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
            ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif
            return;  
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
          Int iTempRasterIdx = (iTempY/pcCU->getPic()->getMinCUHeight()) * pcCU->getPic()->getNumPartInWidth() + (iTempX/pcCU->getPic()->getMinCUWidth());
          Int iTempZscanIdx  = g_auiRasterToZscan[iTempRasterIdx];
          if(iTempZscanIdx >= pcCU->getZorderIdxInCU())
            continue;
        }

        if (!isValidIntraBCSearchArea(pcCU, x + chromaROIStartXInPixels, y + chromaROIStartYInPixels, chromaROIWidthInPixels, chromaROIHeightInPixels))
          continue;

        piRefSrch = piRefY + x;
        m_cDistParam.pCur = piRefSrch;

        m_cDistParam.bitDepth = g_bitDepth[CHANNEL_TYPE_LUMA];
        uiSad = m_cDistParam.DistFunc( &m_cDistParam );

#if SCM__R0309_INTRABC_BVP
        uiSad += m_pcRdCost->getCostMultiplePreds( x, y);
#else
        uiSad += m_pcRdCost->getCost( x, y);
#endif
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

  iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, uiPartOffset);
  iBestX       = cMVCand[iBestCandIdx].getHor();
  iBestY       = cMVCand[iBestCandIdx].getVer();
  uiSadBest    = uiSadBestCand[iBestCandIdx];  
  rcMv.set( iBestX, iBestY );
#if SCM__R0081_CODE_SIMPLIFICATION
  ruiSAD       = uiSadBest;
#else
#if SCM__R0309_INTRABC_BVP
  ruiSAD       = uiSadBest - m_pcRdCost->getCostMultiplePreds( iBestX, iBestY);
#else
  ruiSAD       = uiSadBest - m_pcRdCost->getCost( iBestX, iBestY);
#endif
#endif

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
    HashPic = pcCU->getPic()->getPicYuvOrg();
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

  if(grad < 5)
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

#if SCM__R0081_CODE_SIMPLIFICATION
Void TEncSearch::xIntraBCHashSearch( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, TComMv* pcMvPred, TComMv& rcMv, UInt uiIntraBCECost)
#else
Void TEncSearch::xIntraBCHashSearch( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, TComMv* pcMvPred, TComMv& rcMv, UInt& ruiBits, Distortion& ruiCost, UInt uiIntraBCECost)
#endif
{
  UInt      uiPartAddr;
  Int       iRoiWidth;
  Int       iRoiHeight;

  TComYuv*    pcYuv = pcYuvOrg;

  TComPattern   tmpPattern;
  TComPattern*  pcPatternKey  = &tmpPattern;

#if !SCM__R0081_CODE_SIMPLIFICATION
  Double     fWeight     = 1.0;
#endif 

  Int        iOrgHashIndex;

#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
  Distortion  uiSadBestCand[SCM__R0162_CHROMA_REFINEMENT_CANDIDATES];
  TComMv      cMVCand[SCM__R0162_CHROMA_REFINEMENT_CANDIDATES];
  for(int iCand = 0; iCand < SCM__R0162_CHROMA_REFINEMENT_CANDIDATES; iCand++)
  {
    uiSadBestCand[iCand] = std::numeric_limits<Distortion>::max();
    cMVCand[iCand].set(0,0);
  }  
#endif

  pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iRoiWidth, iRoiHeight ); 

  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
    iRoiWidth,
    iRoiHeight,
    pcYuv->getStride(COMPONENT_Y) );

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

#if !SCM__R0081_CODE_SIMPLIFICATION
  TComMv      ZeroMv(0,0);
#endif

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );
  m_pcRdCost->setPredictor(*pcMvPred);
  m_pcRdCost->setCostScale  ( 0 );
#if SCM__R0186_INTRABC_BVD
  m_pcEntropyCoder->m_pcEntropyCoderIf->estBvdBin0Cost( m_pcRdCost->getMvdBin0CostPtr());  
#endif

  m_pcRdCost->setDistParam( pcPatternKey, piRefY, iRefStride,  m_cDistParam );

  setDistParamComp(COMPONENT_Y);
  m_cDistParam.bitDepth  = g_bitDepth[CHANNEL_TYPE_LUMA];
  m_cDistParam.iRows     = 4;//to calculate the sad line by line;
  m_cDistParam.iSubShift = 0;

  Int  cuPelX     = pcCU->getCUPelX();
  Int  cuPelY     = pcCU->getCUPelY();
#if SCM__FLEXIBLE_INTRABC_SEARCH
  const UInt uiMaxCuWidth         = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt uiMaxCuHeight        = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  const UInt uiSearchWidth        = m_pcEncCfg->getUseIntraBCFullFrameSearch() ? 0 : pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCSearchWidthInCTUs() );
  const UInt uiNonHashSearchWidth = pcCU->getIntraBCSearchAreaWidth( m_pcEncCfg->getIntraBCNonHashSearchWidthInCTUs() );
  const UInt uiCtuPelX            = (cuPelX / uiMaxCuWidth) * uiMaxCuWidth;
  const UInt uiCtuPelY            = (cuPelY / uiMaxCuHeight) * uiMaxCuHeight;
#endif

  Distortion  uiSad;

#if !SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT || !SCM__R0081_CODE_SIMPLIFICATION
  Distortion  uiSadBest = uiIntraBCECost;
#endif 
#if !SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
  Int         iBestX    = 0;
  Int         iBestY    = 0;
#endif
  Int         iTempX;
  Int         iTempY;
  Pel*  piRefSrch;

#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
  xIntraBCSearchMVCandUpdate(uiIntraBCECost, rcMv.getHor(), rcMv.getVer(), uiSadBestCand, cMVCand);
#endif

  while(HashLinklist)
  {
    iTempX = HashLinklist->pos_X;
    iTempY = HashLinklist->pos_Y;

#if SCM__FLEXIBLE_INTRABC_SEARCH
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
#endif

    uiSad = 0;//m_pcRdCost->getCost( iTempX - cuPelX, iTempY - cuPelY);

    for(int r = 0; r < iRoiHeight; )
    {
      piRefSrch = piRefY + iTempY * iRefStride + r*iRefStride + iTempX;
      m_cDistParam.pCur = piRefSrch;
      m_cDistParam.pOrg = pcPatternKey->getROIY() + r * pcPatternKey->getPatternLStride();

      uiSad += m_cDistParam.DistFunc( &m_cDistParam );
#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
      if(uiSad > uiSadBestCand[SCM__R0162_CHROMA_REFINEMENT_CANDIDATES-1])
#else
      if(uiSad > uiSadBest)
#endif
        break;

      r += 4;
    }

    if(uiSad <= 16)
    {
#if SCM__R0309_INTRABC_BVP
      uiSad += m_pcRdCost->getCostMultiplePreds( iTempX - cuPelX, iTempY - cuPelY);
#else
      uiSad += m_pcRdCost->getCost( iTempX - cuPelX, iTempY - cuPelY);
#endif 
#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
      xIntraBCSearchMVCandUpdate(uiSad, iTempX - cuPelX, iTempY - cuPelY, uiSadBestCand, cMVCand);
#else
      if(uiSad < uiSadBest)
      {
        uiSadBest = uiSad;//m_pcRdCost->getCost( iTempX - cuPelX, iTempY - cuPelY);
        iBestX    = iTempX - cuPelX;
        iBestY    = iTempY - cuPelY;
        rcMv.set( iBestX, iBestY );
      }
#endif
      break;
    }

#if SCM__R0309_INTRABC_BVP
    uiSad += m_pcRdCost->getCostMultiplePreds( iTempX - cuPelX, iTempY - cuPelY);
#else
    uiSad += m_pcRdCost->getCost( iTempX - cuPelX, iTempY - cuPelY);
#endif 

#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
    xIntraBCSearchMVCandUpdate(uiSad, iTempX - cuPelX, iTempY - cuPelY, uiSadBestCand, cMVCand);
#else
    if ( uiSad < uiSadBest )
    {
      uiSadBest = uiSad;
      iBestX    = iTempX - cuPelX;
      iBestY    = iTempY - cuPelY;
      rcMv.set( iBestX, iBestY );
    }
#endif

    HashLinklist = HashLinklist->next;
  }

#if SCM__R0162_INTRABC_HASH_SEARCH_ENHANCEMENT
  Int iBestCandIdx = xIntraBCSearchMVChromaRefine(pcCU, iRoiWidth, iRoiHeight, cuPelX, cuPelY, uiSadBestCand, cMVCand, 0);
  rcMv = cMVCand[iBestCandIdx];
#endif

#if !SCM__R0081_CODE_SIMPLIFICATION
#if SCM__R0309_INTRABC_BVP
  ruiCost = uiSadBest - m_pcRdCost->getCostMultiplePreds( rcMv.getHor(), rcMv.getVer());
#else
  ruiCost = uiSadBest - m_pcRdCost->getCost( rcMv.getHor(), rcMv.getVer());
#endif
#endif

#if SCM__R0309_INTRABC_BVP
  UInt uiMvBits = 0;
  UInt uiMvBitsBest = MAX_UINT;
  UInt predIdxBest = 0;
  for(UInt idx = 0; idx < 2; idx++)
  {
    m_pcRdCost->setPredictor( pcMvPred[idx] );
#if SCM__R0186_INTRABC_BVD
    uiMvBits = m_pcRdCost->getBvBits( rcMv.getHor(), rcMv.getVer() );
#else
    uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );
#endif 

    if( uiMvBits < uiMvBitsBest)
    {
      uiMvBitsBest = uiMvBits;
      predIdxBest = idx;
    }
  }

#if !SCM__R0081_CODE_SIMPLIFICATION
  uiMvBits = uiMvBitsBest;
#endif
  pcCU->setMVPIdxSubParts( predIdxBest, REF_PIC_LIST_INTRABC, uiPartAddr, iPartIdx, pcCU->getDepth(uiPartAddr));
#else
#if !SCM__R0081_CODE_SIMPLIFICATION
  UInt uiMvBits = m_pcRdCost->getBits( rcMv.getHor(), rcMv.getVer() );
#endif
#endif

#if !SCM__R0081_CODE_SIMPLIFICATION
  ruiBits      += uiMvBits;
  ruiCost       = (Distortion)( floor( fWeight * ( (Double)ruiCost - (Double)m_pcRdCost->getCost( uiMvBits ) ) ) + (Double)m_pcRdCost->getCost( ruiBits ) );
#endif 

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

  Int        iOrgHashIndex;
  IntraBCHashNode* NewHashNode;


  for(int j = 0; j < MAX_CU_SIZE; j++)
  {
    for(int i = 0; i < MAX_CU_SIZE; i++)
    {
      iTempX = cuPelX - iRoiWidth + 1 + i;
      iTempY = cuPelY - iRoiHeight + 1  + j;

      if((iTempX < 0) || (iTempY < 0) || ((iTempX + iRoiWidth) >= iPicWidth) || ((iTempY + iRoiHeight) >= iPicHeight))
      {
        continue;
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
#if ZERO_MVD_EST
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, Distortion* puiDistBiP, Distortion* puiDist  )
#else
Void TEncSearch::xEstimateMvPredAMVP( TComDataCU* pcCU, TComYuv* pcOrgYuv, UInt uiPartIdx, RefPicList eRefPicList, Int iRefIdx, TComMv& rcMvPred, Bool bFilled, Distortion* puiDistBiP )
#endif
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
#if !ZERO_MVD_EST
  if (pcAMVPInfo->iN <= 1)
  {
    rcMvPred = cBestMv;

    pcCU->setMVPIdxSubParts( iBestIdx, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));

    if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefPicList==REF_PIC_LIST_1)
    {
      (*puiDistBiP) = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, rcMvPred, 0, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
    }
    return;
  }
#endif

  if (bFilled)
  {
    assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
    rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
    return;
  }

  m_cYuvPredTemp.clear();
#if ZERO_MVD_EST
  Distortion uiDist;
#endif
  //-- Check Minimum Cost.
  for ( i = 0 ; i < pcAMVPInfo->iN; i++)
  {
    Distortion uiTmpCost;
#if ZERO_MVD_EST
    uiTmpCost = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight, uiDist );
#else
    uiTmpCost = xGetTemplateCost( pcCU, uiPartIdx, uiPartAddr, pcOrgYuv, &m_cYuvPredTemp, pcAMVPInfo->m_acMvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx, iRoiWidth, iRoiHeight);
#endif
    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      cBestMv   = pcAMVPInfo->m_acMvCand[i];
      iBestIdx  = i;
      (*puiDistBiP) = uiTmpCost;
#if ZERO_MVD_EST
      (*puiDist) = uiDist;
#endif
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

  assert(pcAMVPInfo->m_acMvCand[riMVPIdx] == rcMvPred);

  if (pcAMVPInfo->iN < 2) return;

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(0) );
  m_pcRdCost->setCostScale ( 0    );

  Int iBestMVPIdx = riMVPIdx;

  m_pcRdCost->setPredictor( rcMvPred );
  Int iOrgMvBits  = m_pcRdCost->getBits(cMv.getHor(), cMv.getVer());
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  Int iBestMvBits = iOrgMvBits;

  for (Int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->iN; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx) continue;

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
                                      #if ZERO_MVD_EST
                                       , Distortion& ruiDist
                                      #endif
                                         )
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  TComPicYuv* pcPicYuvRef = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec();

  pcCU->clipMv( cMvCand );

  // prediction pattern
  if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, true );
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pcCU, pcPicYuvRef, uiPartAddr, &cMvCand, iSizeX, iSizeY, pcTemplateCand, false );
  }

  if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType()==P_SLICE )
  {
    xWeightedPredictionUni( pcCU, pcTemplateCand, uiPartAddr, iSizeX, iSizeY, eRefPicList, pcTemplateCand, iRefIdx );
  }

  // calc distortion
#if ZERO_MVD_EST
  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

  DistParam cDistParam;
  m_pcRdCost->setDistParam( cDistParam, g_bitDepth[CHANNEL_TYPE_LUMA],
                            pcOrgYuv->getAddr(COMPONENT_Y, uiPartAddr), pcOrgYuv->getStride(COMPONENT_Y),
                            pcTemplateCand->getAddr(COMPONENT_Y, uiPartAddr), pcTemplateCand->getStride(COMPONENT_Y),
                            iSizeX, iSizeY, m_pcEncCfg->getUseHADME() );

  ruiDist = cDistParam.DistFunc( &cDistParam );
  uiCost = ruiDist + m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );
#else

  uiCost = m_pcRdCost->getDistPart( g_bitDepth[CHANNEL_TYPE_LUMA], pcTemplateCand->getAddr(COMPONENT_Y, uiPartAddr), pcTemplateCand->getStride(COMPONENT_Y), pcOrgYuv->getAddr(COMPONENT_Y, uiPartAddr), pcOrgYuv->getStride(COMPONENT_Y), iSizeX, iSizeY, COMPONENT_Y, DF_SAD );
  uiCost = (UInt) m_pcRdCost->calcRdCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum], uiCost, false, DF_SAD );
#endif
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

    pcYuv->removeHighFreq( pcYuvOther, uiPartAddr, iRoiWidth, iRoiHeight );

    fWeight = 0.5;
  }

  //  Search key pattern initialization
  pcPatternKey->initPattern( pcYuv->getAddr  ( COMPONENT_Y, uiPartAddr ),
                             iRoiWidth,
                             iRoiHeight,
                             pcYuv->getStride(COMPONENT_Y) );

  Pel*        piRefY      = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getAddr( COMPONENT_Y, pcCU->getAddr(), pcCU->getZorderIdxInCU() + uiPartAddr );
  Int         iRefStride  = pcCU->getSlice()->getRefPic( eRefPicList, iRefIdxPred )->getPicYuvRec()->getStride(COMPONENT_Y);

  TComMv      cMvPred = *pcMvPred;

  if ( bBi )  xSetSearchRange   ( pcCU, rcMv   , iSrchRng, cMvSrchRngLT, cMvSrchRngRB );
  else        xSetSearchRange   ( pcCU, cMvPred, iSrchRng, cMvSrchRngLT, cMvSrchRngRB );

  m_pcRdCost->getMotionCost( true, 0, pcCU->getCUTransquantBypass(uiPartAddr) );

  m_pcRdCost->setPredictor  ( *pcMvPred );
  m_pcRdCost->setCostScale  ( 2 );

  setWpScalingDistParam( pcCU, iRefIdxPred, eRefPicList );

  m_currRefPicList = eRefPicList;
  m_currRefPicIndex = iRefIdxPred;
  m_bSkipFracME = false;

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
#if RExt__BACKWARDS_COMPATIBILITY_MOTION_ESTIMATION_R0105
      const Profile::Name profileIdc=pcCU->getSlice()->getSPS()->getPTL()->getGeneralPTL()->getProfileIdc(); // TODO: RExt - temporary profile check to ensure backwards compatibility with HM.
      if (profileIdc != Profile::MAIN && profileIdc != Profile::MAIN10 && profileIdc != Profile::MAINSTILLPICTURE)
#endif
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
  xPatternSearchFracDIF( bIsLosslessCoded, pcPatternKey, piRefY, iRefStride, &rcMv, cMvHalf, cMvQter, ruiCost ,bBi );

  m_pcRdCost->setCostScale( 0 );
  rcMv <<= 2;
  rcMv += (cMvHalf <<= 1);
  rcMv +=  cMvQter;

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

      m_cDistParam.bitDepth = g_bitDepth[CHANNEL_TYPE_LUMA];
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
                                       TComPattern* pcPatternKey,
                                       Pel*         piRefY,
                                       Int          iRefStride,
                                       TComMv*      pcMvInt,
                                       TComMv&      rcMvHalf,
                                       TComMv&      rcMvQter,
                                       Distortion&  ruiCost,
                                       Bool         biPred
                                      )
{
  //  Reference pattern initialization (integer scale)
  TComPattern cPatternRoi;
  Int         iOffset    = pcMvInt->getHor() + pcMvInt->getVer() * iRefStride;
  cPatternRoi.initPattern(piRefY + iOffset,
                          pcPatternKey->getROIYWidth(),
                          pcPatternKey->getROIYHeight(),
                          iRefStride );


  if ( m_bSkipFracME )
  {
    TComMv baseRefMv( 0, 0 );
    rcMvHalf.setZero();
    m_pcRdCost->setCostScale( 0 );
    xExtDIFUpSamplingH( &cPatternRoi, biPred );
    rcMvQter = *pcMvInt;   rcMvQter <<= 2;    // for mv-cost
    ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
    return;
  }


  //  Half-pel refinement
  xExtDIFUpSamplingH ( &cPatternRoi, biPred );

  rcMvHalf = *pcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  TComMv baseRefMv(0, 0);
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 2, rcMvHalf, !bIsLosslessCoded );

  m_pcRdCost->setCostScale( 0 );

  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf, biPred );
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = *pcMvInt;   rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement( pcPatternKey, baseRefMv, 1, rcMvQter, !bIsLosslessCoded );
}


/** encode residual and calculate rate-distortion for a CU block
 * \param pcCU
 * \param pcYuvOrg
 * \param pcYuvPred
 * \param rpcYuvResi
 * \param rpcYuvResiBest
 * \param rpcYuvRec
 * \param bSkipRes
 * \returns Void
 */
Void TEncSearch::encodeResAndCalcRdInterCU( TComDataCU* pcCU, TComYuv* pcYuvOrg, TComYuv* pcYuvPred,
                                            TComYuv*& rpcYuvResi, TComYuv*& rpcYuvResiBest, TComYuv*& rpcYuvRec,
                                            Bool bSkipRes
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
                                           ,TComYuv* pcYuvNoCorrResi
#endif                                            
                                            DEBUG_STRING_FN_DECLARE(sDebug) )
{
  if ( pcCU->isIntra(0) )
  {
    return;
  }

  Bool       bHighPass    = pcCU->getSlice()->getDepth() ? true : false;
  UInt       uiBits       = 0, uiBitsBest       = 0;
  Distortion uiDistortion = 0, uiDistortionBest = 0;

  UInt        uiWidth      = pcCU->getWidth ( 0 );
  UInt        uiHeight     = pcCU->getHeight( 0 );

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  const Bool iColorTransform = pcCU->getColorTransform(0);
#endif
  //  No residual coding : SKIP mode
  if ( bSkipRes )
  {
    pcCU->setSkipFlagSubParts( true, 0, pcCU->getDepth(0) );

    rpcYuvResi->clear();

    pcYuvPred->copyToPartYuv( rpcYuvRec, 0 );

    for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compID=ComponentID(ch);
      const UInt csx=pcYuvOrg->getComponentScaleX(compID);
      const UInt csy=pcYuvOrg->getComponentScaleY(compID);
      uiDistortion += m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], rpcYuvRec->getAddr(compID), rpcYuvRec->getStride(compID), pcYuvOrg->getAddr(compID),
                                               pcYuvOrg->getStride(compID), uiWidth >> csx, uiHeight >> csy, compID);
    }

    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST]);
    m_pcEntropyCoder->resetBits();

    if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
      m_pcEntropyCoder->encodeCUTransquantBypassFlag(pcCU, 0, true);
    }

    m_pcEntropyCoder->encodeSkipFlag(pcCU, 0, true);
    m_pcEntropyCoder->encodeMergeIndex( pcCU, 0, true );

    uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    pcCU->getTotalBits()       = uiBits;
    pcCU->getTotalDistortion() = uiDistortion;
    pcCU->getTotalCost()       = m_pcRdCost->calcRdCost( uiBits, uiDistortion );

    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_TEMP_BEST]);

    static const UInt cbfZero[MAX_NUM_COMPONENT]={0,0,0};
    pcCU->setCbfSubParts( cbfZero, 0, pcCU->getDepth( 0 ) );
    pcCU->setTrIdxSubParts( 0, 0, pcCU->getDepth(0) );

#ifdef DEBUG_STRING
    rpcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
    for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
    {
      sDebug+=debug_reorder_data_token[pcCU->isIntraBC(0)?1:0][i];
    }
#endif

    return;
  }

  //  Residual coding.
  Int qp;
  Int qpBest = 0;
  Int qpMin;
  Int qpMax;
  Double  dCost, dCostBest = MAX_DOUBLE;

  UInt uiTrLevel = 0;
  if( (pcCU->getWidth(0) > pcCU->getSlice()->getSPS()->getMaxTrSize()) )
  {
    while( pcCU->getWidth(0) > (pcCU->getSlice()->getSPS()->getMaxTrSize()<<uiTrLevel) ) uiTrLevel++;
  }

  qpMin =  bHighPass ? Clip3( -pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, pcCU->getQP(0) - m_iMaxDeltaQP ) : pcCU->getQP( 0 );
  qpMax =  bHighPass ? Clip3( -pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, pcCU->getQP(0) + m_iMaxDeltaQP ) : pcCU->getQP( 0 );

  rpcYuvResi->subtract( pcYuvOrg, pcYuvPred, 0, uiWidth );

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM 
  Distortion uiZeroDistortion = 0;
  if(iColorTransform)
  {
    const UInt uiNumSamplesLuma = uiWidth*uiHeight;
    ::memset( m_pTempPel, 0, sizeof( Pel ) * uiNumSamplesLuma ); 
    uiZeroDistortion = m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_LUMA], m_pTempPel, uiWidth, rpcYuvResi->getAddr( COMPONENT_Y, 0 ), rpcYuvResi->getStride(COMPONENT_Y), uiWidth, uiHeight, COMPONENT_Y ); // initialized with zero residual destortion
    const UInt csx=pcYuvOrg->getComponentScaleX(COMPONENT_Cb);
    const UInt csy=pcYuvOrg->getComponentScaleY(COMPONENT_Cb);
    uiZeroDistortion += m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_CHROMA], m_pTempPel, uiWidth >> csx, rpcYuvResi->getAddr( COMPONENT_Cb, 0 ), rpcYuvResi->getStride(COMPONENT_Cb), uiWidth >> csx, uiHeight >> csy, COMPONENT_Cb                                        ); // initialized with zero residual destortion
    uiZeroDistortion += m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_CHROMA], m_pTempPel, uiWidth >> csx, rpcYuvResi->getAddr( COMPONENT_Cr, 0 ), rpcYuvResi->getStride(COMPONENT_Cr), uiWidth >> csx, uiHeight >> csy, COMPONENT_Cr                                        ); // initialized with zero residual destortion
 
    rpcYuvResi->convert(0, 0, uiWidth, true, pcCU->isLosslessCoded(0), pcYuvNoCorrResi);
  }
#endif

  TComTURecurse tuLevel0(pcCU, 0);

  for ( qp = qpMin; qp <= qpMax; qp++ )
  {
    dCost = 0.;
    uiBits = 0;
    uiDistortion = 0;

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_CURR_BEST ] );

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
    if(iColorTransform)
    {
      xEstimateResidualQT( pcYuvNoCorrResi, dCost, uiBits, uiDistortion, NULL, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug), rpcYuvResi );
    }
    else
    {
#else
    Distortion uiZeroDistortion = 0;
#endif

    xEstimateResidualQT( rpcYuvResi,  dCost, uiBits, uiDistortion, &uiZeroDistortion, tuLevel0 DEBUG_STRING_PASS_INTO(sDebug) );

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
    }
#endif

    // -------------------------------------------------------
    // set the coefficients in the pcCU, and also calculates the residual data.
    // If a block full of 0's is efficient, then just use 0's.
    // The costs at this point do not include header bits.

    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeQtRootCbfZero( pcCU );
    UInt zeroResiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    Double dZeroCost = m_pcRdCost->calcRdCost( zeroResiBits, uiZeroDistortion );

    if(pcCU->isLosslessCoded( 0 ))
    {
      dZeroCost = dCost + 1;
    }

    if ( (dZeroCost < dCost) || (!pcCU->isLosslessCoded( 0 ) && pcCU->isIntraBC(0) && uiZeroDistortion == uiDistortion))
    {
      if ( dZeroCost < dCost )
      {
        dCost        = dZeroCost;
      }
      uiDistortion = uiZeroDistortion;

      const UInt uiQPartNum = tuLevel0.GetAbsPartIdxNumParts();
      ::memset( pcCU->getTransformIdx()     , 0, uiQPartNum * sizeof(UChar) );
      for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
      {
        const ComponentID component = ComponentID(ch);
        const UInt componentShift   = pcCU->getPic()->getComponentScaleX(component) + pcCU->getPic()->getComponentScaleY(component);
        ::memset( pcCU->getCbf( component ) , 0, uiQPartNum * sizeof(UChar) );
        ::memset( pcCU->getCoeff(component), 0, (uiWidth*uiHeight*sizeof(TCoeff))>>componentShift );
        ::memset( pcCU->getCrossComponentPredictionAlpha(component), 0, ( uiQPartNum * sizeof(Char) ) );
      }
      static const UInt useTS[MAX_NUM_COMPONENT]={0,0,0};
      pcCU->setTransformSkipSubParts ( useTS, 0, pcCU->getDepth(0) );
#ifdef DEBUG_STRING
      sDebug.clear();
      for(UInt i=0; i<MAX_NUM_COMPONENT+1; i++)
      {
        sDebug+=debug_reorder_data_token[pcCU->isIntraBC(0)?1:0][i];
      }
#endif
    }
    else
    {
      xSetResidualQTData( NULL, false, tuLevel0); // Call first time to set coefficients.
    }

    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[pcCU->getDepth(0)][CI_CURR_BEST] );

    uiBits = 0;
    {
      TComYuv *pDummy = NULL;
      xAddSymbolBitsInter( pcCU, 0, 0, uiBits, pDummy, NULL, pDummy );
    }
    // we've now encoded the pcCU, and so have a valid bit cost


    Double dExactCost = m_pcRdCost->calcRdCost( uiBits, uiDistortion );
    dCost = dExactCost;

    // Is our new cost better?
    if ( dCost < dCostBest )
    {
      if ( !pcCU->getQtRootCbf( 0 ) )
      {
        rpcYuvResiBest->clear(); // Clear the residual image, if we didn't code it.
      }
      else
      {
        xSetResidualQTData( rpcYuvResiBest, true, tuLevel0 ); // else set the residual image data rpcYUVResiBest from the various temp images.
      }

      if( qpMin != qpMax && qp != qpMax )
      {
        const UInt uiQPartNum = tuLevel0.GetAbsPartIdxNumParts();
        ::memcpy( m_puhQTTempTrIdx, pcCU->getTransformIdx(),        uiQPartNum * sizeof(UChar) );
        for(UInt i=0; i<pcCU->getPic()->getNumberValidComponents(); i++)
        {
          const ComponentID compID=ComponentID(i);
          const UInt csr = pcCU->getPic()->getComponentScaleX(compID) + pcCU->getPic()->getComponentScaleY(compID);
          ::memcpy( m_puhQTTempCbf[compID],      pcCU->getCbf( compID ),     uiQPartNum * sizeof(UChar) );
          ::memcpy( m_pcQTTempCoeff[compID],     pcCU->getCoeff(compID),     uiWidth * uiHeight * sizeof( TCoeff ) >> csr     );
#if ADAPTIVE_QP_SELECTION
          ::memcpy( m_pcQTTempArlCoeff[compID],  pcCU->getArlCoeff(compID),  uiWidth * uiHeight * sizeof( TCoeff )>> csr     );
#endif
          ::memcpy( m_puhQTTempTransformSkipFlag[compID], pcCU->getTransformSkip(compID),     uiQPartNum * sizeof( UChar ) );
          ::memcpy( m_phQTTempCrossComponentPredictionAlpha[compID], pcCU->getCrossComponentPredictionAlpha(compID), uiQPartNum * sizeof(Char) );
        }
      }
      uiBitsBest       = uiBits;
      uiDistortionBest = uiDistortion;
      dCostBest        = dCost;
      qpBest           = qp;

      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );
    }
  }

  assert ( dCostBest != MAX_DOUBLE );

  if( qpMin != qpMax && qpBest != qpMax )
  {
    assert( 0 ); // check
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ pcCU->getDepth( 0 ) ][ CI_TEMP_BEST ] );

      // copy best cbf and trIdx to pcCU
    const UInt uiQPartNum = tuLevel0.GetAbsPartIdxNumParts();
    ::memcpy( pcCU->getTransformIdx(),       m_puhQTTempTrIdx,  uiQPartNum * sizeof(UChar) );
    for(UInt i=0; i<pcCU->getPic()->getNumberValidComponents(); i++)
    {
      const ComponentID compID=ComponentID(i);
      const UInt csr = pcCU->getPic()->getComponentScaleX(compID) + pcCU->getPic()->getComponentScaleY(compID);
      ::memcpy( pcCU->getCbf( compID ),     m_puhQTTempCbf[compID],     uiQPartNum * sizeof(UChar) );
      ::memcpy( pcCU->getCoeff(compID),     m_pcQTTempCoeff[compID],    uiWidth * uiHeight * sizeof( TCoeff ) >> csr     );
#if ADAPTIVE_QP_SELECTION
      ::memcpy( pcCU->getArlCoeff(compID),  m_pcQTTempArlCoeff[compID], uiWidth * uiHeight * sizeof( TCoeff    ) >> csr );
#endif
      ::memcpy( pcCU->getTransformSkip(compID),     m_puhQTTempTransformSkipFlag[compID], uiQPartNum * sizeof( UChar ) );
      ::memcpy( pcCU->getCrossComponentPredictionAlpha(compID),  m_phQTTempCrossComponentPredictionAlpha[compID], uiQPartNum * sizeof( Char ) );
    }
  }
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM 
  if(iColorTransform)
  {
    rpcYuvResiBest->convert(0, 0, uiWidth, false, pcCU->isLosslessCoded(0));
  }
#endif

  rpcYuvRec->addClip ( pcYuvPred, rpcYuvResiBest, 0, uiWidth );

  // update with clipped distortion and cost (qp estimation loop uses unclipped values)

  uiDistortionBest = 0;
  for(UInt ch=0; ch<rpcYuvRec->getNumberValidComponents(); ch++)
  {
    const ComponentID compID=ComponentID(ch);
    uiDistortionBest += m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], rpcYuvRec->getAddr(compID ), rpcYuvRec->getStride(compID ), pcYuvOrg->getAddr(compID ), pcYuvOrg->getStride(compID), uiWidth >> pcYuvOrg->getComponentScaleX(compID), uiHeight >> pcYuvOrg->getComponentScaleY(compID), compID);
  }
  dCostBest = m_pcRdCost->calcRdCost( uiBitsBest, uiDistortionBest );

  pcCU->getTotalBits()       = uiBitsBest;
  pcCU->getTotalDistortion() = uiDistortionBest;
  pcCU->getTotalCost()       = dCostBest;

  if ( pcCU->isSkipped(0) )
  {
    static const UInt cbfZero[MAX_NUM_COMPONENT]={0,0,0};
    pcCU->setCbfSubParts( cbfZero, 0, pcCU->getDepth( 0 ) );
  }

  pcCU->setQPSubParts( qpBest, 0, pcCU->getDepth(0) );
}



Void TEncSearch::xEstimateResidualQT( TComYuv    *pcResi,
                                      Double     &rdCost,
                                      UInt       &ruiBits,
                                      Distortion &ruiDist,
                                      Distortion *puiZeroDist,
                                      TComTU     &rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug)
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
                                      ,TComYuv* pcOrgResi
#endif                                      
                                      )
{
  //NOTE: RExt - Ideally this function would be restructured to use just one component loop, but it is kept in this form to maintain HM-compatibility for 4:2:0

  TComDataCU *pcCU        = rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiDepth      = rTu.GetTransformDepthTotal();
  const UInt uiTrMode     = rTu.GetTransformDepthRel();
  const UInt subTUDepth   = uiTrMode + 1;
  const UInt numValidComp = pcCU->getPic()->getNumberValidComponents();
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  const Bool iColorTransform = pcCU->getColorTransform(0);
#endif

  DEBUG_STRING_NEW(sSingleStringComp[MAX_NUM_COMPONENT])

  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiLog2TrSize = rTu.GetLog2LumaTrSize();

  UInt SplitFlag = ((pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) && pcCU->isInter(uiAbsPartIdx) && ( pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N ));
#ifdef DEBUG_STRING
  const Bool isIntraBc    = pcCU->isIntraBC(uiAbsPartIdx);
  const Int debugPredModeMask = DebugStringGetPredModeMask(pcCU->getPredictionMode(uiAbsPartIdx));
#endif

  Bool bCheckFull;

  if ( SplitFlag && uiDepth == pcCU->getDepth(uiAbsPartIdx) && ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) ) )
  {
    bCheckFull = false;
  }
  else
  {
    bCheckFull =  ( uiLog2TrSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() );
  }

  const Bool bCheckSplit  = ( uiLog2TrSize >  pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );

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
      pcCoeffCurr[compID]    = m_ppcQTTempCoeff[compID][uiQTTempAccessLayer] + rTu.getCoefficientOffset(compID);
#if ADAPTIVE_QP_SELECTION
      pcArlCoeffCurr[compID] = m_ppcQTTempArlCoeff[compID ][uiQTTempAccessLayer] +  rTu.getCoefficientOffset(compID);
#endif

      if(rTu.ProcessComponentSection(compID))
      {
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
        QpParam cQP(*pcCU, compID);
        if(!pcCU->isLosslessCoded(0) && iColorTransform)
        {
          cQP.Qp = cQP.Qp + (compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
          cQP.per = cQP.Qp/6;
          cQP.rem= cQP.Qp%6;
          m_pcTrQuant->adjustBitDepthandLambdaForColorTrans(compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
          m_pcRdCost->adjustLambdaForColorTrans(compID==COMPONENT_Cr? SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS );
        }
#else
        const QpParam cQP(*pcCU, compID);
#endif

        checkTransformSkip[compID] = pcCU->getSlice()->getPPS()->getUseTransformSkip() &&
                                     TUCompRectHasAssociatedTransformSkipFlag(rTu.getRect(compID), pcCU->getSlice()->getPPS()->getTransformSkipLog2MaxSize()) &&
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
                                                         && pcCU->getSlice()->getPPS()->getUseCrossComponentPrediction()
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
          const Int crossCPredictionModesToTest = (preCalcAlpha != 0)        ? 2 : 1; //NOTE: RExt - preCalcAlpha cannot be anything other than 0 if isCrossCPredictionAvailable is false

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

                  nonCoeffDist = m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                                                              m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride( compID ), pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                                                              pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual destortion
                }
                else
                {
                  nonCoeffDist = m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], m_pTempPel, tuCompRect.width, pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                                                              pcResi->getStride(compID), tuCompRect.width, tuCompRect.height, compID); // initialized with zero residual destortion
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
                *puiZeroDist += nonCoeffDist; // initialized with zero residual destortion
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

                currCompDist = m_pcRdCost->getDistPart( g_bitDepth[toChannelType(compID)], m_pcQTTempTComYuv[uiQTTempAccessLayer].getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        m_pcQTTempTComYuv[uiQTTempAccessLayer].getStride(compID),
                                                        pcResi->getAddrPix( compID, tuCompRect.x0, tuCompRect.y0 ),
                                                        pcResi->getStride(compID),
                                                        tuCompRect.width, tuCompRect.height, compID);

                currCompCost = m_pcRdCost->calcRdCost(currCompBits, currCompDist);
                  
                if (pcCU->isLosslessCoded(0)) nonCoeffCost = MAX_DOUBLE;
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

#ifdef DEBUG_STRING
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
        } //end of sub-TU loop
        while (TUIterator.nextSection(rTu));
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
        if(!pcCU->isLosslessCoded(0) && iColorTransform)
        {
          m_pcTrQuant->adjustBitDepthandLambdaForColorTrans(compID==COMPONENT_Cr? -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
          m_pcRdCost->adjustLambdaForColorTrans(compID==COMPONENT_Cr? -SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS_V: - SCM__R0147_DELTA_QP_FOR_YCgCo_TRANS);
        }
#endif

      } // processing section
    } // component loop

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
    {
      if(iColorTransform)
      {
        const TComRectangle &tuCompRect=rTu.getRect(COMPONENT_Y);
        for(UInt ch = 0; ch < numValidComp; ch++)
        {
          const ComponentID compID=ComponentID(ch);
          const TComRectangle &tuCompRectTmp = rTu.getRect(compID);
          assert(tuCompRectTmp.width == tuCompRectTmp.height);

          m_pcQTTempTComYuv[uiQTTempAccessLayer].copyPartToPartComponentMxN(compID, &m_tmpYuvPred, tuCompRectTmp);
        }
        m_tmpYuvPred.convert( rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->isLosslessCoded(uiAbsPartIdx) );  


        uiSingleDistComp[COMPONENT_Y ][0] = m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_LUMA], m_tmpYuvPred.getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), m_tmpYuvPred.getStride(COMPONENT_Y),
          pcOrgResi->getAddrPix( COMPONENT_Y, tuCompRect.x0, tuCompRect.y0 ), pcOrgResi->getStride(COMPONENT_Y), tuCompRect.width, tuCompRect.height, COMPONENT_Y );

        const TComRectangle &tuCompRectC=rTu.getRect(COMPONENT_Cb);
        uiSingleDistComp[COMPONENT_Cb][0] = m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_CHROMA], m_tmpYuvPred.getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), m_tmpYuvPred.getStride(COMPONENT_Cb),
          pcOrgResi->getAddrPix( COMPONENT_Cb, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cb), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cb );

        uiSingleDistComp[COMPONENT_Cr][0] = m_pcRdCost->getDistPart(g_bitDepth[CHANNEL_TYPE_CHROMA], m_tmpYuvPred.getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), m_tmpYuvPred.getStride(COMPONENT_Cr),
          pcOrgResi->getAddrPix( COMPONENT_Cr, tuCompRectC.x0, tuCompRectC.y0 ), pcOrgResi->getStride(COMPONENT_Cr), tuCompRectC.width, tuCompRectC.height, COMPONENT_Cr );

        uiSingleDistComp[COMPONENT_Y][1] = uiSingleDistComp[COMPONENT_Cb][1] = uiSingleDistComp[COMPONENT_Cr][1] = 0; 
      }
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
        for (UInt subTUIndex = 0; subTUIndex < 2; subTUIndex++) uiSingleDist += uiSingleDistComp[compID][subTUIndex];
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
            bestsubTUCBF[compID][subTU] = pcCU->getCbf ((uiAbsPartIdx + (subTU * partIdxesPerSubTU)), compID, subTUDepth);
        }
      }
    }


    TComTURecurse tuRecurseChild(rTu, false);
    const UInt uiQPartNumSubdiv = tuRecurseChild.GetAbsPartIdxNumParts();

    DEBUG_STRING_NEW(sSplitString[MAX_NUM_COMPONENT])

    do
    {
      DEBUG_STRING_NEW(childString)
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
      xEstimateResidualQT( pcResi, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist, tuRecurseChild DEBUG_STRING_PASS_INTO(childString), pcOrgResi );
#else
      xEstimateResidualQT( pcResi, dSubdivCost, uiSubdivBits, uiSubdivDist, bCheckFull ? NULL : puiZeroDist,  tuRecurseChild DEBUG_STRING_PASS_INTO(childString));
#endif
#ifdef DEBUG_STRING
      // split the string by component and append to the relevant output (because decoder decodes in channel order, whereas this search searches by TU-order)
      std::size_t lastPos=0;
      const std::size_t endStrng=childString.find(debug_reorder_data_token[isIntraBc?1:0][MAX_NUM_COMPONENT], lastPos);
      for(UInt ch = 0; ch < numValidComp; ch++)
      {
        if (lastPos!=std::string::npos && childString.find(debug_reorder_data_token[isIntraBc?1:0][ch], lastPos)==lastPos) lastPos+=strlen(debug_reorder_data_token[isIntraBc?1:0][ch]); // skip leading string
        std::size_t pos=childString.find(debug_reorder_data_token[isIntraBc?1:0][ch+1], lastPos);
        if (pos!=std::string::npos && pos>endStrng) lastPos=endStrng;
        sSplitString[ch]+=childString.substr(lastPos, (pos==std::string::npos)? std::string::npos : (pos-lastPos) );
        lastPos=pos;
      }
#endif
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
    xEncodeResidualQT( MAX_NUM_COMPONENT, rTu );
    for(UInt ch = 0; ch < numValidComp; ch++)
    {
      xEncodeResidualQT( ComponentID(ch), rTu );
    }

    uiSubdivBits = m_pcEntropyCoder->getNumberOfWrittenBits();
    dSubdivCost  = m_pcRdCost->calcRdCost( uiSubdivBits, uiSubdivDist );

    if (!bCheckFull || (uiCbfAny && (dSubdivCost < dSingleCost)))
    {
      rdCost += dSubdivCost;
      ruiBits += uiSubdivBits;
      ruiDist += uiSubdivDist;
#ifdef DEBUG_STRING
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
#ifdef DEBUG_STRING
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



Void TEncSearch::xEncodeResidualQT( const ComponentID compID, TComTU &rTu )
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
      m_pcEntropyCoder->encodeTransformSubdivFlag( bSubdiv, 5 - uiLog2TrSize );
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
        xEncodeResidualQT( compID, tuRecurseChild );
      } while (tuRecurseChild.nextSection(rTu));
    }
  }
}




Void TEncSearch::xSetResidualQTData( TComYuv* pcResi, Bool bSpatial, TComTU &rTu )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiCurrTrMode=rTu.GetTransformDepthRel();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  assert( pcCU->getDepth( 0 ) == pcCU->getDepth( uiAbsPartIdx ) );
  const UInt uiTrMode = pcCU->getTransformIdx( uiAbsPartIdx );
  TComSPS *sps=pcCU->getSlice()->getSPS();

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
      xSetResidualQTData( pcResi, bSpatial, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}




UInt TEncSearch::xModeBitsIntra( TComDataCU* pcCU, UInt uiMode, UInt uiPartOffset, UInt uiDepth, UInt uiInitTrDepth, const ChannelType chType )
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
    m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiPartOffset);
  else
    m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiPartOffset);

  rIntraDirVal = origVal; // restore

  return m_pcEntropyCoder->getNumberOfWrittenBits();
}




UInt TEncSearch::xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList )
{
  UInt i;
  UInt shift=0;

  while ( shift<uiFastCandNum && uiCost<CandCostList[ uiFastCandNum-1-shift ] ) shift++;

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
 * \param rpcYuvRec
 * \param pcYuvPred
 * \param rpcYuvResi
 * \returns Void
 */
Void  TEncSearch::xAddSymbolBitsInter( TComDataCU* pcCU, UInt uiQp, UInt uiTrMode, UInt& ruiBits, TComYuv*& rpcYuvRec, TComYuv*pcYuvPred, TComYuv*& rpcYuvResi )
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

    if (pcCU->getSlice()->getSPS()->getUseIntraBlockCopy())
    {
      m_pcEntropyCoder->encodeIntraBCFlag(pcCU, 0, true);
      if ( pcCU->isIntraBC( 0 ) )
      {
        m_pcEntropyCoder->encodePartSizeIntraBC( pcCU, 0 );
        m_pcEntropyCoder->encodeIntraBC( pcCU, 0 );
      }
    }

    if( !pcCU->isIntraBC(0))
    {
      m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
      m_pcEntropyCoder->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );
      m_pcEntropyCoder->encodePredInfo( pcCU, 0 );
    }

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
Void TEncSearch::xExtDIFUpSamplingH( TComPattern* pattern, Bool biPred )
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

  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 0, false, chFmt);
  if ( !m_bSkipFracME )
  {
   m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2].getAddr(COMPONENT_Y), intStride, width+1, height+filterSize, 2, false, chFmt);
  }


  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+0, 0, false, true, chFmt);

  if ( m_bSkipFracME )
  {
    return;
  }

  intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+0, height+1, 2, false, true, chFmt);

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+0, 0, false, true, chFmt);

  intPtr = m_filteredBlockTmp[2].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[2][2].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width+1, height+1, 2, false, true, chFmt);
}





/**
 * \brief Generate quarter-sample interpolated blocks
 *
 * \param pattern    Reference picture ROI
 * \param halfPelRef Half-pel mv
 * \param biPred     Flag indicating whether block is for biprediction
 */
Void TEncSearch::xExtDIFUpSamplingQ( TComPattern* pattern, TComMv halfPelRef, Bool biPred )
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
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1, false, chFmt);

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
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3, false, chFmt);

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt);

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt);

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][1].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt);

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
    dstPtr = m_filteredBlock[2][3].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2, false, true, chFmt);
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt);

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3].getAddr(COMPONENT_Y);
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0, false, true, chFmt);
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
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt);

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
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt);
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
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt);

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0].getAddr(COMPONENT_Y);
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt);
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][3].getAddr(COMPONENT_Y);
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1, false, true, chFmt);

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3].getAddr(COMPONENT_Y) + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][3].getAddr(COMPONENT_Y);
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3, false, true, chFmt);
}





/** set wp tables
 * \param TComDataCU* pcCU
 * \param iRefIdx
 * \param eRefPicListCur
 * \returns Void
 */
Void  TEncSearch::setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.bApplyWeight = false;
    return;
  }

  TComSlice       *pcSlice  = pcCU->getSlice();
  TComPPS         *pps      = pcCU->getSlice()->getPPS();
  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.bApplyWeight = ( pcSlice->getSliceType()==P_SLICE && pps->getUseWP() ) || ( pcSlice->getSliceType()==B_SLICE && pps->getWPBiPred() ) ;

  if ( !m_cDistParam.bApplyWeight ) return;

  Int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  Int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcCU, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 ) wp0 = NULL;
  if ( iRefIdx1 < 0 ) wp1 = NULL;

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
