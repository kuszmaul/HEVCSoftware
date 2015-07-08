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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComYuv.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"
#include "TComRdCost.h"

// forward declaration
class TComMv;
class TComTU; 

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
typedef enum PRED_BUF_E
{
  PRED_BUF_UNFILTERED=0,
  PRED_BUF_FILTERED=1,
  NUM_PRED_BUF=2
} PRED_BUF;

#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
struct pltInfoStruct{
  UInt position;
  UChar pltMode;
  UInt       index;
  UInt       indexPred;
  UInt       run;
  UInt64     bitsRun;
  UInt64     bitsInd;
  UInt64     bitsAll; 
  Double     error;
  UInt       escape;
  UInt       usedForCopy;
};
#endif

static const UInt MAX_INTRA_FILTER_DEPTHS=5;

class TComPrediction : public TComWeightPrediction
{
private:
  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

protected:
  Int       m_iPLTErrLimit;
  Pel       m_cIndexBlock[MAX_CU_SIZE * MAX_CU_SIZE];
  UInt*     m_puiScanOrder;
  Pel*      m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int       m_iYuvExtSize;
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  UInt      m_indError[32*32][MAX_PLT_SIZE+1];
  pltInfoStruct m_pltInfo[32*32];
  UInt      m_pltNoElements;

  pltInfoStruct m_pltInfoBest[32*32];
  UInt      m_pltNoElementsBest;

  Pel       m_cPosBlock[32*32];
  Pel       m_cPosBlockRD[32*32];
  Pel       m_cIndexBlockRD[MAX_CU_SIZE * MAX_CU_SIZE];

  Int       m_prevQP;
  UInt      m_uiMaxVal[3];
  Int       m_quantiserScale[3];
  Int       m_quantiserRightShift[3];
  Int       m_rightShiftOffset[3];
  Int       m_invQuantScales[3];
  Int       m_iQPper[3];
  UShort**  m_truncBinBits; //ZF two dimensions related with input bitdepth
#if SCM_U0052_ESCAPE_PIXEL_CODING
  UShort*   m_escapeNumBins; // number of binarization bins fpr escape pixels
#endif
  UInt      m_MaxSymbolSize;
  UInt      m_SymbolSize;

  UChar*     m_SPointRD;
  UChar*     m_EscapeFlagRD;
  TCoeff*    m_RunRD;
  Pel*       m_LevelRD[MAX_NUM_COMPONENT];
  pltInfoStruct m_pltInfoStoreRD[32*32];
#endif

  TComYuv   m_acYuvPred[NUM_REF_PIC_LIST_01];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComYuv m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];

  TComInterpolationFilter m_if;

  Pel*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

  Void xPredIntraAng            ( Int bitDepth, const Pel* pSrc, Int srcStride, Pel* pDst, Int dstStride, UInt width, UInt height, ChannelType channelType, UInt dirMode, const Bool bEnableEdgeFilters );
  Void xPredIntraPlanar         ( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );

  // motion compensation functions
  Void xPredInterUni            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi=false          );
  Void xPredInterBi             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv* pcYuvPred          );
  Void xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth, Bool isIntraBC = false );
  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths  ); 

  Void xGetLLSPrediction ( const Pel* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0, const ChromaFormat chFmt  DEBUG_STRING_FN_DECLARE(sDebug) );

  Void xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType );
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
  Void destroy();

public:
  TComPrediction();
  virtual ~TComPrediction();

  Void    initTempBuff(ChromaFormat chromaFormatIDC);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  Void    initTBCTable(UInt bitDepth);
#endif

  ChromaFormat getChromaFormat() const { return m_cYuvPredTemp.getChromaFormat(); }

  // inter
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );

  // Angular Intra
  Void predIntraAng               ( const ComponentID compID, UInt uiDirMode, Pel *piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM = false );

  Pel  predIntraGetPredValDC      ( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight);

  Pel*  getPredictorPtr           ( const ComponentID compID, const Bool bUseFilteredPredictions )
  {
    return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED];
  }

  // This function is actually still in TComPattern.cpp
  /// set parameters from CU data for accessing intra data
  Void initIntraPatternChType ( TComTU &rTu,
                              const ComponentID compID, const Bool bFilterRefSamples
                              DEBUG_STRING_FN_DECLARE(sDebug)
                              );

  static Bool filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled);

  static Bool UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode);
  Void  derivePLTLossy(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost );
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  Void  derivePLTLossyIterative( TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost);
  UInt  findCandidatePLTPredictors(UInt pltIndBest[], TComDataCU* pcCU, Pel *Palette[3], Pel* pPred[3], UInt uiPLTSizeTemp, UInt maxNoPredInd);
#endif
  Void  derivePLTLossyForcePrediction(TComDataCU *pcCU, Pel *Palette[3], Pel *pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost);
  Void  derivePLTLossless(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, Bool forcePLTPrediction);
  Bool  calLeftRun(TComDataCU* pcCU, Pel* pValue, UChar * pSPoint, UInt uiStartPos, UInt uiTotal, UInt &uiRun, UChar* pEscapeFlag);
  Bool  calAboveRun(TComDataCU* pcCU, Pel* pValue, UChar * pSPoint, UInt uiWidth, UInt uiStartPos, UInt uiTotal, UInt &uiRun, UChar* pEscapeFlag);
  Void  calcPixelPred(TComDataCU* pcCU, Pel* pOrg [3], Pel *pPalette[3], Pel* pValue, Pel*paPixelValue[3], Pel*paRecoValue[3],
                      UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiStartPos );
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  Double calcPixelPredRD(TComDataCU* pcCU, Pel pOrg[3], TComRdCost *pcCost, UInt *error);
  UInt getTruncBinBits(UInt uiSymbol, UInt uiMaxSymbol);
#if SCM_U0052_ESCAPE_PIXEL_CODING
  UInt getEpExGolombNumBins(UInt uiSymbol, UInt uiCount);
#endif
#endif
  Void  preCalcPLTIndex(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize);
#if SCM_U0096_PLT_ENCODER_IMPROVEMENT
  Void preCalcPLTIndexRD(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize, TComRdCost *pcCost, UInt calcErrorBits);
#endif

  Void  reorderPLT(TComDataCU* pcCU, Pel *Palette[3], UInt uiNumComp);
  Void  setPLTErrLimit ( Int iPLTErrLimit ) {  m_iPLTErrLimit = iPLTErrLimit;  }
  Int   getPLTErrLimit () {return m_iPLTErrLimit;}
  Void  rotationScan( Pel* pLevel, UInt uiWidth, UInt uiHeight, Bool isInverse );
};

//! \}

#endif // __TCOMPREDICTION__
