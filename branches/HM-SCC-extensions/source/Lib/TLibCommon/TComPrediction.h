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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComPic.h"
#include "TComMotionInfo.h"
#include "TComPattern.h"
#include "TComTrQuant.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"

class TComTU; // forward declaration

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

static const UInt MAX_INTRA_FILTER_DEPTHS=5; // NOTE: RExt - new definition

class TComPrediction : public TComWeightPrediction
{
private:
  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

protected:
#if PALETTE_MODE
  Int        m_iPLTErrLimit;
  Pel        m_cIndexBlock[MAX_CU_SIZE * MAX_CU_SIZE];
  Pel        m_cIndexBlockChroma[MAX_CU_SIZE * MAX_CU_SIZE];
#if PLT_IDX_ADAPT_SCAN
  UInt*     m_puiScanOrder;
#endif
#endif
  Pel*      m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int       m_iYuvExtSize;

  TComYuv   m_acYuvPred[NUM_REF_PIC_LIST_01];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComYuv m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];

  TComInterpolationFilter m_if;

  Pel*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

  Void xPredIntraAng            ( Int bitDepth, const Pel* pSrc, Int srcStride, Pel* pDst, Int dstStride, UInt width, UInt height, ChannelType channelType, ChromaFormat format, UInt dirMode, Bool blkAboveAvailable, Bool blkLeftAvailable, const Bool bEnableEdgeFilters );
  Void xPredIntraPlanar         ( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height, ChannelType channelType, ChromaFormat format );

  // motion compensation functions
  Void xPredInterUni            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv*& rpcYuvPred, Bool bi=false          );
  Void xPredInterBi             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv*& rpcYuvPred          );
  Void xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic, Bool bi );
  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv*& rpcYuvDst );

  Void xPredIntraBCBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *&dstPic);

  Void xGetLLSPrediction ( const Pel* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0, const ChromaFormat chFmt  DEBUG_STRING_FN_DECLARE(sDebug) );

  Void xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType );
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
  Void destroy();

public:
  TComPrediction();
  virtual ~TComPrediction();

  Void    initTempBuff(ChromaFormat chromaFormatIDC);

  ChromaFormat getChromaFormat() const { return m_cYuvPredTemp.getChromaFormat(); }

  // inter
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );

  Void intraBlockCopy    ( TComDataCU*  pcCU, TComYuv* pcYuvPred, Int iPartIdx = -1 );

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );

  // Angular Intra
  Void predIntraAng               ( const ComponentID compID, UInt uiDirMode, Pel *piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, Bool bAbove, Bool bLeft, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM = false );

  Pel  predIntraGetPredValDC      ( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight, ChannelType channelType, ChromaFormat format, Bool bAbove, Bool bLeft );

  Pel*  getPredictorPtr           ( const ComponentID compID, const Bool bUseFilteredPredictions )
  {
    return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED];
  }

  // This function is actually still in TComPattern.cpp
  /// set parameters from CU data for accessing ADI data
  Void initAdiPatternChType ( TComTU &rTu,
                              Bool&       bAbove,
                              Bool&       bLeft,
                              const ComponentID compID, const Bool bFilterRefSamples
                              DEBUG_STRING_FN_DECLARE(sDebug)
                              );

  static Bool filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled);

  static Bool UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode);
#if PALETTE_MODE
#if CANON_PALETTE_ENCODER
  Void  derivePLT( TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize, TComRdCost *pcCost );
#endif
  Void  derivePLT   (TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc [3],  UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize);
  Void  derivePLTLuma(TComDataCU* pcCU, Pel *Palette, Pel* pSrc, UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize);
  Void  derivePLTChroma(TComDataCU* pcCU, Pel *Palette[2], Pel* pSrc[2], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize);
  Void  deriveRun (TComDataCU* pcCU, Pel* pOrg [3],  Pel *pPalette [3],  Pel* pValue, Pel* pSPoint, Pel * pEscapeFlag, Pel* pPixelPredFlag,  Pel *pRecoValue[], Pel *pPixelRec[], Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize);
  Bool  CalRun(Pel* pValue, Pel * pEscapeFlag, UInt uiStartPos, UInt uiTotal, UInt &uiRun, Bool bChroma = false);
  Bool  CalCopy(Pel* pValue, Pel *pEscapeFlag, UInt uiWidth, UInt uiStartPos, UInt uiTotal, UInt &uiRun, Bool bChroma = false);

  Bool  CalcPixelPred(TComDataCU* pcCU, Pel* pOrg [3], Pel *pPalette[3], Pel* pValue, Pel * pEscapeFlag,  Pel*paPixelValue[3], Pel*paRecoValue[3], UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiStartPos );
  Void  deriveRunLuma (TComDataCU* pcCU, Pel* pOrg ,  Pel *pPalette ,  Pel* pValue, Pel* pSPoint, Pel * pEscapeFlag, Pel* pPixelPredFlag,  Pel *pPixelRec, Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize);
  Void  deriveRunChroma (TComDataCU* pcCU, Pel* pOrg [2],  Pel *pPalette [2],  Pel* pValue[2], Pel* pSPoint, Pel * pEscapeFlag, Pel* pPixelPredFlag, Pel *pPixelRec[2], Pel* pRun, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiPLTSize);  
  Bool  CalcPixelPredLuma(TComDataCU* pcCU, Pel* pOrg, Pel *pPalette, Pel* pValue, Pel * pEscapeFlag,   Pel*paRecoValue, UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiStartPos );
  Bool  CalcPixelPredChroma(TComDataCU* pcCU, Pel* pOrg [2], Pel *pPalette[2], Pel* pValue[2], Pel * pEscapeFlag,   Pel*paRecoValue[2], UInt uiWidth, UInt uiHeight,  UInt uiStrideOrg, UInt uiStartPos );

  Void  preCalcPLTIndexLuma(TComDataCU* pcCU, Pel *Palette, Pel* pSrc, UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize);
  Void  preCalcPLTIndexChroma(TComDataCU* pcCU, Pel *Palette[2], Pel* pSrc[2], UInt uiWidth, UInt uiHeight, UInt uiStride, UInt &uiPLTSize);
  Void  preCalcPLTIndex(TComDataCU* pcCU, Pel *Palette[3], Pel* pSrc[3], UInt uiWidth, UInt uiHeight, UInt uiPLTSize);

  Void  reorderPLT(TComDataCU* pcCU, Pel *Palette[3], UInt uiNumComp);
  Void  setPLTErrLimit ( Int iPLTErrLimit ) {  m_iPLTErrLimit = iPLTErrLimit;  }
  Int   getPLTErrLimit ( ) {return m_iPLTErrLimit;}
#if PLT_IDX_ADAPT_SCAN
  Void   rotationScan                ( Pel* pLevel, UInt uiWidth, UInt uiHeight, Bool isInverse );
#endif
  inline UInt getIdxScanPos( UInt uiIdx );
#if PLT_IDX_ADAPT_SCAN
  Pel *getIndexBlock(Int bChroma) { return (bChroma?  m_cIndexBlockChroma : m_cIndexBlock);};
#endif
#endif
};

//! \}

#endif // __TCOMPREDICTION__
