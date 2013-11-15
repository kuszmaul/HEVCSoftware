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

/** \file     TComSampleAdaptiveOffset.h
    \brief    sample adaptive offset class (header)
*/

#ifndef __TCOMSAMPLEADAPTIVEOFFSET__
#define __TCOMSAMPLEADAPTIVEOFFSET__

#include "CommonDef.h"
#include "TComPic.h"

//! \ingroup TLibCommon
//! \{


#if HM_CLEANUP_SAO
// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_SAO_TRUNCATED_BITDEPTH     10 

// ====================================================================================================================
// Class definition
// ====================================================================================================================
extern UInt g_saoMaxOffsetQVal[MAX_NUM_COMPONENT]; 

class TComSampleAdaptiveOffset
{
public:
  TComSampleAdaptiveOffset();
  virtual ~TComSampleAdaptiveOffset();
  Void SAOProcess(TComPic* pDecPic);
  Void create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth );
  Void destroy();
  Void reconstructBlkSAOParams(TComPic* pic, SAOBlkParam* saoBlkParams);
  Void PCMLFDisableProcess (TComPic* pcPic);
protected:
  Void offsetBlock(ComponentID compIdx, Int typeIdx, Int* offset, Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                  , Bool isLeftAvail, Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail);
  Void invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets);
  Void reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  Int  getMergeList(TComPic* pic, Int ctu, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  Void offsetCTU(Int ctu, TComPicYuv* srcYuv, TComPicYuv* resYuv, SAOBlkParam& saoblkParam, TComPic* pPic);
  Void xPCMRestoration(TComPic* pcPic);
  Void xPCMCURestoration ( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth );
  Void xPCMSampleRestoration (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, ComponentID component);
protected:
  UInt m_offsetStepLog2[MAX_NUM_COMPONENT]; //offset step  
  Int* m_offsetClip[MAX_NUM_COMPONENT]; //clip table for fast operation
  Short* m_sign; //sign table for fast operation
  TComPicYuv*   m_tempPicYuv; //temporary buffer
  Int m_picWidth;
  Int m_picHeight;
  Int m_maxCUWidth;
  Int m_maxCUHeight;
  Int m_numCTUInWidth;
  Int m_numCTUInHeight;
  Int m_numCTUsPic;
  
  
  Int m_lineBufWidth;
  Char* m_signLineBuf1;
  Char* m_signLineBuf2;
  ChromaFormat m_chromaFormatIDC;
private:
  Bool m_picSAOEnabled[MAX_NUM_COMPONENT];
  Int*   m_offsetClipTable[MAX_NUM_COMPONENT];
  Short* m_signTable;
};
#else

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define SAO_MAX_DEPTH                 4
#define SAO_BO_BITS                   5
#define SAO_EO_TABLE_SIZE             9
#define LUMA_GROUP_NUM                (1<<SAO_BO_BITS)
#define MAX_NUM_SAO_CLASS             33

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Sample Adaptive Offset class
class TComSampleAdaptiveOffset
{
protected:
  TComPic*          m_pcPic;

  static const UInt m_uiMaxDepth;
  static const Int m_aiNumCulPartsLevel[SAO_MAX_DEPTH + 1];
  static const UInt m_auiEoTable[SAO_EO_TABLE_SIZE]; //NOTE: RExt - This table appears to be larger than needed.
  Int *m_aiOffsetBo[MAX_NUM_CHANNEL_TYPE];
  Int  m_iOffsetEo[LUMA_GROUP_NUM];                  //NOTE: RExt - This table appears to be larger than needed.
  Int  m_iPicWidth;
  Int  m_iPicHeight;
  UInt m_uiMaxSplitLevel;
  UInt m_uiMaxCUWidth;
  UInt m_uiMaxCUHeight;
  Int  m_iNumCuInWidth;
  Int  m_iNumCuInHeight;
  Int  m_iNumTotalParts;
  static const Int m_iNumClass[MAX_NUM_SAO_TYPE];

  UInt m_auiSaoBitIncrease[MAX_NUM_CHANNEL_TYPE];
  UInt m_uiQP;

  Pel   *m_apClipTable[MAX_NUM_CHANNEL_TYPE];
  Pel   *m_apClipTableBase[MAX_NUM_CHANNEL_TYPE];
  Pel   *m_aTableBo[MAX_NUM_CHANNEL_TYPE];
  Int   *m_iUpBuff1;
  Int   *m_iUpBuff2;
  Int   *m_iUpBufft;
  Int   *ipSwap;
  Bool  m_bUseNIF;       //!< true for performing non-cross slice boundary ALF
  TComPicYuv* m_pcYuvTmp;    //!< temporary picture buffer pointer when non-across slice/tile boundary SAO is enabled

  Pel* m_pTmpU1;
  Pel* m_pTmpU2;
  Pel* m_pTmpL1;
  Pel* m_pTmpL2;
  Int     m_maxNumOffsetsPerPic;
  Bool    m_saoLcuBoundary;
  Bool    m_saoLcuBasedOptimization;

  Void xPCMRestoration        (TComPic* pcPic);
  Void xPCMCURestoration      (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth);
  Void xPCMSampleRestoration  (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, const ComponentID compID);
public:
  TComSampleAdaptiveOffset         ();
  virtual ~TComSampleAdaptiveOffset();

  Void create( UInt uiSourceWidth, UInt uiSourceHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight );
  Void destroy ();

  Int  convertLevelRowCol2Idx(Int level, Int row, Int col);

  Void initSAOParam   (SAOParam *pcSaoParam, Int iPartLevel, Int iPartRow, Int iPartCol, Int iParentPartIdx, Int StartCUX, Int EndCUX, Int StartCUY, Int EndCUY, ComponentID ch);
  Void allocSaoParam  (SAOParam* pcSaoParam);
  Void resetSAOParam  (SAOParam *pcSaoParam);
  static Void freeSaoParam   (SAOParam *pcSaoParam);

  Void SAOProcess(SAOParam* pcSaoParam);
  Void processSaoCu(Int iAddr, Int iSaoType, ComponentID ch);
  Pel* getPicYuvAddr(TComPicYuv* pcPicYuv, ComponentID ch,Int iAddr = 0) { return pcPicYuv->getAddr(ch, iAddr); }


  Void processSaoCuOrg(Int iAddr, Int iPartIdx, ComponentID ch);  //!< LCU-basd SAO process without slice granularity
  Void createPicSaoInfo(TComPic* pcPic);
  Void destroyPicSaoInfo();
  Void processSaoBlock(Pel* pDec, Pel* pRest, Int stride, Int iSaoType, UInt width, UInt height, Bool* pbBorderAvail, ComponentID iYCbCr);

  Void resetLcuPart(SaoLcuParam* saoLcuParam);
  Void convertQT2SaoUnit(SAOParam* saoParam, UInt partIdx, ComponentID ch);
  Void convertOnePart2SaoUnit(SAOParam *saoParam, UInt partIdx, ComponentID ch);
  Void processSaoUnitAll(SaoLcuParam* saoLcuParam, Bool oneUnitFlag, ComponentID ch);
  Void setSaoLcuBoundary (Bool bVal)  {m_saoLcuBoundary = bVal;}
  Bool getSaoLcuBoundary ()           {return m_saoLcuBoundary;}
  Void setSaoLcuBasedOptimization (Bool bVal)  {m_saoLcuBasedOptimization = bVal;}
  Bool getSaoLcuBasedOptimization ()           {return m_saoLcuBasedOptimization;}

  Void resetSaoUnit(SaoLcuParam* saoUnit);
  Void copySaoUnit(SaoLcuParam* saoUnitDst, SaoLcuParam* saoUnitSrc );
  Void PCMLFDisableProcess    ( TComPic* pcPic);                        ///< interface function for ALF process
};

#endif

//! \}
#endif

