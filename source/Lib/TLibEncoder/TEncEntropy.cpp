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

/** \file     TEncEntropy.cpp
    \brief    entropy encoder class
*/

#include "TEncEntropy.h"
#include "TLibCommon/TypeDef.h"
#include "TLibCommon/TComAdaptiveLoopFilter.h"
#include "TLibCommon/TComSampleAdaptiveOffset.h"
#include "TLibCommon/TComTU.h"

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
static const Bool bDebugPredEnabled = DebugOptionList::DebugPred.getInt()!=0;
#endif

//! \ingroup TLibEncoder
//! \{

Void TEncEntropy::setEntropyCoder ( TEncEntropyIf* e, TComSlice* pcSlice )
{
  m_pcEntropyCoderIf = e;
  m_pcEntropyCoderIf->setSlice ( pcSlice );
}

Void TEncEntropy::encodeSliceHeader ( TComSlice* pcSlice )
{
  if (pcSlice->getSPS()->getUseSAO())
  {
#if REMOVE_APS
    SAOParam *saoParam = pcSlice->getPic()->getPicSym()->getSaoParam();
#else
    SAOParam *saoParam = pcSlice->getAPS()->getSaoParam();
#endif
#if SAO_TYPE_SHARING
    pcSlice->setSaoEnabledFlag     (saoParam->bSaoFlag[CHANNEL_TYPE_LUMA]);
#else
    pcSlice->setSaoEnabledFlag     (saoParam->bSaoFlag[COMPONENT_Y]);
#endif
#if !SAO_LUM_CHROMA_ONOFF_FLAGS
    if (pcSlice->getSaoEnabledFlag())
#endif
    {
#if SAO_TYPE_SHARING
      pcSlice->setSaoEnabledFlagChroma   (saoParam->bSaoFlag[CHANNEL_TYPE_CHROMA]);
#else
      pcSlice->setSaoEnabledFlagCb   (saoParam->bSaoFlag[COMPONENT_Cb]);
      pcSlice->setSaoEnabledFlagCr   (saoParam->bSaoFlag[COMPONENT_Cr]);
#endif
    }
#if !SAO_LUM_CHROMA_ONOFF_FLAGS
    else
    {
#if SAO_TYPE_SHARING
      pcSlice->setSaoEnabledFlagChroma (0);
#else
      pcSlice->setSaoEnabledFlagCb   (0);
      pcSlice->setSaoEnabledFlagCr   (0);
#endif
    }
#endif
  }

  m_pcEntropyCoderIf->codeSliceHeader( pcSlice );
  return;
}

Void  TEncEntropy::encodeTilesWPPEntryPoint( TComSlice* pSlice )
{
  m_pcEntropyCoderIf->codeTilesWPPEntryPoint( pSlice );
}

Void TEncEntropy::encodeTerminatingBit      ( UInt uiIsLast )
{
  m_pcEntropyCoderIf->codeTerminatingBit( uiIsLast );
  
  return;
}

Void TEncEntropy::encodeSliceFinish()
{
  m_pcEntropyCoderIf->codeSliceFinish();
}

Void TEncEntropy::encodeFlush()
{
  m_pcEntropyCoderIf->codeFlush();
}
Void TEncEntropy::encodeStart()
{
  m_pcEntropyCoderIf->encodeStart();
}

Void TEncEntropy::encodeSEI(const SEI& sei)
{
  m_pcEntropyCoderIf->codeSEI(sei);
  return;
}

Void TEncEntropy::encodePPS( TComPPS* pcPPS )
{
  m_pcEntropyCoderIf->codePPS( pcPPS );
  return;
}

Void TEncEntropy::encodeSPS( TComSPS* pcSPS )
{
  m_pcEntropyCoderIf->codeSPS( pcSPS );
  return;
}

Void TEncEntropy::encodeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  else if( pcCU->getLastCUSucIPCMFlag() && pcCU->getIPCMFlag(uiAbsPartIdx) )
  {
    return;
  }
  m_pcEntropyCoderIf->codeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
}

Void TEncEntropy::encodeVPS( TComVPS* pcVPS )
{
  m_pcEntropyCoderIf->codeVPS( pcVPS );
  return;
}

Void TEncEntropy::encodeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  if( !bRD )
  {
    if( pcCU->getLastCUSucIPCMFlag() && pcCU->getIPCMFlag(uiAbsPartIdx) )
    {
      return;
    }
  }
  m_pcEntropyCoderIf->codeSkipFlag( pcCU, uiAbsPartIdx );
}

/** encode merge flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiPUIdx
 * \returns Void
 */
Void TEncEntropy::encodeMergeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPUIdx )
{ 
  // at least one merge candidate exists
  m_pcEntropyCoderIf->codeMergeFlag( pcCU, uiAbsPartIdx );
}

/** encode merge index
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiPUIdx
 * \param bRD
 * \returns Void
 */
Void TEncEntropy::encodeMergeIndex( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPUIdx, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
    assert( pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N );
  }

  UInt uiNumCand = MRG_MAX_NUM_CANDS;
  if ( uiNumCand > 1 )
  {
    m_pcEntropyCoderIf->codeMergeIndex( pcCU, uiAbsPartIdx );
  }
}


/** encode prediction mode
 * \param pcCU
 * \param uiAbsPartIdx
 * \param bRD
 * \returns Void
 */
Void TEncEntropy::encodePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  if( !bRD )
  {
    if( pcCU->getLastCUSucIPCMFlag() && pcCU->getIPCMFlag(uiAbsPartIdx) )
    {
      return;
    }
  }

  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  m_pcEntropyCoderIf->codePredMode( pcCU, uiAbsPartIdx );
}

// Split mode
Void TEncEntropy::encodeSplitFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  if( !bRD )
  {
    if( pcCU->getLastCUSucIPCMFlag() && pcCU->getIPCMFlag(uiAbsPartIdx) )
    {
      return;
    }
  }

  m_pcEntropyCoderIf->codeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
}

/** encode partition size
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param bRD
 * \returns Void
 */
Void TEncEntropy::encodePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  if( !bRD )
  {
    if( pcCU->getLastCUSucIPCMFlag() && pcCU->getIPCMFlag(uiAbsPartIdx) )
    {
      return;
    }
  }
  m_pcEntropyCoderIf->codePartSize( pcCU, uiAbsPartIdx, uiDepth );
}

/** Encode I_PCM information. 
 * \param pcCU pointer to CU 
 * \param uiAbsPartIdx CU index
 * \param bRD flag indicating estimation or encoding
 * \returns Void
 */
Void TEncEntropy::encodeIPCMInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if(!pcCU->getSlice()->getSPS()->getUsePCM()
    || pcCU->getWidth(uiAbsPartIdx) > (1<<pcCU->getSlice()->getSPS()->getPCMLog2MaxSize())
    || pcCU->getWidth(uiAbsPartIdx) < (1<<pcCU->getSlice()->getSPS()->getPCMLog2MinSize()))
  {
    return;
  }
  
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  
  Int numIPCM = 0;
  Bool firstIPCMFlag = false;

  if( pcCU->getIPCMFlag(uiAbsPartIdx) )
  {
    numIPCM = 1;
    firstIPCMFlag = true;

    if( !bRD )
    {
      numIPCM = pcCU->getNumSucIPCM();
      firstIPCMFlag = !pcCU->getLastCUSucIPCMFlag();
    }
  }
  m_pcEntropyCoderIf->codeIPCMInfo ( pcCU, uiAbsPartIdx, numIPCM, firstIPCMFlag);

}


Void TEncEntropy::xEncodeTransform( Bool& bCodeDQP, TComTU &rTu )
{
//pcCU, absPartIdxCU, uiAbsPartIdx, uiDepth+1, uiTrIdx+1, quadrant,
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  const UInt numValidComponent = pcCU->getPic()->getNumberValidComponents();
  const Bool bChroma = isChromaEnabled(pcCU->getPic()->getChromaFormat());
  const UInt uiTrIdx = rTu.GetTransformDepthRel();
  const UInt uiDepth = rTu.GetTransformDepthTotal();
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  const Bool bDebugRQT=g_bFinalEncode && DebugOptionList::DebugRQT.getInt()!=0;
  if (bDebugRQT)
    printf("x..codeTransform: offsetLuma=%d offsetChroma=%d absPartIdx=%d, uiDepth=%d\n width=%d, height=%d, uiTrIdx=%d, uiInnerQuadIdx=%d\n",
           rTu.getCoefficientOffset(COMPONENT_Y), rTu.getCoefficientOffset(COMPONENT_Cb), uiAbsPartIdx, uiDepth, rTu.getRect(COMPONENT_Y).width, rTu.getRect(COMPONENT_Y).height, rTu.GetTransformDepthRel(), rTu.GetSectionNumber());
#endif
  const UInt uiSubdiv = pcCU->getTransformIdx( uiAbsPartIdx ) > uiTrIdx;// + pcCU->getDepth( uiAbsPartIdx ) > uiDepth;
  const UInt uiLog2TrafoSize = rTu.GetLog2LumaTrSize();


  UInt cbf[MAX_NUM_COMPONENT]={0,0,0};
  bool bHaveACodedBlock=false;
  for(UInt ch=0; ch<numValidComponent; ch++)
  {
    cbf[ch] = pcCU->getCbf( uiAbsPartIdx, ComponentID(ch) , uiTrIdx );
    if (cbf[ch]) bHaveACodedBlock=true;
  }

  if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA && pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN && uiDepth == pcCU->getDepth(uiAbsPartIdx) )
  {
    assert( uiSubdiv );
  }
  else if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTER && (pcCU->getPartitionSize(uiAbsPartIdx) != SIZE_2Nx2N) && uiDepth == pcCU->getDepth(uiAbsPartIdx) &&  (pcCU->getSlice()->getSPS()->getQuadtreeTUMaxDepthInter() == 1) )
  {
    if ( uiLog2TrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
    {
      assert( uiSubdiv );
    }
    else
    {
      assert(!uiSubdiv );
    }
  }
  else if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2TrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
#if TRANS_SPLIT_FLAG_CTX_REDUCTION
    m_pcEntropyCoderIf->codeTransformSubdivFlag( uiSubdiv, 5 - uiLog2TrafoSize );
#else
    m_pcEntropyCoderIf->codeTransformSubdivFlag( uiSubdiv, uiDepth );
#endif
  }

  const UInt uiTrDepthCurr = uiDepth - pcCU->getDepth( uiAbsPartIdx );
  const Bool bFirstCbfOfCU = uiTrDepthCurr == 0;

  for(UInt ch=COMPONENT_Cb; ch<numValidComponent; ch++)
  {
    const ComponentID compID=ComponentID(ch);
    if( bFirstCbfOfCU || rTu.ProcessingAllQuadrants(compID) )
    {
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepthCurr - 1 ) )
      {
        m_pcEntropyCoderIf->codeQtCbf( rTu, compID );
      }
    }
    else
    {
      assert( pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepthCurr ) == pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepthCurr - 1 ) );
    }
  }
    
  if( uiSubdiv )
  {
    TComTURecurse tuRecurseChild(rTu, true);
    do
    {
      xEncodeTransform( bCodeDQP, tuRecurseChild );
    }
    while (tuRecurseChild.nextSection(rTu));
  }
  else
  {
    {
      DTRACE_CABAC_VL( g_nSymbolCounter++ );
      DTRACE_CABAC_T( "\tTrIdx: abspart=" );
      DTRACE_CABAC_V( uiAbsPartIdx );
      DTRACE_CABAC_T( "\tdepth=" );
      DTRACE_CABAC_V( uiDepth );
      DTRACE_CABAC_T( "\ttrdepth=" );
      DTRACE_CABAC_V( pcCU->getTransformIdx( uiAbsPartIdx ) );
      DTRACE_CABAC_T( "\n" );
    }
    UInt uiLumaTrMode, uiChromaTrMode;
    pcCU->convertTransIdx( uiAbsPartIdx, pcCU->getTransformIdx( uiAbsPartIdx ), uiLumaTrMode, uiChromaTrMode );
#if !REMOVE_NSQT
    if(pcCU->getPredictionMode( uiAbsPartIdx ) == MODE_INTER && pcCU->useNonSquarePU( uiAbsPartIdx ) )
    {
      pcCU->setNSQTIdxSubParts( uiLog2TrafoSize, rTu, uiLumaTrMode );
    }
#endif
    if( pcCU->getPredictionMode(uiAbsPartIdx) != MODE_INTRA && uiDepth == pcCU->getDepth( uiAbsPartIdx ) && (!bChroma || (!pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cb, 0 ) && !pcCU->getCbf( uiAbsPartIdx, COMPONENT_Cr, 0 ) ) ) )
    {
      assert( pcCU->getCbf( uiAbsPartIdx, COMPONENT_Y, 0 ) );
      //      printf( "saved one bin! " );
    }
    else
    {
      m_pcEntropyCoderIf->codeQtCbf( rTu, COMPONENT_Y );
    }
      
    if ( bHaveACodedBlock )
    {
      // dQP: only for LCU once
      if ( pcCU->getSlice()->getPPS()->getUseDQP() )
      {
        if ( bCodeDQP )
        {
          encodeQP( pcCU, rTu.GetAbsPartIdxCU() );
          bCodeDQP = false;
        }
      }
      const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();

      {
        const ComponentID compID=COMPONENT_Y;
        if (rTu.ProcessComponentSection(compID) && cbf[compID])
        {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
            if (bDebugRQT) printf("Call NxN for chan %d? width=%d cbf=%d\n", compID, rTu.getRect(compID).width, 1);
#endif
            const UInt offset=rTu.getCoefficientOffset(compID);
            m_pcEntropyCoderIf->codeCoeffNxN( rTu, (pcCU->getCoeff(compID)+offset), compID );
        }
      }

      for(UInt ch=1; ch<numValidComp; ch++)
      {
        const ComponentID compID=ComponentID(ch);
        if (rTu.ProcessComponentSection(compID) && cbf[compID])
        {
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
            if (bDebugRQT) printf("Call NxN for chan %d? width=%d cbf=%d\n", compID, rTu.getRect(compID).width, 1);
#endif
            const UInt offset=rTu.getCoefficientOffset(compID);
            m_pcEntropyCoderIf->codeCoeffNxN( rTu, (pcCU->getCoeff(compID)+offset), compID );
        }
      }
    }
  }
}


// Intra direction for Luma
Void TEncEntropy::encodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiplePU )
{
  m_pcEntropyCoderIf->codeIntraDirLumaAng( pcCU, absPartIdx , isMultiplePU);
}


// Intra direction for Chroma
Void TEncEntropy::encodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  m_pcEntropyCoderIf->codeIntraDirChroma( pcCU, uiAbsPartIdx );

#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (bDebugPredEnabled && g_bFinalEncode)
  {
    UInt cdir=pcCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiAbsPartIdx);
    if (cdir==36) cdir=pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx);
    printf("coding chroma Intra dir: %d, uiAbsPartIdx: %d, luma dir: %d\n", cdir, uiAbsPartIdx, pcCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx));
  }
#endif
}



Void TEncEntropy::encodePredInfo( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if( pcCU->isIntra( uiAbsPartIdx ) )                                 // If it is Intra mode, encode intra prediction mode.
  {
    encodeIntraDirModeLuma  ( pcCU, uiAbsPartIdx,true );
    if (pcCU->getPic()->getChromaFormat()!=CHROMA_400)
    {
      encodeIntraDirModeChroma( pcCU, uiAbsPartIdx );

      if (enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()) && pcCU->getPartitionSize( uiAbsPartIdx )==SIZE_NxN)
      {
        UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;
        encodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset   );
        encodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset*2 );
        encodeIntraDirModeChroma( pcCU, uiAbsPartIdx + uiPartOffset*3 );
      }
    }
  }
  else                                                                // if it is Inter mode, encode motion vector and reference index
  {
    encodePUWise( pcCU, uiAbsPartIdx );
  }
}

/** encode motion information for every PU block
 * \param pcCU
 * \param uiAbsPartIdx
 * \param bRD
 * \returns Void
 */
Void TEncEntropy::encodePUWise( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  const Bool bDebugPred = bDebugPredEnabled && g_bFinalEncode;
#endif
  
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
  UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
  UInt uiDepth = pcCU->getDepth( uiAbsPartIdx );
  UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() - uiDepth ) << 1 ) ) >> 4;

  for ( UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset )
  {
    encodeMergeFlag( pcCU, uiSubPartIdx, uiPartIdx );
    if ( pcCU->getMergeFlag( uiSubPartIdx ) )
    {
      encodeMergeIndex( pcCU, uiSubPartIdx, uiPartIdx );
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      if (bDebugPred)
      {
        std::cout << "Coded merge flag, CU absPartIdx: " << uiAbsPartIdx << " PU(" << uiPartIdx << ") absPartIdx: " << uiSubPartIdx;
        std::cout << " merge index: " << (UInt)pcCU->getMergeIndex(uiSubPartIdx) << std::endl;
      }
#endif
    }
    else
    {
      encodeInterDirPU( pcCU, uiSubPartIdx );
      for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
      {
        if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
        {
          encodeRefFrmIdxPU ( pcCU, uiSubPartIdx, RefPicList( uiRefListIdx ) );
          encodeMvdPU       ( pcCU, uiSubPartIdx, RefPicList( uiRefListIdx ) );
          encodeMVPIdxPU    ( pcCU, uiSubPartIdx, RefPicList( uiRefListIdx ) );
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
          if (bDebugPred)
          {
            std::cout << "refListIdx: " << uiRefListIdx << std::endl;
            std::cout << "MVD horizontal: " << pcCU->getCUMvField(RefPicList(uiRefListIdx))->getMvd( uiAbsPartIdx ).getHor() << std::endl;
            std::cout << "MVD vertical:   " << pcCU->getCUMvField(RefPicList(uiRefListIdx))->getMvd( uiAbsPartIdx ).getVer() << std::endl;
            std::cout << "MVPIdxPU: " << pcCU->getMVPIdx(RefPicList( uiRefListIdx ), uiSubPartIdx) << std::endl;
            std::cout << "InterDir: " << (UInt)pcCU->getInterDir(uiSubPartIdx) << std::endl;
          }
#endif
        }
      }
    }
  }

  return;
}

Void TEncEntropy::encodeInterDirPU( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  if ( !pcCU->getSlice()->isInterB() )
  {
    return;
  }

  m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );

  return;
}

/** encode reference frame index for a PU block
 * \param pcCU
 * \param uiAbsPartIdx
 * \param eRefList
 * \returns Void
 */
Void TEncEntropy::encodeRefFrmIdxPU( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  assert( !pcCU->isIntra( uiAbsPartIdx ) );

  if ( ( pcCU->getSlice()->getNumRefIdx( eRefList ) == 1 ) )
  {
    return;
  }

  if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
  {
    m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
  }

  return;
}

/** encode motion vector difference for a PU block
 * \param pcCU
 * \param uiAbsPartIdx
 * \param eRefList
 * \returns Void
 */
Void TEncEntropy::encodeMvdPU( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  assert( !pcCU->isIntra( uiAbsPartIdx ) );

  if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
  {
    m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
  }
  return;
}

Void TEncEntropy::encodeMVPIdxPU( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList )
{
  if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
  {
    m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
  }

  return;
}

Void TEncEntropy::encodeQtCbf( TComTU &rTu, const ComponentID compID )
{
  m_pcEntropyCoderIf->codeQtCbf( rTu, compID );
}

Void TEncEntropy::encodeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx )
{
  m_pcEntropyCoderIf->codeTransformSubdivFlag( uiSymbol, uiCtx );
}

Void TEncEntropy::encodeQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  m_pcEntropyCoderIf->codeQtRootCbf( pcCU, uiAbsPartIdx );
}

#if TU_ZERO_CBF_RDO
Void TEncEntropy::encodeQtCbfZero( TComTU &rTu, const ChannelType chType, const Bool useAdjustedDepth )
{
  //NOTE: ECF - In HM8.0, this function is called in multiple ways, which may not be intended.
  //      In xEstimateResidualQT, when coding the chroma channel, it is called with both TrDepth and TrDepthC (adjusted for step-up cases).
  //      In other places, it is called with TrDepth, but only when there is no step-up case (luma-equivalent size > 4)
  m_pcEntropyCoderIf->codeQtCbfZero( rTu, chType, useAdjustedDepth );
}

Void TEncEntropy::encodeQtRootCbfZero( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  m_pcEntropyCoderIf->codeQtRootCbfZero( pcCU, uiAbsPartIdx );
}
#endif

// dQP
Void TEncEntropy::encodeQP( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
  {
    uiAbsPartIdx = 0;
  }
  
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    m_pcEntropyCoderIf->codeDeltaQP( pcCU, uiAbsPartIdx );
  }
}


// texture

/** encode coefficients
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiWidth
 * \param uiHeight
 */
Void TEncEntropy::encodeCoeff( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool& bCodeDQP )
{
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  const Bool bDebugRQT=g_bFinalEncode && DebugOptionList::DebugRQT.getInt()!=0;
#endif

  UInt uiLumaTrMode, uiChromaTrMode;
  pcCU->convertTransIdx( uiAbsPartIdx, pcCU->getTransformIdx(uiAbsPartIdx), uiLumaTrMode, uiChromaTrMode );
  
  if( pcCU->isIntra(uiAbsPartIdx) )
  {
    if (false)
    {
      DTRACE_CABAC_VL( g_nSymbolCounter++ )
      DTRACE_CABAC_T( "\tdecodeTransformIdx()\tCUDepth=" )
      DTRACE_CABAC_V( uiDepth )
      DTRACE_CABAC_T( "\n" )
    }
  }
  else
  {
    if( !(pcCU->getMergeFlag( uiAbsPartIdx ) && pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N ) )
    {
      m_pcEntropyCoderIf->codeQtRootCbf( pcCU, uiAbsPartIdx );
    }
    if ( !pcCU->getQtRootCbf( uiAbsPartIdx ) )
    {
#if !REMOVE_NSQT
      pcCU->setNSQTIdxSubParts( uiAbsPartIdx, uiDepth );
#endif
      return;
    }
  }
  
  TComTURecurse tuRecurse(pcCU, uiAbsPartIdx, uiDepth);
#if ECF__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  if (bDebugRQT) printf("..codeCoeff: uiAbsPartIdx=%d, PU format=%d, 2Nx2N=%d, NxN=%d\n", uiAbsPartIdx, pcCU->getPartitionSize(uiAbsPartIdx), SIZE_2Nx2N, SIZE_NxN);
#endif

  xEncodeTransform( bCodeDQP, tuRecurse );
}

Void TEncEntropy::encodeCoeffNxN( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID)
{
  // This is for Transform unit processing. This may be used at mode selection stage for Inter.
  m_pcEntropyCoderIf->codeCoeffNxN( rTu, pcCoef, compID);
}

Void TEncEntropy::estimateBit (estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, const ChannelType chType)
{  
  m_pcEntropyCoderIf->estBit ( pcEstBitsSbac, width, height, chType );
}

/** Encode SAO Offset
 * \param  saoLcuParam SAO LCU paramters
 */
#if SAO_TYPE_SHARING 
Void TEncEntropy::encodeSaoOffset(SaoLcuParam* saoLcuParam, const ComponentID compID )
#else
Void TEncEntropy::encodeSaoOffset(SaoLcuParam* saoLcuParam)
#endif
{
  UInt uiSymbol;
  Int i;

  uiSymbol = saoLcuParam->typeIdx + 1;
#if SAO_TYPE_SHARING
  if (compID != COMPONENT_Cr)
  {
    m_pcEntropyCoderIf->codeSaoTypeIdx(uiSymbol);
  }
#else
  m_pcEntropyCoderIf->codeSaoTypeIdx(uiSymbol);
#endif
  if (uiSymbol)
  {
#if SAO_TYPE_CODING
#if SAO_TYPE_SHARING
    if (saoLcuParam->typeIdx < 4 && compID != COMPONENT_Cr)
#else
    if (saoLcuParam->typeIdx < 4)
#endif
    {
      saoLcuParam->subTypeIdx = saoLcuParam->typeIdx;
    }
#endif
#if FULL_NBIT
    Int offsetTh = 1 << ( min((Int)(g_uiBitDepth + (g_uiBitDepth-8)-5),5) );
#else
    Int offsetTh = 1 << ( min((Int)(g_uiBitDepth + g_uiBitIncrement-5),5) );
#endif
    if( saoLcuParam->typeIdx == SAO_BO )
    {
#if !SAO_TYPE_CODING
      // Code Left Band Index
      uiSymbol = (UInt) (saoLcuParam->bandPosition);
      m_pcEntropyCoderIf->codeSaoUflc(uiSymbol);
#endif
      for( i=0; i< saoLcuParam->length; i++)
      {
        UInt absOffset = ( (saoLcuParam->offset[i] < 0) ? -saoLcuParam->offset[i] : saoLcuParam->offset[i]);
        m_pcEntropyCoderIf->codeSaoMaxUvlc(absOffset, offsetTh-1);
      }  
      for( i=0; i< saoLcuParam->length; i++)
      {
        if (saoLcuParam->offset[i] != 0)
        {
          UInt sign = (saoLcuParam->offset[i] < 0) ? 1 : 0 ;
          m_pcEntropyCoderIf->codeSAOSign(sign);
        }
      }
#if SAO_TYPE_CODING
      uiSymbol = (UInt) (saoLcuParam->subTypeIdx);
      m_pcEntropyCoderIf->codeSaoUflc(5, uiSymbol);
#endif
    }
    else if( saoLcuParam->typeIdx < 4 )
    {
      m_pcEntropyCoderIf->codeSaoMaxUvlc( saoLcuParam->offset[0], offsetTh-1);
      m_pcEntropyCoderIf->codeSaoMaxUvlc( saoLcuParam->offset[1], offsetTh-1);
      m_pcEntropyCoderIf->codeSaoMaxUvlc(-saoLcuParam->offset[2], offsetTh-1);
      m_pcEntropyCoderIf->codeSaoMaxUvlc(-saoLcuParam->offset[3], offsetTh-1);
#if SAO_TYPE_CODING
#if SAO_TYPE_SHARING
      if (compID != COMPONENT_Cr)
      {
        uiSymbol = (UInt) (saoLcuParam->subTypeIdx);
        m_pcEntropyCoderIf->codeSaoUflc(2, uiSymbol);
      }
#else
     uiSymbol = (UInt) (saoLcuParam->subTypeIdx);
     m_pcEntropyCoderIf->codeSaoUflc(2, uiSymbol);
#endif
#endif
    }
  }
}


/** Encode SAO unit interleaving
* \param  rx
* \param  ry
* \param  pSaoParam
* \param  pcCU
* \param  iCUAddrInSlice
* \param  iCUAddrUpInSlice
* \param  bLFCrossSliceBoundaryFlag
 */

Void TEncEntropy::encodeSaoUnitInterleaving(ComponentID compID, Bool saoFlag, Int rx, Int ry, SaoLcuParam* saoLcuParam, Int cuAddrInSlice, Int cuAddrUpInSlice, Int allowMergeLeft, Int allowMergeUp)
{
  if (saoFlag)
  {
    if (rx>0 && cuAddrInSlice!=0 && allowMergeLeft)
    {
#if SAO_MERGE_ONE_CTX
      m_pcEntropyCoderIf->codeSaoMerge(saoLcuParam->mergeLeftFlag);
#else
      m_pcEntropyCoderIf->codeSaoMergeLeft(saoLcuParam->mergeLeftFlag,compID);
#endif
    }
    else
    {
      saoLcuParam->mergeLeftFlag = 0;
    }
    if (saoLcuParam->mergeLeftFlag == 0)
    {
      if ( (ry > 0) && (cuAddrUpInSlice>=0) && allowMergeUp )
      {
#if SAO_MERGE_ONE_CTX
        m_pcEntropyCoderIf->codeSaoMerge(saoLcuParam->mergeUpFlag);
#else
        m_pcEntropyCoderIf->codeSaoMergeUp(saoLcuParam->mergeUpFlag);
#endif
      }
      else
      {
        saoLcuParam->mergeUpFlag = 0;
      }
      if (!saoLcuParam->mergeUpFlag)
      {
#if SAO_TYPE_SHARING 
        encodeSaoOffset(saoLcuParam, compID);
#else
        encodeSaoOffset(saoLcuParam);
#endif
      }
    }
  }
}


Int TEncEntropy::countNonZeroCoeffs( TCoeff* pcCoef, UInt uiSize )
{
  Int count = 0;
  
  for ( Int i = 0; i < uiSize; i++ )
  {
    count += pcCoef[i] != 0;
  }
  
  return count;
}

/** encode quantization matrix
 * \param scalingList quantization matrix information
 */
Void TEncEntropy::encodeScalingList( TComScalingList* scalingList )
{
  m_pcEntropyCoderIf->codeScalingList( scalingList );
}

//! \}
