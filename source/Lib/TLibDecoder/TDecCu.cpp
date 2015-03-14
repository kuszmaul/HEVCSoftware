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

/** \file     TDecCu.cpp
    \brief    CU decoder class
*/

#include "TDecCu.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/TComPrediction.h"

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCu::TDecCu()
{
  m_ppcYuvResi = NULL;
  m_ppcYuvReco = NULL;
  m_ppcCU      = NULL;
}

TDecCu::~TDecCu()
{
}

Void TDecCu::init( TDecEntropy* pcEntropyDecoder, TComTrQuant* pcTrQuant, TComPrediction* pcPrediction)
{
  m_pcEntropyDecoder  = pcEntropyDecoder;
  m_pcTrQuant         = pcTrQuant;
  m_pcPrediction      = pcPrediction;
}

/**
 \param    uiMaxDepth      total number of allowable depth
 \param    uiMaxWidth      largest CU width
 \param    uiMaxHeight     largest CU height
 \param    chromaFormatIDC chroma format
 */
Void TDecCu::create( UInt uiMaxDepth, UInt uiMaxWidth, UInt uiMaxHeight, ChromaFormat chromaFormatIDC
                    ,UInt uiPLTMaxSize, UInt uiPLTMaxPredSize
  )
{
  m_uiMaxDepth = uiMaxDepth+1;

  m_ppcYuvResi = new TComYuv*[m_uiMaxDepth-1];
  m_ppcYuvReco = new TComYuv*[m_uiMaxDepth-1];
  m_ppcCU      = new TComDataCU*[m_uiMaxDepth-1];

  UInt uiNumPartitions;
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
    uiNumPartitions = 1<<( ( m_uiMaxDepth - ui - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> ui;
    UInt uiHeight = uiMaxHeight >> ui;

    m_ppcYuvResi[ui] = new TComYuv;    m_ppcYuvResi[ui]->create( uiWidth, uiHeight, chromaFormatIDC );
    m_ppcYuvReco[ui] = new TComYuv;    m_ppcYuvReco[ui]->create( uiWidth, uiHeight, chromaFormatIDC );
    m_ppcCU     [ui] = new TComDataCU; m_ppcCU     [ui]->create( chromaFormatIDC, uiNumPartitions, uiWidth, uiHeight, true, uiMaxWidth >> (m_uiMaxDepth - 1)
      , uiPLTMaxSize, uiPLTMaxPredSize
    );
  }

  m_bDecodeDQP = false;
  m_IsChromaQpAdjCoded = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster(m_uiMaxDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );
}

Void TDecCu::destroy()
{
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
    m_ppcYuvResi[ui]->destroy(); delete m_ppcYuvResi[ui]; m_ppcYuvResi[ui] = NULL;
    m_ppcYuvReco[ui]->destroy(); delete m_ppcYuvReco[ui]; m_ppcYuvReco[ui] = NULL;
    m_ppcCU     [ui]->destroy(); delete m_ppcCU     [ui]; m_ppcCU     [ui] = NULL;
  }

  delete [] m_ppcYuvResi; m_ppcYuvResi = NULL;
  delete [] m_ppcYuvReco; m_ppcYuvReco = NULL;
  delete [] m_ppcCU     ; m_ppcCU      = NULL;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** 
 Parse a CTU.
 \param    pCtu                      [in/out] pointer to CTU data structure
 \param    isLastCtuOfSliceSegment   [out]    true, if last CTU of the slice segment
 */
Void TDecCu::decodeCtu( TComDataCU* pCtu, Bool& isLastCtuOfSliceSegment )
{
  if ( pCtu->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  if ( pCtu->getSlice()->getUseChromaQpAdj() )
  {
    setIsChromaQpAdjCoded(true);
  }

  // start from the top level CU
  xDecodeCU( pCtu, 0, 0, isLastCtuOfSliceSegment);
}

/** 
 Decoding process for a CTU.
 \param    pCtu                      [in/out] pointer to CTU data structure
 */
Void TDecCu::decompressCtu( TComDataCU* pCtu )
{
  xDecompressCU( pCtu, 0,  0 );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! decode end-of-slice flag
Bool TDecCu::xDecodeSliceEnd( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiIsLastCtuOfSliceSegment;

  if (pcCU->isLastSubCUOfCtu(uiAbsPartIdx))
  {
    m_pcEntropyDecoder->decodeTerminatingBit( uiIsLastCtuOfSliceSegment );
  }
  else
  {
    uiIsLastCtuOfSliceSegment=0;
  }

  return uiIsLastCtuOfSliceSegment>0;
}

//! decode CU block recursively
Void TDecCu::xDecodeCU( TComDataCU*const pcCU, const UInt uiAbsPartIdx, const UInt uiDepth, Bool &isLastCtuOfSliceSegment)
{
  TComPic* pcPic        = pcCU->getPic();
  const TComSPS &sps    = pcPic->getPicSym()->getSPS();
  const TComPPS &pps    = pcPic->getPicSym()->getPPS();
  const UInt maxCuWidth = sps.getMaxCUWidth();
  const UInt maxCuHeight= sps.getMaxCUHeight();
  UInt uiCurNumParts    = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts      = uiCurNumParts>>2;


  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (maxCuWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (maxCuHeight>>uiDepth) - 1;

  if( ( uiRPelX < sps.getPicWidthInLumaSamples() ) && ( uiBPelY < sps.getPicHeightInLumaSamples() ) )
  {
    m_pcEntropyDecoder->decodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else
  {
    bBoundary = true;
  }
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )
  {
    UInt uiIdx = uiAbsPartIdx;
    if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())
    {
      setdQPFlag(true);
      pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
    }

    if( uiDepth == pps.getMaxCuChromaQpAdjDepth() && pcCU->getSlice()->getUseChromaQpAdj() )
    {
      setIsChromaQpAdjCoded(true);
    }

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if ( !isLastCtuOfSliceSegment && ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )
      {
        xDecodeCU( pcCU, uiIdx, uiDepth+1, isLastCtuOfSliceSegment );
      }
      else
      {
        pcCU->setOutsideCUPart( uiIdx, uiDepth+1 );
      }

      uiIdx += uiQNumParts;
    }
    if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())
    {
      if ( getdQPFlag() )
      {
        UInt uiQPSrcPartIdx = uiAbsPartIdx;
        pcCU->setQPSubParts( pcCU->getRefQP( uiQPSrcPartIdx ), uiAbsPartIdx, uiDepth ); // set QP to default QP
      }
    }
    return;
  }

  if( uiDepth <= pps.getMaxCuDQPDepth() && pps.getUseDQP())
  {
    setdQPFlag(true);
    pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
  }

  if( uiDepth <= pps.getMaxCuChromaQpAdjDepth() && pcCU->getSlice()->getUseChromaQpAdj() )
  {
    setIsChromaQpAdjCoded(true);
  }

  if (pps.getTransquantBypassEnableFlag())
  {
    m_pcEntropyDecoder->decodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx, uiDepth );
  }

  // decode CU mode and the partition size
  if( !pcCU->getSlice()->isIntra())
  {
    m_pcEntropyDecoder->decodeSkipFlag( pcCU, uiAbsPartIdx, uiDepth );
  }


  if( pcCU->isSkipped(uiAbsPartIdx) )
  {
    m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
    m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
    TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;
    for( UInt ui = 0; ui < m_ppcCU[uiDepth]->getSlice()->getMaxNumMergeCand(); ++ui )
    {
      uhInterDirNeighbours[ui] = 0;
    }
    m_pcEntropyDecoder->decodeMergeIndex( pcCU, 0, uiAbsPartIdx, uiDepth );
    UInt uiMergeIndex = pcCU->getMergeIndex(uiAbsPartIdx);
    m_ppcCU[uiDepth]->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand, uiMergeIndex );
    pcCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiAbsPartIdx, 0, uiDepth );

    TComMv cTmpMv( 0, 0 );
    for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
    {
      if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
      {
        pcCU->setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
        pcCU->setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
        pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvd( cTmpMv, SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
        pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvField( cMvFieldNeighbours[ 2*uiMergeIndex + uiRefListIdx ], SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
#if SCM_T0227_INTRABC_SIG_UNIFICATION
        if ( uiRefListIdx == 0 && uhInterDirNeighbours[uiMergeIndex] == 1 &&
             pcCU->getSlice()->getRefPic( REF_PIC_LIST_0, cMvFieldNeighbours[uiMergeIndex<<1].getRefIdx() )->getPOC() == pcCU->getSlice()->getPOC() )
        {
          if( pcCU->getLastIntraBCMv() != cMvFieldNeighbours[uiMergeIndex<<1].getMv())
          {
            pcCU->setLastIntraBCMv( pcCU->getLastIntraBCMv(0), 1 );
            pcCU->setLastIntraBCMv( cMvFieldNeighbours[uiMergeIndex<<1].getMv() );
          }
        }
#endif
      }
    }
    xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
    return;
  }

#if !SCM_T0227_INTRABC_SIG_UNIFICATION
  if (pcCU->getSlice()->getSPS()->getUseIntraBlockCopy())
  {
    m_pcEntropyDecoder->decodeIntraBCFlag( pcCU, uiAbsPartIdx, 0, uiDepth );
  }

  if ( pcCU->isIntraBC( uiAbsPartIdx ) )
  {
    pcCU->setSizeSubParts( pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth, pcCU->getSlice()->getSPS()->getMaxCUHeight()>>uiDepth, uiAbsPartIdx, uiDepth );
    m_pcEntropyDecoder->decodePartSizeIntraBC( pcCU, uiAbsPartIdx, uiDepth );

    const PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
    const UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxTotalCUDepth() - uiDepth ) << 1 ) ) >> 4;
    const UInt iNumPart = pcCU->getNumPartitions( uiAbsPartIdx );

    UInt tempOffset = uiAbsPartIdx;
    for( UInt iPartIdx = 0; iPartIdx < iNumPart ; ++iPartIdx )
    {
      m_pcEntropyDecoder->decodeIntraBC( pcCU, tempOffset, iPartIdx, uiDepth );
      tempOffset += uiPUOffset;
    }
  }
  else
  {
#endif
    m_pcEntropyDecoder->decodePredMode( pcCU, uiAbsPartIdx, uiDepth );

    if (pcCU->getPLTModeFlag(uiAbsPartIdx) )
    {
      Bool bCodeDQP = getdQPFlag();
      setdQPFlag(bCodeDQP);
      xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
      return;
    }

    m_pcEntropyDecoder->decodePartSize( pcCU, uiAbsPartIdx, uiDepth );

    if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
    {
      m_pcEntropyDecoder->decodeIPCMInfo( pcCU, uiAbsPartIdx, uiDepth );

      if(pcCU->getIPCMFlag(uiAbsPartIdx))
      {
        xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
        return;
      }
    }

    // prediction mode ( Intra : direction mode, Inter : Mv, reference idx )
    m_pcEntropyDecoder->decodePredInfo( pcCU, uiAbsPartIdx, uiDepth, m_ppcCU[uiDepth]);
#if !SCM_T0227_INTRABC_SIG_UNIFICATION
  }
#endif

  // Coefficient decoding
  Bool bCodeDQP = getdQPFlag();
  Bool isChromaQpAdjCoded = getIsChromaQpAdjCoded();
  m_pcEntropyDecoder->decodeCoeff( pcCU, uiAbsPartIdx, uiDepth, bCodeDQP, isChromaQpAdjCoded );
  setIsChromaQpAdjCoded( isChromaQpAdjCoded );
  setdQPFlag( bCodeDQP );
  xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
}

Void TDecCu::xFinishDecodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool &isLastCtuOfSliceSegment)
{
  if(  pcCU->getSlice()->getPPS()->getUseDQP())
  {
    pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
  }

  if (pcCU->getSlice()->getUseChromaQpAdj() && !getIsChromaQpAdjCoded())
  {
    pcCU->setChromaQpAdjSubParts( pcCU->getCodedChromaQpAdj(), uiAbsPartIdx, uiDepth ); // set QP
  }

  isLastCtuOfSliceSegment = xDecodeSliceEnd( pcCU, uiAbsPartIdx );
}

Void TDecCu::xDecompressCU( TComDataCU* pCtu, UInt uiAbsPartIdx,  UInt uiDepth )
{
  TComPic* pcPic = pCtu->getPic();
  TComSlice * pcSlice = pCtu->getSlice();
  const TComSPS &sps=*(pcSlice->getSPS());

  Bool bBoundary = false;
  UInt uiLPelX   = pCtu->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (sps.getMaxCUWidth()>>uiDepth)  - 1;
  UInt uiTPelY   = pCtu->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (sps.getMaxCUHeight()>>uiDepth) - 1;

  if( ( uiRPelX >= sps.getPicWidthInLumaSamples() ) || ( uiBPelY >= sps.getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }

  if( ( ( uiDepth < pCtu->getDepth( uiAbsPartIdx ) ) && ( uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )
  {
    UInt uiNextDepth = uiDepth + 1;
    UInt uiQNumParts = pCtu->getTotalNumPart() >> (uiNextDepth<<1);
    UInt uiIdx = uiAbsPartIdx;
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++ )
    {
      uiLPelX = pCtu->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY = pCtu->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if( ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )
      {
        xDecompressCU(pCtu, uiIdx, uiNextDepth );
      }

      uiIdx += uiQNumParts;
    }
    return;
  }

  // Residual reconstruction
  m_ppcYuvResi[uiDepth]->clear();

  m_ppcCU[uiDepth]->copySubCU( pCtu, uiAbsPartIdx, uiDepth );

  switch( m_ppcCU[uiDepth]->getPredictionMode(0) )
  {
    case MODE_INTER:
      xReconInter( m_ppcCU[uiDepth], uiDepth );
      break;
    case MODE_INTRA:
      xReconIntraQT( m_ppcCU[uiDepth], uiDepth );
      break;
#if !SCM_T0227_INTRABC_SIG_UNIFICATION  
    case MODE_INTRABC:
      xReconIntraBC( m_ppcCU[uiDepth], uiDepth );
      break;
#endif
    default:
      assert(0);
      break;
  }

#ifdef DEBUG_STRING
  const PredMode predMode=m_ppcCU[uiDepth]->getPredictionMode(0);
  if (DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode))
  {
    PartSize eSize=m_ppcCU[uiDepth]->getPartitionSize(0);
    std::ostream &ss(std::cout);

    ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":(predMode==MODE_INTER?"Inter   ":"IntraBC ")) << partSizeToString[eSize] << " CU at " << m_ppcCU[uiDepth]->getCUPelX() << ", " << m_ppcCU[uiDepth]->getCUPelY() << " width=" << UInt(m_ppcCU[uiDepth]->getWidth(0)) << std::endl;
  }
#endif

  if ( m_ppcCU[uiDepth]->getPLTModeFlag(0) == false )
  {
    if ( m_ppcCU[uiDepth]->isLosslessCoded(0) && (m_ppcCU[uiDepth]->getIPCMFlag(0) == false))
    {
      xFillPCMBuffer(m_ppcCU[uiDepth], uiDepth);
    }
  }

  xCopyToPic( m_ppcCU[uiDepth], pcPic, uiAbsPartIdx, uiDepth );
}

Void TDecCu::xReconInter( TComDataCU* pcCU, UInt uiDepth )
{

  // inter prediction
  m_pcPrediction->motionCompensation( pcCU, m_ppcYuvReco[uiDepth] );

#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTER);
  if (DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-pred: ", *(m_ppcYuvReco[uiDepth]));
  }
#endif

  // inter recon
  xDecodeInterTexture( pcCU, uiDepth );

#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-resi: ", *(m_ppcYuvResi[uiDepth]));
  }
#endif

  // clip for only non-zero cbp case
  if  ( pcCU->getQtRootCbf( 0) )
  {
    m_ppcYuvReco[uiDepth]->addClip( m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], 0, pcCU->getWidth( 0 ), pcCU->getSlice()->getSPS()->getBitDepths() );
  }
  else
  {
    m_ppcYuvReco[uiDepth]->copyPartToPartYuv( m_ppcYuvReco[uiDepth],0, pcCU->getWidth( 0 ),pcCU->getHeight( 0 ));
  }
#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-reco: ", *(m_ppcYuvReco[uiDepth]));
  }
#endif

}

#if !SCM_T0227_INTRABC_SIG_UNIFICATION
Void TDecCu::xReconIntraBC( TComDataCU* pcCU, UInt uiDepth )
{
  // intra prediction
  const UInt iNumPart = pcCU->getNumPartitions();
  for( Int iPartIdx = 0 ; iPartIdx < iNumPart ; ++iPartIdx )
  {
    // Check Mv validity
    Int         iWidth;
    Int         iHeight;
    UInt        uiPartAddr;

    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    TComMv  cMv         = pcCU->getCUMvField( REF_PIC_LIST_INTRABC )->getMv( uiPartAddr );
    const UInt uiMaxCuWidth         = pcCU->getSlice()->getSPS()->getMaxCUWidth();
    const UInt uiMaxCuHeight        = pcCU->getSlice()->getSPS()->getMaxCUHeight();

    // Curr position
    Int posX         = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiPartAddr] ];
    Int posY         = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiPartAddr] ];
    // Ref Position
    Int refPosX      = posX + cMv.getHor() + iWidth - 1;
    Int refPosY      = posY + cMv.getVer() + iHeight - 1;
    // CTB Position
    Int currCTBPosX  = posX / uiMaxCuWidth;
    Int currCTBPosY  = posY / uiMaxCuHeight;
    Int refCTBPosX   = refPosX / uiMaxCuWidth;
    Int refCTBPosY   = refPosY / uiMaxCuHeight;

    const Int diffX = currCTBPosX - refCTBPosX;
    const Int diffY = currCTBPosY - refCTBPosY;

    if(  diffY < -diffX )
    {
      printf("Error Position uiDepth:%d Pos:(%d,%d) CTB_Curr:(%d,%d) mv:(%d,%d) CTB_REF:(%d,%d) \n", uiDepth, posX, posY, currCTBPosX, currCTBPosY, cMv.getHor(), cMv.getVer(), refCTBPosX, refCTBPosY);
      fflush(stdout);
      assert(0);
    }
    m_pcPrediction->intraBlockCopy( pcCU, m_ppcYuvReco[uiDepth], iPartIdx );
  }

#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRABC);
  if (DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask) printBlockToStream(std::cout, "###inter-pred: ", *(m_ppcYuvReco[uiDepth]));
#endif

  // texture recon
  xDecodeInterTexture( pcCU, uiDepth );

#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask) printBlockToStream(std::cout, "###inter-resi: ", *(m_ppcYuvResi[uiDepth]));
#endif

  // clip for only non-zero cbp case
  if  ( pcCU->getQtRootCbf( 0) )
  {
    m_ppcYuvReco[uiDepth]->addClip( m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], 0, pcCU->getWidth( 0 ), pcCU->getSlice()->getSPS()->getBitDepths() );
  }
  else
  {
    m_ppcYuvReco[uiDepth]->copyPartToPartYuv( m_ppcYuvReco[uiDepth],0, pcCU->getWidth( 0 ),pcCU->getHeight( 0 ));
  }
#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask) printBlockToStream(std::cout, "###inter-reco: ", *(m_ppcYuvReco[uiDepth]));
#endif

}
#endif

Void
TDecCu::xIntraRecBlk(       TComYuv*    pcRecoYuv,
                            TComYuv*    pcPredYuv,
                            TComYuv*    pcResiYuv,
                      const ComponentID compID,
                            TComTU     &rTu)
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }
  const Bool       bIsLuma = isLuma(compID);


  TComDataCU *pcCU = rTu.getCU();
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  const TComRectangle &tuRect  =rTu.getRect(compID);
  const UInt uiWidth           = tuRect.width;
  const UInt uiHeight          = tuRect.height;
  const UInt uiStride          = pcRecoYuv->getStride (compID);
        Pel* piPred            = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const ChromaFormat chFmt     = rTu.GetChromaFormat();

  if (uiWidth != uiHeight)
  {
    //------------------------------------------------

    //split at current level if dividing into square sub-TUs

    TComTURecurse subTURecurse(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

    //recurse further
    do
    {
      xIntraRecBlk(pcRecoYuv, pcPredYuv, pcResiYuv, compID, subTURecurse);
    } while (subTURecurse.nextSection(rTu));

    //------------------------------------------------

    return;
  }

  const UInt uiChPredMode  = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
  const UInt partsPerMinCU = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
  const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
  const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;

  const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

#ifdef DEBUG_STRING
  std::ostream &ss(std::cout);
#endif

  assert( !pcCU->getColourTransform( 0 ));

  DEBUG_STRING_NEW(sTemp)
  m_pcPrediction->initAdiPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions  DEBUG_STRING_PASS_INTO(sTemp) );


  //===== get prediction signal =====

  m_pcPrediction->predIntraAng( compID,   uiChFinalMode, 0 /* Decoder does not have an original image */, 0, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

#ifdef DEBUG_STRING
  ss << sTemp;
#endif

  //===== inverse transform =====
  Pel*      piResi            = pcResiYuv->getAddr( compID, uiAbsPartIdx );
  TCoeff*   pcCoeff           = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);//( uiNumCoeffInc * uiAbsPartIdx );

  const QpParam cQP(*pcCU, compID);


  DEBUG_STRING_NEW(sDebug);
#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
  std::string *psDebug=(DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) ? &sDebug : 0;
#endif

  if (pcCU->getCbf(uiAbsPartIdx, compID, rTu.GetTransformDepthRel()) != 0)
  {
    m_pcTrQuant->invTransformNxN( rTu, compID, piResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO(psDebug) );
  }
  else
  {
    for (UInt y = 0; y < uiHeight; y++)
    {
      for (UInt x = 0; x < uiWidth; x++)
      {
        piResi[(y * uiStride) + x] = 0;
      }
    }
  }

#ifdef DEBUG_STRING
  if (psDebug)
  {
    ss << (*psDebug);
  }
#endif

  //===== reconstruction =====
  const UInt uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride(compID);

  const Bool useCrossComponentPrediction = isChroma(compID) && (pcCU->getCrossComponentPredictionAlpha(uiAbsPartIdx, compID) != 0);
  const Pel* pResiLuma  = pcResiYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
  const Int  strideLuma = pcResiYuv->getStride( COMPONENT_Y );

        Pel* pPred      = piPred;
        Pel* pResi      = piResi;
        Pel* pReco      = pcRecoYuv->getAddr( compID, uiAbsPartIdx );
        Pel* pRecIPred  = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiAbsPartIdx );


#ifdef DEBUG_STRING
  const Bool bDebugPred=((DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  const Bool bDebugResi=((DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  const Bool bDebugReco=((DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  if (bDebugPred || bDebugResi || bDebugReco)
  {
    ss << "###: " << "CompID: " << compID << " pred mode (ch/fin): " << uiChPredMode << "/" << uiChFinalMode << " absPartIdx: " << rTu.GetAbsPartIdxTU() << std::endl;
  }
#endif

  const Int clipbd = sps.getBitDepth(toChannelType(compID));
#if O0043_BEST_EFFORT_DECODING
  const Int bitDepthDelta = sps.getStreamBitDepth(toChannelType(compID)) - clipbd;
#endif

  if( useCrossComponentPrediction )
  {
    TComTrQuant::crossComponentPrediction( rTu, compID, pResiLuma, piResi, piResi, uiWidth, uiHeight, strideLuma, uiStride, uiStride, true );
  }

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
#ifdef DEBUG_STRING
    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "###: ";
    }

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
#endif

    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
#ifdef DEBUG_STRING
      if (bDebugResi)
      {
        ss << pResi[ uiX ] << ", ";
      }
#endif
#if O0043_BEST_EFFORT_DECODING
      pReco    [ uiX ] = ClipBD( rightShiftEvenRounding<Pel>(pPred[ uiX ] + pResi[ uiX ], bitDepthDelta), clipbd );
#else
      pReco    [ uiX ] = ClipBD( pPred[ uiX ] + pResi[ uiX ], clipbd );
#endif
      pRecIPred[ uiX ] = pReco[ uiX ];
    }
#ifdef DEBUG_STRING
    if (bDebugReco)
    {
      ss << " - reco: ";
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        ss << pReco[ uiX ] << ", ";
      }
    }

    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "\n";
    }
#endif
    pPred     += uiStride;
    pResi     += uiStride;
    pReco     += uiStride;
    pRecIPred += uiRecIPredStride;
  }
}

Void
TDecCu::xIntraRecBlk( TComYuv*    pcRecoYuv,
                      TComYuv*    pcPredYuv,
                      TComYuv*    pcResiYuv,
                      TComTU     &rTu)
{
  TComDataCU         *pcCU        = rTu.getCU();
  const UInt         uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const ChromaFormat chFmt        = rTu.GetChromaFormat();
#if !SCM_T0140_ACT_QP_OFFSET
  Bool bModifyQP = !pcCU->isLosslessCoded(0) && pcCU->getColourTransform( 0 );
#endif 
#if SCM_T0132_ACT_CLIP
  const Bool             extendedPrecision = pcCU->getSlice()->getSPS()->getUseExtendedPrecision();
#endif

  for(UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    ComponentID         compID   = ComponentID(ch);
    const TComRectangle &tuRect  = rTu.getRect(compID);
    const UInt          uiWidth  = tuRect.width;
    const UInt          uiHeight = tuRect.height;
    const Bool          bIsLuma  = isLuma(compID);
    assert( uiWidth == uiHeight );

    Pel*                piPred   = pcPredYuv->getAddr( compID, uiAbsPartIdx );
    Pel*                piResi   = pcResiYuv->getAddr( compID, uiAbsPartIdx );
    TCoeff*             pcCoeff  = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);
    const UInt          uiStride = pcPredYuv->getStride( compID );

    //===== get prediction signal =====
    const TComSPS *sps = pcCU->getSlice()->getSPS(); 
    const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
    const UInt uiChPredMode  = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
    const UInt uiChFinalMode = (uiChPredMode == DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;

    Bool  bAboveAvail = false;
    Bool  bLeftAvail  = false;

    const Bool bUseFilteredPredictions = TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());
#ifdef DEBUG_STRING
    std::ostream &ss(std::cout);
#endif
    DEBUG_STRING_NEW(sTemp)
    m_pcPrediction->initAdiPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions  DEBUG_STRING_PASS_INTO(sTemp)  );
    m_pcPrediction->predIntraAng( compID,   uiChFinalMode, 0, 0, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

#ifdef DEBUG_STRING
    ss << sTemp;
#endif

    //===== inverse transform =====
    QpParam cQP(*pcCU, compID);
#if !SCM_T0140_ACT_QP_OFFSET
    if(bModifyQP)
    {
      cQP.Qp = cQP.Qp + (compID==COMPONENT_Cr? DELTA_QP_FOR_YCgCo_TRANS_V:DELTA_QP_FOR_YCgCo_TRANS);
      cQP.Qp = std::max<Int>( cQP.Qp, 0 );
      cQP.per = cQP.Qp/6;
      cQP.rem= cQP.Qp%6;
    }
#endif

    DEBUG_STRING_NEW( sDebug );
#ifdef DEBUG_STRING
    const Int debugPredModeMask=DebugStringGetPredModeMask( MODE_INTRA );
    std::string *psDebug=(DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) ? &sDebug : 0;
#endif

    if (pcCU->getCbf(uiAbsPartIdx, compID, rTu.GetTransformDepthRel()) != 0)
    {
      m_pcTrQuant->invTransformNxN( rTu, compID, piResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO(psDebug) );
    }
    else
    {
      for ( UInt y = 0; y < uiHeight; y++ )
      {
        for ( UInt x = 0; x < uiWidth; x++ )
        {
          piResi[(y * uiStride) + x] = 0;
        }
      }
    }

#ifdef DEBUG_STRING
    if (psDebug)
      ss << (*psDebug);
#endif

    const Bool useCrossComponentPrediction = isChroma(compID) && (pcCU->getCrossComponentPredictionAlpha(uiAbsPartIdx, compID) != 0);
    if( useCrossComponentPrediction ) 
    {
      const Pel* pResiLuma                      = pcResiYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
      const Int  strideLuma                     = pcResiYuv->getStride( COMPONENT_Y );
      TComTrQuant::crossComponentPrediction( rTu, compID, pResiLuma, piResi, piResi, uiWidth, uiHeight, strideLuma, uiStride, uiStride, true );
    }
  }


  if( pcCU->getColourTransform(uiAbsPartIdx) && (pcCU->getCbf(uiAbsPartIdx,COMPONENT_Y)||pcCU->getCbf(uiAbsPartIdx,COMPONENT_Cb)|| pcCU->getCbf(uiAbsPartIdx,COMPONENT_Cr)))
  {
#if SCM_T0132_ACT_CLIP
    pcResiYuv->convert(extendedPrecision, rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(uiAbsPartIdx));
#else
    pcResiYuv->convert(rTu.getRect(COMPONENT_Y).x0, rTu.getRect(COMPONENT_Y).y0, rTu.getRect(COMPONENT_Y).width, false, pcCU->getSlice()->getSPS()->getBitDepths(), pcCU->isLosslessCoded(uiAbsPartIdx));
#endif  
  }

  //===== reconstruction =====

  for(UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    ComponentID         compID           = ComponentID(ch);
    const TComRectangle &tuRect          = rTu.getRect(compID);
    const UInt          uiWidth          = tuRect.width;
    const UInt          uiHeight         = tuRect.height;
    Pel*                pPred            = pcPredYuv->getAddr( compID, uiAbsPartIdx );
    Pel*                pResi            = pcResiYuv->getAddr( compID, uiAbsPartIdx );
    Pel*                pReco            = pcRecoYuv->getAddr( compID, uiAbsPartIdx );
    Pel*                pRecIPred        = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiAbsPartIdx );
    const UInt          uiStride         = pcResiYuv->getStride( compID );
    const UInt          uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride(compID);
    const Int           clipbd           = pcCU->getSlice()->getSPS()->getBitDepths().recon[toChannelType(compID)];
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pReco    [ uiX ] = ClipBD( pPred[ uiX ] + pResi[ uiX ], clipbd );
        pRecIPred[ uiX ] = pReco[ uiX ];
      }

      pPred     += uiStride;
      pResi     += uiStride;
      pReco     += uiStride;
      pRecIPred += uiRecIPredStride;
    }
  }

}


Void
TDecCu::xReconIntraQT( TComDataCU* pcCU, UInt uiDepth )
{
  if (pcCU->getIPCMFlag(0))
  {
    xReconPCM( pcCU, uiDepth );
    return;
  }
  if (pcCU->getPLTModeFlag(0))
  {
#if SCM_T0072_T0109_T0120_PLT_NON444
    xReconPLTMode (pcCU, uiDepth);
#else
    ChromaFormat cCF = pcCU->getPic()->getSlice(0)->getSPS()->getChromaFormatIdc();
    if (cCF !=CHROMA_444)
    {
      xReconPLTModeLuma   (pcCU, uiDepth);
      xReconPLTModeChroma (pcCU, uiDepth);
    }
    else
    {
      xReconPLTMode (pcCU, uiDepth);
    }
#endif
  
    return;
  }

  if( !pcCU->getSlice()->getPPS()->getUseColourTrans () )
  {
  const UInt numChType = pcCU->getPic()->getChromaFormat()!=CHROMA_400 ? 2 : 1;
  for (UInt chType=CHANNEL_TYPE_LUMA; chType<numChType; chType++)
  {
    const ChannelType chanType=ChannelType(chType);
    const Bool NxNPUHas4Parts = ::isChroma(chanType) ? enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()) : true;
    const UInt uiInitTrDepth = ( pcCU->getPartitionSize(0) != SIZE_2Nx2N && NxNPUHas4Parts ? 1 : 0 );

    TComTURecurse tuRecurseCU(pcCU, 0);
    TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

    do
    {
      xIntraRecQT( m_ppcYuvReco[uiDepth], m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], chanType, tuRecurseWithPU );
    } while (tuRecurseWithPU.nextSection(tuRecurseCU));
  }
  }
  else
  {
    assert( pcCU->getPic()->getChromaFormat() == CHROMA_444 );
    const UInt uiInitTrDepth = ( pcCU->getPartitionSize(0) != SIZE_2Nx2N? 1 : 0 );

    TComTURecurse tuRecurseCU(pcCU, 0);
    TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

    do
    {
      xIntraRecQT( m_ppcYuvReco[uiDepth], m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], tuRecurseWithPU );
    } while (tuRecurseWithPU.nextSection(tuRecurseCU));
  }
}



/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
 * \param pcRecoYuv pointer to reconstructed sample arrays
 * \param pcPredYuv pointer to prediction sample arrays
 * \param pcResiYuv pointer to residue sample arrays
 * \param chType    texture channel type (luma/chroma)
 * \param rTu       reference to transform data
 *
 \ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
 */

Void
TDecCu::xIntraRecQT(TComYuv*    pcRecoYuv,
                    TComYuv*    pcPredYuv,
                    TComYuv*    pcResiYuv,
                    const ChannelType chType,
                    TComTU     &rTu)
{
  UInt uiTrDepth    = rTu.GetTransformDepthRel();
  TComDataCU *pcCU  = rTu.getCU();
  UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if( uiTrMode == uiTrDepth )
  {
    if (isLuma(chType))
    {
      xIntraRecBlk( pcRecoYuv, pcPredYuv, pcResiYuv, COMPONENT_Y,  rTu );
    }
    else
    {
      const UInt numValidComp=getNumberValidComponents(rTu.GetChromaFormat());
      for(UInt compID=COMPONENT_Cb; compID<numValidComp; compID++)
      {
        xIntraRecBlk( pcRecoYuv, pcPredYuv, pcResiYuv, ComponentID(compID), rTu );
      }
    }
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xIntraRecQT( pcRecoYuv, pcPredYuv, pcResiYuv, chType, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}


Void
TDecCu::xIntraRecQT(TComYuv*    pcRecoYuv,
                    TComYuv*    pcPredYuv,
                    TComYuv*    pcResiYuv, 
                    TComTU      &rTu)
{
  UInt uiTrDepth    = rTu.GetTransformDepthRel();
  TComDataCU *pcCU  = rTu.getCU();
  UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );

  if( uiTrMode == uiTrDepth ) 
    xIntraRecBlk( pcRecoYuv, pcPredYuv, pcResiYuv, rTu );
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xIntraRecQT( pcRecoYuv, pcPredYuv, pcResiYuv, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}


Void TDecCu::xCopyToPic( TComDataCU* pcCU, TComPic* pcPic, UInt uiZorderIdx, UInt uiDepth )
{
  UInt uiCtuRsAddr = pcCU->getCtuRsAddr();

  m_ppcYuvReco[uiDepth]->copyToPicYuv  ( pcPic->getPicYuvRec (), uiCtuRsAddr, uiZorderIdx );

  return;
}

Void TDecCu::xDecodeInterTexture ( TComDataCU* pcCU, UInt uiDepth )
{

  TComTURecurse tuRecur(pcCU, 0, uiDepth);
  if ( pcCU->getSlice()->getPPS()->getUseColourTrans() && pcCU->getColourTransform( 0 ) )
  {
    m_pcTrQuant->invRecurTransformACTNxN( m_ppcYuvResi[uiDepth], tuRecur );
  }
  else
  {
    for ( UInt ch=0; ch<pcCU->getPic()->getNumberValidComponents(); ch++ )
    {
      const ComponentID compID=ComponentID( ch );
      DEBUG_STRING_OUTPUT( std::cout, debug_reorder_data_token[pcCU->isIntraBC( 0 ) ? 1 : 0][compID] )

      m_pcTrQuant->invRecurTransformNxN( compID, m_ppcYuvResi[uiDepth], tuRecur );
    }
  }

  DEBUG_STRING_OUTPUT(std::cout, debug_reorder_data_token[pcCU->isIntraBC(0)?1:0][MAX_NUM_COMPONENT])
}

#if !SCM_T0072_T0109_T0120_PLT_NON444
Void TDecCu::xDecodePLTTextureLumaChroma( TComDataCU* pcCU, const UInt uiPartIdx, Pel* pPalette,  Pel* pLevel, UChar *pSPoint, Pel *pPixelValue, Pel* piReco,const UInt uiStride, const UInt uiWidth, const UInt uiHeight, const ComponentID compID, UChar* pEscapeFlag)
{
  Bool bLossless = pcCU->getCUTransquantBypass (uiPartIdx);
  Pel* piPicReco         = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiPartIdx);
  const UInt uiPicStride = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  UInt uiIdx = 0;
  Pel iValue = 0;

  const TComSPS &sps=*(pcCU->getSlice()->getSPS());
  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      uiIdx = uiY * uiWidth + uiX;
      if (pEscapeFlag[uiIdx])
      {
        if ( bLossless )
        {
          iValue = pPixelValue[uiIdx];
        }
        else
        {
          QpParam cQP(*pcCU, compID);
          Int iQP = cQP.Qp;
          Int iQPrem = iQP % 6;
          Int iQPper = iQP / 6;
#if SCM_T0118_T0112_ESCAPE_COLOR_CODING
          Int InvquantiserRightShift = IQUANT_SHIFT;
          Int iAdd = 1 << (InvquantiserRightShift - 1);
          iValue = ((((pPixelValue[uiIdx]*g_invQuantScales[iQPrem])<<iQPper) + iAdd)>>InvquantiserRightShift);
#else
          Int InvquantiserRightShift = (IQUANT_SHIFT - iQPper);
          Int iAdd = InvquantiserRightShift == 0 ? 0 : 1 << (InvquantiserRightShift - 1);
          iValue = ((pPixelValue[uiIdx]*g_invQuantScales[iQPrem] + iAdd)>>InvquantiserRightShift);
#endif 
          iValue = Pel(ClipBD<Int>(iValue, sps.getBitDepth( compID ? CHANNEL_TYPE_CHROMA: CHANNEL_TYPE_LUMA )));
        }
      }
      else
      {
        iValue = pPalette[pLevel[uiIdx]];
      }
      piReco[uiX] = iValue;
      piPicReco[uiX] = iValue;
    }

    piReco += uiStride;
    piPicReco += uiPicStride;
  }
}
#endif

Void TDecCu::xDecodePLTTexture( TComDataCU* pcCU, const UInt uiPartIdx, Pel* pPalette,  Pel* pLevel, UChar *pSPoint, Pel *pPixelValue, Pel* piReco,const UInt uiStride, const UInt uiWidth, const UInt uiHeight, const ComponentID compID, UChar* pEscapeFlag )
{
  Bool bLossless = pcCU->getCUTransquantBypass (uiPartIdx);
  Pel* piPicReco         = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiPartIdx);
  const UInt uiPicStride = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  UInt uiIdx = 0;
  Pel iValue = 0;
  Bool bRotation = pcCU->getPLTScanRotationModeFlag(uiPartIdx);

#if SCM_T0072_T0109_T0120_PLT_NON444
  if(!bRotation)
  {
#endif
    for(UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for(UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        uiIdx = (uiY<<pcCU->getPic()->getComponentScaleY(compID))*(uiWidth<<pcCU->getPic()->getComponentScaleX(compID))+(uiX<<pcCU->getPic()->getComponentScaleX(compID));
#if SCM_T0072_T0109_T0120_PLT_NON444
        UInt uiIdxComp = uiY*uiWidth + uiX;
#endif
        if( pEscapeFlag[uiIdx] )
        {
          if ( bLossless )
          {
#if SCM_T0072_T0109_T0120_PLT_NON444
            iValue = pPixelValue[uiIdxComp];
#else
            iValue = pPixelValue[uiIdx];
#endif
          }
          else
          {
            QpParam cQP(*pcCU, compID);
            Int iQP = cQP.Qp;
            Int iQPrem = iQP % 6;          
            Int iQPper = iQP / 6;
#if SCM_T0118_T0112_ESCAPE_COLOR_CODING
            Int InvquantiserRightShift = IQUANT_SHIFT;
            Int iAdd = 1 << (InvquantiserRightShift - 1);
#if SCM_T0072_T0109_T0120_PLT_NON444
            iValue = ((((pPixelValue[uiIdxComp]*g_invQuantScales[iQPrem])<<iQPper) + iAdd)>>InvquantiserRightShift);
#else
            iValue = ((((pPixelValue[uiIdx]*g_invQuantScales[iQPrem])<<iQPper) + iAdd)>>InvquantiserRightShift);
#endif
#else
            Int InvquantiserRightShift = (IQUANT_SHIFT - iQPper);
            Int iAdd = InvquantiserRightShift == 0 ? 0 : 1 << (InvquantiserRightShift - 1);
#if SCM_T0072_T0109_T0120_PLT_NON444
            iValue = ((pPixelValue[uiIdxComp]*g_invQuantScales[iQPrem] + iAdd)>>InvquantiserRightShift);
#else
            iValue = ((pPixelValue[uiIdx]*g_invQuantScales[iQPrem] + iAdd)>>InvquantiserRightShift);
#endif
#endif
            iValue = Pel(ClipBD<Int>(iValue, pcCU->getSlice()->getSPS()->getBitDepths().recon[compID?1:0]));
          }
        }
        else
        {
          iValue = pPalette[pLevel[uiIdx]];
        }
#if SCM_T0072_T0109_T0120_PLT_NON444
        piReco[uiY*uiStride+uiX] = iValue;
        piPicReco[uiY*uiPicStride+uiX] = iValue;
      } 
#else
       if(bRotation)
       {
        piReco[uiX*uiStride+uiY] = iValue;
        piPicReco[uiX*uiPicStride+uiY] = iValue;
       }
       else
       {
        piReco[uiX] = iValue;
        piPicReco[uiX] = iValue;
       }
      }

      if(!bRotation)
      {
        piReco += uiStride;
        piPicReco += uiPicStride;
      }
#endif
    }
#if SCM_T0072_T0109_T0120_PLT_NON444
  }
  else
  {
    for(UInt uiY = 0; uiY < uiWidth; uiY++ )
    {
      for(UInt uiX = 0; uiX < uiHeight; uiX++ )
      {
        uiIdx = (uiY<<pcCU->getPic()->getComponentScaleX(compID))*(uiHeight<<pcCU->getPic()->getComponentScaleY(compID))+(uiX<<pcCU->getPic()->getComponentScaleY(compID));
        UInt uiIdxComp = uiY*uiHeight + uiX;
        if( pEscapeFlag[uiIdx] )
        {
          if ( bLossless )
          {
            iValue = pPixelValue[uiIdxComp];
          }
          else
          {
            QpParam cQP(*pcCU, compID);
            Int iQP = cQP.Qp;
            Int iQPrem = iQP % 6;
            Int iQPper = iQP / 6;
#if SCM_T0118_T0112_ESCAPE_COLOR_CODING
            Int InvquantiserRightShift = IQUANT_SHIFT;
            Int iAdd = 1 << (InvquantiserRightShift - 1);
            iValue = ((((pPixelValue[uiIdxComp]*g_invQuantScales[iQPrem])<<iQPper) + iAdd)>>InvquantiserRightShift);
#else
            Int InvquantiserRightShift = (IQUANT_SHIFT - iQPper);
            Int iAdd = InvquantiserRightShift == 0 ? 0 : 1 << (InvquantiserRightShift - 1);
            iValue = ((pPixelValue[uiIdxComp]*g_invQuantScales[iQPrem] + iAdd)>>InvquantiserRightShift);
#endif 
            iValue = Pel(ClipBD<Int>(iValue, pcCU->getSlice()->getSPS()->getBitDepths().recon[compID?1:0]));
          }
        }
        else
        {
          iValue = pPalette[pLevel[uiIdx]];
        }

        piReco[uiX*uiStride+uiY] = iValue;
        piPicReco[uiX*uiPicStride+uiY] = iValue;
      }
    } 
  }
#endif
}

#if !SCM_T0072_T0109_T0120_PLT_NON444
Void  TDecCu::xReconPLTModeLuma(TComDataCU* pcCU, UInt uiDepth)
{
  Pel  * pLevel, *pPalette, *pRecChannel;

  const TComSPS &sps=*(pcCU->getSlice()->getSPS());
  const UInt uiWidth = (sps.getMaxCUWidth() >> uiDepth);
  const UInt uiHeight = (sps.getMaxCUHeight() >> uiDepth);
  const UInt uiStride = m_ppcYuvResi[uiDepth]->getStride(COMPONENT_Y);

  pLevel = pcCU->getLevel(COMPONENT_Y);
  pPalette = pcCU->getPLT(COMPONENT_Y, 0);
  pRecChannel = m_ppcYuvReco[uiDepth]->getAddr(COMPONENT_Y);

  Pel *pPixelValue = pcCU->getLevel(COMPONENT_Y);
  xDecodePLTTextureLumaChroma(pcCU, 0, pPalette, pLevel, pcCU->getSPoint(COMPONENT_Y), pPixelValue, pRecChannel, uiStride, uiWidth, uiHeight, COMPONENT_Y, pcCU->getEscapeFlag(COMPONENT_Y));
}

Void  TDecCu::xReconPLTModeChroma(TComDataCU* pcCU, UInt uiDepth)
{
  Pel  * pLevel, *pPalette, *pRecChannel;
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());
  for (UInt ch = 1; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID = ComponentID(ch);
    const UInt uiWidth = (sps.getMaxCUWidth() >> (uiDepth + m_ppcYuvResi[uiDepth]->getComponentScaleX(compID)));
    const UInt uiHeight = (sps.getMaxCUHeight() >> (uiDepth + m_ppcYuvResi[uiDepth]->getComponentScaleY(compID)));
    const UInt uiStride = m_ppcYuvResi[uiDepth]->getStride(compID);

    pLevel = pcCU->getLevel(COMPONENT_Cb);
    pPalette = pcCU->getPLT(compID, 0);
    pRecChannel = m_ppcYuvReco[uiDepth]->getAddr(compID);
    Pel *pPixelValue = pcCU->getLevel(compID);
    xDecodePLTTextureLumaChroma(pcCU, 0, pPalette, pLevel, pcCU->getSPoint(compID), pPixelValue, pRecChannel, uiStride, uiWidth, uiHeight, compID, pcCU->getEscapeFlag(compID));
  }
}
#endif

Void TDecCu::xReconPLTMode(TComDataCU *pcCU, UInt uiDepth)
{
  Pel *pLevel, *pPalette, *pRecChannel;
  for (UInt ch = 0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID = ComponentID(ch);
    const UInt uiWidth = (pcCU->getSlice()->getSPS()->getMaxCUWidth() >> (uiDepth + m_ppcYuvResi[uiDepth]->getComponentScaleX(compID)));
    const UInt uiHeight = (pcCU->getSlice()->getSPS()->getMaxCUHeight() >> (uiDepth + m_ppcYuvResi[uiDepth]->getComponentScaleY(compID)));
    const UInt uiStride = m_ppcYuvResi[uiDepth]->getStride(compID);

    pLevel = pcCU->getLevel(COMPONENT_Y);
    pPalette = pcCU->getPLT(compID, 0);
    pRecChannel = m_ppcYuvReco[uiDepth]->getAddr(compID);

    Pel *pPixelValue = pcCU->getLevel(compID);
    xDecodePLTTexture(pcCU, 0, pPalette, pLevel, pcCU->getSPoint(COMPONENT_Y), pPixelValue, pRecChannel, uiStride, uiWidth, uiHeight, compID, pcCU->getEscapeFlag(COMPONENT_Y));
  }
}

/** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiPartIdx part index
 * \param piPCM pointer to PCM code arrays
 * \param piReco pointer to reconstructed sample arrays
 * \param uiStride stride of reconstructed sample arrays
 * \param uiWidth CU width
 * \param uiHeight CU height
 * \param compID colour component ID
 * \returns Void
 */
Void TDecCu::xDecodePCMTexture( TComDataCU* pcCU, const UInt uiPartIdx, const Pel *piPCM, Pel* piReco, const UInt uiStride, const UInt uiWidth, const UInt uiHeight, const ComponentID compID)
{
        Pel* piPicReco         = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiPartIdx);
  const UInt uiPicStride       = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  const TComSPS &sps           = *(pcCU->getSlice()->getSPS());
  const UInt uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piReco[uiX] = (piPCM[uiX] << uiPcmLeftShiftBit);
      piPicReco[uiX] = piReco[uiX];
    }
    piPCM += uiWidth;
    piReco += uiStride;
    piPicReco += uiPicStride;
  }
}

/** Function for reconstructing a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiDepth CU Depth
 * \returns Void
 */
Void TDecCu::xReconPCM( TComDataCU* pcCU, UInt uiDepth )
{
  const UInt maxCuWidth     = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt maxCuHeight    = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID = ComponentID(ch);
    const UInt width  = (maxCuWidth >>(uiDepth+m_ppcYuvResi[uiDepth]->getComponentScaleX(compID)));
    const UInt height = (maxCuHeight>>(uiDepth+m_ppcYuvResi[uiDepth]->getComponentScaleY(compID)));
    const UInt stride = m_ppcYuvResi[uiDepth]->getStride(compID);
    Pel * pPCMChannel = pcCU->getPCMSample(compID);
    Pel * pRecChannel = m_ppcYuvReco[uiDepth]->getAddr(compID);
    xDecodePCMTexture( pcCU, 0, pPCMChannel, pRecChannel, stride, width, height, compID );
  }
}

/** Function for filling the PCM buffer of a CU using its reconstructed sample array
 * \param pCU   pointer to current CU
 * \param depth CU Depth
 */
Void TDecCu::xFillPCMBuffer(TComDataCU* pCU, UInt depth)
{
  const ChromaFormat format = pCU->getPic()->getChromaFormat();
  const UInt numValidComp   = getNumberValidComponents(format);
  const UInt maxCuWidth     = pCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt maxCuHeight    = pCU->getSlice()->getSPS()->getMaxCUHeight();

  for (UInt componentIndex = 0; componentIndex < numValidComp; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    const UInt width  = maxCuWidth  >> (depth + getComponentScaleX(component, format));
    const UInt height = maxCuHeight >> (depth + getComponentScaleY(component, format));

    Pel *source      = m_ppcYuvReco[depth]->getAddr(component, 0, width);
    Pel *destination = pCU->getPCMSample(component);

    const UInt sourceStride = m_ppcYuvReco[depth]->getStride(component);

    for (Int line = 0; line < height; line++)
    {
      for (Int column = 0; column < width; column++)
      {
        destination[column] = source[column];
      }

      source      += sourceStride;
      destination += width;
    }
  }
}

//! \}
