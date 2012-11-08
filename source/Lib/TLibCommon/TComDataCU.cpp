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

/** \file     TComDataCU.cpp
    \brief    CU data structure
    \todo     not all entities are documented
*/

#include "TComDataCU.h"
#include "TComTU.h"
#include "TComPic.h"

//! \ingroup TLibCommon
//! \{

#if ADAPTIVE_QP_SELECTION
  TCoeff * TComDataCU::m_pcGlbArlCoeff[MAX_NUM_COMPONENT] = { NULL, NULL, NULL };
#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComDataCU::TComDataCU()
{
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_puhDepth           = NULL;

#if SKIP_FLAG
  m_skipFlag           = NULL;
#endif

  m_pePartSize         = NULL;
  m_pePredMode         = NULL;
  m_CUTransquantBypass = NULL;
  m_puhWidth           = NULL;
  m_puhHeight          = NULL;
  m_phQP               = NULL;
  m_pbMergeFlag        = NULL;
  m_puhMergeIndex      = NULL;
  for(UInt i=0; i<MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_puhIntraDir[i]     = NULL;
  }
  m_puhInterDir        = NULL;
  m_puhTrIdx           = NULL;
#if !REMOVE_NSQT
  m_nsqtPartIdx        = NULL;
#endif

  for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_puhCbf[comp]           = NULL;
    m_puhTransformSkip[comp] = NULL;
    m_pcTrCoeff[comp]        = NULL;
#if ADAPTIVE_QP_SELECTION
    m_pcArlCoeff[comp]       = NULL;
#endif
    m_pcIPCMSample[comp]     = NULL;
#if !REMOVE_ALF
    m_lcuAlfEnabled[comp]    = false;
#endif
  }
#if ADAPTIVE_QP_SELECTION
  m_ArlCoeffIsAliasedAllocation = false;
#endif
  m_pbIPCMFlag         = NULL;

  m_pcCUAboveLeft      = NULL;
  m_pcCUAboveRight     = NULL;
  m_pcCUAbove          = NULL;
  m_pcCULeft           = NULL;

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
    m_apiMVPIdx[i]       = NULL;
    m_apiMVPNum[i]       = NULL;
  }

  m_bDecSubCu          = false;
  m_uiSliceStartCU        = 0;
  m_uiDependentSliceStartCU = 0;
}

TComDataCU::~TComDataCU()
{
}

Void TComDataCU::create( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
                        , Bool bGlobalRMARLBuffer
#endif
                        )
{
  m_bDecSubCu = bDecSubCu;

  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_uiNumPartition     = uiNumPartition;
  m_unitSize = unitSize;

  if ( !bDecSubCu )
  {
    m_phQP               = (Char*     )xMalloc(Char,     uiNumPartition);
    m_puhDepth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhWidth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhHeight          = (UChar*    )xMalloc(UChar,    uiNumPartition);

#if SKIP_FLAG
    m_skipFlag           = new Bool[ uiNumPartition ];
#endif

    m_pePartSize         = new Char[ uiNumPartition ];
    memset( m_pePartSize, NUMBER_OF_PART_SIZES,uiNumPartition * sizeof( *m_pePartSize ) );
    m_pePredMode         = new Char[ uiNumPartition ];
    m_CUTransquantBypass = new Bool[ uiNumPartition ];

    m_pbMergeFlag        = (Bool*  )xMalloc(Bool,   uiNumPartition);
    m_puhMergeIndex      = (UChar* )xMalloc(UChar,  uiNumPartition);
    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      m_puhIntraDir[ch] = (UChar* )xMalloc(UChar,  uiNumPartition);
    }
    m_puhInterDir        = (UChar* )xMalloc(UChar,  uiNumPartition);

    m_puhTrIdx           = (UChar* )xMalloc(UChar,  uiNumPartition);
#if !REMOVE_NSQT
    m_nsqtPartIdx        = (UChar* )xMalloc(UChar,  uiNumPartition);
#endif

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl]       = new Char[ uiNumPartition ];
      m_apiMVPNum[rpl]       = new Char[ uiNumPartition ];
      memset( m_apiMVPIdx[rpl], -1,uiNumPartition * sizeof( Char ) );
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      const UInt chromaShift = getComponentScaleX(compID, chromaFormatIDC) + getComponentScaleY(compID, chromaFormatIDC);
      const UInt totalSize   = (uiWidth * uiHeight) >> chromaShift;

      m_puhTransformSkip[compID] = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_puhCbf[compID]           = (UChar* )xMalloc(UChar,  uiNumPartition);
      
      m_pcTrCoeff[compID]      = (TCoeff*)xMalloc(TCoeff, totalSize);
      memset( m_pcTrCoeff[compID], 0, (totalSize * sizeof( TCoeff )) );

#if ADAPTIVE_QP_SELECTION
      if( bGlobalRMARLBuffer )
      {
        if (m_pcGlbArlCoeff[compID] == NULL) m_pcGlbArlCoeff[compID] = (TCoeff*)xMalloc(TCoeff, totalSize);

        m_pcArlCoeff[compID] = m_pcGlbArlCoeff[compID];
        m_ArlCoeffIsAliasedAllocation = true;
      }
      else
      {
         m_pcArlCoeff[compID] = (TCoeff*)xMalloc(TCoeff, totalSize);
      }
#endif
      m_pcIPCMSample[compID] = (Pel*   )xMalloc(Pel , totalSize);
    }

    m_pbIPCMFlag         = (Bool*  )xMalloc(Bool, uiNumPartition);

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].create( uiNumPartition );
    }

  }
  else
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].setNumPartition(uiNumPartition );
    }
  }

  m_uiSliceStartCU        = (UInt*  )xMalloc(UInt, uiNumPartition);
  m_uiDependentSliceStartCU = (UInt*  )xMalloc(UInt, uiNumPartition);

  // create motion vector fields

  m_pcCUAboveLeft      = NULL;
  m_pcCUAboveRight     = NULL;
  m_pcCUAbove          = NULL;
  m_pcCULeft           = NULL;

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
  }
}

Void TComDataCU::destroy()
{
  // encoder-side buffer free
  if ( !m_bDecSubCu )
  {
    if ( m_phQP               ) { xFree(m_phQP);                m_phQP               = NULL; }
    if ( m_puhDepth           ) { xFree(m_puhDepth);            m_puhDepth           = NULL; }
    if ( m_puhWidth           ) { xFree(m_puhWidth);            m_puhWidth           = NULL; }
    if ( m_puhHeight          ) { xFree(m_puhHeight);           m_puhHeight          = NULL; }

#if SKIP_FLAG
    if ( m_skipFlag           ) { delete[] m_skipFlag;          m_skipFlag          = NULL; }
#endif

    if ( m_pePartSize         ) { delete[] m_pePartSize;        m_pePartSize         = NULL; }
    if ( m_pePredMode         ) { delete[] m_pePredMode;        m_pePredMode         = NULL; }
    if ( m_CUTransquantBypass ) { delete[] m_CUTransquantBypass;m_CUTransquantBypass = NULL; }
    if ( m_puhInterDir        ) { xFree(m_puhInterDir);         m_puhInterDir        = NULL; }
    if ( m_pbMergeFlag        ) { xFree(m_pbMergeFlag);         m_pbMergeFlag        = NULL; }
    if ( m_puhMergeIndex      ) { xFree(m_puhMergeIndex);       m_puhMergeIndex      = NULL; }
    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      xFree(m_puhIntraDir[ch]);
      m_puhIntraDir[ch] = NULL;
    }

    if ( m_puhTrIdx           ) { xFree(m_puhTrIdx);            m_puhTrIdx          = NULL; }
#if !REMOVE_NSQT
    if ( m_nsqtPartIdx        ) { xFree(m_nsqtPartIdx);         m_nsqtPartIdx       = NULL; }
#endif

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      if ( m_puhTransformSkip[comp]) { xFree(m_puhTransformSkip[comp]); m_puhTransformSkip[comp] = NULL; }
      if ( m_puhCbf[comp]          ) { xFree(m_puhCbf[comp]);           m_puhCbf[comp]           = NULL; }
      if ( m_pcTrCoeff[comp]       ) { xFree(m_pcTrCoeff[comp]);        m_pcTrCoeff[comp]        = NULL; }

#if ADAPTIVE_QP_SELECTION
      if (!m_ArlCoeffIsAliasedAllocation)
      {
        if ( m_pcArlCoeff[comp]     ) { xFree(m_pcArlCoeff[comp]);      m_pcArlCoeff[comp]    = NULL; }
      }

      if ( m_pcGlbArlCoeff[comp]  ) { xFree(m_pcGlbArlCoeff[comp]);   m_pcGlbArlCoeff[comp] = NULL; }
#endif

      if ( m_pcIPCMSample[comp]   ) { xFree(m_pcIPCMSample[comp]);    m_pcIPCMSample[comp]  = NULL; }
    }
    if ( m_pbIPCMFlag         ) { xFree(m_pbIPCMFlag   );       m_pbIPCMFlag        = NULL; }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      if ( m_apiMVPIdx[rpl]       ) { delete[] m_apiMVPIdx[rpl];      m_apiMVPIdx[rpl]      = NULL; }
      if ( m_apiMVPNum[rpl]       ) { delete[] m_apiMVPNum[rpl];      m_apiMVPNum[rpl]      = NULL; }
      m_acCUMvField[rpl].destroy();
    }


  }

  m_pcPic              = NULL;
  m_pcSlice            = NULL;

  m_pcCUAboveLeft       = NULL;
  m_pcCUAboveRight      = NULL;
  m_pcCUAbove           = NULL;
  m_pcCULeft            = NULL;


  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
  }

  if( m_uiSliceStartCU )
  {
    xFree(m_uiSliceStartCU);
    m_uiSliceStartCU=NULL;
  }
  if(m_uiDependentSliceStartCU )
  {
    xFree(m_uiDependentSliceStartCU);
    m_uiDependentSliceStartCU=NULL;
  }
}

const NDBFBlockInfo& NDBFBlockInfo::operator= (const NDBFBlockInfo& src)
{
  this->tileID = src.tileID;
  this->sliceID= src.sliceID;
  this->startSU= src.startSU;
  this->endSU  = src.endSU;
  this->widthSU= src.widthSU;
  this->heightSU=src.heightSU;
  this->posX   = src.posX;
  this->posY   = src.posY;
  this->width  = src.width;
  this->height = src.height;
  memcpy(this->isBorderAvailable, src.isBorderAvailable, sizeof(Bool)*((Int)NUM_SGU_BORDER));
  this->allBordersAvailable = src.allBordersAvailable;

  return *this;
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Initialization
// --------------------------------------------------------------------------------------------------------------------

/**
 - initialize top-level CU
 - internal buffers are already created
 - set values before encoding a CU
 .
 \param  pcPic     picture (TComPic) class pointer
 \param  iCUAddr   CU address
 */
Void TComDataCU::initCU( TComPic* pcPic, UInt iCUAddr )
{

  m_pcPic              = pcPic;
  m_pcSlice            = pcPic->getSlice(pcPic->getCurrSliceIdx());
  m_uiCUAddr           = iCUAddr;
  m_uiCUPelX           = ( iCUAddr % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth;
  m_uiCUPelY           = ( iCUAddr / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight;
  m_uiAbsIdxInLCU      = 0;
  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;
  m_uiNumPartition     = pcPic->getNumPartInCU();
  m_numSucIPCM       = 0;
  m_lastCUSucIPCMFlag   = false;

  for(int i=0; i<pcPic->getNumPartInCU(); i++)
  {
    if(pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr)*pcPic->getNumPartInCU()+i>=getSlice()->getSliceCurStartCUAddr())
    {
      m_uiSliceStartCU[i]=getSlice()->getSliceCurStartCUAddr();
    }
    else
    {
      m_uiSliceStartCU[i]=pcPic->getCU(getAddr())->m_uiSliceStartCU[i];
    }
  }
  for(int i=0; i<pcPic->getNumPartInCU(); i++)
  {
    if(pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr)*pcPic->getNumPartInCU()+i>=getSlice()->getDependentSliceCurStartCUAddr())
    {
      m_uiDependentSliceStartCU[i]=getSlice()->getDependentSliceCurStartCUAddr();
    }
    else
    {
      m_uiDependentSliceStartCU[i]=pcPic->getCU(getAddr())->m_uiDependentSliceStartCU[i];
    }
  }

  Int partStartIdx = getSlice()->getDependentSliceCurStartCUAddr() - pcPic->getPicSym()->getInverseCUOrderMap(iCUAddr) * pcPic->getNumPartInCU();

  Int numElements = min<Int>( partStartIdx, m_uiNumPartition );
  for ( Int ui = 0; ui < numElements; ui++ )
  {
    TComDataCU * pcFrom = pcPic->getCU(getAddr());
#if SKIP_FLAG
    m_skipFlag[ui]   = pcFrom->getSkipFlag(ui);
#endif
    m_pePartSize[ui] = pcFrom->getPartitionSize(ui);
    m_pePredMode[ui] = pcFrom->getPredictionMode(ui);
    m_CUTransquantBypass[ui] = pcFrom->getCUTransquantBypass(ui);
    m_puhDepth[ui] = pcFrom->getDepth(ui);
    m_puhWidth  [ui] = pcFrom->getWidth(ui);
    m_puhHeight [ui] = pcFrom->getHeight(ui);
    m_puhTrIdx  [ui] = pcFrom->getTransformIdx(ui);
#if !REMOVE_NSQT
    m_nsqtPartIdx[ui] = pcFrom->getNSQTPartIdx(ui);
#endif
    for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      m_puhTransformSkip[comp][ui]  = pcFrom->getTransformSkip(ui,ComponentID(comp));
    }
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl][ui] = pcFrom->m_apiMVPIdx[rpl][ui];
      m_apiMVPNum[rpl][ui] = pcFrom->m_apiMVPNum[rpl][ui];
    }
    m_phQP[ui]=pcFrom->m_phQP[ui];

#if !REMOVE_ALF
    for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      m_lcuAlfEnabled[comp] = pcFrom->m_lcuAlfEnabled[comp];
    }
#endif
    m_pbMergeFlag[ui]=pcFrom->m_pbMergeFlag[ui];
    m_puhMergeIndex[ui]=pcFrom->m_puhMergeIndex[ui];
    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      m_puhIntraDir[ch][ui] = pcFrom->m_puhIntraDir[ch][ui];
    }

    m_puhInterDir[ui]=pcFrom->m_puhInterDir[ui];
    for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      m_puhCbf[comp][ui]=pcFrom->m_puhCbf[comp][ui];
    }
    m_pbIPCMFlag[ui] = pcFrom->m_pbIPCMFlag[ui];
  }

  Int firstElement = max<Int>( partStartIdx, 0 );
  numElements = m_uiNumPartition - firstElement;

  if ( numElements > 0 )
  {
#if SKIP_FLAG
    memset( m_skipFlag          + firstElement, false,                      numElements * sizeof( *m_skipFlag ) );
#endif

    memset( m_pePartSize        + firstElement, NUMBER_OF_PART_SIZES,       numElements * sizeof( *m_pePartSize ) );
    memset( m_pePredMode        + firstElement, NUMBER_OF_PREDICTION_MODES, numElements * sizeof( *m_pePredMode ) );
    memset( m_CUTransquantBypass+ firstElement, false,                      numElements * sizeof( *m_CUTransquantBypass) );
    memset( m_puhDepth          + firstElement, 0,                          numElements * sizeof( *m_puhDepth ) );
    memset( m_puhTrIdx          + firstElement, 0,                          numElements * sizeof( *m_puhTrIdx ) );
#if !REMOVE_NSQT
    memset( m_nsqtPartIdx       + firstElement, 0,                          numElements * sizeof( *m_nsqtPartIdx) );
#endif
    memset( m_puhWidth          + firstElement, g_uiMaxCUWidth,             numElements * sizeof( *m_puhWidth ) );
    memset( m_puhHeight         + firstElement, g_uiMaxCUHeight,            numElements * sizeof( *m_puhHeight ) );
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      memset( m_apiMVPIdx[rpl]  + firstElement, -1,                         numElements * sizeof( *m_apiMVPIdx[rpl] ) );
      memset( m_apiMVPNum[rpl]  + firstElement, -1,                         numElements * sizeof( *m_apiMVPNum[rpl] ) );
    }
    memset( m_phQP              + firstElement, getSlice()->getSliceQp(),   numElements * sizeof( *m_phQP ) );

    for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      memset( m_puhTransformSkip[comp] + firstElement, 0,                      numElements * sizeof( *m_puhTransformSkip[comp]) );
#if !REMOVE_ALF
      m_lcuAlfEnabled[comp]= false;
#endif
      memset( m_puhCbf[comp]           + firstElement, 0,                      numElements * sizeof( *m_puhCbf[comp] ) );
    }

    memset( m_pbMergeFlag       + firstElement, false,                    numElements * sizeof( *m_pbMergeFlag ) );
    memset( m_puhMergeIndex     + firstElement, 0,                        numElements * sizeof( *m_puhMergeIndex ) );
    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      memset( m_puhIntraDir[ch] + firstElement, ((ch==0) ? DC_IDX : 0),   numElements * sizeof( *(m_puhIntraDir[ch]) ) );
    }
    memset( m_puhInterDir       + firstElement, 0,                        numElements * sizeof( *m_puhInterDir ) );
    memset( m_pbIPCMFlag        + firstElement, false,                    numElements * sizeof( *m_pbIPCMFlag ) );
  }

  const UInt numCoeffY    = g_uiMaxCUWidth*g_uiMaxCUHeight;
  if ( 0 >= partStartIdx )
  {
    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      const UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(comp)) + m_pcPic->getComponentScaleY(ComponentID(comp));
      memset( m_pcTrCoeff[comp], 0, sizeof(TCoeff)* numCoeffY>>componentShift );
#if ADAPTIVE_QP_SELECTION
      memset( m_pcArlCoeff[comp], 0, sizeof(TCoeff)* numCoeffY>>componentShift );
#endif
    }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].clearMvField();
    }
  }
  else
  {
    TComDataCU * pcFrom = pcPic->getCU(getAddr());
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].copyFrom(&pcFrom->m_acCUMvField[i],m_uiNumPartition,0);
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(comp)) + m_pcPic->getComponentScaleY(ComponentID(comp));
      UInt numCoeffInChannel = numCoeffY >> componentShift;
      for (UInt coeff=0; coeff<numCoeffInChannel; coeff++)
      {
        m_pcTrCoeff[comp][coeff]=pcFrom->m_pcTrCoeff[comp][coeff];
#if ADAPTIVE_QP_SELECTION
        m_pcArlCoeff[comp][coeff]=pcFrom->m_pcArlCoeff[comp][coeff];
#endif
        m_pcIPCMSample[comp][coeff]=pcFrom->m_pcIPCMSample[comp][coeff];
      }
    }
  }

  // Setting neighbor CU
  m_pcCULeft        = NULL;
  m_pcCUAbove       = NULL;
  m_pcCUAboveLeft   = NULL;
  m_pcCUAboveRight  = NULL;


  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
  }

  UInt uiWidthInCU = pcPic->getFrameWidthInCU();
  if ( m_uiCUAddr % uiWidthInCU )
  {
    m_pcCULeft = pcPic->getCU( m_uiCUAddr - 1 );
  }

  if ( m_uiCUAddr / uiWidthInCU )
  {
    m_pcCUAbove = pcPic->getCU( m_uiCUAddr - uiWidthInCU );
  }

  if ( m_pcCULeft && m_pcCUAbove )
  {
    m_pcCUAboveLeft = pcPic->getCU( m_uiCUAddr - uiWidthInCU - 1 );
  }

  if ( m_pcCUAbove && ( (m_uiCUAddr%uiWidthInCU) < (uiWidthInCU-1) )  )
  {
    m_pcCUAboveRight = pcPic->getCU( m_uiCUAddr - uiWidthInCU + 1 );
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    if ( getSlice()->getNumRefIdx( rpl ) > 0 )
    {
      m_apcCUColocated[rpl] = getSlice()->getRefPic( rpl, 0)->getCU( m_uiCUAddr );
    }
  }
}


/** initialize prediction data with enabling sub-LCU-level delta QP
*\param  uiDepth  depth of the current CU
*\param  qp     qp for the current CU
*- set CU width and CU height according to depth
*- set qp value according to input qp
*- set last-coded qp value according to input last-coded qp
*/
Void TComDataCU::initEstData( const UInt uiDepth, const Int qp, const Bool bTransquantBypass )
{
  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;

  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
#if !REMOVE_ALF
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    m_lcuAlfEnabled[ch] = false;
  }
#endif

  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
    if(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU+ui >= getSlice()->getDependentSliceCurStartCUAddr())
    {
      for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
      {
        const RefPicList rpl=RefPicList(i);
        m_apiMVPIdx[rpl][ui]  = -1;
        m_apiMVPNum[rpl][ui]  = -1;
      }
      m_puhDepth  [ui]    = uiDepth;
      m_puhWidth  [ui]    = uhWidth;
      m_puhHeight [ui]    = uhHeight;
      m_puhTrIdx  [ui]    = 0;
#if !REMOVE_NSQT
      m_nsqtPartIdx[ui] = 0;
#endif
      for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
      {
        m_puhTransformSkip[comp][ui] = 0;
      }
#if SKIP_FLAG
      m_skipFlag[ui]      = false;
#endif
      m_pePartSize[ui]    = NUMBER_OF_PART_SIZES;
      m_pePredMode[ui]    = NUMBER_OF_PREDICTION_MODES;
      m_CUTransquantBypass[ui] = bTransquantBypass;
      m_pbIPCMFlag[ui]    = 0;
      m_phQP[ui]          = qp;
      m_pbMergeFlag[ui]   = 0;
      m_puhMergeIndex[ui] = 0;

      for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
      {
        m_puhIntraDir[ch][ui] = ((ch==0) ? DC_IDX : 0);
      }

      m_puhInterDir[ui] = 0;
      for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
      {
        m_puhCbf[comp][ui] = 0;
      }
    }
  }

  if(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU >= getSlice()->getDependentSliceCurStartCUAddr())
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].clearMvField();
    }
    const UInt numCoeffY = uhWidth*uhHeight;
    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      const ComponentID component = ComponentID(comp);
      const UInt numCoeff = numCoeffY >> (getPic()->getComponentScaleX(component) + getPic()->getComponentScaleY(component));
      memset( m_pcTrCoeff[comp],    0, numCoeff * sizeof( TCoeff ) );
#if ADAPTIVE_QP_SELECTION
      memset( m_pcArlCoeff[comp],   0, numCoeff * sizeof( TCoeff ) );
#endif
      memset( m_pcIPCMSample[comp], 0, numCoeff * sizeof( Pel) );
    }
  }
}


// initialize Sub partition
Void TComDataCU::initSubCU( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp )
{
  assert( uiPartUnitIdx<4 );

  UInt uiPartOffset = ( pcCU->getTotalNumPart()>>2 )*uiPartUnitIdx;

  m_pcPic              = pcCU->getPic();
  m_pcSlice            = m_pcPic->getSlice(m_pcPic->getCurrSliceIdx());
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = pcCU->getZorderIdxInCU() + uiPartOffset;

  m_uiCUPelX           = pcCU->getCUPelX() + ( g_uiMaxCUWidth>>uiDepth  )*( uiPartUnitIdx &  1 );
  m_uiCUPelY           = pcCU->getCUPelY() + ( g_uiMaxCUHeight>>uiDepth  )*( uiPartUnitIdx >> 1 );

  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;
  m_uiNumPartition     = pcCU->getTotalNumPart() >> 2;

  m_numSucIPCM       = 0;
  m_lastCUSucIPCMFlag   = false;

  Int iSizeInUchar = sizeof( UChar  ) * m_uiNumPartition;
  Int iSizeInBool  = sizeof( Bool   ) * m_uiNumPartition;

  Int sizeInChar = sizeof( Char  ) * m_uiNumPartition;
  memset( m_phQP,              qp,  sizeInChar );

#if !REMOVE_ALF
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_lcuAlfEnabled[comp] = pcCU->m_lcuAlfEnabled[comp];
  }
#endif

  memset( m_pbMergeFlag,        0, iSizeInBool  );
  memset( m_puhMergeIndex,      0, iSizeInUchar );
  for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    memset( m_puhIntraDir[ch],  ((ch==0) ? DC_IDX : 0), iSizeInUchar );
  }

  memset( m_puhInterDir,        0, iSizeInUchar );
  memset( m_puhTrIdx,           0, iSizeInUchar );

  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    memset( m_puhTransformSkip[comp], 0, iSizeInUchar );
    memset( m_puhCbf[comp],           0, iSizeInUchar );
  }

  memset( m_puhDepth,     uiDepth, iSizeInUchar );

  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
  memset( m_puhWidth,          uhWidth,  iSizeInUchar );
  memset( m_puhHeight,         uhHeight, iSizeInUchar );
  memset( m_pbIPCMFlag,        0, iSizeInBool  );
  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
#if SKIP_FLAG
    m_skipFlag[ui]   = false;
#endif
    m_pePartSize[ui] = NUMBER_OF_PART_SIZES;
    m_pePredMode[ui] = NUMBER_OF_PREDICTION_MODES;
    m_CUTransquantBypass[ui] = false;
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl][ui] = -1;
      m_apiMVPNum[rpl][ui] = -1;
    }
    if(m_pcPic->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU+ui<getSlice()->getDependentSliceCurStartCUAddr())
    {
      for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
      {
        const RefPicList rpl=RefPicList(i);
        m_apiMVPIdx[rpl][ui] = pcCU->m_apiMVPIdx[rpl][uiPartOffset+ui];
        m_apiMVPNum[rpl][ui] = pcCU->m_apiMVPNum[rpl][uiPartOffset+ui];
      }
      m_puhDepth  [ui] = pcCU->getDepth(uiPartOffset+ui);
      m_puhWidth  [ui] = pcCU->getWidth(uiPartOffset+ui);
      m_puhHeight  [ui] = pcCU->getHeight(uiPartOffset+ui);
      m_puhTrIdx  [ui] = pcCU->getTransformIdx(uiPartOffset+ui);
#if !REMOVE_NSQT
      m_nsqtPartIdx[ui] = pcCU->getNSQTPartIdx(uiPartOffset+ui);
#endif
      for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
      {
        m_puhTransformSkip[comp][ui] = pcCU->getTransformSkip(uiPartOffset+ui,ComponentID(comp));
        m_puhCbf[comp][ui]=pcCU->m_puhCbf[comp][uiPartOffset+ui];
      }
#if SKIP_FLAG
      m_skipFlag[ui]   = pcCU->getSkipFlag(uiPartOffset+ui);
#endif
      m_pePartSize[ui] = pcCU->getPartitionSize(uiPartOffset+ui);
      m_pePredMode[ui] = pcCU->getPredictionMode(uiPartOffset+ui);
      m_CUTransquantBypass[ui] = pcCU->getCUTransquantBypass(uiPartOffset+ui);
      m_pbIPCMFlag[ui]=pcCU->m_pbIPCMFlag[uiPartOffset+ui];
      m_phQP[ui] = pcCU->m_phQP[uiPartOffset+ui];
      m_pbMergeFlag[ui]=pcCU->m_pbMergeFlag[uiPartOffset+ui];
      m_puhMergeIndex[ui]=pcCU->m_puhMergeIndex[uiPartOffset+ui];
      for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
      {
        m_puhIntraDir[ch][ui] = pcCU->m_puhIntraDir[ch][uiPartOffset+ui];
      }

      m_puhInterDir[ui]=pcCU->m_puhInterDir[uiPartOffset+ui];

    }
  }

  const UInt numCoeffY    = uhWidth*uhHeight;
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(ch)) + m_pcPic->getComponentScaleY(ComponentID(ch));
    memset( m_pcTrCoeff[ch],  0, sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeff[ch], 0, sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memset( m_pcIPCMSample[ch], 0, sizeof(Pel)* (numCoeffY>>componentShift) );
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acCUMvField[i].clearMvField();
  }

  if(m_pcPic->getPicSym()->getInverseCUOrderMap(getAddr())*m_pcPic->getNumPartInCU()+m_uiAbsIdxInLCU<getSlice()->getDependentSliceCurStartCUAddr())
  {
    // Part of this CU contains data from an older slice. Now copy in that data.
    UInt uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
    UInt uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();
    TComDataCU * bigCU = getPic()->getCU(getAddr());
    Int minui = uiPartOffset;
    minui = -minui;
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      pcCU->m_acCUMvField[i].copyTo(&m_acCUMvField[i],minui,uiPartOffset,m_uiNumPartition);
    }
    const UInt uiCoffOffset = uiMaxCuWidth*uiMaxCuHeight*m_uiAbsIdxInLCU/pcCU->getPic()->getNumPartInCU();

    for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      const UInt componentShift     = m_pcPic->getComponentScaleX(ComponentID(ch)) + m_pcPic->getComponentScaleY(ComponentID(ch));
      const UInt numCoeffInChannel  = numCoeffY >> componentShift;
      for (UInt coeff=0; coeff<numCoeffInChannel; coeff++)
      {
        const UInt offset = uiCoffOffset>>componentShift;
        m_pcTrCoeff[ch][coeff]=bigCU->m_pcTrCoeff[ch][coeff + offset];
#if ADAPTIVE_QP_SELECTION
        m_pcArlCoeff[ch][coeff]=bigCU->m_pcArlCoeff[ch][coeff + offset];
#endif
        m_pcIPCMSample[ch][coeff]=bigCU->m_pcIPCMSample[ch][coeff + offset];
      }
    }
  }

  m_pcCULeft        = pcCU->getCULeft();
  m_pcCUAbove       = pcCU->getCUAbove();
  m_pcCUAboveLeft   = pcCU->getCUAboveLeft();
  m_pcCUAboveRight  = pcCU->getCUAboveRight();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i] = pcCU->getCUColocated(RefPicList(i));
  }
  memcpy(m_uiSliceStartCU,pcCU->m_uiSliceStartCU+uiPartOffset,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_uiDependentSliceStartCU,pcCU->m_uiDependentSliceStartCU+uiPartOffset,sizeof(UInt)*m_uiNumPartition);
}

Void TComDataCU::setOutsideCUPart( UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiNumPartition = m_uiNumPartition >> (uiDepth << 1);
  UInt uiSizeInUchar = sizeof( UChar  ) * uiNumPartition;

  UChar uhWidth  = g_uiMaxCUWidth  >> uiDepth;
  UChar uhHeight = g_uiMaxCUHeight >> uiDepth;
  memset( m_puhDepth    + uiAbsPartIdx,     uiDepth,  uiSizeInUchar );
  memset( m_puhWidth    + uiAbsPartIdx,     uhWidth,  uiSizeInUchar );
  memset( m_puhHeight   + uiAbsPartIdx,     uhHeight, uiSizeInUchar );
}

// --------------------------------------------------------------------------------------------------------------------
// Copy
// --------------------------------------------------------------------------------------------------------------------

Void TComDataCU::copySubCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiPart = uiAbsPartIdx;

  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = uiAbsPartIdx;

  m_uiCUPelX           = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  m_uiCUPelY           = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

#if SKIP_FLAG
  m_skipFlag=pcCU->getSkipFlag()          + uiPart;
#endif

  m_phQP=pcCU->getQP()                    + uiPart;
  m_pePartSize = pcCU->getPartitionSize() + uiPart;
  m_pePredMode=pcCU->getPredictionMode()  + uiPart;
  m_CUTransquantBypass  = pcCU->getCUTransquantBypass()+uiPart;

  m_pbMergeFlag         = pcCU->getMergeFlag()        + uiPart;
  m_puhMergeIndex       = pcCU->getMergeIndex()       + uiPart;

  for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_puhIntraDir[ch]   = pcCU->getIntraDir(ChannelType(ch)) + uiPart;
  }

  m_puhInterDir         = pcCU->getInterDir()         + uiPart;
  m_puhTrIdx            = pcCU->getTransformIdx()     + uiPart;
#if !REMOVE_NSQT
  m_nsqtPartIdx         = pcCU->getNSQTPartIdx()      + uiPart;
#endif
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_puhTransformSkip[comp] = pcCU->getTransformSkip(ComponentID(comp))  + uiPart;
    m_puhCbf[comp]           = pcCU->getCbf(ComponentID(comp))            + uiPart;
  }

  m_puhDepth=pcCU->getDepth()                     + uiPart;
  m_puhWidth=pcCU->getWidth()                     + uiPart;
  m_puhHeight=pcCU->getHeight()                   + uiPart;

  m_pbIPCMFlag         = pcCU->getIPCMFlag()        + uiPart;

  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    m_apcCUColocated[rpl] = pcCU->getCUColocated(rpl);
    m_apiMVPIdx[rpl]=pcCU->getMVPIdx(rpl)  + uiPart;
    m_apiMVPNum[rpl]=pcCU->getMVPNum(rpl)  + uiPart;
    m_acCUMvField[rpl].linkToWithOffset( pcCU->getCUMvField(rpl), uiPart );
  }

//  UInt uiTmp = uiWidth*uiHeight;
  UInt uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
  UInt uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();

  UInt uiCoffOffset = uiMaxCuWidth*uiMaxCuHeight*uiAbsPartIdx/pcCU->getPic()->getNumPartInCU();

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const ComponentID component = ComponentID(ch);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    const UInt offset           = uiCoffOffset >> componentShift;
    m_pcTrCoeff[ch] = pcCU->getCoeff(component) + offset;
#if ADAPTIVE_QP_SELECTION
    m_pcArlCoeff[ch] = pcCU->getArlCoeff(component) + offset;
#endif
    m_pcIPCMSample[ch] = pcCU->getPCMSample(component) + offset;
  }

  memcpy(m_uiSliceStartCU,pcCU->m_uiSliceStartCU+uiPart,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_uiDependentSliceStartCU,pcCU->m_uiDependentSliceStartCU+uiPart,sizeof(UInt)*m_uiNumPartition);
}

// Copy inter prediction info from the biggest CU
Void TComDataCU::copyInterPredInfoFrom    ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList )
{
  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_uiCUAddr           = pcCU->getAddr();
  m_uiAbsIdxInLCU      = uiAbsPartIdx;

  Int iRastPartIdx     = g_auiZscanToRaster[uiAbsPartIdx];
  m_uiCUPelX           = pcCU->getCUPelX() + m_pcPic->getMinCUWidth ()*( iRastPartIdx % m_pcPic->getNumPartInWidth() );
  m_uiCUPelY           = pcCU->getCUPelY() + m_pcPic->getMinCUHeight()*( iRastPartIdx / m_pcPic->getNumPartInWidth() );

  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = pcCU->getCUColocated(RefPicList(i));
  }

#if SKIP_FLAG
  m_skipFlag           = pcCU->getSkipFlag ()             + uiAbsPartIdx;
#endif

  m_pePartSize         = pcCU->getPartitionSize ()        + uiAbsPartIdx;
  m_pePredMode         = pcCU->getPredictionMode()        + uiAbsPartIdx;
  m_CUTransquantBypass = pcCU->getCUTransquantBypass()    + uiAbsPartIdx;
  m_puhInterDir        = pcCU->getInterDir      ()        + uiAbsPartIdx;

  m_puhDepth           = pcCU->getDepth ()                + uiAbsPartIdx;
  m_puhWidth           = pcCU->getWidth ()                + uiAbsPartIdx;
  m_puhHeight          = pcCU->getHeight()                + uiAbsPartIdx;

  m_pbMergeFlag        = pcCU->getMergeFlag()             + uiAbsPartIdx;
  m_puhMergeIndex      = pcCU->getMergeIndex()            + uiAbsPartIdx;

  m_apiMVPIdx[eRefPicList] = pcCU->getMVPIdx(eRefPicList) + uiAbsPartIdx;
  m_apiMVPNum[eRefPicList] = pcCU->getMVPNum(eRefPicList) + uiAbsPartIdx;

  m_acCUMvField[ eRefPicList ].linkToWithOffset( pcCU->getCUMvField(eRefPicList), uiAbsPartIdx );

  memcpy(m_uiSliceStartCU,pcCU->m_uiSliceStartCU+uiAbsPartIdx,sizeof(UInt)*m_uiNumPartition);
  memcpy(m_uiDependentSliceStartCU,pcCU->m_uiDependentSliceStartCU+uiAbsPartIdx,sizeof(UInt)*m_uiNumPartition);
}

// Copy small CU to bigger CU.
// One of quarter parts overwritten by predicted sub part.
Void TComDataCU::copyPartFrom( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth )
{
  assert( uiPartUnitIdx<4 );

  m_dTotalCost         += pcCU->getTotalCost();
  m_uiTotalDistortion  += pcCU->getTotalDistortion();
  m_uiTotalBits        += pcCU->getTotalBits();

  UInt uiOffset         = pcCU->getTotalNumPart()*uiPartUnitIdx;
  const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();
  const UInt numValidChan=pcCU->getPic()->getChromaFormat()==CHROMA_400 ? 1:2;

  UInt uiNumPartition = pcCU->getTotalNumPart();
  Int iSizeInUchar  = sizeof( UChar ) * uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * uiNumPartition;

  Int sizeInChar  = sizeof( Char ) * uiNumPartition;
#if SKIP_FLAG
  memcpy( m_skipFlag   + uiOffset, pcCU->getSkipFlag(),       sizeof( *m_skipFlag )   * uiNumPartition );
#endif
  memcpy( m_phQP       + uiOffset, pcCU->getQP(),             sizeInChar                        );
  memcpy( m_pePartSize + uiOffset, pcCU->getPartitionSize(),  sizeof( *m_pePartSize ) * uiNumPartition );
  memcpy( m_pePredMode + uiOffset, pcCU->getPredictionMode(), sizeof( *m_pePredMode ) * uiNumPartition );
  memcpy( m_CUTransquantBypass + uiOffset, pcCU->getCUTransquantBypass(), sizeof( *m_CUTransquantBypass ) * uiNumPartition );

#if !REMOVE_ALF
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    m_lcuAlfEnabled[comp] = pcCU->m_lcuAlfEnabled[comp];
  }
#endif

  memcpy( m_pbMergeFlag         + uiOffset, pcCU->getMergeFlag(),         iSizeInBool  );
  memcpy( m_puhMergeIndex       + uiOffset, pcCU->getMergeIndex(),        iSizeInUchar );
  for (UInt ch=0; ch<numValidChan; ch++)
  {
    memcpy( m_puhIntraDir[ch]   + uiOffset, pcCU->getIntraDir(ChannelType(ch)), iSizeInUchar );
  }

  memcpy( m_puhInterDir         + uiOffset, pcCU->getInterDir(),          iSizeInUchar );
  memcpy( m_puhTrIdx            + uiOffset, pcCU->getTransformIdx(),      iSizeInUchar );
#if !REMOVE_NSQT
  memcpy( m_nsqtPartIdx         + uiOffset, pcCU->getNSQTPartIdx(),       iSizeInUchar );
#endif
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    memcpy( m_puhTransformSkip[comp] + uiOffset, pcCU->getTransformSkip(ComponentID(comp)),     iSizeInUchar );
    memcpy( m_puhCbf[comp] + uiOffset, pcCU->getCbf(ComponentID(comp))    , iSizeInUchar );
  }

  memcpy( m_puhDepth  + uiOffset, pcCU->getDepth(),  iSizeInUchar );
  memcpy( m_puhWidth  + uiOffset, pcCU->getWidth(),  iSizeInUchar );
  memcpy( m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar );

  memcpy( m_pbIPCMFlag + uiOffset, pcCU->getIPCMFlag(), iSizeInBool );

  m_pcCUAboveLeft      = pcCU->getCUAboveLeft();
  m_pcCUAboveRight     = pcCU->getCUAboveRight();
  m_pcCUAbove          = pcCU->getCUAbove();
  m_pcCULeft           = pcCU->getCULeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    memcpy( m_apiMVPIdx[rpl] + uiOffset, pcCU->getMVPIdx(rpl), iSizeInUchar );
    memcpy( m_apiMVPNum[rpl] + uiOffset, pcCU->getMVPNum(rpl), iSizeInUchar );
    m_apcCUColocated[rpl] = pcCU->getCUColocated(rpl);
    m_acCUMvField[rpl].copyFrom( pcCU->getCUMvField( rpl ), pcCU->getTotalNumPart(), uiOffset );
  }

  const UInt numCoeffY = g_uiMaxCUWidth*g_uiMaxCUHeight >> (uiDepth<<1);
  const UInt offsetY   = uiPartUnitIdx*numCoeffY;
  for (UInt ch=0; ch<numValidComp; ch++)
  {
    const ComponentID component = ComponentID(ch);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    const UInt offset           = offsetY>>componentShift;
    memcpy( m_pcTrCoeff [ch] + offset, pcCU->getCoeff(component),    sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memcpy( m_pcArlCoeff[ch] + offset, pcCU->getArlCoeff(component), sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memcpy( m_pcIPCMSample[ch] + offset, pcCU->getPCMSample(component), sizeof(Pel)*(numCoeffY>>componentShift) );
  }

  m_uiTotalBins += pcCU->getTotalBins();
  memcpy( m_uiSliceStartCU        + uiOffset, pcCU->m_uiSliceStartCU,        sizeof( UInt ) * uiNumPartition  );
  memcpy( m_uiDependentSliceStartCU + uiOffset, pcCU->m_uiDependentSliceStartCU, sizeof( UInt ) * uiNumPartition  );
}

// Copy current predicted part to a CU in picture.
// It is used to predict for next part
Void TComDataCU::copyToPic( UChar uhDepth )
{
  TComDataCU*& rpcCU = m_pcPic->getCU( m_uiCUAddr );
  const UInt numValidComp=rpcCU->getPic()->getNumberValidComponents();
  const UInt numValidChan=rpcCU->getPic()->getChromaFormat()==CHROMA_400 ? 1:2;

  rpcCU->getTotalCost()       = m_dTotalCost;
  rpcCU->getTotalDistortion() = m_uiTotalDistortion;
  rpcCU->getTotalBits()       = m_uiTotalBits;

  Int iSizeInUchar  = sizeof( UChar ) * m_uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * m_uiNumPartition;

  Int sizeInChar  = sizeof( Char ) * m_uiNumPartition;

#if SKIP_FLAG
  memcpy( rpcCU->getSkipFlag() + m_uiAbsIdxInLCU, m_skipFlag, sizeof( *m_skipFlag ) * m_uiNumPartition );
#endif

  memcpy( rpcCU->getQP() + m_uiAbsIdxInLCU, m_phQP, sizeInChar  );

  memcpy( rpcCU->getPartitionSize()  + m_uiAbsIdxInLCU, m_pePartSize, sizeof( *m_pePartSize ) * m_uiNumPartition );
  memcpy( rpcCU->getPredictionMode() + m_uiAbsIdxInLCU, m_pePredMode, sizeof( *m_pePredMode ) * m_uiNumPartition );
  memcpy( rpcCU->getCUTransquantBypass()+ m_uiAbsIdxInLCU, m_CUTransquantBypass, sizeof( *m_CUTransquantBypass ) * m_uiNumPartition );

#if !REMOVE_ALF
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    rpcCU->m_lcuAlfEnabled[comp] = m_lcuAlfEnabled[comp];
  }
#endif

  memcpy( rpcCU->getMergeFlag()         + m_uiAbsIdxInLCU, m_pbMergeFlag,         iSizeInBool  );
  memcpy( rpcCU->getMergeIndex()        + m_uiAbsIdxInLCU, m_puhMergeIndex,       iSizeInUchar );
  for (UInt ch=0; ch<numValidChan; ch++)
  {
    memcpy( rpcCU->getIntraDir(ChannelType(ch)) + m_uiAbsIdxInLCU, m_puhIntraDir[ch], iSizeInUchar);
  }

  memcpy( rpcCU->getInterDir()          + m_uiAbsIdxInLCU, m_puhInterDir,         iSizeInUchar );
  memcpy( rpcCU->getTransformIdx()      + m_uiAbsIdxInLCU, m_puhTrIdx,            iSizeInUchar );
#if !REMOVE_NSQT
  memcpy( rpcCU->getNSQTPartIdx()       + m_uiAbsIdxInLCU, m_nsqtPartIdx,         iSizeInUchar );
#endif
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    memcpy( rpcCU->getTransformSkip(ComponentID(comp))  + m_uiAbsIdxInLCU, m_puhTransformSkip[comp], iSizeInUchar );
    memcpy( rpcCU->getCbf(ComponentID(comp))     + m_uiAbsIdxInLCU, m_puhCbf[comp], iSizeInUchar );
  }

  memcpy( rpcCU->getDepth()  + m_uiAbsIdxInLCU, m_puhDepth,  iSizeInUchar );
  memcpy( rpcCU->getWidth()  + m_uiAbsIdxInLCU, m_puhWidth,  iSizeInUchar );
  memcpy( rpcCU->getHeight() + m_uiAbsIdxInLCU, m_puhHeight, iSizeInUchar );

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    memcpy( rpcCU->getMVPIdx(rpl) + m_uiAbsIdxInLCU, m_apiMVPIdx[rpl], iSizeInUchar );
    memcpy( rpcCU->getMVPNum(rpl) + m_uiAbsIdxInLCU, m_apiMVPNum[rpl], iSizeInUchar );
    m_acCUMvField[rpl].copyTo( rpcCU->getCUMvField( rpl ), m_uiAbsIdxInLCU );
  }


  memcpy( rpcCU->getIPCMFlag() + m_uiAbsIdxInLCU, m_pbIPCMFlag,         iSizeInBool  );

  const UInt numCoeffY    = (g_uiMaxCUWidth*g_uiMaxCUHeight)>>(uhDepth<<1);
  const UInt offsetY      = m_uiAbsIdxInLCU*m_pcPic->getMinCUWidth()*m_pcPic->getMinCUHeight();
  for (UInt comp=0; comp<numValidComp; comp++)
  {
    const ComponentID component = ComponentID(comp);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    memcpy( rpcCU->getCoeff(component)   + (offsetY>>componentShift), m_pcTrCoeff[component], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memcpy( rpcCU->getArlCoeff(component) + (offsetY>>componentShift), m_pcArlCoeff[component], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memcpy( rpcCU->getPCMSample(component) + (offsetY>>componentShift), m_pcIPCMSample[component], sizeof(Pel)*(numCoeffY>>componentShift) );
  }

  rpcCU->getTotalBins() = m_uiTotalBins;
  memcpy( rpcCU->m_uiSliceStartCU        + m_uiAbsIdxInLCU, m_uiSliceStartCU,        sizeof( UInt ) * m_uiNumPartition  );
  memcpy( rpcCU->m_uiDependentSliceStartCU + m_uiAbsIdxInLCU, m_uiDependentSliceStartCU, sizeof( UInt ) * m_uiNumPartition  );
}

Void TComDataCU::copyToPic( UChar uhDepth, UInt uiPartIdx, UInt uiPartDepth )
{
  TComDataCU*&  rpcCU       = m_pcPic->getCU( m_uiCUAddr );
  UInt          uiQNumPart  = m_uiNumPartition>>(uiPartDepth<<1);

  UInt uiPartStart          = uiPartIdx*uiQNumPart;
  UInt uiPartOffset         = m_uiAbsIdxInLCU + uiPartStart;

  const UInt numValidComp=rpcCU->getPic()->getNumberValidComponents();
  const UInt numValidChan=rpcCU->getPic()->getChromaFormat()==CHROMA_400 ? 1:2;

  rpcCU->getTotalCost()       = m_dTotalCost;
  rpcCU->getTotalDistortion() = m_uiTotalDistortion;
  rpcCU->getTotalBits()       = m_uiTotalBits;

  Int iSizeInUchar  = sizeof( UChar  ) * uiQNumPart;
  Int iSizeInBool   = sizeof( Bool   ) * uiQNumPart;

  Int sizeInChar  = sizeof( Char ) * uiQNumPart;
#if SKIP_FLAG
  memcpy( rpcCU->getSkipFlag()       + uiPartOffset, m_skipFlag,   sizeof( *m_skipFlag )   * uiQNumPart );
#endif

  memcpy( rpcCU->getQP() + uiPartOffset, m_phQP, sizeInChar );
  memcpy( rpcCU->getPartitionSize()  + uiPartOffset, m_pePartSize, sizeof( *m_pePartSize ) * uiQNumPart );
  memcpy( rpcCU->getPredictionMode() + uiPartOffset, m_pePredMode, sizeof( *m_pePredMode ) * uiQNumPart );

  memcpy( rpcCU->getCUTransquantBypass()+ uiPartOffset, m_CUTransquantBypass, sizeof( *m_CUTransquantBypass ) * uiQNumPart );

#if !REMOVE_ALF
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    rpcCU->m_lcuAlfEnabled[comp] = m_lcuAlfEnabled[comp];
  }
#endif

  memcpy( rpcCU->getMergeFlag()         + uiPartOffset, m_pbMergeFlag,         iSizeInBool  );
  memcpy( rpcCU->getMergeIndex()        + uiPartOffset, m_puhMergeIndex,       iSizeInUchar );
  for (UInt ch=0; ch<numValidChan; ch++)
  {
    memcpy( rpcCU->getIntraDir(ChannelType(ch)) + uiPartOffset, m_puhIntraDir[ch], iSizeInUchar );
  }

  memcpy( rpcCU->getInterDir()          + uiPartOffset, m_puhInterDir,         iSizeInUchar );
  memcpy( rpcCU->getTransformIdx()      + uiPartOffset, m_puhTrIdx,            iSizeInUchar );
#if !REMOVE_NSQT
  memcpy( rpcCU->getNSQTPartIdx()       + uiPartOffset, m_nsqtPartIdx,         iSizeInUchar );
#endif
  for(UInt comp=0; comp<numValidComp; comp++)
  {
    memcpy( rpcCU->getTransformSkip(ComponentID(comp) ) + uiPartOffset, m_puhTransformSkip[comp], iSizeInUchar );
    memcpy( rpcCU->getCbf(ComponentID(comp))            + uiPartOffset, m_puhCbf[comp], iSizeInUchar );
  }

  memcpy( rpcCU->getDepth()  + uiPartOffset, m_puhDepth,  iSizeInUchar );
  memcpy( rpcCU->getWidth()  + uiPartOffset, m_puhWidth,  iSizeInUchar );
  memcpy( rpcCU->getHeight() + uiPartOffset, m_puhHeight, iSizeInUchar );

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    memcpy( rpcCU->getMVPIdx(rpl) + uiPartOffset, m_apiMVPIdx[rpl], iSizeInUchar );
    memcpy( rpcCU->getMVPNum(rpl) + uiPartOffset, m_apiMVPNum[rpl], iSizeInUchar );
    m_acCUMvField[rpl].copyTo( rpcCU->getCUMvField( rpl ), m_uiAbsIdxInLCU, uiPartStart, uiQNumPart );
  }

  memcpy( rpcCU->getIPCMFlag() + uiPartOffset, m_pbIPCMFlag,         iSizeInBool  );

  const UInt numCoeffY    = (g_uiMaxCUWidth*g_uiMaxCUHeight)>>((uhDepth+uiPartDepth)<<1);
  const UInt offsetY      = uiPartOffset*m_pcPic->getMinCUWidth()*m_pcPic->getMinCUHeight();
  for (UInt comp=0; comp<numValidComp; comp++)
  {
    UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(comp)) + m_pcPic->getComponentScaleY(ComponentID(comp));
    memcpy( rpcCU->getCoeff(ComponentID(comp)) + (offsetY>>componentShift), m_pcTrCoeff[comp], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memcpy( rpcCU->getArlCoeff(ComponentID(comp)) + (offsetY>>componentShift), m_pcArlCoeff[comp], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memcpy( rpcCU->getPCMSample(ComponentID(comp)) + (offsetY>>componentShift), m_pcIPCMSample[comp], sizeof(Pel)*(numCoeffY>>componentShift) );
  }

  rpcCU->getTotalBins() = m_uiTotalBins;
  memcpy( rpcCU->m_uiSliceStartCU        + uiPartOffset, m_uiSliceStartCU,        sizeof( UInt ) * uiQNumPart  );
  memcpy( rpcCU->m_uiDependentSliceStartCU + uiPartOffset, m_uiDependentSliceStartCU, sizeof( UInt ) * uiQNumPart  );
}

// --------------------------------------------------------------------------------------------------------------------
// Other public functions
// --------------------------------------------------------------------------------------------------------------------

TComDataCU* TComDataCU::getPULeft( UInt& uiLPartUnitIdx,
                                   UInt uiCurrPartUnitIdx,
                                   Bool bEnforceSliceRestriction,
                                   Bool bEnforceDependentSliceRestriction,
                                   Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];
    if ( RasterAddress::isEqualCol( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
#if !REMOVE_FGS
      TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
      if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
        ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
      {
        return NULL;
      }
#endif
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiLPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
      if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
        ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
      {
        return NULL;
      }
#endif
      return this;
    }
  }

  uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + uiNumPartInCUWidth - 1 ];


  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || m_pcCULeft->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceDependentSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || m_pcCULeft->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction && ( m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))  )  )
      )
  {
    return NULL;
  }
  return m_pcCULeft;
}


TComDataCU* TComDataCU::getPUAbove( UInt& uiAPartUnitIdx,
                                    UInt uiCurrPartUnitIdx,
                                    Bool bEnforceSliceRestriction,
                                    Bool bEnforceDependentSliceRestriction,
                                    Bool MotionDataCompresssion,
                                    Bool planarAtLCUBoundary ,
                                    Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - uiNumPartInCUWidth ];
    if ( RasterAddress::isEqualRow( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
    {
#if !REMOVE_FGS
      TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
      if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
          ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
      {
        return NULL;
      }
#endif
      return m_pcPic->getCU( getAddr() );
    }
    else
    {
      uiAPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
      if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
        ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
      {
        return NULL;
      }
#endif
      return this;
    }
  }

  if(planarAtLCUBoundary)
  {
    return NULL;
  }

  uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth ];
  if(MotionDataCompresssion)
  {
    uiAPartUnitIdx = g_motionRefer[uiAPartUnitIdx];
  }

  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || m_pcCUAbove->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceDependentSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || m_pcCUAbove->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)))
      ||
       (bEnforceTileRestriction &&(m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL || (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))))
      )
  {
    return NULL;
  }
  return m_pcCUAbove;
}

TComDataCU* TComDataCU::getPUAboveLeft( UInt& uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction, Bool MotionDataCompresssion )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_uiAbsIdxInLCU];
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
    {
      uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - uiNumPartInCUWidth - 1 ];
      if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdx, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
      {
#if !REMOVE_FGS
        TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
        if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
            ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
        {
          return NULL;
        }
#endif
        return m_pcPic->getCU( getAddr() );
      }
      else
      {
        uiALPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
        if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
          ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
        {
          return NULL;
        }
#endif
        return this;
      }
    }
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + getPic()->getNumPartInCU() - uiNumPartInCUWidth - 1 ];
    if(MotionDataCompresssion)
    {
      uiALPartUnitIdx = g_motionRefer[uiALPartUnitIdx];
    }
  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }

  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, uiNumPartInCUWidth ) )
  {
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];
  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }

  uiALPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - 1 ];
  if(MotionDataCompresssion)
  {
    uiALPartUnitIdx = g_motionRefer[uiALPartUnitIdx];
  }
  if ( (bEnforceSliceRestriction && (m_pcCUAboveLeft==NULL || m_pcCUAboveLeft->getSlice()==NULL ||
       m_pcCUAboveLeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveLeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAboveLeft==NULL || m_pcCUAboveLeft->getSlice()==NULL ||
       m_pcCUAboveLeft->getSCUAddr()+uiALPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveLeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveLeft;
}

TComDataCU* TComDataCU::getPUAboveRight( UInt& uiARPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction, Bool MotionDataCompresssion )
{
  UInt uiAbsPartIdxRT     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxRT] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanCol( uiAbsPartIdxRT, uiNumPartInCUWidth - 1, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + 1 ] )
      {
        uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxRT, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
        {
#if !REMOVE_FGS
          TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
          if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
            ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
          {
            return NULL;
          }
#endif
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiARPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
          if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
            ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
          {
            return NULL;
          }
#endif
          return this;
        }
      }
      uiARPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + 1 ];
    if(MotionDataCompresssion)
    {
      uiARPartUnitIdx = g_motionRefer[uiARPartUnitIdx];
    }

  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }

  if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  uiARPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - uiNumPartInCUWidth ];
  if(MotionDataCompresssion)
  {
    uiARPartUnitIdx = g_motionRefer[uiARPartUnitIdx];
  }
  if ( (bEnforceSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveRight;
}

TComDataCU* TComDataCU::getPUBelowLeft( UInt& uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction )
{
  UInt uiAbsPartIdxLB     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdxLB = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + (m_puhHeight[0] / m_pcPic->getMinCUHeight() - 1)*m_pcPic->getNumPartInWidth();
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxLB] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
  {
    uiBLPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanRow( uiAbsPartIdxLB, m_pcPic->getNumPartInHeight() - 1, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroCol( uiAbsPartIdxLB, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth - 1 ] )
      {
        uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth - 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxLB, uiAbsZorderCUIdxLB, uiNumPartInCUWidth ) )
        {
#if !REMOVE_FGS
          TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
          if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
            ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
          {
            return NULL;
          }
#endif
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiBLPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
          if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
            ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
          {
            return NULL;
          }
#endif
          return this;
        }
      }
      uiBLPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiNumPartInCUWidth*2 - 1 ];
  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }

  uiBLPartUnitIdx = MAX_UINT;
  return NULL;
}

TComDataCU* TComDataCU::getPUBelowLeftAdi(UInt& uiBLPartUnitIdx, UInt uiPuHeight,  UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction )
{
  UInt uiAbsPartIdxLB     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdxLB = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ((m_puhHeight[0] / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInWidth();
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxLB] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples())
  {
    uiBLPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanRow( uiAbsPartIdxLB, m_pcPic->getNumPartInHeight() - uiPartUnitOffset, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroCol( uiAbsPartIdxLB, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * uiNumPartInCUWidth - 1 ] )
      {
        uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * uiNumPartInCUWidth - 1 ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxLB, uiAbsZorderCUIdxLB, uiNumPartInCUWidth ) )
        {
#if !REMOVE_FGS
          TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
          if ((bEnforceSliceRestriction        && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx)))
              ||(bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))
          {
            return NULL;
          }
#endif
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiBLPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
          if ((bEnforceSliceRestriction        && (this->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (uiCurrPartUnitIdx) || m_pcSlice==NULL))
              ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
          {
            return NULL;
          }
#endif
          return this;
        }
      }
      uiBLPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + (1+uiPartUnitOffset) * uiNumPartInCUWidth - 1 ];
  if ( (bEnforceSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCULeft==NULL || m_pcCULeft->getSlice()==NULL ||
       m_pcCULeft->getSCUAddr()+uiBLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCULeft->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCULeft;
  }

  uiBLPartUnitIdx = MAX_UINT;
  return NULL;
}

TComDataCU* TComDataCU::getPUAboveRightAdi(UInt&  uiARPartUnitIdx, UInt uiPuWidth, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction )
{
  UInt uiAbsPartIdxRT     = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + (m_puhWidth[0] / m_pcPic->getMinCUWidth()) - 1;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

  if( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxRT] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanCol( uiAbsPartIdxRT, uiNumPartInCUWidth - uiPartUnitOffset, uiNumPartInCUWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + uiPartUnitOffset ] )
      {
        uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT - uiNumPartInCUWidth + uiPartUnitOffset ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxRT, uiAbsZorderCUIdx, uiNumPartInCUWidth ) )
        {
#if !REMOVE_FGS
          TComDataCU* pcTempReconCU = m_pcPic->getCU( getAddr() );
          if ((bEnforceSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)))
           ||( bEnforceDependentSliceRestriction && (pcTempReconCU==NULL || pcTempReconCU->getSlice() == NULL || pcTempReconCU->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx))))

          {
            return NULL;
          }
#endif
          return m_pcPic->getCU( getAddr() );
        }
        else
        {
          uiARPartUnitIdx -= m_uiAbsIdxInLCU;
#if !REMOVE_FGS
          if ((bEnforceSliceRestriction && (this->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL))
            ||(bEnforceDependentSliceRestriction && (this->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx) || m_pcSlice==NULL)))
          {
            return NULL;
          }
#endif
          return this;
        }
      }
      uiARPartUnitIdx = MAX_UINT;
      return NULL;
    }
    uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT + m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + uiPartUnitOffset ];
  if ( (bEnforceSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAbove==NULL || m_pcCUAbove->getSlice()==NULL ||
       m_pcCUAbove->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAbove->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
    {
      return NULL;
    }
    return m_pcCUAbove;
  }

  if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, uiNumPartInCUWidth ) )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  uiARPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartInCU() - uiNumPartInCUWidth + uiPartUnitOffset-1 ];
  if ( (bEnforceSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))||
       (bEnforceDependentSliceRestriction && (m_pcCUAboveRight==NULL || m_pcCUAboveRight->getSlice()==NULL ||
       m_pcPic->getPicSym()->getInverseCUOrderMap( m_pcCUAboveRight->getAddr()) > m_pcPic->getPicSym()->getInverseCUOrderMap( getAddr()) ||
       m_pcCUAboveRight->getSCUAddr()+uiARPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiCurrPartUnitIdx)||
       (m_pcPic->getPicSym()->getTileIdxMap( m_pcCUAboveRight->getAddr() ) != m_pcPic->getPicSym()->getTileIdxMap(getAddr()))
       ))
     )
  {
    return NULL;
  }
  return m_pcCUAboveRight;
}

/** Get left QpMinCu
*\param   uiLPartUnitIdx
*\param   uiCurrAbsIdxInLCU
*\param   bEnforceSliceRestriction
*\param   bEnforceDependentSliceRestriction
*\returns TComDataCU*   point of TComDataCU of left QpMinCu
*/
TComDataCU* TComDataCU::getQpMinCuLeft( UInt& uiLPartUnitIdx, UInt uiCurrAbsIdxInLCU, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction)
{
  UInt numPartInCUWidth = m_pcPic->getNumPartInWidth();
  UInt absZorderQpMinCUIdx = (uiCurrAbsIdxInLCU>>((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))<<((g_uiMaxCUDepth -getSlice()->getPPS()->getMaxCuDQPDepth())<<1);
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for left LCU boundary
  if ( RasterAddress::isZeroCol(absRorderQpMinCUIdx, numPartInCUWidth) )
  {
    return NULL;
  }

  // get index of left-CU relative to top-left corner of current quantization group
  uiLPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - 1];

#if !REMOVE_FGS
  // check for fine-grain slice boundaries
  if ((bEnforceSliceRestriction        && (m_pcPic->getCU( getAddr() )->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (absZorderQpMinCUIdx)))
    ||(bEnforceDependentSliceRestriction && (m_pcPic->getCU( getAddr() )->getSCUAddr()+uiLPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(absZorderQpMinCUIdx))))
  {
    return NULL;
  }
#endif

  // return pointer to current LCU
  return m_pcPic->getCU( getAddr() );

}

/** Get Above QpMinCu
*\param   uiAPartUnitIdx
*\param   uiCurrAbsIdxInLCU
*\param   bEnforceSliceRestriction
*\param   bEnforceDependentSliceRestriction
*\returns TComDataCU*   point of TComDataCU of above QpMinCu
*/
TComDataCU* TComDataCU::getQpMinCuAbove( UInt& uiAPartUnitIdx, UInt uiCurrAbsIdxInLCU, Bool bEnforceSliceRestriction, Bool bEnforceDependentSliceRestriction )
{
  UInt numPartInCUWidth = m_pcPic->getNumPartInWidth();
  UInt absZorderQpMinCUIdx = (uiCurrAbsIdxInLCU>>((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))<<((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1);
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for top LCU boundary
  if ( RasterAddress::isZeroRow( absRorderQpMinCUIdx, numPartInCUWidth) )
  {
    return NULL;
  }

  // get index of top-CU relative to top-left corner of current quantization group
  uiAPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - numPartInCUWidth];

#if !REMOVE_FGS
  // check for fine-grain slice boundaries
  if ((bEnforceSliceRestriction        && (m_pcPic->getCU( getAddr() )->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getSliceStartCU       (absZorderQpMinCUIdx)))
    ||(bEnforceDependentSliceRestriction && (m_pcPic->getCU( getAddr() )->getSCUAddr()+uiAPartUnitIdx < m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(absZorderQpMinCUIdx))))
  {
    return NULL;
  }
#endif

  // return pointer to current LCU
  return m_pcPic->getCU( getAddr() );
}



/** Get reference QP from left QpMinCu or latest coded QP
*\param   uiCurrAbsIdxInLCU
*\returns Char   reference QP value
*/
Char TComDataCU::getRefQP( UInt uiCurrAbsIdxInLCU )
{
  UInt lPartIdx = MAX_UINT;
  UInt aPartIdx = MAX_UINT;
  TComDataCU* cULeft  = getQpMinCuLeft ( lPartIdx, m_uiAbsIdxInLCU + uiCurrAbsIdxInLCU );
  TComDataCU* cUAbove = getQpMinCuAbove( aPartIdx, m_uiAbsIdxInLCU + uiCurrAbsIdxInLCU );
  return (((cULeft? cULeft->getQP( lPartIdx ): getLastCodedQP( uiCurrAbsIdxInLCU )) + (cUAbove? cUAbove->getQP( aPartIdx ): getLastCodedQP( uiCurrAbsIdxInLCU )) + 1) >> 1);
}

Int TComDataCU::getLastValidPartIdx( Int iAbsPartIdx )
{
  Int iLastValidPartIdx = iAbsPartIdx-1;
  while ( iLastValidPartIdx >= 0
       && getPredictionMode( iLastValidPartIdx ) == NUMBER_OF_PREDICTION_MODES )
  {
    UInt uiDepth = getDepth( iLastValidPartIdx );
    iLastValidPartIdx -= m_uiNumPartition>>(uiDepth<<1);
  }
  return iLastValidPartIdx;
}

Char TComDataCU::getLastCodedQP( UInt uiAbsPartIdx )
{
  UInt uiQUPartIdxMask = ~((1<<((g_uiMaxCUDepth - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))-1);
  Int iLastValidPartIdx = getLastValidPartIdx( uiAbsPartIdx&uiQUPartIdxMask );
  if ( uiAbsPartIdx < m_uiNumPartition
    && (getSCUAddr()+iLastValidPartIdx < getSliceStartCU(m_uiAbsIdxInLCU+uiAbsPartIdx) || getSCUAddr()+iLastValidPartIdx < getDependentSliceStartCU(m_uiAbsIdxInLCU+uiAbsPartIdx) ))
  {
    return getSlice()->getSliceQp();
  }
  else
  if ( iLastValidPartIdx >= 0 )
  {
    return getQP( iLastValidPartIdx );
  }
  else
  {
    if ( getZorderIdxInCU() > 0 )
    {
      return getPic()->getCU( getAddr() )->getLastCodedQP( getZorderIdxInCU() );
    }
    else if ( getPic()->getPicSym()->getInverseCUOrderMap(getAddr()) > 0
      && getPic()->getPicSym()->getTileIdxMap(getAddr()) == getPic()->getPicSym()->getTileIdxMap(getPic()->getPicSym()->getCUOrderMap(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())-1))
      && !( getSlice()->getPPS()->getTilesOrEntropyCodingSyncIdc() == 2 && getAddr() % getPic()->getFrameWidthInCU() == 0 )
      )
    {
      return getPic()->getCU( getPic()->getPicSym()->getCUOrderMap(getPic()->getPicSym()->getInverseCUOrderMap(getAddr())-1) )->getLastCodedQP( getPic()->getNumPartInCU() );
    }
    else
    {
      return getSlice()->getSliceQp();
    }
  }
}


/** Check whether the CU is coded in lossless coding mode
 * \param   uiAbsPartIdx
 * \returns true if the CU is coded in lossless coding mode; false if otherwise 
 */
Bool TComDataCU::isLosslessCoded(UInt absPartIdx)
{
  return (getSlice()->getPPS()->getTransquantBypassEnableFlag() && getCUTransquantBypass (absPartIdx));
}


/** Get allowed chroma intra modes
*\param   uiAbsPartIdx
*\param   uiModeList  pointer to chroma intra modes array
*\returns
*- fill uiModeList with chroma intra modes
*/
Void TComDataCU::getAllowedChromaDir( UInt uiAbsPartIdx, UInt uiModeList[NUM_CHROMA_MODE] )
{
  uiModeList[0] = PLANAR_IDX;
  uiModeList[1] = VER_IDX;
  uiModeList[2] = HOR_IDX;
  uiModeList[3] = DC_IDX;
#if !REMOVE_LMCHROMA
  uiModeList[4] = LM_CHROMA_IDX;
  uiModeList[5] = DM_CHROMA_IDX;
  assert(5<NUM_CHROMA_MODE);
#else
  uiModeList[4] = DM_CHROMA_IDX;
  assert(4<NUM_CHROMA_MODE);
#endif

  UInt uiLumaMode = getIntraDir( CHANNEL_TYPE_LUMA, uiAbsPartIdx );

#if REMOVE_LMCHROMA
  for( Int i = 0; i < NUM_CHROMA_MODE - 1; i++ )
#else
  for( Int i = 0; i < NUM_CHROMA_MODE - 2; i++ )
#endif
  {
    if( uiLumaMode == uiModeList[i] )
    {
      uiModeList[i] = 34; // VER+8 mode
      break;
    }
  }
}

/** Get most probable intra modes
*\param   uiAbsPartIdx
*\param   uiIntraDirPred  pointer to the array for MPM storage
*\param   piMode          it is set with MPM mode in case both MPM are equal. It is used to restrict RD search at encode side.
*\returns Number of MPM
*/
Int TComDataCU::getIntraDirPredictor( UInt uiAbsPartIdx, Int uiIntraDirPred[NUM_MOST_PROBABLE_MODES], const ComponentID compID, Int* piMode  )
{
  TComDataCU* pcCULeft, *pcCUAbove;
  UInt        LeftPartIdx  = MAX_UINT;
  UInt        AbovePartIdx = MAX_UINT;
  Int         iLeftIntraDir, iAboveIntraDir;
  Int         uiPredNum = 0;

  const ChannelType chType = toChannelType(compID);
  const ChromaFormat chForm = getPic()->getChromaFormat();
  // Get intra direction of left PU
#if DEPENDENT_SLICES
  Bool bDepSliceRestriction = ( !m_pcSlice->getPPS()->getDependentSlicesEnabledFlag());
  pcCULeft = getPULeft( LeftPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction );
#else
  pcCULeft = getPULeft( LeftPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
#endif
  if (isChroma(compID)) LeftPartIdx = getChromasCorrespondingPULumaIdx(LeftPartIdx, chForm);
  iLeftIntraDir  = pcCULeft ? ( pcCULeft->isIntra( LeftPartIdx ) ? pcCULeft->getIntraDir( chType, LeftPartIdx ) : DC_IDX ) : DC_IDX;

  // Get intra direction of above PU
#if DEPENDENT_SLICES
  pcCUAbove = getPUAbove( AbovePartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction, false, true );
#else
  pcCUAbove = getPUAbove( AbovePartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, true, false, true );
#endif
  if (isChroma(compID)) AbovePartIdx = getChromasCorrespondingPULumaIdx(AbovePartIdx, chForm);
  iAboveIntraDir = pcCUAbove ? ( pcCUAbove->isIntra( AbovePartIdx ) ? pcCUAbove->getIntraDir( chType, AbovePartIdx ) : DC_IDX ) : DC_IDX;

  if (isChroma(chType))
  {
    if (iLeftIntraDir==DM_CHROMA_IDX)
    {
      iLeftIntraDir = pcCULeft->getIntraDir( CHANNEL_TYPE_LUMA, LeftPartIdx );
    }
    else if (iLeftIntraDir==LM_CHROMA_IDX)
    {
      iLeftIntraDir = PLANAR_IDX;
    }

    if (iAboveIntraDir==DM_CHROMA_IDX)
    {
      iAboveIntraDir = pcCUAbove->getIntraDir( CHANNEL_TYPE_LUMA, AbovePartIdx );
    }
    else if (iAboveIntraDir==LM_CHROMA_IDX)
    {
      iAboveIntraDir = PLANAR_IDX;
    }
  }

  assert (2<NUM_MOST_PROBABLE_MODES);
  uiPredNum = NUM_MOST_PROBABLE_MODES;
  if(iLeftIntraDir == iAboveIntraDir)
  {
    if( piMode )
    {
      *piMode = 1;
    }

    if (iLeftIntraDir > 1) // angular modes
    {
      uiIntraDirPred[0] = iLeftIntraDir;
      uiIntraDirPred[1] = ((iLeftIntraDir + 29) % 32) + 2;
      uiIntraDirPred[2] = ((iLeftIntraDir - 1 ) % 32) + 2;
    }
    else //non-angular
    {
      uiIntraDirPred[0] = PLANAR_IDX;
      uiIntraDirPred[1] = DC_IDX;
      uiIntraDirPred[2] = VER_IDX;
    }
  }
  else
  {
    if( piMode )
    {
      *piMode = 2;
    }
    uiIntraDirPred[0] = iLeftIntraDir;
    uiIntraDirPred[1] = iAboveIntraDir;

    if (iLeftIntraDir && iAboveIntraDir ) //both modes are non-planar
    {
      uiIntraDirPred[2] = PLANAR_IDX;
    }
    else
    {
      uiIntraDirPred[2] =  (iLeftIntraDir+iAboveIntraDir)<2? VER_IDX : DC_IDX;
    }
  }
  for (Int i=0; i<uiPredNum; i++)
    assert(uiIntraDirPred[i] < 35);

  return uiPredNum;
}

UInt TComDataCU::getCtxSplitFlag( UInt uiAbsPartIdx, UInt uiDepth )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx;
  // Get left split flag
#if DEPENDENT_SLICES
  Bool bDepSliceRestriction = ( !m_pcSlice->getPPS()->getDependentSlicesEnabledFlag());
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction );
#else
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
#endif
  uiCtx  = ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;
  
  // Get above split flag
#if DEPENDENT_SLICES
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction );
#else
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
#endif
  uiCtx += ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;
  
  return uiCtx;
}

UInt TComDataCU::getCtxQtCbf( TComTU &rTu, const ChannelType chType, const Bool useAdjustedDepth )
{
  const UInt transformDepth = useAdjustedDepth ? rTu.GetTransformDepthRelAdj(chType) : rTu.GetTransformDepthRel();

  if (useTransformDepthForCbfCtxSelection(rTu.GetChromaFormat(), chType))
  {
    return transformDepth;
  }
  else
  {
#if SIMPLE_LUMA_CBF_CTX_DERIVATION
    const UInt uiCtx = ( transformDepth == 0 ? 1 : 0 );
#else
    const UInt uiLog2TrafoSize = rTu.GetLog2LumaTrSize();
    const UInt uiCtx = transformDepth == 0 || uiLog2TrafoSize == getSlice()->getSPS()->getQuadtreeTULog2MaxSize() ? 1 : 0;
#endif
    return uiCtx;
  }
}

UInt TComDataCU::getQuadtreeTULog2MinSizeInCU( UInt absPartIdx )
{
  UInt log2CbSize = g_aucConvertToBit[getWidth( absPartIdx )] + 2;
  PartSize  partSize  = getPartitionSize( absPartIdx );
  UInt quadtreeTUMaxDepth = getPredictionMode( absPartIdx ) == MODE_INTRA ? m_pcSlice->getSPS()->getQuadtreeTUMaxDepthIntra() : m_pcSlice->getSPS()->getQuadtreeTUMaxDepthInter(); 
  Int intraSplitFlag = ( getPredictionMode( absPartIdx ) == MODE_INTRA && partSize == SIZE_NxN ) ? 1 : 0;
  Int interSplitFlag = ((quadtreeTUMaxDepth == 1) && (getPredictionMode( absPartIdx ) == MODE_INTER) && (partSize != SIZE_2Nx2N) );
  
  UInt log2MinTUSizeInCU = 0;
  if (log2CbSize < (m_pcSlice->getSPS()->getQuadtreeTULog2MinSize() + quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag) ) 
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is < QuadtreeTULog2MinSize
    log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MinSize();
  }
  else
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still >= QuadtreeTULog2MinSize
    log2MinTUSizeInCU = log2CbSize - ( quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag); // stop when trafoDepth == hierarchy_depth = splitFlag
    if ( log2MinTUSizeInCU > m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize())
    {
      // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still > QuadtreeTULog2MaxSize
      log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize();
    }  
  }
  return log2MinTUSizeInCU;
}

UInt TComDataCU::getCtxSkipFlag( UInt uiAbsPartIdx )
{
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
#if DEPENDENT_SLICES
  Bool bDepSliceRestriction = ( !m_pcSlice->getPPS()->getDependentSlicesEnabledFlag());
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction );
#else
  pcTempCU = getPULeft( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
#endif
  uiCtx    = ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;
  
  // Get BCBP of above PU
#if DEPENDENT_SLICES
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx, true, bDepSliceRestriction );
#else
  pcTempCU = getPUAbove( uiTempPartIdx, m_uiAbsIdxInLCU + uiAbsPartIdx );
#endif
  uiCtx   += ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;
  
  return uiCtx;
}

UInt TComDataCU::getCtxInterDir( UInt uiAbsPartIdx )
{
  return getDepth( uiAbsPartIdx );
}

UChar TComDataCU::getQtRootCbf( UInt uiIdx )
{ 
  const UInt numberValidComponents = getPic()->getNumberValidComponents();
  return getCbf( uiIdx, COMPONENT_Y, 0 )
          || ((numberValidComponents > COMPONENT_Cb) && getCbf( uiIdx, COMPONENT_Cb, 0 ))
          || ((numberValidComponents > COMPONENT_Cr) && getCbf( uiIdx, COMPONENT_Cr, 0 ));
}

Void TComDataCU::setCbfSubParts( const UInt uiCbf[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    memset( m_puhCbf[comp] + uiAbsPartIdx, uiCbf[comp], sizeof( UChar ) * uiCurrPartNumb );
  }
}

Void TComDataCU::setCbfSubParts( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  memset( m_puhCbf[compID] + uiAbsPartIdx, uiCbf, sizeof( UChar ) * uiCurrPartNumb );
}

/** Sets a coded block flag for all sub-partitions of a partition
 * \param uiCbf The value of the coded block flag to be set
 * \param eTType
 * \param uiAbsPartIdx
 * \param uiPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TComDataCU::setCbfSubParts ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiCbf, m_puhCbf[compID], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setDepthSubParts( UInt uiDepth, UInt uiAbsPartIdx )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  memset( m_puhDepth + uiAbsPartIdx, uiDepth, sizeof(UChar)*uiCurrPartNumb );
}

Bool TComDataCU::isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  return (((m_uiAbsIdxInLCU + uiAbsPartIdx)% uiPartNumb) == 0);
}

Void TComDataCU::setPartSizeSubParts( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  assert( sizeof( *m_pePartSize) == 1 );
  memset( m_pePartSize + uiAbsPartIdx, eMode, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setCUTransquantBypassSubParts( bool flag, UInt uiAbsPartIdx, UInt uiDepth )
{
  memset( m_CUTransquantBypass + uiAbsPartIdx, flag, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

#if SKIP_FLAG
Void TComDataCU::setSkipFlagSubParts( Bool skip, UInt absPartIdx, UInt depth )
{
  assert( sizeof( *m_skipFlag) == 1 );
  memset( m_skipFlag + absPartIdx, skip, m_pcPic->getNumPartInCU() >> ( 2 * depth ) );
}
#endif

Void TComDataCU::setPredModeSubParts( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth )
{
  assert( sizeof( *m_pePartSize) == 1 );
  memset( m_pePredMode + uiAbsPartIdx, eMode, m_pcPic->getNumPartInCU() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setQPSubCUs( Int qp, TComDataCU* pcCU, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf )
{
  UInt currPartNumb = m_pcPic->getNumPartInCU() >> (depth << 1);
  UInt currPartNumQ = currPartNumb >> 2;
  const UInt numValidComp = m_pcPic->getNumberValidComponents();

  if(!foundNonZeroCbf)
  {
    if(pcCU->getDepth(absPartIdx) > depth)
    {
      for ( UInt partUnitIdx = 0; partUnitIdx < 4; partUnitIdx++ )
      {
        pcCU->setQPSubCUs( qp, pcCU, absPartIdx+partUnitIdx*currPartNumQ, depth+1, foundNonZeroCbf );
      }
    }
    else
    {
      if(pcCU->getCbf( absPartIdx, COMPONENT_Y ) || (numValidComp>COMPONENT_Cb && pcCU->getCbf( absPartIdx, COMPONENT_Cb )) || (numValidComp>COMPONENT_Cr && pcCU->getCbf( absPartIdx, COMPONENT_Cr) ) )
      {
        foundNonZeroCbf = true;
      }
      else
      {
        setQPSubParts(qp, absPartIdx, depth);
      }
    }
  }
}

Void TComDataCU::setQPSubParts( Int qp, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);
  TComSlice * pcSlice = getPic()->getSlice(getPic()->getCurrSliceIdx());

  for(UInt uiSCUIdx = uiAbsPartIdx; uiSCUIdx < uiAbsPartIdx+uiCurrPartNumb; uiSCUIdx++)
  {
    if( m_pcPic->getCU( getAddr() )->getDependentSliceStartCU(uiSCUIdx+getZorderIdxInCU()) == pcSlice->getDependentSliceCurStartCUAddr() )
    {
      m_phQP[uiSCUIdx] = qp;
    }
  }
}

Void TComDataCU::setIntraDirSubParts( const ChannelType channelType, const UInt dir, const UInt absPartIdx, const UInt depth )
{
  UInt numPart = m_pcPic->getNumPartInCU() >> (depth << 1);
  memset( m_puhIntraDir[channelType] + absPartIdx, dir,sizeof(UChar)*numPart );
}

template<typename T>
Void TComDataCU::setSubPart( T uiParameter, T* puhBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx )
{
  assert( sizeof(T) == 1 ); // Using memset() works only for types of size 1

  UInt uiCurrPartNumQ = (m_pcPic->getNumPartInCU() >> (2 * uiCUDepth)) >> 2;
  switch ( m_pePartSize[ uiCUAddr ] )
  {
    case SIZE_2Nx2N:
      memset( puhBaseLCU + uiCUAddr, uiParameter, 4 * uiCurrPartNumQ );
      break;
    case SIZE_2NxN:
      memset( puhBaseLCU + uiCUAddr, uiParameter, 2 * uiCurrPartNumQ );
      break;
    case SIZE_Nx2N:
      memset( puhBaseLCU + uiCUAddr, uiParameter, uiCurrPartNumQ );
      memset( puhBaseLCU + uiCUAddr + 2 * uiCurrPartNumQ, uiParameter, uiCurrPartNumQ );
      break;
    case SIZE_NxN:
      memset( puhBaseLCU + uiCUAddr, uiParameter, uiCurrPartNumQ );
      break;
    case SIZE_2NxnU:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, ((uiCurrPartNumQ >> 1) + (uiCurrPartNumQ << 1)) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, ((uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1)) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N:
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseLCU + uiCUAddr + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else if ( uiPUIdx == 1 )
      {
        memset( puhBaseLCU + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseLCU + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert( 0 );
      break;
  }
}

Void TComDataCU::setMergeFlagSubParts ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart( bMergeFlag, m_pbMergeFlag, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiMergeIndex, m_puhMergeIndex, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setInterDirSubParts( UInt uiDir, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<UChar>( uiDir, m_puhInterDir, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPIdxSubParts( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<Char>( iMVPIdx, m_apiMVPIdx[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPNumSubParts( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )
{
  setSubPart<Char>( iMVPNum, m_apiMVPNum[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}


Void TComDataCU::setTrIdxSubParts( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset( m_puhTrIdx + uiAbsPartIdx, uiTrIdx, sizeof(UChar)*uiCurrPartNumb );
}

Void TComDataCU::setTransformSkipSubParts( const UInt useTransformSkip[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    memset( m_puhTransformSkip[i] + uiAbsPartIdx, useTransformSkip[i], sizeof( UChar ) * uiCurrPartNumb );
  }
}

Void TComDataCU::setTransformSkipSubParts( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset( m_puhTransformSkip[compID] + uiAbsPartIdx, useTransformSkip, sizeof( UChar ) * uiCurrPartNumb );
}

#if !REMOVE_NSQT
Void TComDataCU::setNSQTIdxSubParts( UInt absPartIdx, UInt depth )
{
  UInt currPartNumb = m_pcPic->getNumPartInCU() >> (depth << 1);

  memset( m_nsqtPartIdx + absPartIdx, absPartIdx, sizeof(UChar)*currPartNumb );
}

Void  TComDataCU::setNSQTIdxSubParts( UInt log2TrafoSize, const TComTU &rTu, UInt trMode )
{
  const UInt absPartIdxCU=rTu.GetAbsPartIdxCU();
  const UInt absPartIdx  =rTu.GetAbsPartIdxTU();
  TComPic *pPic=getPic();
  const UInt minCuWidth=pPic->getMinCUWidth();
  const UInt minCuHeight=pPic->getMinCUHeight();
  const TComRectangle &tuRectY=rTu.getRect(COMPONENT_Y);
  const UInt tuX0InBaseUnits=tuRectY.x0 / minCuWidth;
  const UInt tuY0InBaseUnits=tuRectY.y0 / minCuHeight;
  const UInt tuWidthInBaseUnits  = tuRectY.width / minCuWidth;
  const UInt tuHeightInBaseUnits = tuRectY.height / minCuHeight;

  const UInt lcuWidthInBaseUnits = pPic->getNumPartInWidth();
  const UInt rasterRelCU=tuY0InBaseUnits*lcuWidthInBaseUnits+tuX0InBaseUnits;
  const UInt absTUPartIdx=g_auiRasterToZscan[rasterRelCU+g_auiZscanToRaster[absPartIdxCU]];

  //  0  1   4  5  16 17  20 21       0  0  16 16  32 32  48 48
  //  2  3   6  7  18 19  22 23       0  0  16 16  32 32  48 48
  //  8  9  12 13  24 25  28 29       0  0  16 16  32 32  48 48
  // 10 11  14 15  26 27  30 31       0  0  16 16  32 32  48 48
  // 32 33  36 37  48 49  52 53   ->  0  0  16 16  32 32  48 48
  // 34 35  38 39  50 51  54 55       0  0  16 16  32 32  48 48
  // 40 41  44 45  56 57  60 61       0  0  16 16  32 32  48 48
  // 42 43  46 47  58 59  62 63       0  0  16 16  32 32  48 48

  
  if ( tuWidthInBaseUnits > tuHeightInBaseUnits )
  {
    UInt currPartNumb = tuHeightInBaseUnits*tuHeightInBaseUnits;
    memset( m_nsqtPartIdx + absTUPartIdx                                                              , absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+  tuHeightInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+2*tuHeightInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+3*tuHeightInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
  }
  else if ( tuWidthInBaseUnits < tuHeightInBaseUnits )
  {
    UInt currPartNumb = tuWidthInBaseUnits*tuWidthInBaseUnits;
    memset( m_nsqtPartIdx + absTUPartIdx                                                                                 , absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+  tuWidthInBaseUnits*lcuWidthInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+2*tuWidthInBaseUnits*lcuWidthInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
    memset( m_nsqtPartIdx + g_auiRasterToZscan[g_auiZscanToRaster[absTUPartIdx]+3*tuWidthInBaseUnits*lcuWidthInBaseUnits], absPartIdx, sizeof(UChar)*(currPartNumb) );
  }
  else
  {
    UInt currPartNumb = tuWidthInBaseUnits*tuHeightInBaseUnits;
    memset( m_nsqtPartIdx + absTUPartIdx, absPartIdx, sizeof(UChar)*(currPartNumb) );
  }
}
#endif

Void TComDataCU::setSizeSubParts( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset( m_puhWidth  + uiAbsPartIdx, uiWidth,  sizeof(UChar)*uiCurrPartNumb );
  memset( m_puhHeight + uiAbsPartIdx, uiHeight, sizeof(UChar)*uiCurrPartNumb );
}

UChar TComDataCU::getNumPartInter()
{
  UChar iNumPart = 0;

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:    iNumPart = 1; break;
    case SIZE_2NxN:     iNumPart = 2; break;
    case SIZE_Nx2N:     iNumPart = 2; break;
    case SIZE_NxN:      iNumPart = 4; break;
    case SIZE_2NxnU:    iNumPart = 2; break;
    case SIZE_2NxnD:    iNumPart = 2; break;
    case SIZE_nLx2N:    iNumPart = 2; break;
    case SIZE_nRx2N:    iNumPart = 2; break;
    default:            assert (0);   break;
  }

  return  iNumPart;
}

Void TComDataCU::getPartIndexAndSize( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight )
{
  switch ( m_pePartSize[0] )
  {
    case SIZE_2NxN:
      riWidth = getWidth(0);      riHeight = getHeight(0) >> 1; ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0);      ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0) >> 1; ruiPartAddr = ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 3);
      break;
    case SIZE_nLx2N:
      riWidth     = ( uiPartIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 4;
      break;
    case SIZE_nRx2N:
      riWidth     = ( uiPartIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert ( m_pePartSize[0] == SIZE_2Nx2N );
      riWidth = getWidth(0);      riHeight = getHeight(0);      ruiPartAddr = 0;
      break;
  }
}


Void TComDataCU::getMvField ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField )
{
  if ( pcCU == NULL )  // OUT OF BOUNDARY
  {
    TComMv  cZeroMv;
    rcMvField.setMvField( cZeroMv, NOT_VALID );
    return;
  }

  TComCUMvField*  pcCUMvField = pcCU->getCUMvField( eRefPicList );
  rcMvField.setMvField( pcCUMvField->getMv( uiAbsPartIdx ), pcCUMvField->getRefIdx( uiAbsPartIdx ) );
}

Void TComDataCU::deriveLeftRightTopIdxGeneral ( PartSize eCUMode, UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )
{
  ruiPartIdxLT = m_uiAbsIdxInLCU + uiAbsPartIdx;
  UInt uiPUWidth = 0;

  switch ( m_pePartSize[uiAbsPartIdx] )
  {
    case SIZE_2Nx2N: uiPUWidth = m_puhWidth[uiAbsPartIdx];  break;
    case SIZE_2NxN:  uiPUWidth = m_puhWidth[uiAbsPartIdx];   break;
    case SIZE_Nx2N:  uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1;  break;
    case SIZE_NxN:   uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1; break;
    case SIZE_2NxnU:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_2NxnD:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_nLx2N:
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2;
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2);
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N:
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2);
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2;
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert (0);
      break;
  }

  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + uiPUWidth / m_pcPic->getMinCUWidth() - 1 ];
}

Void TComDataCU::deriveLeftBottomIdxGeneral( PartSize eCUMode, UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB )
{
  UInt uiPUHeight = 0;
  switch ( m_pePartSize[uiAbsPartIdx] )
  {
    case SIZE_2Nx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];    break;
    case SIZE_2NxN:  uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_Nx2N:  uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_NxN:   uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_2NxnU:
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD:
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_nRx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    default:
      assert (0);
      break;
  }

  ruiPartIdxLB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU + uiAbsPartIdx ] + ((uiPUHeight / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInWidth()];
}

Void TComDataCU::deriveLeftRightTopIdx ( PartSize eCUMode, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )
{
  ruiPartIdxLT = m_uiAbsIdxInLCU;
  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1 ];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:                                                                                                                                break;
    case SIZE_2NxN:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1; ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2; ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN:
      ruiPartIdxLT += ( m_uiNumPartition >> 2 ) * uiPartIdx;         ruiPartIdxRT +=  ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );
      break;
    case SIZE_2NxnU:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      break;
    case SIZE_nLx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 4;
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      break;
    case SIZE_nRx2N:
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 4;
      break;
    default:
      assert (0);
      break;
  }

}

Void TComDataCU::deriveLeftBottomIdx( PartSize      eCUMode,   UInt  uiPartIdx,      UInt&      ruiPartIdxLB )
{
  ruiPartIdxLB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInWidth()];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:
      ruiPartIdxLB += m_uiNumPartition >> 1;
      break;
    case SIZE_2NxN:
      ruiPartIdxLB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 )? m_uiNumPartition >> 1 : (m_uiNumPartition >> 2)*3;
      break;
    case SIZE_NxN:
      ruiPartIdxLB += ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 4);
      break;
    case SIZE_nRx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert (0);
      break;
  }
}

/** Derives the partition index of neighbouring bottom right block
 * \param [in]  eCUMode
 * \param [in]  uiPartIdx
 * \param [out] ruiPartIdxRB
 */
Void TComDataCU::deriveRightBottomIdx( PartSize      eCUMode,   UInt  uiPartIdx,      UInt&      ruiPartIdxRB )
{
  ruiPartIdxRB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_uiAbsIdxInLCU ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInWidth() +  m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:
      ruiPartIdxRB += m_uiNumPartition >> 1;
      break;
    case SIZE_2NxN:
      ruiPartIdxRB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 )? m_uiNumPartition >> 2 : (m_uiNumPartition >> 1);
      break;
    case SIZE_NxN:
      ruiPartIdxRB += ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );
      break;
    case SIZE_2NxnU:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4): m_uiNumPartition >> 1;
      break;
    case SIZE_nRx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4) : m_uiNumPartition >> 1;
      break;
    default:
      assert (0);
      break;
  }
}

Void TComDataCU::deriveLeftRightTopIdxAdi ( UInt& ruiPartIdxLT, UInt& ruiPartIdxRT, UInt uiPartOffset, UInt uiPartDepth )
{
  UInt uiNumPartInWidth = (m_puhWidth[0]/m_pcPic->getMinCUWidth())>>uiPartDepth;
  ruiPartIdxLT = m_uiAbsIdxInLCU + uiPartOffset;
  ruiPartIdxRT = g_auiRasterToZscan[ g_auiZscanToRaster[ ruiPartIdxLT ] + uiNumPartInWidth - 1 ];
}

Void TComDataCU::deriveLeftBottomIdxAdi( UInt& ruiPartIdxLB, UInt uiPartOffset, UInt uiPartDepth )
{
  UInt uiAbsIdx;
  UInt uiMinCuWidth, uiWidthInMinCus;

  uiMinCuWidth    = getPic()->getMinCUWidth();
  uiWidthInMinCus = (getWidth(0)/uiMinCuWidth)>>uiPartDepth;
  uiAbsIdx        = getZorderIdxInCU()+uiPartOffset+(m_uiNumPartition>>(uiPartDepth<<1))-1;
  uiAbsIdx        = g_auiZscanToRaster[uiAbsIdx]-(uiWidthInMinCus-1);
  ruiPartIdxLB    = g_auiRasterToZscan[uiAbsIdx];
}

Bool TComDataCU::hasEqualMotion( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx )
{
  if ( getInterDir( uiAbsPartIdx ) != pcCandCU->getInterDir( uiCandAbsPartIdx ) )
  {
    return false;
  }

  for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
  {
    if ( getInterDir( uiAbsPartIdx ) & ( 1 << uiRefListIdx ) )
    {
      if ( getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiAbsPartIdx )     != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiCandAbsPartIdx ) ||
        getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiAbsPartIdx ) != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiCandAbsPartIdx ) )
      {
        return false;
      }
    }
  }

  return true;
}

/** Constructs a list of merging candidates
 * \param uiAbsPartIdx
 * \param uiPUIdx
 * \param uiDepth
 * \param pcMvFieldNeighbours
 * \param puhInterDirNeighbours
 * \param numValidMergeCand
 */
Void TComDataCU::getInterMergeCandidates( UInt uiAbsPartIdx, UInt uiPUIdx, UInt uiDepth, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand, Int mrgCandIdx )
{
  UInt uiAbsPartAddr = m_uiAbsIdxInLCU + uiAbsPartIdx;
  UInt uiIdx = 1;
  bool abCandIsInter[ MRG_MAX_NUM_CANDS ];
  for( UInt ui = 0; ui < MRG_MAX_NUM_CANDS; ++ui )
  {
    abCandIsInter[ui] = false;
  }
  // compute the location of the current PU
  Int xP, yP, nPSW, nPSH;
  this->getPartPosition(uiPUIdx, xP, yP, nPSW, nPSH);

  Int iCount = 0;

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  PartSize cCurPS = getPartitionSize( uiAbsPartIdx );
  deriveLeftRightTopIdxGeneral( cCurPS, uiAbsPartIdx, uiPUIdx, uiPartIdxLT, uiPartIdxRT );
  deriveLeftBottomIdxGeneral( cCurPS, uiAbsPartIdx, uiPUIdx, uiPartIdxLB );

  //left
  UInt uiLeftPartIdx = 0;
  TComDataCU* pcCULeft = 0;
  pcCULeft = getPULeft( uiLeftPartIdx, uiPartIdxLB, true, false );
  if (pcCULeft) 
  {
    if (!pcCULeft->isDiffMER(xP -1, yP+nPSH-1, xP, yP))
    {
      pcCULeft = NULL;
    }
  }
  PartSize partSize = getPartitionSize( uiAbsPartIdx );
  if (!(uiPUIdx == 1 && (partSize == SIZE_Nx2N || partSize == SIZE_nLx2N || partSize == SIZE_nRx2N)))
  {
  if ( pcCULeft && !pcCULeft->isIntra( uiLeftPartIdx ) )
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeft->getInterDir( uiLeftPartIdx );
    // get Mv from Left
    pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  }

  // above
  UInt uiAbovePartIdx = 0;
  TComDataCU* pcCUAbove = 0;
  pcCUAbove = getPUAbove( uiAbovePartIdx, uiPartIdxRT, true, false, true );
    if (pcCUAbove) 
    {
      if (!pcCUAbove->isDiffMER(xP+nPSW-1, yP-1, xP, yP))
      {
        pcCUAbove = NULL;
      }
    }
  if ( pcCUAbove && !pcCUAbove->isIntra( uiAbovePartIdx ) 
    && !(uiPUIdx == 1 && (cCurPS == SIZE_2NxN || cCurPS == SIZE_2NxnU || cCurPS == SIZE_2NxnD))
    && ( !pcCULeft || pcCULeft->isIntra( uiLeftPartIdx ) || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAbove, uiAbovePartIdx ) ) )
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAbove->getInterDir( uiAbovePartIdx );
    // get Mv from Left
    pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }

  // above right
  UInt uiAboveRightPartIdx = 0;
  TComDataCU* pcCUAboveRight = 0;
  pcCUAboveRight = getPUAboveRight( uiAboveRightPartIdx, uiPartIdxRT, true, false, true );
  if (pcCUAboveRight) 
  {
    if (!pcCUAboveRight->isDiffMER(xP+nPSW, yP-1, xP, yP))
    {
      pcCUAboveRight = NULL;
    }
  }
  if ( pcCUAboveRight && !pcCUAboveRight->isIntra( uiAboveRightPartIdx ) && ( !pcCUAbove || pcCUAbove->isIntra( uiAbovePartIdx ) || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveRight, uiAboveRightPartIdx ) ) )
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAboveRight->getInterDir( uiAboveRightPartIdx );
    // get Mv from Left
    pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }

  //left bottom
  UInt uiLeftBottomPartIdx = 0;
  TComDataCU* pcCULeftBottom = 0;
  pcCULeftBottom = this->getPUBelowLeft( uiLeftBottomPartIdx, uiPartIdxLB, true, false );
  if (pcCULeftBottom)
  {
    if (!pcCULeftBottom->isDiffMER(xP-1, yP+nPSH, xP, yP))
    {
      pcCULeftBottom = NULL;
    }
  }
  if ( pcCULeftBottom && !pcCULeftBottom->isIntra( uiLeftBottomPartIdx ) && ( !pcCULeft || pcCULeft->isIntra( uiLeftPartIdx ) || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) ) )
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeftBottom->getInterDir( uiLeftBottomPartIdx );
    // get Mv from Left
    pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }

  // above left 
  if( iCount < 4 )
  {
    UInt uiAboveLeftPartIdx = 0;
    TComDataCU* pcCUAboveLeft = 0;
    pcCUAboveLeft = getPUAboveLeft( uiAboveLeftPartIdx, uiAbsPartAddr, true, false, true );
    if (pcCUAboveLeft) 
    {
      if (!pcCUAboveLeft->isDiffMER(xP-1, yP-1, xP, yP))
      {
        pcCUAboveLeft = NULL;
      }
    }
    if( pcCUAboveLeft && !pcCUAboveLeft->isIntra( uiAboveLeftPartIdx )
     && ( !pcCULeft || pcCULeft->isIntra( uiLeftPartIdx ) || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
     && ( !pcCUAbove || pcCUAbove->isIntra( uiAbovePartIdx ) || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )
     )
    {
      abCandIsInter[iCount] = true;
      // get Inter Dir
      puhInterDirNeighbours[iCount] = pcCUAboveLeft->getInterDir( uiAboveLeftPartIdx );
      // get Mv from Left
      pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
      if ( getSlice()->isInterB() )
      {
        pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
      }
      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount ++;
    }
  }

  if ( getSlice()->getEnableTMVPFlag() )
  {
    //>> MTK colocated-RightBottom
    UInt uiPartIdxRB;
    Int uiLCUIdx = getAddr();
    PartSize eCUMode = getPartitionSize( 0 );

    deriveRightBottomIdx( eCUMode, uiPUIdx, uiPartIdxRB );  

    UInt uiAbsPartIdxTmp = g_auiZscanToRaster[uiPartIdxRB];
    UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();

    TComMv cColMv;
    Int iRefIdx;

    if      ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxTmp] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )  // image boundary check
    {
      uiLCUIdx = -1;
    }
    else if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxTmp] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
    {
      uiLCUIdx = -1;
    }
    else
    {
      if ( ( uiAbsPartIdxTmp % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 ) &&           // is not at the last column of LCU 
        ( uiAbsPartIdxTmp / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) ) // is not at the last row    of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + uiNumPartInCUWidth + 1 ];
        uiLCUIdx = getAddr();
      }
      else if ( uiAbsPartIdxTmp % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 )           // is not at the last column of LCU But is last row of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdxTmp + uiNumPartInCUWidth + 1) % m_pcPic->getNumPartInCU() ];
        uiLCUIdx = -1 ; 
      }
      else if ( uiAbsPartIdxTmp / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) // is not at the last row of LCU But is last column of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + 1 ];
        uiLCUIdx = getAddr() + 1;
      }
      else //is the right bottom corner of LCU                       
      {
        uiAbsPartAddr = 0;
        uiLCUIdx = -1 ; 
      }
    }

    iRefIdx = 0;

    Bool bExistMV = false;
    UInt uiPartIdxCenter;
    UInt uiCurLCUIdx = getAddr();
    xDeriveCenterIdx( eCUMode, uiPUIdx, uiPartIdxCenter );
    bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_0, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx );
    if( bExistMV == false )
    {
      bExistMV = xGetColMVP( REF_PIC_LIST_0, uiCurLCUIdx, uiPartIdxCenter,  cColMv, iRefIdx );
    }
    if( bExistMV )
    {
      UInt uiArrayAddr = iCount;
      abCandIsInter[uiArrayAddr] = true;
      pcMvFieldNeighbours[uiArrayAddr << 1].setMvField( cColMv, iRefIdx );

      if ( getSlice()->isInterB() )
      {       
        iRefIdx = 0;

        bExistMV = uiLCUIdx >= 0 && xGetColMVP( REF_PIC_LIST_1, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx);
        if( bExistMV == false )
        {
          bExistMV = xGetColMVP( REF_PIC_LIST_1, uiCurLCUIdx, uiPartIdxCenter,  cColMv, iRefIdx );
        }
        if( bExistMV )
        {
          pcMvFieldNeighbours[ ( uiArrayAddr << 1 ) + 1 ].setMvField( cColMv, iRefIdx );
          puhInterDirNeighbours[uiArrayAddr] = 3;
        }
        else
        {
          puhInterDirNeighbours[uiArrayAddr] = 1;
        }
      }
      else
      {
        puhInterDirNeighbours[uiArrayAddr] = 1;
      }
      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount++;
    }
    uiIdx++;

  }

  UInt uiArrayAddr = iCount;
  UInt uiCutoff = uiArrayAddr;
    
  if ( getSlice()->isInterB() )
  {
    static const UInt NUM_PRIORITY_LIST=12; // NOTE: ECF - new definition
    static const UInt uiPriorityList0[NUM_PRIORITY_LIST] = {0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3};
    static const UInt uiPriorityList1[NUM_PRIORITY_LIST] = {1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2};

    for (Int idx=0; idx<uiCutoff*(uiCutoff-1) && uiArrayAddr!=MRG_MAX_NUM_CANDS; idx++)
    {
      assert(idx<NUM_PRIORITY_LIST);
      Int i = uiPriorityList0[idx];
      Int j = uiPriorityList1[idx];
      if (abCandIsInter[i] && abCandIsInter[j]&& (puhInterDirNeighbours[i]&0x1)&&(puhInterDirNeighbours[j]&0x2))
      {
        abCandIsInter[uiArrayAddr] = true;
        puhInterDirNeighbours[uiArrayAddr] = 3;

        // get Mv from cand[i] and cand[j]
        pcMvFieldNeighbours[uiArrayAddr << 1].setMvField(pcMvFieldNeighbours[i<<1].getMv(), pcMvFieldNeighbours[i<<1].getRefIdx());
        pcMvFieldNeighbours[( uiArrayAddr << 1 ) + 1].setMvField(pcMvFieldNeighbours[(j<<1)+1].getMv(), pcMvFieldNeighbours[(j<<1)+1].getRefIdx());

        Int iRefPOCL0 = m_pcSlice->getRefPOC( REF_PIC_LIST_0, pcMvFieldNeighbours[(uiArrayAddr<<1)].getRefIdx() );
        Int iRefPOCL1 = m_pcSlice->getRefPOC( REF_PIC_LIST_1, pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getRefIdx() );
        if (iRefPOCL0 == iRefPOCL1 && pcMvFieldNeighbours[(uiArrayAddr<<1)].getMv() == pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getMv())
        {
          abCandIsInter[uiArrayAddr] = false;
        }
        else
        {
          uiArrayAddr++;
        }
      }
    }
  }

  Int iNumRefIdx = (getSlice()->isInterB()) ? min(m_pcSlice->getNumRefIdx(REF_PIC_LIST_0), m_pcSlice->getNumRefIdx(REF_PIC_LIST_1)) : m_pcSlice->getNumRefIdx(REF_PIC_LIST_0);

  Int r = 0;
  Int refcnt = 0;
  while (uiArrayAddr < MRG_MAX_NUM_CANDS)
  {
    abCandIsInter[uiArrayAddr] = true;
    puhInterDirNeighbours[uiArrayAddr] = 1;
    pcMvFieldNeighbours[uiArrayAddr << 1].setMvField( TComMv(0, 0), r);

    if ( getSlice()->isInterB() )
    {
      puhInterDirNeighbours[uiArrayAddr] = 3;
      pcMvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(TComMv(0, 0), r);
    }
    uiArrayAddr++;

    if ( refcnt == iNumRefIdx - 1 )
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  if (uiArrayAddr > MRG_MAX_NUM_CANDS_SIGNALED)
  {
    uiArrayAddr = MRG_MAX_NUM_CANDS_SIGNALED;
  }
  numValidMergeCand = uiArrayAddr;
}

/** Check the duplicated candidate in the list
 * \param pcMvFieldNeighbours
 * \param puhInterDirNeighbours
 * \param pbCandIsInter
 * \param ruiArrayAddr
 * \returns Void
 */
Void TComDataCU::xCheckDuplicateCand(TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, bool* pbCandIsInter, UInt& ruiArrayAddr)
{
  if (getSlice()->isInterB())
  {
    UInt uiMvFieldNeighIdxCurr = ruiArrayAddr << 1;
    Int iRefIdxL0 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr ].getRefIdx();
    Int iRefIdxL1 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr + 1 ].getRefIdx();
    TComMv MvL0 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr ].getMv();
    TComMv MvL1 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr + 1 ].getMv();

    for (int k=0; k<ruiArrayAddr; k++)
    {
      UInt uiMvFieldNeighIdxComp = k << 1;
      if (iRefIdxL0 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp ].getRefIdx() &&
          iRefIdxL1 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp + 1 ].getRefIdx() &&
          MvL0 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp ].getMv() &&
          MvL1 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp + 1 ].getMv() &&
          puhInterDirNeighbours[ ruiArrayAddr ] == puhInterDirNeighbours[ k ] )
      {
        pbCandIsInter[ruiArrayAddr] = false;
        break;
      }
    }
  }
  else
  {
    UInt uiMvFieldNeighIdxCurr = ruiArrayAddr << 1;
    Int iRefIdxL0 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr ].getRefIdx();
    TComMv MvL0 = pcMvFieldNeighbours[ uiMvFieldNeighIdxCurr ].getMv();

    for (int k=0; k<ruiArrayAddr; k++)
    {
      UInt uiMvFieldNeighIdxComp = k << 1;
      if (iRefIdxL0 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp ].getRefIdx() &&
          MvL0 == pcMvFieldNeighbours[ uiMvFieldNeighIdxComp ].getMv() &&
          puhInterDirNeighbours[ ruiArrayAddr ] == puhInterDirNeighbours[ k ] )
      {
        pbCandIsInter[ruiArrayAddr] = false;
        break;
      }
    }
  }

  if (pbCandIsInter[ruiArrayAddr])
  {
    ++ruiArrayAddr;
  }
}

Void TComDataCU::xCheckCornerCand( TComDataCU* pcCorner, UInt uiCornerPUIdx, UInt uiIter, Bool& rbValidCand )
{
  if( uiIter == 0 )
  {
    if( pcCorner && !pcCorner->isIntra( uiCornerPUIdx ) )
    {
      rbValidCand = true;
      if( getSlice()->isInterB() )
      {
        if ( pcCorner->getInterDir( uiCornerPUIdx ) == 1 )
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) != 0 )
          {
            rbValidCand = false;
          }
        }
        else if ( pcCorner->getInterDir( uiCornerPUIdx ) == 2 )
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_1)->getRefIdx( uiCornerPUIdx ) != 0 )
          {
            rbValidCand = false;
          }
        }
        else
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) != 0 || pcCorner->getCUMvField(REF_PIC_LIST_1)->getRefIdx( uiCornerPUIdx ) != 0 )
          {
            rbValidCand = false;
          }
        }
      }
      else if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) != 0 )
      {
        rbValidCand = false;
      }
    }
  }
  else
  {
    if( pcCorner && !pcCorner->isIntra( uiCornerPUIdx ) )
    {
      rbValidCand = true;
      if( getSlice()->isInterB() )
      {
        if ( pcCorner->getInterDir( uiCornerPUIdx ) == 1 )
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) < 0 )
          {
            rbValidCand = false;
          }
        }
        else if ( pcCorner->getInterDir( uiCornerPUIdx ) == 2 )
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_1)->getRefIdx( uiCornerPUIdx ) < 0 )
          {
            rbValidCand = false;
          }
        }
        else
        {
          if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) < 0 || pcCorner->getCUMvField(REF_PIC_LIST_1)->getRefIdx( uiCornerPUIdx ) < 0 )
          {
            rbValidCand = false;
          }
        }
      }
      else if( pcCorner->getCUMvField(REF_PIC_LIST_0)->getRefIdx( uiCornerPUIdx ) < 0 )
      {
        rbValidCand = false;
      }
    }
  }
}

/** Check whether the current PU and a spatial neighboring PU are in a same ME region.
 * \param xN, xN   location of the upper-left corner pixel of a neighboring PU
 * \param xP, yP   location of the upper-left corner pixel of the current PU
 * \returns Bool
 */
Bool TComDataCU::isDiffMER(Int xN, Int yN, Int xP, Int yP)
{

  UInt plevel = this->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() + 2;
  if ((xN>>plevel)!= (xP>>plevel))
  {
    return true;
  }
  if ((yN>>plevel)!= (yP>>plevel))
  {
    return true;
  }
  return false;
}

/** calculate the location of upper-left corner pixel and size of the current PU.
 * \param partIdx  PU index within a CU
 * \param xP, yP   location of the upper-left corner pixel of the current PU
 * \param PSW, nPSH    size of the curren PU
 * \returns Void
 */
Void TComDataCU::getPartPosition( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH)
{
  UInt col = m_uiCUPelX;
  UInt row = m_uiCUPelY;

  switch ( m_pePartSize[0] )
  {
  case SIZE_2NxN:
    nPSW = getWidth(0);      
    nPSH = getHeight(0) >> 1; 
    xP   = col;
    yP   = (partIdx ==0)? row: row + nPSH;
    break;
  case SIZE_Nx2N:
    nPSW = getWidth(0) >> 1; 
    nPSH = getHeight(0);      
    xP   = (partIdx ==0)? col: col + nPSW;
    yP   = row;
    break;
  case SIZE_NxN:
    nPSW = getWidth(0) >> 1; 
    nPSH = getHeight(0) >> 1; 
    xP   = col + (partIdx&0x1)*nPSW;
    yP   = row + (partIdx>>1)*nPSH;
    break;
  case SIZE_2NxnU:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;

    break;
  case SIZE_2NxnD:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;
    break;
  case SIZE_nLx2N:
    nPSW = ( partIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  case SIZE_nRx2N:
    nPSW = ( partIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  default:
    assert ( m_pePartSize[0] == SIZE_2Nx2N );
    nPSW = getWidth(0);      
    nPSH = getHeight(0);      
    xP   = col ;
    yP   = row ;

    break;
  }
}

AMVP_MODE TComDataCU::getAMVPMode(UInt uiIdx)
{
  return m_pcSlice->getSPS()->getAMVPMode(m_puhDepth[uiIdx]);
}

/** Constructs a list of candidates for AMVP
 * \param uiPartIdx
 * \param uiPartAddr 
 * \param eRefPicList
 * \param iRefIdx
 * \param pInfo
 */
Void TComDataCU::fillMvpCand ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo )
{
  PartSize eCUMode = getPartitionSize( 0 );
  
  TComMv cMvPred;
  Bool bAddedSmvp = false;

  pInfo->iN = 0;  
  if (iRefIdx < 0)
  {
    return;
  }
  
  //-- Get Spatial MV
  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  UInt uiNumPartInCUWidth = m_pcPic->getNumPartInWidth();
  Bool bAdded = false;
  
  deriveLeftRightTopIdx( eCUMode, uiPartIdx, uiPartIdxLT, uiPartIdxRT );
  deriveLeftBottomIdx( eCUMode, uiPartIdx, uiPartIdxLB );
  
  TComDataCU* tmpCU = NULL;
  UInt idx;
  tmpCU = getPUBelowLeft(idx, uiPartIdxLB, true, false);
  bAddedSmvp = (tmpCU != NULL) && (tmpCU->getPredictionMode(idx) != MODE_INTRA);

  if (!bAddedSmvp)
  {
    tmpCU = getPULeft(idx, uiPartIdxLB, true, false);
    bAddedSmvp = (tmpCU != NULL) && (tmpCU->getPredictionMode(idx) != MODE_INTRA);
  }

  // Left predictor search
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);
  if (!bAdded) 
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );
  }
  
  if(!bAdded)
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);
    if (!bAdded) 
    {
      bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );
    }
  }
  // Above predictor search
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);

  if (!bAdded) 
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);
  }

  if(!bAdded)
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);
  }
  bAdded = bAddedSmvp;
  if (pInfo->iN==2) bAdded = true;

  if(!bAdded)
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);
    if (!bAdded) 
    {
      bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);
    }

    if(!bAdded)
    {
      bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);
    }
  }
  
  if (getAMVPMode(uiPartAddr) == AM_NONE)  //Should be optimized later for special cases
  {
    assert(pInfo->iN > 0);
    pInfo->iN = 1;
    return;
  }

  if ( pInfo->iN == 2 )
  {
    if ( pInfo->m_acMvCand[ 0 ] == pInfo->m_acMvCand[ 1 ] )
    {
      pInfo->iN = 1;
    }
  }

  if ( getSlice()->getEnableTMVPFlag() )
  {
    // Get Temporal Motion Predictor
    int iRefIdx_Col = iRefIdx;
    TComMv cColMv;
    UInt uiPartIdxRB;
    UInt uiAbsPartIdx;  
    UInt uiAbsPartAddr;
    int uiLCUIdx = getAddr();

    deriveRightBottomIdx( eCUMode, uiPartIdx, uiPartIdxRB );
    uiAbsPartAddr = m_uiAbsIdxInLCU + uiPartAddr;

    //----  co-located RightBottom Temporal Predictor (H) ---//
    uiAbsPartIdx = g_auiZscanToRaster[uiPartIdxRB];
    if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdx] + m_pcPic->getMinCUWidth() ) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )  // image boundary check
    {
      uiLCUIdx = -1;
    }
    else if ( ( m_pcPic->getCU(m_uiCUAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdx] + m_pcPic->getMinCUHeight() ) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples() )
    {
      uiLCUIdx = -1;
    }
    else
    {
      if ( ( uiAbsPartIdx % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 ) &&           // is not at the last column of LCU 
        ( uiAbsPartIdx / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) ) // is not at the last row    of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + uiNumPartInCUWidth + 1 ];
        uiLCUIdx = getAddr();
      }
      else if ( uiAbsPartIdx % uiNumPartInCUWidth < uiNumPartInCUWidth - 1 )           // is not at the last column of LCU But is last row of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdx + uiNumPartInCUWidth + 1) % m_pcPic->getNumPartInCU() ];
        uiLCUIdx      = -1 ; 
      }
      else if ( uiAbsPartIdx / uiNumPartInCUWidth < m_pcPic->getNumPartInHeight() - 1 ) // is not at the last row of LCU But is last column of LCU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + 1 ];
        uiLCUIdx = getAddr() + 1;
      }
      else //is the right bottom corner of LCU                       
      {
        uiAbsPartAddr = 0;
        uiLCUIdx      = -1 ; 
      }
    }
    if ( uiLCUIdx >= 0 && xGetColMVP( eRefPicList, uiLCUIdx, uiAbsPartAddr, cColMv, iRefIdx_Col ) )
    {
      pInfo->m_acMvCand[pInfo->iN++] = cColMv;
    }
    else 
    {
      UInt uiPartIdxCenter;
      UInt uiCurLCUIdx = getAddr();
      xDeriveCenterIdx( eCUMode, uiPartIdx, uiPartIdxCenter );
      if (xGetColMVP( eRefPicList, uiCurLCUIdx, uiPartIdxCenter,  cColMv, iRefIdx_Col ))
      {
        pInfo->m_acMvCand[pInfo->iN++] = cColMv;
      }
    }
    //----  co-located RightBottom Temporal Predictor  ---//
  }

  if (pInfo->iN > AMVP_MAX_NUM_CANDS)
  {
    pInfo->iN = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->iN < AMVP_MAX_NUM_CANDS)
  {
    pInfo->m_acMvCand[pInfo->iN].set(0,0);
    pInfo->iN++;
  }
  return ;
}


Bool TComDataCU::isBipredRestriction(UInt puIdx)
{
  Int width = 0;
  Int height = 0;
  UInt partAddr;

  getPartIndexAndSize( puIdx, partAddr, width, height );
  if ( getWidth(0) == 8 && (width < 8 || height < 8) )
  {
    return true;
  }
  return false;
}


Void TComDataCU::clipMv    (TComMv&  rcMv)
{
  Int  iMvShift = 2;
  Int iOffset = 8;
  Int iHorMax = ( m_pcSlice->getSPS()->getPicWidthInLumaSamples() + iOffset - m_uiCUPelX - 1 ) << iMvShift;
  Int iHorMin = (       -(Int)g_uiMaxCUWidth - iOffset - (Int)m_uiCUPelX + 1 ) << iMvShift;
  
  Int iVerMax = ( m_pcSlice->getSPS()->getPicHeightInLumaSamples() + iOffset - m_uiCUPelY - 1 ) << iMvShift;
  Int iVerMin = (       -(Int)g_uiMaxCUHeight - iOffset - (Int)m_uiCUPelY + 1 ) << iMvShift;
  
  rcMv.setHor( min (iHorMax, max (iHorMin, rcMv.getHor())) );
  rcMv.setVer( min (iVerMax, max (iVerMin, rcMv.getVer())) );
}


Void TComDataCU::convertTransIdx( UInt uiAbsPartIdx, UInt uiTrIdx, UInt& ruiLumaTrMode, UInt& ruiChromaTrMode )
{
  ruiLumaTrMode   = uiTrIdx;
  ruiChromaTrMode = uiTrIdx;
  return;
}

UInt TComDataCU::getIntraSizeIdx(UInt uiAbsPartIdx)
{
  UInt uiShift = ( (m_puhTrIdx[uiAbsPartIdx]==0) && (m_pePartSize[uiAbsPartIdx]==SIZE_NxN) ) ? m_puhTrIdx[uiAbsPartIdx]+1 : m_puhTrIdx[uiAbsPartIdx];
  uiShift = ( m_pePartSize[uiAbsPartIdx]==SIZE_NxN ? 1 : 0 );
  
  UChar uiWidth = m_puhWidth[uiAbsPartIdx]>>uiShift;
  UInt  uiCnt = 0;
  while( uiWidth )
  {
    uiCnt++;
    uiWidth>>=1;
  }
  uiCnt-=2;
  return uiCnt > 6 ? 6 : uiCnt;
}

Void TComDataCU::clearCbf( UInt uiIdx, ComponentID compID, UInt uiNumParts )
{
  memset( &m_puhCbf[compID][uiIdx], 0, sizeof(UChar)*uiNumParts);
}

/** Set a I_PCM flag for all sub-partitions of a partition.
 * \param bIpcmFlag I_PCM flag
 * \param uiAbsPartIdx patition index
 * \param uiDepth CU depth
 * \returns Void
 */
Void TComDataCU::setIPCMFlagSubParts  (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartInCU() >> (uiDepth << 1);

  memset(m_pbIPCMFlag + uiAbsPartIdx, bIpcmFlag, sizeof(Bool)*uiCurrPartNumb );
}

/** Test whether the current block is skipped
 * \param uiPartIdx Block index
 * \returns Flag indicating whether the block is skipped
 */
Bool TComDataCU::isSkipped( UInt uiPartIdx )
{
#if SKIP_FLAG
  return ( getSkipFlag( uiPartIdx ) );
#else
  if ( m_pcSlice->isIntra () )
  {
    return false;
  }
  return ( getMergeFlag( uiPartIdx ) && getPartitionSize( uiPartIdx ) == SIZE_2Nx2N && !getQtRootCbf( uiPartIdx ) );
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Bool TComDataCU::xAddMVPCand( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )
  {
    case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx, true, false);
      break;
    }
    case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
    case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
    case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx, true, false);
      break;
    }
    case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
    default:
    {
      break;
    }
  }
 
  if ( pcTmpCU != NULL && m_pcSlice->isEqualRef(eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx), iRefIdx) )
  {
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);
    
    pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;
    return true;
  }

  if ( pcTmpCU == NULL ) 
  {
    return false;
  }
  
  RefPicList eRefPicList2nd = REF_PIC_LIST_0;
  if(       eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }


  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();
  Int iNeibRefPOC;


  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0 )
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    if( iNeibRefPOC == iCurrRefPOC ) // Same Reference Frame But Diff List//
    {
      TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
      pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;
      return true;
    }
  }
  return false;
}

/** 
 * \param pInfo
 * \param eRefPicList 
 * \param iRefIdx
 * \param uiPartUnitIdx
 * \param eDir
 * \returns Bool
 */
Bool TComDataCU::xAddMVPCandOrder( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )
  {
  case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx, true, false);
      break;
    }
  case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
  case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
  case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx, true, false);
      break;
    }
  case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx, true, false, true);
      break;
    }
  default:
    {
      break;
    }
  }

  if ( pcTmpCU == NULL ) 
  {
    return false;
  }
  
  RefPicList eRefPicList2nd = REF_PIC_LIST_0;
  if(       eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }

  Int iCurrPOC = m_pcSlice->getPOC();
  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();
  Int iNeibPOC = iCurrPOC;
  Int iNeibRefPOC;
  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getIsLongTerm();
  Bool bIsNeibRefLongTerm = false;

  //---------------  V1 (END) ------------------//
  if( pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) >= 0)
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )
    {
      rcMv = cMvPred;
    }
    else
    {
      Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );
      if ( iScale == 4096 )
      {
        rcMv = cMvPred;
      }
      else
      {
        rcMv = cMvPred.scaleMv( iScale );
      }
    }

    pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
    return true;
  }
  //---------------------- V2(END) --------------------//
  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0)
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )
    {
      rcMv = cMvPred;
    }
    else
    {
      Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );
      if ( iScale == 4096 )
      {
        rcMv = cMvPred;
      }
      else
      {
        rcMv = cMvPred.scaleMv( iScale );
      }
    }

    pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
    return true;
  }
  //---------------------- V3(END) --------------------//
  return false;
}

/** 
 * \param eRefPicList
 * \param uiCUAddr 
 * \param uiPartUnitIdx
 * \param riRefIdx
 * \returns Bool
 */
Bool TComDataCU::xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx )
{
  UInt uiAbsPartAddr = uiPartUnitIdx;

  RefPicList  eColRefPicList;
  Int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  TComMv cColMv;

  // use coldir.
  TComPic *pColPic = getSlice()->getRefPic( RefPicList(getSlice()->isInterB() ? getSlice()->getColDir() : 0), getSlice()->getColRefIdx());
  TComDataCU *pColCU = pColPic->getCU( uiCUAddr );
  if(pColCU->getPic()==0||pColCU->getPartitionSize(uiPartUnitIdx)==NUMBER_OF_PART_SIZES)
  {
    return false;
  }
  iCurrPOC = m_pcSlice->getPOC();    
  iCurrRefPOC = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getPOC();
  iColPOC = pColCU->getSlice()->getPOC();  

  if (pColCU->isIntra(uiAbsPartAddr))
  {
    return false;
  }

  eColRefPicList = getSlice()->getCheckLDC() ? eRefPicList : RefPicList(1-getSlice()->getColDir());

  Int iColRefIdx = pColCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);

  if (iColRefIdx < 0 )
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = pColCU->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);

    if (iColRefIdx < 0 )
    {
      return false;
    }
  }

  // Scale the vector.
  iColRefPOC = pColCU->getSlice()->getRefPOC(eColRefPicList, iColRefIdx);
  cColMv = pColCU->getCUMvField(eColRefPicList)->getMv(uiAbsPartAddr);

  iCurrRefPOC = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getPOC();

  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getIsLongTerm();
  Bool bIsColRefLongTerm = pColCU->getSlice()->getRefPic(eColRefPicList, iColRefIdx)->getIsUsedAsLongTerm();
  if ( bIsCurrRefLongTerm || bIsColRefLongTerm )
  {
    rcMv = cColMv;
  }
  else
  {
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
    if ( iScale == 4096 )
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv( iScale );
    }
  }

  return true;
}

UInt TComDataCU::xGetMvdBits(TComMv cMvd)
{
  return ( xGetComponentBits(cMvd.getHor()) + xGetComponentBits(cMvd.getVer()) );
}

UInt TComDataCU::xGetComponentBits(Int iVal)
{
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);
  
  assert ( uiTemp );
  
  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  
  return uiLength;
}


Int TComDataCU::xGetDistScaleFactor(Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC)
{
  Int iDiffPocD = iColPOC - iColRefPOC;
  Int iDiffPocB = iCurrPOC - iCurrRefPOC;
  
  if( iDiffPocD == iDiffPocB )
  {
    return 4096;
  }
  else
  {
    Int iTDB      = Clip3( -128, 127, iDiffPocB );
    Int iTDD      = Clip3( -128, 127, iDiffPocD );
    Int iX        = (0x4000 + abs(iTDD/2)) / iTDD;
    Int iScale    = Clip3( -4096, 4095, (iTDB * iX + 32) >> 6 );
    return iScale;
  }
}

/** 
 * \param eCUMode
 * \param uiPartIdx 
 * \param ruiPartIdxCenter
 * \returns Void
 */
Void TComDataCU::xDeriveCenterIdx( PartSize eCUMode, UInt uiPartIdx, UInt& ruiPartIdxCenter )
{
  UInt uiPartAddr;
  Int  iPartWidth;
  Int  iPartHeight;
  getPartIndexAndSize( uiPartIdx, uiPartAddr, iPartWidth, iPartHeight);
  
  ruiPartIdxCenter = m_uiAbsIdxInLCU+uiPartAddr; // partition origin.
  ruiPartIdxCenter = g_auiRasterToZscan[ g_auiZscanToRaster[ ruiPartIdxCenter ]
                                        + ( iPartHeight/m_pcPic->getMinCUHeight()  )/2*m_pcPic->getNumPartInWidth()
                                        + ( iPartWidth/m_pcPic->getMinCUWidth()  )/2];
}

/** 
 * \param uiPartIdx
 * \param eRefPicList 
 * \param iRefIdx
 * \param pcMv
 * \returns Bool
 */
/*
Bool TComDataCU::xGetCenterCol( UInt uiPartIdx, RefPicList eRefPicList, int iRefIdx, TComMv *pcMv )
{
  PartSize eCUMode = getPartitionSize( 0 );
  
  Int iCurrPOC = m_pcSlice->getPOC();
  
  // use coldir.
  TComPic *pColPic = getSlice()->getRefPic( RefPicList(getSlice()->isInterB() ? getSlice()->getColDir() : 0), getSlice()->getColRefIdx());
  TComDataCU *pColCU = pColPic->getCU( m_uiCUAddr );
  
  Int iColPOC = pColCU->getSlice()->getPOC();
  UInt uiPartIdxCenter;
  xDeriveCenterIdx( eCUMode, uiPartIdx, uiPartIdxCenter );
  
  if (pColCU->isIntra(uiPartIdxCenter))
  {
    return false;
  }
  
  // Prefer a vector crossing us.  Prefer shortest.
  RefPicList eColRefPicList = REF_PIC_LIST_0;
  bool bFirstCrosses = false;
  Int  iFirstColDist = -1;
  for (Int l = 0; l < 2; l++)
  {
    bool bSaveIt = false;
    int iColRefIdx = pColCU->getCUMvField(RefPicList(l))->getRefIdx(uiPartIdxCenter);
    if (iColRefIdx < 0)
    {
      continue;
    }
    int iColRefPOC = pColCU->getSlice()->getRefPOC(RefPicList(l), iColRefIdx);
    int iColDist = abs(iColRefPOC - iColPOC);
    bool bCrosses = iColPOC < iCurrPOC ? iColRefPOC > iCurrPOC : iColRefPOC < iCurrPOC;
    if (iFirstColDist < 0)
    {
      bSaveIt = true;
    }
    else if (bCrosses && !bFirstCrosses)
    {
      bSaveIt = true;
    }
    else if (bCrosses == bFirstCrosses && l == eRefPicList)
    {
      bSaveIt = true;
    }
    
    if (bSaveIt)
    {
      bFirstCrosses = bCrosses;
      iFirstColDist = iColDist;
      eColRefPicList = RefPicList(l);
    }
  }
  
  // Scale the vector.
  Int iColRefPOC = pColCU->getSlice()->getRefPOC(eColRefPicList, pColCU->getCUMvField(eColRefPicList)->getRefIdx(uiPartIdxCenter));
  TComMv cColMv = pColCU->getCUMvField(eColRefPicList)->getMv(uiPartIdxCenter);
  
  Int iCurrRefPOC = m_pcSlice->getRefPic(eRefPicList, iRefIdx)->getPOC();

  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic(eRefPicList, iRefIdx)->getIsLongTerm();
  Bool bIsColRefLongTerm = pColCU->getSlice()->getRefPic(eColRefPicList, pColCU->getCUMvField(eColRefPicList)->getRefIdx(uiPartIdxCenter))->getIsUsedAsLongTerm();
  if ( bIsCurrRefLongTerm || bIsColRefLongTerm )
  {
    pcMv[0] = cColMv;
  }
  else
  {
    Int iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
    if ( iScale == 4096 )
    {
      pcMv[0] = cColMv;
    }
    else
    {
      pcMv[0] = cColMv.scaleMv( iScale );
    }
  }

  return true;
}
*/

Void TComDataCU::compressMV()
{
  Int scaleFactor = 4 * AMVP_DECIMATION_FACTOR / m_unitSize;
  if (scaleFactor > 0)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].compress(m_pePredMode, scaleFactor);
    }
  }
}

UInt TComDataCU::getCoefScanIdx(const UInt uiAbsPartIdx, const UInt uiWidth, const UInt uiHeight, const ComponentID compID) const
{
  //------------------------------------------------

  //this mechanism is available for intra only

  if (!isIntra(uiAbsPartIdx)) return SCAN_ZIGZAG;

  //------------------------------------------------

  //check that MDCS can be used for this TU

  const MDCSMode mode = getMDCSMode(uiWidth, uiHeight, compID, getPic()->getChromaFormat());

  if (mode == MDCS_DISABLED) return SCAN_ZIGZAG;

  //------------------------------------------------

  //otherwise, select the appropriate mode

  const UInt angleLimit = getMDCSAngleLimit(compID);
        UInt uiDirMode  = getIntraDir(toChannelType(compID), uiAbsPartIdx);

  if (uiDirMode==DM_CHROMA_IDX)
  {
    uiDirMode = getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, getPic()->getChromaFormat()));
  }

  //------------------

  switch (mode)
  {
    case MDCS_BOTH_DIRECTIONS:
      if      (abs((Int)uiDirMode - VER_IDX) <= angleLimit) return SCAN_HOR;
      else if (abs((Int)uiDirMode - HOR_IDX) <= angleLimit) return SCAN_VER;
      break;
      
    case MDCS_VERTICAL_ONLY:
      if      (abs((Int)uiDirMode - HOR_IDX) <= angleLimit) return SCAN_VER;
      break;

    case MDCS_HORIZONTAL_ONLY:
      if      (abs((Int)uiDirMode - VER_IDX) <= angleLimit) return SCAN_HOR;
      break;

    case MDCS_DISABLED:
      break;

    default:
      std::cerr << "ERROR: Unrecognised MDCS mode" << std::endl;
      assert(false);
      exit(1);
      break;
  }

  //------------------------------------------------

  return SCAN_ZIGZAG;
}

#if !REMOVE_NSQT
Bool TComDataCU::useNonSquarePU(UInt absPartIdx)
{
  if ( ( m_pePartSize[absPartIdx] == SIZE_Nx2N ) || ( m_pePartSize[absPartIdx] == SIZE_2NxN ) || ( m_pePartSize[absPartIdx] >= SIZE_2NxnU && m_pePartSize[absPartIdx] <= SIZE_nRx2N ) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

UInt TComDataCU::getInterTUSplitDirection( Int trWidth, Int trHeight, Int trLastWidth, Int trLastHeight )
{
  UInt interTUSplitDirection = 2;
  if ( ( trWidth == trLastWidth ) && ( trHeight < trLastHeight ) )
  {
    interTUSplitDirection = 0;
  }
  else if ( ( trWidth < trLastWidth ) && ( trHeight == trLastHeight ) )
  {
    interTUSplitDirection = 1;
  }    

  return interTUSplitDirection;
}
#endif


UInt TComDataCU::getSCUAddr()
{
  return getPic()->getPicSym()->getInverseCUOrderMap(m_uiCUAddr)*(1<<(m_pcSlice->getSPS()->getMaxCUDepth()<<1))+m_uiAbsIdxInLCU;
}

/** Set neighboring blocks availabilities for non-deblocked filtering
 * \param numLCUInPicWidth number of LCUs in picture width
 * \param numLCUInPicHeight number of LCUs in picture height
 * \param numSUInLCUWidth number of SUs in LCU width
 * \param numSUInLCUHeight number of SUs in LCU height
 * \param picWidth picture width
 * \param picHeight picture height
 * \param bIndependentSliceBoundaryEnabled true for independent slice boundary enabled
 * \param bTopTileBoundary true means that top boundary coincides tile boundary
 * \param bDownTileBoundary true means that bottom boundary coincides tile boundary
 * \param bLeftTileBoundary true means that left boundary coincides tile boundary
 * \param bRightTileBoundary true means that right boundary coincides tile boundary
 * \param bIndependentTileBoundaryEnabled true for independent tile boundary enabled
 */
Void TComDataCU::setNDBFilterBlockBorderAvailability(UInt numLCUInPicWidth, UInt numLCUInPicHeight, UInt numSUInLCUWidth, UInt numSUInLCUHeight, UInt picWidth, UInt picHeight
                                                    ,std::vector<Bool>& LFCrossSliceBoundary
                                                    ,Bool bTopTileBoundary, Bool bDownTileBoundary, Bool bLeftTileBoundary, Bool bRightTileBoundary
                                                    ,Bool bIndependentTileBoundaryEnabled)
{
  UInt numSUInLCU = numSUInLCUWidth*numSUInLCUHeight;
  Int* pSliceIDMapLCU = m_piSliceSUMap;
#if MODIFIED_CROSS_SLICE
  Bool onlyOneSliceInPic = ((Int)LFCrossSliceBoundary.size() == 1);
#endif
  UInt uiLPelX, uiTPelY;
  UInt width, height;
  Bool bPicRBoundary, bPicBBoundary, bPicTBoundary, bPicLBoundary;
  Bool bLCURBoundary= false, bLCUBBoundary= false, bLCUTBoundary= false, bLCULBoundary= false;
  Bool* pbAvailBorder;
  Bool* pbAvail;
  UInt rTLSU, rBRSU, widthSU, heightSU;
  UInt zRefSU;
  Int* pRefID;
  Int* pRefMapLCU;
  UInt rTRefSU= 0, rBRefSU= 0, rLRefSU= 0, rRRefSU= 0;
  Int* pRRefMapLCU= NULL;
  Int* pLRefMapLCU= NULL;
  Int* pTRefMapLCU= NULL;
  Int* pBRefMapLCU= NULL;
  Int  sliceID;
  UInt numSGU = (UInt)m_vNDFBlock.size();

  for(Int i=0; i< numSGU; i++)
  {
    NDBFBlockInfo& rSGU = m_vNDFBlock[i];

    sliceID = rSGU.sliceID;
    uiLPelX = rSGU.posX;
    uiTPelY = rSGU.posY;
    width   = rSGU.width;
    height  = rSGU.height;
#if !MODIFIED_CROSS_SLICE
    Bool bIndependentSliceBoundaryEnabled = !(LFCrossSliceBoundary[sliceID]);
#endif
    rTLSU     = g_auiZscanToRaster[ rSGU.startSU ];
    rBRSU     = g_auiZscanToRaster[ rSGU.endSU   ];
    widthSU   = rSGU.widthSU;
    heightSU  = rSGU.heightSU;

    pbAvailBorder = rSGU.isBorderAvailable;

    bPicTBoundary= (uiTPelY == 0                       )?(true):(false);
    bPicLBoundary= (uiLPelX == 0                       )?(true):(false);
    bPicRBoundary= (!(uiLPelX+ width < picWidth )  )?(true):(false);
    bPicBBoundary= (!(uiTPelY + height < picHeight))?(true):(false);

    bLCULBoundary = (rTLSU % numSUInLCUWidth == 0)?(true):(false);
    bLCURBoundary = ( (rTLSU+ widthSU) % numSUInLCUWidth == 0)?(true):(false);
    bLCUTBoundary = ( (UInt)(rTLSU / numSUInLCUWidth)== 0)?(true):(false);
    bLCUBBoundary = ( (UInt)(rBRSU / numSUInLCUWidth) == (numSUInLCUHeight-1) )?(true):(false);

    //       SGU_L
    pbAvail = &(pbAvailBorder[SGU_L]);
    if(bPicLBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      //      bLCULBoundary = (rTLSU % uiNumSUInLCUWidth == 0)?(true):(false);
      if(bLCULBoundary)
      {
        rLRefSU     = rTLSU + numSUInLCUWidth -1;
        zRefSU      = g_auiRasterToZscan[rLRefSU];
        pRefMapLCU = pLRefMapLCU= (pSliceIDMapLCU - numSUInLCU);
      }
      else
      {
        zRefSU   = g_auiRasterToZscan[rTLSU - 1];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_R
    pbAvail = &(pbAvailBorder[SGU_R]);
    if(bPicRBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      //       bLCURBoundary = ( (rTLSU+ uiWidthSU) % uiNumSUInLCUWidth == 0)?(true):(false);
      if(bLCURBoundary)
      {
        rRRefSU      = rTLSU + widthSU - numSUInLCUWidth;
        zRefSU       = g_auiRasterToZscan[rRRefSU];
        pRefMapLCU  = pRRefMapLCU= (pSliceIDMapLCU + numSUInLCU);
      }
      else
      {
        zRefSU       = g_auiRasterToZscan[rTLSU + widthSU];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_T
    pbAvail = &(pbAvailBorder[SGU_T]);
    if(bPicTBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      //      bLCUTBoundary = ( (UInt)(rTLSU / uiNumSUInLCUWidth)== 0)?(true):(false);
      if(bLCUTBoundary)
      {
        rTRefSU      = numSUInLCU - (numSUInLCUWidth - rTLSU);
        zRefSU       = g_auiRasterToZscan[rTRefSU];
        pRefMapLCU  = pTRefMapLCU= (pSliceIDMapLCU - (numLCUInPicWidth*numSUInLCU));
      }
      else
      {
        zRefSU       = g_auiRasterToZscan[rTLSU - numSUInLCUWidth];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_B
    pbAvail = &(pbAvailBorder[SGU_B]);
    if(bPicBBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      //      bLCUBBoundary = ( (UInt)(rBRSU / uiNumSUInLCUWidth) == (uiNumSUInLCUHeight-1) )?(true):(false);
      if(bLCUBBoundary)
      {
        rBRefSU      = rTLSU % numSUInLCUWidth;
        zRefSU       = g_auiRasterToZscan[rBRefSU];
        pRefMapLCU  = pBRefMapLCU= (pSliceIDMapLCU + (numLCUInPicWidth*numSUInLCU));
      }
      else
      {
        zRefSU       = g_auiRasterToZscan[rTLSU + (heightSU*numSUInLCUWidth)];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_TL
    pbAvail = &(pbAvailBorder[SGU_TL]);
    if(bPicTBoundary || bPicLBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      if(bLCUTBoundary && bLCULBoundary)
      {
        zRefSU       = numSUInLCU -1;
        pRefMapLCU  = pSliceIDMapLCU - ( (numLCUInPicWidth+1)*numSUInLCU);
      }
      else if(bLCUTBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rTRefSU- 1];
        pRefMapLCU  = pTRefMapLCU;
      }
      else if(bLCULBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rLRefSU- numSUInLCUWidth ];
        pRefMapLCU  = pLRefMapLCU;
      }
      else //inside LCU
      {
        zRefSU       = g_auiRasterToZscan[ rTLSU - numSUInLCUWidth -1];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_TR
    pbAvail = &(pbAvailBorder[SGU_TR]);
    if(bPicTBoundary || bPicRBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      if(bLCUTBoundary && bLCURBoundary)
      {
        zRefSU      = g_auiRasterToZscan[numSUInLCU - numSUInLCUWidth];
        pRefMapLCU  = pSliceIDMapLCU - ( (numLCUInPicWidth-1)*numSUInLCU);        
      }
      else if(bLCUTBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rTRefSU+ widthSU];
        pRefMapLCU  = pTRefMapLCU;
      }
      else if(bLCURBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rRRefSU- numSUInLCUWidth ];
        pRefMapLCU  = pRRefMapLCU;
      }
      else //inside LCU
      {
        zRefSU       = g_auiRasterToZscan[ rTLSU - numSUInLCUWidth +widthSU];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_BL
    pbAvail = &(pbAvailBorder[SGU_BL]);
    if(bPicBBoundary || bPicLBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      if(bLCUBBoundary && bLCULBoundary)
      {
        zRefSU      = g_auiRasterToZscan[numSUInLCUWidth - 1];
        pRefMapLCU  = pSliceIDMapLCU + ( (numLCUInPicWidth-1)*numSUInLCU);        
      }
      else if(bLCUBBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rBRefSU - 1];
        pRefMapLCU  = pBRefMapLCU;
      }
      else if(bLCULBoundary)
      {
        zRefSU       = g_auiRasterToZscan[ rLRefSU+ heightSU*numSUInLCUWidth ];
        pRefMapLCU  = pLRefMapLCU;
      }
      else //inside LCU
      {
        zRefSU       = g_auiRasterToZscan[ rTLSU + heightSU*numSUInLCUWidth -1];
        pRefMapLCU  = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    //       SGU_BR
    pbAvail = &(pbAvailBorder[SGU_BR]);
    if(bPicBBoundary || bPicRBoundary)
    {
      *pbAvail = false;
    }
#if MODIFIED_CROSS_SLICE
    else if (onlyOneSliceInPic)
#else
    else if (!bIndependentSliceBoundaryEnabled)
#endif
    {
      *pbAvail = true;
    }
    else
    {
      if(bLCUBBoundary && bLCURBoundary)
      {
        zRefSU = 0;
        pRefMapLCU = pSliceIDMapLCU+ ( (numLCUInPicWidth+1)*numSUInLCU);
      }
      else if(bLCUBBoundary)
      {
        zRefSU      = g_auiRasterToZscan[ rBRefSU + widthSU];
        pRefMapLCU = pBRefMapLCU;
      }
      else if(bLCURBoundary)
      {
        zRefSU      = g_auiRasterToZscan[ rRRefSU + (heightSU*numSUInLCUWidth)];
        pRefMapLCU = pRRefMapLCU;
      }
      else //inside LCU
      {
        zRefSU      = g_auiRasterToZscan[ rTLSU + (heightSU*numSUInLCUWidth)+ widthSU];
        pRefMapLCU = pSliceIDMapLCU;
      }
      pRefID = pRefMapLCU + zRefSU;
#if MODIFIED_CROSS_SLICE
      *pbAvail = (*pRefID == sliceID)?(true):((*pRefID > sliceID)?(LFCrossSliceBoundary[*pRefID]):(LFCrossSliceBoundary[sliceID]));
#else
      *pbAvail = (*pRefID == sliceID)?(true):(false);
#endif
    }

    if(bIndependentTileBoundaryEnabled)
    {
      //left LCU boundary
      if(!bPicLBoundary && bLCULBoundary)
      {
        if(bLeftTileBoundary)
        {
          pbAvailBorder[SGU_L] = pbAvailBorder[SGU_TL] = pbAvailBorder[SGU_BL] = false;
        }
      }
      //right LCU boundary
      if(!bPicRBoundary && bLCURBoundary)
      {
        if(bRightTileBoundary)
        {
          pbAvailBorder[SGU_R] = pbAvailBorder[SGU_TR] = pbAvailBorder[SGU_BR] = false;
        }
      }
      //top LCU boundary
      if(!bPicTBoundary && bLCUTBoundary)
      {
        if(bTopTileBoundary)
        {
          pbAvailBorder[SGU_T] = pbAvailBorder[SGU_TL] = pbAvailBorder[SGU_TR] = false;
        }
      }
      //down LCU boundary
      if(!bPicBBoundary && bLCUBBoundary)
      {
        if(bDownTileBoundary)
        {
          pbAvailBorder[SGU_B] = pbAvailBorder[SGU_BL] = pbAvailBorder[SGU_BR] = false;
        }
      }
    }
    rSGU.allBordersAvailable = true;
    for(Int b=0; b< NUM_SGU_BORDER; b++)
    {
      if(pbAvailBorder[b] == false)
      {
        rSGU.allBordersAvailable = false;
        break;
      }
    }
  }
}


//! \}
