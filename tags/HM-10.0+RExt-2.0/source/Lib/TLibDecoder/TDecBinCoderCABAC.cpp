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

/** \file     TDecBinCoderCABAC.cpp
    \brief    binary entropy decoder of CABAC
*/

#include "TDecBinCoderCABAC.h"
#include "TLibCommon/Debug.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif

//! \ingroup TLibDecoder
//! \{

TDecBinCABAC::TDecBinCABAC()
: m_pcTComBitstream( 0 )
{
}

TDecBinCABAC::~TDecBinCABAC()
{
}

Void
TDecBinCABAC::init( TComInputBitstream* pcTComBitstream )
{
  m_pcTComBitstream = pcTComBitstream;
}

Void
TDecBinCABAC::uninit()
{
  m_pcTComBitstream = 0;
}

Void
TDecBinCABAC::start()
{
  assert( m_pcTComBitstream->getNumBitsUntilByteAligned() == 0 );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::UpdateCABACStat(STATS__CABAC_INITIALISATION, 512, 510, 0);
#endif
  m_uiRange    = 510;
  m_bitsNeeded = -8;
  m_uiValue    = (m_pcTComBitstream->readByte() << 8);
  m_uiValue   |= m_pcTComBitstream->readByte();
}

Void
TDecBinCABAC::finish()
{
}

Void 
TDecBinCABAC::flush()
{
  while (m_pcTComBitstream->getNumBitsLeft() > 0 && m_pcTComBitstream->getNumBitsUntilByteAligned() != 0)
  {
    UInt uiBits;
    m_pcTComBitstream->read ( 1, uiBits );
  }
  start();
}

/**
 - Copy CABAC state.
 .
 \param pcTDecBinIf The source CABAC engine.
 */
Void
TDecBinCABAC::copyState( TDecBinIf* pcTDecBinIf )
{
  TDecBinCABAC* pcTDecBinCABAC = pcTDecBinIf->getTDecBinCABAC();
  m_uiRange   = pcTDecBinCABAC->m_uiRange;
  m_uiValue   = pcTDecBinCABAC->m_uiValue;
  m_bitsNeeded= pcTDecBinCABAC->m_bitsNeeded;
}



#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecBinCABAC::decodeBin( UInt& ruiBin, ContextModel &rcCtxModel, const TComCodingStatisticsClassType &whichStat )
#else
Void TDecBinCABAC::decodeBin( UInt& ruiBin, ContextModel &rcCtxModel )
#endif
{
#ifdef DEBUG_CABAC_BINS
  const UInt startingRange = m_uiRange;
#endif

  UInt uiLPS = TComCABACTables::sm_aucLPSTable[ rcCtxModel.getState() ][ ( m_uiRange >> 6 ) - 4 ];
  m_uiRange -= uiLPS;
  UInt scaledRange = m_uiRange << 7;
  
  if( m_uiValue < scaledRange )
  {
    // MPS path
    ruiBin = rcCtxModel.getMps();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::UpdateCABACStat(whichStat, m_uiRange+uiLPS, m_uiRange, Int(ruiBin));
#endif
    rcCtxModel.updateMPS();
    
    if ( scaledRange < ( 256 << 7 ) )
    {
      m_uiRange = scaledRange >> 6;
      m_uiValue += m_uiValue;
      
      if ( ++m_bitsNeeded == 0 )
      {
        m_bitsNeeded = -8;
        m_uiValue += m_pcTComBitstream->readByte();      
      }
    }
  }
  else
  {
    // LPS path
    ruiBin      = 1 - rcCtxModel.getMps();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::UpdateCABACStat(whichStat, m_uiRange+uiLPS, uiLPS, Int(ruiBin));
#endif
    Int numBits = TComCABACTables::sm_aucRenormTable[ uiLPS >> 3 ];
    m_uiValue   = ( m_uiValue - scaledRange ) << numBits;
    m_uiRange   = uiLPS << numBits;
    rcCtxModel.updateLPS();
    
    m_bitsNeeded += numBits;
    
    if ( m_bitsNeeded >= 0 )
    {
      m_uiValue += m_pcTComBitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded -= 8;
    }
  }

#ifdef DEBUG_CABAC_BINS
  if ((g_debugCounter + debugCabacBinWindow) >= debugCabacBinTargetLine)
    std::cout << g_debugCounter << ": coding bin value " << ruiBin << ", range = [" << startingRange << "->" << m_uiRange << "]\n";

  if (g_debugCounter >= debugCabacBinTargetLine)
  {
    char breakPointThis;
    breakPointThis = 7;
  }
  if (g_debugCounter >= (debugCabacBinTargetLine + debugCabacBinWindow)) exit(0);
  g_debugCounter++;
#endif
}


#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecBinCABAC::decodeBinEP( UInt& ruiBin, const TComCodingStatisticsClassType &whichStat )
#else
Void TDecBinCABAC::decodeBinEP( UInt& ruiBin )
#endif
{
  m_uiValue += m_uiValue;
  
  if ( ++m_bitsNeeded >= 0 )
  {
    m_bitsNeeded = -8;
    m_uiValue += m_pcTComBitstream->readByte();
  }
  
  ruiBin = 0;
  UInt scaledRange = m_uiRange << 7;
  if ( m_uiValue >= scaledRange )
  {
    ruiBin = 1;
    m_uiValue -= scaledRange;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(whichStat, 1, Int(ruiBin));
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecBinCABAC::decodeBinsEP( UInt& ruiBin, Int numBins, const TComCodingStatisticsClassType &whichStat )
#else
Void TDecBinCABAC::decodeBinsEP( UInt& ruiBin, Int numBins )
#endif
{
  UInt bins = 0;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  Int origNumBins=numBins;
#endif
  while ( numBins > 8 )
  {
    m_uiValue = ( m_uiValue << 8 ) + ( m_pcTComBitstream->readByte() << ( 8 + m_bitsNeeded ) );
    
    UInt scaledRange = m_uiRange << 15;
    for ( Int i = 0; i < 8; i++ )
    {
      bins += bins;
      scaledRange >>= 1;
      if ( m_uiValue >= scaledRange )
      {
        bins++;
        m_uiValue -= scaledRange;
      }
    }
    numBins -= 8;
  }
  
  m_bitsNeeded += numBins;
  m_uiValue <<= numBins;
  
  if ( m_bitsNeeded >= 0 )
  {
    m_uiValue += m_pcTComBitstream->readByte() << m_bitsNeeded;
    m_bitsNeeded -= 8;
  }
  
  UInt scaledRange = m_uiRange << ( numBins + 7 );
  for ( Int i = 0; i < numBins; i++ )
  {
    bins += bins;
    scaledRange >>= 1;
    if ( m_uiValue >= scaledRange )
    {
      bins++;
      m_uiValue -= scaledRange;
    }
  }
  
  ruiBin = bins;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(whichStat, origNumBins, Int(ruiBin));
#endif

}

Void
TDecBinCABAC::decodeBinTrm( UInt& ruiBin )
{
  m_uiRange -= 2;
  UInt scaledRange = m_uiRange << 7;
  if( m_uiValue >= scaledRange )
  {
    ruiBin = 1;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::UpdateCABACStat(STATS__CABAC_TRM_BITS, m_uiRange+2, 2, ruiBin);
    TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS, -m_bitsNeeded, 0);
#endif
  }
  else
  {
    ruiBin = 0;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::UpdateCABACStat(STATS__CABAC_TRM_BITS, m_uiRange+2, m_uiRange, ruiBin);
#endif
    if ( scaledRange < ( 256 << 7 ) )
    {
      m_uiRange = scaledRange >> 6;
      m_uiValue += m_uiValue;
      
      if ( ++m_bitsNeeded == 0 )
      {
        m_bitsNeeded = -8;
        m_uiValue += m_pcTComBitstream->readByte();      
      }
    }
  }
}

/** Reset BAC register values.
 * \returns Void
 */
Void TDecBinCABAC::resetBac()
{
  m_uiRange    = 510;
  m_bitsNeeded = -8;
  m_uiValue    = m_pcTComBitstream->read( 16 );
}

/** Decode PCM alignment zero bits.
 * \returns Void
 */
Void TDecBinCABAC::decodePCMAlignBits()
{
  Int iNum = m_pcTComBitstream->getNumBitsUntilByteAligned();
  
  UInt uiBit = 0;
  m_pcTComBitstream->read( iNum, uiBit );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(STATS__CABAC_PCM_ALIGN_BITS, iNum, 0);
#endif
}

/** Read a PCM code.
 * \param uiLength code bit-depth
 * \param ruiCode pointer to PCM code value
 * \returns Void
 */
Void  TDecBinCABAC::xReadPCMCode(UInt uiLength, UInt& ruiCode)
{
  assert ( uiLength > 0 );
  m_pcTComBitstream->read (uiLength, ruiCode);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(STATS__CABAC_PCM_CODE_BITS, uiLength, ruiCode);
#endif
}
//! \}
