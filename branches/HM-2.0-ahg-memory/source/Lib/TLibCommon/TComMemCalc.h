/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and   contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2010, NEC, Panasonic, and TI
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The names of NEC, Panasonic, and TI
      may not be used to endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ====================================================================================================================
*/

/** \file     TComMemCalc.h
    \brief    frame memory access calculator class (header)
*/

#ifndef __TCOMMEMCALC__
#define __TCOMMEMCALC__

#include "TComDataCU.h"
#include <algorithm>

// ====================================================================================================================
// Struct definition
// ====================================================================================================================

struct MemCmpParam
{
  Int iUnitWidth;       //! width of memory compression unit
  Int iUnitHeight;      //! height of memory compression unit
  Int iCmpRatioNum;     //! compression ratio = m_iCmpRatioNum/m_iCmpRatioDenom;
  Int iCmpRatioDenom;   //! compression ratio = m_iCmpRatioNum/m_iCmpRatioDenom;

  MemCmpParam()
  {
    iUnitWidth = 1;
    iUnitHeight = 1;
    iCmpRatioNum = 1;
    iCmpRatioDenom = 1;
  }

  MemCmpParam& operator=( const MemCmpParam& cParamSet )
  {
    iUnitWidth = cParamSet.iUnitWidth;
    iUnitHeight = cParamSet.iUnitHeight;
    iCmpRatioNum = cParamSet.iCmpRatioNum;
    iCmpRatioDenom = cParamSet.iCmpRatioDenom;
    return *this;
  }
};

struct CacheElement
{
  UInt64 ui64LineAddress;  // Address of line being stored
  Bool   bValid;           // Indicates whether cache element is valid
  UInt   uiLRUCount;
};


// ====================================================================================================================
// Virtual cache class definition
// ====================================================================================================================

class TComVirtualCache
{
public:
  enum {
    DIRECT_MAPPED      = 0,
    TWO_WAY            = 1,
    FOUR_WAY           = 2,
    EIGHT_WAY          = 3,
    FULL_ASSOCIATIVITY = sizeof(Int)*8,
    DEFAULT_LOG2_CACHE_LINE_SIZE = 6,
    DEFAULT_LOG2_CACHE_NUM_LINES = 11,
  };

public:
  TComVirtualCache();
  virtual ~TComVirtualCache();

  Void initCache( Int iLog2Associativity, Int iLog2CacheLineSize, Int iLog2NumCacheLines );
  Int xCalcCacheMisshitBytes( UInt64 ui64Address );

  Int getLog2Associativity( Void )  { return m_iLog2Associativity; }
  Int getLog2CacheLineSize( Void )  { return m_iLog2CacheLineSize; }
  Int getLog2NumCacheLines( Void )  { return m_iLog2NumCacheLines; }
  Int getCacheLineSize( Void )      { return 1<<m_iLog2CacheLineSize; }
  Int getNumCacheLines( Void )      { return 1<<m_iLog2NumCacheLines; }
  Int getCacheSize( Void )          { return getNumCacheLines() * getCacheLineSize(); }

protected:
  Int getNumCachePartitions( Void ) { return Max(1, getNumCacheLines()>>getLog2Associativity()); }

  virtual Void buildCache( Void ) = 0;
  virtual Void clearCache( Void ) = 0;
  virtual Void updateCache( Int iLineIdx ) = 0;
  virtual Void replaceCache( Int iPartition, Int iSubCacheSize, UInt64 ui64LineAddress ) = 0;

protected:
  Int m_iLog2Associativity;
  Int m_iLog2CacheLineSize;
  Int m_iLog2NumCacheLines;
  CacheElement* m_pcCacheLines;
};

class TComVirtualCacheFIFO : public TComVirtualCache
{
public:
  TComVirtualCacheFIFO();
  virtual ~TComVirtualCacheFIFO();

protected:
  virtual Void buildCache( Void );
  virtual Void clearCache( Void );
  virtual Void updateCache( Int iLineIdx );
  virtual Void replaceCache( Int iPartition, Int iSubCacheSize, UInt64 ui64LineAddress );

protected:
  Int* m_piCachePtrs;
};

class TComVirtualCacheLRU : public TComVirtualCache
{
public:
  TComVirtualCacheLRU();
  virtual ~TComVirtualCacheLRU();

protected:
  virtual Void buildCache( Void );
  virtual Void clearCache( Void );
  virtual Void updateCache( Int iLineIdx );
  virtual Void replaceCache( Int iPartition, Int iSubCacheSize, UInt64 ui64LineAddress );
};


// ====================================================================================================================
// Memory architecture class definition
// ====================================================================================================================

class TComMemoryArchitecture
{
public:
  TComMemoryArchitecture();
  virtual ~TComMemoryArchitecture();

  Void initMemoryArchitecture( Int iAlignmentBits, Int iBurstReadBits );
  Void attachCache( TComVirtualCache* pcCache );

  Int getAlignmentBits( Void )       { return m_iAlignmentBits; }
  Int getBurstReadBits( Void )       { return m_iBurstReadBits; }
  TComVirtualCache* getCache( Void ) { return m_pcCache; }

protected:
  Int               m_iAlignmentBits; //! alignment size of memory access unit
  Int               m_iBurstReadBits; //! burst read size of memory access
  TComVirtualCache* m_pcCache;
};


// ====================================================================================================================
// Frame memory access calculator class definition
// ====================================================================================================================

class TComFrameMemoryAccessCalculator : public TComMemoryArchitecture
{
public:
  TComFrameMemoryAccessCalculator();
  virtual ~TComFrameMemoryAccessCalculator();

  virtual Int xCalcAccessBytes(Int iMCBlockWidth, Int iMCBlockHeight, Int iMCBlockPosX, Int iMCBlockPosY, Int iFilterTapH, Int iFilterTapV, Bool bChroma);
  Void   update( Void );

  Void   setLumaBitDepth( Int iBitDepth )   { m_iLumaBitDepth = iBitDepth; }
  Void   setChromaBitDepth( Int iBitDepth ) { m_iChromaBitDepth = iBitDepth; }
  Void   setPictureWidth( Int iPicWidth )   { m_iPicWidth = iPicWidth; }
  Void   setPictureIndex( Int iPicIndex )   { m_iPicIndex = iPicIndex; }
  Void   resetTotalAccessBytes( Void )      { m_ui64TotalAccessBytes = 0; }

  Int    getLumaBitDepth( Void )            { return m_iLumaBitDepth; }
  Int    getChromaBitDepth( Void )          { return m_iChromaBitDepth; }
  Int    getPictureWidth( Void )            { return m_iPicWidth; }
  Int    getPictureIndex( Void )            { return m_iPicIndex; }
  UInt64 getCurrAccessBytes( Void )         { return m_ui64CurrAccessBytes; }
  UInt64 getMaxAccessBytes( Void )          { return m_ui64MaxAccessBytes; }
  UInt64 getTotalAccessBytes( Void )        { return m_ui64TotalAccessBytes; }

protected:
  Void   addAccessBytes( Int iBytes )       { m_ui64CurrAccessBytes += iBytes; }

protected:
  Int     m_iLumaBitDepth;
  Int     m_iChromaBitDepth;
  Int     m_iPicWidth;
  Int     m_iPicIndex;
  UInt64  m_ui64CurrAccessBytes;
  UInt64  m_ui64MaxAccessBytes;
  UInt64  m_ui64TotalAccessBytes;
};


// ====================================================================================================================
// Compressed frame memory access calculator class definition
// ====================================================================================================================

class TComCompressedFrameMemoryAccessCalculator : public TComFrameMemoryAccessCalculator
{
public:
  TComCompressedFrameMemoryAccessCalculator();
  virtual ~TComCompressedFrameMemoryAccessCalculator();

  virtual Int xCalcAccessBytes(Int iMCBlockWidth, Int iMCBlockHeight, Int iMCBlockPosX, Int iMCBlockPosY, Int iFilterTapH, Int iFilterTapV, Bool bChroma);

  Void setLumaMemoryCompressionParameterSet( const MemCmpParam& cParamSet )   { m_cLumaParam = cParamSet; }
  Void setChromaMemoryCompressionParameterSet( const MemCmpParam& cParamSet ) { m_cChromaParam = cParamSet; }

  MemCmpParam* getLumaMemoryCompressionParameterSet( Void )                   { return &m_cLumaParam; }
  MemCmpParam* getChromaMemoryCompressionParameterSet( Void )                 { return &m_cChromaParam; }

protected:
  MemCmpParam m_cLumaParam;
  MemCmpParam m_cChromaParam;
};

#endif // __TCOMMEMCALC__
