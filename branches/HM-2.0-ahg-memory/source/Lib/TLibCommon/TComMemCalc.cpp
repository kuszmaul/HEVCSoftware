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

/** \file     TComMemCalc.cpp
    \brief    frame memory access calculator class
*/

#include "TComMemCalc.h"

namespace {
  // \brief Determine aligned address
  //
  // \param iAddr unaligned address
  // \param iAccessUnit alignment unit
  // \return aligned address
  template <typename T>
  __inline T AlignRoundDown( T iAddr, Int iAccessUnit )
  {
    T iAlignedAddr = iAccessUnit * (iAddr/iAccessUnit);
    if (iAddr < 0)
    {
      iAlignedAddr -= (iAddr%iAccessUnit) ? iAccessUnit : 0;
    }
    return iAlignedAddr;
  }

  // \brief Determine aligned address
  //
  // \param iAddr unaligned address
  // \param iAccessUnit alignment unit
  // \return aligned address
  template <typename T>
  __inline T AlignRoundUp( T iAddr, Int iAccessUnit )
  {
    return iAccessUnit * (iAddr/iAccessUnit) + ((iAddr%iAccessUnit) ? iAccessUnit : 0);
  }
}


// ====================================================================================================================
// Memory architecture class implementation
// ====================================================================================================================

TComMemoryArchitecture::TComMemoryArchitecture()
: m_iAlignmentBits(8)
, m_iBurstReadBits(8)
, m_pcCache(0)
{
}

TComMemoryArchitecture::~TComMemoryArchitecture()
{
}

Void TComMemoryArchitecture::initMemoryArchitecture( Int iAlignmentBits, Int iBurstReadBits )
{
  m_iAlignmentBits = iAlignmentBits;
  m_iBurstReadBits = iBurstReadBits;
}

Void TComMemoryArchitecture::attachCache( TComVirtualCache* pcCache )
{
  m_pcCache = pcCache;
}


// ====================================================================================================================
// Virtual cache class implementation
// ====================================================================================================================

TComVirtualCache::TComVirtualCache()
: m_iLog2Associativity(TComVirtualCache::FOUR_WAY)
, m_iLog2CacheLineSize(6)
, m_iLog2NumCacheLines(11)
, m_pcCacheLines(0)
{
}

TComVirtualCache::~TComVirtualCache()
{
  if (m_pcCacheLines) xFree(m_pcCacheLines);
}

Void TComVirtualCache::initCache( Int iLog2Associativity, Int iLog2CacheLineSize, Int iLog2NumCacheLines )
{
  assert(iLog2Associativity >= DIRECT_MAPPED);
  assert(iLog2CacheLineSize >= 0);
  assert(iLog2NumCacheLines >= 0);

  m_iLog2Associativity = iLog2Associativity;
  m_iLog2CacheLineSize = iLog2CacheLineSize;
  m_iLog2NumCacheLines = iLog2NumCacheLines;

  buildCache();
  clearCache();
}

Int TComVirtualCache::xCalcCacheMisshitBytes( UInt64 ui64Address )
{
  /* Get line address of data being accessed */
  UInt64 ui64LineAddress = ui64Address >> getLog2CacheLineSize();

  Int iPartition = ui64LineAddress & ((getNumCacheLines()>>getLog2Associativity()) - 1);
  Int iSubCacheSize = 1 << Min(getLog2Associativity(), getLog2NumCacheLines());

  /* Search within partition to see if its a cache hit. BW = 0 for cache hit. */
  Int iSearchStart = iPartition*iSubCacheSize;
  Int iSearchEnd = iSearchStart + iSubCacheSize;
  for (Int i=iSearchStart; i<iSearchEnd; i++)
  {
    if (m_pcCacheLines[i].bValid && m_pcCacheLines[i].ui64LineAddress == ui64LineAddress)
    {
      updateCache(i);
      return 0;
    }
  }

  /* Cache miss. Insert new data in cache. BW = CACHE_LINE_SIZE*8 bits for cache miss. */
  replaceCache(iPartition, iSubCacheSize, ui64LineAddress);
  return getCacheLineSize();
}

TComVirtualCacheFIFO::TComVirtualCacheFIFO()
: m_piCachePtrs(0)
{
}

TComVirtualCacheFIFO::~TComVirtualCacheFIFO()
{
  if (m_piCachePtrs)  xFree(m_piCachePtrs);
}

Void TComVirtualCacheFIFO::buildCache( Void )
{
  if (m_pcCacheLines) xFree(m_pcCacheLines);
  if (m_piCachePtrs)  xFree(m_piCachePtrs);
  m_pcCacheLines = (CacheElement*) xMalloc(CacheElement, getNumCacheLines());
  m_piCachePtrs = (Int*) xMalloc(Int, getNumCachePartitions());
}

Void TComVirtualCacheFIFO::clearCache( Void )
{
  if (m_pcCacheLines) memset(m_pcCacheLines, 0, getNumCacheLines() * sizeof(CacheElement));
  if (m_piCachePtrs)  memset(m_piCachePtrs, 0, getNumCachePartitions() * sizeof(Int));
}

Void TComVirtualCacheFIFO::updateCache( Int iLineIdx )
{
  // do nothing
}

Void TComVirtualCacheFIFO::replaceCache( Int iPartition, Int iSubCacheSize, UInt64 ui64LineAddress )
{
  Int iSearchStart = iPartition*iSubCacheSize;
  m_pcCacheLines[iSearchStart+m_piCachePtrs[iPartition]].ui64LineAddress = ui64LineAddress;
  m_pcCacheLines[iSearchStart+m_piCachePtrs[iPartition]].bValid = true;
  m_piCachePtrs[iPartition] = (m_piCachePtrs[iPartition]+1) % iSubCacheSize;
}

TComVirtualCacheLRU::TComVirtualCacheLRU()
{
}

TComVirtualCacheLRU::~TComVirtualCacheLRU()
{
}

Void TComVirtualCacheLRU::buildCache( Void )
{
  if (m_pcCacheLines) xFree(m_pcCacheLines);
  m_pcCacheLines = (CacheElement*) xMalloc(CacheElement, getNumCacheLines());
}

Void TComVirtualCacheLRU::clearCache( Void )
{
  if (m_pcCacheLines) memset(m_pcCacheLines, 0, getNumCacheLines() * sizeof(CacheElement));
}

Void TComVirtualCacheLRU::updateCache( Int iLineIdx )
{
  assert(m_pcCacheLines[iLineIdx].uiLRUCount<MAX_UINT);
  m_pcCacheLines[iLineIdx].uiLRUCount++;
}

Void TComVirtualCacheLRU::replaceCache( Int iPartition, Int iSubCacheSize, UInt64 ui64LineAddress )
{
  Int iSearchStart = iPartition*iSubCacheSize;
  Int iSearchEnd = iSearchStart + iSubCacheSize;

  UInt uiMinLRUCount = MAX_UINT;
  UInt uiMaxLRUCount = 0;
  Int iMinLRUIdx = iSearchStart;
  for (Int i=iSearchStart; i<iSearchEnd; i++)
  {
    if (m_pcCacheLines[i].uiLRUCount < uiMinLRUCount)
    {
      uiMinLRUCount = m_pcCacheLines[i].uiLRUCount;
      iMinLRUIdx = i;
    }
    if (m_pcCacheLines[i].uiLRUCount > uiMaxLRUCount)
    {
      uiMaxLRUCount = m_pcCacheLines[i].uiLRUCount;
    }
  }

  m_pcCacheLines[iMinLRUIdx].ui64LineAddress = ui64LineAddress;
  m_pcCacheLines[iMinLRUIdx].bValid = true;
  m_pcCacheLines[iMinLRUIdx].uiLRUCount = uiMaxLRUCount + 1;

  for (Int i=iSearchStart; i<iSearchEnd; i++)
  {
    m_pcCacheLines[i].uiLRUCount -= uiMinLRUCount;
  }
}


// ====================================================================================================================
// Frame memory access calculator class implementation
// ====================================================================================================================

TComFrameMemoryAccessCalculator::TComFrameMemoryAccessCalculator()
: m_iLumaBitDepth(8)
, m_iChromaBitDepth(8)
, m_iPicWidth(0)
, m_iPicIndex(0)
, m_ui64CurrAccessBytes(0)
, m_ui64MaxAccessBytes(0)
, m_ui64TotalAccessBytes(0)
{
}

TComFrameMemoryAccessCalculator::~TComFrameMemoryAccessCalculator()
{
}

Int TComFrameMemoryAccessCalculator::xCalcAccessBytes(Int iMCBlockWidth, Int iMCBlockHeight, Int iMCBlockPosX, Int iMCBlockPosY, Int iFilterTapH, Int iFilterTapV, Bool bChroma)
{
  Int iMul = bChroma ? 2 : 1;
  Int iBitDepth = bChroma ? m_iChromaBitDepth    : m_iLumaBitDepth;
  // should be changed for supporting 4:4:4 format
  Int iPicWidth = bChroma ? getPictureWidth()>>1 : getPictureWidth();

  /* initialization */
  Int iMCBlockOverheadLeft = Max(0, (iFilterTapH>>1)-1);
  Int iMCBlockOverheadRight = (iFilterTapH>>1) + (iFilterTapH&0x1);
  Int iMCBlockOverheadTop = Max(0, (iFilterTapV>>1)-1);
  Int iMCBlockOverheadBottom = (iFilterTapV>>1) + (iFilterTapV&0x1);

  /* derive pixel positions */
  Int iLeftPos   = iMul * (iMCBlockPosX - iMCBlockOverheadLeft);
  Int iRightPos  = iMul * (iMCBlockPosX + iMCBlockWidth + iMCBlockOverheadRight);
  Int iTopPos    = (iMCBlockPosY - iMCBlockOverheadTop);
  Int iBottomPos = (iMCBlockPosY + iMCBlockHeight + iMCBlockOverheadBottom);

  Int64 i64StartAddress = iBitDepth * (iMul*static_cast<Int64>(iPicWidth) * iTopPos + iLeftPos);
  Int iNumHorizontalSamples = iRightPos - iLeftPos;

  /* derive virtual memory access */
  Int iMemAccessBytes = 0;
  for (Int y=0; y<(iBottomPos-iTopPos); y++)
  {
    /* derive read address */
    Int64 i64LeftAddress  = i64StartAddress;
    Int64 i64RightAddress = i64StartAddress + iBitDepth*iNumHorizontalSamples;

    /* derive memory aligned read address */
    Int64 i64MemAlignedLeftAddress  = AlignRoundDown (i64LeftAddress,  getAlignmentBits());
    Int64 i64MemAlignedRightAddress = AlignRoundUp   (i64RightAddress, getAlignmentBits());

    /* derive memory aligned read address */
    Int iMemAccessBytesPerLine = AlignRoundUp(static_cast<int>(i64MemAlignedRightAddress - i64MemAlignedLeftAddress), getBurstReadBits()) >> 3;

    if (m_pcCache)
    {
      /* access via cache */
      static const UInt64 MAX_FRAME_SIZE_BITS = UInt64(8192)*4096*16;
      UInt64 ui64VirtualFrameByteAddress = (static_cast<UInt64>(getPictureIndex()*2+static_cast<Int>(bChroma)+1) * MAX_FRAME_SIZE_BITS) >> 3;
      for (Int n=0; n<iMemAccessBytesPerLine; n+=getAlignmentBits()/8)
      {
        UInt64 ui64ByteAddress = ui64VirtualFrameByteAddress + (i64MemAlignedLeftAddress>>3) + n;
        iMemAccessBytes += m_pcCache->xCalcCacheMisshitBytes(ui64ByteAddress);
      }
    }
    else
    {
      /* access directly to memory */
      iMemAccessBytes += iMemAccessBytesPerLine;
    }

    /* update the start address of the first access unit of the next compression-unit row. */
    i64StartAddress += iBitDepth * (iMul*iPicWidth);
  }

  addAccessBytes(iMemAccessBytes);
  return iMemAccessBytes;
}

Void TComFrameMemoryAccessCalculator::update( Void )
{
  m_ui64TotalAccessBytes += m_ui64CurrAccessBytes;
  m_ui64MaxAccessBytes = Max(m_ui64MaxAccessBytes, m_ui64CurrAccessBytes);
  m_ui64CurrAccessBytes = 0;
}


// ====================================================================================================================
// Compressed frame memory access calculator class implementation
// ====================================================================================================================

TComCompressedFrameMemoryAccessCalculator::TComCompressedFrameMemoryAccessCalculator()
{
}

TComCompressedFrameMemoryAccessCalculator::~TComCompressedFrameMemoryAccessCalculator()
{
}

Int TComCompressedFrameMemoryAccessCalculator::xCalcAccessBytes(Int iMCBlockWidth, Int iMCBlockHeight, Int iMCBlockPosX, Int iMCBlockPosY, Int iFilterTapH, Int iFilterTapV, Bool bChroma)
{
  Int iMul = bChroma ? 2 : 1;
  Int iBitDepth      = bChroma ? m_iChromaBitDepth             : m_iLumaBitDepth;
  Int iCmpUnitWidth  = bChroma ? m_cChromaParam.iUnitWidth     : m_cLumaParam.iUnitWidth;
  Int iCmpUnitHeight = bChroma ? m_cChromaParam.iUnitHeight    : m_cLumaParam.iUnitHeight;
  Int iCmpRatioNum   = bChroma ? m_cChromaParam.iCmpRatioNum   : m_cLumaParam.iCmpRatioNum;
  Int iCmpRatioDenom = bChroma ? m_cChromaParam.iCmpRatioDenom : m_cLumaParam.iCmpRatioDenom;
  // should be changed for supporting 4:4:4 format
  Int iPicWidth      = bChroma ? getPictureWidth()>>1          : getPictureWidth();

  if (iCmpUnitWidth==1 && iCmpUnitHeight==1 && iCmpRatioNum==iCmpRatioDenom)
  {
    // call uncompressed version
    return TComFrameMemoryAccessCalculator::xCalcAccessBytes(iMCBlockWidth, iMCBlockHeight, iMCBlockPosX, iMCBlockPosY, iFilterTapH, iFilterTapV, bChroma);
  }

  /* initialization */
  Int iMCBlockOverheadLeft = Max(0, (iFilterTapH>>1)-1);
  Int iMCBlockOverheadRight = (iFilterTapH>>1) + (iFilterTapH&0x1);
  Int iMCBlockOverheadTop = Max(0, (iFilterTapV>>1)-1);
  Int iMCBlockOverheadBottom = (iFilterTapV>>1) + (iFilterTapV&0x1);

  /* derive pixel positions */
  Int iLeftPos   = iMul * (iMCBlockPosX - iMCBlockOverheadLeft);
  Int iRightPos  = iMul * (iMCBlockPosX + iMCBlockWidth + iMCBlockOverheadRight);
  Int iTopPos    = (iMCBlockPosY - iMCBlockOverheadTop);
  Int iBottomPos = (iMCBlockPosY + iMCBlockHeight + iMCBlockOverheadBottom);

  /* derive compression-unit aligned pixel positions */
  Int iAlignedLeftPos    = AlignRoundDown (iLeftPos,   iMul * iCmpUnitWidth);
  Int iAlignedRightPos   = AlignRoundUp   (iRightPos,  iMul * iCmpUnitWidth);
  Int iAlignedTopPos     = AlignRoundDown (iTopPos,    iCmpUnitHeight);
  Int iAlignedBottomPos  = AlignRoundUp   (iBottomPos, iCmpUnitHeight);

  /* derive # of compression units for each direction */
  Int iNumHorizontalCmpUnit = (iAlignedRightPos  - iAlignedLeftPos) / iCmpUnitWidth;
  Int iNumVerticalCmpUnit   = (iAlignedBottomPos - iAlignedTopPos ) / iCmpUnitHeight;

  /* derive the start address of the first access unit of the first compression-unit row. */
  Int iAlignBits = 8;
  Int iAlignedAccessUnitBits = AlignRoundUp((iBitDepth * iCmpUnitWidth*iCmpUnitHeight * iCmpRatioNum + iCmpRatioNum-1) / iCmpRatioDenom, iAlignBits);
  Int iAccessUnitAlignedPicWidth = iMul * AlignRoundUp(iPicWidth, iCmpUnitWidth);
  Int64 i64StartAddress = iAlignedAccessUnitBits * (static_cast<Int64>(iAccessUnitAlignedPicWidth/iCmpUnitWidth) * (iAlignedTopPos/iCmpUnitHeight) + (iAlignedLeftPos/iCmpUnitWidth));

  /* derive virtual memory access */
  Int iMemAccessBytes = 0;
  for (Int y=0; y<iNumVerticalCmpUnit; y++)
  {
    /* derive read address */
    Int64 i64LeftAddress  = i64StartAddress;
    Int64 i64RightAddress = i64StartAddress + iAlignedAccessUnitBits*iNumHorizontalCmpUnit;

    /* derive memory aligned read address */
    Int64 i64MemAlignedLeftAddress  = AlignRoundDown (i64LeftAddress,  getAlignmentBits());
    Int64 i64MemAlignedRightAddress = AlignRoundUp   (i64RightAddress, getAlignmentBits());

    /* derive memory aligned read address */
    Int iMemAccessBytesPerLine = AlignRoundUp(static_cast<int>(i64MemAlignedRightAddress - i64MemAlignedLeftAddress), getBurstReadBits()) >> 3;

    if (m_pcCache)
    {
      static const UInt64 MAX_FRAME_SIZE_BITS = UInt64(8192)*4096*16;
      UInt64 ui64VirtualFrameByteAddress = (static_cast<UInt64>(getPictureIndex()*2+static_cast<Int>(bChroma)+1) * MAX_FRAME_SIZE_BITS) >> 3;
      for (Int n=0; n<iMemAccessBytesPerLine; n+=getAlignmentBits()/8)
      {
        UInt64 ui64ByteAddress = ui64VirtualFrameByteAddress + (i64MemAlignedLeftAddress>>3) + n;
        iMemAccessBytes += m_pcCache->xCalcCacheMisshitBytes(ui64ByteAddress);
      }
    }
    else
    {
      /* access directly to memory */
      iMemAccessBytes += iMemAccessBytesPerLine;
    }

    /* update the start address of the first access unit of the next compression-unit row. */
    i64StartAddress += iAlignedAccessUnitBits * (iAccessUnitAlignedPicWidth/iCmpUnitWidth);
  }

  addAccessBytes(iMemAccessBytes);
  return iMemAccessBytes;
}
