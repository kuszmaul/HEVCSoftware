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

/** \file     TEncHash.cpp
    \brief    hash encoder class
*/

#include "CommonDef.h"
#include "TComHash.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

Int TComHash::m_blockSizeToIndex[65][65];
TCRCCalculatorLight TComHash::m_crcCalculator1(24, 0x5D6DCB);
TCRCCalculatorLight TComHash::m_crcCalculator2(24, 0x864CFB);
TCRCCalculatorLight TComHash::m_crcCalculator3(16, 0x8005);
TCRCCalculatorLight TComHash::m_crcCalculator4(16, 0xA001);

TCRCCalculatorLight::TCRCCalculatorLight( UInt bits, UInt truncPoly )
{
  m_remainder = 0;
  m_bits = bits;
  m_truncPoly = truncPoly;
  m_finalResultMask = (1<<bits) - 1;

  xInitTable();
}

TCRCCalculatorLight::~TCRCCalculatorLight()
{

}

Void TCRCCalculatorLight::xInitTable()
{
  const UInt highBit = 1<<(m_bits-1);
  const UInt ByteHighBit  = 1<<(8-1);

  for ( UInt value = 0; value < 256; value++ )
  {
    UInt remainder = 0;
    for ( UChar mask = ByteHighBit; mask != 0; mask >>= 1 )
    {
      if ( value & mask )
      {
        remainder ^= highBit;
      }

      if ( remainder & highBit )
      {
        remainder <<= 1;
        remainder ^= m_truncPoly;
      }
      else
      {
        remainder <<= 1;
      }
    }

    m_table[value] = remainder;
  }
}

Void TCRCCalculatorLight::processData( UChar* pData, UInt dataLength )
{
  for ( UInt i=0; i<dataLength; i++ )
  {
    UChar index = ( m_remainder>>(m_bits-8) ) ^ pData[i];
    m_remainder <<= 8;
    m_remainder ^= m_table[index];
  }
}


TComHash::TComHash()
{
  m_pLookupTable = NULL;
}

TComHash::~TComHash()
{
  clearAll();
  if ( m_pLookupTable != NULL )
  {
    delete[] m_pLookupTable;
  }
}

Void TComHash::create()
{
  if ( m_pLookupTable != NULL )
  {
    clearAll();
    return;
  }
  Int maxAddr = 1<<(m_CRCBits+m_blockSizeBits);
  m_pLookupTable = new vector<BlockHash>*[maxAddr];
  memset( m_pLookupTable, 0, sizeof(vector<BlockHash>*) * maxAddr );
}

Void TComHash::clearAll()
{
  if ( m_pLookupTable == NULL )
  {
    return;
  }
  Int maxAddr = 1<<(m_CRCBits+m_blockSizeBits);
  for ( Int i=0; i<maxAddr; i++ )
  {
    if ( m_pLookupTable[i] != NULL )
    {
      delete m_pLookupTable[i];
      m_pLookupTable[i] = NULL;
    }
  }
}

Void TComHash::addToTable( UInt hashValue, const BlockHash& blockHash )
{
  if ( m_pLookupTable[hashValue] == NULL )
  {
    m_pLookupTable[hashValue] = new vector<BlockHash>;
    m_pLookupTable[hashValue]->push_back( blockHash );
  }
  else
  {
    m_pLookupTable[hashValue]->push_back( blockHash );
  }
}

Int TComHash::count( UInt hashValue )
{
  if ( m_pLookupTable[hashValue] == NULL )
  {
    return 0;
  }
  else
  {
    return static_cast<Int>( m_pLookupTable[hashValue]->size() );
  }
}

MapIterator TComHash::getFirstIterator( UInt hashValue )
{
  assert( count( hashValue ) > 0 );
  return m_pLookupTable[hashValue]->begin();
}

Void TComHash::addToHashMapByRow( TComPicYuv* pPicYuv, Int picWidth, Int picHeight, Int width, Int height )
{
  Int xStart = 0;
  Int xEnd = picWidth - width + 1;
  Int yEnd = picHeight - height + 1;
  Int addValue = m_blockSizeToIndex[width][height];
  assert( addValue >= 0 );
  addValue <<= m_CRCBits;
  Int crcMask = 1<<m_CRCBits;
  crcMask -= 1;

  UShort* hashValue3Row = new UShort[picHeight];
  UShort* hashValue4Row = new UShort[picHeight];
  Bool* isSameValueRow = new Bool[picHeight];
  Int length = width;
  Bool bIncludeChroma = false;
  if ( pPicYuv->getChromaFormat() == CHROMA_444 )
  {
    length = width*3;
    bIncludeChroma = true;
  }
  UChar* p = new UChar[length];
  UShort* toHash1 = new UShort[height];
  UShort* toHash2 = new UShort[height];

  for ( Int xPos = xStart; xPos < xEnd; xPos++ )
  {
    for ( Int yPos = 0; yPos < picHeight; yPos++ )
    {
      TComHash::getPixelsIn1DCharArrayByRow( pPicYuv, p, width, xPos, yPos, bIncludeChroma );
      isSameValueRow[yPos] = isRowSameValue( p, width );
      hashValue3Row[yPos] = TComHash::getCRCValue3( p, length );
      hashValue4Row[yPos] = TComHash::getCRCValue4( p, length );
    }

    for ( Int yPos = 0; yPos < yEnd; yPos++ )
    {
      Bool isHorizontalPerfectTemp = true;
      for ( Int y=0; y<height; y++ )
      {
        if ( !isSameValueRow[yPos+y] )
        {
          isHorizontalPerfectTemp = false;
          break;
        }
      }

      if ( isHorizontalPerfectTemp || TComHash::isVerticalPerfect( pPicYuv, width, height, xPos, yPos ) )
      {
        continue;
      }

      for ( Int y=0; y<height; y++ )
      {
        toHash1[y] = hashValue3Row[yPos+y];
        toHash2[y] = hashValue4Row[yPos+y];
      }

      BlockHash blockHash;
      blockHash.x = xPos;
      blockHash.y = yPos;
      UInt      hashValue1 = ( TComHash::getCRCValue1( (UChar*)(toHash1), height*sizeof(UShort) ) & crcMask ) + addValue;
      blockHash.hashValue2 =   TComHash::getCRCValue2( (UChar*)(toHash2), height*sizeof(UShort) );

      addToTable( hashValue1, blockHash );
    }
  }

  delete[] toHash1;
  delete[] toHash2;
  delete[] p;
  delete[] isSameValueRow;
  delete[] hashValue3Row;
  delete[] hashValue4Row;
}

Bool TComHash::isRowSameValue( UChar* p, Int width, Bool includeAllComponent )
{
  if ( includeAllComponent )
  {
    for ( Int i=1; i<width; i++ )
    {
      if ( p[i*3] != p[0] )
      {
        return false;
      }
      if ( p[i*3+1] != p[1] )
      {
        return false;
      }
      if ( p[i*3+2] != p[2] )
      {
        return false;
      }
    }
  }
  else
  {
    for ( Int i=1; i<width; i++ )
    {
      if ( p[i] != p[0] )
      {
        return false;
      }
    }
  }

  return true;
}


Void TComHash::getPixelsIn1DCharArrayByRow( TComPicYuv* pPicYuv, UChar* pPixelsIn1D, Int width, Int xStart, Int yStart, Bool includeAllComponent )
{
  if ( pPicYuv->getChromaFormat() != CHROMA_444 )
  {
    includeAllComponent = false;
  }

  if ( g_bitDepth[CHANNEL_TYPE_LUMA] == 8 )
  {
    Pel* pPel[3];
    Int stride[3];
    for ( Int id=0; id<3; id++ )
    {
      ComponentID compID = ComponentID( id );
      stride[id] = pPicYuv->getStride( compID );
      pPel[id] = pPicYuv->getAddr( compID );
      pPel[id] += (yStart >> pPicYuv->getComponentScaleX( compID )) * stride[id] + (xStart >> pPicYuv->getComponentScaleX( compID ));
    }

    Int index = 0;
    for ( Int j=0; j<width; j++ )
    {
      pPixelsIn1D[index++] = static_cast<UChar>( pPel[0][j] );
      if ( includeAllComponent )
      {
        pPixelsIn1D[index++] = static_cast<UChar>( pPel[1][j] );
        pPixelsIn1D[index++] = static_cast<UChar>( pPel[2][j] );
      }
    }
  }
  else
  {
    Int shift = g_bitDepth[CHANNEL_TYPE_LUMA] - 8;
    Pel* pPel[3];
    Int stride[3];
    for ( Int id=0; id<3; id++ )
    {
      ComponentID compID = ComponentID( id );
      stride[id] = pPicYuv->getStride( compID );
      pPel[id] = pPicYuv->getAddr( compID );
      pPel[id] += (yStart >> pPicYuv->getComponentScaleX( compID )) * stride[id] + (xStart >> pPicYuv->getComponentScaleX( compID ));
    }

    Int index = 0;
    for ( Int j=0; j<width; j++ )
    {
      pPixelsIn1D[index++] = static_cast<UChar>( pPel[0][j] >> shift );
      if ( includeAllComponent )
      {
        pPixelsIn1D[index++] = static_cast<UChar>( pPel[1][j] >> shift );
        pPixelsIn1D[index++] = static_cast<UChar>( pPel[2][j] >> shift );
      }
    }
  }
}

Bool TComHash::isHorizontalPerfect( TComPicYuv* pPicYuv, Int width, Int height, Int xStart, Int yStart, Bool includeAllComponent )
{
  if ( pPicYuv->getChromaFormat() != CHROMA_444 )
  {
    includeAllComponent = false;
  }

  for ( Int id=0; id < (includeAllComponent ? 3 : 1); id++ )
  {
    ComponentID compID = ComponentID( id );
    Int stride = pPicYuv->getStride( compID );
    Pel* p = pPicYuv->getAddr( compID );
    p += (yStart >> pPicYuv->getComponentScaleX( compID )) * stride + (xStart >> pPicYuv->getComponentScaleX( compID ));

    for ( Int i=0; i<height; i++ )
    {
      for ( Int j=1; j<width; j++ )
      {
        if ( p[j] != p[0] )
        {
          return false;
        }
      }
      p += stride;
    }
  }

  return true;
}

Bool TComHash::isVerticalPerfect( TComPicYuv* pPicYuv, Int width, Int height, Int xStart, Int yStart, Bool includeAllComponent )
{
  if ( pPicYuv->getChromaFormat() != CHROMA_444 )
  {
    includeAllComponent = false;
  }

  for ( Int id=0; id < (includeAllComponent ? 3 : 1); id++ )
  {
    ComponentID compID = ComponentID( id );
    Int stride = pPicYuv->getStride( compID );
    Pel* p = pPicYuv->getAddr( compID );
    p += (yStart >> pPicYuv->getComponentScaleX( compID )) * stride + (xStart >> pPicYuv->getComponentScaleX( compID ));

    for ( Int i=0; i<width; i++ )
    {
      for ( Int j=1; j<height; j++ )
      {
        if ( p[j*stride+i] != p[i] )
        {
          return false;
        }
      }
    }
  }

  return true;
}

Bool TComHash::getBlockHashValue( TComPicYuv* pPicYuv, Int width, Int height, Int xStart, Int yStart, UInt& hashValue1, UInt& hashValue2 )
{
  if ( TComHash::isHorizontalPerfect( pPicYuv, width, height, xStart, yStart ) ||
       TComHash::isVerticalPerfect( pPicYuv, width, height, xStart, yStart ) )
  {
    return false;
  }

  Int addValue = m_blockSizeToIndex[width][height];
  assert( addValue >= 0 );
  addValue <<= m_CRCBits;
  Int crcMask = 1<<m_CRCBits;
  crcMask -= 1;

  Int length = width; 
  Bool bIncludeChroma = false;
  if ( pPicYuv->getChromaFormat() == CHROMA_444 )
  {
    length = width*3;
    bIncludeChroma = true;
  }
  UChar* p = new UChar[length];
  UShort* toHash3 = new UShort[height];
  UShort* toHash4 = new UShort[height];

  for ( Int y=0; y<height; y++ )
  {
    TComHash::getPixelsIn1DCharArrayByRow( pPicYuv, p, width, xStart, yStart+y, bIncludeChroma );
    toHash3[y] = TComHash::getCRCValue3( p, length );
    toHash4[y] = TComHash::getCRCValue4( p, length );
  }

  hashValue1 = ( TComHash::getCRCValue1( (UChar*)(toHash3), height*sizeof(UShort) ) & crcMask ) + addValue;
  hashValue2 =   TComHash::getCRCValue2( (UChar*)(toHash4), height*sizeof(UShort) );

  delete[] toHash3;
  delete[] toHash4;
  delete[] p;

  return true;
}

Void TComHash::initBlockSizeToIndex()
{
  for ( Int i=0; i<65; i++ )
  {
    for ( Int j=0; j<65; j++ )
    {
      m_blockSizeToIndex[i][j] = -1;
    }
  }

  m_blockSizeToIndex[ 8][ 8] = 0;
  m_blockSizeToIndex[16][16] = 1;
  m_blockSizeToIndex[32][32] = 2;
  m_blockSizeToIndex[64][64] = 3;
}

UInt TComHash::getCRCValue1( UChar* p, Int length )
{
  m_crcCalculator1.reset();
  m_crcCalculator1.processData( p, length );
  return m_crcCalculator1.getCRC();
}

UInt TComHash::getCRCValue2( UChar* p, Int length )
{
  m_crcCalculator2.reset();
  m_crcCalculator2.processData( p, length );
  return m_crcCalculator2.getCRC();
}

UShort TComHash::getCRCValue3( UChar* p, Int length )
{
  m_crcCalculator3.reset();
  m_crcCalculator3.processData( p, length );
  return m_crcCalculator3.getCRC();
}

UShort TComHash::getCRCValue4( UChar* p, Int length )
{
  m_crcCalculator4.reset();
  m_crcCalculator4.processData( p, length );
  return m_crcCalculator4.getCRC();
}



//! \}
