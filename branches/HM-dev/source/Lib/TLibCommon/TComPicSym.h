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

/** \file     TComPicSym.h
    \brief    picture symbol class (header)
*/

#ifndef __TCOMPICSYM__
#define __TCOMPICSYM__


// Include files
#include "CommonDef.h"
#include "TComSlice.h"
#include "TComDataCU.h"
class TComSampleAdaptiveOffset;
class TComPPS;

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TComTile
{
private:
  UInt      m_uiTileWidth;         // NOTE: code-tidy - rename to m_tileWidthInCtus
  UInt      m_uiTileHeight;        // NOTE: code-tidy - rename to m_tileHeightInCtus
  UInt      m_uiRightEdgePosInCU;  // NOTE: code-tidy - rename to m_rightEdgePosInCtus
  UInt      m_uiBottomEdgePosInCU; // NOTE: code-tidy - rename to m_bottomEdgePosInCtus
  UInt      m_uiFirstCUAddr;       // NOTE: code-tidy - rename to m_firstCtuRsAddr

public:
  TComTile();
  virtual ~TComTile();

  Void      setTileWidth         ( UInt i )            { m_uiTileWidth = i; }            // NOTE: code-tidy - rename to setTileWidthInCtus
  UInt      getTileWidth         () const              { return m_uiTileWidth; }         // NOTE: code-tidy - rename to getTileWidthInCtus
  Void      setTileHeight        ( UInt i )            { m_uiTileHeight = i; }           // NOTE: code-tidy - rename to setTileHeightInCtus
  UInt      getTileHeight        () const              { return m_uiTileHeight; }        // NOTE: code-tidy - rename to getTileHeightInCtus
  Void      setRightEdgePosInCU  ( UInt i )            { m_uiRightEdgePosInCU = i; }     // NOTE: code-tidy - rename to setRightEdgePosInCtus
  UInt      getRightEdgePosInCU  () const              { return m_uiRightEdgePosInCU; }  // NOTE: code-tidy - rename to getRightEdgePosInCtus
  Void      setBottomEdgePosInCU ( UInt i )            { m_uiBottomEdgePosInCU = i; }    // NOTE: code-tidy - rename to setBottomEdgePosInCtus
  UInt      getBottomEdgePosInCU () const              { return m_uiBottomEdgePosInCU; } // NOTE: code-tidy - rename to getBottomEdgePosInCtus
  Void      setFirstCUAddr       ( UInt i )            { m_uiFirstCUAddr = i; }          // NOTE: code-tidy - rename to setFirstCtuRsAddr
  UInt      getFirstCUAddr       () const              { return m_uiFirstCUAddr; }       // NOTE: code-tidy - rename to getFirstCtuRsAddr
};

/// picture symbol class
class TComPicSym
{
private:
  UInt          m_uiWidthInCU;   // NOTE: code-tidy - rename to m_frameWidthInCtus
  UInt          m_uiHeightInCU;  // NOTE: code-tidy - rename to m_frameHeightInCtus

  UInt          m_uiMaxCUWidth;
  UInt          m_uiMaxCUHeight;
  UInt          m_uiMinCUWidth;
  UInt          m_uiMinCUHeight;

  UChar         m_uhTotalDepth;       ///< max. depth
  UInt          m_uiNumPartitions;  // NOTE: code-tidy - rename to m_numPartitionsInCtu
  UInt          m_uiNumPartInWidth; // NOTE: code-tidy - rename to m_numPartInCtuWidth
  UInt          m_uiNumPartInHeight;// NOTE: code-tidy - rename to m_numPartInCtuHeight
  UInt          m_uiNumCUsInFrame;  // NOTE: code-tidy - rename to m_numCtusInFrame

  TComSlice**   m_apcTComSlice;
  UInt          m_uiNumAllocatedSlice;
  TComDataCU**  m_apcTComDataCU;        ///< array of CU data. NOTE: code-tidy - rename to m_pictureCtuArray

  Int           m_iNumColumnsMinus1;
  Int           m_iNumRowsMinus1;
  std::vector<TComTile> m_tileParameters;
  UInt*         m_ctuTsToRsAddrMap;    ///< for a given TS (Tile-Scan; coding order) address, returns the RS (Raster-Scan) address. cf CtbAddrTsToRs in specification.
  UInt*         m_puiTileIdxMap;       ///< the map of the tile index relative to LCU raster scan address
  UInt*         m_ctuRsToTsAddrMap;    ///< for a given RS (Raster-Scan) address, returns the TS (Tile-Scan; coding order) address. cf CtbAddrRsToTs in specification.

  SAOBlkParam *m_saoBlkParams;

public:
  Void               create  ( ChromaFormat chromaFormatIDC, Int iPicWidth, Int iPicHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth );
  Void               destroy ();

  TComPicSym  ();
  TComSlice*         getSlice(UInt i)                                      { return  m_apcTComSlice[i];            }
  const TComSlice*   getSlice(UInt i) const                                { return  m_apcTComSlice[i];            }
  UInt               getFrameWidthInCU()                                   { return m_uiWidthInCU;                 } // NOTE: code-tidy - rename to getFrameWidthInCtus
  UInt               getFrameHeightInCU()                                  { return m_uiHeightInCU;                } // NOTE: code-tidy - rename to getFrameHeightInCtus
  UInt               getMinCUWidth()                                       { return m_uiMinCUWidth;                }
  UInt               getMinCUHeight()                                      { return m_uiMinCUHeight;               }
  UInt               getNumberOfCUsInFrame()                               { return m_uiNumCUsInFrame;             } // NOTE: code-tidy - rename to getNumberOfCtusInFrame
  TComDataCU*&       getCU( UInt ctuRsAddr )                               { return m_apcTComDataCU[ctuRsAddr];    } // NOTE: code-tidy - rename to getCtu
  const TComDataCU*  getCU( UInt ctuRsAddr ) const                         { return m_apcTComDataCU[ctuRsAddr];    } // NOTE: code-tidy - rename to getCtu

  Void               setSlice(TComSlice* p, UInt i)                        { m_apcTComSlice[i] = p;           }
  UInt               getNumAllocatedSlice() const                          { return m_uiNumAllocatedSlice;         }
  Void               allocateNewSlice();
  Void               clearSliceBuffer();
  UInt               getNumPartition() const                               { return m_uiNumPartitions;   } // NOTE: code-tidy - rename to getNumPartitionsInCtu
  UInt               getNumPartInWidth() const                             { return m_uiNumPartInWidth;  } // NOTE: code-tidy - rename to getNumPartInCtuWidth
  UInt               getNumPartInHeight() const                            { return m_uiNumPartInHeight; } // NOTE: code-tidy - rename to getNumPartInCtuHeight
  Void               setNumColumnsMinus1( Int i )                          { m_iNumColumnsMinus1 = i;    } // NOTE: code-tidy - rename to setNumTileColumnsMinus1
  Int                getNumColumnsMinus1() const                           { return m_iNumColumnsMinus1; } // NOTE: code-tidy - rename to getNumTileColumnsMinus1
  Void               setNumRowsMinus1( Int i )                             { m_iNumRowsMinus1 = i;       } // NOTE: code-tidy - rename to setNumTileRowsMinus1
  Int                getNumRowsMinus1() const                              { return m_iNumRowsMinus1;    } // NOTE: code-tidy - rename to getNumTileRowsMinus1
  Int                getNumTiles() const                                   { return (m_iNumRowsMinus1+1)*(m_iNumColumnsMinus1+1); }
  TComTile*          getTComTile  ( UInt tileIdx )                         { return &(m_tileParameters[tileIdx]); }
  const TComTile*    getTComTile  ( UInt tileIdx ) const                   { return &(m_tileParameters[tileIdx]); }
  Void               setCtuTsToRsAddrMap( Int ctuTsAddr, Int ctuRsAddr )   { *(m_ctuTsToRsAddrMap + ctuTsAddr) = ctuRsAddr; }
  UInt               getCtuTsToRsAddrMap( Int ctuTsAddr ) const            { return *(m_ctuTsToRsAddrMap + (ctuTsAddr>=m_uiNumCUsInFrame ? m_uiNumCUsInFrame : ctuTsAddr)); }
  UInt               getTileIdxMap( Int ctuRsAddr ) const                  { return *(m_puiTileIdxMap + ctuRsAddr); }
  Void               setCtuRsToTsAddrMap( Int ctuRsAddr, Int ctuTsOrder )  { *(m_ctuRsToTsAddrMap + ctuRsAddr) = ctuTsOrder; }
  UInt               getCtuRsToTsAddrMap( Int ctuRsAddr ) const            { return *(m_ctuRsToTsAddrMap + (ctuRsAddr>=m_uiNumCUsInFrame ? m_uiNumCUsInFrame : ctuRsAddr)); }
  Void               initTiles(TComPPS *pps);

  Void               initCtuTsRsAddrMaps();
  SAOBlkParam*       getSAOBlkParam()                                      { return m_saoBlkParams;}
  const SAOBlkParam* getSAOBlkParam() const                                { return m_saoBlkParams;}
  Void               deriveLoopFilterBoundaryAvailibility(Int ctuRsAddr,
                                                          Bool& isLeftAvail, Bool& isRightAvail, Bool& isAboveAvail, Bool& isBelowAvail,
                                                          Bool& isAboveLeftAvail, Bool& isAboveRightAvail, Bool& isBelowLeftAvail, Bool& isBelowRightAvail);
protected:
  UInt               xCalculateNextCtuRSAddr( UInt uiCurrCtuRSAddr );

};// END CLASS DEFINITION TComPicSym

//! \}

#endif // __TCOMPICSYM__

