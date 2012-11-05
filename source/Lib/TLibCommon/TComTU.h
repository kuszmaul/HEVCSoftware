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

#ifndef __TCOMTU__
#define __TCOMTU__

class TComTU; // forward declaration

#include "CommonDef.h"
#include "TComRectangle.h"
#include "TComChromaFormat.h"

class TComDataCU; // forward declaration

//----------------------------------------------------------------------------------------------------------------------


class TComTU
{
  public:
    typedef enum TU_SPLIT_MODE { DONT_SPLIT=0, VERTICAL_SPLIT=1, QUAD_SPLIT=2 } SPLIT_MODE;

  protected:
    ChromaFormat  mChromaFormat;
    Bool          mbProcessLastOfLevel; // if true, then if size n/2 x n/2 is invalid, the nxn block for a channel is processed only for the last block, not the first.
#if !REMOVE_NSQT
    UInt          mPartOption; // 0=NSQT disabled (Square TUs), 1=4hx1v blocks, 2=1hx4v blocks
#endif
    UInt          mCuTrDepth444;
    UInt          mTrDepth444RelCU;
    UInt          mSection;
    TU_SPLIT_MODE mSplitMode;
    TComRectangle mRect[MAX_NUM_COMPONENT];
    Bool          mCodeAll[MAX_NUM_COMPONENT];
    UInt          mOrigWidth[MAX_NUM_COMPONENT];
    UInt          mOffsets[MAX_NUM_COMPONENT];
    UInt          mAbsPartIdxCU;
    UInt          mAbsPartIdxTURelCU;
    UInt          mAbsPartIdxStep;
    TComDataCU   *mpcCU;
    UInt          mLog2TrLumaSize;
    TComTU       *mpParent;

    TComTU(const TComTU &);           // not defined - do not use
    TComTU&operator=(const TComTU &); // not defined - do not use

  public:
    TComTU(      TComDataCU *pcCU,
           const UInt        absPartIdxCU,
           const UInt        cuDepth,
           const UInt        initTrDepth444relCU);

  protected:
    TComTU(      TComTU             &parentLevel,
           const Bool                bProcessLastOfLevel,
           const TU_SPLIT_MODE       splitMode=QUAD_SPLIT);

  public:
          TComTU *Parent()       { return mpParent; }
    const TComTU *Parent() const { return mpParent; }

    UInt getCoefficientOffset(const ComponentID compID)        const { return mOffsets[compID]; }

    const TComRectangle &getRect(const ComponentID compID)     const { return mRect[compID];    }

    Bool ProcessingAllQuadrants(const ComponentID compID)      const { return mCodeAll[compID]; }
    Bool ProcessComponentSection(const ComponentID compID)     const { return mRect[compID].width != 0; }
    Bool ProcessChannelSection(const ChannelType chType)       const { return mRect[chType].width != 0; }
    UInt GetSectionNumber()                                    const { return mSection; }

    UInt getCUDepth()                                          const { return mCuTrDepth444; }

    UInt GetTransformDepthTotal()                              const { return mCuTrDepth444+mTrDepth444RelCU; }
    UInt GetTransformDepthTotalAdj(const ComponentID compID)   const { return mCuTrDepth444+GetTransformDepthRelAdj(compID); }

    UInt GetTransformDepthRel()                                const { return mTrDepth444RelCU; }
    UInt GetTransformDepthRelAdj(const ComponentID compID)     const { return mTrDepth444RelCU - (mCodeAll[compID] ? 0 : 1); }
    UInt GetTransformDepthRelAdj(const ChannelType chType)     const
    {
      assert(isLuma(chType) || (mCodeAll[COMPONENT_Cb] == mCodeAll[COMPONENT_Cr]));
      return mTrDepth444RelCU - (mCodeAll[isLuma(chType) ? COMPONENT_Y : COMPONENT_Cb] ? 0 : 1);
    }

    UInt GetAbsPartIdxCU()                                     const { return mAbsPartIdxCU; }
    UInt GetAbsPartIdxTU()                                     const { return mAbsPartIdxTURelCU + mAbsPartIdxCU; }
    UInt GetAbsPartIdxTU(const ComponentID compID)             const { return ProcessingAllQuadrants(compID) ? (mAbsPartIdxTURelCU + mAbsPartIdxCU) : (mAbsPartIdxTURelCU + mAbsPartIdxCU)&~3; }
    UInt GetAbsPartIdxNumParts()                               const { return mAbsPartIdxStep; }

    ChromaFormat GetChromaFormat()                             const { return mChromaFormat; }

    TComDataCU *getCU()                                              { return mpcCU; }
    const TComDataCU *getCU()                                  const { return mpcCU; }
    Bool IsLastSection() const { return mSection+1>=((1<<mSplitMode)); }

    UInt GetLog2LumaTrSize()                                   const { return mLog2TrLumaSize; }
    UInt GetEquivalentLog2TrSize(const ComponentID compID)     const;
    TU_SPLIT_MODE GetSplitMode()                               const { return mSplitMode; }
};



class TComTURecurse : public TComTU
{
  public:

    TComTURecurse(      TComDataCU *pcCU,
                  const UInt        absPartIdxCU,
                  const UInt        forcedDepthOfCU)
      : TComTU(pcCU, absPartIdxCU, forcedDepthOfCU, 0) { }

    TComTURecurse(      TComDataCU *pcCU,
                  const UInt        absPartIdxCU); // CU's depth is taken from CU->getDepth(idx)

    TComTURecurse(      TComTU &parentLevel,
                  const Bool    bProcessLastOfLevel,
                  const TU_SPLIT_MODE       splitMode=QUAD_SPLIT) : TComTU(parentLevel, bProcessLastOfLevel, splitMode) { }

    Bool nextSection(const TComTU &parent); // returns true if there is another section to process, and prepares internal structures, else returns false
};

//----------------------------------------------------------------------------------------------------------------------

#endif
