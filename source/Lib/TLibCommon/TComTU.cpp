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


#include "TComTU.h"
#include "TComRom.h"
#include "TComDataCU.h"
#include "TComPic.h"

//----------------------------------------------------------------------------------------------------------------------

TComTU::TComTU(TComDataCU *pcCU, const UInt absPartIdxCU, const UInt cuDepth, const UInt initTrDepth444relCU)
  : mChromaFormat(pcCU->getSlice()->getSPS()->getChromaFormatIdc()),
    mbProcessLastOfLevel(true), // does not matter. the top level is not 4 quadrants.
#if !REMOVE_NSQT
    mPartOption(0),
#endif
    mCuTrDepth444(cuDepth),
    mTrDepth444RelCU(initTrDepth444relCU),
    mSection(0),
    mSplitMode(DONT_SPLIT),
    mAbsPartIdxCU(absPartIdxCU),
    mAbsPartIdxTURelCU(0),
    mAbsPartIdxStep(pcCU->getPic()->getNumPartInCU() >> (pcCU->getDepth(absPartIdxCU)<<1)),
    mpcCU(pcCU),
    mLog2TrLumaSize(0),
    mpParent(NULL)
{
  TComSPS *pSPS=pcCU->getSlice()->getSPS();
  mLog2TrLumaSize = g_aucConvertToBit[pSPS->getMaxCUWidth() >> (mCuTrDepth444+mTrDepth444RelCU)]+2;

#if !REMOVE_NSQT
  if (pSPS->getUseNSQT())
  {
    PartSize ePartSize(pcCU->getPartitionSize(absPartIdxCU));

    if (ePartSize==SIZE_Nx2N || ePartSize==SIZE_nLx2N || ePartSize==SIZE_nRx2N)
      mPartOption=2;
    else if (ePartSize != SIZE_2Nx2N && ePartSize != SIZE_NxN)
      mPartOption=1;
  }
#endif

  const UInt baseOffset444=pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight()*absPartIdxCU;

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    const UInt csx=getComponentScaleX(ComponentID(i), mChromaFormat);
    const UInt csy=getComponentScaleY(ComponentID(i), mChromaFormat);
    mOrigWidth[i]=mRect[i].width = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getWidth( absPartIdxCU) >> csx) : 0;
    mRect[i].height              = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getHeight(absPartIdxCU) >> csy) : 0;
    mRect[i].x0=0;
    mRect[i].y0=0;
    mCodeAll[i]=true;
    mOffsets[i]=baseOffset444>>(csx+csy);
  }
}



TComTURecurse::TComTURecurse(      TComDataCU *pcCU,
                             const UInt        absPartIdxCU)
  : TComTU(pcCU, absPartIdxCU, pcCU->getDepth(absPartIdxCU), 0)
{ }



TComTU::TComTU(TComTU &parent, const Bool bProcessLastOfLevel, const TU_SPLIT_MODE splitMode)
  : mChromaFormat(parent.mChromaFormat),
    mbProcessLastOfLevel(bProcessLastOfLevel),
#if !REMOVE_NSQT
    mPartOption(parent.mPartOption),
#endif
    mCuTrDepth444(parent.mCuTrDepth444),
    mTrDepth444RelCU(parent.mTrDepth444RelCU + (splitMode==DONT_SPLIT?0:1)),
    mSection(0),
    mSplitMode(splitMode),
    mAbsPartIdxCU(parent.mAbsPartIdxCU),
    mAbsPartIdxTURelCU(parent.mAbsPartIdxTURelCU),
    mAbsPartIdxStep(parent.mAbsPartIdxStep >> splitMode),
    mpcCU(parent.mpcCU),
    mLog2TrLumaSize(parent.mLog2TrLumaSize - (splitMode==DONT_SPLIT?0:1)),
    mpParent(&parent)
{
  if (mSplitMode==DONT_SPLIT)
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i] = (parent.mRect[i]);
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 1 TU at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
    return;
  }
  else if (mSplitMode==VERTICAL_SPLIT)
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i].x0 = (parent.mRect[i].x0);
      mRect[i].y0 = (parent.mRect[i].y0);
      mRect[i].width  = (parent.mRect[i].width);
      mRect[i].height = (parent.mRect[i].height)>>1;
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 2 TUs at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
    return;
  }

#if !REMOVE_NSQT
  const UInt initialPartOption=mPartOption;
  Bool bFirstChannelChangedToNSQT=false;
#endif
  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    mRect[i].width = (parent.mRect[i].width >> 1);
    mRect[i].height= (parent.mRect[i].height>> 1);
    mRect[i].x0=parent.mRect[i].x0;
    mRect[i].y0=parent.mRect[i].y0;
    mOffsets[i]=parent.mOffsets[i];

#if !REMOVE_NSQT
    if (initialPartOption && mRect[i].width < MAX_TU_SIZE && (i==0 || bFirstChannelChangedToNSQT))
    {
      bFirstChannelChangedToNSQT=true;
      if (initialPartOption==2)
      {
        mRect[i].width >>= 1;
        mRect[i].height<<= 1;
      }
      else
      {
        mRect[i].width <<= 1;
        mRect[i].height>>= 1;
      }
      mPartOption=0;
    }
#endif

    if ((mRect[i].width < MIN_TU_SIZE || mRect[i].height < MIN_TU_SIZE) && mRect[i].width!=0)
    {
      const UInt numPels=mRect[i].width * mRect[i].height;
      if (numPels < (MIN_TU_SIZE*MIN_TU_SIZE))
      {
        // this level doesn't have enough pixels to have 4 blocks of any relative dimension
        mRect[i].width = parent.mRect[i].width;
        mRect[i].height= parent.mRect[i].height;
        mCodeAll[i]=false; // go up a level, so only process one entry of a quadrant
      }
      else if (mRect[i].width < mRect[i].height)
      {
        mRect[i].width=MIN_TU_SIZE;
        mRect[i].height=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
      else
      {
        mRect[i].height=MIN_TU_SIZE;
        mRect[i].width=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
    }
    else
    {
      mCodeAll[i]=true;
    }

    if (allFormatsUse420TUTreeStructure())
    {
      if (i && mChromaFormat==CHROMA_422 && mRect[i].width==(MIN_TU_SIZE*2) && mRect[i].height==MIN_TU_SIZE)
      {
        mRect[i].width=MIN_TU_SIZE;
        mRect[i].height=MIN_TU_SIZE*2;
      }
      else if (i && mChromaFormat==CHROMA_444 && (mRect[i].width==MIN_TU_SIZE || mRect[i].height==MIN_TU_SIZE))
      {
        mCodeAll[i]=mRect[i].width!=mRect[i].height;
        mRect[i].width=MIN_TU_SIZE*2;
        mRect[i].height=MIN_TU_SIZE*2;
      }
    }

    mOrigWidth[i]=mRect[i].width;
    if (!mCodeAll[i] && mbProcessLastOfLevel) mRect[i].width=0;
  }
}

Bool TComTURecurse::nextSection(const TComTU &parent)
{
  if (mSplitMode==DONT_SPLIT)
  {
    mSection++;
    return false;
  }
  else
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mOffsets[i]+=mRect[i].width*mRect[i].height;
      if (mbProcessLastOfLevel) mRect[i].width=mOrigWidth[i];
      mRect[i].x0+=mRect[i].width;
      const TComRectangle &parentRect=parent.getRect(ComponentID(i));
      if (mRect[i].x0 >= parentRect.x0+parentRect.width)
      {
        mRect[i].x0=parentRect.x0;
        mRect[i].y0+=mRect[i].height;
      }
      if (!mCodeAll[i])
      {
        if (!mbProcessLastOfLevel || mSection!=2) mRect[i].width=0;
      }
    }
    assert(mRect[COMPONENT_Cb].x0==mRect[COMPONENT_Cr].x0);
    assert(mRect[COMPONENT_Cb].y0==mRect[COMPONENT_Cr].y0);
    assert(mRect[COMPONENT_Cb].width==mRect[COMPONENT_Cr].width);
    assert(mRect[COMPONENT_Cb].height==mRect[COMPONENT_Cr].height);

    mAbsPartIdxTURelCU+=mAbsPartIdxStep;
    mSection++;
    return mSection< (1<<mSplitMode);
  }
}



UInt TComTU::GetEquivalentLog2TrSize(const ComponentID compID)     const
{
  const TComRectangle &rect = getRect(compID);
  const UInt height=rect.height;
#if REMOVE_NSQT
  return g_aucConvertToBit[ height ] + 2;
#else
  const UInt width=rect.width;
  const UInt csx=getComponentScaleX(compID, mChromaFormat);
  const UInt csy=getComponentScaleY(compID, mChromaFormat);

       if ((width<<csx) == (height<<csy))    return g_aucConvertToBit[ height ] + 2;
  else if ((width<<csx) <  (height<<csy))    return g_aucConvertToBit[ height ] + 2 - 1;
  else                                       return g_aucConvertToBit[ height ] + 2 + 1;
#endif
}
