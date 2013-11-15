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

/** \file     TComSampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "TComSampleAdaptiveOffset.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup TLibCommon
//! \{
#if HM_CLEANUP_SAO
UInt g_saoMaxOffsetQVal[MAX_NUM_COMPONENT]; 

SAOOffset::SAOOffset()
{ 
  reset();
}

SAOOffset::~SAOOffset()
{

}

Void SAOOffset::reset()
{
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(Int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(Int)* MAX_NUM_SAO_CLASSES);

  return *this;
}


SAOBlkParam::SAOBlkParam()
{
  reset();
}

SAOBlkParam::~SAOBlkParam()
{

}

Void SAOBlkParam::reset()
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;

}

TComSampleAdaptiveOffset::TComSampleAdaptiveOffset()
{
  m_tempPicYuv = NULL;
  for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_offsetClipTable[compIdx] = NULL;
  }
  m_signTable = NULL; 

  
  m_lineBufWidth = 0;
  m_signLineBuf1 = NULL;
  m_signLineBuf2 = NULL;
}


TComSampleAdaptiveOffset::~TComSampleAdaptiveOffset()
{
  destroy();
  
  if (m_signLineBuf1) delete[] m_signLineBuf1; m_signLineBuf1 = NULL;
  if (m_signLineBuf2) delete[] m_signLineBuf2; m_signLineBuf2 = NULL;
}

Void TComSampleAdaptiveOffset::create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth )
{
  destroy();

  m_picWidth        = picWidth;
  m_picHeight       = picHeight;
  m_chromaFormatIDC = format;
  m_maxCUWidth      = maxCUWidth; 
  m_maxCUHeight     = maxCUHeight;

  m_numCTUInWidth   = (m_picWidth/m_maxCUWidth) + ((m_picWidth % m_maxCUWidth)?1:0);
  m_numCTUInHeight  = (m_picHeight/m_maxCUHeight) + ((m_picHeight % m_maxCUHeight)?1:0);
  m_numCTUsPic      = m_numCTUInHeight*m_numCTUInWidth;

  //temporary picture buffer
  if ( !m_tempPicYuv )
  {
    m_tempPicYuv = new TComPicYuv;
    m_tempPicYuv->create( m_picWidth, m_picHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, maxCUDepth );
  }

  //bit-depth related
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    Int bitDepthSample = g_bitDepth[toChannelType(ComponentID(compIdx))];
    m_offsetStepLog2  [compIdx] = max(bitDepthSample - MAX_SAO_TRUNCATED_BITDEPTH, 0);
    g_saoMaxOffsetQVal[compIdx] = (1<<(min(bitDepthSample,MAX_SAO_TRUNCATED_BITDEPTH)-5))-1; //Table 9-32, inclusive
  }

  //look-up table for clipping
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    Int bitDepthSample = g_bitDepth[toChannelType(ComponentID(compIdx))]; //exclusive
    Int maxSampleValue = (1<< bitDepthSample); //exclusive
    Int maxOffsetValue = (g_saoMaxOffsetQVal[compIdx] << m_offsetStepLog2[compIdx]); 

    m_offsetClipTable[compIdx] = new Int[(maxSampleValue + maxOffsetValue -1)+ (maxOffsetValue)+1 ]; //positive & negative range plus 0
    m_offsetClip[compIdx] = &(m_offsetClipTable[compIdx][maxOffsetValue]);

    //assign clipped values 
    Int* offsetClipPtr = m_offsetClip[compIdx];
    for(Int k=0; k< maxSampleValue; k++)
    {
      *(offsetClipPtr + k) = k;
    }
    for(Int k=0; k< maxOffsetValue; k++ )
    {
      *(offsetClipPtr + maxSampleValue+ k) = maxSampleValue-1;
      *(offsetClipPtr -k -1 )              = 0;
    }
    if(compIdx == COMPONENT_Y) //g_bitDepthY is always larger than or equal to g_bitDepthC
    {
      m_signTable = new Short[ 2*(maxSampleValue-1) + 1 ];
      m_sign = &(m_signTable[maxSampleValue-1]);

      m_sign[0] = 0;
      for(Int k=1; k< maxSampleValue; k++)
      {
        m_sign[k] = 1;
        m_sign[-k]= -1;
      }
    }
  }  
}

Void TComSampleAdaptiveOffset::destroy()
{
  if ( m_tempPicYuv )
  {
    m_tempPicYuv->destroy();
    delete m_tempPicYuv;
    m_tempPicYuv = NULL;
  }

  for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    if(m_offsetClipTable[compIdx])
    {
      delete[] m_offsetClipTable[compIdx]; m_offsetClipTable[compIdx] = NULL;
    }
  }
  if( m_signTable )
  {
    delete[] m_signTable; m_signTable = NULL;
  }
}

Void TComSampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets)
{
  Int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(Int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(Int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO
  {
    for(Int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    assert(dstOffsets[SAO_CLASS_EO_PLAIN] == 0); //keep EO plain offset as zero
  }

}

Int TComSampleAdaptiveOffset::getMergeList(TComPic* pic, Int ctu, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  Int ctuX = ctu % m_numCTUInWidth;
  Int ctuY = ctu / m_numCTUInWidth;
  Int mergedCTUPos;
  Int numValidMergeCandidates = 0;

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      {
        if(ctuY > 0)
        {
          mergedCTUPos = ctu- m_numCTUInWidth;
          if( pic->getSAOMergeAvailability(ctu, mergedCTUPos) )
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    case SAO_MERGE_LEFT:
      {
        if(ctuX > 0)
        {
          mergedCTUPos = ctu- 1;
          if( pic->getSAOMergeAvailability(ctu, mergedCTUPos) )
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    default:
      {
        printf("not a supported merge type");
        assert(0);
        exit(-1);
      }
    }

    mergeList[mergeType]=mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


Void TComSampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        assert(mergeTarget != NULL);

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        printf("Not a supported mode");
        assert(0);
        exit(-1);
      }
    }
  }
}

Void TComSampleAdaptiveOffset::reconstructBlkSAOParams(TComPic* pic, SAOBlkParam* saoBlkParams)
{
  m_picSAOEnabled[COMPONENT_Y] = m_picSAOEnabled[COMPONENT_Cb] = m_picSAOEnabled[COMPONENT_Cr] = false;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int ctu=0; ctu< m_numCTUsPic; ctu++)
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(pic, ctu, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctu], mergeList);

    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if(saoBlkParams[ctu][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}


Void TComSampleAdaptiveOffset::offsetBlock(ComponentID compIdx, Int typeIdx, Int* offset  
                                          , Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                                          , Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail)
{
  if(m_lineBufWidth != m_maxCUWidth)
  {
    m_lineBufWidth = m_maxCUWidth;
    
    if (m_signLineBuf1) delete[] m_signLineBuf1; m_signLineBuf1 = NULL;
    m_signLineBuf1 = new Char[m_lineBufWidth+1];
    
    if (m_signLineBuf2) delete[] m_signLineBuf2; m_signLineBuf2 = NULL;
    m_signLineBuf2 = new Char[m_lineBufWidth+1];
  }

  Int* offsetClip = m_offsetClip[compIdx];

  Int x,y, startX, startY, endX, endY, edgeType;
  Int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  Char signLeft, signRight, signDown;

  Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (Char)m_sign[srcLine[startX] - srcLine[startX-1]];
        for (x=startX; x< endX; x++)
        {
          signRight = (Char)m_sign[srcLine[x] - srcLine[x+1]]; 
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90:
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1;

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (Char)m_sign[srcLine[x] - srcLineAbove[x]];
      }

      Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=0; x< width; x++)
        {
          signDown  = (Char)m_sign[srcLine[x] - srcLineBelow[x]]; 
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;

          resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];
        }
        srcLine += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_135:
    {
      offset += 2;
      Char *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = m_signLineBuf1;
      signDownLine= m_signLineBuf2;

      startX = isLeftAvail ? 0 : 1 ;
      endX   = isRightAvail ? width : (width-1);

      //prepare 2nd line's upper sign
      Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (Char)m_sign[srcLineBelow[x] - srcLine[x- 1]];
      }

      //1st line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType  =  m_sign[srcLine[x] - srcLineAbove[x- 1]] - signUpLine[x+1];
        resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];
      }
      srcLine  += srcStride;
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=startX; x<endX; x++)
        {
          signDown =  (Char)m_sign[srcLine[x] - srcLineBelow[x+ 1]] ;
          edgeType =  signDown + signUpLine[x];
          resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];

          signDownLine[x+1] = -signDown; 
        }
        signDownLine[startX] = (Char)m_sign[srcLineBelow[startX] - srcLine[startX-1]];

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType =  m_sign[srcLine[x] - srcLineBelow[x+ 1]] + signUpLine[x];
        resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];

      }
    }
    break;
  case SAO_TYPE_EO_45:
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1+1;

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (Char)m_sign[srcLineBelow[x] - srcLine[x+1]];
      }


      //first line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType = m_sign[srcLine[x] - srcLineAbove[x+1]] -signUpLine[x-1];
        resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (Char)m_sign[srcLine[x] - srcLineBelow[x-1]] ;
          edgeType =  signDown + signUpLine[x];
          resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];
          signUpLine[x-1] = -signDown; 
        }
        signUpLine[endX-1] = (Char)m_sign[srcLineBelow[endX-1] - srcLine[endX]];
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType = m_sign[srcLine[x] - srcLineBelow[x-1]] + signUpLine[x];
        resLine[x] = offsetClip[srcLine[x] + offset[edgeType]];

      }
    }
    break;
  case SAO_TYPE_BO:
    {
      Int shiftBits = g_bitDepth[toChannelType(compIdx)] - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = offsetClip[ srcLine[x] + offset[srcLine[x] >> shiftBits] ];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      printf("Not a supported SAO types\n");
      assert(0);
      exit(-1);
    }
  }
}

Void TComSampleAdaptiveOffset::offsetCTU(Int ctu, TComPicYuv* srcYuv, TComPicYuv* resYuv, SAOBlkParam& saoblkParam, TComPic* pPic)
{
  Bool isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail;

  if( 
    (saoblkParam[COMPONENT_Y ].modeIdc == SAO_MODE_OFF) &&
    (saoblkParam[COMPONENT_Cb].modeIdc == SAO_MODE_OFF) &&
    (saoblkParam[COMPONENT_Cr].modeIdc == SAO_MODE_OFF)
    )
  {
    return;
  }

  //block boundary availability
  pPic->getPicSym()->deriveLoopFilterBoundaryAvailibility(ctu, isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

  Int yPos   = (ctu / m_numCTUInWidth)*m_maxCUHeight;
  Int xPos   = (ctu % m_numCTUInWidth)*m_maxCUWidth;
  Int height = (yPos + m_maxCUHeight > m_picHeight)?(m_picHeight- yPos):m_maxCUHeight;
  Int width  = (xPos + m_maxCUWidth  > m_picWidth )?(m_picWidth - xPos):m_maxCUWidth;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& ctbOffset = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      const UInt componentScaleX = getComponentScaleX(component, pPic->getChromaFormat());
      const UInt componentScaleY = getComponentScaleY(component, pPic->getChromaFormat());

      Int  blkWidth   = (width  >> componentScaleX);
      Int  blkHeight  = (height >> componentScaleY);
      Int  blkXPos    = (xPos   >> componentScaleX);
      Int  blkYPos    = (yPos   >> componentScaleY);

      Int  srcStride  = srcYuv->getStride(component);
      Pel* srcBlk     = srcYuv->getAddr(component) + (yPos >> componentScaleY)*srcStride + (xPos >> componentScaleX);

      Int  resStride  = resYuv->getStride(component);
      Pel* resBlk     = resYuv->getAddr(component) + blkYPos*resStride + blkXPos;

      offsetBlock( component, ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, blkWidth, blkHeight
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  );
    }
  } //compIdx

}


Void TComSampleAdaptiveOffset::SAOProcess(TComPic* pDecPic)
{
  if(!m_picSAOEnabled[COMPONENT_Y] && !m_picSAOEnabled[COMPONENT_Cb] && !m_picSAOEnabled[COMPONENT_Cr])
  {
    return;
  }
  TComPicYuv* resYuv = pDecPic->getPicYuvRec();
  TComPicYuv* srcYuv = m_tempPicYuv;
  resYuv->copyToPic(srcYuv);
  for(Int ctu= 0; ctu < m_numCTUsPic; ctu++)
  {
    offsetCTU(ctu, srcYuv, resYuv, (pDecPic->getPicSym()->getSAOBlkParam())[ctu], pDecPic);
  } //ctu
}

#else

SAOParam::~SAOParam()
{
  for (Int i = 0 ; i<MAX_NUM_COMPONENT; i++)
  {
    if (psSaoPart[i])
    {
      delete [] psSaoPart[i];
    }
  }
}

// ====================================================================================================================
// Tables
// ====================================================================================================================

TComSampleAdaptiveOffset::TComSampleAdaptiveOffset()
{
  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_apClipTable[ch] = NULL;
    m_apClipTableBase[ch] = NULL;
    m_aiOffsetBo[ch] = NULL;
    m_aTableBo[ch] = NULL;
  }
  m_iUpBuff1 = NULL;
  m_iUpBuff2 = NULL;
  m_iUpBufft = NULL;
  ipSwap = NULL;

  m_pTmpU1 = NULL;
  m_pTmpU2 = NULL;
  m_pTmpL1 = NULL;
  m_pTmpL2 = NULL;
}

TComSampleAdaptiveOffset::~TComSampleAdaptiveOffset()
{

}

const Int TComSampleAdaptiveOffset::m_aiNumCulPartsLevel[SAO_MAX_DEPTH + 1] =
{
  1,   //level 0
  5,   //level 1
  21,  //level 2
  85,  //level 3
  341, //level 4
};

const UInt TComSampleAdaptiveOffset::m_auiEoTable[SAO_EO_TABLE_SIZE] = //NOTE: RExt - This table appears to be larger than needed.
{
  1, //0
  2, //1
  0, //2
  3, //3
  4, //4
  0, //5
  0, //6
  0, //7
  0
};

const Int TComSampleAdaptiveOffset::m_iNumClass[MAX_NUM_SAO_TYPE] =
{
  SAO_EO_LEN,
  SAO_EO_LEN,
  SAO_EO_LEN,
  SAO_EO_LEN,
  SAO_BO_LEN
};

const UInt TComSampleAdaptiveOffset::m_uiMaxDepth = SAO_MAX_DEPTH;


/** convert Level Row Col to Idx
 * \param   level,  row,  col
 */
Int  TComSampleAdaptiveOffset::convertLevelRowCol2Idx(Int level, Int row, Int col)
{
  Int idx;
  if (level == 0)
  {
    idx = 0;
  }
  else if (level == 1)
  {
    idx = 1 + row*2 + col;
  }
  else if (level == 2)
  {
    idx = 5 + row*4 + col;
  }
  else if (level == 3)
  {
    idx = 21 + row*8 + col;
  }
  else // (level == 4)
  {
    idx = 85 + row*16 + col;
  }
  return idx;
}

/** create SampleAdaptiveOffset memory.
 * \param
 */
Void TComSampleAdaptiveOffset::create( UInt uiSourceWidth, UInt uiSourceHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight )
{
  m_iPicWidth  = uiSourceWidth;
  m_iPicHeight = uiSourceHeight;

  m_uiMaxCUWidth  = uiMaxCUWidth;
  m_uiMaxCUHeight = uiMaxCUHeight;

  m_iNumCuInWidth  = m_iPicWidth / m_uiMaxCUWidth;
  m_iNumCuInWidth += ( m_iPicWidth % m_uiMaxCUWidth ) ? 1 : 0;

  m_iNumCuInHeight  = m_iPicHeight / m_uiMaxCUHeight;
  m_iNumCuInHeight += ( m_iPicHeight % m_uiMaxCUHeight ) ? 1 : 0;

  Int iMaxSplitLevelHeight = (Int)(logf((Float)m_iNumCuInHeight)/logf(2.0));
  Int iMaxSplitLevelWidth  = (Int)(logf((Float)m_iNumCuInWidth )/logf(2.0));

  m_uiMaxSplitLevel = (iMaxSplitLevelHeight < iMaxSplitLevelWidth)?(iMaxSplitLevelHeight):(iMaxSplitLevelWidth);
  m_uiMaxSplitLevel = (m_uiMaxSplitLevel< m_uiMaxDepth)?(m_uiMaxSplitLevel):(m_uiMaxDepth);
  /* various structures are overloaded to store per component data.
   * m_iNumTotalParts must allow for sufficient storage in any allocated arrays */
  m_iNumTotalParts  = max(Int(MAX_NUM_COMPONENT),m_aiNumCulPartsLevel[m_uiMaxSplitLevel]);


  m_iUpBuff1 = new Int[m_iPicWidth+2];
  m_iUpBuff2 = new Int[m_iPicWidth+2];
  m_iUpBufft = new Int[m_iPicWidth+2];

  m_iUpBuff1++;
  m_iUpBuff2++;
  m_iUpBufft++;

  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    UInt uiPixelRange = 1 << g_bitDepth[ch];
    UInt uiBoRangeShift = g_bitDepth[ch] - SAO_BO_BITS;

    m_aTableBo[ch] = new Pel [uiPixelRange];
    for (Int k2=0; k2<uiPixelRange; k2++)
    {
      m_aTableBo[ch][k2] = 1 + (k2>>uiBoRangeShift);
    }

    UInt uiMax  = uiPixelRange - 1;
    UInt uiMin  = 0;

    Int iCRangeExt = uiMax>>1;

    m_apClipTableBase[ch] = new Pel[uiMax+2*iCRangeExt];
    m_aiOffsetBo[ch]      = new Int[uiMax+2*iCRangeExt];

    for(Pel i=0;i<(uiMin+iCRangeExt);i++)
    {
      m_apClipTableBase[ch][i] = uiMin;
    }

    for(Pel i=uiMin+iCRangeExt;i<(uiMax+  iCRangeExt);i++)
    {
      m_apClipTableBase[ch][i] = i-iCRangeExt;
    }

    for(Pel i=uiMax+iCRangeExt;i<(uiMax+2*iCRangeExt);i++)
    {
      m_apClipTableBase[ch][i] = uiMax;
    }

    m_apClipTable[ch] = &(m_apClipTableBase[ch][iCRangeExt]);
  }

  m_pTmpL1 = new Pel [m_uiMaxCUHeight+1];
  m_pTmpL2 = new Pel [m_uiMaxCUHeight+1];
  m_pTmpU1 = new Pel [m_iPicWidth];
  m_pTmpU2 = new Pel [m_iPicWidth];
}

/** destroy SampleAdaptiveOffset memory.
 * \param
 */
Void TComSampleAdaptiveOffset::destroy()
{
  for(UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    if (m_apClipTableBase[ch])
    {
      delete [] m_apClipTableBase[ch]; m_apClipTableBase[ch] = NULL;
    }
    if (m_aiOffsetBo[ch])
    {
      delete [] m_aiOffsetBo[ch]; m_aiOffsetBo[ch] = NULL;
    }
    if (m_aTableBo[ch])
    {
      delete[] m_aTableBo[ch]; m_aTableBo[ch] = NULL;
    }
  }

  if (m_iUpBuff1)
  {
    m_iUpBuff1--;
    delete [] m_iUpBuff1; m_iUpBuff1 = NULL;
  }
  if (m_iUpBuff2)
  {
    m_iUpBuff2--;
    delete [] m_iUpBuff2; m_iUpBuff2 = NULL;
  }
  if (m_iUpBufft)
  {
    m_iUpBufft--;
    delete [] m_iUpBufft; m_iUpBufft = NULL;
  }
  if (m_pTmpL1)
  {
    delete [] m_pTmpL1; m_pTmpL1 = NULL;
  }
  if (m_pTmpL2)
  {
    delete [] m_pTmpL2; m_pTmpL2 = NULL;
  }
  if (m_pTmpU1)
  {
    delete [] m_pTmpU1; m_pTmpU1 = NULL;
  }
  if (m_pTmpU2)
  {
    delete [] m_pTmpU2; m_pTmpU2 = NULL;
  }
}

/** allocate memory for SAO parameters
 * \param    *pcSaoParam
 */
Void TComSampleAdaptiveOffset::allocSaoParam(SAOParam *pcSaoParam)
{
  pcSaoParam->iMaxSplitLevel = m_uiMaxSplitLevel;
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    const ComponentID compId=ComponentID(comp);
    pcSaoParam->psSaoPart[compId] = new SAOQTPart[ m_aiNumCulPartsLevel[pcSaoParam->iMaxSplitLevel] ];
    initSAOParam(pcSaoParam, 0, 0, 0, -1, 0, m_iNumCuInWidth-1,  0, m_iNumCuInHeight-1,compId);
  }
  pcSaoParam->numCuInWidth  = m_iNumCuInWidth;
  pcSaoParam->numCuInHeight = m_iNumCuInHeight;
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    pcSaoParam->saoLcuParam[comp] = new SaoLcuParam [m_iNumCuInHeight*m_iNumCuInWidth];
  }
}

/** initialize SAO parameters
 * \param    *pcSaoParam,  iPartLevel,  iPartRow,  iPartCol,  iParentPartIdx,  StartCUX,  EndCUX,  StartCUY,  EndCUY,  iYCbCr
 */
Void TComSampleAdaptiveOffset::initSAOParam(SAOParam *pcSaoParam, Int iPartLevel, Int iPartRow, Int iPartCol, Int iParentPartIdx, Int StartCUX, Int EndCUX, Int StartCUY, Int EndCUY, ComponentID ch)
{
  Int j;
  Int iPartIdx = convertLevelRowCol2Idx(iPartLevel, iPartRow, iPartCol);

  SAOQTPart* pSaoPart;

  pSaoPart = &(pcSaoParam->psSaoPart[ch][iPartIdx]);

  pSaoPart->PartIdx   = iPartIdx;
  pSaoPart->PartLevel = iPartLevel;
  pSaoPart->PartRow   = iPartRow;
  pSaoPart->PartCol   = iPartCol;

  pSaoPart->StartCUX  = StartCUX;
  pSaoPart->EndCUX    = EndCUX;
  pSaoPart->StartCUY  = StartCUY;
  pSaoPart->EndCUY    = EndCUY;

  pSaoPart->UpPartIdx = iParentPartIdx;
  pSaoPart->iBestType   = -1;
  pSaoPart->iLength     =  0;

  pSaoPart->subTypeIdx = 0;

  for (j=0;j<MAX_NUM_SAO_OFFSETS;j++)
  {
    pSaoPart->iOffset[j] = 0;
  }

  if(pSaoPart->PartLevel != m_uiMaxSplitLevel)
  {
    Int DownLevel    = (iPartLevel+1 );
    Int DownRowStart = (iPartRow << 1);
    Int DownColStart = (iPartCol << 1);

    Int iDownRowIdx, iDownColIdx;
    Int NumCUWidth,  NumCUHeight;
    Int NumCULeft;
    Int NumCUTop;

    Int DownStartCUX, DownStartCUY;
    Int DownEndCUX, DownEndCUY;

    NumCUWidth  = EndCUX - StartCUX +1;
    NumCUHeight = EndCUY - StartCUY +1;
    NumCULeft   = (NumCUWidth  >> 1);
    NumCUTop    = (NumCUHeight >> 1);

    DownStartCUX= StartCUX;
    DownEndCUX  = DownStartCUX + NumCULeft - 1;
    DownStartCUY= StartCUY;
    DownEndCUY  = DownStartCUY + NumCUTop  - 1;
    iDownRowIdx = DownRowStart + 0;
    iDownColIdx = DownColStart + 0;

    pSaoPart->DownPartsIdx[0]= convertLevelRowCol2Idx(DownLevel, iDownRowIdx, iDownColIdx);

    initSAOParam(pcSaoParam, DownLevel, iDownRowIdx, iDownColIdx, iPartIdx, DownStartCUX, DownEndCUX, DownStartCUY, DownEndCUY, ch);

    DownStartCUX = StartCUX + NumCULeft;
    DownEndCUX   = EndCUX;
    DownStartCUY = StartCUY;
    DownEndCUY   = DownStartCUY + NumCUTop -1;
    iDownRowIdx  = DownRowStart + 0;
    iDownColIdx  = DownColStart + 1;

    pSaoPart->DownPartsIdx[1] = convertLevelRowCol2Idx(DownLevel, iDownRowIdx, iDownColIdx);

    initSAOParam(pcSaoParam, DownLevel, iDownRowIdx, iDownColIdx, iPartIdx,  DownStartCUX, DownEndCUX, DownStartCUY, DownEndCUY, ch);

    DownStartCUX = StartCUX;
    DownEndCUX   = DownStartCUX + NumCULeft -1;
    DownStartCUY = StartCUY + NumCUTop;
    DownEndCUY   = EndCUY;
    iDownRowIdx  = DownRowStart + 1;
    iDownColIdx  = DownColStart + 0;

    pSaoPart->DownPartsIdx[2] = convertLevelRowCol2Idx(DownLevel, iDownRowIdx, iDownColIdx);

    initSAOParam(pcSaoParam, DownLevel, iDownRowIdx, iDownColIdx, iPartIdx, DownStartCUX, DownEndCUX, DownStartCUY, DownEndCUY, ch);

    DownStartCUX = StartCUX+ NumCULeft;
    DownEndCUX   = EndCUX;
    DownStartCUY = StartCUY + NumCUTop;
    DownEndCUY   = EndCUY;
    iDownRowIdx  = DownRowStart + 1;
    iDownColIdx  = DownColStart + 1;

    pSaoPart->DownPartsIdx[3] = convertLevelRowCol2Idx(DownLevel, iDownRowIdx, iDownColIdx);

    initSAOParam(pcSaoParam, DownLevel, iDownRowIdx, iDownColIdx, iPartIdx,DownStartCUX, DownEndCUX, DownStartCUY, DownEndCUY, ch);
  }
  else
  {
    pSaoPart->DownPartsIdx[0]=pSaoPart->DownPartsIdx[1]= pSaoPart->DownPartsIdx[2]= pSaoPart->DownPartsIdx[3]= -1;
  }
}

/** free memory of SAO parameters
 * \param   pcSaoParam
 */
Void TComSampleAdaptiveOffset::freeSaoParam(SAOParam *pcSaoParam)
{
  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    delete [] pcSaoParam->psSaoPart[component];
    pcSaoParam->psSaoPart[component] = 0;
    if( pcSaoParam->saoLcuParam[component])
    {
      delete [] pcSaoParam->saoLcuParam[component]; pcSaoParam->saoLcuParam[component] = NULL;
    }
  }
}

/** reset SAO parameters
 * \param   pcSaoParam
 */
Void TComSampleAdaptiveOffset::resetSAOParam(SAOParam *pcSaoParam)
{

  for(Int c=0; c<MAX_NUM_COMPONENT; c++)
  {
    if (c<MAX_NUM_CHANNEL_TYPE)
    {
      pcSaoParam->bSaoFlag[c] = 0;
    }

    for(Int i=0; i< m_aiNumCulPartsLevel[m_uiMaxSplitLevel]; i++)
    {
      pcSaoParam->psSaoPart[c][i].iBestType     = -1;
      pcSaoParam->psSaoPart[c][i].iLength       =  0;
      pcSaoParam->psSaoPart[c][i].bSplit        = false;
      pcSaoParam->psSaoPart[c][i].bProcessed    = false;
      pcSaoParam->psSaoPart[c][i].dMinCost      = MAX_DOUBLE;
      pcSaoParam->psSaoPart[c][i].iMinDist      = MAX_INT;
      pcSaoParam->psSaoPart[c][i].iMinRate      = MAX_INT;
      pcSaoParam->psSaoPart[c][i].subTypeIdx    = 0;
      for (Int j=0;j<MAX_NUM_SAO_OFFSETS;j++)
      {
        pcSaoParam->psSaoPart[c][i].iOffset[j] = 0;
        pcSaoParam->psSaoPart[c][i].iOffset[j] = 0;
        pcSaoParam->psSaoPart[c][i].iOffset[j] = 0;
      }
    }
    pcSaoParam->oneUnitFlag[c]   = 0;
    resetLcuPart(pcSaoParam->saoLcuParam[c]);
  }
}

/** get the sign of input variable
 * \param   x
 */
inline Int xSign(Int x)
{
  static const UInt shift = (sizeof(Int) * 8) - 1;
  return ((x >> shift) | ((Int)( (((UInt) -x)) >> shift)));
}

/** initialize variables for SAO process
 * \param  pcPic picture data pointer
 */
Void TComSampleAdaptiveOffset::createPicSaoInfo(TComPic* pcPic)
{
  m_pcPic   = pcPic;
  m_bUseNIF = ( pcPic->getIndependentSliceBoundaryForNDBFilter() || pcPic->getIndependentTileBoundaryForNDBFilter() );
  if(m_bUseNIF)
  {
    m_pcYuvTmp = pcPic->getYuvPicBufferForIndependentBoundaryProcessing();
  }
}

Void TComSampleAdaptiveOffset::destroyPicSaoInfo()
{

}

/** sample adaptive offset process for one LCU
 * \param   iAddr, iSaoType, iYCbCr
 */
Void TComSampleAdaptiveOffset::processSaoCu(Int iAddr, Int iSaoType, ComponentID ch)
{
  if(!m_bUseNIF)
  {
    processSaoCuOrg( iAddr, iSaoType, ch);
  }
  else
  {
    Int  stride   = m_pcPic->getStride(ch);
    Pel* pPicRest = getPicYuvAddr(m_pcPic->getPicYuvRec(), ch);
    Pel* pPicDec  = getPicYuvAddr(m_pcYuvTmp, ch);

    std::vector<NDBFBlockInfo>& vFilterBlocks = *(m_pcPic->getCU(iAddr)->getNDBFilterBlocks());

    //variables
    UInt  xPos, yPos, width, height;
    Bool* pbBorderAvail;
    UInt  posOffset;

    UInt csx = m_pcPic->getComponentScaleX(ch);
    UInt csy = m_pcPic->getComponentScaleY(ch);

    for(Int i=0; i< vFilterBlocks.size(); i++)
    {
      xPos        = vFilterBlocks[i].posX   >> csx;
      yPos        = vFilterBlocks[i].posY   >> csy;
      width       = vFilterBlocks[i].width  >> csx;
      height      = vFilterBlocks[i].height >> csy;
      pbBorderAvail = vFilterBlocks[i].isBorderAvailable;

      posOffset = (yPos* stride) + xPos;

      processSaoBlock(pPicDec+ posOffset, pPicRest+ posOffset, stride, iSaoType, width, height, pbBorderAvail, ch);
    }
  }
}

/** Perform SAO for non-cross-slice or non-cross-tile process
 * \param  pDec to-be-filtered block buffer pointer
 * \param  pRest filtered block buffer pointer
 * \param  stride picture buffer stride
 * \param  saoType SAO offset type
 * \param  xPos x coordinate
 * \param  yPos y coordinate
 * \param  width block width
 * \param  height block height
 * \param  pbBorderAvail availabilities of block border pixels
 */
Void TComSampleAdaptiveOffset::processSaoBlock(Pel* pDec, Pel* pRest, Int stride, Int saoType, UInt width, UInt height, Bool* pbBorderAvail, ComponentID compID)
{
  //variables
  Int startX, startY, endX, endY, x, y;
  Int signLeft,signRight,signDown,signDown1;
  UInt edgeType;
  Pel *pClipTbl  = m_apClipTable[toChannelType(compID)];
  Int *pOffsetBo = m_aiOffsetBo[toChannelType(compID)];

  switch (saoType)
  {
  case SAO_EO_0: // dir: -
    {

      startX = (pbBorderAvail[SGU_L]) ? 0 : 1;
      endX   = (pbBorderAvail[SGU_R]) ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = xSign(pDec[startX] - pDec[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight =  xSign(pDec[x] - pDec[x+1]);
          edgeType =  signRight + signLeft + 2;
          signLeft  = -signRight;

          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
        pDec  += stride;
        pRest += stride;
      }
      break;
    }
  case SAO_EO_1: // dir: |
    {
      startY = (pbBorderAvail[SGU_T]) ? 0 : 1;
      endY   = (pbBorderAvail[SGU_B]) ? height : height-1;
      if (!pbBorderAvail[SGU_T])
      {
        pDec  += stride;
        pRest += stride;
      }
      for (x=0; x< width; x++)
      {
        m_iUpBuff1[x] = xSign(pDec[x] - pDec[x-stride]);
      }
      for (y=startY; y<endY; y++)
      {
        for (x=0; x< width; x++)
        {
          signDown  = xSign(pDec[x] - pDec[x+stride]);
          edgeType = signDown + m_iUpBuff1[x] + 2;
          m_iUpBuff1[x]= -signDown;

          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
        pDec  += stride;
        pRest += stride;
      }
      break;
    }
  case SAO_EO_2: // dir: 135
    {
      Int posShift= stride + 1;

      startX = (pbBorderAvail[SGU_L]) ? 0 : 1 ;
      endX   = (pbBorderAvail[SGU_R]) ? width : (width-1);

      //prepare 2nd line upper sign
      pDec += stride;
      for (x=startX; x< endX+1; x++)
      {
        m_iUpBuff1[x] = xSign(pDec[x] - pDec[x- posShift]);
      }

      //1st line
      pDec -= stride;
      if(pbBorderAvail[SGU_TL])
      {
        x= 0;
        edgeType      =  xSign(pDec[x] - pDec[x- posShift]) - m_iUpBuff1[x+1] + 2;
        pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];

      }
      if(pbBorderAvail[SGU_T])
      {
        for(x= 1; x< endX; x++)
        {
          edgeType      =  xSign(pDec[x] - pDec[x- posShift]) - m_iUpBuff1[x+1] + 2;
          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
      }
      pDec   += stride;
      pRest  += stride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        for (x=startX; x<endX; x++)
        {
          signDown1      =  xSign(pDec[x] - pDec[x+ posShift]) ;
          edgeType      =  signDown1 + m_iUpBuff1[x] + 2;
          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];

          m_iUpBufft[x+1] = -signDown1;
        }
        m_iUpBufft[startX] = xSign(pDec[stride+startX] - pDec[startX-1]);

        ipSwap     = m_iUpBuff1;
        m_iUpBuff1 = m_iUpBufft;
        m_iUpBufft = ipSwap;

        pDec  += stride;
        pRest += stride;
      }

      //last line
      if(pbBorderAvail[SGU_B])
      {
        for(x= startX; x< width-1; x++)
        {
          edgeType =  xSign(pDec[x] - pDec[x+ posShift]) + m_iUpBuff1[x] + 2;
          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
      }
      if(pbBorderAvail[SGU_BR])
      {
        x= width -1;
        edgeType =  xSign(pDec[x] - pDec[x+ posShift]) + m_iUpBuff1[x] + 2;
        pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
      }
      break;
    }
  case SAO_EO_3: // dir: 45
    {
      Int  posShift     = stride - 1;
      startX = (pbBorderAvail[SGU_L]) ? 0 : 1;
      endX   = (pbBorderAvail[SGU_R]) ? width : (width -1);

      //prepare 2nd line upper sign
      pDec += stride;
      for (x=startX-1; x< endX; x++)
      {
        m_iUpBuff1[x] = xSign(pDec[x] - pDec[x- posShift]);
      }


      //first line
      pDec -= stride;
      if(pbBorderAvail[SGU_T])
      {
        for(x= startX; x< width -1; x++)
        {
          edgeType = xSign(pDec[x] - pDec[x- posShift]) -m_iUpBuff1[x-1] + 2;
          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
      }
      if(pbBorderAvail[SGU_TR])
      {
        x= width-1;
        edgeType = xSign(pDec[x] - pDec[x- posShift]) -m_iUpBuff1[x-1] + 2;
        pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
      }
      pDec  += stride;
      pRest += stride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        for(x= startX; x< endX; x++)
        {
          signDown1      =  xSign(pDec[x] - pDec[x+ posShift]) ;
          edgeType      =  signDown1 + m_iUpBuff1[x] + 2;

          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
          m_iUpBuff1[x-1] = -signDown1;
        }
        m_iUpBuff1[endX-1] = xSign(pDec[endX-1 + stride] - pDec[endX]);

        pDec  += stride;
        pRest += stride;
      }

      //last line
      if(pbBorderAvail[SGU_BL])
      {
        x= 0;
        edgeType = xSign(pDec[x] - pDec[x+ posShift]) + m_iUpBuff1[x] + 2;
        pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];

      }
      if(pbBorderAvail[SGU_B])
      {
        for(x= 1; x< endX; x++)
        {
          edgeType = xSign(pDec[x] - pDec[x+ posShift]) + m_iUpBuff1[x] + 2;
          pRest[x] = pClipTbl[pDec[x] + m_iOffsetEo[edgeType]];
        }
      }
      break;
    }
  case SAO_BO:
    {
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
          pRest[x] = pOffsetBo[pDec[x]];
        }
        pRest += stride;
        pDec  += stride;
      }
      break;
    }
  default: break;
  }

}

/** sample adaptive offset process for one LCU crossing LCU boundary
 * \param   iAddr, iSaoType, iYCbCr
 */
Void TComSampleAdaptiveOffset::processSaoCuOrg(Int iAddr, Int iSaoType, ComponentID ch)
{
  Int x,y;
  TComDataCU *pTmpCu = m_pcPic->getCU(iAddr);
  Int  iLcuWidth  = m_uiMaxCUWidth;
  Int  iLcuHeight = m_uiMaxCUHeight;
  UInt uiLPelX    = pTmpCu->getCUPelX();
  UInt uiTPelY    = pTmpCu->getCUPelY();
  UInt uiRPelX;
  UInt uiBPelY;
  Int  iSignLeft;
  Int  iSignRight;
  Int  iSignDown;
  Int  iSignDown1;
  Int  iSignDown2;
  UInt uiEdgeType;
  Int iPicWidthTmp;
  Int iPicHeightTmp;
  Int iStartX;
  Int iStartY;
  Int iEndX;
  Int iEndY;
  Int iShift;
  Int iCuHeightTmp;
  Pel *pTmpLSwap;
  Pel *pTmpL;
  Pel *pTmpU;
  Pel *pClipTbl = m_apClipTable[toChannelType(ch)];
  Int *pOffsetBo = m_aiOffsetBo[toChannelType(ch)];

  const UInt csx = m_pcPic->getComponentScaleX(ch);
  const UInt csy = m_pcPic->getComponentScaleY(ch);

  iPicWidthTmp  = m_iPicWidth  >> csx;
  iPicHeightTmp = m_iPicHeight >> csy;
  iLcuWidth     = iLcuWidth    >> csx;
  iLcuHeight    = iLcuHeight   >> csy;
  uiLPelX       = uiLPelX      >> csx;
  uiTPelY       = uiTPelY      >> csy;
  uiRPelX       = uiLPelX + iLcuWidth  ;
  uiBPelY       = uiTPelY + iLcuHeight ;
  uiRPelX       = uiRPelX > iPicWidthTmp  ? iPicWidthTmp  : uiRPelX;
  uiBPelY       = uiBPelY > iPicHeightTmp ? iPicHeightTmp : uiBPelY;
  iLcuWidth     = uiRPelX - uiLPelX;
  iLcuHeight    = uiBPelY - uiTPelY;

  if(pTmpCu->getPic()==0)
  {
    return;
  }

  Pel* pRec             = m_pcPic->getPicYuvRec()->getAddr(ch, iAddr);
  const Int  iStride    = m_pcPic->getStride(ch);

//   if (iSaoType!=SAO_BO_0 || iSaoType!=SAO_BO_1)
  {
    iCuHeightTmp = (m_uiMaxCUHeight >> csy);
    iShift = (m_uiMaxCUWidth>> csx)-1;
    for (Int i=0;i<iCuHeightTmp+1;i++)
    {
      m_pTmpL2[i] = pRec[iShift];
      pRec += iStride;
    }
    pRec -= (iStride*(iCuHeightTmp+1));

    pTmpL = m_pTmpL1;
    pTmpU = &(m_pTmpU1[uiLPelX]);
  }

  switch (iSaoType)
  {
  case SAO_EO_0: // dir: -
    {
      iStartX = (uiLPelX == 0) ? 1 : 0;
      iEndX   = (uiRPelX == iPicWidthTmp) ? iLcuWidth-1 : iLcuWidth;
      for (y=0; y<iLcuHeight; y++)
      {
        iSignLeft = xSign(pRec[iStartX] - pTmpL[y]);
        for (x=iStartX; x< iEndX; x++)
        {
          iSignRight =  xSign(pRec[x] - pRec[x+1]);
          uiEdgeType =  iSignRight + iSignLeft + 2;
          iSignLeft  = -iSignRight;

          pRec[x] = pClipTbl[pRec[x] + m_iOffsetEo[uiEdgeType]];
        }
        pRec += iStride;
      }
      break;
    }
  case SAO_EO_1: // dir: |
    {
      iStartY = (uiTPelY == 0) ? 1 : 0;
      iEndY   = (uiBPelY == iPicHeightTmp) ? iLcuHeight-1 : iLcuHeight;
      if (uiTPelY == 0)
      {
        pRec += iStride;
      }
      for (x=0; x< iLcuWidth; x++)
      {
        m_iUpBuff1[x] = xSign(pRec[x] - pTmpU[x]);
      }
      for (y=iStartY; y<iEndY; y++)
      {
        for (x=0; x<iLcuWidth; x++)
        {
          iSignDown  = xSign(pRec[x] - pRec[x+iStride]);
          uiEdgeType = iSignDown + m_iUpBuff1[x] + 2;
          m_iUpBuff1[x]= -iSignDown;

          pRec[x] = pClipTbl[pRec[x] + m_iOffsetEo[uiEdgeType]];
        }
        pRec += iStride;
      }
      break;
    }
  case SAO_EO_2: // dir: 135
    {
      iStartX = (uiLPelX == 0)            ? 1 : 0;
      iEndX   = (uiRPelX == iPicWidthTmp) ? iLcuWidth-1 : iLcuWidth;

      iStartY = (uiTPelY == 0) ?             1 : 0;
      iEndY   = (uiBPelY == iPicHeightTmp) ? iLcuHeight-1 : iLcuHeight;

      if (uiTPelY == 0)
      {
        pRec += iStride;
      }

      for (x=iStartX; x<iEndX; x++)
      {
        m_iUpBuff1[x] = xSign(pRec[x] - pTmpU[x-1]);
      }
      for (y=iStartY; y<iEndY; y++)
      {
        iSignDown2 = xSign(pRec[iStride+iStartX] - pTmpL[y]);
        for (x=iStartX; x<iEndX; x++)
        {
          iSignDown1      =  xSign(pRec[x] - pRec[x+iStride+1]) ;
          uiEdgeType      =  iSignDown1 + m_iUpBuff1[x] + 2;
          m_iUpBufft[x+1] = -iSignDown1;
          pRec[x] = pClipTbl[pRec[x] + m_iOffsetEo[uiEdgeType]];
        }
        m_iUpBufft[iStartX] = iSignDown2;

        ipSwap     = m_iUpBuff1;
        m_iUpBuff1 = m_iUpBufft;
        m_iUpBufft = ipSwap;

        pRec += iStride;
      }
      break;
    }
  case SAO_EO_3: // dir: 45
    {
      iStartX = (uiLPelX == 0) ? 1 : 0;
      iEndX   = (uiRPelX == iPicWidthTmp) ? iLcuWidth-1 : iLcuWidth;

      iStartY = (uiTPelY == 0) ? 1 : 0;
      iEndY   = (uiBPelY == iPicHeightTmp) ? iLcuHeight-1 : iLcuHeight;

      if (iStartY == 1)
      {
        pRec += iStride;
      }

      for (x=iStartX-1; x<iEndX; x++)
      {
        m_iUpBuff1[x] = xSign(pRec[x] - pTmpU[x+1]);
      }
      for (y=iStartY; y<iEndY; y++)
      {
        x=iStartX;
        iSignDown1      =  xSign(pRec[x] - pTmpL[y+1]) ;
        uiEdgeType      =  iSignDown1 + m_iUpBuff1[x] + 2;
        m_iUpBuff1[x-1] = -iSignDown1;
        pRec[x] = pClipTbl[pRec[x] + m_iOffsetEo[uiEdgeType]];
        for (x=iStartX+1; x<iEndX; x++)
        {
          iSignDown1      =  xSign(pRec[x] - pRec[x+iStride-1]) ;
          uiEdgeType      =  iSignDown1 + m_iUpBuff1[x] + 2;
          m_iUpBuff1[x-1] = -iSignDown1;
          pRec[x] = pClipTbl[pRec[x] + m_iOffsetEo[uiEdgeType]];
        }
        m_iUpBuff1[iEndX-1] = xSign(pRec[iEndX-1 + iStride] - pRec[iEndX]);

        pRec += iStride;
      }
      break;
    }
  case SAO_BO:
    {
      for (y=0; y<iLcuHeight; y++)
      {
        for (x=0; x<iLcuWidth; x++)
        {
          pRec[x] = pOffsetBo[pRec[x]];
        }
        pRec += iStride;
      }
      break;
    }
  default: break;
  }
//   if (iSaoType!=SAO_BO_0 || iSaoType!=SAO_BO_1)
  {
    pTmpLSwap = m_pTmpL1;
    m_pTmpL1  = m_pTmpL2;
    m_pTmpL2  = pTmpLSwap;
  }
}

/** Sample adaptive offset process
 * \param pcPic, pcSaoParam
 */
Void TComSampleAdaptiveOffset::SAOProcess(SAOParam* pcSaoParam)
{
  for(UInt ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_auiSaoBitIncrease[ch] = max(g_bitDepth[ch] - 10, 0);
  }

  if(m_bUseNIF)
  {
    m_pcPic->getPicYuvRec()->copyToPic(m_pcYuvTmp);
  }

  if (m_saoLcuBasedOptimization)
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      pcSaoParam->oneUnitFlag[i] = 0;
    }
  }

  for(UInt chan=0; chan<m_pcPic->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    processSaoUnitAll( pcSaoParam->saoLcuParam[ch], pcSaoParam->oneUnitFlag[ch], ch);
  }

  m_pcPic = NULL;
}

/** Process SAO all units
 * \param saoLcuParam SAO LCU parameters
 * \param oneUnitFlag one unit flag
 * \param yCbCr color componet index
 */
Void TComSampleAdaptiveOffset::processSaoUnitAll(SaoLcuParam* saoLcuParam, Bool oneUnitFlag, ComponentID ch)
{

  const UInt sx=m_pcPic->getComponentScaleX(ch);
  const UInt sy=m_pcPic->getComponentScaleY(ch);
  const Int picWidthTmp = m_iPicWidth>>sx;
  const Int stride = m_pcPic->getStride(ch);

  {
    Pel *pRec        = m_pcPic->getPicYuvRec()->getAddr(ch);
    memcpy(m_pTmpU1, pRec, sizeof(Pel)*picWidthTmp);
  }

  Int  i;
  UInt edgeType;
  Int* pOffsetBo = m_aiOffsetBo[toChannelType(ch)];
  Int  typeIdx;

  Int offset[LUMA_GROUP_NUM+1];
  Int idxX;
  Int idxY;
  Int addr;
  Int frameWidthInCU = m_pcPic->getFrameWidthInCU();
  Int frameHeightInCU = m_pcPic->getFrameHeightInCU();
  Pel *tmpUSwap;
  Bool mergeLeftFlag;
  Int saoBitIncrease = m_auiSaoBitIncrease[toChannelType(ch)];

  offset[0] = 0;
  for (idxY = 0; idxY< frameHeightInCU; idxY++)
  {
    addr = idxY * frameWidthInCU;
    Pel *pRec        = m_pcPic->getPicYuvRec()->getAddr(ch, addr);

    //     pRec += iStride*(m_uiMaxCUHeight-1);
    for (i=0;i<(m_uiMaxCUHeight>>sy)+1;i++)
    {
      m_pTmpL1[i] = pRec[0];
      pRec+=stride;
    }
    pRec-=(stride<<1);

    memcpy(m_pTmpU2, pRec, sizeof(Pel)*picWidthTmp);

    for (idxX = 0; idxX < frameWidthInCU; idxX++)
    {
      addr = idxY * frameWidthInCU + idxX;

      if (oneUnitFlag)
      {
        typeIdx = saoLcuParam[0].typeIdx;
        mergeLeftFlag = (addr == 0)? 0:1;
      }
      else
      {
        typeIdx = saoLcuParam[addr].typeIdx;
        mergeLeftFlag = saoLcuParam[addr].mergeLeftFlag;
      }

      if (typeIdx>=0)
      {
        if (!mergeLeftFlag)
        {

          if (typeIdx == SAO_BO)
          {
            for (i=0; i<SAO_MAX_BO_CLASSES+1;i++)
            {
              offset[i] = 0;
            }
            for (i=0; i<saoLcuParam[addr].length; i++)
            {
              offset[ (saoLcuParam[addr].subTypeIdx +i)%SAO_MAX_BO_CLASSES  +1] = saoLcuParam[addr].offset[i] << saoBitIncrease;
            }

            Pel* ppTable = m_aTableBo[toChannelType(ch)];
            Pel* pClipTable = m_apClipTable[toChannelType(ch)];
            Int bitDepth = g_bitDepth[toChannelType(ch)];

            for (i=0;i<(1<<bitDepth);i++)
            {
              pOffsetBo[i] = pClipTable[i + offset[ppTable[i]]];
            }

          }
          if (typeIdx == SAO_EO_0 || typeIdx == SAO_EO_1 || typeIdx == SAO_EO_2 || typeIdx == SAO_EO_3)
          {
            for (i=0;i<saoLcuParam[addr].length;i++)
            {
              offset[i+1] = saoLcuParam[addr].offset[i] << saoBitIncrease;
            }
            for (edgeType=0;edgeType<6;edgeType++)
            {
              m_iOffsetEo[edgeType]= offset[m_auiEoTable[edgeType]];
            }
          }
        }
        processSaoCu(addr, typeIdx, ch);
      }
      else
      {
        if (idxX != (frameWidthInCU-1))
        {
          pRec        = m_pcPic->getPicYuvRec()->getAddr(ch, addr);
          Int widthShift = m_uiMaxCUWidth>>sx;
          for (i=0;i<(m_uiMaxCUHeight>>sy)+1;i++)
          {
            m_pTmpL1[i] = pRec[widthShift-1];
            pRec+=stride;
          }
        }
      }
    }
    tmpUSwap = m_pTmpU1;
    m_pTmpU1 = m_pTmpU2;
    m_pTmpU2 = tmpUSwap;
  }

}
/** Reset SAO LCU part
 * \param saoLcuParam
 */
Void TComSampleAdaptiveOffset::resetLcuPart(SaoLcuParam* saoLcuParam)
{
  Int i,j;
  for (i=0;i<m_iNumCuInWidth*m_iNumCuInHeight;i++)
  {
    saoLcuParam[i].mergeUpFlag     =  1;
    saoLcuParam[i].mergeLeftFlag =  0;
    saoLcuParam[i].partIdx   =  0;
    saoLcuParam[i].typeIdx      = -1;
    for (j=0;j<MAX_NUM_SAO_OFFSETS;j++)
    {
      saoLcuParam[i].offset[j] = 0;
    }
    saoLcuParam[i].subTypeIdx = 0;
  }
}

/** convert QP part to SAO unit
* \param saoParam SAO parameter
* \param partIdx SAO part index
* \param yCbCr color component index
 */
Void TComSampleAdaptiveOffset::convertQT2SaoUnit(SAOParam *saoParam, UInt partIdx, ComponentID ch)
{

  SAOQTPart*  saoPart= &(saoParam->psSaoPart[ch][partIdx]);
  if (!saoPart->bSplit)
  {
    convertOnePart2SaoUnit(saoParam, partIdx, ch);
    return;
  }

  if (saoPart->PartLevel < m_uiMaxSplitLevel)
  {
    convertQT2SaoUnit(saoParam, saoPart->DownPartsIdx[0], ch);
    convertQT2SaoUnit(saoParam, saoPart->DownPartsIdx[1], ch);
    convertQT2SaoUnit(saoParam, saoPart->DownPartsIdx[2], ch);
    convertQT2SaoUnit(saoParam, saoPart->DownPartsIdx[3], ch);
  }
}
/** convert one SAO part to SAO unit
* \param saoParam SAO parameter
* \param partIdx SAO part index
* \param yCbCr color component index
 */
Void TComSampleAdaptiveOffset::convertOnePart2SaoUnit(SAOParam *saoParam, UInt partIdx, ComponentID ch)
{
  Int j;
  Int idxX;
  Int idxY;
  Int addr;
  Int frameWidthInCU = m_pcPic->getFrameWidthInCU();
  SAOQTPart* saoQTPart = saoParam->psSaoPart[ch];
  SaoLcuParam* saoLcuParam = saoParam->saoLcuParam[ch];

  for (idxY = saoQTPart[partIdx].StartCUY; idxY<= saoQTPart[partIdx].EndCUY; idxY++)
  {
    for (idxX = saoQTPart[partIdx].StartCUX; idxX<= saoQTPart[partIdx].EndCUX; idxX++)
    {
      addr = idxY * frameWidthInCU + idxX;
      saoLcuParam[addr].partIdxTmp = (Int)partIdx;
      saoLcuParam[addr].typeIdx    = saoQTPart[partIdx].iBestType;
      saoLcuParam[addr].subTypeIdx = saoQTPart[partIdx].subTypeIdx;
      if (saoLcuParam[addr].typeIdx!=-1)
      {
        saoLcuParam[addr].length    = saoQTPart[partIdx].iLength;
        for (j=0;j<MAX_NUM_SAO_OFFSETS;j++)
        {
          saoLcuParam[addr].offset[j] = saoQTPart[partIdx].iOffset[j];
        }
      }
      else
      {
        saoLcuParam[addr].length    = 0;
        saoLcuParam[addr].subTypeIdx = saoQTPart[partIdx].subTypeIdx;
        for (j=0;j<MAX_NUM_SAO_OFFSETS;j++)
        {
          saoLcuParam[addr].offset[j] = 0;
        }
      }
    }
  }
}

Void TComSampleAdaptiveOffset::resetSaoUnit(SaoLcuParam* saoUnit)
{
  saoUnit->partIdx       = 0;
  saoUnit->partIdxTmp    = 0;
  saoUnit->mergeLeftFlag = 0;
  saoUnit->mergeUpFlag   = 0;
  saoUnit->typeIdx       = -1;
  saoUnit->length        = 0;
  saoUnit->subTypeIdx    = 0;

  for (Int i=0;i<4;i++)
  {
    saoUnit->offset[i] = 0;
  }
}

Void TComSampleAdaptiveOffset::copySaoUnit(SaoLcuParam* saoUnitDst, SaoLcuParam* saoUnitSrc )
{
  saoUnitDst->mergeLeftFlag = saoUnitSrc->mergeLeftFlag;
  saoUnitDst->mergeUpFlag   = saoUnitSrc->mergeUpFlag;
  saoUnitDst->typeIdx       = saoUnitSrc->typeIdx;
  saoUnitDst->length        = saoUnitSrc->length;
  saoUnitDst->subTypeIdx    = saoUnitSrc->subTypeIdx;

  for (Int i=0;i<4;i++)
  {
    saoUnitDst->offset[i] = saoUnitSrc->offset[i];
  }
}
#endif
/** PCM LF disable process.
 * \param pcPic picture (TComPic) pointer
 * \returns Void
 *
 * \note Replace filtered sample values of PCM mode blocks with the transmitted and reconstructed ones.
 */
Void TComSampleAdaptiveOffset::PCMLFDisableProcess (TComPic* pcPic)
{
  xPCMRestoration(pcPic);
}

/** Picture-level PCM restoration.
 * \param pcPic picture (TComPic) pointer
 * \returns Void
 */
Void TComSampleAdaptiveOffset::xPCMRestoration(TComPic* pcPic)
{
  Bool  bPCMFilter = (pcPic->getSlice(0)->getSPS()->getUsePCM() && pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag())? true : false;

  if(bPCMFilter || pcPic->getSlice(0)->getPPS()->getTransquantBypassEnableFlag())
  {
    for( UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumCUsInFrame() ; uiCUAddr++ )
    {
      TComDataCU* pcCU = pcPic->getCU(uiCUAddr);

      xPCMCURestoration(pcCU, 0, 0);
    }
  }
}

/** PCM CU restoration.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param uiDepth CU depth
 * \returns Void
 */
Void TComSampleAdaptiveOffset::xPCMCURestoration ( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth )
{
  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartInCU() >> (uiDepth<<1);
  UInt uiQNumParts   = uiCurNumParts>>2;

  // go to sub-CU
  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
        xPCMCURestoration( pcCU, uiAbsZorderIdx, uiDepth+1 );
    }
    return;
  }

  // restore PCM samples
  if ((pcCU->getIPCMFlag(uiAbsZorderIdx)&& pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag()) || pcCU->isLosslessCoded( uiAbsZorderIdx))
  {
    const UInt numComponents=pcPic->getNumberValidComponents();
    for(UInt comp=0; comp<numComponents; comp++)
    {
      xPCMSampleRestoration (pcCU, uiAbsZorderIdx, uiDepth, ComponentID(comp));
    }
  }
}

/** PCM sample restoration.
 * \param pcCU pointer to current CU
 * \param uiAbsPartIdx part index
 * \param uiDepth CU depth
 * \param ttText texture component type
 * \returns Void
 */
Void TComSampleAdaptiveOffset::xPCMSampleRestoration (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, const ComponentID compID)
{
        TComPicYuv* pcPicYuvRec = pcCU->getPic()->getPicYuvRec();
        UInt uiPcmLeftShiftBit;
  const UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  const UInt csx=pcPicYuvRec->getComponentScaleX(compID);
  const UInt csy=pcPicYuvRec->getComponentScaleY(compID);
  const UInt uiOffset   = (uiMinCoeffSize*uiAbsZorderIdx)>>(csx+csy);

        Pel *piSrc = pcPicYuvRec->getAddr(compID, pcCU->getAddr(), uiAbsZorderIdx);
  const Pel *piPcm = pcCU->getPCMSample(compID) + uiOffset;
  const UInt uiStride  = pcPicYuvRec->getStride(compID);
  const UInt uiWidth  = ((g_uiMaxCUWidth >> uiDepth) >> csx);
  const UInt uiHeight = ((g_uiMaxCUWidth >> uiDepth) >> csy);

  if ( pcCU->isLosslessCoded(uiAbsZorderIdx) && !pcCU->getIPCMFlag(uiAbsZorderIdx) )
  {
    uiPcmLeftShiftBit = 0;
  }
  else
  {
    uiPcmLeftShiftBit = g_bitDepth[toChannelType(compID)] - pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
  }

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piSrc[uiX] = (piPcm[uiX] << uiPcmLeftShiftBit);
    }
    piPcm += uiWidth;
    piSrc += uiStride;
  }
}

//! \}
