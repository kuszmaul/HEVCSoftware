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

/** \file     WeightedPredAnalysis.cpp
    \brief    encoder class
*/

#include "../TLibCommon/TypeDef.h"
#include "../TLibCommon/TComSlice.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComPicYuv.h"
#include "WeightPredAnalysis.h"

#define ABS(a)    ((a) < 0 ? - (a) : (a))
#define DTHRESH (0.99)

WeightPredAnalysis::WeightPredAnalysis()
{
  m_weighted_pred_flag = false;
  m_weighted_bipred_flag = false;

  for ( UInt lst =0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )
  {
    for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ ) 
    {
      for ( int comp=0 ; comp<MAX_NUM_COMPONENT ;comp++ )
      {
        wpScalingParam  *pwp   = &(m_wp[lst][iRefIdx][comp]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

/** calculate AC and DC values for current original image
 * \param TComSlice *slice
 * \returns Void
 */
Bool  WeightPredAnalysis::xCalcACDCParamSlice(TComSlice *slice)
{
  //===== calculate AC/DC value =====
  TComPicYuv*   pPic = slice->getPic()->getPicYuvOrg();

  wpACDCParam weightACDCParam[MAX_NUM_COMPONENT];

  for(Int chan=0; chan<pPic->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);


    // calculate DC/AC value for channel

    const Int iStride = pPic->getStride(ch);
    const Int iWidth  = pPic->getWidth(ch);
    const Int iHeight = pPic->getHeight(ch);

    const Int iSample = iWidth*iHeight;

    Int64 iOrgDC = 0;
    {
      const Pel*  pPel    = pPic->getAddr(ch);
      for(Int y = 0; y < iHeight; y++, pPel+=iStride )
        for(Int x = 0; x < iWidth; x++ )
          iOrgDC += (Int)( pPel[x] );
    }

    Int64  iOrgNormDC = ((iOrgDC+(iSample>>1)) / iSample);

    Int64 iOrgAC = 0;
    {
      const Pel*  pPel    = pPic->getAddr(ch);
      for(Int y = 0; y < iHeight; y++, pPel += iStride )
        for(Int x = 0; x < iWidth; x++ )
          iOrgAC += abs( (Int)pPel[x] - (Int)iOrgNormDC );
    }


    weightACDCParam[ch].iDC = iOrgNormDC;
    weightACDCParam[ch].iAC = iOrgAC;
  }

  slice->setWpAcDcParam(weightACDCParam);
  return (true);
}

/** store weighted_pred_flag and weighted_bipred_idc values
 * \param weighted_pred_flag
 * \param weighted_bipred_idc
 * \returns Void
 */
Void  WeightPredAnalysis::xStoreWPparam(Bool weighted_pred_flag, Bool weighted_bipred_flag)
{
  m_weighted_pred_flag = weighted_pred_flag;
  m_weighted_bipred_flag = weighted_bipred_flag;
}

/** restore weighted_pred_flag and weighted_bipred_idc values
 * \param TComSlice *slice
 * \returns Void
 */
Void  WeightPredAnalysis::xRestoreWPparam(TComSlice *slice)
{
  slice->getPPS()->setUseWP(m_weighted_pred_flag);
  slice->getPPS()->setWPBiPred(m_weighted_bipred_flag);
}

/** check weighted pred or non-weighted pred
 * \param TComSlice *slice
 * \returns Void
 */
Void  WeightPredAnalysis::xCheckWPEnable(TComSlice *slice)
{
  const TComPicYuv*   pPic = slice->getPic()->getPicYuvOrg();

  Int iPresentCnt = 0;
  for ( UInt lst=0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )
  {
    for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ ) 
    {
      for(Int chan=0; chan<pPic->getNumberValidComponents(); chan++)
      {
        wpScalingParam  *pwp = &(m_wp[lst][iRefIdx][chan]);
        iPresentCnt += (Int)pwp->bPresentFlag;
      }
    }
  }

  if(iPresentCnt==0)
  {
    slice->getPPS()->setUseWP(false);
    slice->getPPS()->setWPBiPred(false);

    for ( UInt lst=0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )
    {
      for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ ) 
      {
        for(Int chan=0; chan<pPic->getNumberValidComponents(); chan++)
        {
          wpScalingParam  *pwp = &(m_wp[lst][iRefIdx][chan]);
          pwp->bPresentFlag      = false;
          pwp->uiLog2WeightDenom = 0;
          pwp->iWeight           = 1;
          pwp->iOffset           = 0;
        }
      }
    }
    slice->setWpScaling( m_wp );
  }
}

/** estimate wp tables for explicit wp
 * \param TComSlice *slice
 * \returns Bool
 */
Bool  WeightPredAnalysis::xEstimateWPParamSlice(TComSlice *slice)
{
  Int iDenom  = 6;
#if WP_PARAM_RANGE_LIMIT
  Bool validRangeFlag = false;
#else
  Int iRealDenom = iDenom + (g_uiBitDepth+g_uiBitIncrement-8);
  Int iRealOffset = ((Int)1<<(iRealDenom-1));
  const Int numComp = slice->getPic()->getPicYuvOrg()->getNumberValidComponents();
#endif

  if(slice->getNumRefIdx(REF_PIC_LIST_0)>3)
  {
    iDenom  = 7;
#if WP_PARAM_RANGE_LIMIT
#else
    iRealDenom = iDenom + (g_uiBitDepth+g_uiBitIncrement-8);
    iRealOffset = ((Int)1<<(iRealDenom-1));
#endif
  }

#if WP_PARAM_RANGE_LIMIT
  do
  {
    validRangeFlag = xUpdatingWPParameters(slice, m_wp, iDenom);
    if (!validRangeFlag)
    {
      iDenom--; // decrement to satisfy the range limitation
    }
  } while (validRangeFlag == false);
#else
  Int iNumPredDir = slice->isInterP() ? 1 : 2;
  assert (iNumPredDir <= Int(NUM_REF_PIC_LIST_01));
  for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
  {
    RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdxTemp = 0; iRefIdxTemp < slice->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
    {
      wpACDCParam *CurrWeightACDCParam, *RefWeightACDCParam;
      slice->getWpAcDcParam(CurrWeightACDCParam);
      slice->getRefPic(eRefPicList, iRefIdxTemp)->getSlice(0)->getWpAcDcParam(RefWeightACDCParam);

      for ( Int iComp = 0; iComp < numComp; iComp++ )
      {
        // current frame
        Int64 iCurrDC = CurrWeightACDCParam[iComp].iDC;
        Int64 iCurrAC = CurrWeightACDCParam[iComp].iAC;
        // reference frame
        Int64 iRefDC = RefWeightACDCParam[iComp].iDC;
        Int64 iRefAC = RefWeightACDCParam[iComp].iAC;

        // calculating iWeight and iOffset params
        Double dWeight = (iRefAC==0) ? (Double)1.0 : Clip3( -16.0, 15.0, ((Double)iCurrAC / (Double)iRefAC) );
        Int iWeight = (Int)( 0.5 + dWeight * (Double)(1<<iDenom) );
        Int iOffset = (Int)( ((iCurrDC<<iDenom) - ((Int64)iWeight * iRefDC) + (Int64)iRealOffset) >> iRealDenom );

        m_wp[iRefList][iRefIdxTemp][iComp].bPresentFlag = true;
        m_wp[iRefList][iRefIdxTemp][iComp].iWeight = (Int)iWeight;
        m_wp[iRefList][iRefIdxTemp][iComp].iOffset = (Int)iOffset;
        m_wp[iRefList][iRefIdxTemp][iComp].uiLog2WeightDenom = (Int)iDenom;
      }
    }
  }
#endif

  // selecting whether WP is used, or not
  xSelectWP(slice, m_wp, iDenom);
  
  slice->setWpScaling( m_wp );

  return (true);
}

#if WP_PARAM_RANGE_LIMIT
/** update wp tables for explicit wp w.r.t ramge limitation
 * \param TComSlice *slice
 * \returns Bool
 */
Bool WeightPredAnalysis::xUpdatingWPParameters(TComSlice *slice, wpScalingParam weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT], Int log2Denom)
{
  const Int numComp = slice->getPic()->getPicYuvOrg()->getNumberValidComponents();
  Int realLog2Denom = log2Denom + (g_uiBitDepth+g_uiBitIncrement-8);
  Int realOffset = ((Int)1<<(realLog2Denom-1));

  Int numPredDir = slice->isInterP() ? 1 : 2;
  assert (numPredDir <= Int(NUM_REF_PIC_LIST_01));
  for ( Int refList = 0; refList < numPredDir; refList++ )
  {
    RefPicList  eRefPicList = ( refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int refIdxTemp = 0; refIdxTemp < slice->getNumRefIdx(eRefPicList); refIdxTemp++ )
    {
      wpACDCParam *currWeightACDCParam, *refWeightACDCParam;
      slice->getWpAcDcParam(currWeightACDCParam);
      slice->getRefPic(eRefPicList, refIdxTemp)->getSlice(0)->getWpAcDcParam(refWeightACDCParam);

      for ( Int comp = 0; comp < numComp; comp++ )
      {
        // current frame
        Int64 currDC = currWeightACDCParam[comp].iDC;
        Int64 currAC = currWeightACDCParam[comp].iAC;
        // reference frame
        Int64 refDC = refWeightACDCParam[comp].iDC;
        Int64 refAC = refWeightACDCParam[comp].iAC;

        // calculating iWeight and iOffset params
        Double dWeight = (refAC==0) ? (Double)1.0 : Clip3( -16.0, 15.0, ((Double)currAC / (Double)refAC) );
        Int weight = (Int)( 0.5 + dWeight * (Double)(1<<log2Denom) );
        Int offset = (Int)( ((currDC<<log2Denom) - ((Int64)weight * refDC) + (Int64)realOffset) >> realLog2Denom );

        // Chroma offset range limination
        if(comp)
        {
          Int shift = ((1<<(g_uiBitDepth+g_uiBitIncrement-1)));
          Int pred = ( shift - ( ( shift*weight)>>(log2Denom) ) );
          Int deltaOffset = Clip3( -512, 511, (offset - pred) );    // signed 10bit
          offset = Clip3( -128, 127, (deltaOffset + pred) );        // signed 8bit
        }

        // Weighting factor limitation
        Int defaultWeight = (1<<log2Denom);
        Int deltaWeight = (defaultWeight - weight);
        if(deltaWeight > 127 || deltaWeight < -128)
          return (false);

        m_wp[refList][refIdxTemp][comp].bPresentFlag = true;
        m_wp[refList][refIdxTemp][comp].iWeight = (Int)weight;
        m_wp[refList][refIdxTemp][comp].iOffset = (Int)offset;
        m_wp[refList][refIdxTemp][comp].uiLog2WeightDenom = (Int)log2Denom;
      }
    }
  }
  return (true);
}
#endif

/** select whether weighted pred enables or not. 
 * \param TComSlice *slice
 * \param wpScalingParam
 * \param iDenom
 * \returns Bool
 */
Bool WeightPredAnalysis::xSelectWP(TComSlice *slice, wpScalingParam weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT], Int iDenom)
{
  TComPicYuv*   pPic = slice->getPic()->getPicYuvOrg();
  Int iDefaultWeight = ((Int)1<<iDenom);
  Int iNumPredDir = slice->isInterP() ? 1 : 2;
  assert (iNumPredDir <= Int(NUM_REF_PIC_LIST_01));

  for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
  {
    RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdxTemp = 0; iRefIdxTemp < slice->getNumRefIdx(eRefPicList); iRefIdxTemp++ )
    {
      Int64 iSADWP = 0, iSADnoWP = 0;
      for(Int chan=0; chan<pPic->getNumberValidComponents(); chan++)
      {
        const ComponentID ch=ComponentID(chan);

        Pel*  pOrg    = pPic->getAddr(ch);
        Pel*  pRef    = slice->getRefPic(eRefPicList, iRefIdxTemp)->getPicYuvRec()->getAddr(ch);
        const Int   iOrgStride = pPic->getStride(ch);
        const Int   iRefStride = slice->getRefPic(eRefPicList, iRefIdxTemp)->getPicYuvRec()->getStride(ch);
        const Int iWidth  = pPic->getWidth(ch);
        const Int iHeight = pPic->getHeight(ch);

        // calculate SAD costs with/without wp for luma
        iSADWP   += xCalcSADvalueWP(pOrg, pRef, iWidth, iHeight, iOrgStride, iRefStride, iDenom, weightPredTable[iRefList][iRefIdxTemp][ch].iWeight, weightPredTable[iRefList][iRefIdxTemp][ch].iOffset);
        iSADnoWP += xCalcSADvalueWP(pOrg, pRef, iWidth, iHeight, iOrgStride, iRefStride, iDenom, iDefaultWeight, 0);
      }

      Double dRatio = ((Double)iSADWP / (Double)iSADnoWP);
      if(dRatio >= (Double)DTHRESH)
      {
        for(Int chan=0; chan<pPic->getNumberValidComponents(); chan++)
        {
          weightPredTable[iRefList][iRefIdxTemp][chan].bPresentFlag = false;
          weightPredTable[iRefList][iRefIdxTemp][chan].iOffset = (Int)0;
          weightPredTable[iRefList][iRefIdxTemp][chan].iWeight = (Int)iDefaultWeight;
          weightPredTable[iRefList][iRefIdxTemp][chan].uiLog2WeightDenom = (Int)iDenom;
        }
      }
    }
  }
  return (true);
}


/** calculate SAD values for both WP version and non-WP version. 
 * \param Pel *pOrgPel
 * \param Pel *pRefPel
 * \param Int iWidth
 * \param Int iHeight
 * \param Int iOrgStride
 * \param Int iRefStride
 * \param Int iDenom
 * \param Int iWeight
 * \param Int iOffset
 * \returns Int64
 */
Int64 WeightPredAnalysis::xCalcSADvalueWP(Pel *pOrgPel, Pel *pRefPel, Int iWidth, Int iHeight, Int iOrgStride, Int iRefStride, Int iDenom, Int iWeight, Int iOffset)
{
  Int x, y;
  Int64 iSAD = 0;
  Int64 iSize   = iWidth*iHeight;
  Int64 iRealDenom = iDenom + (g_uiBitDepth+g_uiBitIncrement-8);
  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iSAD += ABS(( ((Int64)pOrgPel[x]<<(Int64)iDenom) - ( (Int64)pRefPel[x] * (Int64)iWeight + ((Int64)iOffset<<iRealDenom) ) ) );
    }
    pOrgPel += iOrgStride;
    pRefPel += iRefStride;
  }
  return (iSAD/iSize);
}


