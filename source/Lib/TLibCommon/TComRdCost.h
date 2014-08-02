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

/** \file     TComRdCost.h
    \brief    RD cost computation classes (header)
*/

#ifndef __TCOMRDCOST__
#define __TCOMRDCOST__


#include "CommonDef.h"
#include "TComPattern.h"
#include "TComMv.h"

#include "TComSlice.h"
#include "TComRdCostWeightPrediction.h"

//! \ingroup TLibCommon
//! \{

#define FIX203 1

class DistParam;
class TComPattern;

// ====================================================================================================================
// Type definition
// ====================================================================================================================

// for function pointer
typedef Distortion (*FpDistFunc) (DistParam*); // TODO: RExt - can this pointer be replaced with a reference? - there are no NULL checks on pointer.

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// distortion parameter class
class DistParam
{
public:
  Pel*  pOrg;
  Pel*  pCur;
  Int   iStrideOrg;
  Int   iStrideCur;
  Int   iRows;
  Int   iCols;
  Int   iStep;
  FpDistFunc DistFunc;
  Int   bitDepth;

  Bool            bApplyWeight;     // whether weighted prediction is used or not
  WPScalingParam  *wpCur;           // weighted prediction scaling parameters for current ref
  ComponentID     compIdx;

  // (vertical) subsampling shift (for reducing complexity)
  // - 0 = no subsampling, 1 = even rows, 2 = every 4th, etc.
  Int   iSubShift;

  DistParam()
  {
    pOrg = NULL;
    pCur = NULL;
    iStrideOrg = 0;
    iStrideCur = 0;
    iRows = 0;
    iCols = 0;
    iStep = 1;
    DistFunc = NULL;
    iSubShift = 0;
    bitDepth = 0;
  }
};

/// RD cost computation class
class TComRdCost
{
private:
  // for distortion

  FpDistFunc              m_afpDistortFunc[DF_TOTAL_FUNCTIONS]; // [eDFunc]
  CostMode                m_costMode;
  Double                  m_distortionWeight[MAX_NUM_COMPONENT]; // only chroma values are used.
  Double                  m_dLambda;
  Double                  m_sqrtLambda;
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  Double                  m_dLambdaMotionSAD[2 /* 0=standard, 1=for transquant bypass when mixed-lossless cost evaluation enabled*/];
  Double                  m_dLambdaMotionSSE[2 /* 0=standard, 1=for transquant bypass when mixed-lossless cost evaluation enabled*/];
#else
  UInt                    m_uiLambdaMotionSAD[2 /* 0=standard, 1=for transquant bypass when mixed-lossless cost evaluation enabled*/];
  UInt                    m_uiLambdaMotionSSE[2 /* 0=standard, 1=for transquant bypass when mixed-lossless cost evaluation enabled*/];
#endif
  Double                  m_dFrameLambda;

  // for motion cost
#if FIX203
  TComMv                  m_mvPredictor;
#if SCM__R0309_INTRABC_BVP
  TComMv                  m_mvPredictors[2];
#endif
#else
  UInt*                   m_puiComponentCostOriginP;
  UInt*                   m_puiComponentCost;
  UInt*                   m_puiVerCost;
  UInt*                   m_puiHorCost;
#endif
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  Double                  m_dCost;
#else
  UInt                    m_uiCost;
#endif
  Int                     m_iCostScale;
#if !FIX203
  Int                     m_iSearchLimit;
#endif
#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  Bool                    m_bRGBformat;
  Bool                    m_useColorTrans;
  Bool                    m_useLL;
#endif
#if SCM__R0348_PALETTE_MODE
  Bool                    m_usePaletteMode;
#endif
#if SCM__R0186_INTRABC_BVD
 Int                      m_mvdBin0Cost[4];
#endif
public:
  TComRdCost();
  virtual ~TComRdCost();

  Double  calcRdCost  ( UInt   uiBits, Distortion uiDistortion, Bool bFlag = false, DFunc eDFunc = DF_DEFAULT );
  Double  calcRdCost64( UInt64 uiBits, UInt64 uiDistortion, Bool bFlag = false, DFunc eDFunc = DF_DEFAULT );

  Void    setDistortionWeight  ( const ComponentID compID, const Double distortionWeight ) { m_distortionWeight[compID] = distortionWeight; }
  Void    setLambda      ( Double dLambda );
  Void    setFrameLambda ( Double dLambda ) { m_dFrameLambda = dLambda; }

  Double  getSqrtLambda ()   { return m_sqrtLambda; }

  Double  getLambda() { return m_dLambda; }
  Double  getChromaWeight () { return ((m_distortionWeight[COMPONENT_Cb] + m_distortionWeight[COMPONENT_Cr]) / 2.0); }

  Void    setCostMode(CostMode   m )    { m_costMode = m; }

  // Distortion Functions
  Void    init();

  Void    setDistParam( UInt uiBlkWidth, UInt uiBlkHeight, DFunc eDFunc, DistParam& rcDistParam );
  Void    setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride,            DistParam& rcDistParam );
  Void    setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, Int iStep, DistParam& rcDistParam, Bool bHADME=false );
  Void    setDistParam( DistParam& rcDP, Int bitDepth, Pel* p1, Int iStride1, Pel* p2, Int iStride2, Int iWidth, Int iHeight, Bool bHadamard = false );

  Distortion calcHAD(Int bitDepth, Pel* pi0, Int iStride0, Pel* pi1, Int iStride1, Int iWidth, Int iHeight );

  // for motion cost
#if !FIX203
  Void    initRateDistortionModel( Int iSubPelSearchLimit );
  Void    xUninit();
#endif
  UInt    xGetComponentBits( Int iVal );
#if SCM__R0186_INTRABC_BVD
  UInt    xGetBvdComponentBits( Int iVal,  Int iComponent );
#endif
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  Void    getMotionCost( Bool bSad, Int iAdd, Bool bIsTransquantBypass ) { m_dCost = (bSad ? m_dLambdaMotionSAD[(bIsTransquantBypass && m_costMode==COST_MIXED_LOSSLESS_LOSSY_CODING) ?1:0] + iAdd : m_dLambdaMotionSSE[(bIsTransquantBypass && m_costMode==COST_MIXED_LOSSLESS_LOSSY_CODING)?1:0] + iAdd); }
#else
  Void    getMotionCost( Bool bSad, Int iAdd, Bool bIsTransquantBypass ) { m_uiCost = (bSad ? m_uiLambdaMotionSAD[(bIsTransquantBypass && m_costMode==COST_MIXED_LOSSLESS_LOSSY_CODING) ?1:0] + iAdd : m_uiLambdaMotionSSE[(bIsTransquantBypass && m_costMode==COST_MIXED_LOSSLESS_LOSSY_CODING)?1:0] + iAdd); }
#endif
  Void    setPredictor( TComMv& rcMv )
  {
#if FIX203
    m_mvPredictor = rcMv;
#else
    m_puiHorCost = m_puiComponentCost - rcMv.getHor();
    m_puiVerCost = m_puiComponentCost - rcMv.getVer();
#endif
  }

#if SCM__R0309_INTRABC_BVP
  Void    setPredictors( TComMv* pcMv )
  {
#if FIX203
    for(Int i=0; i<2; i++)
    {
      m_mvPredictors[i] = pcMv[i];
    }
#else
    m_puiHorCost = m_puiComponentCost - rcMv.getHor();
    m_puiVerCost = m_puiComponentCost - rcMv.getVer();
#endif
  }

  __inline Distortion getCostMultiplePreds( Int x, Int y )
  {
    return m_uiCost * getBitsMultiplePreds(x, y) >> 16;
  }

  UInt    getBitsMultiplePreds( Int x, Int y )
  {
    Int rmvH[2];
    Int rmvV[2];
    rmvH[0] = x - m_mvPredictors[0].getHor();
    rmvH[1] = x - m_mvPredictors[1].getHor();

    rmvV[0] = y - m_mvPredictors[0].getVer();
    rmvV[1] = y - m_mvPredictors[1].getVer();

    Int absCand[2];
    absCand[0] = abs(rmvH[0])+abs(rmvV[0]);
    absCand[1] = abs(rmvH[1])+abs(rmvV[1]);


    if(absCand[0] < absCand[1] )
    {
#if SCM__R0186_INTRABC_BVD
      return (xGetBvdComponentBits(rmvH[0],0) + xGetBvdComponentBits(rmvV[0],1) + (1 << 14)) >> 15;
#else
      return getIComponentBits(rmvH[0]) + getIComponentBits(rmvV[0]);
#endif 
    }
    else
    {
#if SCM__R0186_INTRABC_BVD
      return (xGetBvdComponentBits(rmvH[1],0) + xGetBvdComponentBits(rmvV[1],1) + (1 << 14)) >> 15;
#else
      return getIComponentBits(rmvH[1]) + getIComponentBits(rmvV[1]);
#endif 
    }
  }

  UInt getIComponentBits( Int iVal )
  {
    if( !iVal ) return 1;

    UInt uiLength = 1;
    UInt uiTemp   = ( iVal <= 0) ? (-iVal<<1)+1: (iVal<<1);

    while ( 1 != uiTemp )
    {
      uiTemp >>= 1;
      uiLength += 2;
    }

    return uiLength;
  }
#endif

  Void    setCostScale( Int iCostScale )    { m_iCostScale = iCostScale; }
  __inline Distortion getCost( Int x, Int y )
  {
#if RExt__HIGH_BIT_DEPTH_SUPPORT
#if FIX203
    return Distortion((m_dCost * getBits(x, y)) / 65536.0);
#else
    return Distortion(( m_dCost * (m_puiHorCost[ x * (1<<m_iCostScale) ] + m_puiVerCost[ y * (1<<m_iCostScale) ]) ) / 65536.0);
#endif
#else
#if FIX203
    return m_uiCost * getBits(x, y) >> 16;
#else
    return (( m_uiCost * (m_puiHorCost[ x * (1<<m_iCostScale) ] + m_puiVerCost[ y * (1<<m_iCostScale) ]) ) >> 16);
#endif
#endif
  }
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  Distortion getCost( UInt b )                 { return Distortion(( m_dCost * b ) / 65536.0); }
#else
  Distortion getCost( UInt b )                 { return ( m_uiCost * b ) >> 16; }
#endif
  UInt    getBits( Int x, Int y )
  {
#if FIX203
    return xGetComponentBits((x << m_iCostScale) - m_mvPredictor.getHor())
    +      xGetComponentBits((y << m_iCostScale) - m_mvPredictor.getVer());
#else
    return m_puiHorCost[ x * (1<<m_iCostScale)] + m_puiVerCost[ y * (1<<m_iCostScale) ];
#endif
  }

#if SCM__R0186_INTRABC_BVD
__inline Distortion getBvCost( Int x, Int y ) { 
    return m_uiCost * getBvBits(x, y) >> 16;
  } 

  UInt    getBvBits( Int x, Int y )
  {
    return (xGetBvdComponentBits((x << m_iCostScale) - m_mvPredictor.getHor(),0)
      +      xGetBvdComponentBits((y << m_iCostScale) - m_mvPredictor.getVer(),1) + (1 << 14)) >> 15;
  }

  Int*    getMvdBin0CostPtr() { return m_mvdBin0Cost; }
#endif 

private:

  static Distortion xGetSSE           ( DistParam* pcDtParam );
  static Distortion xGetSSE4          ( DistParam* pcDtParam );
  static Distortion xGetSSE8          ( DistParam* pcDtParam );
  static Distortion xGetSSE16         ( DistParam* pcDtParam );
  static Distortion xGetSSE32         ( DistParam* pcDtParam );
  static Distortion xGetSSE64         ( DistParam* pcDtParam );
  static Distortion xGetSSE16N        ( DistParam* pcDtParam );

  static Distortion xGetSAD           ( DistParam* pcDtParam );
  static Distortion xGetSAD4          ( DistParam* pcDtParam );
  static Distortion xGetSAD8          ( DistParam* pcDtParam );
  static Distortion xGetSAD16         ( DistParam* pcDtParam );
  static Distortion xGetSAD32         ( DistParam* pcDtParam );
  static Distortion xGetSAD64         ( DistParam* pcDtParam );
  static Distortion xGetSAD16N        ( DistParam* pcDtParam );

#if AMP_SAD
  static Distortion xGetSAD12         ( DistParam* pcDtParam );
  static Distortion xGetSAD24         ( DistParam* pcDtParam );
  static Distortion xGetSAD48         ( DistParam* pcDtParam );

#endif

  static Distortion xGetHADs          ( DistParam* pcDtParam );
  static Distortion xCalcHADs2x2      ( Pel *piOrg, Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );
  static Distortion xCalcHADs4x4      ( Pel *piOrg, Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );
  static Distortion xCalcHADs8x8      ( Pel *piOrg, Pel *piCurr, Int iStrideOrg, Int iStrideCur, Int iStep );


public:

  Distortion   getDistPart(Int bitDepth, Pel* piCur, Int iCurStride,  Pel* piOrg, Int iOrgStride, UInt uiBlkWidth, UInt uiBlkHeight, const ComponentID compID, DFunc eDFunc = DF_SSE );

#if SCM__R0147_ADAPTIVE_COLOR_TRANSFORM
  Bool      getRGBFormatFlag                  ()                 const { return m_bRGBformat;   } 
  Void      setRGBFormatFlag                  (const Bool value)       { m_bRGBformat = value;  } 
  Bool      getUseColorTrans                  ()                 const { return m_useColorTrans;}
  Void      setUseColorTrans                  (const Bool value)       { m_useColorTrans= value;}
  Bool      getUseLossless                    ()                 const { return m_useLL;}
  Void      setUseLossless                    (const Bool value)       { m_useLL= value;}
  Void      adjustLambdaForColorTrans         (Int delta_QP);
#endif
};// END CLASS DEFINITION TComRdCost

//! \}

#endif // __TCOMRDCOST__
