/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and   contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2010, SAMSUNG ELECTRONICS CO., LTD. and BRITISH BROADCASTING CORPORATION
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of SAMSUNG ELECTRONICS CO., LTD. nor the name of the BRITISH BROADCASTING CORPORATION
      may be used to endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ====================================================================================================================
*/

/** \file     TEncAdaptiveLoopFilter.h
    \brief    estimation part of adaptive loop filter class (header)
*/

#ifndef __TENCADAPTIVELOOPFILTER__
#define __TENCADAPTIVELOOPFILTER__

#include "../TLibCommon/TComAdaptiveLoopFilter.h"
#include "../TLibCommon/TComPic.h"

#include "TEncEntropy.h"
#include "TEncSbac.h"
#include "../TLibCommon/TComBitCounter.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if (WIENER_3_INPUT && !QC_ALF)
class TEncAdaptiveLoopFilter : public TComAdaptiveLoopFilter
{
private:
  TComPic*     m_pcPic;
  TEncEntropy* m_pcEntropyCoder;    
  
  Int       m_iALF_fs_max_rec;
  Int       m_iALF_fs_max_pred;
  Int       m_iALF_fs_max_qpe;
  
  Double    m_dLambdaLuma;
  Double    m_dLambdaChroma;
  Double    m_dLambda;
  
  Pel    *image;
  Double *h;
  Double *rho;
  Double *rho_n;
  Double *rho_RD;  
  Double **a_final;
  Double *r_final;
  Double *h_final;  
  Double **a;
  Double **an;
  Double **a_RD;
  Double **a_1;
  Double *r1;
  Double *h1;
  Double *h2;
  Int  *null;
  Int  *null_orig;
  Int  *coeffs_save;
  Int  *coeffs_best;
  Int  *coeffs_best_precision;
  Int  *dont_care_save;
  
  template <typename T, typename U, typename V>
  void calc_correlation(const Plane<T> &rec, const Plane<U> &pred, const Plane<V> &qpe, Pel *org, Int max_rec, Int max_pred, Int max_qpe, Int width, Int height, Int Stride);
   
  
  void set_lines_and_columns_of_auto_to_zero(int tap, int tap_minus_one_half, int tap_check, int on);  
  void   sgaus  (double **a, double *x, double *b, int n);
  double get_mse(Pel *image1, Pel *image2, Int width, Int height, Int stride1, Int stride2);
    
  UInt64 get_rate(ALFParam* pAlfParam, Int component);
  Void   set_golomb_parameter(ALFParam* pAlfParam, Int component, Int max_k);
  
  Int  get_mem1Ddouble(Double **array1D, Int rows);  
  Int  get_mem2Ddouble(Double ***array2D, Int rows, Int columns);
  Void free_mem1Ddouble(Double *array1D);  
  Void free_mem2Ddouble(Double **array2D);    
  
public:
  TEncAdaptiveLoopFilter ();
  virtual ~TEncAdaptiveLoopFilter () {}

        /// allocate temporal memory
  Void startALFEnc(TComPic* pcPic, TEncEntropy* pcEntropyCoder);

        /// destroy temporal memory
  Void endALFEnc();

        /// estimate ALF parameters
  Void ALFProcess(ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth );
  
  Double estimate_filter(ALFParam* pcAlfParam, Pel* dY, Pel* xY, Pel* pY, Pel* qpe, Int component, Int height_f, Int width_f, Int Stride);
  
  Double filter_d(ALFParam* pcAlfParam, Double *rounding, const Pel *org, const Plane<Pel> &xY, Pel *oY, const Plane<Pel> &pY, const Plane<Int> &qpe, Int width, Int height, Int Stride_org, Int Stride_out, Int component);
};

#else //WIENER_3_INPUT




#if HHI_ALF
/// estimation part of adaptive loop filter class
class TEncAdaptiveLoopFilter : public TComAdaptiveLoopFilter
{
private:

  Double**          m_ppdAlfCorr;
  Double*           m_pdDoubleAlfCoeff;

  UInt****          m_puiCUHorizontalCorr;
  UInt****          m_puiCUVerticalCorr;

  SliceType         m_eSliceType;
  Int               m_iPicNalReferenceIdc;

  Double            m_dLambdaLuma;
  Double            m_dLambdaChroma;

  TEncEntropy*      m_pcEntropyCoder;
  TEncSbac***       m_pppcRDSbacCoder;
  TEncSbac*         m_pcRDGoOnSbacCoder;
  TComBitIf*        m_pcCoderFixedBits;
  Bool              m_bUseSBACRD;

  TComPic*          m_pcPic;
  ALFParam*         m_pcBestAlfParam;
  ALFParam*         m_pcTempAlfParam;

  TComPicYuv*       m_pcPicYuvBest;
  TComPicYuv*       m_pcPicYuvTmp;
  TComPicYuv*       m_pcPicYuvFiltered;

  UInt64            m_uiMinRate;
  UInt64            m_uiMinDist;
  Double            m_dMinCost;

  UInt              m_uiNumSCUInCU;
  UInt              m_uiSCUWidth;
  UInt              m_uiSCUHeight;

  //parameters
  Bool              m_bALFSeparateQt;
  Bool              m_bALFSymmetry;
  Int               m_iALFMinLength;
  Int               m_iALFMaxLength;


private:
  // init / uninit internal variables
  Void xInitParam      ();
  Void xUninitParam    ();

  // create/destroy/copy/set functions of ALF control flags
  Void xCreateTmpAlfCtrlFlags   ( TComPicSym* pcQuadTree );
  Void xDestroyTmpAlfCtrlFlags  ( TComPicSym* pcQuadTree );
  Void xCopyTmpAlfCtrlFlagsTo   ( TComPicSym* pcQuadTree );
  Void xCopyTmpAlfCtrlFlagsFrom ( TComPicSym* pcQuadTree );
  Void xSetCUAlfCtrlFlags       ( UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest,TComPicSym* pcQuadTree, UInt64& ruiDist, UInt64& ruiRate );
  Void xSetCUAlfCtrlFlag        ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist );
  Void xSetCUAlfCtrlFlag        ( TComPicSym* pcQuadTree,TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiResultDist,  UInt64* auiFixedBitsCurrBest, UInt64* auiFixedBitsNextBest );

  // functions related to correlation computation
  Void xReadOrCalcCorrFromCUs           ( TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr );
  Void xReadOrCalcCorrFromFUs           ( TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr );
  Void xReadOrCalcCorrFromFU            ( TComPicSym* pcQuadTree, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr );
  Void xCalcALFCoeff                    ( AlfFilter& rcFilter );
  Void xCalcCorrelationFunc             ( Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);
  Void xCalcCorrelationFuncBlock        ( Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);
  Void xCalcStoredCorrelationFuncBlock  ( Pel* pOrg, Pel* pCmp, UInt** ppuiCorr, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);

  Void xEstimateCorr                    ( Pel* pOrig, Pel* pDec, Int iWidth, Int iHeight, Int iOrigStride , Int iDecStride, AlfFilter& rcFilter , double** ppdCorr , Int iOffsetX , Int iOffsetY, Int iMaxX , Int iMaxY );
  Void xEstimateCorr                    ( Pel* pOrig, Pel* pDec, Int iWidth, Int iHeight, Int iOrigStride , Int iDecStride, AlfFilter& rcFilter , UInt** ppuiCorr , Int iOffsetX , Int iOffsetY, Int iMaxX , Int iMaxY );
  Void xEstimateCorrCU                  ( TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiSCUIndx, UChar uhDepth, AlfFilter& rcFilter, double** pdCUCorr  );
  Void xEstimateCorrCU                  ( TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiSCUIndx, UChar uhDepth, AlfFilter& rcFilter, UInt** puiCUCorr  );
  Void xEstimateCorrFU                  (TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, AlfFilter& rcFilter, double** pdCUCorr  );

  // functions related to filtering
  Void xFilterCoefQuickSort     ( Double *coef_data, Int *coef_num, Int upper, Int lower );
  Void xQuantFilterCoef         ( Double* adDoubleCoeffs, AlfFilter& rcFilter, Int iBit_depth );
  Void xClearFilterCoefInt      (  AlfFilter& rcFilter );

  Void xReDesignFilterCoeff     ( TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, AlfFilter& rcFilter, UInt**** ppuiAlfCorr, Bool bReadCorr);
  Void xFilteringFrameLuma      ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Bool bStoreCorr );
  Void xFilteringFrameChroma    ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );

  Void xEstimateFrameFilterLuma ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, AlfFilter& rcFilter , Bool bStoreCorr , UInt**** puiCUCorr );
  Void xEstimateFrameFilter     ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, AlfFilter& rcFilter , Bool bStoreCorr , UInt**** puiCUCorr, Int iPlane ) ;

  Void  xPredictALFCoeff        ( ALFParam* pAlfParam) ;
  Void  xPredictALFCoeff        ( ALFParam* pAlfParam, Int iPlane) ;

  Void xCheckFilterReuse        (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, ALFParam* pcFilterParam, Double& rcMinCost, Int iPlane) ;
  // distortion / misc functions
  UInt64 xCalcSSD               ( Pel* pOrg, Pel* pCmp, Int iWidth, Int iHeight, Int iStride );
  Void   xCalcRDCost            ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost, Int iPlane );
  Void   xCalcRDCostChroma      ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  Void   xCalcRDCost            ( ALFParam* pAlfParam, UInt64& ruiRate, UInt64 uiDist, Double& rdCost );
  Int    xGauss                 ( Double **a, Int N );

protected:
  /// test ALF for luma
  Void xEncALFFullFrameLuma     ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );
  Void xEncALFFullFrame         ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int iPlane );
  /// test CU-based partition
  Void xCUAdaptiveControl       ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );
  Void xCUAdaptiveControl       ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicFiltered, ALFParam* pcAlfParam, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost ) ;
  /// test various filter taps
  Void xFilterTapDecision       ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int iPlane );
  Void xFilterTapDecision       (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicFiltered, ALFParam* pcAlfParams, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int iPlane) ;

  /// do ALF for chroma
  Void xEncALFChroma            ( UInt64 uiLumaRate, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, UInt64& ruiBits );
public:
  TEncAdaptiveLoopFilter          ();
  virtual ~TEncAdaptiveLoopFilter () {}

  /// allocate temporal memory
  Void startALFEnc(TComPic* pcPic, TEncEntropy* pcEntropyCoder , TEncSbac*** pppcRDSbacCoder,  TEncSbac* pcRDGoOnSbacCoder);

  /// destroy temporal memory
  Void endALFEnc();

  /// estimate ALF parameters
  Void ALFProcess(ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth );


  // encode ALF QuadTree
  Void encodeQuadTree          ( ALFParam*    pAlfParam     , TEncEntropy* pcEntropyCoder, UInt uiMaxAlfCtrlDepth );
  Void encodeFUAlfCtrlFlags    ( TEncEntropy* pcEntropyCoder, TComPicSym*  pcQuadTree    , UInt uiMaxDepth        );
  Void encodeFUAlfCtrlFlag     ( TEncEntropy* pcEntropyCoder, TComPicSym*  pcQuadTree    , TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth );

};
#else
/// estimation part of adaptive loop filter class
class TEncAdaptiveLoopFilter : public TComAdaptiveLoopFilter
{
private:
  static const Int	m_aiSymmetricArray9x9[81];					///< scan index for 9x9 filter
  static const Int	m_aiSymmetricArray7x7[49];					///< scan index for 7x7 filter
  static const Int	m_aiSymmetricArray5x5[25];					///< scan index for 5x5 filter

  Double**					m_ppdAlfCorr;
  Double*						m_pdDoubleAlfCoeff;
  UInt****					m_puiCUCorr;

  SliceType					m_eSliceType;
  Int								m_iPicNalReferenceIdc;

  Double						m_dLambdaLuma;
  Double						m_dLambdaChroma;

  TEncEntropy*			m_pcEntropyCoder;

  TComPic*					m_pcPic;
  ALFParam*					m_pcBestAlfParam;
  ALFParam*					m_pcTempAlfParam;

  TComPicYuv*				m_pcPicYuvBest;
  TComPicYuv*				m_pcPicYuvTmp;

  UInt							m_uiNumSCUInCU;
  UInt							m_uiSCUWidth;
  UInt							m_uiSCUHeight;
#if QC_ALF
#if WIENER_3_INPUT
  imgpel		**imgY_pred;
  imgpel		**imgY_resi;
  imgpel		**imgY_pext;
  imgpel		**imgY_rext;
#endif
  imgpel		**imgY_rest;
  imgpel		**imgY_ext;
  imgpel		**imgY_temp;
  imgpel		**imgY_org;          
  imgpel		**imgY_rec;
  
  imgpel                    **varImg;
  imgpel					**maskImg;
  Int						varIndTab[NO_VAR_BINS];
  double		***yGlobalSym;
  double		****EGlobalSym;
  double	        *pixAcc;
  Int			**g_filterCoeffSymQuant;
  Int			**g_filterCoeffSym;
  Int			**g_filterCoeffPrevSelected;
  Int   		im_width;
  Int			im_height;
  ALFParam		*ALFp;
  ALFParam		*tempALFp;
  TEncEntropy*	        m_pcDummyEntropyCoder;
#if (WIENER_3_INPUT && WIENER_3_INPUT_FAST)
  Int                   adapt_precision;
#endif  
#endif

private:
	// init / uninit internal variables
  Void xInitParam 	   ();
  Void xUninitParam	   ();

	// create/destroy/copy/set functions of ALF control flags
  Void xCreateTmpAlfCtrlFlags		();
  Void xDestroyTmpAlfCtrlFlags	();
  Void xCopyTmpAlfCtrlFlagsTo		();
  Void xCopyTmpAlfCtrlFlagsFrom	();
  Void xSetCUAlfCtrlFlags				( UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist );
  Void xSetCUAlfCtrlFlag				( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist );

	// encoder ALF control flags
	Void xEncodeCUAlfCtrlFlags	();
  Void xEncodeCUAlfCtrlFlag		( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth);

	// functions related to correlation computation
	Void xReadOrCalcCorrFromCUs						( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr );
  Void xCalcALFCoeff										( ALFParam* pAlfParam );
  Void xCalcCorrelationFunc							( Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);
  Void xCalcCorrelationFuncBlock				( Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);
  Void xCalcStoredCorrelationFuncBlock	( Pel* pOrg, Pel* pCmp, UInt** ppuiCorr, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride);

	// functions related to filtering
	Void xFilterCoefQuickSort		( Double *coef_data, Int *coef_num, Int upper, Int lower );
  Void xQuantFilterCoef				( Double* h, Int* qh, Int tap, int bit_depth );
  Void xClearFilterCoefInt		( Int* qh, Int N );
  Void xReDesignFilterCoeff		( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr );
  Void xCopyDecToRestCUs			( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void xCopyDecToRestCU				( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void xFilteringFrameLuma		( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Bool bStoreCorr );
  Void xFilteringFrameChroma	( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );

	// distortion / misc functions
  UInt64 xCalcSSD							( Pel* pOrg, Pel* pCmp, Int iWidth, Int iHeight, Int iStride );
  Void	 xCalcRDCost					( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  Void   xCalcRDCostChroma		( TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost );
  Void   xCalcRDCost					( ALFParam* pAlfParam, UInt64& ruiRate, UInt64 uiDist, Double& rdCost );
  Int		 xGauss								( Double **a, Int N );
#if WIENER_3_INPUT
  Void   sgaus  (double **a, double *x, double *b, int n);  
  Void   xCalcRDCost_precision     (imgpel** ImgOrg, imgpel** imgCmp, Int width, Int height, ALFParam* pAlfParam, Int64& iRate, Int64& iDist, Double& rdCost);
#endif
protected:
	/// test ALF for luma
  Void xEncALFLuma						( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );

	/// test CU-based partition
  Void xCUAdaptiveControl			( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost );


        /// test various filter taps
  Void xFilterTapDecision                       ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost);

	/// do ALF for chroma
  Void xEncALFChroma ( UInt64 uiLumaRate, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, UInt64& ruiBits );
public:
  TEncAdaptiveLoopFilter					();
	virtual ~TEncAdaptiveLoopFilter	() {}
	/// allocate temporal memory
  Void startALFEnc(TComPic* pcPic, TEncEntropy* pcEntropyCoder);

	/// destroy temporal memory
  Void endALFEnc();

	/// estimate ALF parameters
  Void ALFProcess(ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth );
#if QC_ALF
	/// test ALF for luma
  Void xSetCUAlfCtrlFlags_qc            (UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, 
	UInt64& ruiDist);
  Void xSetCUAlfCtrlFlag_qc             (TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg,
	TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist);
  Void xdecideCoeffForce0(int codedVarBins[NO_VAR_BINS], double errorForce0Coeff[], double errorForce0CoeffTab[NO_VAR_BINS][2], 
	int bitsVarBin[NO_VAR_BINS], double lambda, int filters_per_fr);
#if WIENER_3_INPUT
  Void xReDesignFilterCoeff_qc (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec,  TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, Bool bReadCorr);
  Void xcodeFiltCoeff(Int **filterCoeffSymQuant, Int filtNo, int filtNo_pred, int filtNo_resi, Int varIndTab[], Int filters_per_fr_best, Int frNo, ALFParam* ALFp);
  Void xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, Int **filterCoeffSym, Int **filterCoeffSymQuant,
        Int filtNo, int filtNo_pred, int filtNo_resi, Int *filters_per_fr_best, Int varIndTab[], imgpel **imgY_rec, imgpel **varImg, 
        imgpel **maskImg, imgpel **imgY_pad, double lambda_val, int prec_rec, int prec_pred, int prec_resi, int shift_rec, int shift_pred, int shift_resi);
  Void xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int filters_per_group, 
        int bitsVarBin[]);
  double xfindBestCoeffCodMethod(int codedVarBins[NO_VAR_BINS], int *forceCoeff0, int prec_rec, int prec_pred, int prec_resi,
                              int **filterCoeffSymQuant, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi,
                              int filters_per_fr, double errorForce0CoeffTab[NO_VAR_BINS][2], 
                              double *errorQuant, double lambda);
  Void xFilterTapDecision_qc            (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost);
  Void xCUAdaptiveControl_qc                    ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost );
  Void xcalcPredFilterCoeff(Int filtNo, int filtNo_pred, int filtNo_resi);
  Void xfilterFrame_en(imgpel** ImgDec, imgpel** ImgRest, imgpel** ImgResi, imgpel** ImgPred, int filtNo, int filtNo_resi, int filtNo_pred, int bits_enc, int bits_pred, int bits_resi);
  Void xstoreInBlockMatrix (imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgResi, imgpel** ImgPred, Int tap, Int tap_pred, Int tap_resi);
  Void xFilteringFrameLuma_qc (imgpel** ImgOrg, imgpel** imgY_pad, imgpel** ImgFilt, imgpel** ImgResi, imgpel** ImgPred, ALFParam* ALFp, Int tap);
  Void xFirstFilteringFrameLuma(imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgRest, imgpel** ImgResi, imgpel** ImgPred, ALFParam* ALFp, Int tap);
  Void xEncALFLuma_qc ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost );
  Int xsendAllFiltersPPPredForce0(int **FilterCoeffQuant, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int prec_rec, int prec_pred, int prec_resi, int filters_per_group, 
                               int codedVarBins[NO_VAR_BINS], int createBistream, ALFParam* ALFp);
  Int xsendAllFiltersPPPred(int **FilterCoeffQuant, int fl, int sqrFiltLength,  int sqrFiltLength_pred, int sqrFiltLength_resi, int prec_rec, int prec_pred, int prec_resi,
                         int filters_per_group, int createBistream, ALFParam* ALFp);
#else  
  Void xReDesignFilterCoeff_qc          (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec,  TComPicYuv* pcPicRest, Bool bReadCorr);
  Void xcodeFiltCoeff(Int **filterCoeffSymQuant, Int filtNo, Int varIndTab[], Int filters_per_fr_best, Int frNo, ALFParam* ALFp);
  Void xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, Int **filterCoeffSym, Int **filterCoeffSymQuant,
        Int filtNo, Int *filters_per_fr_best, Int varIndTab[], imgpel **imgY_rec, imgpel **varImg, 
        imgpel **maskImg, imgpel **imgY_pad, double lambda_val);
  Void xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int filters_per_group, 
        int bitsVarBin[]);
  double xfindBestCoeffCodMethod(int codedVarBins[NO_VAR_BINS], int *forceCoeff0, 
                              int **filterCoeffSymQuant, int fl, int sqrFiltLength, 
                              int filters_per_fr, double errorForce0CoeffTab[NO_VAR_BINS][2], 
                              double *errorQuant, double lambda);
  Void xFilterTapDecision_qc            (TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost);
  Void xCUAdaptiveControl_qc                    ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost );
  Void xcalcPredFilterCoeff(Int filtNo);
  Void xfilterFrame_en(imgpel** ImgDec, imgpel** ImgRest, Int filtNo);
  Void xstoreInBlockMatrix              (imgpel** ImgOrg, imgpel** ImgDec, Int tap);
  Void xFilteringFrameLuma_qc            (imgpel** ImgOrg, imgpel** imgY_pad, imgpel** ImgFilt, ALFParam* ALFp, Int tap);
  Void xFirstFilteringFrameLuma         (imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgRest, ALFParam* ALFp, Int tap);
  Void xEncALFLuma_qc                                   ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, 
        UInt64& ruiMinDist, Double& rdMinCost );
  Int xsendAllFiltersPPPredForce0(int **FilterCoeffQuant, int fl, int sqrFiltLength, int filters_per_group, 
                               int codedVarBins[NO_VAR_BINS], int createBistream, ALFParam* ALFp);
  Int xsendAllFiltersPPPred(int **FilterCoeffQuant, int fl, int sqrFiltLength, 
                         int filters_per_group, int createBistream, ALFParam* ALFp);
#endif
  Int xcodeAuxInfo(int filtNo, int noFilters, int varIndTab[NO_VAR_BINS], int frNo, int createBitstream,int realfiltNo, ALFParam* ALFp);
  Int xcodeFilterCoeff(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int filters_per_group, int createBitstream);
  Int lengthGolomb(int coeffVal, int k);
  Int lengthPredFlags(int force0, int predMethod, int codedVarBins[NO_VAR_BINS], 
                   int filters_per_group, int createBitstream);
  Int lengthFilterCoeffs(int sqrFiltLength, int filters_per_group, int pDepthInt[], 
                      int **FilterCoeff, int kMinTab[], int createBitstream);
//cholesky related
#if WIENER_3_INPUT      
  Double findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq,
        int **filterCoeffQuantSeq, int intervalBest[NO_VAR_BINS][2], int varIndTab[NO_VAR_BINS], int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, 
        int filters_per_fr, int *weights, int shift_rec, int shift_pred, int shift_resi, double errorTabForce0Coeff[NO_VAR_BINS][2]);
  Double QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, 
        int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int *weights, int shift_rec, int shift_pred, int shift_resi);
  Void roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int factor_rec, int factor_pred, int factor_resi);
#else  
  Double findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq,
        int **filterCoeffQuantSeq, int intervalBest[NO_VAR_BINS][2], int varIndTab[NO_VAR_BINS], int sqrFiltLength, 
        int filters_per_fr, int *weights, int bit_depth, double errorTabForce0Coeff[NO_VAR_BINS][2]);
  Double QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, 
        int sqrFiltLength, int *weights, int bit_depth);
  Void roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor);
#endif
  double findFilterGroupingError(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, 
	int intervalBest[NO_VAR_BINS][2], int sqrFiltLength, int filters_per_fr);
  double mergeFiltersGreedy(double **yGlobalSeq, double ***EGlobalSeq, double *pixAccGlobalSeq, 
	int intervalBest[NO_VAR_BINS][2], int sqrFiltLength, int noIntervals);
  double calculateErrorAbs(double **A, double *b, double y, int size);
  double calculateErrorCoeffProvidedInt(double **A, double *b, int *c, int size,int *pattern);
  double calculateErrorCoeffProvided(double **A, double *b, double *c, int size);
  double add_pixAcc(double *pixAcc, int start, int stop);
  Void add_b(double *bmerged, double **b, int start, int stop, int size);
  Void add_A(double **Amerged, double ***A, int start, int stop, int size);
  Int gnsSolveByChol(double **LHS, double *rhs, double *x, int noEq);
  Void  gnsBacksubstitution(double R[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], double z[MAX_SQR_FILT_LENGTH], 
	int R_size, double A[MAX_SQR_FILT_LENGTH]);
  Void gnsTransposeBacksubstitution(double U[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], double rhs[], double x[],
	int order);
  Int gnsCholeskyDec(double **inpMatr, double outMatr[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], int noEq);
#endif
};
#endif
#endif



#endif //WIENER_3_INPUT

