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

/** \file     TComAdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __TCOMADAPTIVELOOPFILTER__
#define __TCOMADAPTIVELOOPFILTER__

#include "TComPic.h"



#if (WIENER_3_INPUT && !QC_ALF)
//-----------------------------------------------------------------------------
/*!
 * \brief Plane class
 *
 * A template class to hold 2D data of any type
 */
//-----------------------------------------------------------------------------
template <typename T>
class Plane
{
  T *m_data;         //!< Pointer to allocated memory
  T *m_origin;       //!< Pointer to top-left sample
  int m_width;       //!< Width of plane
  int m_height;      //!< Height of plane
  int m_pitch;       //!< Data pitch
  int m_border_size; //!< Border size

public:
  Plane();
  Plane(int width, int height, int border_size);
  virtual ~Plane();

  void set_size(int width, int height, int border_size);

  int get_width() const { return m_width; }   //!< Get width
  int get_height() const { return m_height; } //!< Get height
  int get_pitch() const { return m_pitch; }   //!< Get pitch

  const T *operator[](int row) const { return m_origin + row * m_pitch; } //!< Get pointer to a given row
  T *operator[](int row) { return m_origin + row * m_pitch; }             //!< Get pointer to a given row

  void set(int row, int col, T val, int numRows, int numCols);

  void mirror();
};





class TComAdaptiveLoopFilter
{
protected:
  Int  get_mem1Dint(Int **array1D, Int rows);
  Int  get_mem2Dint(Int ***array2D, Int rows, Int columns);
  Void free_mem1Dint(Int *array1D);
  Void free_mem2Dint(Int **array2D);
  Void no_mem_exit(const char *where);  
public:
  TComAdaptiveLoopFilter();
  virtual ~TComAdaptiveLoopFilter() {}
  
  Void  allocALFParam           ( ALFParam* pAlfParam );
  Void  freeALFParam            ( ALFParam* pAlfParam );
  Void  copyALFParam            ( ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam );
  Void  destroy                 ();
  Void  create                  ( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth );


  // interface function
  Void  ALFProcess              ( TComPic* pcPic, ALFParam* pcAlfParam ); ///< interface function for ALF process
  
  Void  filter                  (ALFParam* pAlfParam, const Plane<Pel> &xY, Pel *oY, const Plane<Pel> &pY, const Plane<Int> &qpe, Int width, Int height, Int Stride, Int component);
  Void  filter                  (ALFParam* pAlfParam, Pel *in, Pel *out, Pel *pred, Pel *qpe, Int component, Int height, Int width, Int Stride_in, Int Stride_out);
};

#else //WIENER_3_INPUT


// ====================================================================================================================
// Constants
// ====================================================================================================================

#if HHI_ALF
#define ALF_MAX_NUM_TAP       12                                         ///< maximum number of filter taps (9x9)
#define ALF_MAX_VERT_LENGTH   9
#define ALF_MAX_HORIZ_LENGTH  9
#define ALF_FILT_FOR_CHROMA   2                                         ///< number of filter used for chroma planes
#define ALF_MAX_NUM_COEF      12                                         ///< maximum number of filter coefficients
#define ALF_NUM_BIT_SHIFT     8                                         ///< bit shift parameter for quantization of ALF param.
#else
#if WIENER_3_INPUT
  #define ALF_MIN_NUM_TAP_PQ     1                                    ///< minimum number of filter taps (1x1)
  #define ALF_MAX_NUM_COEF     126                                    ///< maximum number of filter coefficients
#else
  #define ALF_MAX_NUM_COEF      42                                    ///< maximum number of filter coefficients
#endif
#define ALF_MAX_NUM_TAP       9                                       ///< maximum number of filter taps (9x9)
#define ALF_MIN_NUM_TAP       5                                       ///< minimum number of filter taps (5x5)
#define ALF_MAX_NUM_TAP_C     5                                       ///< number of filter taps for chroma (5x5)
#define ALF_MIN_NUM_COEF      14                                      ///< minimum number of filter coefficients
#define ALF_MAX_NUM_COEF_C    14                                      ///< number of filter taps for chroma
#define ALF_NUM_BIT_SHIFT     8                                       ///< bit shift parameter for quantization of ALF param.
#define ALF_ROUND_OFFSET      ( 1 << ( ALF_NUM_BIT_SHIFT - 1 ) )      ///< rounding offset for ALF quantization

#if QC_ALF
#include "../TLibCommon/CommonDef.h"

#define FATAL_ERROR_0(MESSAGE, EXITCODE)                                \
          {                                                             \
            printf(MESSAGE);                                            \
            exit(EXITCODE);                                             \
          }

#define NUM_BITS               9
#define NO_TEST_FILT           3       // Filter supports (5/7/9)
#define NO_VAR_BINS           16 
#define NO_FILTERS            16
#define VAR_SIZE               3
#define FILTER_LENGTH          9

#if WIENER_3_INPUT
  #define FILTER_LENGTH_PRED     9
  #define FILTER_LENGTH_RESI     9
  #define MAX_SQR_FILT_LENGTH   ((FILTER_LENGTH*FILTER_LENGTH) / 2 + 2) + ((FILTER_LENGTH_PRED*FILTER_LENGTH_PRED) / 2 + 1) + ((FILTER_LENGTH_RESI*FILTER_LENGTH_RESI) / 2 + 1)
  #define SQR_FILT_LENGTH_3SYM  ((3*3) / 4 + 1)
  #define SQR_FILT_LENGTH_1SYM  ((1*1) / 4 + 1)
  #define SQR_FILT_LENGTH_3     ((3*3) / 2 + 1)
  #define SQR_FILT_LENGTH_1     ((1*1) / 2 + 1)
  #define SQR_FILT_LENGTH_9     ((9*9) / 2 + 1)
  #define SQR_FILT_LENGTH_7     ((7*7) / 2 + 1)
  #define SQR_FILT_LENGTH_5     ((5*5) / 2 + 1)
#else
  #define MAX_SQR_FILT_LENGTH   ((FILTER_LENGTH*FILTER_LENGTH) / 2 + 2)
#endif

#define SQR_FILT_LENGTH_9SYM  ((9*9) / 4 + 2) 
#define SQR_FILT_LENGTH_7SYM  ((7*7) / 4 + 2) 
#define SQR_FILT_LENGTH_5SYM  ((5*5) / 4 + 2) 
#define MAX_SCAN_VAL    11
#define MAX_EXP_GOLOMB  16

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define imgpel  unsigned short

#if WIENER_3_INPUT
extern Int *pDepthIntTab_pr[NO_TEST_FILT+2];
extern Int weights[MAX_SQR_FILT_LENGTH];
extern Int depth[MAX_SQR_FILT_LENGTH];
#endif

extern Int depthInt9x9Sym[22];
extern Int depthInt7x7Sym[14];
extern Int depthInt5x5Sym[8];
extern Int *pDepthIntTab[NO_TEST_FILT];
void destroyMatrix_int(int **m2D);
void initMatrix_int(int ***m2D, int d1, int d2);
#endif
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if HHI_ALF
/// adaptive loop filter class
class TComAdaptiveLoopFilter
{
protected:

  // temporary picture buffer
  TComPicYuv*   m_pcTempPicYuv;                                         ///< temporary picture buffer for ALF processing


  // ------------------------------------------------------------------------------------------------------------------
  // For luma component
  // ------------------------------------------------------------------------------------------------------------------

  /// ALF for luma component
  Void  xALFLuma          ( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicRest );
  Void  xALFFilter        ( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicRest, Int iPlane);

  /// sub function: CU-adaptive ALF process for luma
  Void  xCUAdaptive       ( TComPicSym*  pcPicSym, AlfFilter& rcFilter, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void  xSubCUAdaptive    ( TComPicSym* pcQuadTree, TComDataCU*  pcCU    , AlfFilter& rcFilter, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt uiAbsPartIdx, UInt uiDepth);
  Void  xCopyDecToRestCUs ( TComPicSym*  pcQuadTree, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest);
  Void  xCopyDecToRestCU  ( TComPicSym* pcQuadTree, TComDataCU*  pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest);
  /// sub function: non-adaptive ALF process for luma
  Void  xApplyFrame       ( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, AlfFilter& rcFilter );
  Void  xApplyFrame       ( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, AlfFilter& rcFilter, Int iPlane );

  Void  xApplyFilter      ( Pel* pDec, Pel* pRest, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride, AlfFilter& rcFilter );
  Void  xApplyFilter      ( Pel* pDec, Pel* pRest, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride, AlfFilter& rcFilter,Int iOffsetX, Int iOffsetY, Int iMaxX, Int iMaxY );
  // ------------------------------------------------------------------------------------------------------------------
  // For chroma component
  // ------------------------------------------------------------------------------------------------------------------

  /// ALF for chroma component
  Void  xALFChroma      ( ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );

  /// sub function: non-adaptive ALF process for chroma
  Void  xFrameChroma    ( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, ALFParam* pcAlfParam , Int iColor );

  Void xFillAlfFilterInitParam ( AlfFilter& rcFilter , Int iLength , Int iSymmetry );

public:
  TComAdaptiveLoopFilter();
  virtual ~TComAdaptiveLoopFilter() {}


  // initialize & destory temporary buffer
  Void  create                  ( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth );
  Void  destroy                 ();

  // alloc & free & set functions
  Void  allocALFParam           ( ALFParam* pAlfParam );
  Void  freeALFParam            ( ALFParam* pAlfParam );
  Void  copyALFParam            ( ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam );
  Void  destroyQuadTree         ( ALFParam* pcAlfParam );
  Void  copyALFFilter           ( AlfFilter& rDesAlfFilter, AlfFilter& rSrcAlfFilter) ;

  // predict filter coefficients
  Void  xPredictALFCoeff        ( ALFParam* pAlfParam) ;
  Void  predictALFCoeff         ( ALFParam* pAlfParam );                  ///< prediction of luma ALF coefficients
  Void  predictALFCoeffChroma   ( ALFParam* pAlfParam );                  ///< prediction of chroma ALF coefficients

  // interface function
  Void  ALFProcess              ( TComPic* pcPic, ALFParam* pcAlfParam ); ///< interface function for ALF process
};
#else
/// adaptive loop filter class
class TComAdaptiveLoopFilter
{
protected:
  // quantized filter coefficients
  static const	Int m_aiSymmetricMag9x9[41];														///< quantization scaling factor for 9x9 filter
  static const	Int m_aiSymmetricMag7x7[25];														///< quantization scaling factor for 7x7 filter
  static const	Int m_aiSymmetricMag5x5[13];														///< quantization scaling factor for 5x5 filter

	// temporary picture buffer
	TComPicYuv*		m_pcTempPicYuv;																					///< temporary picture buffer for ALF processing
#if (WIENER_3_INPUT && ALF_MEM_PATCH)
  TComPicYuv* m_pcTempPicPredYuv;
  TComPicYuv* m_pcTempPicResiYuv;   
#endif  
	// ------------------------------------------------------------------------------------------------------------------
	// For luma component
	// ------------------------------------------------------------------------------------------------------------------
#if QC_ALF
#if WIENER_3_INPUT 
  Int  get_mem2Dint(Int ***array2D, Int rows, Int columns);
  Void free_mem2Dint(Int **array2D);
   
  static Int pattern9x9Sym[SQR_FILT_LENGTH_9];
  static Int weights9x9Sym[SQR_FILT_LENGTH_9SYM];
  static Int pattern9x9Sym_Quart[45];
  static Int pattern7x7Sym[SQR_FILT_LENGTH_7];
  static Int weights7x7Sym[SQR_FILT_LENGTH_7SYM];
  static Int pattern7x7Sym_Quart[45];
  static Int pattern5x5Sym[SQR_FILT_LENGTH_5];
  static Int weights5x5Sym[SQR_FILT_LENGTH_5SYM];
  static Int pattern5x5Sym_Quart[45];
  static Int pattern9x9Sym_9[SQR_FILT_LENGTH_9];
  static Int pattern9x9Sym_7[SQR_FILT_LENGTH_7];
  static Int pattern9x9Sym_5[SQR_FILT_LENGTH_5];
  static Int pattern3x3Sym_Quart[6];
  static Int pattern1x1Sym_Quart[6];
  static Int pattern9x9Sym_3[SQR_FILT_LENGTH_3];
  static Int pattern9x9Sym_1[SQR_FILT_LENGTH_1];
  static Int pattern1x1Sym[SQR_FILT_LENGTH_1];
  static Int pattern3x3Sym[SQR_FILT_LENGTH_3];
  static Int weights3x3Sym[SQR_FILT_LENGTH_3SYM];
  static Int weights1x1Sym[SQR_FILT_LENGTH_1SYM];
  static Int *patternTab_pr[NO_TEST_FILT+2];
  static Int sqrFiltLengthTab_pr[NO_TEST_FILT+2];
  static Int *weightsTab_pr[NO_TEST_FILT+2];
  static Int *patternTab_filt_pr[NO_TEST_FILT+2];
  static Int *patternMapTab_pr[NO_TEST_FILT+2];
#else
  static Int pattern9x9Sym[41];
  static Int weights9x9Sym[22];
  static Int pattern9x9Sym_Quart[42];
  static Int pattern7x7Sym[25];
  static Int weights7x7Sym[14];
  static Int pattern7x7Sym_Quart[42];
  static Int pattern5x5Sym[13];
  static Int weights5x5Sym[8];
  static Int pattern5x5Sym_Quart[45];
  static Int pattern9x9Sym_9[41];
  static Int pattern9x9Sym_7[25];
  static Int pattern9x9Sym_5[13];
#endif
  static Int *patternTab_filt[NO_TEST_FILT];
  static Int flTab[NO_TEST_FILT];
  static Int *patternTab[NO_TEST_FILT]; 
  static Int *patternMapTab[NO_TEST_FILT];
  static Int *weightsTab[NO_TEST_FILT];
  static Int sqrFiltLengthTab[NO_TEST_FILT];

  Int img_height,img_width;

#if !ALF_MEM_PATCH
  imgpel **ImgDec;
  imgpel **ImgRest;
#if WIENER_3_INPUT
  imgpel **ImgResi;
  imgpel **ImgPred;
  imgpel **imgYr_pad;
  imgpel **imgYp_pad;
#endif  
#endif
  imgpel **imgY_pad;
  imgpel **imgY_var;
  Int    **imgY_temp;
  Int **filterCoeffSym;
  Int **filterCoeffPrevSelected;
  Int **filterCoeffTmp;
  Int **filterCoeffSymTmp;

	/// ALF for luma component
  Void	xALFLuma_qc				( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void  xCUAdaptive_qc                  ( TComPic*        pcPic, ALFParam* pcAlfParam, imgpel **imgY_rec_post, imgpel **imgY_rec );
  Void	xSubCUAdaptive_qc	( TComDataCU* pcCU,  ALFParam* pcAlfParam, imgpel **imgY_rec_post, imgpel **imgY_rec,
				    UInt uiAbsPartIdx, UInt uiDepth );
#if WIENER_3_INPUT
  Void	xALFLuma_qc ( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi );
  Void  filterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int filtNo_resi, int filtNo_pred, int bits_rec, int bits_pred, int bits_resi);
  Void  subfilterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int filtNo_resi, int filtNo_pred, int start_height, int end_height, int start_width, int end_width, int bits_rec, int bits_pred, int bits_resi);
#endif  
  Void calcVar(imgpel **imgY_var, imgpel **imgY_pad,int pad_size, int fl, int img_height, int img_width);
  Void  filterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec,int filtNo);
  Void  subfilterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int start_height, int end_height, int start_width, int end_width);

  Void  DecFilter_qc(imgpel** imgY_rec,ALFParam* pcAlfParam);
  Void reconstructFilterCoeffs(ALFParam* pcAlfParam,int **pfilterCoeffSym, int bit_depth);
  Void getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam);
  Void padImage(imgpel **imgY,  imgpel **imgY_pad, int fl, int img_height, int img_width);
  // memory allocation
  Void destroyMatrix_imgpel(imgpel **m2D);
  Void destroyMatrix_int(int **m2D);
  Void initMatrix_int(int ***m2D, int d1, int d2);
  Void initMatrix_imgpel(imgpel ***m2D, int d1, int d2);
  Void destroyMatrix4D_double(double ****m4D, int d1, int d2);
  Void destroyMatrix3D_double(double ***m3D, int d1);
  Void destroyMatrix_ushort(unsigned short **m2D);
  Void destroyMatrix_double(double **m2D);
  Void initMatrix4D_double(double *****m4D, int d1, int d2, int d3, int d4);
  Void initMatrix3D_double(double ****m3D, int d1, int d2, int d3);
  Void initMatrix_ushort(unsigned short ***m2D, int d1, int d2);
  Void initMatrix_double(double ***m2D, int d1, int d2);
  Void free_mem1Dint(int *array1D);
  Void get_mem1Dint(int **array1D, int rows);
  Void free_mem2Dpel(imgpel **array2D);
  Void get_mem2Dpel(imgpel ***array2D, int rows, int columns);
  Void no_mem_exit(const char *where);
  Void error(const char *text, int code);
#if ALF_MEM_PATCH
  Void calcVar(imgpel **imgY_var, imgpel *imgY_pad, int pad_size, int fl, int img_height, int img_width, int img_stride);
  Void DecFilter_qc(imgpel* imgY_rec,ALFParam* pcAlfParam, int Stride);
#if WIENER_3_INPUT
  Void xSubCUAdaptive_qc(TComDataCU* pcCU, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, imgpel *imgY_pred, imgpel *imgY_resi, UInt uiAbsPartIdx, UInt uiDepth, Int Stride);
  Void xCUAdaptive_qc(TComPic* pcPic, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, imgpel *imgY_pred, imgpel *imgY_resi, Int Stride);
  Void subfilterFrame(imgpel *imgY_rec_post, imgpel *imgY_rec, imgpel *imgY_pred, imgpel *imgY_resi, int filtNo, int filtNo_resi, int filtNo_pred, int start_height, int end_height, int start_width, int end_width, int bits_rec, int bits_pred, int bits_resi, int Stride);
  Void filterFrame(imgpel *imgY_rec_post, imgpel *imgY_rec, imgpel *imgY_pred, imgpel *imgY_resi, int filtNo, int filtNo_resi, int filtNo_pred, int bits_rec, int bits_pred, int bits_resi, int Stride);
#else  
  Void xSubCUAdaptive_qc(TComDataCU* pcCU, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, UInt uiAbsPartIdx, UInt uiDepth, Int Stride);
  Void xCUAdaptive_qc(TComPic* pcPic, ALFParam* pcAlfParam, imgpel *imgY_rec_post, imgpel *imgY_rec, Int Stride);
  Void subfilterFrame(imgpel *imgY_rec_post, imgpel *imgY_rec, int filtNo, int start_height, int end_height, int start_width, int end_width, int Stride);
  Void filterFrame(imgpel *imgY_rec_post, imgpel *imgY_rec, int filtNo, int Stride);
#endif  
#endif
#if TSB_ALF_HEADER
  UInt  m_uiNumCUsInFrame;
  Void  setAlfCtrlFlags (ALFParam *pAlfParam, TComDataCU *pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt &idx);
#endif
#endif
	/// ALF for luma component
  Void	xALFLuma				( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );

	/// sub function: CU-adaptive ALF process for luma
  Void	xCUAdaptive			( TComPic*	  pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );
  Void	xSubCUAdaptive	( TComDataCU* pcCU,  ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest,
													UInt uiAbsPartIdx, UInt uiDepth );
  Void	xSubCUFilter		( Pel* pDec, Pel* pRest, Int *qh, Int iTap, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride );

	/// sub function: non-adaptive ALF process for luma
  Void	xFrame					( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap );

	// ------------------------------------------------------------------------------------------------------------------
	// For chroma component
	// ------------------------------------------------------------------------------------------------------------------

	/// ALF for chroma component
  Void	xALFChroma			( ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest );

	/// sub function: non-adaptive ALF process for chroma
  Void	xFrameChroma		( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap, Int iColor );

public:
	TComAdaptiveLoopFilter();
	virtual ~TComAdaptiveLoopFilter() {}

	// initialize & destory temporary buffer
	Void	create									( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth );
	Void	destroy									();

	// alloc & free & set functions
  Void	allocALFParam						( ALFParam* pAlfParam );
  Void	freeALFParam						( ALFParam* pAlfParam );
  Void	copyALFParam						( ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam );
#if TSB_ALF_HEADER
  Void  setNumCUsInFrame        (TComPic *pcPic);
#endif

	// predict filter coefficients
  Void	predictALFCoeff					( ALFParam* pAlfParam );									///< prediction of luma ALF coefficients
  Void	predictALFCoeffChroma		( ALFParam* pAlfParam );									///< prediction of chroma ALF coefficients

	// interface function
  Void	ALFProcess							( TComPic* pcPic, ALFParam* pcAlfParam );	///< interface function for ALF process
};
#endif
#endif

#endif //WIENER_3_INPUT
