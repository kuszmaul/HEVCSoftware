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

/** \file     TComAdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "TComAdaptiveLoopFilter.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#if (WIENER_3_INPUT && !QC_ALF)

//-----------------------------------------------------------------------------
// Constructor/destructor
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/*!
 * \brief Default constructor
 */
//-----------------------------------------------------------------------------
template <typename T>
Plane<T>::Plane()
{
  m_width = 0;
  m_height = 0;
  m_pitch = 0;
  m_border_size =0;
  m_data = NULL;
  m_origin = NULL;
}

//-----------------------------------------------------------------------------
/*!
 * \brief Constructor
 *
 * \param[in] width Width of plane
 * \param[in] height Height of plane
 * \param[in] border_size Number of samples added at each border
 */
//-----------------------------------------------------------------------------
template <typename T>
Plane<T>::Plane(int width, int height, int border_size)
{
  m_data = NULL;
  set_size(width, height, border_size);
}

//-----------------------------------------------------------------------------
/*!
 * \brief Destructor
 */
//-----------------------------------------------------------------------------
template <typename T>
Plane<T>::~Plane()
{
  if (m_data != NULL)
    delete[] m_data;
}

//-----------------------------------------------------------------------------
/*!
 * \brief Set size
 *
 * \param[in] width Width of plane
 * \param[in] height Height of plane
 * \param[in] border_size Number of samples added at each border
 */
//-----------------------------------------------------------------------------
template <typename T>
void Plane<T>::set_size(int width, int height, int border_size)
{
  if (m_data != NULL)
    delete[] m_data;

  m_width = width;
  m_height = height;
  m_pitch = width + 2 * border_size;
  m_border_size = border_size;

  m_data = new T[m_pitch * (height + 2 * border_size)];
  m_origin = m_data + border_size * m_pitch + border_size;
}

//-----------------------------------------------------------------------------
/*!
 * \brief Fill a rectangular window with a given value
 *
 * \param[in] row Top row of window
 * \param[in] col Leftmost column of window
 * \param[in] val Value to be assigned
 * \param[in] numRows Number of rows to fill
 * \param[in] numCols Number of columns to fill
 */
//-----------------------------------------------------------------------------
template <typename T>
void Plane<T>::set(int row, int col, T val, int numRows, int numCols)
{
  int i, j;

  for (i = 0; i < numRows; i++)
  {
    for (j = 0; j < numCols; j++)
    {
      (*this)[row+i][col+j] = val;
    }
  }
}

//-----------------------------------------------------------------------------
/*!
 * \brief Fill border samples by mirroring
 */
//-----------------------------------------------------------------------------
template <typename T>
void Plane<T>::mirror()
{
  int i, j;

  for (i = 0; i < m_height; i++)
  {
    for (j = 0; j < m_border_size; j++)
    {
      (*this)[i][-(1 + j)] = (*this)[i][1 + j];
      (*this)[i][m_width + j] = (*this)[i][m_width - 2 - j];
    }
  }

  for (i = 0; i < m_border_size; i++)
  {
    for (j = 0; j < m_pitch; j++)
    {
      (*this)[-(1 + i)][j - m_border_size] = (*this)[1 + i][j - m_border_size];
      (*this)[m_height + i][j - m_border_size] = (*this)[m_height - 2 - i][j - m_border_size];
    }
  }
}

// Explicit instantiations
template class Plane<Pel>;
template class Plane<Int>;


TComAdaptiveLoopFilter::TComAdaptiveLoopFilter()
{

}


Void TComAdaptiveLoopFilter::allocALFParam(ALFParam* pAlfParam)
{  
  Int qtable [FILTER_PRECISION_TABLE_NUMBER]={8,16,32,64,128,256,512,1024,2048,4096,8192};
  Int qtable2[FILTER_PRECISION_TABLE_NUMBER]={3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13};
  Int qtable3[FILTER_PRECISION_TABLE_NUMBER]={4, 8,16,32, 64,128,256, 512,1024,2048,4096};
  
  for (Int i=0; i<FILTER_PRECISION_TABLE_NUMBER; i++)
  {
    pAlfParam->filter_precision_table      [i] = qtable [i];
    pAlfParam->filter_precision_table_shift[i] = qtable2[i];
    pAlfParam->filter_precision_table_half [i] = qtable3[i];
  }
 
  get_mem2Dint   (&(pAlfParam->coeffs), 3, 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  ::memset(pAlfParam->coeffs[0], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));
  ::memset(pAlfParam->coeffs[1], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));
  ::memset(pAlfParam->coeffs[2], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));

    
  get_mem2Dint   (&(pAlfParam->dont_care), 3, 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  ::memset(pAlfParam->dont_care[0], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));
  ::memset(pAlfParam->dont_care[1], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));
  ::memset(pAlfParam->dont_care[2], 0, sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH));

  
  pAlfParam->alf_flag        = false;
  pAlfParam->cu_control_flag = false;
    
  for (Int c=0;c<3;c++)
  {
    pAlfParam->enable_flag  [c]    = 0;
    pAlfParam->golomb_enable[c]    = 0;
    pAlfParam->golomb_code  [c][0] = 0;
    pAlfParam->golomb_code  [c][1] = 0;
  }
}

Void TComAdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{  
  free_mem2Dint   (pAlfParam->coeffs);
  free_mem2Dint   (pAlfParam->dont_care);
}

Void TComAdaptiveLoopFilter::destroy()
{
}

Void TComAdaptiveLoopFilter::create( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth )
{
}

Void TComAdaptiveLoopFilter::copyALFParam(ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam)
{
  pDesAlfParam->alf_flag = pSrcAlfParam->alf_flag;
  pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
  
  for (Int c=0;c<3;c++)
  {
    pDesAlfParam->filter_length_RD_rec [c]    = pSrcAlfParam->filter_length_RD_rec [c];
    pDesAlfParam->filter_length_RD_pred[c]    = pSrcAlfParam->filter_length_RD_pred[c];
    pDesAlfParam->filter_length_RD_qpe [c]    = pSrcAlfParam->filter_length_RD_qpe [c];
    pDesAlfParam->enable_flag          [c]    = pSrcAlfParam->enable_flag          [c];
    pDesAlfParam->golomb_enable        [c]    = pSrcAlfParam->golomb_enable        [c];
    pDesAlfParam->golomb_code          [c][0] = pSrcAlfParam->golomb_code          [c][0];
    pDesAlfParam->golomb_code          [c][1] = pSrcAlfParam->golomb_code          [c][1];
    pDesAlfParam->filter_precision     [c][0] = pSrcAlfParam->filter_precision     [c][0];
    pDesAlfParam->filter_precision     [c][1] = pSrcAlfParam->filter_precision     [c][1];
    pDesAlfParam->filter_precision     [c][2] = pSrcAlfParam->filter_precision     [c][2];
  }
  
  ::memcpy(pDesAlfParam->coeffs[0], pSrcAlfParam->coeffs[0], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );
  ::memcpy(pDesAlfParam->coeffs[1], pSrcAlfParam->coeffs[1], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );
  ::memcpy(pDesAlfParam->coeffs[2], pSrcAlfParam->coeffs[2], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );

  ::memcpy(pDesAlfParam->dont_care[0], pSrcAlfParam->dont_care[0], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );
  ::memcpy(pDesAlfParam->dont_care[1], pSrcAlfParam->dont_care[1], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );
  ::memcpy(pDesAlfParam->dont_care[2], pSrcAlfParam->dont_care[2], sizeof(Int)* (1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH) );
}


// --------------------------------------------------------------------------------------------------------------------
// interface function for actual ALF process
// --------------------------------------------------------------------------------------------------------------------

/**
    \param  pcPic         picture (TComPic) class (input/output)
    \param  pcAlfParam    ALF parameter
 */
Void TComAdaptiveLoopFilter::ALFProcess(  TComPic* pcPic , ALFParam* pcAlfParam )
{
  if(!pcAlfParam->alf_flag)
  {
    return;
  }
  
  Int height;
  Int width;
  Int stride;
  Int stride_out;
  
  Pel* pDec;
  Pel* pP;
  Pel* pQ;
  Pel* pout;
  
  TComPicYuv* pcPicYuvRec          = pcPic->getPicYuvRec();
  TComPicYuv* pcPicYuvP            = pcPic->getPicYuvP();
  TComPicYuv* pcPicYuvQ            = pcPic->getPicYuvQ();

  TComPicYuv* pcPicYuvFiltered = new TComPicYuv ;
  pcPicYuvFiltered->create(pcPicYuvRec->getWidth(), pcPicYuvRec->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth ) ;
  
  
  for (Int c = 0; c < 3; c++)
  {
    if (1 == pcAlfParam->enable_flag[c])
    {
      if (c==0)
      {
        pDec = pcPicYuvRec->getLumaAddr();
        pP   = pcPicYuvP->getLumaAddr();
        pQ   = pcPicYuvQ->getLumaAddr();
        pout = pcPicYuvFiltered->getLumaAddr();
        
        height      = pcPic->getSlice()->getSPS()->getHeight();
        width       = pcPic->getSlice()->getSPS()->getWidth ();
        stride      = pcPicYuvRec->getStride();
        stride_out  = pcPicYuvFiltered->getStride();
      }
      else if (c==1)
      {
        pDec = pcPicYuvRec->getCbAddr();
        pP   = pcPicYuvP->getCbAddr();
        pQ   = pcPicYuvQ->getCbAddr();
        pout = pcPicYuvFiltered->getCbAddr();
        
        height      = (pcPic->getSlice()->getSPS()->getHeight())>>1;
        width       = (pcPic->getSlice()->getSPS()->getWidth ())>>1;            
        stride      = pcPicYuvRec->getCStride();
        stride_out  = pcPicYuvFiltered->getCStride();
      }
      else
      {
        pDec = pcPicYuvRec->getCrAddr();
        pP   = pcPicYuvP->getCrAddr();
        pQ   = pcPicYuvQ->getCrAddr();      
        pout = pcPicYuvFiltered->getCrAddr();
        
        height      = (pcPic->getSlice()->getSPS()->getHeight())>>1;
        width       = (pcPic->getSlice()->getSPS()->getWidth ())>>1;            
        stride      = pcPicYuvRec->getCStride();
        stride_out  = pcPicYuvFiltered->getCStride();
      }
    
      filter (pcAlfParam, pDec, pout, pP, pQ, c, height, width, stride, stride_out);
      
      if (0 == c)
      {
        pcPicYuvFiltered->copyToPicLuma( pcPicYuvRec );
      }
      else if (1 == c)
      {
        pcPicYuvFiltered->copyToPicCb( pcPicYuvRec );
      }
      else
      {
        pcPicYuvFiltered->copyToPicCr( pcPicYuvRec );
      }
    }
  }
  
  pcPicYuvFiltered->destroy() ;
  delete pcPicYuvFiltered ;  
}

Void TComAdaptiveLoopFilter::filter(ALFParam* pAlfParam, Pel *in, Pel *out, Pel *pred, Pel *qpe, Int component, Int height, Int width, Int Stride_in, Int Stride_out)
{

  if (1 == pAlfParam->enable_flag[component])
  {
    Pel *p_out = out;
    
    Pel* pPel_1;
    Pel* pPel_2;
    Pel* pPel_3;
    
    Plane<Pel> p_xY (width, height, (MAX_WIENER_FILTER_LENGTH - 1) / 2);
    Plane<Pel> p_pY (width, height, (MAX_WIENER_FILTER_LENGTH - 1) / 2);
    Plane<Int> p_qpe(width, height, (MAX_WIENER_FILTER_LENGTH - 1) / 2);
  
    pPel_1 = in;
    pPel_2 = pred; 
    pPel_3 = qpe; 
    for (Int i = 0; i < height; i++)
    {
      for (Int j = 0; j < width; j++)
      {
        p_xY [i][j] = pPel_1 [j];
        p_pY [i][j] = pPel_2 [j];
        p_qpe[i][j] = pPel_3 [j] - g_uiIBDI_MAX_Q;
      }
      pPel_1 +=Stride_in;
      pPel_2 +=Stride_in;
      pPel_3 +=Stride_in;
    }

    p_xY.mirror();
    p_pY.mirror();
    p_qpe.mirror();
    
    filter(pAlfParam, 
           p_xY, 
           out, 
           p_pY, 
           p_qpe, 
           width, 
           height,
           Stride_out,
           component);
  }
  else
  {
    //leave as is
  }
}





Void TComAdaptiveLoopFilter::filter  (ALFParam* pAlfParam,
                                      const Plane<Pel> &xY,
                                      Pel *oY, 
                                      const Plane<Pel> &pY, 
                                      const Plane<Int> &qpe, 
                                      Int width, 
                                      Int height, 
                                      Int Stride, 
                                      Int component)
{
  
      

  
  int i, j, m, n, sum;
  int tmp_rec1  = pAlfParam->filter_length_RD_rec [component];
  int tmp_pred1 = pAlfParam->filter_length_RD_pred[component];
  int tmp_qpe1  = pAlfParam->filter_length_RD_qpe [component];
  int tmp_rec  = (tmp_rec1  - 1) >> 1;
  int tmp_pred = (tmp_pred1 - 1) >> 1;
  int tmp_qpe  = (tmp_qpe1  - 1) >> 1;

  Pel *p_oY = oY;
  const Pel *uc_ptr;
  const Int *s_ptr;
  int pitch;
  int filter_precision_half  = pAlfParam->filter_precision_table_half  [pAlfParam->filter_precision[component][0]];
  int filter_precision_shift = pAlfParam->filter_precision_table_shift [pAlfParam->filter_precision[component][0]];

  Int half = pAlfParam->filter_precision_table_half [pAlfParam->filter_precision[component][0]];
  Int shift= pAlfParam->filter_precision_table_shift[pAlfParam->filter_precision[component][0]];
  
  
  Int *p_coeffs = new Int[tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1+1];
  
  for (i = 0; i < tmp_rec1*tmp_rec1; i++)
  {
    p_coeffs[i]=pAlfParam->coeffs[component][i];
  }
  for (i = tmp_rec1*tmp_rec1; i < tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1; i++)
  {
    p_coeffs[i] =pAlfParam->coeffs[component][i]*pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][0]];
    p_coeffs[i]/=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][1]];
  }
  for (i = tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1; i < tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1; i++)
  {
    p_coeffs[i] =pAlfParam->coeffs[component][i]*pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][0]];
    p_coeffs[i]/=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][2]];
  }
  p_coeffs[tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1]=pAlfParam->coeffs[component][tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1];
  
  for (j = 0; j < height; j++) 
  {
    for (i = 0; i < width; i++) 
    {
//       int *co = (pAlfParam->coeffs)[component];
      Int *co = p_coeffs;
            
      sum = 0;
      
      uc_ptr = xY[j - tmp_rec] + (i - tmp_rec);
      pitch = xY.get_pitch() - tmp_rec1;

      for (n = 0; n < tmp_rec1; n++)
      {        
        for (m = 0; m < tmp_rec; m++)
        {
          sum += (*uc_ptr++) * (*co++);
          sum += (*uc_ptr++) * (*co++);
        }
        sum += (*uc_ptr++) * (*co++);

        uc_ptr += pitch;
      }
            
      uc_ptr = pY[j - tmp_pred] + (i - tmp_pred);
      pitch = pY.get_pitch() - tmp_pred1;

      for (n = 0; n < tmp_pred1; n++)
      {
        for (m = 0; m < tmp_pred; m++)
        {
          sum += (*uc_ptr++) * (*co++);
          sum += (*uc_ptr++) * (*co++);
        }
        sum += (*uc_ptr++) * (*co++);

        uc_ptr += pitch;
      }

      s_ptr = qpe[j - tmp_qpe] + (i - tmp_qpe);
      pitch = qpe.get_pitch() - tmp_qpe1;

      for (n = 0; n < tmp_qpe1; n++) 
      {//y
        for (m = 0; m < tmp_qpe; m++)
        {//x
          sum += (*s_ptr++) * (*co++);
          sum += (*s_ptr++) * (*co++);
        }
        sum += (*s_ptr++) * (*co++);

        s_ptr += pitch;
      }
     
      sum +=(*co++);
      
      sum  +=  filter_precision_half;
      sum >>=  filter_precision_shift;
      p_oY[i] = (Pel)Clip(sum);
    }
    p_oY +=Stride;
  }  
  
  delete [] p_coeffs;
}








Int TComAdaptiveLoopFilter::get_mem2Dint(Int ***array2D, Int rows, Int columns)
{
  Int i;

  if((*array2D      = (Int**)calloc(rows,        sizeof(Int*))) == NULL)
    no_mem_exit("get_mem2Dint: array2D");
  if(((*array2D)[0] = (Int* )calloc(rows*columns,sizeof(Int ))) == NULL)
    no_mem_exit("get_mem2Dint: array2D");

  for(i=1 ; i<rows ; i++)
    (*array2D)[i] =  (*array2D)[i-1] + columns  ;

  return rows*columns*sizeof(Int);
}

void TComAdaptiveLoopFilter::free_mem2Dint(Int **array2D)
{
  if (array2D)
  {
    if (array2D[0]) 
      free (array2D[0]);
    else exit(0);

    free (array2D);

  } 
  else
  {
    exit(0);
  }
}

void TComAdaptiveLoopFilter::no_mem_exit(const char *where)
{
  printf ("Memory error in %s\n", where);
  exit(0);
}

Int TComAdaptiveLoopFilter::get_mem1Dint(Int **array1D, Int raws)
{
  Int j;

  if((*array1D      = (Int*)calloc(raws, sizeof(Int))) == NULL)
    no_mem_exit("get_mem1Dint: array3D");

  for(j = 0; j < raws; j++)
    (*array1D)[j] = 0;
  return sizeof(int)*raws;
}

void TComAdaptiveLoopFilter::free_mem1Dint(Int *array1D)
{
  if (array1D)
    free (array1D);
  else
    no_mem_exit("free_mem1Dint: array1D");
}



#else


#if HHI_ALF
// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComAdaptiveLoopFilter::TComAdaptiveLoopFilter()
{
  m_pcTempPicYuv  = NULL;
}

Void TComAdaptiveLoopFilter::create( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth )
{
  if ( !m_pcTempPicYuv )
  {
    m_pcTempPicYuv = new TComPicYuv;
    m_pcTempPicYuv->create( iPicWidth, iPicHeight, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth );
  }
}

Void TComAdaptiveLoopFilter::destroy()
{
  if ( m_pcTempPicYuv )
  {
    m_pcTempPicYuv->destroy();
    delete m_pcTempPicYuv;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// allocate / free / copy functions
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::allocALFParam(ALFParam* pAlfParam)
{
  pAlfParam->alf_flag = false;
  pAlfParam->pcQuadTree = NULL;
  pAlfParam->bSeparateQt = false;

  pAlfParam->acHorizontalAlfFilter   = new AlfFilter[ ALF_FILT_FOR_CHROMA +1 ];
  pAlfParam->acVerticalAlfFilter     = new AlfFilter[ ALF_FILT_FOR_CHROMA +1 ];

  ::memset(pAlfParam->acHorizontalAlfFilter,  0, sizeof(AlfFilter)*( ALF_FILT_FOR_CHROMA +1 ) );
  ::memset(pAlfParam->acVerticalAlfFilter  ,  0, sizeof(AlfFilter)*( ALF_FILT_FOR_CHROMA +1 ) );


  pAlfParam->acHorizontalAlfFilter[ 0 ].bIsHorizontal         = true;
  pAlfParam->acHorizontalAlfFilter[ 0 ].bIsVertical           = false;
  pAlfParam->acHorizontalAlfFilter[ 0 ].aiQuantFilterCoeffs   = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->acHorizontalAlfFilter[ 0 ].aiTapCoeffMapping     = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->acHorizontalAlfFilter[ 0 ].aiCoeffWeights        = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->acVerticalAlfFilter  [ 0 ].bIsHorizontal         = false;
  pAlfParam->acVerticalAlfFilter  [ 0 ].bIsVertical           = true;
  pAlfParam->acVerticalAlfFilter  [ 0 ].aiQuantFilterCoeffs   = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->acVerticalAlfFilter  [ 0 ].aiTapCoeffMapping     = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->acVerticalAlfFilter  [ 0 ].aiCoeffWeights        = new Int[ALF_MAX_NUM_COEF];
  ::memset( pAlfParam->acHorizontalAlfFilter[ 0 ].aiQuantFilterCoeffs , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  ::memset( pAlfParam->acHorizontalAlfFilter[ 0 ].aiTapCoeffMapping   , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  ::memset( pAlfParam->acHorizontalAlfFilter[ 0 ].aiCoeffWeights      , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  ::memset( pAlfParam->acVerticalAlfFilter  [ 0 ].aiQuantFilterCoeffs , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  ::memset( pAlfParam->acVerticalAlfFilter  [ 0 ].aiTapCoeffMapping   , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  ::memset( pAlfParam->acVerticalAlfFilter  [ 0 ].aiCoeffWeights      , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );

  for( UInt uiIndx=1; uiIndx <= ALF_FILT_FOR_CHROMA ; uiIndx++ )
  {
    pAlfParam->acHorizontalAlfFilter[ uiIndx ].bIsHorizontal         = true;
    pAlfParam->acHorizontalAlfFilter[ uiIndx ].bIsVertical           = false;
    pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs   = new Int[ALF_MAX_NUM_COEF];
    pAlfParam->acVerticalAlfFilter  [ uiIndx ].bIsHorizontal         = false;
    pAlfParam->acVerticalAlfFilter  [ uiIndx ].bIsVertical           = true;
    pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiQuantFilterCoeffs   = new Int[ALF_MAX_NUM_COEF];
    pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiTapCoeffMapping     = new Int[ALF_MAX_NUM_COEF];
    pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiCoeffWeights        = new Int[ALF_MAX_NUM_COEF];
    pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiTapCoeffMapping     = new Int[ALF_MAX_NUM_COEF];
    pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiCoeffWeights        = new Int[ALF_MAX_NUM_COEF];
    ::memset( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
    ::memset( pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiQuantFilterCoeffs , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
    ::memset( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiTapCoeffMapping   , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
    ::memset( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiCoeffWeights      , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
    ::memset( pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiTapCoeffMapping   , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
    ::memset( pAlfParam->acVerticalAlfFilter  [ uiIndx ].aiCoeffWeights      , 0 , sizeof(Int) * ALF_MAX_NUM_COEF );
  }

}


Void TComAdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{
  assert(pAlfParam != NULL);

  if( pAlfParam->acHorizontalAlfFilter != NULL )
  {
    for( UInt uiIndx=0; uiIndx <=  ALF_FILT_FOR_CHROMA  ; uiIndx++ )
    {
      if( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs != NULL )
      {
        delete[] pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs;
        pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs = NULL;
      }
      if( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiTapCoeffMapping != NULL )
      {
        delete[] pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiTapCoeffMapping;
        pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiTapCoeffMapping = NULL;
      }
      if( pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiCoeffWeights != NULL )
      {
        delete[] pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiCoeffWeights;
        pAlfParam->acHorizontalAlfFilter[ uiIndx ].aiCoeffWeights = NULL;
      }
    }
    delete[] pAlfParam->acHorizontalAlfFilter;
    pAlfParam->acHorizontalAlfFilter = NULL;
  }


  if( pAlfParam->acVerticalAlfFilter != NULL )
    {
      for( UInt uiIndx=0; uiIndx <=  ALF_FILT_FOR_CHROMA  ; uiIndx++ )
      {
        if( pAlfParam->acVerticalAlfFilter[ uiIndx ].aiQuantFilterCoeffs != NULL )
        {
          delete[] pAlfParam->acVerticalAlfFilter[ uiIndx ].aiQuantFilterCoeffs;
          pAlfParam->acVerticalAlfFilter[ uiIndx ].aiQuantFilterCoeffs = NULL;
        }
        if( pAlfParam->acVerticalAlfFilter[ uiIndx ].aiTapCoeffMapping != NULL )
        {
          delete[] pAlfParam->acVerticalAlfFilter[ uiIndx ].aiTapCoeffMapping;
          pAlfParam->acVerticalAlfFilter[ uiIndx ].aiTapCoeffMapping = NULL;
        }
        if( pAlfParam->acVerticalAlfFilter[ uiIndx ].aiCoeffWeights != NULL )
        {
          delete[] pAlfParam->acVerticalAlfFilter[ uiIndx ].aiCoeffWeights;
          pAlfParam->acVerticalAlfFilter[ uiIndx ].aiCoeffWeights = NULL;
        }
      }
      delete[] pAlfParam->acVerticalAlfFilter;
      pAlfParam->acVerticalAlfFilter = NULL;
    }

  pAlfParam->pcQuadTree = NULL;

}

Void TComAdaptiveLoopFilter::destroyQuadTree(ALFParam* pcAlfParam)
{
  pcAlfParam->pcQuadTree->destroy();
  delete pcAlfParam->pcQuadTree;
  pcAlfParam->pcQuadTree = NULL;
}

Void TComAdaptiveLoopFilter::copyALFParam(ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam)
{
  pDesAlfParam->alf_flag        = pSrcAlfParam->alf_flag;
  pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
  pDesAlfParam->pcQuadTree      = pSrcAlfParam->pcQuadTree;
  pDesAlfParam->bSeparateQt     = pSrcAlfParam->bSeparateQt;
  pDesAlfParam->chroma_idc      = pSrcAlfParam->chroma_idc;

  if( pSrcAlfParam->acVerticalAlfFilter != NULL )
  {
    for( UInt uiIndx=0; uiIndx <=  ALF_FILT_FOR_CHROMA  ; uiIndx++ )
    {
      xFillAlfFilterInitParam( pDesAlfParam->acVerticalAlfFilter[ uiIndx ] , pSrcAlfParam->acVerticalAlfFilter[ uiIndx ].iFilterLength, pSrcAlfParam->acVerticalAlfFilter[ uiIndx ].iFilterSymmetry );
      ::memcpy( pDesAlfParam->acVerticalAlfFilter[ uiIndx ].aiQuantFilterCoeffs, pSrcAlfParam->acVerticalAlfFilter[ uiIndx ].aiQuantFilterCoeffs, sizeof(Int)*ALF_MAX_NUM_COEF );
      pDesAlfParam->acVerticalAlfFilter[ uiIndx ].bIsValid =  pSrcAlfParam->acVerticalAlfFilter[ uiIndx ].bIsValid;
    }
  }


  if( pSrcAlfParam->acHorizontalAlfFilter != NULL )
  {
    for( UInt uiIndx=0; uiIndx <=  ALF_FILT_FOR_CHROMA  ; uiIndx++ )
    {
      xFillAlfFilterInitParam( pDesAlfParam->acHorizontalAlfFilter[ uiIndx ] , pSrcAlfParam->acHorizontalAlfFilter[ uiIndx ].iFilterLength, pSrcAlfParam->acHorizontalAlfFilter[ uiIndx ].iFilterSymmetry );
      ::memcpy( pDesAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs, pSrcAlfParam->acHorizontalAlfFilter[ uiIndx ].aiQuantFilterCoeffs, sizeof(Int)*ALF_MAX_NUM_COEF );
      pDesAlfParam->acHorizontalAlfFilter[ uiIndx ].bIsValid =  pSrcAlfParam->acHorizontalAlfFilter[ uiIndx ].bIsValid;
    }
  }

  for(Int i = 0; i<3; i++)
  {
    pDesAlfParam->aiPlaneFilterMapping[i] = pSrcAlfParam->aiPlaneFilterMapping[i] ;
  }
}

Void TComAdaptiveLoopFilter::copyALFFilter(AlfFilter& rDesAlfFilter, AlfFilter& rSrcAlfFilter)
{
  xFillAlfFilterInitParam( rDesAlfFilter , rSrcAlfFilter.iFilterLength, rSrcAlfFilter.iFilterSymmetry );
  ::memcpy( rDesAlfFilter.aiQuantFilterCoeffs, rSrcAlfFilter.aiQuantFilterCoeffs, sizeof(Int)*ALF_MAX_NUM_COEF );
  rDesAlfFilter.bIsValid =  rSrcAlfFilter.bIsValid;
}

// --------------------------------------------------------------------------------------------------------------------
// prediction of filter coefficients
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::xPredictALFCoeff( ALFParam* pAlfParam)
{
  Int iCenterPos = 0;
  Int iLength    = 0;
  for(Int i=0; i<ALF_FILT_FOR_CHROMA+1; i++)
  {
    if ( ( pAlfParam->chroma_idc & i) || ( i == 0 ) )
    {
      //horizontal
      iLength = pAlfParam->acHorizontalAlfFilter[i].iFilterLength;
      iCenterPos = iLength >> 1;
    if(pAlfParam->acHorizontalAlfFilter[i].iFilterSymmetry == 0)
      {
        for(Int j=0 ; j < iCenterPos; j++ )
                pAlfParam->acHorizontalAlfFilter[i].aiQuantFilterCoeffs[ iLength-j-1 ] += pAlfParam->acHorizontalAlfFilter[i].aiQuantFilterCoeffs[j];
      }

      Int iSum = 0 ;
      for (Int j = 0; j< pAlfParam->acHorizontalAlfFilter[i].iFilterLength; j++)
      {
        if(j!=iCenterPos)
          iSum += pAlfParam->acHorizontalAlfFilter[i].aiQuantFilterCoeffs[ pAlfParam->acHorizontalAlfFilter[i].aiTapCoeffMapping[j] ] ;
      }

      pAlfParam->acHorizontalAlfFilter[i].aiQuantFilterCoeffs[iCenterPos]+= ((1<<ALF_NUM_BIT_SHIFT) - iSum) ;

      ///
      iLength = pAlfParam->acVerticalAlfFilter[i].iFilterLength;
      iCenterPos = iLength >> 1;
    if(pAlfParam->acVerticalAlfFilter[i].iFilterSymmetry == 0)
      {
        for(Int j=0 ; j < iCenterPos; j++ )
          pAlfParam->acVerticalAlfFilter[i].aiQuantFilterCoeffs[ iLength-j-1 ] += pAlfParam->acVerticalAlfFilter[i].aiQuantFilterCoeffs[j];
      }
      iSum = 0 ;
      for (Int j = 0; j< pAlfParam->acVerticalAlfFilter[i].iFilterLength; j++)
      {
        if(j!=iCenterPos)
          iSum += pAlfParam->acVerticalAlfFilter[i].aiQuantFilterCoeffs[ pAlfParam->acVerticalAlfFilter[i].aiTapCoeffMapping[j] ] ;
      }
      pAlfParam->acVerticalAlfFilter[i].aiQuantFilterCoeffs[iCenterPos]+= ((1<<ALF_NUM_BIT_SHIFT) - iSum) ;
    }
  }
}


// --------------------------------------------------------------------------------------------------------------------
// interface function for actual ALF process
// --------------------------------------------------------------------------------------------------------------------

/**
    \param  pcPic         picture (TComPic) class (input/output)
    \param  pcAlfParam    ALF parameter
 */
Void TComAdaptiveLoopFilter::ALFProcess(  TComPic* pcPic , ALFParam* pcAlfParam )
{
  if(!pcAlfParam->alf_flag)
  {
    return;
  }
  
  xFillAlfFilterInitParam(pcAlfParam->acHorizontalAlfFilter[0], pcAlfParam->acHorizontalAlfFilter[0].iFilterLength, pcAlfParam->acHorizontalAlfFilter[0].iFilterSymmetry) ;
  xFillAlfFilterInitParam(pcAlfParam->acVerticalAlfFilter[0], pcAlfParam->acVerticalAlfFilter[0].iFilterLength, pcAlfParam->acVerticalAlfFilter[0].iFilterSymmetry) ;

  for (Int iPlane = 1; iPlane < ALF_FILT_FOR_CHROMA + 1; iPlane++)
  {
    if ( pcAlfParam->chroma_idc & iPlane)
    {
      xFillAlfFilterInitParam( pcAlfParam->acHorizontalAlfFilter[iPlane], pcAlfParam->acHorizontalAlfFilter[iPlane].iFilterLength, pcAlfParam->acHorizontalAlfFilter[iPlane].iFilterSymmetry ) ;
      xFillAlfFilterInitParam( pcAlfParam->acVerticalAlfFilter  [iPlane], pcAlfParam->acVerticalAlfFilter  [iPlane].iFilterLength, pcAlfParam->acVerticalAlfFilter  [iPlane].iFilterSymmetry ) ;
      if( pcAlfParam->aiPlaneFilterMapping[iPlane] != iPlane)
      {
        copyALFFilter(pcAlfParam->acHorizontalAlfFilter[iPlane],pcAlfParam->acHorizontalAlfFilter[pcAlfParam->aiPlaneFilterMapping[iPlane]]);
        copyALFFilter(pcAlfParam->acVerticalAlfFilter[iPlane],pcAlfParam->acVerticalAlfFilter[pcAlfParam->aiPlaneFilterMapping[iPlane]]);
      }
    }
  }
  xPredictALFCoeff(pcAlfParam);

  TComPicYuv* pcPicYuvRec    = pcPic->getPicYuvRec();
  xALFFilter( pcPic , pcAlfParam, pcPicYuvRec, 0 );

  for (Int iPlane = 1; iPlane < ALF_FILT_FOR_CHROMA + 1 ; iPlane++)
  {
    if ( pcAlfParam->chroma_idc & iPlane )
    {
      xALFFilter( pcPic , pcAlfParam, pcPicYuvRec, iPlane );
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


Void TComAdaptiveLoopFilter::xFillAlfFilterInitParam( AlfFilter& rcFilter, Int iLength, Int iSymmetry )
{
  rcFilter.iFilterLength   = iLength;
  rcFilter.iFilterSymmetry = iSymmetry;
  rcFilter.iNumOfCoeffs    = iSymmetry ? ( iLength >> 1 ) + 1 : iLength;
  rcFilter.iOverlap        = iLength >> 1;
  if( iSymmetry )
  {
    for( Int i = 0; i < rcFilter.iNumOfCoeffs; i++ )
    {
      rcFilter.aiTapCoeffMapping[ i ]               = i;
      rcFilter.aiTapCoeffMapping[ iLength - i  -1 ] = i;
      rcFilter.aiCoeffWeights   [ i ]               = 2;
    }
    rcFilter.aiCoeffWeights[ rcFilter.iNumOfCoeffs - 1 ] = 1;
  }
  else
  {
    for( Int i = 0; i < iLength ; i++ )
    {
      rcFilter.aiTapCoeffMapping[i] = i;
      rcFilter.aiCoeffWeights   [i] = 1;
    }
  }
  if( rcFilter.bIsHorizontal )
  {
    rcFilter.iHorizontalOverlap = rcFilter.iOverlap;
    rcFilter.iVerticalOverlap   = 0;
  }
  else if( rcFilter.bIsVertical )
  {
    rcFilter.iHorizontalOverlap = 0;
    rcFilter.iVerticalOverlap   = rcFilter.iOverlap;
  }

  rcFilter.bIsValid = false;
}



// --------------------------------------------------------------------------------------------------------------------
// ALF for luma
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::xALFFilter( TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicRest, Int iPlane)
{
  if(pcAlfParam->cu_control_flag && iPlane ==0 )
  {
    // block-adaptive ALF process
    TComPicYuv* pcTempPicYuv2 = new TComPicYuv;
    pcTempPicYuv2->create( pcPicRest->getWidth(), pcPicRest->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
    pcPicRest->copyToPicLuma( pcTempPicYuv2 );
    xApplyFrame      ( pcPicRest, m_pcTempPicYuv , pcAlfParam->acVerticalAlfFilter  [0] );
    xApplyFrame      ( m_pcTempPicYuv , pcPicRest, pcAlfParam->acHorizontalAlfFilter[0] );
    //xCUAdaptive      ( m_pcAlfQuadTree, pcAlfParam->acHorizontalAlfFilter[0] , m_pcTempPicYuv, pcPicRest );
    xCopyDecToRestCUs( pcAlfParam->pcQuadTree ,  pcTempPicYuv2 , pcPicRest );
    pcTempPicYuv2->destroy();
    delete pcTempPicYuv2;
  }
  else
  {
    // non-adaptive ALF process
    xApplyFrame( pcPicRest, m_pcTempPicYuv , pcAlfParam->acVerticalAlfFilter  [iPlane], iPlane );
    xApplyFrame( m_pcTempPicYuv, pcPicRest , pcAlfParam->acHorizontalAlfFilter[iPlane], iPlane );
  }
}

Void TComAdaptiveLoopFilter::xCUAdaptive(TComPicSym* pcQuadTree, AlfFilter& rcFilter , TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  // for every CU, call CU-adaptive ALF process
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    xSubCUAdaptive(pcQuadTree, pcCU, rcFilter, pcPicDec, pcPicRest, 0, 0);
  }
}

/**
    - For every sub-CU's, it is called recursively
    .
    \param  pcCU          CU data structure
    \param  pcAlfParam    ALF parameter
    \param  pcPicDec      picture before ALF
    \retval pcPicRest     picture after  ALF
    \param  uiAbsPartIdx  current sub-CU position
    \param  uiDepth       current sub-CU depth
 */
Void TComAdaptiveLoopFilter::xSubCUAdaptive(TComPicSym* pcQuadTree, TComDataCU* pcCU, AlfFilter& rcFilter, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt uiAbsPartIdx, UInt uiDepth)
{
  Bool bBoundary = false;
  Int iLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Int iRPelX   = iLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  Int iTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Int iBPelY   = iTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  // check picture boundary
  if ( ( iRPelX >= pcCU->getSlice()->getSPS()->getWidth() ) || ( iBPelY >= pcCU->getSlice()->getSPS()->getHeight() ) )
  {
    bBoundary = true;
  }

  // go to sub-CU?
  if ( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcQuadTree->getNumPartition() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      iLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      iTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( iLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( iTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xSubCUAdaptive(pcQuadTree, pcCU, rcFilter, pcPicDec, pcPicRest, uiAbsPartIdx, uiDepth+1);
    }
    return;
  }

  // check CU-based flag
  if ( pcCU->getAlfCtrlFlag(uiAbsPartIdx) )
  {
    const Int iOffsetX = iLPelX - Max( 0 , iLPelX - rcFilter.iHorizontalOverlap );
    const Int iOffsetY = iTPelY - Max( 0 , iTPelY - rcFilter.iVerticalOverlap   );
    const Int iMaxX    = pcCU->getSlice()->getSPS()->getWidth()  - iLPelX + iOffsetX;
    const Int iMaxY    = pcCU->getSlice()->getSPS()->getHeight() - iTPelY + iOffsetY;

    Pel* pDec = pcPicDec->getLumaAddr( pcCU->getAddr(), uiAbsPartIdx );
    pDec -= iOffsetX + pcPicDec->getStride() * iOffsetY;


    xApplyFilter( pDec,
                  pcPicRest->getLumaAddr(pcCU->getAddr(), uiAbsPartIdx),
                  pcCU->getWidth(uiAbsPartIdx),
                  pcCU->getHeight(uiAbsPartIdx),
                  pcPicDec->getStride(),
                  pcPicRest->getStride(),
                  rcFilter,
                  iOffsetX,
                  iOffsetY,
                  iMaxX,
                  iMaxY
                  );

  }
}



Void TComAdaptiveLoopFilter::xApplyFilter  ( Pel* pDec, Pel* pRest, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride, AlfFilter& rcFilter )
{
  xApplyFilter(pDec, pRest, iWidth, iHeight, iDecStride, iRestStride, rcFilter, 0, 0, iWidth, iHeight );
}


Void TComAdaptiveLoopFilter::xApplyFilter  ( Pel* pDec, Pel* pRest, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride, AlfFilter& rcFilter,Int iOffsetX, Int iOffsetY, Int iMaxX, Int iMaxY )
{
  Int iAccu = 0;

  const Int iFilterLength  = rcFilter.iFilterLength;
  const Int iFilterOverlap = iFilterLength >> 1;
//  const Int iBitShift      = g_uiBitDepth + g_uiBitIncrement - 8;
  const Int iBitShift      = ALF_NUM_BIT_SHIFT ;
  Int iDcOffset      = 0;

  const Pel *pcSourceData = pDec;
        Pel *pcDestData   = pRest;

  const Int iSourceStride      = iDecStride ;
  const Int iDestStride        = iRestStride ;

  const Int iAdd = 1 << (iBitShift - 1 );

  const Int iSizeX    = iWidth;
  const Int iSizeY    = iHeight;

  Int piCoeffs[ALF_MAX_NUM_TAP];
  if( rcFilter.iFilterSymmetry == 0 )
  {
#if ALF_DC_CONSIDERED
    iDcOffset = rcFilter.aiQuantFilterCoeffs[ rcFilter.iNumOfCoeffs ];
#else
    iDcOffset = 0;
#endif
    for(Int i = 0; i < iFilterLength; i++)
    {
      piCoeffs[i] = rcFilter.aiQuantFilterCoeffs[i];
    }
  }
  else
  {
#if ALF_DC_CONSIDERED
    iDcOffset = rcFilter.aiQuantFilterCoeffs[ rcFilter.iNumOfCoeffs ];
#else
    iDcOffset = 0;
#endif
    for(Int i = 0; i <= iFilterOverlap; i++)
    {
      piCoeffs[i] = rcFilter.aiQuantFilterCoeffs[i];
      piCoeffs[ iFilterLength -1 -i ] = rcFilter.aiQuantFilterCoeffs[i];
    }

  }



  if (rcFilter.bIsHorizontal )
  {
  Int iStartX = Max( iOffsetX , iFilterOverlap       );
  Int iStopX  = Min( iMaxX - iFilterOverlap , iSizeX + iOffsetX);

  for (Int y = 0; y < iSizeY; y++)
  {
   for (Int x = iOffsetX ; x < iFilterOverlap ; x++)  // x - uiFilterOverlap + i < 0
   {
     iAccu = 0 ;
     for (Int i = 0; i < iFilterOverlap-x; i++)
     {
       iAccu += pcSourceData[ iSourceStride * y  ] * piCoeffs[i];
     }
     for (Int i = iFilterOverlap-x; i < iFilterLength; i++)
     {
       iAccu += pcSourceData[ x - iFilterOverlap + i  + iSourceStride * y  ] * piCoeffs[i];
     }
     pcDestData[x - iOffsetX + iDestStride * y ] =  (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift)  ;
   }

   for (Int x = iStartX ; x < iStopX ; x++)  // 0 < x - uiFilterOverlap + i < uiSizeX
   {
     iAccu = 0 ;
     for (Int i = 0; i < iFilterLength; i++)
     {
       iAccu += pcSourceData[ int(x - iFilterOverlap + i ) + iSourceStride * y  ] * piCoeffs[i];
     }
     pcDestData[x -iOffsetX + iDestStride * y ] =  (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift) ;
   }

   for (Int x = iStopX ; x < iSizeX + iOffsetX ; x++)   // uiSizeX < x - uiFilterOverlap + i
   {
     iAccu = 0 ;
     for (Int i = 0; i < iSizeX + iOffsetX + iFilterOverlap - x; i++)
     {
       iAccu += pcSourceData[ x - iFilterOverlap + i + iSourceStride * y  ] * piCoeffs[i];
     }
     for (Int i  = iSizeX + iOffsetX + iFilterOverlap-x; i < iFilterLength; i++)
     {
       iAccu += pcSourceData[ iSizeX + iOffsetX - 1 + iSourceStride * y  ] * piCoeffs[i];
     }
     pcDestData[x -iOffsetX + iDestStride * y ] =  (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift) ;
   }
  }

  }
  else if( rcFilter.bIsVertical )
  {


  Int iStartY = Max( iOffsetY , iFilterOverlap       );
  Int iStopY  = Min( iMaxY - iFilterOverlap , iSizeY + iOffsetY);

  for (Int y = iOffsetY; y < iFilterOverlap ; y++)
  {
   for (Int x = 0; x < iSizeX; x++)
   {
     iAccu = 0 ;
     for (Int i = 0; i < iFilterOverlap-y; i++)
     {
         iAccu += pcSourceData[ x ] * piCoeffs[i];
     }

     for (Int i = iFilterOverlap-y; i < iFilterLength; i++)
     {
         iAccu += pcSourceData[  x + iSourceStride * (y - iFilterOverlap + i ) ] * piCoeffs[i];
     }
     pcDestData[x + iDestStride * ( y - iOffsetY ) ] =  (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift) ;
   }
  }
  for (Int y = iStartY ; y < iStopY ; y++)
  {
   for (Int x = 0; x < iSizeX; x++)
   {
     iAccu = 0 ;
     for (Int i = 0; i < iFilterLength; i++)
     {
         iAccu += pcSourceData[  x + iSourceStride *  int(y - iFilterOverlap + i ) ] * piCoeffs[i];
     }
     pcDestData[x + iDestStride * ( y - iOffsetY ) ] = (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift) ;
   }
  }
  for (Int y = iStopY  ; y < iSizeY + iOffsetY ; y++)
  {
   for (Int x = 0; x < iSizeX; x++)
   {
     iAccu = 0 ;
     for (Int i = 0; i < iSizeY+ iOffsetY + iFilterOverlap-y; i++)
     {
         iAccu += pcSourceData[  x + iSourceStride * (y - iFilterOverlap + i ) ] * piCoeffs[i];
     }

     for (Int i = iSizeY+ iOffsetY + iFilterOverlap-y; i < iFilterLength; i++)
     {
         iAccu += pcSourceData[  x + iSourceStride * ( iHeight + iOffsetY - 1 ) ] * piCoeffs[i];
     }
     pcDestData[x + iDestStride * ( y - iOffsetY ) ] =  (Pel)Clip((iAccu  + iDcOffset + iAdd)>>iBitShift) ;
   }
  }
  }

}

Void TComAdaptiveLoopFilter::xApplyFrame(TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, AlfFilter& rcFilter, Int iPlane )
{
  Int iHeight = 0;
  Int iWidth  = 0;

  Pel* pDec = NULL;
  Int iDecStride = 0;

  Pel* pRest = NULL;
  Int iRestStride = 0;

  if (iPlane == 0)
  {
    iHeight     = pcPicRest->getHeight();
    iWidth      = pcPicRest->getWidth();

    pDec        = pcPicDec->getLumaAddr();
    iDecStride  = pcPicDec->getStride();

    pRest       = pcPicRest->getLumaAddr();
    iRestStride = pcPicRest->getStride();
  }
  else if (iPlane ==1)
  {
    iHeight     = pcPicRest->getHeight() >> 1;
    iWidth      = pcPicRest->getWidth()  >> 1;

    pDec        = pcPicDec->getCbAddr();
    iDecStride  = pcPicDec->getCStride();

    pRest       = pcPicRest->getCbAddr();
    iRestStride = pcPicRest->getCStride();
  }
  else if (iPlane ==2)
  {
    iHeight     = pcPicRest->getHeight() >> 1;
    iWidth      = pcPicRest->getWidth()  >> 1;

    pDec        = pcPicDec->getCrAddr();
    iDecStride  = pcPicDec->getCStride();

    pRest       = pcPicRest->getCrAddr();
    iRestStride = pcPicRest->getCStride();
  }
  xApplyFilter( pDec, pRest, iWidth, iHeight, iDecStride, iRestStride, rcFilter );

}


Void TComAdaptiveLoopFilter::xApplyFrame(TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, AlfFilter& rcFilter )
{

  Int iHeight = pcPicRest->getHeight();
  Int iWidth = pcPicRest->getWidth();

  Pel* pDec = pcPicDec->getLumaAddr();
  Int iDecStride = pcPicDec->getStride();

  Pel* pRest = pcPicRest->getLumaAddr();
  Int iRestStride = pcPicRest->getStride();

  xApplyFilter( pDec, pRest, iWidth, iHeight, iDecStride, iRestStride, rcFilter );




}




Void TComAdaptiveLoopFilter::xCopyDecToRestCUs(TComPicSym* pcQuadTree, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    xCopyDecToRestCU( pcQuadTree, pcCU, 0, 0, pcPicDec, pcPicRest);
  }
}

Void TComAdaptiveLoopFilter::xCopyDecToRestCU(TComPicSym* pcQuadTree, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  if( ( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() ) )
  {
    bBoundary = true;
  }

  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcQuadTree->getNumPartition() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xCopyDecToRestCU(pcQuadTree, pcCU, uiAbsPartIdx, uiDepth+1, pcPicDec, pcPicRest);
    }
    return;
  }

  if (!pcCU->getAlfCtrlFlag(uiAbsPartIdx))
  {
    UInt uiCUAddr = pcCU->getAddr();

    Int iWidth  = g_uiMaxCUWidth  >> uiDepth;
    Int iHeight = g_uiMaxCUHeight >> uiDepth;

    Pel* pRec = pcPicDec->getLumaAddr(uiCUAddr, uiAbsPartIdx);
    Pel* pFilt = pcPicRest->getLumaAddr(uiCUAddr, uiAbsPartIdx);

    Int iRecStride = pcPicDec->getStride();
    Int iFiltStride = pcPicRest->getStride();

    for (Int y = 0; y < iHeight; y++)
    {
      for (Int x = 0; x < iWidth; x++)
      {
        pFilt[x] = pRec[x];
      }
      pRec += iRecStride;
      pFilt += iFiltStride;
    }
  }
}


// --------------------------------------------------------------------------------------------------------------------
// ALF for chroma
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::xALFChroma(ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  if((pcAlfParam->chroma_idc>>1)&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam, 0);
  }

  if(pcAlfParam->chroma_idc&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam , 1);
  }
}

/** \param  pcPicDec      picture before ALF
    \param  pcPicRest     picture after  ALF
    \param  pcAlfParam    AlfParameter
    \param  iColor        0 for Cb and 1 for Cr
 */
Void TComAdaptiveLoopFilter::xFrameChroma( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, ALFParam* pcAlfParam , Int iColor )
{
  Int iHeight = pcPicRest->getHeight() >> 1;
  Int iWidth = pcPicRest->getWidth() >> 1;

  Pel* pDec;
  Int iDecStride = pcPicDec->getCStride();

  Pel* pRest;
  Int iRestStride = pcPicRest->getCStride();

  AlfFilter* pcAlfFilterHorizontal;
  AlfFilter* pcAlfFilterVertical;

  if (iColor)
  {
    pDec  = pcPicDec->getCrAddr();
    pRest = pcPicRest->getCrAddr();
    pcAlfFilterHorizontal = &pcAlfParam->acHorizontalAlfFilter[ 1 ];
    pcAlfFilterVertical   = &pcAlfParam->acVerticalAlfFilter[ 1 ];
  }
  else
  {
    pDec  = pcPicDec->getCbAddr();
    pRest = pcPicRest->getCbAddr();
    pcAlfFilterHorizontal = &pcAlfParam->acHorizontalAlfFilter[ ALF_FILT_FOR_CHROMA ];
    pcAlfFilterVertical   = &pcAlfParam->acVerticalAlfFilter  [ ALF_FILT_FOR_CHROMA ];
  }

  xApplyFilter( pDec, pRest, iWidth, iHeight, iDecStride, iRestStride, *pcAlfFilterVertical   );
  xApplyFilter( pDec, pRest, iWidth, iHeight, iDecStride, iRestStride, *pcAlfFilterHorizontal );
}
#else
// ====================================================================================================================
// Tables
// ====================================================================================================================
#if QC_ALF
Int TComAdaptiveLoopFilter::pattern9x9Sym[SQR_FILT_LENGTH_9] = 
{
                   0,
               1,  2,  3,
           4,  5,  6,  7,  8,
       9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 19, 18, 17, 16,
      15, 14, 13, 12, 11, 10,  9, 
           8,  7,  6,  5,  4,
               3,  2,  1,
		   0
};

Int TComAdaptiveLoopFilter::weights9x9Sym[SQR_FILT_LENGTH_9SYM] = 
{
                  2,
              2,  2,  2,   
           2,  2,  2,  2,  2, 
       2,  2,  2,  2,  2,  2,  2,  
   2,  2, 2,  2,  1,  1
};

Int TComAdaptiveLoopFilter::pattern9x9Sym_Quart[45] = 
{
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  9,  0,  0,  
   0, 10, 11, 12, 13, 14, 15, 16,  0,
  17, 18, 19, 20, 21, 22,  0,  0,  0
};

Int TComAdaptiveLoopFilter::pattern7x7Sym[SQR_FILT_LENGTH_7] = 
{
                   0,
               1,  2,  3,
           4,  5,  6,  7,  8,
       9, 10, 11, 12, 11, 10, 9,
           8,  7,  6,  5,  4,
               3,  2,  1,
   	           0
};

Int TComAdaptiveLoopFilter::weights7x7Sym[SQR_FILT_LENGTH_7SYM] = 
{
                  2,  
              2,  2,  2,   
	      2,  2,  2,  2,  2,    
      2,  2,  2,  1,  1
};

Int TComAdaptiveLoopFilter::pattern7x7Sym_Quart[45] = 
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  9,  0,  0,  
   0, 10, 11, 12, 13, 14,  0,  0,  0
};

Int TComAdaptiveLoopFilter::pattern5x5Sym[SQR_FILT_LENGTH_5] = 
{
                   0,
               1,  2,  3,
           4,  5,  6,  5,  4,
               3,  2,  1,
                   0
};

Int TComAdaptiveLoopFilter::weights5x5Sym[SQR_FILT_LENGTH_5SYM] = 
{
           2, 
        2, 2, 2,
     2, 2, 1, 1
};

Int TComAdaptiveLoopFilter::pattern5x5Sym_Quart[45] = 
{
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  0,  0,  0,  0,  0,
   0,  0,  0,  0,  1,  0,  0,  0,  0,
   0,  0,  0,  2,  3,  4,  0,  0,  0,
   0,  0,  5,  6,  7,  8,  0,  0,  0,  
};
Int TComAdaptiveLoopFilter::pattern9x9Sym_9[SQR_FILT_LENGTH_9] = 
{
                   4,  
              12, 13, 14,  
          20, 21, 22, 23, 24, 
      28, 29, 30, 31, 32, 33, 34,      
  36, 37, 38, 39, 40, 39, 38, 37, 36, 
      34, 33, 32, 31, 30, 29, 28,  
          24, 23, 22, 21, 20, 
              14, 13, 12, 
                   4,  
};

Int TComAdaptiveLoopFilter::pattern9x9Sym_7[SQR_FILT_LENGTH_7] = 
{    
               13,   
           21, 22, 23,  
       29, 30, 31, 32, 33,       
   37, 38, 39, 40, 39, 38, 37,  
       33, 32, 31, 30, 29,   
           23, 22, 21,  
               13  
                     
};

Int TComAdaptiveLoopFilter::pattern9x9Sym_5[SQR_FILT_LENGTH_5] = 
{
          22, 
      30, 31, 32,    
  38, 39, 40, 39, 38,
      32, 31, 30,
          22,
 };

Int* TComAdaptiveLoopFilter::patternTab_filt[NO_TEST_FILT]={pattern9x9Sym_9, pattern9x9Sym_7, pattern9x9Sym_5}; 
Int* TComAdaptiveLoopFilter::patternTab[NO_TEST_FILT]={pattern9x9Sym, pattern7x7Sym, pattern5x5Sym}; 
Int* TComAdaptiveLoopFilter::patternMapTab[NO_TEST_FILT]={pattern9x9Sym_Quart, pattern7x7Sym_Quart, pattern5x5Sym_Quart};
Int* TComAdaptiveLoopFilter::weightsTab[NO_TEST_FILT]={weights9x9Sym, weights7x7Sym, weights5x5Sym};
Int TComAdaptiveLoopFilter::flTab[NO_TEST_FILT]={9/2, 7/2, 5/2};
Int TComAdaptiveLoopFilter::sqrFiltLengthTab[NO_TEST_FILT]={SQR_FILT_LENGTH_9SYM, SQR_FILT_LENGTH_7SYM, SQR_FILT_LENGTH_5SYM};


Int depthInt9x9Sym[22] = 
{
              5, 
           5, 6, 5, 
        5, 6, 7, 6, 5,
     5, 6, 7, 8, 7, 6, 5,
  5, 6, 7, 8, 9, 9 
};

Int depthInt7x7Sym[14] = 
{
           4, 
        4, 5, 4, 
     4, 5, 6, 5, 4, 
  4, 5, 6, 7, 7 
};

Int depthInt5x5Sym[8] = 
{
        3,   
     3, 4, 3,
  3, 4, 5, 5  
};

Int* pDepthIntTab[NO_TEST_FILT]={ depthInt5x5Sym, depthInt7x7Sym, depthInt9x9Sym };


#if WIENER_3_INPUT
Int depthInt3x3Sym[SQR_FILT_LENGTH_3SYM] =
{
      5,
  5,  6,
};

Int depthInt1x1Sym[SQR_FILT_LENGTH_1SYM] =
{
      5
};

Int TComAdaptiveLoopFilter::pattern3x3Sym_Quart[6] =
{
    0,  1,  0,
    2,  3,  0,
};
Int TComAdaptiveLoopFilter::pattern1x1Sym_Quart[6] =
{
    0,  0,  0,
    0,  1,  0,
};
Int TComAdaptiveLoopFilter::pattern9x9Sym_3[SQR_FILT_LENGTH_3] =
{
      1,
  3,  4,  3,
      1,
};

Int TComAdaptiveLoopFilter::pattern9x9Sym_1[SQR_FILT_LENGTH_1] =
{
      4
};

Int TComAdaptiveLoopFilter::pattern3x3Sym[SQR_FILT_LENGTH_3] =
{
      0,
  1,  2,  1,
      0
};
Int TComAdaptiveLoopFilter::pattern1x1Sym[SQR_FILT_LENGTH_1] =
{
      0
};
Int TComAdaptiveLoopFilter::weights3x3Sym[SQR_FILT_LENGTH_3SYM] =
{
      2,
   2, 1,
};
Int TComAdaptiveLoopFilter::weights1x1Sym[SQR_FILT_LENGTH_1SYM] =
{
      1
};


Int* TComAdaptiveLoopFilter::patternTab_pr[NO_TEST_FILT+2]={pattern1x1Sym, pattern3x3Sym, pattern5x5Sym, pattern7x7Sym, pattern9x9Sym};
Int TComAdaptiveLoopFilter::sqrFiltLengthTab_pr[NO_TEST_FILT+2]={SQR_FILT_LENGTH_1SYM, SQR_FILT_LENGTH_3SYM, SQR_FILT_LENGTH_5SYM-1, SQR_FILT_LENGTH_7SYM-1, SQR_FILT_LENGTH_9SYM-1};
Int* TComAdaptiveLoopFilter::weightsTab_pr[NO_TEST_FILT+2]={weights1x1Sym, weights3x3Sym, weights5x5Sym, weights7x7Sym, weights9x9Sym};//DC may be to much
Int* pDepthIntTab_pr[NO_TEST_FILT+2]={ depthInt1x1Sym, depthInt3x3Sym, depthInt5x5Sym, depthInt7x7Sym, depthInt9x9Sym };//DC may be to much
Int* TComAdaptiveLoopFilter::patternMapTab_pr[NO_TEST_FILT+2]={pattern1x1Sym_Quart, pattern3x3Sym_Quart, pattern5x5Sym_Quart, pattern7x7Sym_Quart, pattern9x9Sym_Quart};
Int* TComAdaptiveLoopFilter::patternTab_filt_pr[NO_TEST_FILT+2]={pattern9x9Sym_1, pattern9x9Sym_3, pattern9x9Sym_5, pattern9x9Sym_7, pattern9x9Sym_9};
Int weights[MAX_SQR_FILT_LENGTH];
Int depth[MAX_SQR_FILT_LENGTH];
#endif


#endif
// scaling factor for quantization of filter coefficients (9x9)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag9x9[41] =
{
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (7x7)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag7x7[25] =
{
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2,
  2, 2, 2, 1
};

// scaling factor for quantization of filter coefficients (5x5)
const Int TComAdaptiveLoopFilter::m_aiSymmetricMag5x5[13] =
{
  2, 2, 2, 2, 2,
  2, 2, 2, 2, 2,
  2, 2, 1
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComAdaptiveLoopFilter::TComAdaptiveLoopFilter()
{
  m_pcTempPicYuv = NULL;
}
#if QC_ALF
Void TComAdaptiveLoopFilter:: error(const char *text, int code)
{
  fprintf(stderr, "%s\n", text);
  exit(code);
}


Void TComAdaptiveLoopFilter:: no_mem_exit(const char *where)
{
   char errortext[200];
   sprintf(errortext, "Could not allocate memory: %s",where);
   error (errortext, 100);
}


Void TComAdaptiveLoopFilter::initMatrix_imgpel(imgpel ***m2D, int d1, int d2)
{
  int i;
      
  if(!(*m2D = (imgpel **) calloc(d1, sizeof(imgpel *))))
    FATAL_ERROR_0("initMatrix_imgpel: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (imgpel *) calloc(d1 * d2, sizeof(imgpel))))
    FATAL_ERROR_0("initMatrix_imgpel: memory allocation problem\n", -1);

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}
Void TComAdaptiveLoopFilter::initMatrix_int(int ***m2D, int d1, int d2)
{
  int i;
      
  if(!(*m2D = (int **) calloc(d1, sizeof(int *))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (int *) calloc(d1 * d2, sizeof(int))))
    FATAL_ERROR_0("initMatrix_int: memory allocation problem\n", -1);

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}
Void TComAdaptiveLoopFilter::destroyMatrix_int(int **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_int: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_int: memory free problem\n", -1);
  }
}
Void TComAdaptiveLoopFilter::destroyMatrix_imgpel(imgpel **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_imgpel: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_imgpel: memory free problem\n", -1);
  }
}

Void TComAdaptiveLoopFilter::get_mem2Dpel(imgpel ***array2D, int rows, int columns)
{
  int i;
      
  if((*array2D      = (imgpel**)calloc(rows,        sizeof(imgpel*))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");
  if(((*array2D)[0] = (imgpel* )calloc(rows*columns,sizeof(imgpel ))) == NULL)
    no_mem_exit("get_mem2Dpel: array2D");

  for(i=1 ; i<rows ; i++)
    (*array2D)[i] =  (*array2D)[i-1] + columns  ;
}

Void TComAdaptiveLoopFilter::free_mem2Dpel(imgpel **array2D)
{
  if (array2D)
  {
    if (array2D[0])
      free (array2D[0]);
    else error ("free_mem2Dpel: trying to free unused memory",100);
      
    free (array2D);
  } else
  {
    error ("free_mem2Dpel: trying to free unused memory",100);
  }
}

Void TComAdaptiveLoopFilter::get_mem1Dint(int **array1D, int rows)
{
  int i;

  if((*array1D      = (int*)calloc(rows, sizeof(int))) == NULL)
    FATAL_ERROR_0("get_mem1Dint: array1D", -1);
  for(i=0; i<rows; i++)
    (*array1D)[i] = 0;
}

Void TComAdaptiveLoopFilter::free_mem1Dint(int *array1D)
{
  if (array1D)
    free (array1D);
  else
    FATAL_ERROR_0("free_mem1Dint: array1D", -1);
}


Void TComAdaptiveLoopFilter::initMatrix_double(double ***m2D, int d1, int d2)
{
  int i;
      
  if(!(*m2D = (double **) calloc(d1, sizeof(double *))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (double *) calloc(d1 * d2, sizeof(double))))
    FATAL_ERROR_0("initMatrix_double: memory allocation problem\n", -1);

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::initMatrix_ushort(unsigned short ***m2D, int d1, int d2)
{
  int i;
      
  if(!(*m2D = (unsigned short **) calloc(d1, sizeof(unsigned short *))))
    FATAL_ERROR_0("initMatrix_ushort: memory allocation problem\n", -1);
  if(!((*m2D)[0] = (unsigned short *) calloc(d1 * d2, sizeof(unsigned short))))
    FATAL_ERROR_0("initMatrix_ushort: memory allocation problem\n", -1);

  for(i = 1; i < d1; i++)
    (*m2D)[i] = (*m2D)[i-1] + d2;
}

Void TComAdaptiveLoopFilter::initMatrix3D_double(double ****m3D, int d1, int d2, int d3)
{
  int  j;
      
  if(!((*m3D) = (double ***) calloc(d1, sizeof(double **))))
    FATAL_ERROR_0("initMatrix3D_double: memory allocation problem\n", -1);

  for(j = 0; j < d1; j++)
    initMatrix_double((*m3D) + j, d2, d3);
}


Void TComAdaptiveLoopFilter::initMatrix4D_double(double *****m4D, int d1, int d2, int d3, int d4)
{
  int  j;

  if(!((*m4D) = (double ****) calloc(d1, sizeof(double ***))))
    FATAL_ERROR_0("initMatrix4D_double: memory allocation problem\n", -1);

  for(j = 0; j < d1; j++)
    initMatrix3D_double((*m4D) + j, d2, d3, d4);
}


Void TComAdaptiveLoopFilter::destroyMatrix_double(double **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_double: memory free problem\n", -1);
  }
}

Void TComAdaptiveLoopFilter::destroyMatrix_ushort(unsigned short **m2D)
{
  if(m2D)
  {
    if(m2D[0])
      free(m2D[0]);
    else
      FATAL_ERROR_0("destroyMatrix_ushort: memory free problem\n", -1);
    free(m2D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix_ushort: memory free problem\n", -1);
  }
}

Void TComAdaptiveLoopFilter::destroyMatrix3D_double(double ***m3D, int d1)
{
  int i;

  if(m3D)
  {
    for(i = 0; i < d1; i++)
      destroyMatrix_double(m3D[i]);
    free(m3D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix3D_double: memory free problem\n", -1);
  }
}


Void TComAdaptiveLoopFilter::destroyMatrix4D_double(double ****m4D, int d1, int d2)
{
  int  j;

  if(m4D)
  {
    for(j = 0; j < d1; j++)
      destroyMatrix3D_double(m4D[j], d2);
    free(m4D);
  } 
  else
  {
    FATAL_ERROR_0("destroyMatrix4D_double: memory free problem\n", -1);
  }
}

#endif

Void TComAdaptiveLoopFilter::create( Int iPicWidth, Int iPicHeight, UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxCUDepth )
{
  if ( !m_pcTempPicYuv )
  {
    m_pcTempPicYuv = new TComPicYuv;
    m_pcTempPicYuv->create( iPicWidth, iPicHeight, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth );
  }
#if QC_ALF
  img_height = iPicHeight;
  img_width = iPicWidth;
  initMatrix_imgpel(&imgY_var, img_height, img_width); 
  initMatrix_imgpel(&ImgDec, img_height, img_width); 
  initMatrix_imgpel(&ImgRest, img_height, img_width); 
  initMatrix_imgpel(&imgY_pad, img_height+2*(FILTER_LENGTH/2), img_width+2*(FILTER_LENGTH/2));
  initMatrix_int(&imgY_temp, img_height+2*VAR_SIZE+3, img_width+2*VAR_SIZE+3);

  initMatrix_int(&filterCoeffSym, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
  initMatrix_int(&filterCoeffPrevSelected, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
  initMatrix_int(&filterCoeffTmp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);      
  initMatrix_int(&filterCoeffSymTmp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);   
#if WIENER_3_INPUT
  initMatrix_imgpel(&ImgResi, img_height, img_width);
  initMatrix_imgpel(&ImgPred, img_height, img_width);
  initMatrix_imgpel(&imgYr_pad, img_height+2*(FILTER_LENGTH/2), img_width+2*(FILTER_LENGTH/2));
  initMatrix_imgpel(&imgYp_pad, img_height+2*(FILTER_LENGTH/2), img_width+2*(FILTER_LENGTH/2));
#endif

#endif
}

Void TComAdaptiveLoopFilter::destroy()
{
	if ( m_pcTempPicYuv )
	{
          m_pcTempPicYuv->destroy();
	  delete m_pcTempPicYuv;
	}
#if QC_ALF
	destroyMatrix_imgpel(imgY_var); 
	destroyMatrix_imgpel(imgY_pad);
	destroyMatrix_imgpel(ImgDec); 
	destroyMatrix_imgpel(ImgRest);
	destroyMatrix_int(imgY_temp);

	destroyMatrix_int(filterCoeffSym);
  destroyMatrix_int(filterCoeffPrevSelected);
  destroyMatrix_int(filterCoeffTmp);
  destroyMatrix_int(filterCoeffSymTmp);
#if WIENER_3_INPUT
  destroyMatrix_imgpel(imgYr_pad);
  destroyMatrix_imgpel(imgYp_pad);
  destroyMatrix_imgpel(ImgResi);
	destroyMatrix_imgpel(ImgPred); 
#endif
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// allocate / free / copy functions
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::allocALFParam(ALFParam* pAlfParam)
{
#if (QC_ALF && WIENER_3_INPUT)
  Int qtable [FILTER_PRECISION_TABLE_NUMBER]={8,16,32,64,128,256,512,1024,2048,4096,8192};
  Int qtable2[FILTER_PRECISION_TABLE_NUMBER]={3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13};
  Int qtable3[FILTER_PRECISION_TABLE_NUMBER]={4, 8,16,32, 64,128,256, 512,1024,2048,4096};
  
  for (Int i=0; i<FILTER_PRECISION_TABLE_NUMBER; i++)
  {
    pAlfParam->filter_precision_table      [i] = qtable [i];
    pAlfParam->filter_precision_table_shift[i] = qtable2[i];
    pAlfParam->filter_precision_table_half [i] = qtable3[i];
  }
#endif  
  
  pAlfParam->alf_flag = 0;

  pAlfParam->coeff                              = new Int[ALF_MAX_NUM_COEF];
  pAlfParam->coeff_chroma = new Int[ALF_MAX_NUM_COEF_C];

  ::memset(pAlfParam->coeff,                            0, sizeof(Int)*ALF_MAX_NUM_COEF         );
  ::memset(pAlfParam->coeff_chroma, 0, sizeof(Int)*ALF_MAX_NUM_COEF_C   );
#if QC_ALF
  pAlfParam->coeffmulti = new Int*[NO_VAR_BINS];
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    pAlfParam->coeffmulti[i] = new Int[ALF_MAX_NUM_COEF];
    ::memset(pAlfParam->coeffmulti[i],                          0, sizeof(Int)*ALF_MAX_NUM_COEF         );
  }
#endif  
}

Void TComAdaptiveLoopFilter::freeALFParam(ALFParam* pAlfParam)
{
  assert(pAlfParam != NULL);
  if (pAlfParam->coeff != NULL)
  {
    delete[] pAlfParam->coeff;
    pAlfParam->coeff = NULL;
  }

  if (pAlfParam->coeff_chroma != NULL)
  {
    delete[] pAlfParam->coeff_chroma;
    pAlfParam->coeff_chroma = NULL;
  }
#if QC_ALF
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    delete[] pAlfParam->coeffmulti[i];
    pAlfParam->coeffmulti[i] = NULL;
  }
  delete[] pAlfParam->coeffmulti;
  pAlfParam->coeffmulti = NULL;
#endif
}

Void TComAdaptiveLoopFilter::copyALFParam(ALFParam* pDesAlfParam, ALFParam* pSrcAlfParam)
{
  pDesAlfParam->alf_flag = pSrcAlfParam->alf_flag;
  pDesAlfParam->cu_control_flag = pSrcAlfParam->cu_control_flag;
  pDesAlfParam->chroma_idc = pSrcAlfParam->chroma_idc;
  pDesAlfParam->tap = pSrcAlfParam->tap;
  pDesAlfParam->num_coeff = pSrcAlfParam->num_coeff;
  pDesAlfParam->tap_chroma = pSrcAlfParam->tap_chroma;
  pDesAlfParam->num_coeff_chroma = pSrcAlfParam->num_coeff_chroma;

  ::memcpy(pDesAlfParam->coeff, pSrcAlfParam->coeff, sizeof(Int)*ALF_MAX_NUM_COEF);
  ::memcpy(pDesAlfParam->coeff_chroma, pSrcAlfParam->coeff_chroma, sizeof(Int)*ALF_MAX_NUM_COEF_C);
#if QC_ALF
  pDesAlfParam->realfiltNo = pSrcAlfParam->realfiltNo;
  pDesAlfParam->filtNo = pSrcAlfParam->filtNo;
#if WIENER_3_INPUT
  pDesAlfParam->filter_precision[0] = pSrcAlfParam->filter_precision[0];
  pDesAlfParam->filter_precision[1] = pSrcAlfParam->filter_precision[1];
  pDesAlfParam->filter_precision[2] = pSrcAlfParam->filter_precision[2];
  pDesAlfParam->num_coeff_resi = pSrcAlfParam->num_coeff_resi;
  pDesAlfParam->num_coeff_pred = pSrcAlfParam->num_coeff_pred;
  pDesAlfParam->tap_pred = pSrcAlfParam->tap_pred;
  pDesAlfParam->tap_resi = pSrcAlfParam->tap_resi;
#endif
  ::memcpy(pDesAlfParam->filterPattern, pSrcAlfParam->filterPattern, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->startSecondFilter = pSrcAlfParam->startSecondFilter;
  pDesAlfParam->noFilters = pSrcAlfParam->noFilters;

  //Coeff send related
  pDesAlfParam->filters_per_group_diff = pSrcAlfParam->filters_per_group_diff; //this can be updated using codedVarBins
  pDesAlfParam->filters_per_group = pSrcAlfParam->filters_per_group; //this can be updated using codedVarBins
  ::memcpy(pDesAlfParam->codedVarBins, pSrcAlfParam->codedVarBins, sizeof(Int)*NO_VAR_BINS);
  pDesAlfParam->forceCoeff0 = pSrcAlfParam->forceCoeff0;
  pDesAlfParam->predMethod = pSrcAlfParam->predMethod;
  for (int i=0; i<NO_VAR_BINS; i++)
  {
    ::memcpy(pDesAlfParam->coeffmulti[i], pSrcAlfParam->coeffmulti[i], sizeof(Int)*ALF_MAX_NUM_COEF);
  }

#endif
}

// --------------------------------------------------------------------------------------------------------------------
// prediction of filter coefficients
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::predictALFCoeff( ALFParam* pAlfParam)
{
  Int i, sum, pred, tap, N;
  const Int* pFiltMag = NULL;

  tap = pAlfParam->tap;
  
  switch(tap)
  {
  case 5:
    pFiltMag = m_aiSymmetricMag5x5;
    break;
  case 7:
    pFiltMag = m_aiSymmetricMag7x7;
    break;
  case 9:
    pFiltMag = m_aiSymmetricMag9x9;
    break;
  default:
    assert(0);
    break;
  }
  N = (tap*tap+1)>>1;
  sum=0;
  for(i=0; i<N-1;i++)
  {
    sum+=pFiltMag[i]*pAlfParam->coeff[i];
  }
  pred=(1<<ALF_NUM_BIT_SHIFT)-sum;
  pAlfParam->coeff[N-1]=pred-pAlfParam->coeff[N-1];
}

Void TComAdaptiveLoopFilter::predictALFCoeffChroma( ALFParam* pAlfParam )
{
  Int i, sum, pred, tap, N;
  const Int* pFiltMag = NULL;
  
  tap = pAlfParam->tap_chroma;
  switch(tap)
  {
  case 5:
    pFiltMag = m_aiSymmetricMag5x5;
    break;
  case 7:
    pFiltMag = m_aiSymmetricMag7x7;
    break;
  case 9:
    pFiltMag = m_aiSymmetricMag9x9;
    break;
  default:
    assert(0);
    break;
  }
  N = (tap*tap+1)>>1;
  sum=0;
  for(i=0; i<N;i++)
  {
    sum+=pFiltMag[i]*pAlfParam->coeff_chroma[i];
  }
  pred=(1<<ALF_NUM_BIT_SHIFT)-(sum-pAlfParam->coeff_chroma[N-1]);
  pAlfParam->coeff_chroma[N-1]=pred-pAlfParam->coeff_chroma[N-1];
}

// --------------------------------------------------------------------------------------------------------------------
// interface function for actual ALF process
// --------------------------------------------------------------------------------------------------------------------

/**
		\param	pcPic					picture (TComPic) class (input/output)
		\param	pcAlfParam		ALF parameter
		\todo   for temporal buffer, it uses residual picture buffer, which may not be safe. Make it be safer.
 */
Void TComAdaptiveLoopFilter::ALFProcess(TComPic* pcPic, ALFParam* pcAlfParam)
{
  if(!pcAlfParam->alf_flag)
  {
    return;
  }

  TComPicYuv* pcPicYuvRec    = pcPic->getPicYuvRec();
  TComPicYuv* pcPicYuvExtRec = m_pcTempPicYuv;
#if WIENER_3_INPUT
  TComPicYuv* pcPicYuvPred   = pcPic->getPicYuvP();
  TComPicYuv* pcPicYuvResi   = pcPic->getPicYuvQ();
#endif

  pcPicYuvRec->copyToPic	        ( pcPicYuvExtRec );
  pcPicYuvExtRec->setBorderExtension	( false );
  pcPicYuvExtRec->extendPicBorder	();
  
#if QC_ALF
 
#if WIENER_3_INPUT
  xALFLuma_qc(pcPic, pcAlfParam, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvPred, pcPicYuvResi);
#else
  xALFLuma_qc(pcPic, pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
#endif
#else
  predictALFCoeff(pcAlfParam);
  xALFLuma(pcPic, pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
#endif

  if(pcAlfParam->chroma_idc)
  {
    predictALFCoeffChroma(pcAlfParam);
    xALFChroma( pcAlfParam, pcPicYuvExtRec, pcPicYuvRec);
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// ALF for luma
// --------------------------------------------------------------------------------------------------------------------
#if QC_ALF

#if WIENER_3_INPUT
Void TComAdaptiveLoopFilter::xALFLuma_qc(TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi)
#else
Void TComAdaptiveLoopFilter::xALFLuma_qc(TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
#endif    
{
  Int  Height = pcPicDec->getHeight();
  Int    Width = pcPicDec->getWidth();
  Int    LumaStride = pcPicDec->getStride();
  Pel* pDec = pcPicDec->getLumaAddr();
  Pel* pRest = pcPicRest->getLumaAddr();
#if WIENER_3_INPUT
  Pel* pResi = pcPicResi->getLumaAddr();
  Pel* pPred = pcPicPred->getLumaAddr();
  int fl=FILTER_LENGTH/2;
#endif  
  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
    {
      ImgDec[i][j] = ImgRest[i][j] = pDec[j + i*LumaStride];
#if WIENER_3_INPUT      
      ImgResi[i][j] = pResi[j + i*LumaStride];
      ImgPred[i][j] = pPred[j + i*LumaStride];
#endif
    }
    
#if WIENER_3_INPUT      
  padImage(ImgResi, imgYr_pad, fl, img_height, img_width);
  padImage(ImgPred, imgYp_pad, fl, img_height, img_width);
#endif
  
  //Decode and reconst filter coefficients
  DecFilter_qc(ImgDec,pcAlfParam);

  //set maskImg using cu adaptive one.
  if(pcAlfParam->cu_control_flag)
  {
    xCUAdaptive_qc(pcPic, pcAlfParam, ImgRest, ImgDec);
  }  
  else
  {
    //then do whole frame filtering
#if WIENER_3_INPUT      
    filterFrame(ImgRest, ImgDec, pcAlfParam->realfiltNo, (pcAlfParam->tap_resi-1)/2, (pcAlfParam->tap_pred-1)/2,
                pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[0]],
                pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[1]],
                pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[2]]);
#else
    filterFrame(ImgRest, ImgDec, pcAlfParam->realfiltNo);
#endif
  }

  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
    {
      pRest[j + i*LumaStride] = ImgRest[i][j];
    }
}


Void TComAdaptiveLoopFilter::DecFilter_qc(imgpel** imgY_rec,ALFParam* pcAlfParam)
{  
  int i;
  int numBits = NUM_BITS; 
  int fl=FILTER_LENGTH/2;
  int **pfilterCoeffSym;
  pfilterCoeffSym= filterCoeffSym;

  if(pcAlfParam->filtNo>=0)
  {
    //// Reconstruct filter coefficients
    reconstructFilterCoeffs( pcAlfParam, pfilterCoeffSym, numBits);
  }
  else
  {
    for(i = 0; i < NO_VAR_BINS; i++)
    {
      pcAlfParam->varIndTab[i]=0;
      memset(pfilterCoeffSym[i],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
    }
  }
  getCurrentFilter(pfilterCoeffSym,pcAlfParam);
	
  padImage(imgY_rec, imgY_pad, fl, img_height, img_width);
  memset(imgY_temp[0],0,sizeof(int)*(img_height+2*VAR_SIZE)*(img_width+2*VAR_SIZE));

  calcVar(imgY_var, imgY_pad, fl, VAR_SIZE, img_height, img_width);
}

Void TComAdaptiveLoopFilter::padImage(imgpel **imgY,  imgpel **imgY_pad, int fl, int img_height, int img_width)
{
  int x, y;
  int x_size = img_width + fl;
  int x_size2= img_width + 2 * fl;
  int y_size = img_height + fl;
  int y_size2= img_height + 2 * fl;
  for(y = 0; y < img_height; ++y)
  {
    memcpy(imgY_pad[y+fl]+fl,imgY[y],sizeof(imgpel)* img_width);
  }
  for(x = fl; x < img_width + fl; ++x) 
  {
    imgpel temp=imgY_pad[fl][x];
    for(y = 0; y < fl; ++y)
    {
      imgY_pad[y][x] = temp;
    }
    temp=imgY_pad[y_size - 1][x];
    for(y = y_size; y < y_size2; ++y)
      imgY_pad[y][x] = temp;
  }
  
  for(y = 0; y < img_height + 2 * fl; ++y)
  {
    imgpel temp=imgY_pad[y][fl];
    imgpel *p_img= imgY_pad[y];
    for(x = 0; x < fl; ++x)
      *(p_img++) = temp;
    temp=imgY_pad[y][x_size - 1];
    p_img= &(imgY_pad[y][x_size]);
    for(x = x_size; x < x_size2; ++x)
      *(p_img++)  = temp;
  }
}

Void TComAdaptiveLoopFilter::getCurrentFilter(int **filterCoeffSym,ALFParam* pcAlfParam)
{ 
  int i,  k, varInd;
  int *patternMap;
#if WIENER_3_INPUT
  int *patternMap_resi, *patternMap_pred;
  int length = (FILTER_LENGTH*FILTER_LENGTH)/2 + 1;
  int length_pred = (FILTER_LENGTH_PRED*FILTER_LENGTH_PRED)/2 + 1;
  int length_resi = (FILTER_LENGTH_RESI*FILTER_LENGTH_RESI)/2 + 1;
  int filtNo_resi = (pcAlfParam->tap_resi-1)/2;
  int filtNo_pred = (pcAlfParam->tap_pred-1)/2;
  int filtNo = pcAlfParam->realfiltNo;
#else   
  int *patternMapTab[3]={pattern9x9Sym_Quart, pattern7x7Sym_Quart, pattern5x5Sym_Quart};
#endif
  for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
  {		
    memset(filterCoeffPrevSelected[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
  }
#if WIENER_3_INPUT  
  if (filtNo_resi<2)
    length_resi = (3*3)/2 + 1;
  if (filtNo_pred<2)
    length_pred = (3*3)/2 + 1;
  
  patternMap      = patternMapTab   [filtNo];
  patternMap_resi = patternMapTab_pr[filtNo_resi];
  patternMap_pred = patternMapTab_pr[filtNo_pred];
  // filter for reconstructions
  for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
  {		
    k=0;
    for(i = 0; i < length; i++)
    {
      if (patternMap[i]>0)
      {
        filterCoeffPrevSelected[varInd][i]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
        k++;
      }
      else
      {
        filterCoeffPrevSelected[varInd][i]=0;
      }
    }
    for(i = 0; i < length_resi; i++)
    {
      if (patternMap_resi[i]>0)
      {
        filterCoeffPrevSelected[varInd][i+length]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
        k++;
      }
      else
      {
        filterCoeffPrevSelected[varInd][i+length]=0;
      }
    }
    for(i = 0; i < length_pred; i++)
    {
      if (patternMap_pred[i]>0)
      {
        filterCoeffPrevSelected[varInd][i+length+length]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
        k++;
      }
      else
      {
        filterCoeffPrevSelected[varInd][i+length+length]=0;
      }
    }
     
    filterCoeffPrevSelected[varInd][MAX_SQR_FILT_LENGTH-1]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];//DC
  }
#else
    patternMap=patternMapTab[pcAlfParam->realfiltNo];
    for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
    {		
      k=0;
      for(i = 0; i < MAX_SQR_FILT_LENGTH; i++)
      {
        if (patternMap[i]>0)
        {
          filterCoeffPrevSelected[varInd][i]=filterCoeffSym[pcAlfParam->varIndTab[varInd]][k];
          k++;
        }
	else
	{
          filterCoeffPrevSelected[varInd][i]=0;
        }
      }
    }
#endif
}

Void TComAdaptiveLoopFilter::reconstructFilterCoeffs(ALFParam* pcAlfParam,int **pfilterCoeffSym, int bit_depth)
{
  int i, src, ind;
#if WIENER_3_INPUT
  int mid_rec =pcAlfParam->num_coeff-pcAlfParam->num_coeff_pred-pcAlfParam->num_coeff_resi-2;
  int mid_pred=mid_rec+pcAlfParam->num_coeff_resi+pcAlfParam->num_coeff_pred;
  int mid_resi=mid_rec+pcAlfParam->num_coeff_resi;
  int one   =1<<(pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[0]]);
  int bits_rec =(pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[0]]);
  int bits_pred=(pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[1]]);
  int bits_resi=(pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[2]]);
#endif  

  // Copy non zero filters in filterCoeffTmp
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    for(i = 0; i < pcAlfParam->num_coeff; i++)
      filterCoeffTmp[ind][i] = pcAlfParam->coeffmulti[ind][i];
  }
  // Undo prediction
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    if((!pcAlfParam->predMethod) || (ind == 0)) 
    {
      memcpy(filterCoeffSymTmp[ind],filterCoeffTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
    }
    else
    {
      // Prediction
      for(i = 0; i < pcAlfParam->num_coeff; ++i)
        filterCoeffSymTmp[ind][i] = (int)(filterCoeffTmp[ind][i] + filterCoeffSymTmp[ind - 1][i]);
    }
  }
  
#if WIENER_3_INPUT
  for(ind = 0; ind < pcAlfParam->filters_per_group_diff; ++ind)
  {
    filterCoeffSymTmp[ind][mid_rec ]+=one;
    filterCoeffSymTmp[ind][mid_pred]+=((one-filterCoeffSymTmp[ind][mid_rec]) * (1<<bits_pred))/(1<<bits_rec );
    filterCoeffSymTmp[ind][mid_resi]+=(filterCoeffSymTmp[ind][mid_pred]      * (1<<bits_resi))/(1<<bits_pred);
  }
#endif
  
  // Inverse quantization
  // Add filters forced to zero
  if(pcAlfParam->forceCoeff0)
  {
    assert(pcAlfParam->filters_per_group_diff < pcAlfParam->filters_per_group);
    src = 0;
    for(ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
    {
      if(pcAlfParam->codedVarBins[ind])
      {
        memcpy(pfilterCoeffSym[ind],filterCoeffSymTmp[src],sizeof(int)*pcAlfParam->num_coeff);
        ++src;
      }
      else
      {
        memset(pfilterCoeffSym[ind],0,sizeof(int)*pcAlfParam->num_coeff);
      }
    }
    assert(src == pcAlfParam->filters_per_group_diff);
  }
  else
  {
    assert(pcAlfParam->filters_per_group_diff == pcAlfParam->filters_per_group);
    for(ind = 0; ind < pcAlfParam->filters_per_group; ++ind)
      memcpy(pfilterCoeffSym[ind],filterCoeffSymTmp[ind],sizeof(int)*pcAlfParam->num_coeff);
  }
}


static imgpel Clip_post(int high, int val)
{
  return (imgpel)(((val > high)? high: val));
}

Void TComAdaptiveLoopFilter::calcVar(imgpel **imgY_var, imgpel **imgY_pad,int pad_size, int fl, int img_height, int img_width)
{
  int i, j, ii, jj;
  int sum;
  int *p_imgY_temp;
  int shift= (11+ g_uiBitIncrement);
  int fl2plusOne= (VAR_SIZE<<1)+1;
  int pad_offset = pad_size-fl-1;
  int var_max= NO_VAR_BINS-1;
  int mult_fact_int_tab[4]= {1,114,41,21};
  int mult_fact_int = mult_fact_int_tab[VAR_SIZE];


  if (VAR_SIZE ==0)
  {
    imgpel *p_imgY_var;
    shift = g_uiBitIncrement;
    //current
    for(i = 1; i < img_height + fl2plusOne; i++)
    {
      imgpel *p_imgY_pad = &imgY_pad[pad_offset+i][pad_offset];
      imgpel *p_imgY_pad_up   = &imgY_pad[pad_offset+i+1][pad_offset];
      imgpel *p_imgY_pad_down = &imgY_pad[pad_offset+i-1][pad_offset];
      p_imgY_temp = (int*)&imgY_temp[i-1][0];
      p_imgY_var  = &imgY_var [i-1][0];
      for(j = 1; j < img_width +fl2plusOne; j++)	
      {
        *(p_imgY_temp) = abs((p_imgY_pad[j]<<1) - p_imgY_pad[j+1] - p_imgY_pad[j-1])+
                        abs((p_imgY_pad[j]<<1) - p_imgY_pad_down[j] - p_imgY_pad_up[j]);
                        *(p_imgY_var++) =(imgpel) Clip_post(var_max, (int) ((*(p_imgY_temp++))>>shift));
      }
    }
    return;
  }
  
//current
  for(i = 1; i < img_height + fl2plusOne; i++)
  {
    imgpel *p_imgY_pad = &imgY_pad[pad_offset+i][pad_offset];
    imgpel *p_imgY_pad_up   = &imgY_pad[pad_offset+i+1][pad_offset];
    imgpel *p_imgY_pad_down = &imgY_pad[pad_offset+i-1][pad_offset];
    p_imgY_temp = (int*)&imgY_temp[i-1][0];
    for(j = 1; j < img_width +fl2plusOne; j++)	
    {
      *(p_imgY_temp++) = abs((p_imgY_pad[j]<<1) - p_imgY_pad[j+1] - p_imgY_pad[j-1])+
                         abs((p_imgY_pad[j]<<1) - p_imgY_pad_down[j] - p_imgY_pad_up[j]);
    }
  }
  {
    //int temp;
    int sum_0=0;
    int y=img_height+((VAR_SIZE+1)<<1);
    
    int *p_imgY_temp_sum;
    i = fl;
    j = fl;

    memset(imgY_temp[y],0,sizeof(int)*(img_width+((VAR_SIZE+1)<<1)));
    //--------------------------------------------------------------------------------------------

    for(ii = i - fl; ii <= i + fl; ii++)
    {
      p_imgY_temp= (int*)&imgY_temp[ii][j - fl];
      p_imgY_temp_sum=(int*)&imgY_temp[y][j - fl];
      for(jj = j - fl; jj <= j + fl; jj++)
      {
        *(p_imgY_temp_sum++) += *(p_imgY_temp++);
      }
    }
    p_imgY_temp_sum=(int*)&imgY_temp[y][j - fl];
    for(jj = j - fl; jj <= j + fl; jj++)
      sum_0+=*(p_imgY_temp_sum++);
    imgY_var[i - fl][j - fl] = (imgpel) Clip_post(var_max, (int) ((sum_0 * mult_fact_int)>>shift));
    //--------------------------------------------------------------------------------------------
    sum = sum_0;
    for(j = fl+1; j < img_width + fl; ++j)
    {	
      int k=j+fl;
      for(ii = i - fl; ii <= i + fl; ii++)
        imgY_temp[y][k] += (imgY_temp[ii][k]);
      
      sum += (imgY_temp[y][k]-imgY_temp[y][j - fl-1]);
      imgY_var[i - fl][j - fl] = (imgpel) Clip_post(var_max, (int) ((sum * mult_fact_int)>>shift));

    }
    //--------------------------------------------------------------------------------------------

    for(i = fl+1; i < img_height + fl; ++i)
    {
      imgpel  *pimgY_var= &imgY_var[i-fl][0];
      int *p_imgY_temp1;
      int *p_imgY_temp2;
      sum = sum_0;	
      j= fl;
      p_imgY_temp1= (int*)&imgY_temp[i+fl  ][j - fl];
      p_imgY_temp2= (int*)&imgY_temp[i-fl-1][j - fl];
      p_imgY_temp_sum=(int*)&imgY_temp[y][j - fl];
      for(jj = j - fl; jj <= j + fl; jj++)
      {
        int diff = *(p_imgY_temp1++)-*(p_imgY_temp2++);
        *(p_imgY_temp_sum++) += diff;
        sum += diff;
      }
      sum_0=sum;

      *(pimgY_var++) = (imgpel) Clip_post(var_max, (int) ((sum * mult_fact_int)>>shift));
      //--------------------------------------------------------------------------------------------
      p_imgY_temp_sum=(int*)imgY_temp[y];
      for(j = fl+1; j < img_width + fl; ++j)
      {	
        int k = j+fl;
        p_imgY_temp_sum[k] += (imgY_temp[i + fl][k]-imgY_temp[i-fl-1][k]);
        sum += (p_imgY_temp_sum[k]-p_imgY_temp_sum[j-fl-1]);
        *(pimgY_var++) = (imgpel) Clip_post(var_max, (int) ((sum * mult_fact_int)>>shift));
      }
    }
  }
}


#if WIENER_3_INPUT
Void TComAdaptiveLoopFilter::filterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int filtNo_resi, int filtNo_pred, int bits_rec, int bits_pred, int bits_resi)
#else    
Void TComAdaptiveLoopFilter::filterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo)
#endif
{
  int i, j, ii, jj, pixelInt,m=0;
  imgpel *p_imgY_var,*p_imgY_pad;
  int max_val=g_uiIBDI_MAX;
  int fl=FILTER_LENGTH/2;
  int *pattern=pattern9x9Sym;
  int fl_temp;
  int last_coef= MAX_SQR_FILT_LENGTH-1;
  imgpel *im1,*im2;
  int *coef;
#if WIENER_3_INPUT
  imgpel *p_imgYr_pad, *p_imgYp_pad;
  int offset     = (1<<(bits_rec-1));
  int shift_pred = (1<<(bits_rec - bits_pred));
  int shift_resi = (1<<(bits_rec - bits_resi));
  int num_bits_minus_1= bits_rec;
  int flTab_pr[5] = {0,1,2,3,4};
  int length = 0;
  int coef_new[NO_VAR_BINS][MAX_SQR_FILT_LENGTH];
  int *cp, *cp1;
#else    
  int num_bits_minus_1= NUM_BITS-1;
  int offset = (1<<(NUM_BITS-2));
  int *pattern_fix=patternTab_filt[filtNo];
  fl_temp=flTab[filtNo];  
#endif

#if WIENER_3_INPUT
  for (j=0;j<NO_VAR_BINS;j++)
  {
    cp  = coef_new[j];
    cp1 = filterCoeffPrevSelected[j];
    
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++;
    }
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++ * shift_resi;
    }
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++ * shift_pred;
    }
    *cp++=*cp1++;
  }
#endif  
  // Filter
  for (i = fl; i < img_height+fl; i++)
  {
    p_imgY_var = imgY_var[i-fl];
    p_imgY_pad = imgY_pad[i];
#if WIENER_3_INPUT  
    p_imgYr_pad = imgYr_pad[i];
    p_imgYp_pad = imgYp_pad[i];
#endif    
    for (j = fl; j < img_width+fl; j++)
    {
#if WIENER_3_INPUT  
      coef = coef_new[*(p_imgY_var++)];
#else      
      coef = filterCoeffPrevSelected[*(p_imgY_var++)];
#endif
      pixelInt=coef[last_coef];
#if WIENER_3_INPUT
      // reconstruction
      fl_temp=flTab[filtNo];
      pattern=patternTab_filt[filtNo];
#else      
      pattern=pattern_fix;
#endif
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgY_pad[i+ii][j-m]);
        im2= &(imgY_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
        {
          pixelInt+=((*(im1++)+ *(im2--))*coef[*(pattern++)]);
        }          
      }
  
      im1= &(p_imgY_pad[j-fl_temp]);
      im2= &(p_imgY_pad[j+fl_temp]);	
      for (jj=0; jj<fl_temp; jj++)
      {
        pixelInt+=((*(im1++)+ *(im2--))*coef[*(pattern++)]);
      }
      
      pixelInt+=(p_imgY_pad[j]*coef[*(pattern++)]);
#if WIENER_3_INPUT  
      // residual
      length = (FILTER_LENGTH*FILTER_LENGTH)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_resi];
      fl_temp=flTab_pr[filtNo_resi];
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgYr_pad[i+ii][j-m]);
        im2= &(imgYr_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
        {
          pixelInt+=(( (Int)(*(im1++)+ *(im2--))-(Int)(2*g_uiIBDI_MAX_Q) )*coef[(*(pattern++))+length]);
        }
      }

      im1= &(p_imgYr_pad[j-fl_temp]);
      im2= &(p_imgYr_pad[j+fl_temp]);
      for (jj=0; jj<fl_temp; jj++)
      {
        pixelInt+=(( (Int)(*(im1++)+ *(im2--))-(Int)(2*g_uiIBDI_MAX_Q) )*coef[(*(pattern++))+length]);
      }
      
      pixelInt+=(((Int)p_imgYr_pad[j] - (Int)(g_uiIBDI_MAX_Q))*coef[(*(pattern++))+length]);

      // prediction
      length += (FILTER_LENGTH_RESI*FILTER_LENGTH_RESI)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_pred];
      fl_temp=flTab_pr[filtNo_pred];
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgYp_pad[i+ii][j-m]);
        im2= &(imgYp_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
        {
          pixelInt+=((*(im1++)+ *(im2--))*coef[(*(pattern++))+length]);
        }
      }

      im1= &(p_imgYp_pad[j-fl_temp]);
      im2= &(p_imgYp_pad[j+fl_temp]);
      for (jj=0; jj<fl_temp; jj++)
      {
        pixelInt+=((*(im1++)+ *(im2--))*coef[(*(pattern++))+length]);
      }
      
      pixelInt+=(p_imgYp_pad[j]*coef[(*(pattern++))+length]);
#endif
      pixelInt=(int)((pixelInt+offset) >> (num_bits_minus_1));
      imgY_rec_post[i-fl][j-fl]=max(0, min(pixelInt,max_val));
    }
  }
}

#if WIENER_3_INPUT  
Void TComAdaptiveLoopFilter::subfilterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int filtNo_resi, int filtNo_pred, int start_height, int end_height, int start_width, int end_width, int bits_rec, int bits_pred, int bits_resi)
#else    
Void TComAdaptiveLoopFilter::subfilterFrame(imgpel **imgY_rec_post, imgpel **imgY_rec, int filtNo, int start_height, int end_height, int start_width, int end_width)
#endif
{
  int i, j, ii, jj, pixelInt,m=0;
  imgpel *p_imgY_var,*p_imgY_pad;
  int max_val=g_uiIBDI_MAX;
  int fl=FILTER_LENGTH/2;
  int *pattern=pattern9x9Sym;
  int fl_temp;
  int last_coef= MAX_SQR_FILT_LENGTH-1;
  imgpel *im1,*im2;
  int *coef;
#if WIENER_3_INPUT
  imgpel *p_imgYr_pad, *p_imgYp_pad;
  int offset     = (1<<(bits_rec - 1));
  int shift_pred = (1<<(bits_rec - bits_pred));
  int shift_resi = (1<<(bits_rec - bits_resi));
  int num_bits_minus_1= bits_rec;
  int length = 0;
  int flTab_pr[5] = {0,1,2,3,4};
  int coef_new[NO_VAR_BINS][MAX_SQR_FILT_LENGTH];
  int *cp, *cp1;
#else  
  int num_bits_minus_1= NUM_BITS-1;
  int offset = (1<<(NUM_BITS-2));
  int *pattern_fix=patternTab_filt[filtNo];
  fl_temp=flTab[filtNo];
#endif
  
#if WIENER_3_INPUT
  for (j=0;j<NO_VAR_BINS;j++)
  {
    cp  = coef_new[j];
    cp1 = filterCoeffPrevSelected[j];
    
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++;
    }
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++ * shift_resi;
    }
    for (i=0;i<(FILTER_LENGTH*FILTER_LENGTH)/2 + 1;i++)
    {
      *cp++=*cp1++ * shift_pred;
    }
    *cp++=*cp1++;
  }
#endif  
  // Filter
  for (i = fl + start_height; i < end_height+fl; i++)
  {
    p_imgY_var = imgY_var[i-fl] + start_width;
    p_imgY_pad = imgY_pad[i];
#if WIENER_3_INPUT  
    p_imgYr_pad = imgYr_pad[i];
    p_imgYp_pad = imgYp_pad[i];
#endif    
    for (j = fl + start_width; j < end_width+fl; j++)
    {
#if WIENER_3_INPUT  
      coef = coef_new[*(p_imgY_var++)];
#else
      coef = filterCoeffPrevSelected[*(p_imgY_var++)];
#endif
      pixelInt=coef[last_coef];
#if WIENER_3_INPUT  
      // reconstruction
      fl_temp=flTab[filtNo];
      pattern=patternTab_filt[filtNo];
#else      
      pattern=pattern_fix;
#endif    	  
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgY_pad[i+ii][j-m]);
        im2= &(imgY_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
          pixelInt+=((*(im1++)+ *(im2--))*coef[*(pattern++)]);
      }

      im1= &(p_imgY_pad[j-fl_temp]);
      im2= &(p_imgY_pad[j+fl_temp]);	
      for (jj=0; jj<fl_temp; jj++)
        pixelInt+=((*(im1++)+ *(im2--))*coef[*(pattern++)]);

      pixelInt+=(p_imgY_pad[j]*coef[*(pattern++)]);
#if WIENER_3_INPUT  
      // residual
      length = (FILTER_LENGTH*FILTER_LENGTH)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_resi];
      fl_temp=flTab_pr[filtNo_resi];
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgYr_pad[i+ii][j-m]);
        im2= &(imgYr_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
          pixelInt+=(((Int)(*(im1++)+ *(im2--))-(Int)(2*g_uiIBDI_MAX_Q))*coef[(*(pattern++))+length]);
      }

      im1= &(p_imgYr_pad[j-fl_temp]);
      im2= &(p_imgYr_pad[j+fl_temp]);	
      for (jj=0; jj<fl_temp; jj++)
        pixelInt+=(((Int)(*(im1++)+ *(im2--))-(Int)(2*g_uiIBDI_MAX_Q))*coef[(*(pattern++))+length]);
      pixelInt+=(((Int)p_imgYr_pad[j]-(Int)(g_uiIBDI_MAX_Q))*coef[(*(pattern++))+length]);
      // pred
      length += (FILTER_LENGTH_RESI*FILTER_LENGTH_RESI)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_pred];
      fl_temp=flTab_pr[filtNo_pred];
      for (ii=-fl_temp, m = 0; ii<0; ii++,m++)
      {
        im1= &(imgYp_pad[i+ii][j-m]);
        im2= &(imgYp_pad[i-ii][j+m]);
        for (jj=-m; jj<=m; jj++)
          pixelInt+=((*(im1++)+ *(im2--))*coef[(*(pattern++))+length]);
      }
  
      im1= &(p_imgYp_pad[j-fl_temp]);
      im2= &(p_imgYp_pad[j+fl_temp]);	
      for (jj=0; jj<fl_temp; jj++)
        pixelInt+=((*(im1++)+ *(im2--))*coef[(*(pattern++))+length]);
      pixelInt+=(p_imgYp_pad[j]*coef[(*(pattern++))+length]);
#endif
      pixelInt=(int)((pixelInt+offset) >> (num_bits_minus_1));
      imgY_rec_post[i-fl][j-fl]=max(0, min(pixelInt,max_val));
    }
  }
}



Void TComAdaptiveLoopFilter::xCUAdaptive_qc(TComPic* pcPic, ALFParam* pcAlfParam, imgpel **imgY_rec_post, imgpel **imgY_rec)
{
	// for every CU, call CU-adaptive ALF process
  for( UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcPic->getCU( uiCUAddr );
    xSubCUAdaptive_qc(pcCU, pcAlfParam, imgY_rec_post, imgY_rec, 0, 0);
  }
}

Void TComAdaptiveLoopFilter::xSubCUAdaptive_qc(TComDataCU* pcCU, ALFParam* pcAlfParam, imgpel **imgY_rec_post, imgpel **imgY_rec, UInt uiAbsPartIdx, UInt uiDepth)
{
  TComPic* pcPic = pcCU->getPic();

  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

	// check picture boundary
  if ( ( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() ) )
  {
    bBoundary = true;
  }

	// go to sub-CU?
  if ( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xSubCUAdaptive_qc(pcCU, pcAlfParam, imgY_rec_post, imgY_rec, uiAbsPartIdx, uiDepth+1);
    }
    return;
  }

	// check maskImagedec
  if ( pcCU->getAlfCtrlFlag(uiAbsPartIdx) )
  {
#if WIENER_3_INPUT  
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam->realfiltNo, (pcAlfParam->tap_resi-1)/2, (pcAlfParam->tap_pred-1)/2, uiTPelY, min(uiBPelY+1,(unsigned int)(img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(img_width)), pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[0]], pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[1]], pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[2]]);
#else
    subfilterFrame(imgY_rec_post, imgY_rec, pcAlfParam->realfiltNo, uiTPelY, min(uiBPelY+1,(unsigned int)(img_height)), uiLPelX, min(uiRPelX+1,(unsigned int)(img_width)));
#endif    
  }
}

#endif

Void TComAdaptiveLoopFilter::xALFLuma(TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  if(pcAlfParam->cu_control_flag)
  {
    // block-adaptive ALF process
    xCUAdaptive(pcPic, pcAlfParam, pcPicDec, pcPicRest);
  }
  else
  {
    // non-adaptive ALF process
    xFrame(pcPicDec, pcPicRest, pcAlfParam->coeff, pcAlfParam->tap);
  }
}

Void TComAdaptiveLoopFilter::xCUAdaptive(TComPic* pcPic, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
	// for every CU, call CU-adaptive ALF process
  for( UInt uiCUAddr = 0; uiCUAddr < pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcPic->getCU( uiCUAddr );
    xSubCUAdaptive(pcCU, pcAlfParam, pcPicDec, pcPicRest, 0, 0);
  }
}

/**
		- For every sub-CU's, it is called recursively
		.
		\param	pcCU					CU data structure
		\param	pcAlfParam		ALF parameter
		\param  pcPicDec			picture before ALF
		\retval	pcPicRest			picture after  ALF
		\param  uiAbsPartIdx	current sub-CU position
		\param	uiDepth				current sub-CU depth
 */
Void TComAdaptiveLoopFilter::xSubCUAdaptive(TComDataCU* pcCU, ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt uiAbsPartIdx, UInt uiDepth)
{
  TComPic* pcPic = pcCU->getPic();

  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

	// check picture boundary
  if ( ( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() ) )
  {
    bBoundary = true;
  }

	// go to sub-CU?
  if ( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xSubCUAdaptive(pcCU, pcAlfParam, pcPicDec, pcPicRest, uiAbsPartIdx, uiDepth+1);
    }
    return;
  }

	// check CU-based flag
  if ( pcCU->getAlfCtrlFlag(uiAbsPartIdx) )
  {
    xSubCUFilter( pcPicDec->getLumaAddr(pcCU->getAddr(), uiAbsPartIdx), pcPicRest->getLumaAddr(pcCU->getAddr(), uiAbsPartIdx),
                       pcAlfParam->coeff, pcAlfParam->tap,
                       pcCU->getWidth(uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx),
                       pcPicDec->getStride(), pcPicRest->getStride() );
  }
}

/** \todo		filtering operation is not optimized as frame-based functions
 */
Void TComAdaptiveLoopFilter::xSubCUFilter( Pel* pDec, Pel* pRest, Int *qh, Int iTap, Int iWidth, Int iHeight, Int iDecStride, Int iRestStride )
{
  Int i, x, y, value, N, offset;
  Pel PixSum[ALF_MAX_NUM_COEF];

  N      = (iTap*iTap+1)>>1;
  offset = iTap>>1;

  Pel* pTmpDec;

  Int iShift = g_uiBitDepth + g_uiBitIncrement - 8;

  switch(iTap)
  {
  case 5:
    {
    pDec -= iDecStride*2;
    for (y = 0; y < iHeight; y++)
    {
      for (x = 0; x < iWidth; x++)
      {
        pTmpDec = pDec + x - 2;
        PixSum[0] = pTmpDec[0];
        PixSum[1] = pTmpDec[1];
        PixSum[2] = pTmpDec[2];
        PixSum[3] = pTmpDec[3];
        PixSum[4] = pTmpDec[4];

        pTmpDec += iDecStride;
        PixSum[5] = pTmpDec[0];
        PixSum[6] = pTmpDec[1];
        PixSum[7] = pTmpDec[2];
        PixSum[8] = pTmpDec[3];
        PixSum[9] = pTmpDec[4];

        pTmpDec += iDecStride;
        PixSum[10] = pTmpDec[0];
        PixSum[11] = pTmpDec[1];
        PixSum[12] = pTmpDec[2];
        PixSum[11]+= pTmpDec[3];
        PixSum[10]+= pTmpDec[4];

        pTmpDec += iDecStride;
        PixSum[9]+= pTmpDec[0];
        PixSum[8]+= pTmpDec[1];
        PixSum[7]+= pTmpDec[2];
        PixSum[6]+= pTmpDec[3];
        PixSum[5]+= pTmpDec[4];

        pTmpDec += iDecStride;
        PixSum[4]+= pTmpDec[0];
        PixSum[3]+= pTmpDec[1];
        PixSum[2]+= pTmpDec[2];
        PixSum[1]+= pTmpDec[3];
        PixSum[0]+= pTmpDec[4];

        value = 0;
        for(i=0; i<N; i++)
        {
          value += qh[i]*PixSum[i];
        }
        // DC offset
        value += qh[N] << iShift;
        value = ( value + ALF_ROUND_OFFSET ) >> ALF_NUM_BIT_SHIFT;

        pRest[x] = (Pel) Clip(value);
      }
      pRest += iRestStride;
      pDec += iDecStride;
    }
    }
    break;
  case 7:
    {
    pDec -= iDecStride*3;
    for (y = 0; y < iHeight; y++)
    {
      for (x = 0; x < iWidth; x++)
      {
        pTmpDec = pDec + x - 3;
        PixSum[0] = pTmpDec[0];
        PixSum[1] = pTmpDec[1];
        PixSum[2] = pTmpDec[2];
        PixSum[3] = pTmpDec[3];
        PixSum[4] = pTmpDec[4];
        PixSum[5] = pTmpDec[5];
        PixSum[6] = pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[7] = pTmpDec[0];
        PixSum[8] = pTmpDec[1];
        PixSum[9] = pTmpDec[2];
        PixSum[10] = pTmpDec[3];
        PixSum[11] = pTmpDec[4];
        PixSum[12] = pTmpDec[5];
        PixSum[13] = pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[14] = pTmpDec[0];
        PixSum[15] = pTmpDec[1];
        PixSum[16] = pTmpDec[2];
        PixSum[17] = pTmpDec[3];
        PixSum[18] = pTmpDec[4];
        PixSum[19] = pTmpDec[5];
        PixSum[20] = pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[21] = pTmpDec[0];
        PixSum[22] = pTmpDec[1];
        PixSum[23] = pTmpDec[2];
        PixSum[24] = pTmpDec[3];
        PixSum[23]+= pTmpDec[4];
        PixSum[22]+= pTmpDec[5];
        PixSum[21]+= pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[20]+= pTmpDec[0];
        PixSum[19]+= pTmpDec[1];
        PixSum[18]+= pTmpDec[2];
        PixSum[17]+= pTmpDec[3];
        PixSum[16]+= pTmpDec[4];
        PixSum[15]+= pTmpDec[5];
        PixSum[14]+= pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[13]+= pTmpDec[0];
        PixSum[12]+= pTmpDec[1];
        PixSum[11]+= pTmpDec[2];
        PixSum[10]+= pTmpDec[3];
        PixSum[9]+= pTmpDec[4];
        PixSum[8]+= pTmpDec[5];
        PixSum[7]+= pTmpDec[6];

        pTmpDec += iDecStride;
        PixSum[6]+= pTmpDec[0];
        PixSum[5]+= pTmpDec[1];
        PixSum[4]+= pTmpDec[2];
        PixSum[3]+= pTmpDec[3];
        PixSum[2]+= pTmpDec[4];
        PixSum[1]+= pTmpDec[5];
        PixSum[0]+= pTmpDec[6];

        value = 0;
        for(i=0; i<N; i++)
        {
          value += qh[i]*PixSum[i];
        }
        // DC offset
        value += qh[N] << iShift;
        value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

        pRest[x] = (Pel) Clip(value);
      }
      pRest += iRestStride;
      pDec += iDecStride;
    }
    }
    break;
  case 9:
    {
    pDec -= iDecStride*4;
    for (y = 0; y < iHeight; y++)
    {
      for (x = 0; x < iWidth; x++)
      {
        pTmpDec = pDec + x - 4;
        PixSum[0] = pTmpDec[0];
        PixSum[1] = pTmpDec[1];
        PixSum[2] = pTmpDec[2];
        PixSum[3] = pTmpDec[3];
        PixSum[4] = pTmpDec[4];
        PixSum[5] = pTmpDec[5];
        PixSum[6] = pTmpDec[6];
        PixSum[7] = pTmpDec[7];
        PixSum[8] = pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[9] = pTmpDec[0];
        PixSum[10] = pTmpDec[1];
        PixSum[11] = pTmpDec[2];
        PixSum[12] = pTmpDec[3];
        PixSum[13] = pTmpDec[4];
        PixSum[14] = pTmpDec[5];
        PixSum[15] = pTmpDec[6];
        PixSum[16] = pTmpDec[7];
        PixSum[17] = pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[18] = pTmpDec[0];
        PixSum[19] = pTmpDec[1];
        PixSum[20] = pTmpDec[2];
        PixSum[21] = pTmpDec[3];
        PixSum[22] = pTmpDec[4];
        PixSum[23] = pTmpDec[5];
        PixSum[24] = pTmpDec[6];
        PixSum[25] = pTmpDec[7];
        PixSum[26] = pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[27] = pTmpDec[0];
        PixSum[28] = pTmpDec[1];
        PixSum[29] = pTmpDec[2];
        PixSum[30] = pTmpDec[3];
        PixSum[31] = pTmpDec[4];
        PixSum[32] = pTmpDec[5];
        PixSum[33] = pTmpDec[6];
        PixSum[34] = pTmpDec[7];
        PixSum[35] = pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[36] = pTmpDec[0];
        PixSum[37] = pTmpDec[1];
        PixSum[38] = pTmpDec[2];
        PixSum[39] = pTmpDec[3];
        PixSum[40] = pTmpDec[4];
        PixSum[39]+= pTmpDec[5];
        PixSum[38]+= pTmpDec[6];
        PixSum[37]+= pTmpDec[7];
        PixSum[36]+= pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[35]+= pTmpDec[0];
        PixSum[34]+= pTmpDec[1];
        PixSum[33]+= pTmpDec[2];
        PixSum[32]+= pTmpDec[3];
        PixSum[31]+= pTmpDec[4];
        PixSum[30]+= pTmpDec[5];
        PixSum[29]+= pTmpDec[6];
        PixSum[28]+= pTmpDec[7];
        PixSum[27]+= pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[26]+= pTmpDec[0];
        PixSum[25]+= pTmpDec[1];
        PixSum[24]+= pTmpDec[2];
        PixSum[23]+= pTmpDec[3];
        PixSum[22]+= pTmpDec[4];
        PixSum[21]+= pTmpDec[5];
        PixSum[20]+= pTmpDec[6];
        PixSum[19]+= pTmpDec[7];
        PixSum[18]+= pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[17]+= pTmpDec[0];
        PixSum[16]+= pTmpDec[1];
        PixSum[15]+= pTmpDec[2];
        PixSum[14]+= pTmpDec[3];
        PixSum[13]+= pTmpDec[4];
        PixSum[12]+= pTmpDec[5];
        PixSum[11]+= pTmpDec[6];
        PixSum[10]+= pTmpDec[7];
        PixSum[9]+=  pTmpDec[8];

        pTmpDec += iDecStride;
        PixSum[8]+= pTmpDec[0];
        PixSum[7]+= pTmpDec[1];
        PixSum[6]+= pTmpDec[2];
        PixSum[5]+= pTmpDec[3];
        PixSum[4]+= pTmpDec[4];
        PixSum[3]+= pTmpDec[5];
        PixSum[2]+= pTmpDec[6];
        PixSum[1]+= pTmpDec[7];
        PixSum[0]+= pTmpDec[8];

        value = 0;
        for(i=0; i<N; i++)
        {
          value += qh[i]*PixSum[i];
        }
        // DC offset
        value += qh[N] << iShift;
        value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

        pRest[x] = (Pel) Clip(value);
      }
      pRest += iRestStride;
      pDec += iDecStride;
    }
    }
    break;
  default:
    assert(0);
    break;
  }
}

Void TComAdaptiveLoopFilter::xFrame(TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap)
{
  Int i, x, y, value, N, offset;
  Pel PixSum[ALF_MAX_NUM_COEF];

  N      = (iTap*iTap+1)>>1;
  offset = iTap>>1;

  Int iHeight = pcPicRest->getHeight();
  Int iWidth = pcPicRest->getWidth();

  Pel* pDec = pcPicDec->getLumaAddr();
  Int iDecStride = pcPicDec->getStride();

  Pel* pRest = pcPicRest->getLumaAddr();
  Int iRestStride = pcPicRest->getStride();

  Pel* pTmpDec1, *pTmpDec2;
  Pel* pTmpPixSum;

  Int iShift = g_uiBitDepth + g_uiBitIncrement - 8;

  switch(iTap)
  {
  case 5:
    {
      Int iJump = iDecStride - 4;
      pDec -= iDecStride*2;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-2;
          pTmpDec2 = pTmpDec1+4+(4*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;


          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          for(i=0; i<13; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[13] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  case 7:
    {
      Int iJump = iDecStride - 6;
      pDec -= iDecStride*3;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-3;
          pTmpDec2 = pTmpDec1+6+(6*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          for(i=0; i<25; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[25] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  case 9:
    {
      Int iJump = iDecStride - 8;
      pDec -= iDecStride*4;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {

          pTmpDec1 = pDec+x-4;
          pTmpDec2 = pTmpDec1+8+(8*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          for(i=0; i<41; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[41] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  default:
    assert(0);
    break;
  }
}

// --------------------------------------------------------------------------------------------------------------------
// ALF for chroma
// --------------------------------------------------------------------------------------------------------------------

Void TComAdaptiveLoopFilter::xALFChroma(ALFParam* pcAlfParam, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  if((pcAlfParam->chroma_idc>>1)&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 0);
  }

  if(pcAlfParam->chroma_idc&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, pcAlfParam->coeff_chroma, pcAlfParam->tap_chroma, 1);
  }
}

/** \param	pcPicDec			picture before ALF
		\param	pcPicRest			picture after  ALF
		\param	qh						filter coefficient
		\param	iTap					filter tap
		\param	iColor				0 for Cb and 1 for Cr
 */
Void TComAdaptiveLoopFilter::xFrameChroma( TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Int *qh, Int iTap, Int iColor )
{
  Int i, x, y, value, N, offset;
//  Pel PixSum[ALF_MAX_NUM_COEF_C];// th
  Pel PixSum[ALF_MAX_NUM_COEF]; 

  N      = (iTap*iTap+1)>>1;
  offset = iTap>>1;

  Int iHeight = pcPicRest->getHeight() >> 1;
  Int iWidth = pcPicRest->getWidth() >> 1;

  Pel* pDec;
  Int iDecStride = pcPicDec->getCStride();

  Pel* pRest;
  Int iRestStride = pcPicRest->getCStride();

  Int iShift = g_uiBitDepth + g_uiBitIncrement - 8;

  if (iColor)
  {
    pDec = pcPicDec->getCrAddr();
    pRest = pcPicRest->getCrAddr();
  }
  else
  {
    pDec = pcPicDec->getCbAddr();
    pRest = pcPicRest->getCbAddr();
  }

  Pel* pTmpDec1, *pTmpDec2;
  Pel* pTmpPixSum;

  switch(iTap)
  {
  case 5:
    {
      Int iJump = iDecStride - 4;
      pDec -= iDecStride*2;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-2;
          pTmpDec2 = pTmpDec1+4+(4*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  case 7:
    {
      Int iJump = iDecStride - 6;
      pDec -= iDecStride*3;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-3;
          pTmpDec2 = pTmpDec1+6+(6*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum = (*pTmpDec1);

          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  case 9:
    {
      Int iJump = iDecStride - 8;
      pDec -= iDecStride*4;
      for (y = 0; y < iHeight; y++)
      {
        for (x = 0; x < iWidth; x++)
        {
          pTmpDec1 = pDec+x-4;
          pTmpDec2 = pTmpDec1+8+(8*iDecStride);
          pTmpPixSum = PixSum;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;

          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1 += iJump; pTmpDec2 -= iJump;


          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++; pTmpDec2--;
          *pTmpPixSum = ((*pTmpDec1) + (*pTmpDec2));
          pTmpPixSum++; pTmpDec1++;
          *pTmpPixSum =(*pTmpDec1);

          value = 0;
          for(i=0; i<N; i++)
          {
            value += qh[i]*PixSum[i];
          }
          // DC offset
          value += qh[N] << iShift;
          value = (value + ALF_ROUND_OFFSET)>>ALF_NUM_BIT_SHIFT;

          pRest[x] = (Pel) Clip(value);
        }
        pRest += iRestStride;
        pDec += iDecStride;
      }
    }
    break;
  default:
    assert(0);
    break;
  }
}
#endif


#endif //WIENER_3_INPUT
