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

/** \file     TEncAdaptiveLoopFilter.cpp
    \brief    estimation part of adaptive loop filter class
*/
#include "TEncAdaptiveLoopFilter.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#if WIENER_3_INPUT
#ifdef _WIN32
#include <float.h>
#define isnan(x) _isnan(x)
#endif
#endif

#if (WIENER_3_INPUT && !QC_ALF)


// ====================================================================================================================
// Constructor / destructor
// ====================================================================================================================

TEncAdaptiveLoopFilter::TEncAdaptiveLoopFilter()
{
  m_pcPic          = NULL;
  m_pcEntropyCoder = NULL;
}

Void TEncAdaptiveLoopFilter::startALFEnc( TComPic* pcPic, TEncEntropy* pcEntropyCoder )
{  
  m_pcPic = pcPic;
  m_pcEntropyCoder = pcEntropyCoder;
}

Void TEncAdaptiveLoopFilter::endALFEnc()
{
  m_pcPic = NULL;
  m_pcEntropyCoder = NULL;
}


Void TEncAdaptiveLoopFilter::set_golomb_parameter(ALFParam* pAlfParam, Int component, Int max_k)
{
  Int size_rec    = pAlfParam->filter_length_RD_rec [component];
  Int size_pred   = pAlfParam->filter_length_RD_pred[component];
  Int size_qpe    = pAlfParam->filter_length_RD_qpe [component];
  Int prec        = pAlfParam->filter_precision[component][0];
  Int prec2       = pAlfParam->filter_precision[component][1];
  Int length      = size_rec *size_rec + size_pred*size_pred+ size_qpe *size_qpe + 1;
  
  ALFParam* pcTempAlfParams = new ALFParam ;
  Int k_min=0;
  Int bits=0;
  Int bits_min;
  Int bits_min_tmp;
  Int enable_min=0;

  if (length>19 && prec>4)//only worth for a lot of coefficients of higher precision
  {
    allocALFParam( pcTempAlfParams );
    copyALFParam(pcTempAlfParams, pAlfParam);

    pcTempAlfParams->golomb_enable[component]=0;
    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam_control_data(pcTempAlfParams, component);
    m_pcEntropyCoder->encodeAlfParam_golomb      (pcTempAlfParams, component);
    m_pcEntropyCoder->encodeAlfParam_coeffs      (pcTempAlfParams, component);

    bits_min = m_pcEntropyCoder->getNumberOfWrittenBits();
    
    pcTempAlfParams->golomb_enable[component]=1;
        
    for (Int k=0;k<max_k;k++)
    {
      Int code2=k;
      if (length>39 && size_rec<5 && prec2>4)//only worth for a lot of coefficients of higher precision
      {
        code2=0;
      }
      pcTempAlfParams->golomb_code  [component][0]=k;
      pcTempAlfParams->golomb_code  [component][1]=code2;
      m_pcEntropyCoder->resetEntropy();
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeAlfParam_control_data(pcTempAlfParams, component);  
      m_pcEntropyCoder->encodeAlfParam_golomb      (pcTempAlfParams, component);
      m_pcEntropyCoder->encodeAlfParam_coeffs      (pcTempAlfParams, component);
    
      bits = m_pcEntropyCoder->getNumberOfWrittenBits();
      if (bits<bits_min_tmp || k==0)
      {
        bits_min_tmp=bits;
        k_min=k;
      }
    }
    pAlfParam->golomb_code  [component][0]=k_min;
    if (length>39 && size_rec<5 && prec2>4)//only worth for a lot of coefficients of higher precision
    {
      for (Int k=0;k<max_k;k++)
      {
        pcTempAlfParams->golomb_code  [component][1]=k;
        m_pcEntropyCoder->resetEntropy();
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeAlfParam_control_data(pcTempAlfParams, component);  
        m_pcEntropyCoder->encodeAlfParam_golomb      (pcTempAlfParams, component);
        m_pcEntropyCoder->encodeAlfParam_coeffs      (pcTempAlfParams, component);
    
        bits = m_pcEntropyCoder->getNumberOfWrittenBits();
        if (bits<bits_min || k==0)
        {
          bits_min=bits;
          k_min=k;
          enable_min=1;
        }
      }    
      pAlfParam->golomb_code  [component][1]=k_min;
    }
    else
    {
      if (bits_min_tmp<bits_min)
      {
        enable_min=1;
        pAlfParam->golomb_code  [component][1]=k_min;
      }
    }
    pAlfParam->golomb_enable[component]=enable_min;
    freeALFParam( pcTempAlfParams );
    delete pcTempAlfParams ;
  }
  else
  {
    pAlfParam->golomb_enable[component]=enable_min;
    pAlfParam->golomb_code  [component][0]=k_min;
    pAlfParam->golomb_code  [component][1]=k_min;
  }
}

UInt64 TEncAdaptiveLoopFilter::get_rate(ALFParam* pAlfParam, Int component)
{
  UInt64 ruiRate=0;
  
  if(pAlfParam != NULL)
  {
    ALFParam* pcTempAlfParams = new ALFParam ;
    allocALFParam( pcTempAlfParams );
    copyALFParam(pcTempAlfParams, pAlfParam);
    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam_control_data(pcTempAlfParams, component);
    m_pcEntropyCoder->encodeAlfParam_golomb      (pcTempAlfParams, component);
    m_pcEntropyCoder->encodeAlfParam_coeffs      (pcTempAlfParams, component);
    freeALFParam( pcTempAlfParams );
    delete pcTempAlfParams ;

    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    ruiRate = 1;
  }
  return ruiRate;
}


/** \param  pcAlfParam          ALF parameter
    \param  dLambda             lambda value for RD cost computation
    \retval ruiDist             distortion
    \retval ruiBits             required bits
    \retval ruiMaxAlfCtrlDepth  optimal partition depth
 */
Void TEncAdaptiveLoopFilter::ALFProcess( ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth )
{
  Int height;
  Int width;
  Int stride;
  Int stride_out;
  
  Pel* pOrg;
  Pel* pDec;
  Pel* pP;
  Pel* pQ;
  Pel* pout;
  
  TComPicYuv* pcPicOrg             = m_pcPic->getPicYuvOrg();
  TComPicYuv* pcPicYuvRec          = m_pcPic->getPicYuvRec();
  TComPicYuv* pcPicYuvP            = m_pcPic->getPicYuvP();
  TComPicYuv* pcPicYuvQ            = m_pcPic->getPicYuvQ();
  
  TComPicYuv* pcPicYuvFiltered = new TComPicYuv ;
  pcPicYuvFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth ) ;
  
  m_dLambdaLuma   = dLambda;
  m_dLambdaChroma = dLambda;
  
  m_iALF_fs_max_rec       = m_pcPic->getSlice()->getSPS()->getALFMaxFilterSize_rec ();
  m_iALF_fs_max_pred      = m_pcPic->getSlice()->getSPS()->getALFMaxFilterSize_pred();
  m_iALF_fs_max_qpe       = m_pcPic->getSlice()->getSPS()->getALFMaxFilterSize_qpe ();
  
  pcAlfParam->enable_flag[0] = m_pcPic->getSlice()->getSPS()->getALFEnableY();
  pcAlfParam->enable_flag[1] = m_pcPic->getSlice()->getSPS()->getALFEnableU();
  pcAlfParam->enable_flag[2] = m_pcPic->getSlice()->getSPS()->getALFEnableV();
  
  
  pcAlfParam->cu_control_flag = 0;
  pcAlfParam->alf_flag = 1 ;
  
  for (Int c = 0; c < 3; c++)
  {
    if (1 == pcAlfParam->enable_flag[c])
    {    
      if (c==0)
      {
        pOrg = pcPicOrg->getLumaAddr();
        pDec = pcPicYuvRec->getLumaAddr();
        pP   = pcPicYuvP->getLumaAddr();
        pQ   = pcPicYuvQ->getLumaAddr();
        pout = pcPicYuvFiltered->getLumaAddr();
        
        height      = m_pcPic->getSlice()->getSPS()->getHeight();
        width       = m_pcPic->getSlice()->getSPS()->getWidth ();
        stride      = pcPicOrg->getStride();
        stride_out  = pcPicYuvFiltered->getStride();
        m_dLambda = m_dLambdaLuma;
      }
      else if (c==1)
      {
        pOrg = pcPicOrg->getCbAddr();
        pDec = pcPicYuvRec->getCbAddr();
        pP   = pcPicYuvP->getCbAddr();
        pQ   = pcPicYuvQ->getCbAddr();
        pout = pcPicYuvFiltered->getCbAddr();
        
        height      = (m_pcPic->getSlice()->getSPS()->getHeight())>>1;
        width       = (m_pcPic->getSlice()->getSPS()->getWidth ())>>1;            
        stride      = pcPicOrg->getCStride();
        stride_out  = pcPicYuvFiltered->getCStride();
        m_dLambda = m_dLambdaChroma;
      }
      else
      {
        pOrg = pcPicOrg->getCrAddr();
        pDec = pcPicYuvRec->getCrAddr();
        pP   = pcPicYuvP->getCrAddr();
        pQ   = pcPicYuvQ->getCrAddr();      
        pout = pcPicYuvFiltered->getCrAddr();
        
        height      = (m_pcPic->getSlice()->getSPS()->getHeight())>>1;
        width       = (m_pcPic->getSlice()->getSPS()->getWidth ())>>1;            
        stride      = pcPicOrg->getCStride();
        stride_out  = pcPicYuvFiltered->getCStride();
        m_dLambda = m_dLambdaChroma;
      }
      
      estimate_filter(pcAlfParam, pOrg, pDec, pP, pQ, c, height, width, stride);
    }    
    if (1 == pcAlfParam->enable_flag[c])
    {  
      filter         (pcAlfParam, pDec, pout, pP, pQ, c, height, width, stride, stride_out);
      
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


//-----------------------------------------------------------------------------
/*!
* \brief Estimation of a non separable 2D Wiener Filter of equal filter size in horizontal and vertical direction
*
* \param[in] *dY                  % pointer to desired image signal
* \param[in] *xY                  % pointer to reconstructed image signal
* \param[in] *pY                  % pointer to prediction image signal
* \param[in] *qpe                 % pointer to quantized prediction error signal
* \param[in] component            % color component
*
* \return RD-Costs
*/
//-----------------------------------------------------------------------------
Double TEncAdaptiveLoopFilter::estimate_filter(ALFParam* pcAlfParam, 
                                               Pel* dY, 
                                               Pel* xY, 
                                               Pel* pY, 
                                               Pel* qpe, 
                                               Int component, 
                                               Int height_f, 
                                               Int width_f,
                                               Int Stride)
{  
#if WIENER_3_INPUT_FAST
  Int    k_max_golomb=5;
#else  
  Int    k_max_golomb=10;
#endif
  Double rd_costs=999999999.0;    
  Double rd_cost_precision=0.0;
  Double rd_cost_min_precision=0.0;
  Int    prec_best[3];
  Int    precision_best[3];
  Int    i, j, ii, m, n, l, k, k1, o, s, count, new_x, new_y;
  Int    tap_rec, tap_pred, tap_qpe;
  UInt64 rate=0;
  Double rounding=0.0;
  Double mse=0.0;
  Double mse_wlf_off=0.0;
  Double rd_cost=0.0;
  Double rd_cost_min=0.0;
  Double rd_cost_wlf_off=0.0;
  
  Double *p_d=NULL;
  Double *p_d1=NULL;
  Double *p_d2=NULL;
  
  Int *p_i=NULL;  
  Int *p_int2=NULL;  
  Int *p_int3=NULL;  

  Pel* pPel_1=NULL;
  Pel* pPel_2=NULL;
  Pel* pPel_3=NULL;
  
  Int filtersize_rec_best =1;
  Int filtersize_pred_best=1;
  Int filtersize_qpe_best =1;
  Int TAP_rec_1_2 =(m_iALF_fs_max_rec -1)>>1;
  Int TAP_pred_1_2=(m_iALF_fs_max_pred-1)>>1;
  Int TAP_qpe_1_2 =(m_iALF_fs_max_qpe -1)>>1;
  Int o_rec =0;
  Int o_pred=m_iALF_fs_max_rec*m_iALF_fs_max_rec;
  Int o_qpe =o_pred+m_iALF_fs_max_pred*m_iALF_fs_max_pred;
  Int length=o_qpe+m_iALF_fs_max_qpe*m_iALF_fs_max_qpe+1;
  
  Double diff;
  Int identical;
  Int all_zero;
  Int tmp;
  
  pcAlfParam->filter_length_RD_rec  [component] = 1;
  pcAlfParam->filter_length_RD_pred [component] = 1;
  pcAlfParam->filter_length_RD_qpe  [component] = 1;
  
  pcAlfParam->golomb_enable         [component] = 1;
  
  image = new Pel[width_f*height_f];

  get_mem1Ddouble(&rho         , length);
  get_mem1Ddouble(&rho_n       , length);
  get_mem1Ddouble(&rho_RD      , length);
  get_mem2Ddouble(&a           , length,length);
  get_mem2Ddouble(&a_1         , length, length);
  get_mem2Ddouble(&an          , length,length);
  get_mem2Ddouble(&a_RD        , length,length);
  get_mem1Ddouble(&h           , length);
  get_mem1Ddouble(&h1          , length);
  get_mem1Ddouble(&h2          , length);
  get_mem1Ddouble(&r1          , length);
  get_mem1Dint   (&null        , length);
  get_mem1Dint   (&null_orig   , length);
  get_mem1Dint   (&coeffs_save           , 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  get_mem1Dint   (&coeffs_best           , 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  get_mem1Dint   (&coeffs_best_precision , 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  
  get_mem1Dint   (&dont_care_save        , 1+3*MAX_WIENER_FILTER_LENGTH*MAX_WIENER_FILTER_LENGTH);
  
  Plane<Pel> p_xY (width_f, height_f, (MAX_WIENER_FILTER_LENGTH - 1) / 2);
  Plane<Pel> p_pY (width_f, height_f, (MAX_WIENER_FILTER_LENGTH - 1) / 2);
  Plane<Int> p_qpe(width_f, height_f, (MAX_WIENER_FILTER_LENGTH - 1) / 2);

  prec_best[0]=prec_best[1]=prec_best[2]=0;
  precision_best[0]=precision_best[1]=precision_best[2]=0;
  
  
  pPel_1 = xY;
  pPel_2 = pY; 
  pPel_3 = qpe; 
  for (i = 0; i < height_f; i++)
  {
    for (j = 0; j < width_f; j++)
    {
      p_xY [i][j] = pPel_1 [j];
      p_pY [i][j] = pPel_2 [j];
      p_qpe[i][j] = pPel_3 [j] - g_uiIBDI_MAX_Q;
    }
    pPel_1 +=Stride;
    pPel_2 +=Stride;
    pPel_3 +=Stride;
  }

  p_xY.mirror();
  p_pY.mirror();
  p_qpe.mirror();

  for (i = 0; i < length; i++)
  {
    rho   [i] = rho_n [i] = 0.0;
    for (j = 0; j < length; j++)
    {
      a  [i][j] = an [i][j] = 0.0;
    }
  }
       
  calc_correlation(p_xY, p_pY, p_qpe, dY,  m_iALF_fs_max_rec , m_iALF_fs_max_pred , m_iALF_fs_max_qpe, height_f, width_f, Stride);

  for (n=0;n<length;n++) 
  {
    memcpy(a_RD[n],a[n],length*sizeof(double));
  }
  memcpy(rho_RD,rho,length*sizeof(double));
  
  mse_wlf_off = get_mse(dY,xY,width_f, height_f, Stride, Stride);  
  rd_cost_wlf_off= mse_wlf_off;
  rd_cost_min=rd_cost_wlf_off;
  filtersize_rec_best  = 0;
  filtersize_pred_best = 0;
  filtersize_qpe_best  = 0;

  pcAlfParam->enable_flag[component]=0; //turn off
  
#if WIENER_3_INPUT_FAST
  for (tap_rec=1 ; tap_rec<=(component==0?m_iALF_fs_max_rec:max(1,m_iALF_fs_max_rec-6)) ; tap_rec+=2)
#else    
  for (tap_rec=1 ; tap_rec<=m_iALF_fs_max_rec ; tap_rec+=2)
#endif
  {
#if WIENER_3_INPUT_FAST
    for (tap_pred=1 ; tap_pred<=(component==0?m_iALF_fs_max_pred:max(1,tap_rec-2)) ; tap_pred+=2)
#else    
    for (tap_pred=1 ; tap_pred<=m_iALF_fs_max_pred ; tap_pred+=2)
#endif      
    {
#if WIENER_3_INPUT_FAST
      for (tap_qpe=1 ; tap_qpe<=(component==0?m_iALF_fs_max_qpe:max(1,tap_pred-2)) ; tap_qpe+=2)
#else    
      for (tap_qpe=1 ; tap_qpe<=m_iALF_fs_max_qpe ; tap_qpe+=2)
#endif
      {
        for (n=0;n<length;n++) 
        {
          memcpy(a[n],a_RD[n],length*sizeof(double));
        }
        memcpy(rho,rho_RD,length*sizeof(double));
        
        set_lines_and_columns_of_auto_to_zero(m_iALF_fs_max_rec , TAP_rec_1_2 , tap_rec , o_rec );
        set_lines_and_columns_of_auto_to_zero(m_iALF_fs_max_pred, TAP_pred_1_2, tap_pred, o_pred);
        set_lines_and_columns_of_auto_to_zero(m_iALF_fs_max_qpe , TAP_qpe_1_2 , tap_qpe , o_qpe );
        
        for (k = 0; k < length; k++)
        {
          null[k]=length<<2;
        }
        memset(null_orig                       ,0,length*sizeof(int));
        memset(pcAlfParam->dont_care[component],0,length*sizeof(int));
    
        
        //Mark zero lines
        for (k = 0; k < length; k++)
        {
          all_zero =1;
          for (l = 0; l < length; l++)
          {
            if(a[k][l]>0.000001 || a[k][l]<-0.000001)
            {
              all_zero=0;
              break;
            }
          }
          if (all_zero!=0)
          {
            null_orig[k]=1;
          }
        }
          
        
        //Set equal lines to zero
        for (k = 0; k < length; k++)
        {
          for (l = k+1; l < length; l++)
          {
            identical=1;
            for (j = 0; j < length; j++)
            {
              diff=a[k][j]-a[l][j];
              diff*=diff;
              if (diff>0.0001)
              {
                identical=0;
                break;
              }
            }
            if (1 == identical)
            {
              for (j = 0; j < length; j++)
              {
                a[l][j] = 0.0;
                a[j][l] = 0.0;
                rho [l] = 0.0;
              }
            }
          }
        }
    
        //Take out ZERO-Lines
        count=0;
        for (k = 0; k < length; k++)
        {
          l=0;
          s=0;
          while(l< length)
          {
            if(a[k][l]>0.000001 || a[k][l]<-0.000001)
            {
              s=1;
              break;
              l= length;
            }
            else
            {
              l++;
            }
          }
    
          if (s==0)
          {
            null[count]=k;
            count++;
          }
          else 
          {
            memcpy(a_1[k-count],a[k],length*sizeof(double));
            r1[k-count]=rho[k];
          }
        }
    
        //Take out ZERO-Columns
        k1=0;
        for(l = 0; l < length; l++)
        {
          if(l==null[k1]) 
          {
            k1++;
          }
          else
          {
            for(k = 0; k < length; k++)
            {
              a_1[k][l-k1]=a_1[k][l];
            }
          }
        }
        
        
        if (length-count>0)
        {
          get_mem2Ddouble(&a_final  , length-count, length-count);
          get_mem1Ddouble(&r_final  , length-count);
          get_mem1Ddouble(&h_final  , length-count);
    
          for(k = 0; k < length-count; k++)
          {
            memcpy(a_final[k],a_1[k],(length-count)*sizeof(double));
          }
          memcpy(r_final,r1,(length-count)*sizeof(double));
          
          sgaus(a_final, h_final, r_final, length-count);
    
          free_mem1Ddouble(r_final);
          free_mem2Ddouble(a_final);
        }
    
        k1=0;
        p_d=&(h1[0]);
        for(k = 0; k < length; k++)
        {
          if(k==null[k1]) //This needs to be a ZERO-Column
          {
            *(p_d++)=0.0;
            if (null_orig[k]!=0)
            {
              pcAlfParam->dont_care[component][k]=1;
            }
            k1++;
          }
          else 
          {
            *(p_d++)=h_final[k-k1];
          }
        }
    
        if (length-count>0)
        {
          free_mem1Ddouble(h_final);
        }
        
        memcpy(dont_care_save, (pcAlfParam->dont_care)[component],  length * sizeof(int));
    
        p_d1  =&(h2[0]);    
        p_int3=&((pcAlfParam->dont_care)[component][0]);
        tmp=(tap_rec-1)>>1;
        p_d2=&(h1[0]);
        p_int2=&(dont_care_save[0]);
        for (k = 0; k < m_iALF_fs_max_rec*m_iALF_fs_max_rec; k++)
        {
          new_x =k%m_iALF_fs_max_rec-TAP_rec_1_2;
          new_y =k/m_iALF_fs_max_rec-TAP_rec_1_2;
          if (!(new_x<-tmp || new_x>tmp || new_y<-tmp || new_y>tmp || tap_rec==0))
          {
            *(p_d1++  )=p_d2  [k];
            *(p_int3++)=p_int2[k];
          }
        }
        
        tmp=(tap_pred-1)>>1;
        p_d2=&(h1[m_iALF_fs_max_rec*m_iALF_fs_max_rec]);
        p_int2=&(dont_care_save[m_iALF_fs_max_rec*m_iALF_fs_max_rec]);
        for (k = 0; k < m_iALF_fs_max_pred*m_iALF_fs_max_pred; k++)
        {
          new_x =k%m_iALF_fs_max_pred-TAP_pred_1_2;
          new_y =k/m_iALF_fs_max_pred-TAP_pred_1_2;
          if (!(new_x<-tmp || new_x>tmp || new_y<-tmp || new_y>tmp || tap_pred==0))
          {
            *(p_d1++  )=p_d2  [k];
            *(p_int3++)=p_int2[k];
          }
        }
        
        tmp=(tap_qpe-1)>>1;
        p_d2=&(h1[m_iALF_fs_max_rec*m_iALF_fs_max_rec+m_iALF_fs_max_pred*m_iALF_fs_max_pred]);
        p_int2=&(dont_care_save[m_iALF_fs_max_rec*m_iALF_fs_max_rec+m_iALF_fs_max_pred*m_iALF_fs_max_pred]);
        for (k = 0; k < m_iALF_fs_max_qpe*m_iALF_fs_max_qpe; k++)
        {
          new_x =k%m_iALF_fs_max_qpe-TAP_qpe_1_2;
          new_y =k/m_iALF_fs_max_qpe-TAP_qpe_1_2;
          if (!(new_x<-tmp || new_x>tmp || new_y<-tmp || new_y>tmp || tap_qpe==0))
          {
            *(p_d1++  )=p_d2  [k];
            *(p_int3++)=p_int2[k];
          }
        }
        
        *(p_d1++)  =h1            [m_iALF_fs_max_rec*m_iALF_fs_max_rec+m_iALF_fs_max_pred*m_iALF_fs_max_pred+m_iALF_fs_max_qpe*m_iALF_fs_max_qpe];
        *(p_int3++)=dont_care_save[m_iALF_fs_max_rec*m_iALF_fs_max_rec+m_iALF_fs_max_pred*m_iALF_fs_max_pred+m_iALF_fs_max_qpe*m_iALF_fs_max_qpe];
        
        pcAlfParam->filter_length_RD_rec [component]  = tap_rec;
        pcAlfParam->filter_length_RD_pred[component]  = tap_pred;
        pcAlfParam->filter_length_RD_qpe [component]  = tap_qpe;
        
        rd_cost_min_precision=999999999.0;
#if WIENER_3_INPUT_FAST
        for (ii=(component==0?0:2);ii<(component==0?(Int)FILTER_PRECISION_TABLE_NUMBER:(Int)(FILTER_PRECISION_TABLE_NUMBER)-2);ii++)
#else          
        for (ii=0;ii<FILTER_PRECISION_TABLE_NUMBER;ii++)
#endif
        {
          //Quantize filter coefficients
          p_d=&(h2[0]);
          p_i=&((pcAlfParam->coeffs)[component][0]);
          for (m = 0; m < tap_rec*tap_rec+tap_pred*tap_pred+tap_qpe*tap_qpe+1; m++)
          {
            if (*p_d>0.0)
            {
              *(p_i++) =  (int) ( *(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
            else
            {
              *(p_i++) = -(int) (-*(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
          }
                      
          pcAlfParam->filter_precision[component][0]=ii;
          pcAlfParam->filter_precision[component][1]=ii;
          pcAlfParam->filter_precision[component][2]=ii;
          
          filter(pcAlfParam, p_xY, image, p_pY, p_qpe, width_f, height_f, width_f, component);
    
          set_golomb_parameter(pcAlfParam, component, k_max_golomb);
          rate = get_rate(pcAlfParam, component);
          
          mse = get_mse(dY, image, width_f, height_f, Stride, width_f);
                
          rd_cost_precision= mse + m_dLambda * (double)(rate);
          
          if (rd_cost_precision < rd_cost_min_precision || ii==1)
          {
            precision_best[0]=ii;
            precision_best[1]=ii;
            precision_best[2]=ii;
            rd_cost_min_precision =rd_cost_precision;
            memcpy(coeffs_best_precision, (pcAlfParam->coeffs)[component], length * sizeof(int));
          }
        }
        
        memcpy(pcAlfParam->filter_precision[component], precision_best,  3 * sizeof(int));
          
        memcpy((pcAlfParam->coeffs)[component], coeffs_best_precision,  length * sizeof(int));
        
        if (precision_best[0]!=0)
        {
          ii=precision_best[0]-1;
          
          //Quantize filter coefficients
          p_d=&(h2[tap_rec*tap_rec]);
          p_i=&((pcAlfParam->coeffs)[component][tap_rec*tap_rec]);
          for (m = 0; m < tap_pred*tap_pred+tap_qpe*tap_qpe; m++)
          {
            if (*p_d>0.0)
            {
              *(p_i++) =  (int) ( *(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
            else
            {
              *(p_i++) = -(int) (-*(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
          }
    
          pcAlfParam->filter_precision[component][1]=ii;
          pcAlfParam->filter_precision[component][2]=ii;
          
          filter(pcAlfParam, p_xY, image, p_pY, p_qpe, width_f, height_f, width_f, component);
    
          set_golomb_parameter(pcAlfParam, component, k_max_golomb);
          rate = get_rate(pcAlfParam, component);
          
          mse = get_mse(dY, image, width_f, height_f, Stride, width_f);
                
          rd_cost_precision= mse + m_dLambda * (double)(rate);
            
          if (rd_cost_precision < rd_cost_min_precision)
          {
            precision_best[1]=ii;
            precision_best[2]=ii;
            rd_cost_min_precision =rd_cost_precision;
            memcpy(coeffs_best_precision, (pcAlfParam->coeffs)[component], length * sizeof(int));
          }
        }
        
        memcpy(pcAlfParam->filter_precision[component], precision_best,  3 * sizeof(int));
          
        memcpy((pcAlfParam->coeffs)[component], coeffs_best_precision,  length * sizeof(int));
        
        if (precision_best[0]>1 && precision_best[0]!=precision_best[1])
        {
          ii=precision_best[1]-1;
          
          //Quantize filter coefficients
          p_d=&(h2[tap_rec*tap_rec]);
          p_i=&((pcAlfParam->coeffs)[component][tap_rec*tap_rec+tap_pred*tap_pred]);
          for (m = 0; m < tap_qpe*tap_qpe; m++)
          {
            if (*p_d>0.0)
            {
              *(p_i++) =  (int) ( *(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
            else
            {
              *(p_i++) = -(int) (-*(p_d++) * (double)(pcAlfParam->filter_precision_table[ii]) + 0.5);
            }
          }
    
          pcAlfParam->filter_precision[component][2]=ii;
          
          filter(pcAlfParam, p_xY, image, p_pY, p_qpe, width_f, height_f, width_f, component);
    
          set_golomb_parameter(pcAlfParam, component, k_max_golomb);
          rate = get_rate(pcAlfParam, component);
          
          mse = get_mse(dY, image, width_f, height_f, Stride, width_f);
                
          rd_cost_precision= mse + m_dLambda * (double)(rate);
            
          if (rd_cost_precision < rd_cost_min_precision)
          {
            precision_best[2]=ii;
            rd_cost_min_precision =rd_cost_precision;
            memcpy(coeffs_best_precision, (pcAlfParam->coeffs)[component], length * sizeof(int));
          }
        }
        
        memcpy(pcAlfParam->filter_precision[component], precision_best,  3 * sizeof(int));
          
        memcpy((pcAlfParam->coeffs)[component], coeffs_best_precision,  length * sizeof(int));
        
        k1=tap_rec*tap_rec+tap_pred*tap_pred+tap_qpe*tap_qpe+1;
        
#if WIENER_3_INPUT_FAST
        for (o=0;o<2;o++)
#else
        for (o=0;o<10;o++)
#endif          
        {
          filter_d(pcAlfParam, &rounding, dY, p_xY, image, p_pY, p_qpe, width_f, height_f, Stride, width_f, component);
          int sign = (rounding > 0.0) ? 1 : -1;
          int delta = (int)(sign * rounding * 0.25 * pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][0]] + 0.5);
          if (delta == 0)
            break; // Not modifying coeff, so there is no point in re-testing the same coeffs over and over again
          (pcAlfParam->coeffs)[component][k1 - 1] -= sign * delta;
        }
        
        filter(pcAlfParam, p_xY, image, p_pY, p_qpe, width_f, height_f, width_f, component);
    
        //get_rate
        set_golomb_parameter(pcAlfParam, component, k_max_golomb);
        rate =get_rate(pcAlfParam, component);
        //get_mse
        mse = get_mse(dY, image, width_f, height_f, Stride, width_f);
        
        //calc RD costs
        rd_cost= mse + m_dLambda * (double)(rate);
        
        if (rd_cost < rd_cost_min)
        {
          filtersize_rec_best  = tap_rec;
          filtersize_pred_best = tap_pred;
          filtersize_qpe_best  = tap_qpe;
          rd_cost_min          = rd_cost;
          memcpy (prec_best, pcAlfParam->filter_precision[component], 3 * sizeof(int));          
          pcAlfParam->enable_flag[component]  = 1; //turn on
          memcpy(coeffs_best, (pcAlfParam->coeffs)[component], length * sizeof(int));
        }
        
        memcpy((pcAlfParam->coeffs)[component], coeffs_best,  length * sizeof(int));
      }
    }
  }
  memcpy((pcAlfParam->coeffs)[component], coeffs_best,  length * sizeof(int));

  pcAlfParam->filter_length_RD_rec [component] = filtersize_rec_best;
  pcAlfParam->filter_length_RD_pred[component] = filtersize_pred_best;
  pcAlfParam->filter_length_RD_qpe [component] = filtersize_qpe_best;
    
  memcpy (pcAlfParam->filter_precision[component], prec_best, 3 * sizeof(int));
  
  if (0 != pcAlfParam->enable_flag[component])
  {
    set_golomb_parameter(pcAlfParam, component, k_max_golomb);
  }
    
  delete[] image;
  free_mem1Ddouble(h);
  free_mem1Ddouble(rho);
  free_mem1Ddouble(rho_n);
  free_mem1Ddouble(rho_RD);
  free_mem2Ddouble(a);
  free_mem2Ddouble(an);
  free_mem2Ddouble(a_RD);
  
  free_mem1Ddouble(r1);
  free_mem1Ddouble(h1);
  free_mem1Ddouble(h2);
  free_mem1Dint   (null);
  free_mem1Dint   (null_orig);
  free_mem2Ddouble(a_1);
  free_mem1Dint   (coeffs_save);
  free_mem1Dint   (coeffs_best);
  free_mem1Dint   (coeffs_best_precision);
  free_mem1Dint   (dont_care_save);

  return rd_cost_min;
}



//-----------------------------------------------------------------------------
/*!
* \brief Non separable 2D Wiener Filter
*
* \param[in] *xY                  % pointer to reconstructed image signal
* \param[in] *oY                  % pointer to output image signal
* \param[in] *pY                  % pointer to prediction image signal
* \param[in] *qpe                 % pointer to quantized prediction error signal
* \param[in] component            % color component
* \param[in] height_f             % height of image data
* \param[in] width_f              % width of image data
* \param[in] Stride               % Stride
*
* \param[out] Rounding eror 
*
*/
//-----------------------------------------------------------------------------
Double TEncAdaptiveLoopFilter::filter_d(ALFParam* pcAlfParam,
                                        Double *rounding, 
                                        const Pel *org, 
                                        const Plane<Pel> &xY, 
                                        Pel *oY, 
                                        const Plane<Pel> &pY, 
                                        const Plane<Int> &qpe, 
                                        Int width, 
                                        Int height, 
                                        Int Stride_org, 
                                        Int Stride_out,
                                        Int component)
{
  int i, j, m, n, sum;
  int tmp_rec1  = pcAlfParam->filter_length_RD_rec [component];
  int tmp_pred1 = pcAlfParam->filter_length_RD_pred[component];
  int tmp_qpe1  = pcAlfParam->filter_length_RD_qpe [component];
  int tmp_rec  = (tmp_rec1  - 1) >> 1;
  int tmp_pred = (tmp_pred1 - 1) >> 1;
  int tmp_qpe  = (tmp_qpe1  - 1) >> 1;

  Pel *p_oY = oY;
  const Pel *p_org = org;
  const Pel *uc_ptr;
  const Int *s_ptr;
  int pitch;
  double mse = 0.0;
  double precision_rec  = (double)(pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][0]]);
  double precision_pred = (double)(pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][1]]);
  double precision_qpe  = (double)(pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][2]]);
  Int half = pcAlfParam->filter_precision_table_half [pcAlfParam->filter_precision[component][0]];
  Int shift= pcAlfParam->filter_precision_table_shift[pcAlfParam->filter_precision[component][0]];
  
  
  Int *p_coeffs = new Int[tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1+1];
  
  for (i = 0; i < tmp_rec1*tmp_rec1; i++)
  {
    p_coeffs[i]=pcAlfParam->coeffs[component][i];
  }
  for (i = tmp_rec1*tmp_rec1; i < tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1; i++)
  {
    p_coeffs[i] =pcAlfParam->coeffs[component][i]*pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][0]];
    p_coeffs[i]/=pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][1]];
  }
  for (i = tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1; i < tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1; i++)
  {
    p_coeffs[i] =pcAlfParam->coeffs[component][i]*pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][0]];
    p_coeffs[i]/=pcAlfParam->filter_precision_table[pcAlfParam->filter_precision[component][2]];
  }
  p_coeffs[tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1]=pcAlfParam->coeffs[component][tmp_rec1*tmp_rec1+tmp_pred1*tmp_pred1+tmp_qpe1*tmp_qpe1];
  
  *rounding = 0.0;

  for (j = 0; j < height; j++) 
  {
    for (i = 0; i < width; i++) 
    {
      Int *co = (pcAlfParam->coeffs)[component];
      
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
          sum += ((*s_ptr++)) * (*co++);
          sum += ((*s_ptr++)) * (*co++);
        }
        sum += (*s_ptr++) * (*co++);

        s_ptr += pitch;
      }
     
      sum +=(*(co++));
      
      double org_val = (double)(p_org[i]);
      double avg = (double)sum / precision_rec;
      
      mse += (avg - org_val) * (avg - org_val);

      *rounding += avg;

      sum  +=  half;
      sum >>=  shift;

      *rounding -= (double)sum;

      p_oY[i] = (Pel)Clip(sum);
    }
    p_org+=Stride_org;
    p_oY +=Stride_out;
  }

  delete [] p_coeffs;
  
  *rounding /= height * width;
  return mse / (height * width);
}




//-----------------------------------------------------------------------------
/*!
* \brief Setting certain lines and columns of the autocorrelation matrix to zero
*
* \param[in] tap                  % filter size max
* \param[in] tap_minus_one_half   % one half of filter size max minus one
* \param[in] tap_check            % filter size to be realized
* \param[in] on                   % horizontal and vertical offset in the autocorrelation matrix
*
*/
//-----------------------------------------------------------------------------
void TEncAdaptiveLoopFilter::set_lines_and_columns_of_auto_to_zero(int tap, int tap_minus_one_half, int tap_check, int on)
{
  int k,l;
  int taptap=tap*tap;
  int new_x,new_y;  
  int length=m_iALF_fs_max_rec*m_iALF_fs_max_rec+m_iALF_fs_max_pred*m_iALF_fs_max_pred+m_iALF_fs_max_qpe*m_iALF_fs_max_qpe+1;
  int tmp=(tap_check-1)>>1;
  int tmp1;
  
  for (k = 0; k < taptap; k++)
  {
    new_x =k%tap-tap_minus_one_half;
    new_y =k/tap-tap_minus_one_half;
    tmp1=k+on;
    if (new_x<-tmp || new_x>tmp || new_y<-tmp || new_y>tmp || tap_check==0)
    {
      memset (a  [tmp1],0,length*sizeof(Double));
      for (l = 0; l < length; l++)
      {
//         a  [tmp1][l]=0.0;
        a  [l][tmp1]=0.0;
      }
    }
  }
}


//-----------------------------------------------------------------------------
/*!
* \brief Calculation of mean squared error
*
* \param[in] *image1              % pointer to first image data
* \param[in] *image2              % pointer to second image data
* \param[in] size                 % Size of image data
*
* \return mse
*/
//-----------------------------------------------------------------------------
double TEncAdaptiveLoopFilter::get_mse(Pel *image1, Pel *image2, Int width, Int height, Int stride1, Int stride2)
{
  Int    i,j;
  Double mse=0.0;
  Int    tmp;
  Pel *i1=image1;
  Pel *i2=image2;
  UInt uiAdd   = (g_uiBitIncrement>0)?(1<<(g_uiBitIncrement-1)):0;
  
  for (j=0;j<height;j++)
  {
    for (i=0;i<width;i++)
    {
      tmp = (i1[i]+uiAdd)>>g_uiBitIncrement;
      tmp-= (i2[i]+uiAdd)>>g_uiBitIncrement;
      mse+=(double)(tmp*tmp);
    }
    i1+=stride1;
    i2+=stride2;
  }
  return mse;
}

template <typename T, typename U, typename V>
void TEncAdaptiveLoopFilter::calc_correlation(const Plane<T> &rec, const Plane<U> &pred, const Plane<V> &qpe, Pel *org, Int max_rec, Int max_pred, Int max_qpe, Int height, Int width, Int Stride)
{
  Int taptap1 = max_rec  *  max_rec;
  Int taptap2 = max_pred *  max_pred;
  Int taptap3 = max_qpe  *  max_qpe;
  Int length  = taptap1 + taptap2 + taptap3;
  Int m_rec  = (max_rec -1)>>1;
  Int m_pred = (max_pred-1)>>1;
  Int m_qpe  = (max_qpe -1)>>1;
  Double *pd;
  Double *pd1;
  Double *pd2;
  Double *accum = new Double[length*length];
  Double *accum1= new Double[length];
  Double *accum2= new Double[length];
  Double   *vec = new Double[length];
  Double *p_accum;
  Double *p_accum1;
  Double *p_accum2;
  Double *p_vec_y;
  Double *p_vec_x;
  Double mean_org=0.0;
  const T *rec_row;
  const U *pred_row;    
  const V *qpe_row;
  Pel *p_dY = org;
  
  memset (accum  ,0,length*length*sizeof(Double));
  memset (accum1 ,0,length*sizeof(Double));
  memset (accum2 ,0,length*sizeof(Double));

  for (Int j = 0; j < height; j++) 
  {    
    for (Int i = 0; i < width; i++)
    {
      Double val_org = (double)(p_dY[i]);
      mean_org+=val_org;
      
      Double *p_vec=vec;
      for (Int my = 0; my < max_rec; my++)
      {
        rec_row = rec[j - m_rec + my] + (i - m_rec);
        for (Int mx = 0; mx < max_rec; mx++)
        {
	  *p_vec++ = (double)(*rec_row++);
        }
      }
      for (Int my = 0; my < max_pred; my++)
      {
        pred_row = pred[j - m_pred + my] + (i - m_pred);
        for (Int mx = 0; mx < max_pred; mx++)
        {
	  *p_vec++ = (double)(*pred_row++);
        }
      }
      for (Int my = 0; my < max_qpe; my++)
      {
        qpe_row = qpe[j - m_qpe + my] + (i - m_qpe);
        for (Int mx = 0; mx < max_qpe; mx++)
        {
	  *p_vec++ = (double)(*qpe_row++);
        }
      }
      
      p_accum = accum;
      p_accum1= accum1;
      p_accum2= accum2;
      p_vec_y = vec;
      for (Int my = 0; my < length; my++)
      {
        Double value=*p_vec_y;
	p_vec_x = vec+my;
        
        *p_accum1++ +=val_org*value;
        *p_accum2++ +=value;
        for (Int mx = my; mx < length; mx++)
        {
	  *p_accum++ += (*p_vec_x++)*value;
        }
	p_vec_y++;
      }       
       
    }  
    p_dY+=Stride;
  }

  pd =accum;
  pd1=accum1;
  pd2=accum2;
  for (Int my = 0; my < length; my++)
  {
    rho  [my]=*pd1++;
    
    a [my][length]=*pd2;
    a [length][my]=*pd2++;
    
    rho  [length]=mean_org;
        
    for (Int mx = my; mx < length; mx++)
    {
      a[my][mx]=*pd;
      a[mx][my]=*pd++;
    }
  }

  a [length][length]=(Double)(height*width);
  
  delete[] accum;
  delete[] accum1;
  delete[] accum2;
  delete[] vec;
}

Int TEncAdaptiveLoopFilter::get_mem1Ddouble(Double **array1D, Int raws)
{
  Int j;

  if((*array1D      = (Double*)calloc(raws, sizeof(Double))) == NULL)
    no_mem_exit("get_mem1Ddouble: array3D");

  for(j = 0; j < raws; j++)
    (*array1D)[j] = 0.0;
  return sizeof(Double)*raws;
}

Int TEncAdaptiveLoopFilter::get_mem2Ddouble(Double ***array2D, Int rows, Int columns)
{
  Int i,j;

  if((*array2D      = (double**)calloc(rows, sizeof(Double*))) == NULL)
    no_mem_exit("get_mem2Ddouble: array2D");

  if(((*array2D)[0] = (double* )calloc(columns*rows,sizeof(Double ))) == NULL)
    no_mem_exit("get_mem3Ddouble: array2D");

  for(i=1;i<rows;i++)
    (*array2D)[i] = (*array2D)[i-1] + columns ;

  for(i = 0; i < rows; i++)
    for(j = 0; j < columns; j++)
      (*array2D)[i][j] = 0.0;
  return sizeof(double)*rows*columns;
}

void TEncAdaptiveLoopFilter::free_mem1Ddouble(Double *array1D)
{
  if (array1D)
    free (array1D);
  else
    no_mem_exit("free_mem1Ddouble: array1D");
}

void TEncAdaptiveLoopFilter::free_mem2Ddouble(Double **array2D)
{

  if (array2D)
  {
    if (array2D[0])
      free (array2D[0]);
    else
      no_mem_exit("free_mem2Ddouble: array2D");

    free (array2D);
  }
  else
  {
    printf("free_mem2DDouble: trying to free unused memory");
    exit(-1);
  }
}

#else //WIENER_3_INPUT


// ====================================================================================================================
// Constants
// ====================================================================================================================

#define ALF_NUM_OF_REDESIGN 3

#if HHI_ALF
// ====================================================================================================================
// Constructor / destructor
// ====================================================================================================================

TEncAdaptiveLoopFilter::TEncAdaptiveLoopFilter()
{
  m_ppdAlfCorr = NULL;
  m_pdDoubleAlfCoeff = NULL;
  m_puiCUHorizontalCorr = NULL;
  m_puiCUVerticalCorr = NULL;
  m_pcPic = NULL;
  m_pcEntropyCoder = NULL;
  m_pcBestAlfParam = NULL;
  m_pcTempAlfParam = NULL;
  m_pcPicYuvBest = NULL;
  m_pcPicYuvTmp = NULL;
  m_dMinCost = MAX_DOUBLE ;
  m_pcCoderFixedBits = NULL;
  m_bUseSBACRD = false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  pcPic           picture (TComPic) pointer
    \param  pcEntropyCoder  entropy coder class
 */
Void TEncAdaptiveLoopFilter::startALFEnc( TComPic* pcPic, TEncEntropy* pcEntropyCoder , TEncSbac*** pppcRDSbacCoder, TEncSbac* pcRDGoOnSbacCoder )
{
  m_pcPic = pcPic;
  m_pcEntropyCoder = pcEntropyCoder;
  m_pcCoderFixedBits = new TComBitCounter;
  m_pcEntropyCoder->setBitstream( m_pcCoderFixedBits );
  m_pppcRDSbacCoder = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder = pcRDGoOnSbacCoder;
  if( pcRDGoOnSbacCoder )
    m_bUseSBACRD = true;

  m_bALFSeparateQt = pcPic->getSlice()->getSPS()->getALFSeparateQt();
  m_bALFSymmetry   = pcPic->getSlice()->getSPS()->getALFSymmetry();
  m_iALFMinLength  = pcPic->getSlice()->getSPS()->getALFMinLength();
  m_iALFMaxLength  = pcPic->getSlice()->getSPS()->getALFMaxLength();

  assert( m_iALFMinLength >= ALF_MIN_LENGTH       );
  assert( m_iALFMaxLength <= ALF_MAX_VERT_LENGTH  );
  assert( m_iALFMaxLength <= ALF_MAX_HORIZ_LENGTH );


  m_eSliceType = pcPic->getSlice()->getSliceType();
  m_iPicNalReferenceIdc = (pcPic->getSlice()->isReferenced() ? 1 :0);

  m_uiNumSCUInCU = m_pcPic->getNumPartInCU();
  m_uiSCUWidth = (m_pcPic->getMinCUWidth()<<1);
  m_uiSCUHeight = (m_pcPic->getMinCUHeight()<<1);

  xInitParam();
  Int iWidth  = pcPic->getPicYuvOrg()->getWidth();
  Int iHeight = pcPic->getPicYuvOrg()->getHeight();

  m_pcPicYuvTmp = new TComPicYuv();
  m_pcPicYuvTmp->create(iWidth, iHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);

  m_pcPicYuvFiltered = new TComPicYuv();
  m_pcPicYuvFiltered->create(iWidth, iHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);

  m_pcPicYuvBest = pcPic->getPicYuvPred();

  m_pcBestAlfParam = new ALFParam;
  m_pcTempAlfParam = new ALFParam;
  allocALFParam(m_pcBestAlfParam);
  allocALFParam(m_pcTempAlfParam);
}


Void TEncAdaptiveLoopFilter::endALFEnc()
{
  xUninitParam();

  m_pcPicYuvFiltered->destroy() ;
  delete m_pcPicYuvFiltered ;

  m_pcPicYuvTmp->destroy();
  delete m_pcPicYuvTmp ;
//  m_pcPicYuvTmp = NULL;
  m_pcPic = NULL;
  m_pcEntropyCoder = NULL;
  delete m_pcCoderFixedBits;

  freeALFParam(m_pcBestAlfParam);
  freeALFParam(m_pcTempAlfParam);
  delete m_pcBestAlfParam ;
  delete m_pcTempAlfParam;
}

/** \param  pcAlfParam          ALF parameter
    \param  dLambda             lambda value for RD cost computation
    \retval ruiDist             distortion
    \retval ruiBits             required bits
    \retval ruiMaxAlfCtrlDepth  optimal partition depth
 */
Void TEncAdaptiveLoopFilter::ALFProcess( ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth )
{
  // set lambda
  m_dLambdaLuma   = dLambda;
  m_dLambdaChroma = dLambda;

  TComPicYuv* pcPicOrg             = m_pcPic->getPicYuvOrg();
  TComPicYuv* pcPicYuvRec          = m_pcPic->getPicYuvRec();

  TComPicYuv* pcPicYuvFiltered = new TComPicYuv ;
  pcPicYuvFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth ) ;

  // set min cost
  UInt64 uiMinRate = MAX_INT;
  UInt64 uiMinDist = MAX_INT;
  Double dMinCost  = MAX_DOUBLE;

  UInt64  uiOrigRate;
  UInt64  uiOrigDist;
  Double  dOrigCost;

  // calc original cost
  xCalcRDCost( pcPicOrg, pcPicYuvRec, NULL, uiOrigRate, uiOrigDist, dOrigCost,0 );

  Int iInitSymmetry = m_bALFSymmetry ? 1 : 0;
  pcAlfParam->chroma_idc      = 0;
  pcAlfParam->cu_control_flag = 0;
  pcAlfParam->alf_flag        = 0;
  pcAlfParam->bSeparateQt      = m_bALFSeparateQt;
  xFillAlfFilterInitParam( pcAlfParam->acHorizontalAlfFilter[0], ALF_MAX_HORIZ_LENGTH, iInitSymmetry  );
  xFillAlfFilterInitParam( pcAlfParam->acVerticalAlfFilter  [0], ALF_MAX_VERT_LENGTH , iInitSymmetry  );
  for(Int i = 0; i<3; i++)
    pcAlfParam->aiPlaneFilterMapping[i] = i ;

  // adaptive in-loop wiener filtering
  Int iPlane = 0 ;

  // adaptive filter-lengths
  xFilterTapDecision( pcPicOrg, pcPicYuvRec, pcPicYuvFiltered, pcAlfParam, uiMinRate, uiMinDist, dMinCost, iPlane)  ;

  xCUAdaptiveControl( pcPicOrg, pcPicYuvRec, pcPicYuvFiltered, pcAlfParam, uiMinRate, uiMinDist, dMinCost ) ;


  if( dMinCost < dOrigCost )
  {
    pcPicYuvFiltered->copyToPicLuma( pcPicYuvRec );
  }
  // if ALF works
  if( pcAlfParam->alf_flag )
  {
    if( ALF_FILT_FOR_CHROMA ==2 )
    {
      for (iPlane=1;iPlane<3; iPlane++)
      {
        uiMinRate = MAX_INT;
        uiMinDist = MAX_INT;
        dMinCost  = MAX_DOUBLE;

        // calc original cost
        xCalcRDCost( pcPicOrg, pcPicYuvRec, pcAlfParam, uiOrigRate, uiOrigDist, dOrigCost, iPlane );

        xFillAlfFilterInitParam( pcAlfParam->acHorizontalAlfFilter[iPlane], ALF_MAX_HORIZ_LENGTH, iInitSymmetry );
        xFillAlfFilterInitParam( pcAlfParam->acVerticalAlfFilter  [iPlane], ALF_MAX_VERT_LENGTH , iInitSymmetry );
        xFilterTapDecision(pcPicOrg, pcPicYuvRec, pcPicYuvFiltered, pcAlfParam, uiMinRate, uiMinDist, dMinCost, iPlane)  ;
        xCheckFilterReuse(pcPicOrg, pcPicYuvRec, pcPicYuvFiltered, pcAlfParam, dMinCost, iPlane) ;

        if( dMinCost < dOrigCost )
        {
          if (iPlane == 1)
            pcPicYuvFiltered->copyToPicCb( pcPicYuvRec );
          else if (iPlane == 2)
            pcPicYuvFiltered->copyToPicCr( pcPicYuvRec );
        }
      }
    }
    // predict ALF coefficients
   xPredictALFCoeff( pcAlfParam );
  }

  // copy to best storage
  // store best depth
  ruiMaxAlfCtrlDepth = m_pcEntropyCoder->getMaxAlfCtrlDepth();
  pcPicYuvFiltered->destroy() ;
  delete pcPicYuvFiltered ;
}



// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
Void TEncAdaptiveLoopFilter::xPredictALFCoeff(ALFParam *pALFParam)
{
  for(Int i=0; i<ALF_FILT_FOR_CHROMA+1; i++)
  {
    if( (pALFParam->chroma_idc & i) || (i==0 ) )
    {
        xPredictALFCoeff(pALFParam, i);
    }
  }
}


Void TEncAdaptiveLoopFilter::xPredictALFCoeff(ALFParam *pALFParam, Int iPlane)
{
  Int iCenterPos = 0;
  Int iLength;
// horizontal
  iLength = pALFParam->acHorizontalAlfFilter[iPlane].iFilterLength;
  iCenterPos =  iLength >>1;

  Int iSum = 0 ;
  for (Int i = 0; i< pALFParam->acHorizontalAlfFilter[iPlane].iFilterLength; i++)
  {
    if(i!=iCenterPos)
      iSum += pALFParam->acHorizontalAlfFilter[iPlane].aiQuantFilterCoeffs[ pALFParam->acHorizontalAlfFilter[iPlane].aiTapCoeffMapping[i] ] ;
  }
  pALFParam->acHorizontalAlfFilter[iPlane].aiQuantFilterCoeffs[iCenterPos]-= ((1<<ALF_NUM_BIT_SHIFT) - iSum) ;
  if(pALFParam->acHorizontalAlfFilter[iPlane].iFilterSymmetry == 0)
  {
    for(Int j=0 ; j < iCenterPos  ; j++ )
        pALFParam->acHorizontalAlfFilter[iPlane].aiQuantFilterCoeffs[iLength-j-1] -= pALFParam->acHorizontalAlfFilter[iPlane].aiQuantFilterCoeffs[j];
  }
// vertical
  iLength = pALFParam->acVerticalAlfFilter[iPlane].iFilterLength;
  iCenterPos = iLength >> 1;

  iSum = 0 ;
  for (Int i = 0; i< pALFParam->acVerticalAlfFilter[iPlane].iFilterLength; i++)
  {
    if(i!=iCenterPos)
      iSum += pALFParam->acVerticalAlfFilter[iPlane].aiQuantFilterCoeffs[ pALFParam->acVerticalAlfFilter[iPlane].aiTapCoeffMapping[i] ] ;
  }
  pALFParam->acVerticalAlfFilter[iPlane].aiQuantFilterCoeffs[iCenterPos]-= ((1<<ALF_NUM_BIT_SHIFT) - iSum) ;

  if(pALFParam->acVerticalAlfFilter[iPlane].iFilterSymmetry == 0)
  {
    for(Int j=0 ; j < iCenterPos  ; j++ )
      pALFParam->acVerticalAlfFilter[iPlane].aiQuantFilterCoeffs[iLength-j-1] -= pALFParam->acVerticalAlfFilter[iPlane].aiQuantFilterCoeffs[j];
  }
}

Void TEncAdaptiveLoopFilter::xCheckFilterReuse(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicFiltered, ALFParam* pcFilterParam, Double& rdMinCost, Int iPlane)
{
  UInt64 uiRate   ;
  UInt64 uiDist   ;
  Double dCmpCost   = rdMinCost;
  
  TComPicYuv* pcTmpPicVerticallyFiltered = new TComPicYuv;
  TComPicYuv* pcTmpPicHorizontallyFiltered = new TComPicYuv;

  pcTmpPicVerticallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth) ;
  pcTmpPicHorizontallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth) ;

  ALFParam* pcTmpAlfParams = new ALFParam ;
  allocALFParam( pcTmpAlfParams );

  for(Int iPrevPlane=0; iPrevPlane<iPlane; iPrevPlane++)
  {
    if ( ( pcFilterParam->chroma_idc & iPrevPlane) || (iPrevPlane==0) )
    {
      // set filters of iPlane to filters of iPrevPlane in m_pcTempAlfParam
      copyALFParam( pcTmpAlfParams, pcFilterParam) ;
      pcTmpAlfParams->aiPlaneFilterMapping[iPlane] = iPrevPlane ;

      if( !( pcTmpAlfParams->chroma_idc & iPlane ))
        pcTmpAlfParams->chroma_idc += iPlane ;

      copyALFFilter(pcTmpAlfParams->acHorizontalAlfFilter[iPlane], pcFilterParam->acHorizontalAlfFilter[iPrevPlane]);
      copyALFFilter(pcTmpAlfParams->acVerticalAlfFilter[iPlane], pcFilterParam->acVerticalAlfFilter[iPrevPlane]);

      // check for every previous coded plane if the recent plane can be coded with a previous filter
      xApplyFrame( pcPicDec, pcTmpPicVerticallyFiltered, pcTmpAlfParams->acVerticalAlfFilter[iPlane], iPlane);
      xApplyFrame( pcTmpPicVerticallyFiltered , pcTmpPicHorizontallyFiltered, pcTmpAlfParams->acHorizontalAlfFilter[iPlane], iPlane);

      // check new rdcosts
      xCalcRDCost( pcPicOrg, pcTmpPicHorizontallyFiltered, pcTmpAlfParams, uiRate, uiDist, dCmpCost, iPlane );

      if(dCmpCost < rdMinCost )
      {
        rdMinCost = dCmpCost;
        copyALFParam(pcFilterParam, pcTmpAlfParams);
        if(iPlane == 1)
          pcTmpPicHorizontallyFiltered->copyToPicCb(pcPicFiltered);
        else if(iPlane  == 2)
          pcTmpPicHorizontallyFiltered->copyToPicCr(pcPicFiltered);
      }
    }
  }
  pcTmpPicVerticallyFiltered->destroy() ;
  pcTmpPicHorizontallyFiltered->destroy() ;
  delete pcTmpPicVerticallyFiltered ;
  delete pcTmpPicHorizontallyFiltered ;
  freeALFParam( pcTmpAlfParams );
  delete pcTmpAlfParams ;
}

Void TEncAdaptiveLoopFilter::xFilterTapDecision(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicFiltered, ALFParam* pcAlfParams, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost, Int iPlane)
{
  // restriction for non-referenced B-slice
  // if (m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0)
  // {
  //   return;
  // }
  UInt64 uiTmpRate, uiTmpDist;
  Double dTmpCost;
  TComPicYuv* pcTmpPicVerticallyFiltered = new TComPicYuv;
  TComPicYuv* pcTmpPicHorizontallyFiltered = new TComPicYuv;
  ALFParam* pcTmpAlfParams = new ALFParam ;
  allocALFParam( pcTmpAlfParams );


  xCalcRDCost(pcPicOrg, pcPicDec, NULL , ruiMinRate, ruiMinDist, rdMinCost, iPlane );

  pcTmpPicVerticallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth) ;
  pcTmpPicHorizontallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth) ;

  pcPicDec->copyToPic( pcPicFiltered ) ;
  copyALFParam( pcTmpAlfParams, pcAlfParams );
  uiTmpRate = ruiMinRate ;
  uiTmpDist = ruiMinDist ;
  dTmpCost  = rdMinCost ;
  pcTmpAlfParams->alf_flag = 1 ;
  pcTmpAlfParams->chroma_idc += iPlane ;

  for (Int iLengthVert = m_iALFMaxLength ; iLengthVert >= m_iALFMinLength ; iLengthVert -= 2 )
  {
    xFillAlfFilterInitParam( pcTmpAlfParams->acVerticalAlfFilter[iPlane] , iLengthVert , pcTmpAlfParams->acVerticalAlfFilter[iPlane].iFilterSymmetry );
    xEstimateFrameFilter   ( pcPicOrg, pcPicDec      , pcTmpAlfParams->acVerticalAlfFilter[iPlane], false , m_puiCUVerticalCorr, iPlane   );
    xApplyFrame            ( pcPicDec, pcTmpPicVerticallyFiltered, pcTmpAlfParams->acVerticalAlfFilter[iPlane], iPlane );

    for (Int iLengthHoriz = m_iALFMaxLength ; iLengthHoriz >= m_iALFMinLength ; iLengthHoriz -= 2 )
    {
      xFillAlfFilterInitParam( pcTmpAlfParams->acHorizontalAlfFilter[iPlane] , iLengthHoriz , pcTmpAlfParams->acHorizontalAlfFilter[iPlane].iFilterSymmetry );
      xEstimateFrameFilter   ( pcPicOrg,        pcTmpPicVerticallyFiltered, pcTmpAlfParams->acHorizontalAlfFilter[iPlane], false , NULL, iPlane   );
      xApplyFrame            ( pcTmpPicVerticallyFiltered , pcTmpPicHorizontallyFiltered , pcTmpAlfParams->acHorizontalAlfFilter[iPlane], iPlane );

      xCalcRDCost(pcPicOrg, pcTmpPicHorizontallyFiltered, pcTmpAlfParams , uiTmpRate, uiTmpDist, dTmpCost, iPlane );
      if (dTmpCost < rdMinCost)
      {
        rdMinCost = dTmpCost;
        ruiMinDist = uiTmpDist;
        ruiMinRate = uiTmpRate;
        if (iPlane == 0)
          pcTmpPicHorizontallyFiltered->copyToPicLuma( pcPicFiltered   );
        else if (iPlane == 1)
          pcTmpPicHorizontallyFiltered->copyToPicCb  ( pcPicFiltered   );
        else if (iPlane == 2)
          pcTmpPicHorizontallyFiltered->copyToPicCr  ( pcPicFiltered   );
        copyALFParam( pcAlfParams, pcTmpAlfParams );
      }
    }
  }
  freeALFParam( pcTmpAlfParams );
  pcTmpPicHorizontallyFiltered->destroy();
  pcTmpPicVerticallyFiltered->destroy();
  delete pcTmpAlfParams;
  delete pcTmpPicHorizontallyFiltered ;
  delete pcTmpPicVerticallyFiltered ;

}


Void TEncAdaptiveLoopFilter::xCUAdaptiveControl( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicFiltered, ALFParam* pcAlfParam, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost )
{
  m_pcEntropyCoder->setAlfCtrl(true);
  UInt uiBestDepth = 0;

  ALFParam* pcTmpAlfParam = new ALFParam ;
  allocALFParam(pcTmpAlfParam);
  copyALFParam(pcTmpAlfParam, pcAlfParam);

  ALFParam* pcFrmAlfParam = new ALFParam ;
  allocALFParam(pcFrmAlfParam);
  copyALFParam(pcFrmAlfParam, pcAlfParam);

  TComPicYuv* pcFilteredFrm = new TComPicYuv();
  pcFilteredFrm->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
  pcPicFiltered->copyToPic( pcFilteredFrm );

  TComPicYuv* pcTmpPicVerticallyFiltered = new TComPicYuv;
  pcTmpPicVerticallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
  TComPicYuv* pcTmpPicHorizontallyFiltered = new TComPicYuv;
  pcTmpPicHorizontallyFiltered->create(pcPicOrg->getWidth(), pcPicOrg->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);

  TComPicSym* pcQuadTreeBest;
  TComPicSym* pcQuadTreeTmp;
  TComPicSym* pcQuadTreeHelp;
  if( m_bALFSeparateQt )
  {
    pcQuadTreeBest = new TComPicSym();
    pcQuadTreeBest->create( pcPicDec->getWidth(), pcPicDec->getHeight(),  g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
    pcQuadTreeTmp = new TComPicSym();
    pcQuadTreeTmp->create( pcPicDec->getWidth(), pcPicDec->getHeight(),  g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
    for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTreeBest->getNumberOfCUsInFrame(); uiCUAddr++ )
    {
      TComDataCU* pcCU = pcQuadTreeBest->getCU( uiCUAddr );
      pcCU->initCU( m_pcPic , uiCUAddr );
      pcCU = pcQuadTreeTmp->getCU( uiCUAddr );
      pcCU->initCU( m_pcPic , uiCUAddr );
    }
  }
  else
  {
    pcQuadTreeBest = m_pcPic->getPicSym();
    pcQuadTreeTmp  = m_pcPic->getPicSym();
    xCreateTmpAlfCtrlFlags( pcQuadTreeBest );
  }
  pcQuadTreeHelp = pcQuadTreeTmp;
  bool bUsedRedesign = false;
  for( UInt uiDepth = 0; uiDepth < g_uiMaxCUDepth; uiDepth++ )
  {
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiDepth);
    copyALFParam( pcTmpAlfParam , pcFrmAlfParam );
    pcTmpAlfParam->cu_control_flag = 1;

    for( UInt uiRD = 0; uiRD <= ALF_NUM_OF_REDESIGN; uiRD++ )
    {
      UInt64 uiRate, uiDist, uiEstRate;
      Double dCost;
      if( uiRD )
      {
        // re-design filter coefficients
        xReDesignFilterCoeff( pcQuadTreeHelp, pcPicOrg , pcPicDec, pcTmpAlfParam->acVerticalAlfFilter[0] , m_puiCUVerticalCorr, false );
        xApplyFrame( pcPicDec , pcTmpPicVerticallyFiltered , pcTmpAlfParam->acVerticalAlfFilter  [0]);
        xReDesignFilterCoeff( pcQuadTreeHelp, pcPicOrg , pcTmpPicVerticallyFiltered, pcTmpAlfParam->acHorizontalAlfFilter[0] , m_puiCUHorizontalCorr, false);
        xApplyFrame( pcTmpPicVerticallyFiltered , pcTmpPicHorizontallyFiltered , pcTmpAlfParam->acHorizontalAlfFilter[0] );
        //xCUAdaptive( m_pcPic, cFrmAlfParam.acHorizontalAlfFilter[0], m_pcPicYuvTmp, pcPicRest);
        xSetCUAlfCtrlFlags(uiDepth, pcPicOrg, pcPicDec, pcTmpPicHorizontallyFiltered, pcQuadTreeTmp, uiDist, uiEstRate);
      }
      else
      {
        xSetCUAlfCtrlFlags(uiDepth, pcPicOrg, pcPicDec, pcFilteredFrm, pcQuadTreeTmp, uiDist, uiEstRate);
      }

      pcTmpAlfParam->pcQuadTree = pcQuadTreeTmp;
      xCalcRDCost( pcTmpAlfParam , uiRate, uiDist, dCost );

      pcQuadTreeHelp = pcQuadTreeTmp;
      if( dCost < rdMinCost )
      {
        pcQuadTreeHelp = pcQuadTreeBest;
        pcQuadTreeBest = pcQuadTreeTmp;
        pcQuadTreeTmp  = pcQuadTreeHelp;
        pcQuadTreeHelp = pcQuadTreeBest;
        uiBestDepth = uiDepth;
        rdMinCost   = dCost;
        ruiMinDist  = uiDist;
        ruiMinRate  = uiRate;
        bUsedRedesign = uiRD ? true : false ;
        if( uiRD )
        {
          pcTmpPicHorizontallyFiltered->copyToPicLuma( pcPicFiltered );
        }
        copyALFParam( pcAlfParam , pcTmpAlfParam );
        if( !m_bALFSeparateQt )
          xCopyTmpAlfCtrlFlagsFrom( pcQuadTreeBest );
      }
    }
  }

  if (pcAlfParam->cu_control_flag)
  {
    m_pcEntropyCoder->setAlfCtrl(true);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiBestDepth);
    if( !m_bALFSeparateQt)
    {
      xCopyTmpAlfCtrlFlagsTo( pcQuadTreeBest );
    }
    if( !bUsedRedesign )
    {
      pcFilteredFrm->copyToPicLuma( pcPicFiltered );
    }
    xCopyDecToRestCUs( pcQuadTreeBest , pcPicDec , pcPicFiltered  );
  }
  else
  {
    m_pcEntropyCoder->setAlfCtrl(false);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(0);
    if( m_bALFSeparateQt )
    {
      pcQuadTreeBest->destroy();
      delete pcQuadTreeBest;
    }
  }

  if( m_bALFSeparateQt)
  {
    pcQuadTreeTmp->destroy();
    delete pcQuadTreeTmp;
  }
  else
  {
    xDestroyTmpAlfCtrlFlags( pcQuadTreeBest );
  }

  pcFilteredFrm->destroy();
  delete pcFilteredFrm;

  //
  freeALFParam(pcTmpAlfParam);
  delete pcTmpAlfParam ;
  freeALFParam(pcFrmAlfParam);
  delete pcFrmAlfParam ;

  pcTmpPicVerticallyFiltered->destroy() ;
  delete pcTmpPicVerticallyFiltered ;
  pcTmpPicHorizontallyFiltered->destroy() ;
  delete pcTmpPicHorizontallyFiltered ;

}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

Void TEncAdaptiveLoopFilter::xInitParam()
{
  Int i, j, k, l;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      for (j = 0; j < ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }
  else
  {
    m_ppdAlfCorr = new Double*[ALF_MAX_NUM_COEF];
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_ppdAlfCorr[i] = new Double[ALF_MAX_NUM_COEF+1];
      for (j = 0; j < ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }

  if (m_puiCUVerticalCorr != NULL)
  {
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        for (k = 0; k < ALF_MAX_NUM_COEF; k++)
        {
          for (l = 0; l< ALF_MAX_NUM_COEF+1; l++)
          {
            m_puiCUVerticalCorr[i][j][k][l] = 0;
          }
        }
      }
    }
  }
  else
  {
    m_puiCUVerticalCorr = new UInt***[m_pcPic->getNumCUsInFrame()];
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      m_puiCUVerticalCorr[i] = new UInt**[m_uiNumSCUInCU];
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        m_puiCUVerticalCorr[i][j] = new UInt*[ALF_MAX_NUM_COEF];
        for (k = 0; k < ALF_MAX_NUM_COEF; k++)
        {
          m_puiCUVerticalCorr[i][j][k] = new UInt[ALF_MAX_NUM_COEF+1];
          for (l = 0; l< ALF_MAX_NUM_COEF+1; l++)
          {
            m_puiCUVerticalCorr[i][j][k][l] = 0;
          }
        }
      }
    }
  }


  if (m_puiCUHorizontalCorr != NULL)
  {
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        for (k = 0; k < ALF_MAX_NUM_COEF; k++)
        {
          for (l = 0; l< ALF_MAX_NUM_COEF+1; l++)
          {
            m_puiCUHorizontalCorr[i][j][k][l] = 0;
          }
        }
      }
    }
  }
  else
  {
    m_puiCUHorizontalCorr = new UInt***[m_pcPic->getNumCUsInFrame()];
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      m_puiCUHorizontalCorr[i] = new UInt**[m_uiNumSCUInCU];
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        m_puiCUHorizontalCorr[i][j] = new UInt*[ALF_MAX_NUM_COEF];
        for (k = 0; k < ALF_MAX_NUM_COEF; k++)
        {
          m_puiCUHorizontalCorr[i][j][k] = new UInt[ALF_MAX_NUM_COEF+1];
          for (l = 0; l< ALF_MAX_NUM_COEF+1; l++)
          {
            m_puiCUHorizontalCorr[i][j][k][l] = 0;
          }
        }
      }
    }
  }


  if (m_pdDoubleAlfCoeff != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
  else
  {
    m_pdDoubleAlfCoeff = new Double[ALF_MAX_NUM_COEF];
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
}

Void TEncAdaptiveLoopFilter::xUninitParam()
{
  Int i, j, k;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      delete[] m_ppdAlfCorr[i];
      m_ppdAlfCorr[i] = NULL;
    }
    delete[] m_ppdAlfCorr;
    m_ppdAlfCorr = NULL;
  }

  if (m_puiCUVerticalCorr != NULL)
  {
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        for (k = 0; k < ALF_MAX_NUM_COEF; k++)
        {
          delete[] m_puiCUVerticalCorr[i][j][k];
          m_puiCUVerticalCorr[i][j][k] = NULL;
        }
        delete[] m_puiCUVerticalCorr[i][j];
        m_puiCUVerticalCorr[i][j] = NULL;
      }
      delete[] m_puiCUVerticalCorr[i];
      m_puiCUVerticalCorr[i] = NULL;
    }
    delete[] m_puiCUVerticalCorr;
    m_puiCUVerticalCorr = NULL;
  }

  if (m_puiCUHorizontalCorr != NULL)
    {
      for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
      {
        for (j = 0; j < m_uiNumSCUInCU; j++)
        {
          for (k = 0; k < ALF_MAX_NUM_COEF; k++)
          {
            delete[] m_puiCUHorizontalCorr[i][j][k];
            m_puiCUHorizontalCorr[i][j][k] = NULL;
          }
          delete[] m_puiCUHorizontalCorr[i][j];
          m_puiCUHorizontalCorr[i][j] = NULL;
        }
        delete[] m_puiCUHorizontalCorr[i];
        m_puiCUHorizontalCorr[i] = NULL;
      }
      delete[] m_puiCUHorizontalCorr;
      m_puiCUHorizontalCorr = NULL;
    }


  if (m_pdDoubleAlfCoeff != NULL)
  {
    delete[] m_pdDoubleAlfCoeff;
    m_pdDoubleAlfCoeff = NULL;
  }
}

Void TEncAdaptiveLoopFilter::xCreateTmpAlfCtrlFlags( TComPicSym* pcQuadTree )
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame(); uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    pcCU->createTmpAlfCtrlFlag();
  }
}

Void TEncAdaptiveLoopFilter::xDestroyTmpAlfCtrlFlags( TComPicSym* pcQuadTree )
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame(); uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    pcCU->destroyTmpAlfCtrlFlag();
  }
}

Void TEncAdaptiveLoopFilter::xCopyTmpAlfCtrlFlagsTo( TComPicSym* pcQuadTree )
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    pcCU->copyAlfCtrlFlagFromTmp();
  }
}

Void TEncAdaptiveLoopFilter::xCopyTmpAlfCtrlFlagsFrom( TComPicSym* pcQuadTree )
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    pcCU->copyAlfCtrlFlagToTmp();
  }
}

Void TEncAdaptiveLoopFilter::xReadOrCalcCorrFromCUs(TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr )
{
  Int iNumCoeffs = rcFilter.iNumOfCoeffs;

  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    for (UInt uiIdx = 0; uiIdx < pcCU->getTotalNumPart(); uiIdx+=4)
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if (uiLPelX >= pcPicOrg->getWidth() || uiTPelY >= pcPicOrg->getHeight())
      {
        continue;
      }

      if (pcCU->getAlfCtrlFlag(uiIdx))
      {
        if (bReadCorr)
        {
#if ALF_DC_CONSIDERED
          for(Int j=0 ; j < iNumCoeffs + 1; j++ )
          {
            for(Int k = j ; k < iNumCoeffs + 1 ; k++ )
            {
              m_ppdAlfCorr[k][j] += (Double) ppdAlfCorr[uiCUAddr][(uiIdx>>2)][k][j];
            }
            m_ppdAlfCorr[j][iNumCoeffs + 1 ] += ( Double ) ppdAlfCorr[uiCUAddr][(uiIdx>>2)][j][ iNumCoeffs + 1];
          }
#else
          for(Int j=0 ; j < iNumCoeffs; j++ )
          {
            for(Int k = j ; k < iNumCoeffs; k++ )
            {
              m_ppdAlfCorr[k][j] += (Double) ppdAlfCorr[uiCUAddr][(uiIdx>>2)][k][j];
            }
          }
#endif
        }
        else
        {
          xEstimateCorrCU( pcPicOrg , pcPicDec, pcCU, uiIdx, *pcCU->getDepth(), rcFilter, m_ppdAlfCorr );
        }
      }
    }
  }
}


Void TEncAdaptiveLoopFilter::xReadOrCalcCorrFromFUs(TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr )
{

  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    xReadOrCalcCorrFromFU( pcQuadTree, pcCU, 0, 0, pcPicOrg, pcPicDec, bReadCorr, rcFilter, ppdAlfCorr );
  }
}

Void TEncAdaptiveLoopFilter::xReadOrCalcCorrFromFU(TComPicSym* pcQuadTree, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr, AlfFilter& rcFilter, UInt**** ppdAlfCorr )
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
        xReadOrCalcCorrFromFU( pcQuadTree, pcCU, uiAbsPartIdx, uiDepth + 1, pcPicOrg, pcPicDec,bReadCorr, rcFilter, ppdAlfCorr );
    }
    return;
  }

  if ( pcCU->getAlfCtrlFlag( uiAbsPartIdx ) )
  {
    if (bReadCorr)
    {
      assert(0);
    }
    else
    {
      xEstimateCorrFU( pcPicOrg , pcPicDec, pcCU, uiAbsPartIdx, uiDepth, rcFilter, m_ppdAlfCorr );
    }
  }
}



Void TEncAdaptiveLoopFilter::encodeQuadTree( ALFParam* pAlfParam, TEncEntropy* pcEntropyCoder, UInt uiMaxAlfCtrlDepth )
{
  encodeFUAlfCtrlFlags( pcEntropyCoder, pAlfParam->pcQuadTree, uiMaxAlfCtrlDepth);
}

Void TEncAdaptiveLoopFilter::encodeFUAlfCtrlFlags(TEncEntropy* pcEntropyCoder, TComPicSym* pcQuadTree , UInt uiMaxDepth )
{
  for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame(); uiCUAddr++ )
  {
    TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
    encodeFUAlfCtrlFlag( pcEntropyCoder, pcQuadTree, pcCU, 0, 0 , uiMaxDepth);
  }
}

Void TEncAdaptiveLoopFilter::encodeFUAlfCtrlFlag(TEncEntropy* pcEntropyCoder, TComPicSym* pcQuadTree, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth )
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
  if( m_bALFSeparateQt )
  {
   pcEntropyCoder->encodeAlfQTSplitFlag( pcCU , uiAbsPartIdx, uiDepth, uiMaxDepth );
  }

  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || ( bBoundary && !m_bALFSeparateQt) )
  {
    UInt uiQNumParts = ( pcQuadTree->getNumPartition() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        encodeFUAlfCtrlFlag(pcEntropyCoder, pcQuadTree, pcCU, uiAbsPartIdx, uiDepth+1 , uiMaxDepth );
    }
    return;
  }
  pcEntropyCoder->encodeAlfCtrlFlag(pcCU, uiAbsPartIdx, false, m_bALFSeparateQt );

}

Void TEncAdaptiveLoopFilter::xCalcALFCoeff( AlfFilter& rcFilter )
{
  Int iErrCode;
  Int iNumCoeffs  = rcFilter.iNumOfCoeffs;

  Double* h             = m_pdDoubleAlfCoeff;
  iErrCode = xGauss( m_ppdAlfCorr , iNumCoeffs );

  if(iErrCode)
  {
    xClearFilterCoefInt( rcFilter );
  }
  else
  {
    for(Int i = 0; i < iNumCoeffs ; i++)
      h[i] = m_ppdAlfCorr[i][iNumCoeffs];
    xQuantFilterCoef(h, rcFilter, g_uiBitDepth + g_uiBitIncrement);
  }
}




UInt64 TEncAdaptiveLoopFilter::xCalcSSD(Pel* pOrg, Pel* pCmp, Int iWidth, Int iHeight, Int iStride )
{
  UInt64 uiSSD = 0;
  Int x, y;

  UInt uiShift = g_uiBitIncrement<<1;
  Int iTemp;

  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pOrg[x] - pCmp[x]; uiSSD += ( iTemp * iTemp ) >> uiShift;
    }
    pOrg += iStride;
    pCmp += iStride;
  }

  return uiSSD;;
}

Int TEncAdaptiveLoopFilter::xGauss(Double **a, Int N)
{
  Int i, j, k;
  Double t;

#if ALF_FIX
  for(k=0; k<N; k++)
  {
    if (a[k][k] <0.000001)
        return 1;
  }
#endif

  for(k=0; k<N-1; k++)
  {
    for(i=k+1;i<N; i++)
    {
      t=a[i][k]/a[k][k];
      for(j=k+1; j<=N; j++)
      {
        a[i][j] -= t * a[k][j];
        if(i==j && fabs(a[i][j])<0.000001) return 1;
      }
    }
  }
  for(i=N-1; i>=0; i--)
  {
    t = a[i][N];
    for(j=i+1; j<N; j++)
      t -= a[i][j] * a[j][N];
    a[i][N] = t / a[i][i];
  }
  return 0;
}

Void TEncAdaptiveLoopFilter::xFilterCoefQuickSort( Double *coef_data, Int *coef_num, Int upper, Int lower )
{
  Double mid, tmp_data;
  Int i, j, tmp_num;

  i = upper;
  j = lower;
  mid = coef_data[(lower+upper)>>1];
  do
  {
    while( coef_data[i] < mid ) i++;
    while( mid < coef_data[j] ) j--;
    if( i <= j )
    {
      tmp_data = coef_data[i];
      tmp_num  = coef_num[i];
      coef_data[i] = coef_data[j];
      coef_num[i]  = coef_num[j];
      coef_data[j] = tmp_data;
      coef_num[j]  = tmp_num;
      i++;
      j--;
    }
  } while( i <= j );
  if ( upper < j ) xFilterCoefQuickSort(coef_data, coef_num, upper, j);
  if ( i < lower ) xFilterCoefQuickSort(coef_data, coef_num, i, lower);
}

Void TEncAdaptiveLoopFilter::xQuantFilterCoef( Double* adDoubleCoeffs, AlfFilter& rcFilter, Int iBit_depth )
{
  Int i;
  const Int iNumCoeffs       = rcFilter.iNumOfCoeffs;
  Int iMaxValue = ( 1 << ( 1 + ALF_NUM_BIT_SHIFT ) ) - 1;
  Int iMinValue = 0 - ( 1 << ( 1 + ALF_NUM_BIT_SHIFT ) );

  Double dGainNotQuant;
  Int    iGainNotQuant;
  Int    iGainQuant;

  Int* aiQuantCoeffs = rcFilter.aiQuantFilterCoeffs;

  Double* adDelta   = new Double[ iNumCoeffs+1 ];
  Int*    aiPosIndx = new Int   [ iNumCoeffs+1 ];


  dGainNotQuant = 0.0;
  iGainQuant    = 0;

  for(i=0 ; i < iNumCoeffs ; i++)
  {
    if( adDoubleCoeffs[i] >= 0.0 )
      aiQuantCoeffs[i] =  (Int)( adDoubleCoeffs[i] * ( 1 << ALF_NUM_BIT_SHIFT ) +0.5 );
    else
      aiQuantCoeffs[i] = -(Int)(-adDoubleCoeffs[i] * ( 1 << ALF_NUM_BIT_SHIFT ) +0.5 );

    adDelta[i]  = (Double) aiQuantCoeffs[i] / (Double) (1<<ALF_NUM_BIT_SHIFT) - adDoubleCoeffs[i];
    adDelta[i] *= rcFilter.aiCoeffWeights[i];

    dGainNotQuant += adDoubleCoeffs[i] * rcFilter.aiCoeffWeights[i];
    iGainQuant    += aiQuantCoeffs [i] * rcFilter.aiCoeffWeights[i];
    aiPosIndx[i]   = i;
  }
#if ALF_DC_CONSIDERED
  // quant dc offset
  if( adDoubleCoeffs[iNumCoeffs] >= 0.0 )
    aiQuantCoeffs[iNumCoeffs] =  (Int)( adDoubleCoeffs[iNumCoeffs] * ( 1 << ALF_NUM_BIT_SHIFT ) +0.5 );
  else
    aiQuantCoeffs[iNumCoeffs] = -(Int)(-adDoubleCoeffs[iNumCoeffs] * ( 1 << ALF_NUM_BIT_SHIFT ) +0.5 );
#endif

  iGainNotQuant = (Int)( dGainNotQuant * ( 1 << ALF_NUM_BIT_SHIFT ) + 0.5 );


 // modification of quantized filter coefficients
  Int iUpper, iLower;
  if( iGainQuant != iGainNotQuant )
  {
    xFilterCoefQuickSort( adDelta , aiPosIndx, 0, iNumCoeffs - 1 );
    if( iGainQuant > iGainNotQuant )
    {
      iUpper = iNumCoeffs - 1;
      while( iGainQuant > iGainNotQuant )
      {
        i = aiPosIndx[ iUpper % iNumCoeffs ];
        aiQuantCoeffs[ i ]--;
        iGainQuant -= rcFilter.aiCoeffWeights[i];
        iUpper--;
      }
    }
    else if( iGainQuant < iGainNotQuant )
    {
      iLower = 0;
      while( iGainQuant < iGainNotQuant )
      {
        i = aiPosIndx[ iLower % iNumCoeffs ];
        aiQuantCoeffs[ i ]++;
        iGainQuant += rcFilter.aiCoeffWeights[i];
        iLower++;
      }
    }
  }
  // set of filter coefficients
  for( i=0; i < iNumCoeffs ; i++ )
  {
    aiQuantCoeffs[i] = Max( iMinValue , Min( iMaxValue , aiQuantCoeffs[i] ) );
  }

  delete[] adDelta;
  adDelta = NULL;

  delete[] aiPosIndx;
  aiPosIndx = NULL;
}

Void TEncAdaptiveLoopFilter::xClearFilterCoefInt( AlfFilter& rcFilter )
{
  // clear
  memset( rcFilter.aiQuantFilterCoeffs , 0, sizeof( Int ) * ( rcFilter.iNumOfCoeffs + 1 ) );
  rcFilter.bIsValid = false;
}


Void TEncAdaptiveLoopFilter::xCalcRDCost(ALFParam* pAlfParam, UInt64& ruiRate, UInt64 uiDist, Double& rdCost )
{
  if(pAlfParam != NULL)
  {
    ALFParam* pcTempAlfParams = new ALFParam ;
    allocALFParam( pcTempAlfParams );
    copyALFParam(pcTempAlfParams, pAlfParam);
    xPredictALFCoeff(pcTempAlfParams, 0 );
    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();

    if(pAlfParam->cu_control_flag)
    {
      encodeFUAlfCtrlFlags( m_pcEntropyCoder, pAlfParam->pcQuadTree , m_pcEntropyCoder->getMaxAlfCtrlDepth() );
    }

    m_pcEntropyCoder->encodeAlfParam( pcTempAlfParams );
    freeALFParam( pcTempAlfParams );
    delete pcTempAlfParams ;
    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    ruiRate = 1;
  }
  rdCost = (Double)(ruiRate) * m_dLambdaLuma + (Double)(uiDist);
}

Void TEncAdaptiveLoopFilter::xCalcRDCost(TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost, Int iPlane)
{
  if(pAlfParam != NULL)
  {
    ALFParam* pcTempAlfParams = new ALFParam ;
    allocALFParam( pcTempAlfParams );
    copyALFParam(pcTempAlfParams, pAlfParam);
    xPredictALFCoeff(pcTempAlfParams, iPlane );
    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam(pcTempAlfParams);
    freeALFParam( pcTempAlfParams );
    delete pcTempAlfParams ;

    if(pAlfParam->cu_control_flag)
    {
      encodeFUAlfCtrlFlags( m_pcEntropyCoder, pAlfParam->pcQuadTree, m_pcEntropyCoder->getMaxAlfCtrlDepth() );
    }
    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  else
  {
    ruiRate = 1;
  }
  if (iPlane == 0)
    ruiDist     = xCalcSSD(pcPicOrg->getLumaAddr(), pcPicCmp->getLumaAddr(), pcPicOrg->getWidth(), pcPicOrg->getHeight(), pcPicOrg->getStride());
  else if (iPlane == 1)
    ruiDist     = xCalcSSD(pcPicOrg->getCbAddr(), pcPicCmp->getCbAddr(), pcPicOrg->getWidth()>>1, pcPicOrg->getHeight()>>1, pcPicOrg->getCStride());
  else if (iPlane == 2)
      ruiDist     = xCalcSSD(pcPicOrg->getCrAddr(), pcPicCmp->getCrAddr(), pcPicOrg->getWidth()>>1, pcPicOrg->getHeight()>>1, pcPicOrg->getCStride());

  rdCost      = (Double)(ruiRate) * m_dLambdaLuma + (Double)(ruiDist);
}




Void TEncAdaptiveLoopFilter::xEstimateFrameFilter( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, AlfFilter& rcFilter , Bool bStoreCorr , UInt**** puiCUCorr, Int iPlane )
{
  Int  iNumCoeffs   = rcFilter.iNumOfCoeffs;
#if ALF_DC_CONSIDERED
  for(Int i = 0 ; i < iNumCoeffs + 1 ; i++ )
  {
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*( iNumCoeffs + 2 ) );
  }
#else
  for(Int i = 0 ; i < iNumCoeffs ; i++ )
  {
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*( iNumCoeffs + 1 ) );
  }
#endif
  if( bStoreCorr && (iPlane ==0 ))
  {
    UInt** ppuiBlkCorr;
    // store correlation per minimum size cu
    for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
    {
      for( UInt uiIdx = 0; uiIdx < m_pcPic->getNumPartInCU() ; uiIdx += 4 )
      {
        TComDataCU* pcCU = m_pcPic->getCU(uiCUAddr);
        UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiIdx ] ];
        UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiIdx ] ];

        if (uiLPelX >= pcPicOrg->getWidth() || uiTPelY >= pcPicOrg->getHeight())
        {
          continue;
        }
        ppuiBlkCorr  = puiCUCorr[uiCUAddr][(uiIdx>>2)];
#if ALF_DC_CONSIDERED
        for(Int i = 0 ; i < iNumCoeffs ; i++ )
        {
          memset( ppuiBlkCorr[i], 0, sizeof(UInt)*( iNumCoeffs + 1 - i) );
        }

        xEstimateCorrCU( pcPicOrg, pcPicDec, pcCU, uiIdx, *pcCU->getDepth(), rcFilter, ppuiBlkCorr );
        for(Int j=0 ; j < iNumCoeffs + 1; j++ )
        {
          for(Int k = j ; k < iNumCoeffs + 1 ; k++ )
          {
            m_ppdAlfCorr[k][j] += (Double) ppuiBlkCorr[ k ][ j ];
          }
          m_ppdAlfCorr[j][iNumCoeffs + 1 ] += ( Double ) ppuiBlkCorr[ j ][ iNumCoeffs + 1 ];
        }
#else
        for(Int i = 0 ; i < iNumCoeffs ; i++ )
        {
          memset( ppuiBlkCorr[i], 0, sizeof(UInt)*( iNumCoeffs - i) );
        }

        xEstimateCorrCU( pcPicOrg, pcPicDec, pcCU, uiIdx, *pcCU->getDepth(), rcFilter, ppuiBlkCorr );
        for(Int j=0 ; j < iNumCoeffs ; j++ )
        {
          for(Int k = j ; k < iNumCoeffs  ; k++ )
          {
            m_ppdAlfCorr[k][j] += (Double) ppuiBlkCorr[ k ][ j ];
          }
        }
#endif
      }
    }
  }
  else
  {
    Pel* pOrg = NULL;
    Pel* pCmp = NULL;
    Int iWidth = 0;
    Int iHeight = 0;
    Int iOrgStride = 0;
    Int iDecStride = 0;
    if (iPlane == 0)
    {
      pOrg        = pcPicOrg->getLumaAddr();
      pCmp        = pcPicDec->getLumaAddr();
      iWidth      = pcPicOrg->getWidth() ;
      iHeight     = pcPicOrg->getHeight() ;
      iOrgStride  = pcPicOrg->getStride() ;
      iDecStride  = pcPicDec->getStride() ;
    }
    else if (iPlane == 1)
    {
      pOrg        = pcPicOrg->getCbAddr();
      pCmp        = pcPicDec->getCbAddr();
      iWidth      = pcPicOrg->getWidth()>>1 ;
      iHeight     = pcPicOrg->getHeight()>>1 ;
      iOrgStride  = pcPicOrg->getCStride() ;
      iDecStride  = pcPicDec->getCStride() ;
    }
    if (iPlane == 2)
    {
      pOrg        = pcPicOrg->getCrAddr();
      pCmp        = pcPicDec->getCrAddr();
      iWidth      = pcPicOrg->getWidth()>>1 ;
      iHeight     = pcPicOrg->getHeight()>>1 ;
      iOrgStride  = pcPicOrg->getCStride() ;
      iDecStride  = pcPicDec->getCStride() ;
    }

    xEstimateCorr( pOrg, pCmp, iWidth, iHeight, iOrgStride, iDecStride, rcFilter , m_ppdAlfCorr , 0 , 0 , iWidth, iHeight );
  }
#if ALF_DC_CONSIDERED
  for( Int j=0 ; j < iNumCoeffs ; j++ )
    {
      for( Int k=j+1; k< iNumCoeffs + 1 ; k++)
      {
        m_ppdAlfCorr[j][k] = m_ppdAlfCorr[k][j];
      }
    }
  Int iErr_code = xGauss(m_ppdAlfCorr, iNumCoeffs + 1);
#else
  for( Int j=0 ; j < iNumCoeffs ; j++ )
  {
    for( Int k=j+1; k< iNumCoeffs  ; k++)
    {
      m_ppdAlfCorr[j][k] = m_ppdAlfCorr[k][j];
    }
  }
  Int iErr_code = xGauss(m_ppdAlfCorr, iNumCoeffs );
#endif
  if(iErr_code)
  {
    xClearFilterCoefInt( rcFilter );
  }
  else
  {
#if ALF_DC_CONSIDERED
    for(Int i = 0; i < iNumCoeffs+1 ; i++)
    {
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][iNumCoeffs+1];
    }
#else
    for(Int i = 0; i < iNumCoeffs ; i++)
    {
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][iNumCoeffs];
    }
#endif
    xQuantFilterCoef( m_pdDoubleAlfCoeff, rcFilter, g_uiBitDepth + g_uiBitIncrement);
    rcFilter.bIsValid = true;
  }
}


Void TEncAdaptiveLoopFilter::xReDesignFilterCoeff( TComPicSym* pcQuadTree, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, AlfFilter& rcFilter, UInt**** ppuiAlfCorr, Bool bReadCorr)
{

#if ALF_DC_CONSIDERED
  Int N = rcFilter.iNumOfCoeffs + 1;
#else
  Int N = rcFilter.iNumOfCoeffs;
#endif
  // initialize correlation

  for(Int i=0; i<N; i++)
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*(N+1));

  if( m_bALFSeparateQt )
    xReadOrCalcCorrFromFUs( pcQuadTree, pcPicOrg, pcPicDec, bReadCorr, rcFilter, ppuiAlfCorr );
  else
    xReadOrCalcCorrFromCUs( pcQuadTree, pcPicOrg, pcPicDec, bReadCorr, rcFilter, ppuiAlfCorr );


  for( Int j=0 ; j < N - 1 ; j++ )
  {
    for( Int k=j+1; k< N ; k++)
    {
      m_ppdAlfCorr[j][k] = m_ppdAlfCorr[k][j];
    }
  }

  Int err_code = xGauss(m_ppdAlfCorr, N);

  if(err_code)
  {
    xClearFilterCoefInt( rcFilter );
  }
  else
  {
    for(Int i=0; i<N; i++)
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][N];

    xQuantFilterCoef(m_pdDoubleAlfCoeff, rcFilter, g_uiBitDepth + g_uiBitIncrement);
  }
}

Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlags(UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest,TComPicSym* pcQuadTree, UInt64& ruiDist, UInt64& ruiRate)
{
  ruiDist = 0;
  ruiRate = 0;
  if( m_bALFSeparateQt )
  {
    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[0][CI_NEXT_BEST]);
    }
    UInt64* auiFixedBitsCurrBest;
    UInt64* auiFixedBitsNextBest;
    auiFixedBitsCurrBest = new UInt64[MAX_CU_DEPTH];
    auiFixedBitsNextBest = new UInt64[MAX_CU_DEPTH];
    ::memset( auiFixedBitsCurrBest , 0 , sizeof(UInt64) * MAX_CU_DEPTH );
    ::memset( auiFixedBitsNextBest , 0 , sizeof(UInt64) * MAX_CU_DEPTH );
    for( UInt uiCUAddr = 0; uiCUAddr < pcQuadTree->getNumberOfCUsInFrame() ; uiCUAddr++ )
    {
      TComDataCU* pcCU = pcQuadTree->getCU( uiCUAddr );
      if( m_bUseSBACRD )
      {
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_NEXT_BEST]);
      }
      auiFixedBitsCurrBest[0] = auiFixedBitsNextBest[0];
      xSetCUAlfCtrlFlag( pcQuadTree, pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist, auiFixedBitsCurrBest, auiFixedBitsNextBest );
    }
    m_pcCoderFixedBits->resetBits();
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[0][CI_NEXT_BEST] );
    }
    ruiRate = auiFixedBitsNextBest[0] + m_pcEntropyCoder->getNumberOfWrittenBits() ;
    delete[] auiFixedBitsCurrBest;
    delete[] auiFixedBitsNextBest;
  }
  else
  {
    for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
    {
      TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
      xSetCUAlfCtrlFlag(pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
    }
  }
}


Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist)
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
    UInt uiQNumParts = ( m_pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xSetCUAlfCtrlFlag(pcCU, uiAbsPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
    }
    return;
  }

  if( uiDepth > uiAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    return;
  }

  UInt uiCUAddr = pcCU->getAddr();
  UInt64 uiRecSSD = 0;
  UInt64 uiFiltSSD = 0;

  Int iWidth;
  Int iHeight;
  UInt uiSetDepth;

  if (uiDepth > uiAlfCtrlDepth && pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    iWidth = g_uiMaxCUWidth >> uiAlfCtrlDepth;
    iHeight = g_uiMaxCUHeight >> uiAlfCtrlDepth;

    uiRPelX   = uiLPelX + iWidth  - 1;
    uiBPelY   = uiTPelY + iHeight - 1;

    if( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() )
    {
      iWidth = pcCU->getSlice()->getSPS()->getWidth() - uiLPelX;
    }

    if( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() )
    {
      iHeight = pcCU->getSlice()->getSPS()->getHeight() - uiTPelY;
    }

    uiSetDepth = uiAlfCtrlDepth;
  }
  else
  {
    iWidth = pcCU->getWidth(uiAbsPartIdx);
    iHeight = pcCU->getHeight(uiAbsPartIdx);
    uiSetDepth = uiDepth;
  }

  Pel* pOrg = pcPicOrg->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pRec = pcPicDec->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pFilt = pcPicRest->getLumaAddr(uiCUAddr, uiAbsPartIdx);

  uiRecSSD  += xCalcSSD( pOrg, pRec,  iWidth, iHeight, pcPicOrg->getStride() );
  uiFiltSSD += xCalcSSD( pOrg, pFilt, iWidth, iHeight, pcPicOrg->getStride() );

  if (uiFiltSSD < uiRecSSD)
  {
    ruiDist += uiFiltSSD;
    pcCU->setAlfCtrlFlagSubParts(1, uiAbsPartIdx, uiSetDepth);
  }
  else
  {
    ruiDist += uiRecSSD;
    pcCU->setAlfCtrlFlagSubParts(0, uiAbsPartIdx, uiSetDepth);
  }
}

Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag(TComPicSym* pcQuadTree, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiResultDist, UInt64* auiFixedBitsCurrBest, UInt64* auiFixedBitsNextBest )
{
 
  pcCU->setWidth (uiAbsPartIdx, g_uiMaxCUWidth  >> uiDepth );
  pcCU->setHeight(uiAbsPartIdx, g_uiMaxCUHeight >> uiDepth );
  pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );

  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  if( ( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() ) || ( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() ) )
  {
    bBoundary = true;
  }

  m_pcCoderFixedBits->resetBits();
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
  }
  m_pcEntropyCoder->encodeAlfQTSplitFlag( pcCU , uiAbsPartIdx , uiDepth, uiAlfCtrlDepth, false );
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST] );
  }
  UInt uiFixedBitsNoSplit = m_pcCoderFixedBits->getNumberOfWrittenBits() +  UInt( auiFixedBitsCurrBest[ uiDepth ] );


  UInt64 uiSplitDist = 0;
  UInt64 uiSplitRate = 0;
  Double dSplitCost  = 0;
  if( ( uiDepth < uiAlfCtrlDepth ) )
  {
    UInt uiQNumParts = ( pcQuadTree->getNumPartition() >> (uiDepth<<1) )>>2;
    UInt uiNextPartIdx = uiAbsPartIdx;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiNextPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiNextPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiNextPartIdx] ];

      if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
      {
        m_pcCoderFixedBits->resetBits();
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
        }
        pcCU->setDepthSubParts( uiDepth + 1, uiAbsPartIdx );
        m_pcEntropyCoder->encodeAlfQTSplitFlag( pcCU , uiAbsPartIdx , uiDepth, uiAlfCtrlDepth, false );
        if( m_bUseSBACRD )
        {
          m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth + 1][CI_CURR_BEST]);
        }
        auiFixedBitsCurrBest[ uiDepth + 1 ] = auiFixedBitsCurrBest[ uiDepth ] + m_pcCoderFixedBits->getNumberOfWrittenBits();
      }
      else
      {
        if( m_bUseSBACRD )
        {
          m_pppcRDSbacCoder[ uiDepth + 1 ][CI_CURR_BEST]->load(m_pppcRDSbacCoder[ uiDepth + 1 ][CI_NEXT_BEST]);
        }
        auiFixedBitsCurrBest[ uiDepth + 1 ] = auiFixedBitsNextBest[ uiDepth + 1 ];
      }

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
      {
        xSetCUAlfCtrlFlag(pcQuadTree, pcCU, uiNextPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, uiSplitDist, auiFixedBitsCurrBest, auiFixedBitsNextBest  );
      }
    }
  }

  if( uiDepth > uiAlfCtrlDepth )
  {
    return;
  }

  Int iWidth  = g_uiMaxCUWidth  >> uiDepth;
  Int iHeight = g_uiMaxCUHeight >> uiDepth;
  uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  if( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() )
  {
    iWidth = pcCU->getSlice()->getSPS()->getWidth() - uiLPelX;
  }
  if( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() )
  {
    iHeight = pcCU->getSlice()->getSPS()->getHeight() - uiTPelY;
  }

  Pel* pOrg  = pcPicOrg->getLumaAddr ( pcCU->getAddr() , uiAbsPartIdx );
  Pel* pRec  = pcPicDec->getLumaAddr ( pcCU->getAddr() , uiAbsPartIdx );
  Pel* pFilt = pcPicRest->getLumaAddr( pcCU->getAddr() , uiAbsPartIdx );

  UInt64 uiRecSSD   = xCalcSSD( pOrg, pRec,  iWidth, iHeight, pcPicOrg->getStride() );
  UInt64 uiFiltSSD  = xCalcSSD( pOrg, pFilt, iWidth, iHeight, pcPicOrg->getStride() );

  UInt64 uiNoSplitRate = 0;
  UInt64 uiNoSplitDist = 0;

  UInt uiPrevFlag = pcCU->getAlfCtrlFlag( uiAbsPartIdx);

  m_pcCoderFixedBits->resetBits();
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth ][CI_TEMP_BEST] );
  }
  if ( uiFiltSSD < uiRecSSD )
  {
    uiNoSplitDist = uiFiltSSD;
    pcCU->setAlfCtrlFlag( uiAbsPartIdx , 1 );
    m_pcEntropyCoder->encodeAlfCtrlFlag( pcCU , uiAbsPartIdx, false, true );
  }
  else
  {
    uiNoSplitDist = uiRecSSD;
    pcCU->setAlfCtrlFlag( uiAbsPartIdx , 0 );
    m_pcEntropyCoder->encodeAlfCtrlFlag( pcCU , uiAbsPartIdx, false, true );
  }
  uiFixedBitsNoSplit +=  m_pcCoderFixedBits->getNumberOfWrittenBits();
  m_pcCoderFixedBits->resetBits();
  if( m_bUseSBACRD )
  {
    m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
  }
  uiNoSplitRate = uiFixedBitsNoSplit + m_pcEntropyCoder->getNumberOfWrittenBits();
  Double dNoSplitCost  = (Double)( uiNoSplitRate ) * m_dLambdaLuma + (Double)(uiNoSplitDist);

  if( uiDepth < uiAlfCtrlDepth)
  {
    m_pcCoderFixedBits->resetBits();
    if( m_bUseSBACRD )
    {
      m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[ uiDepth + 1 ][CI_NEXT_BEST] );
    }
    uiSplitRate = m_pcEntropyCoder->getNumberOfWrittenBits() + auiFixedBitsNextBest[ uiDepth + 1 ];
    dSplitCost  = (Double)( uiSplitRate ) * m_dLambdaLuma + (Double)( uiSplitDist );

    if( dSplitCost < dNoSplitCost )
    {
      ruiResultDist += uiSplitDist;
      if( m_bUseSBACRD )
      {
        m_pppcRDSbacCoder[ uiDepth ][CI_NEXT_BEST]->load( m_pppcRDSbacCoder[ uiDepth + 1 ][CI_NEXT_BEST] );
      }
      auiFixedBitsNextBest[ uiDepth ] = auiFixedBitsNextBest[ uiDepth + 1 ];
      pcCU->setAlfCtrlFlag( uiAbsPartIdx , uiPrevFlag );
      return;
    }
  }

  ruiResultDist += uiNoSplitDist;
  if( m_bUseSBACRD )
  {
    m_pppcRDSbacCoder[ uiDepth ][CI_NEXT_BEST]->load( m_pppcRDSbacCoder[ uiDepth  ][CI_TEMP_BEST] );
  }
  auiFixedBitsNextBest[ uiDepth ] = uiFixedBitsNoSplit;

  if( bBoundary )
  {
    pcCU->setAlfCtrlFlagSubParts( pcCU->getAlfCtrlFlag( uiAbsPartIdx) , uiAbsPartIdx , uiDepth );
  }
  else
  {
    pcCU->setWidth (uiAbsPartIdx, iWidth  );
    pcCU->setHeight(uiAbsPartIdx, iHeight );
  }
  pcCU->setDepth( uiAbsPartIdx, uiDepth );
}



Void TEncAdaptiveLoopFilter::xEstimateCorrFU(TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiAbsPartIdx,UInt uiDepth, AlfFilter& rcFilter, double** pdCUCorr  )
{

  Int iLPelX    = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Int iTPelY    = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Int iFuWidth  = g_uiMaxCUWidth  >> uiDepth;
  Int iFuHeight = g_uiMaxCUHeight >> uiDepth;

  const Int iOffsetX = iLPelX - Max( 0 , iLPelX - rcFilter.iHorizontalOverlap );
  const Int iOffsetY = iTPelY - Max( 0 , iTPelY - rcFilter.iVerticalOverlap   );
  const Int iMaxX    = pcPicDec->getWidth() -  iLPelX + iOffsetX;
  const Int iMaxY    = pcPicDec->getHeight() - iTPelY + iOffsetY;

  Pel* pOrg = pcPicOrg->getLumaAddr( pcCU->getAddr(), uiAbsPartIdx );
  Pel* pCmp = pcPicDec->getLumaAddr( pcCU->getAddr(), uiAbsPartIdx );

  pCmp -= iOffsetX + pcPicDec->getStride() * iOffsetY;

   xEstimateCorr( pOrg, pCmp, iFuWidth, iFuHeight, pcPicOrg->getStride(), pcPicDec->getStride() , rcFilter , pdCUCorr, iOffsetX, iOffsetY, iMaxX , iMaxY );

}



Void TEncAdaptiveLoopFilter::xEstimateCorrCU(TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiSCUIndx, UChar uhDepth, AlfFilter& rcFilter, double** pdCUCorr  )
{
  Int iLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiSCUIndx ] ];
  Int iTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiSCUIndx ] ];

  const Int iOffsetX = iLPelX - Max( 0 , iLPelX - rcFilter.iHorizontalOverlap );
  const Int iOffsetY = iTPelY - Max( 0 , iTPelY - rcFilter.iVerticalOverlap   );
  const Int iMaxX    = pcPicDec->getWidth() -  iLPelX + iOffsetX;
  const Int iMaxY    = pcPicDec->getHeight() - iTPelY + iOffsetY;

  Pel* pOrg = pcPicOrg->getLumaAddr( pcCU->getAddr(), uiSCUIndx );
  Pel* pCmp = pcPicDec->getLumaAddr( pcCU->getAddr(), uiSCUIndx );

  pCmp -= iOffsetX + pcPicDec->getStride() * iOffsetY;
  xEstimateCorr( pOrg, pCmp, m_uiSCUWidth, m_uiSCUHeight, pcPicOrg->getStride(), pcPicDec->getStride() , rcFilter , pdCUCorr, iOffsetX, iOffsetY, iMaxX , iMaxY );
}


Void TEncAdaptiveLoopFilter::xEstimateCorrCU(TComPicYuv* pcPicOrg , TComPicYuv* pcPicDec, TComDataCU* pcCU, UInt uiSCUIndx, UChar uhDepth, AlfFilter& rcFilter, UInt** puiCUCorr  )
{
  const Int iLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiSCUIndx ] ];
  const Int iTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiSCUIndx ] ];

  const Int iOffsetX = iLPelX - Max( 0 , iLPelX - rcFilter.iHorizontalOverlap );
  const Int iOffsetY = iTPelY - Max( 0 , iTPelY - rcFilter.iVerticalOverlap   );
  const Int iMaxX    = pcPicDec->getWidth() -  iLPelX + iOffsetX;
  const Int iMaxY    = pcPicDec->getHeight() - iTPelY + iOffsetY;

  Pel* pOrg = pcPicOrg->getLumaAddr( pcCU->getAddr(), uiSCUIndx );
  Pel* pCmp = pcPicDec->getLumaAddr( pcCU->getAddr(), uiSCUIndx );

  pCmp -= iOffsetX + pcPicDec->getStride() * iOffsetY;
  xEstimateCorr( pOrg, pCmp, m_uiSCUWidth, m_uiSCUHeight, pcPicOrg->getStride(), pcPicDec->getStride() , rcFilter , puiCUCorr, iOffsetX, iOffsetY, iMaxX , iMaxY );
}


void TEncAdaptiveLoopFilter::xEstimateCorr(  Pel* pOrig, Pel* pDec, Int iWidth, Int iHeight, Int iOrigStride , Int iDecStride, AlfFilter& rcFilter , double** ppdCorr , Int iOffsetX , Int iOffsetY, Int iMaxX , Int iMaxY )
{

  double adPosAccumulation[ ALF_MAX_NUM_COEF ];

  Int iPosIndx = 0;
  Int iPosX = 0;
  Int iPosY = 0;

  for (Int y = iOffsetY ; y < iHeight + iOffsetY ; y++ )
  {
    for (Int x = iOffsetX ; x < iWidth + iOffsetX ; x++ )
    {
      //accumulate values for each CoeffPosition
      memset( adPosAccumulation , 0 , sizeof( double ) * rcFilter.iNumOfCoeffs);

      iPosIndx = 0;
      if( rcFilter.bIsVertical )
      {
        for( Int yy = 0 ; yy < rcFilter.iFilterLength ; yy++ )
        {
            iPosY = std::max( 0 , y - rcFilter.iOverlap + yy );
            iPosY = std::min( iMaxY - 1 , iPosY );

            adPosAccumulation[ rcFilter.aiTapCoeffMapping[ iPosIndx ] ] += pDec[ x + iPosY * iDecStride ];
            iPosIndx++;
        }
      }
      else if( rcFilter.bIsHorizontal )
      {
        for( Int xx = 0 ; xx < rcFilter.iFilterLength ; xx++ )
        {
            iPosX = std::max( 0 , x - rcFilter.iOverlap + xx );
            iPosX = std::min( iMaxX - 1 , iPosX );

            adPosAccumulation[ rcFilter.aiTapCoeffMapping[ iPosIndx ] ] += pDec[ iPosX + y * iDecStride ];
            iPosIndx++;
        }
      }

      for (UInt j = 0; j < rcFilter.iNumOfCoeffs ; j++)
      {
        //autocorrelation of decoded Block
        ppdCorr[j][j] += adPosAccumulation[ j ] * adPosAccumulation[ j ];
        for (UInt i = j + 1 ; i < rcFilter.iNumOfCoeffs ; i++ )
        {
          ppdCorr[i][j] += adPosAccumulation[j] * adPosAccumulation[i];
        }
        //DC offset
#if ALF_DC_CONSIDERED
        ppdCorr[rcFilter.iNumOfCoeffs][j] += adPosAccumulation[j];

        //cross-correlation between Pixels of decoded Block at CoeffPositions and Pixel (x,y) from original Block
        ppdCorr[j][rcFilter.iNumOfCoeffs + 1] += pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride] * adPosAccumulation[j];
#else
        //cross-correlation between Pixels of decoded Block at CoeffPositions and Pixel (x,y) from original Block
        ppdCorr[j][rcFilter.iNumOfCoeffs ] += pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride] * adPosAccumulation[j];
#endif
      }
#if ALF_DC_CONSIDERED
      ppdCorr[rcFilter.iNumOfCoeffs][rcFilter.iNumOfCoeffs]     += 1;
      ppdCorr[rcFilter.iNumOfCoeffs][rcFilter.iNumOfCoeffs + 1] += pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride ];
#endif
    }
  }


}

void TEncAdaptiveLoopFilter::xEstimateCorr(  Pel* pOrig, Pel* pDec, Int iWidth, Int iHeight, Int iOrigStride , Int iDecStride, AlfFilter& rcFilter , UInt** ppuiCorr , Int iOffsetX , Int iOffsetY, Int iMaxX , Int iMaxY )
{

  double adPosAccumulation[ ALF_MAX_NUM_COEF ];


   Int iPosIndx = 0;
   Int iPosX = 0;
   Int iPosY = 0;


   for (Int y = iOffsetY ; y < iHeight + iOffsetY ; y++ )
   {
     for (Int x = iOffsetX ; x < iWidth + iOffsetX ; x++ )
     {
       //accumulate values for each CoeffPosition
       memset( adPosAccumulation , 0 , sizeof( double ) * rcFilter.iNumOfCoeffs);

       iPosIndx = 0;
       if( rcFilter.bIsVertical )
       {
         for( Int yy = 0 ; yy < rcFilter.iFilterLength ; yy++ )
         {
             iPosY = std::max( 0 , y - rcFilter.iOverlap + yy );
             iPosY = std::min( iMaxY - 1 , iPosY );

             adPosAccumulation[ rcFilter.aiTapCoeffMapping[ iPosIndx ] ] += pDec[ x + iPosY * iDecStride ];
             iPosIndx++;
         }

       }
       else if( rcFilter.bIsHorizontal )
       {
         for( Int xx = 0 ; xx < rcFilter.iFilterLength ; xx++ )
         {
             iPosX = std::max( 0 , x - rcFilter.iOverlap + xx );
             iPosX = std::min( iMaxX - 1 , iPosX );

             adPosAccumulation[ rcFilter.aiTapCoeffMapping[ iPosIndx ] ] += pDec[ iPosX + y * iDecStride ];
             iPosIndx++;
         }
       }

       for (UInt j = 0; j < rcFilter.iNumOfCoeffs ; j++)
       {
         //autocorrelation of decoded Block
         ppuiCorr[j][j] += (UInt)(adPosAccumulation[ j ] * adPosAccumulation[ j ]);
         for (UInt i = j + 1 ; i < rcFilter.iNumOfCoeffs ; i++ )
         {
           ppuiCorr[i][j] += (UInt)(adPosAccumulation[j] * adPosAccumulation[i]);
         }
         //DC offset
#if ALF_DC_CONSIDERED
         ppuiCorr[rcFilter.iNumOfCoeffs][j] += (UInt)adPosAccumulation[j];

         //cross-correlation between Pixels of decoded Block at CoeffPositions and Pixel (x,y) from original Block
         ppuiCorr[j][rcFilter.iNumOfCoeffs + 1] += (UInt)(pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride] * adPosAccumulation[j]);
#else
         //cross-correlation between Pixels of decoded Block at CoeffPositions and Pixel (x,y) from original Block
         ppuiCorr[j][rcFilter.iNumOfCoeffs    ] += (UInt)(pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride] * adPosAccumulation[j]);
#endif
       }
#if ALF_DC_CONSIDERED
       ppuiCorr[rcFilter.iNumOfCoeffs][rcFilter.iNumOfCoeffs]     += 1;
       ppuiCorr[rcFilter.iNumOfCoeffs][rcFilter.iNumOfCoeffs + 1] += pOrig[  ( x - iOffsetX ) + ( y - iOffsetY )  * iOrigStride ];
#endif
     }
   }
}
#else
// ====================================================================================================================
// Tables
// ====================================================================================================================

const Int TEncAdaptiveLoopFilter::m_aiSymmetricArray9x9[81] =
{
   0,  1,  2,  3,  4,  5,  6,  7,  8,
   9, 10, 11, 12, 13, 14, 15, 16, 17,
  18, 19, 20, 21, 22, 23, 24, 25, 26,
  27, 28, 29, 30, 31, 32, 33, 34, 35,
  36, 37, 38, 39, 40, 39, 38, 37, 36,
  35, 34, 33, 32, 31, 30, 29, 28, 27,
  26, 25, 24, 23, 22, 21, 20, 19, 18,
  17, 16, 15, 14, 13, 12, 11, 10,  9,
   8,  7,  6,  5,  4,  3,  2,  1,  0
};
const Int TEncAdaptiveLoopFilter::m_aiSymmetricArray7x7[49] =
{
   0,  1,  2,  3,  4,  5,  6,
   7,  8,  9, 10, 11, 12, 13,
  14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 23, 22, 21,
  20, 19, 18, 17, 16, 15, 14,
  13, 12, 11, 10,  9,  8,  7,
   6,  5,  4,  3,  2,  1,  0,
};
const Int TEncAdaptiveLoopFilter::m_aiSymmetricArray5x5[25] =
{
   0,  1,  2,  3,  4,
   5,  6,  7,  8,  9,
  10, 11, 12, 11, 10,
   9,  8,  7,  6,  5,
   4,  3,  2,  1,  0,
};

// ====================================================================================================================
// Constructor / destructor
// ====================================================================================================================

TEncAdaptiveLoopFilter::TEncAdaptiveLoopFilter()
{
  m_ppdAlfCorr = NULL;
  m_pdDoubleAlfCoeff = NULL;
  m_puiCUCorr = NULL;
  m_pcPic = NULL;
  m_pcEntropyCoder = NULL;
  m_pcBestAlfParam = NULL;
  m_pcTempAlfParam = NULL;
  m_pcPicYuvBest = NULL;
  m_pcPicYuvTmp = NULL;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param	pcPic						picture (TComPic) pointer
		\param	pcEntropyCoder	entropy coder class
 */
Void TEncAdaptiveLoopFilter::startALFEnc( TComPic* pcPic, TEncEntropy* pcEntropyCoder )
{
  m_pcPic = pcPic;
  m_pcEntropyCoder = pcEntropyCoder;

  m_eSliceType = pcPic->getSlice()->getSliceType();
  m_iPicNalReferenceIdc = (pcPic->getSlice()->isReferenced() ? 1 :0);

  m_uiNumSCUInCU = m_pcPic->getNumPartInCU();
  m_uiSCUWidth = (m_pcPic->getMinCUWidth()<<1);
  m_uiSCUHeight = (m_pcPic->getMinCUHeight()<<1);

  xInitParam();
  xCreateTmpAlfCtrlFlags();

  Int iWidth = pcPic->getPicYuvOrg()->getWidth();
  Int iHeight = pcPic->getPicYuvOrg()->getHeight();

  m_pcPicYuvTmp = new TComPicYuv();
  m_pcPicYuvTmp->createLuma(iWidth, iHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
  m_pcPicYuvBest = pcPic->getPicYuvPred();

  m_pcBestAlfParam = new ALFParam;
  m_pcTempAlfParam = new ALFParam;
  allocALFParam(m_pcBestAlfParam);
  allocALFParam(m_pcTempAlfParam);
#if QC_ALF
  im_width = iWidth;
  im_height = iHeight;

  //original and reconst 
#if !ALF_MEM_PATCH
  get_mem2Dpel (&(imgY_rec), im_height, im_width);
  get_mem2Dpel (&(imgY_org), im_height, im_width);
  get_mem2Dpel (&(imgY_rest), im_height, im_width);
  get_mem2Dpel (&(imgY_ext), im_height+ALF_MAX_NUM_TAP, im_width+ALF_MAX_NUM_TAP);
  get_mem2Dpel (&(imgY_temp), im_height, im_width);
#endif
#if WIENER_3_INPUT
#if !ALF_MEM_PATCH
  get_mem2Dpel (&(imgY_pred), im_height, im_width);
  get_mem2Dpel (&(imgY_resi), im_height, im_width);
  get_mem2Dpel (&(imgY_pext), im_height+ALF_MAX_NUM_TAP, im_width+ALF_MAX_NUM_TAP);
  get_mem2Dpel (&(imgY_rext), im_height+ALF_MAX_NUM_TAP, im_width+ALF_MAX_NUM_TAP);
#endif  
  initMatrix4D_double(&EGlobalSym, (NO_TEST_FILT+2)*(NO_TEST_FILT+2)*(NO_TEST_FILT+2), NO_VAR_BINS, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
  initMatrix3D_double(&yGlobalSym, (NO_TEST_FILT+2)*(NO_TEST_FILT+2)*(NO_TEST_FILT+2), NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
#else
  // init qc_filter
  initMatrix4D_double(&EGlobalSym, NO_TEST_FILT,  NO_VAR_BINS, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
  initMatrix3D_double(&yGlobalSym, NO_TEST_FILT, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
#endif
#if !ALF_MEM_PATCH
  initMatrix_int(&g_filterCoeffSym, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
  initMatrix_int(&g_filterCoeffPrevSelected, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
#endif
  initMatrix_int(&g_filterCoeffSymQuant, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 

  pixAcc = (double *) calloc(NO_VAR_BINS, sizeof(double));
  get_mem2Dpel(&varImg, im_height, im_width);
  get_mem2Dpel(&maskImg, im_height, im_width);
  

#if ALF_MEM_PATCH
  initMatrix_double(&E_temp, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);//
  y_temp = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));//
  initMatrix3D_double(&E_merged, NO_VAR_BINS, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);//
  initMatrix_double(&y_merged, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); //
  pixAcc_merged = (double *) calloc(NO_VAR_BINS, sizeof(double));//

  filterCoeffQuantMod = (int *) calloc(MAX_SQR_FILT_LENGTH, sizeof(int));//
  filterCoeff = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));//
  filterCoeffQuant = (int *) calloc(MAX_SQR_FILT_LENGTH, sizeof(int));//
  initMatrix_int(&diffFilterCoeffQuant, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);//
  initMatrix_int(&FilterCoeffQuantTemp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);//
#if WIENER_3_INPUT
  initMatrix_int(&FilterCoeffQuant_predicted, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);//
#endif

#endif

  ALFp = new ALFParam;
  tempALFp = new ALFParam;
  allocALFParam(ALFp);
  allocALFParam(tempALFp);
  m_pcDummyEntropyCoder = m_pcEntropyCoder;
#endif
}

Void TEncAdaptiveLoopFilter::endALFEnc()
{
  xUninitParam();
  xDestroyTmpAlfCtrlFlags();

  m_pcPicYuvTmp->destroyLuma();
  delete m_pcPicYuvTmp;
  m_pcPicYuvTmp = NULL;
  m_pcPic = NULL;
  m_pcEntropyCoder = NULL;

  freeALFParam(m_pcBestAlfParam);
  freeALFParam(m_pcTempAlfParam);
  delete m_pcBestAlfParam;
  delete m_pcTempAlfParam;
#if QC_ALF
#if !ALF_MEM_PATCH
  free_mem2Dpel (imgY_rec);
  free_mem2Dpel (imgY_org);

  free_mem2Dpel (imgY_rest);
  free_mem2Dpel (imgY_ext);
  free_mem2Dpel (imgY_temp);
#endif
#if WIENER_3_INPUT
#if !ALF_MEM_PATCH
  free_mem2Dpel (imgY_pred);
  free_mem2Dpel (imgY_resi);
  free_mem2Dpel (imgY_pext);
  free_mem2Dpel (imgY_rext);
#endif
  destroyMatrix4D_double(EGlobalSym, (NO_TEST_FILT+2)*(NO_TEST_FILT+2)*(NO_TEST_FILT+2),  NO_VAR_BINS);
  destroyMatrix3D_double(yGlobalSym, (NO_TEST_FILT+2)*(NO_TEST_FILT+2)*(NO_TEST_FILT+2));  
#else  
// delete qc filters
  destroyMatrix4D_double(EGlobalSym, NO_TEST_FILT,  NO_VAR_BINS);
  destroyMatrix3D_double(yGlobalSym, NO_TEST_FILT);
#endif
  destroyMatrix_int(g_filterCoeffSymQuant);
#if !ALF_MEM_PATCH
  destroyMatrix_int(g_filterCoeffSym);
  destroyMatrix_int(g_filterCoeffPrevSelected);
#endif

  free(pixAcc);
  free_mem2Dpel(varImg);
  free_mem2Dpel(maskImg);

#if ALF_MEM_PATCH
    destroyMatrix3D_double(E_merged, NO_VAR_BINS);
	destroyMatrix_double(y_merged);
	destroyMatrix_double(E_temp);
	free(pixAcc_merged);

	free(filterCoeffQuantMod);
	free(y_temp);

	free(filterCoeff);
	free(filterCoeffQuant);
	destroyMatrix_int(diffFilterCoeffQuant);
	destroyMatrix_int(FilterCoeffQuantTemp);
#if WIENER_3_INPUT
  destroyMatrix_int(FilterCoeffQuant_predicted);
#endif
#endif
  freeALFParam(ALFp);
  freeALFParam(tempALFp);
  delete ALFp;
  delete tempALFp;
#endif
}

/** \param	pcAlfParam					ALF parameter
		\param	dLambda							lambda value for RD cost computation
		\retval	ruiDist							distortion
		\retval	ruiBits							required bits
		\retval	ruiMaxAlfCtrlDepth	optimal partition depth
 */
Void TEncAdaptiveLoopFilter::ALFProcess( ALFParam* pcAlfParam, Double dLambda, UInt64& ruiDist, UInt64& ruiBits, UInt& ruiMaxAlfCtrlDepth )
{
  Int tap, num_coef;

  // set global variables
  tap         = ALF_MAX_NUM_TAP;
  num_coef    = (tap*tap+1)>>1;
  num_coef    = num_coef + 1; // DC offset

  // set lambda
  m_dLambdaLuma   = dLambda;
  m_dLambdaChroma = dLambda;

  TComPicYuv* pcPicOrg = m_pcPic->getPicYuvOrg();

  // extend image for filtering
  TComPicYuv* pcPicYuvRec    = m_pcPic->getPicYuvRec();
  TComPicYuv* pcPicYuvExtRec = m_pcTempPicYuv;
#if WIENER_3_INPUT
  TComPicYuv* pcPicYuvPred   = m_pcPic->getPicYuvP();
  TComPicYuv* pcPicYuvResi   = m_pcPic->getPicYuvQ();  
#if ALF_MEM_PATCH
  TComPicYuv* pcPicYuvExtPred = m_pcTempPicPredYuv;
  TComPicYuv* pcPicYuvExtResi = m_pcTempPicResiYuv;
#endif  
#endif
  
  pcPicYuvRec->copyToPic(pcPicYuvExtRec);
  pcPicYuvExtRec->setBorderExtension( false );
  pcPicYuvExtRec->extendPicBorder   ();

#if (WIENER_3_INPUT && ALF_MEM_PATCH)
  pcPicYuvPred->copyToPic(pcPicYuvExtPred);
  pcPicYuvResi->copyToPic(pcPicYuvExtResi);
  pcPicYuvExtPred->setBorderExtension( false );
  pcPicYuvExtResi->setBorderExtension( false );
  pcPicYuvExtPred->extendPicBorder   ();
  pcPicYuvExtResi->extendPicBorder   ();
#endif  
  
  // set min cost
  UInt64 uiMinRate = MAX_INT;
  UInt64 uiMinDist = MAX_INT;
  Double dMinCost  = MAX_DOUBLE;

  UInt64  uiOrigRate;
  UInt64  uiOrigDist;
  Double	dOrigCost;

  // calc original cost
  xCalcRDCost( pcPicOrg, pcPicYuvRec, NULL, uiOrigRate, uiOrigDist, dOrigCost );
  m_pcBestAlfParam->alf_flag = 0;
  m_pcBestAlfParam->cu_control_flag = 0;

  // initialize temp_alfps
  m_pcTempAlfParam->alf_flag        = 1;
  m_pcTempAlfParam->tap							= tap;
  m_pcTempAlfParam->num_coeff				= num_coef;
  m_pcTempAlfParam->chroma_idc      = 0;
  m_pcTempAlfParam->cu_control_flag = 0;

#if QC_ALF
#if WIENER_3_INPUT
  adapt_precision=1;
 // adaptive in-loop wiener filtering
#if ALF_MEM_PATCH  
  xEncALFLuma_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvExtPred, pcPicYuvExtResi, uiMinRate, uiMinDist, dMinCost );
  // cu-based filter on/off control
  xCUAdaptiveControl_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvExtPred, pcPicYuvExtResi, uiMinRate, uiMinDist, dMinCost );

  // adaptive tap-length
  xFilterTapDecision_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvExtPred, pcPicYuvExtResi, uiMinRate, uiMinDist, dMinCost );
#else
  xEncALFLuma_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvPred, pcPicYuvResi, uiMinRate, uiMinDist, dMinCost );
  // cu-based filter on/off control
  xCUAdaptiveControl_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvPred, pcPicYuvResi, uiMinRate, uiMinDist, dMinCost );

  // adaptive tap-length
  xFilterTapDecision_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, pcPicYuvPred, pcPicYuvResi, uiMinRate, uiMinDist, dMinCost );
#endif
#else
 // adaptive in-loop wiener filtering
  xEncALFLuma_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );

  // cu-based filter on/off control
  xCUAdaptiveControl_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );

  // adaptive tap-length
  xFilterTapDecision_qc( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );
#endif
#else
  // adaptive in-loop wiener filtering
  xEncALFLuma( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );

  // cu-based filter on/off control
  xCUAdaptiveControl( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );

  // adaptive tap-length
  xFilterTapDecision( pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, uiMinRate, uiMinDist, dMinCost );
#endif  

	// compute RD cost
  xCalcRDCost( pcPicOrg, pcPicYuvRec, m_pcBestAlfParam, uiMinRate, uiMinDist, dMinCost );

	// compare RD cost to non-ALF case
  if( dMinCost < dOrigCost )
  {
    m_pcBestAlfParam->alf_flag = 1;

    ruiBits = uiMinRate;
    ruiDist = uiMinDist;
  }
  else
  {
    m_pcBestAlfParam->alf_flag				= 0;
    m_pcBestAlfParam->cu_control_flag = 0;

    uiMinRate = uiOrigRate;
    uiMinDist = uiOrigDist;
    dMinCost = dMinCost;

    m_pcEntropyCoder->setAlfCtrl(false);
    pcPicYuvExtRec->copyToPicLuma(pcPicYuvRec);

    ruiBits = uiOrigRate;
    ruiDist = uiOrigDist;
  }

	// if ALF works
  if( m_pcBestAlfParam->alf_flag )
  {
		// predict ALF coefficients
    predictALFCoeff( m_pcBestAlfParam );

    // do additional ALF process for chroma
    xEncALFChroma( uiMinRate, pcPicOrg, pcPicYuvExtRec, pcPicYuvRec, ruiDist, ruiBits );
  }

	// copy to best storage
  copyALFParam(pcAlfParam, m_pcBestAlfParam);

	// store best depth
  ruiMaxAlfCtrlDepth = m_pcEntropyCoder->getMaxAlfCtrlDepth();
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void TEncAdaptiveLoopFilter::xEncALFLuma(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
{
  UInt64  uiRate;
  UInt64  uiDist;
  Double dCost;

  Int tap, num_coef;

  tap                 = ALF_MIN_NUM_TAP;
  m_pcTempAlfParam->tap = tap;
  num_coef            = (tap*tap+1)>>1;
  num_coef            = num_coef + 1; // DC offset
  m_pcTempAlfParam->num_coeff = num_coef;

  xFilteringFrameLuma(pcPicOrg, pcPicDec, pcPicRest, true);
  xCalcRDCost(pcPicOrg, pcPicRest, m_pcTempAlfParam, uiRate, uiDist, dCost);

  if( dCost < rdMinCost)
  {
    ruiMinRate = uiRate;
    ruiMinDist = uiDist;
    rdMinCost = dCost;
    copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
  }
}

Void TEncAdaptiveLoopFilter::xEncALFChroma( UInt64 uiLumaRate, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, UInt64& ruiBits )
{
  // restriction for non-referenced B-slice
  if (m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0)
  {
    return;
  }

  Int tap, num_coef;

  // set global variables
  tap         = ALF_MAX_NUM_TAP_C;
  num_coef    = (tap*tap+1)>>1;
  num_coef    = num_coef + 1; // DC offset

  // set min cost
  UInt64 uiMinRate = uiLumaRate;
  UInt64 uiMinDist = MAX_INT;
  Double dMinCost  = MAX_DOUBLE;

  // calc original cost
  copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
  xCalcRDCostChroma(pcPicOrg, pcPicRest, m_pcTempAlfParam, uiMinRate, uiMinDist, dMinCost);

  // initialize temp_alfps
  m_pcTempAlfParam->chroma_idc = 3;
  m_pcTempAlfParam->tap_chroma       = tap;
  m_pcTempAlfParam->num_coeff_chroma = num_coef;

  // Adaptive in-loop wiener filtering for chroma
  xFilteringFrameChroma(pcPicOrg, pcPicDec, pcPicRest);

  // filter on/off decision for chroma
  Int iCWidth = (pcPicOrg->getWidth()>>1);
  Int iCHeight = (pcPicOrg->getHeight()>>1);
  Int iCStride = pcPicOrg->getCStride();
  UInt64 uiFiltDistCb = xCalcSSD(pcPicOrg->getCbAddr(), pcPicRest->getCbAddr(), iCWidth, iCHeight, iCStride);
  UInt64 uiFiltDistCr = xCalcSSD(pcPicOrg->getCrAddr(), pcPicRest->getCrAddr(), iCWidth, iCHeight, iCStride);
  UInt64 uiOrgDistCb = xCalcSSD(pcPicOrg->getCbAddr(), pcPicDec->getCbAddr(), iCWidth, iCHeight, iCStride);
  UInt64 uiOrgDistCr = xCalcSSD(pcPicOrg->getCrAddr(), pcPicDec->getCrAddr(), iCWidth, iCHeight, iCStride);

  m_pcTempAlfParam->chroma_idc = 0;
  if(uiOrgDistCb > uiFiltDistCb)
    m_pcTempAlfParam->chroma_idc += 2;
  if(uiOrgDistCr  > uiFiltDistCr )
    m_pcTempAlfParam->chroma_idc += 1;

  if(m_pcTempAlfParam->chroma_idc)
  {
    if(m_pcTempAlfParam->chroma_idc!=3)
    {
      // chroma filter re-design
      xFilteringFrameChroma(pcPicOrg, pcPicDec, pcPicRest);
    }

    UInt64 uiRate, uiDist;
    Double dCost;
    xCalcRDCostChroma(pcPicOrg, pcPicRest, m_pcTempAlfParam, uiRate, uiDist, dCost);

    if( dCost < dMinCost )
    {
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      predictALFCoeffChroma(m_pcBestAlfParam);

      ruiBits += uiRate;
      ruiDist += uiDist;
    }
    else
    {
      m_pcBestAlfParam->chroma_idc = 0;

      if((m_pcTempAlfParam->chroma_idc>>1)&0x01)
        pcPicDec->copyToPicCb(pcPicRest);
      if(m_pcTempAlfParam->chroma_idc&0x01)
        pcPicDec->copyToPicCr(pcPicRest);

      ruiBits += uiMinRate;
      ruiDist += uiMinDist;
    }
  }
  else
  {
    m_pcBestAlfParam->chroma_idc = 0;

    ruiBits += uiMinRate;
    ruiDist += uiMinDist;

		pcPicDec->copyToPicCb(pcPicRest);
    pcPicDec->copyToPicCr(pcPicRest);
  }
}

Void TEncAdaptiveLoopFilter::xCUAdaptiveControl(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
{
  m_pcEntropyCoder->setAlfCtrl(true);

  UInt uiBestDepth = 0;

  ALFParam cFrmAlfParam;
  allocALFParam(&cFrmAlfParam);
  copyALFParam(&cFrmAlfParam, m_pcBestAlfParam);

  for (UInt uiDepth = 0; uiDepth < g_uiMaxCUDepth; uiDepth++)
  {
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiDepth);
    pcPicRest->copyToPicLuma(m_pcPicYuvTmp);
    copyALFParam(m_pcTempAlfParam, &cFrmAlfParam);
    m_pcTempAlfParam->cu_control_flag = 1;

    for (UInt uiRD = 0; uiRD <= ALF_NUM_OF_REDESIGN; uiRD++)
    {
      if (uiRD)
      {
        // re-design filter coefficients
        xReDesignFilterCoeff(pcPicOrg, pcPicDec, true);
        xFrame(pcPicDec, m_pcPicYuvTmp, m_pcTempAlfParam->coeff, m_pcTempAlfParam->tap);
      }

      UInt64 uiRate, uiDist;
      Double dCost;
#if TSB_ALF_HEADER
      xSetCUAlfCtrlFlags(uiDepth, pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam);
#else
      xSetCUAlfCtrlFlags(uiDepth, pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist);
#endif

			// compute RD cost
      xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);

      if (dCost < rdMinCost)
      {
        uiBestDepth = uiDepth;
        rdMinCost = dCost;
        ruiMinDist = uiDist;
        ruiMinRate = uiRate;
        m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);
        copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
        xCopyTmpAlfCtrlFlagsFrom();
      }
    }
  }

  if (m_pcBestAlfParam->cu_control_flag)
  {
    m_pcEntropyCoder->setAlfCtrl(true);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiBestDepth);
    xCopyTmpAlfCtrlFlagsTo();
    m_pcPicYuvBest->copyToPicLuma(pcPicRest);
    xCopyDecToRestCUs(pcPicDec, pcPicRest);
  }
  else
  {
    m_pcEntropyCoder->setAlfCtrl(false);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(0);
  }
  freeALFParam(&cFrmAlfParam);
}

Void TEncAdaptiveLoopFilter::xFilterTapDecision(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
{
  // restriction for non-referenced B-slice
  if (m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0)
  {
    return;
  }

  UInt64 uiRate, uiDist;
  Double dCost;

  if (m_pcBestAlfParam->cu_control_flag)
  {
    xCopyTmpAlfCtrlFlagsFrom();
  }

  Bool bChanged = false;
  for (Int iTap = ALF_MIN_NUM_TAP; iTap <= ALF_MAX_NUM_TAP; iTap += 2)
  {
    copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
    m_pcTempAlfParam->tap = iTap;
    m_pcTempAlfParam->num_coeff = ((iTap*iTap+1)>>1) + 1;
    if (m_pcTempAlfParam->cu_control_flag)
    {
      xReDesignFilterCoeff(pcPicOrg, pcPicDec, false);
      xFrame(pcPicDec, m_pcPicYuvTmp, m_pcTempAlfParam->coeff, m_pcTempAlfParam->tap);
#if TSB_ALF_HEADER
      xSetCUAlfCtrlFlags(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam);
#else
      xSetCUAlfCtrlFlags(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist);
#endif

			// compute RD cost
      xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);
    }
    else
    {
      xFilteringFrameLuma(pcPicOrg, pcPicDec, m_pcPicYuvTmp, false);
      xCalcRDCost(pcPicOrg, m_pcPicYuvTmp, m_pcTempAlfParam, uiRate, uiDist, dCost);
    }

    if (dCost < rdMinCost)
    {
      rdMinCost = dCost;
      ruiMinDist = uiDist;
      ruiMinRate = uiRate;
      m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      bChanged = true;
      if (m_pcTempAlfParam->cu_control_flag)
      {
        xCopyTmpAlfCtrlFlagsFrom();
      }
    }
  }

  if (m_pcBestAlfParam->cu_control_flag)
  {
    xCopyTmpAlfCtrlFlagsTo();
    if (bChanged)
    {
      m_pcPicYuvBest->copyToPicLuma(pcPicRest);
      xCopyDecToRestCUs(pcPicDec, pcPicRest);
    }
  }
  else if (m_pcBestAlfParam->tap > ALF_MIN_NUM_TAP)
  {
    m_pcPicYuvBest->copyToPicLuma(pcPicRest);
  }

  copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

Void TEncAdaptiveLoopFilter::xInitParam()
{
  Int i, j, k, l;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      for (j = 0; j < ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }
  else
  {
    m_ppdAlfCorr = new Double*[ALF_MAX_NUM_COEF];
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_ppdAlfCorr[i] = new Double[ALF_MAX_NUM_COEF+1];
      for (j = 0; j < ALF_MAX_NUM_COEF+1; j++)
      {
        m_ppdAlfCorr[i][j] = 0;
      }
    }
  }

  if (m_puiCUCorr != NULL)
  {
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        for (k = 0; k < ALF_MIN_NUM_COEF; k++)
        {
          for (l = 0; l< ALF_MIN_NUM_COEF+1; l++)
          {
            m_puiCUCorr[i][j][k][l] = 0;
          }
        }
      }
    }
  }
  else
  {
    m_puiCUCorr = new CorrBlk*[m_pcPic->getNumCUsInFrame()];
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      m_puiCUCorr[i] = new CorrBlk[m_uiNumSCUInCU];

      for (j = 0; j < m_uiNumSCUInCU; j++)
      {
        for (k = 0; k < ALF_MIN_NUM_COEF; k++)
        {
          for (l = 0; l< ALF_MIN_NUM_COEF+1; l++)
          {
            m_puiCUCorr[i][j][k][l] = 0;
          }
        }
      }        
    }
  }

  if (m_pdDoubleAlfCoeff != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
  else
  {
    m_pdDoubleAlfCoeff = new Double[ALF_MAX_NUM_COEF];
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      m_pdDoubleAlfCoeff[i] = 0;
    }
  }
}

Void TEncAdaptiveLoopFilter::xUninitParam()
{
  Int i;

  if (m_ppdAlfCorr != NULL)
  {
    for (i = 0; i < ALF_MAX_NUM_COEF; i++)
    {
      delete[] m_ppdAlfCorr[i];
      m_ppdAlfCorr[i] = NULL;
    }
    delete[] m_ppdAlfCorr;
    m_ppdAlfCorr = NULL;
  }

  if (m_puiCUCorr != NULL)
  {
    for (i = 0; i < m_pcPic->getNumCUsInFrame(); i++)
    {
      delete[] m_puiCUCorr[i];
      m_puiCUCorr[i] = NULL;
    }
    delete[] m_puiCUCorr;
    m_puiCUCorr = NULL;
  }

  if (m_pdDoubleAlfCoeff != NULL)
  {
    delete[] m_pdDoubleAlfCoeff;
    m_pdDoubleAlfCoeff = NULL;
  }
}

Void TEncAdaptiveLoopFilter::xCreateTmpAlfCtrlFlags()
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    pcCU->createTmpAlfCtrlFlag();
  }
}

Void TEncAdaptiveLoopFilter::xDestroyTmpAlfCtrlFlags()
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    pcCU->destroyTmpAlfCtrlFlag();
  }
}

Void TEncAdaptiveLoopFilter::xCopyTmpAlfCtrlFlagsTo()
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    pcCU->copyAlfCtrlFlagFromTmp();
  }
}

Void TEncAdaptiveLoopFilter::xCopyTmpAlfCtrlFlagsFrom()
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    pcCU->copyAlfCtrlFlagToTmp();
  }
}

Void TEncAdaptiveLoopFilter::xReadOrCalcCorrFromCUs(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr)
{
  Int N = m_pcTempAlfParam->num_coeff;
  Int tap = m_pcTempAlfParam->tap;

  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    for (UInt uiIdx = 0; uiIdx < pcCU->getTotalNumPart(); uiIdx+=4)
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if (uiLPelX >= pcPicOrg->getWidth() || uiTPelY >= pcPicOrg->getHeight())
      {
        continue;
      }

      if (pcCU->getAlfCtrlFlag(uiIdx))
      {
        if (bReadCorr)
        {
          for(Int j=0; j<N; j++)
          {
            for(Int k=j; k<N+1; k++)
            {
              m_ppdAlfCorr[j][k] += (Double) m_puiCUCorr[uiCUAddr][(uiIdx>>2)][j][k-j];
            }
          }
        }
        else
        {
          Pel* pOrg = pcPicOrg->getLumaAddr(uiCUAddr, uiIdx);
          Pel* pCmp = pcPicDec->getLumaAddr(uiCUAddr, uiIdx);

          xCalcCorrelationFuncBlock(pOrg, pCmp, tap, m_uiSCUWidth, m_uiSCUHeight, pcPicOrg->getStride(), pcPicDec->getStride());
        }
      }
    }
  }
}

Void TEncAdaptiveLoopFilter::xEncodeCUAlfCtrlFlags()
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    xEncodeCUAlfCtrlFlag(pcCU, 0, 0);
  }
}

Void TEncAdaptiveLoopFilter::xEncodeCUAlfCtrlFlag(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth)
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
    UInt uiQNumParts = ( m_pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xEncodeCUAlfCtrlFlag(pcCU, uiAbsPartIdx, uiDepth+1);
    }
    return;
  }

  m_pcEntropyCoder->encodeAlfCtrlFlag(pcCU, uiAbsPartIdx);
}

Void TEncAdaptiveLoopFilter::xCalcALFCoeff( ALFParam* pAlfParam )
{
  Int iErrCode;

  Int    *qh;

  Int tap			= pAlfParam->tap;
  Int N				= pAlfParam->num_coeff;
  Double* h   = m_pdDoubleAlfCoeff;
  qh					= pAlfParam->coeff;

  iErrCode = xGauss(m_ppdAlfCorr, N);

  if(iErrCode)
  {
    xClearFilterCoefInt(pAlfParam->coeff, N);
  }
  else
  {
    for(Int i=0; i<N; i++)
      h[i] = m_ppdAlfCorr[i][N];
    xQuantFilterCoef(h, pAlfParam->coeff, tap, g_uiBitDepth + g_uiBitIncrement);
  }
}

Void TEncAdaptiveLoopFilter::xCalcStoredCorrelationFuncBlock(Pel* pOrg, Pel* pCmp, CorrBlk& ppuiCorr, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride)
{
  Int N      = (iTap*iTap+1)>>1;
  Int offset = iTap>>1;

  const Int* pFiltPos;

  switch(iTap)
  {
  case 5:
    pFiltPos = m_aiSymmetricArray5x5;
    break;
  case 7:
    pFiltPos = m_aiSymmetricArray7x7;
    break;
  case 9:
    pFiltPos = m_aiSymmetricArray9x9;
    break;
  default:
		pFiltPos = m_aiSymmetricArray9x9;
    assert(0);
    break;
  }

  UInt* pTerm = new UInt[N];

  Int i, j;

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      i = 0;
      ::memset(pTerm, 0, sizeof(UInt)*N);
      for(Int yy=y-offset; yy<=y+offset; yy++)
      {
        for(Int xx=x-offset; xx<=x+offset; xx++)
        {
          pTerm[pFiltPos[i]] += (UInt) pCmp[xx + yy*iCmpStride];
          i++;
        }
      }

      for(j=0; j<N; j++)
      {
        ppuiCorr[j][0] += pTerm[j]*pTerm[j];
        for(i=j+1; i<N; i++)
          ppuiCorr[j][i-j] += pTerm[j]*pTerm[i];

        // DC offset
        ppuiCorr[j][N-j]   += pTerm[j];
        ppuiCorr[j][N-j+1] += (UInt) pOrg[x+y*iOrgStride]*pTerm[j];
      }
      // DC offset
      ppuiCorr[N][0] += 1;
      ppuiCorr[N][1] += (UInt) pOrg[x+y*iOrgStride];
    }
  }

  delete[] pTerm;
  pTerm = NULL;
}

Void TEncAdaptiveLoopFilter::xCalcCorrelationFunc(Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride)
{
  //Patch should be extended before this point................
  //ext_offset  = tap>>1;

  Int N      = (iTap*iTap+1)>>1;
  Int offset = iTap>>1;

  const Int* pFiltPos;

  switch(iTap)
  {
  case 5:
    pFiltPos = m_aiSymmetricArray5x5;
    break;
  case 7:
    pFiltPos = m_aiSymmetricArray7x7;
    break;
  case 9:
    pFiltPos = m_aiSymmetricArray9x9;
    break;
  default:
		pFiltPos = m_aiSymmetricArray9x9;
    assert(0);
    break;
  }

  Pel* pTerm = new Pel[N];

  Int i, j;

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      i = 0;
      ::memset(pTerm, 0, sizeof(Pel)*N);
      for(Int yy=y-offset; yy<=y+offset; yy++)
      {
        for(Int xx=x-offset; xx<=x+offset; xx++)
        {
          pTerm[pFiltPos[i]] += pCmp[xx + yy*iCmpStride];
          i++;
        }
      }

      for(j=0; j<N; j++)
      {
        m_ppdAlfCorr[j][j] += pTerm[j]*pTerm[j];
        for(i=j+1; i<N; i++)
          m_ppdAlfCorr[j][i] += pTerm[j]*pTerm[i];

        // DC offset
        m_ppdAlfCorr[j][N]   += pTerm[j];
        m_ppdAlfCorr[j][N+1] += pOrg[x+y*iOrgStride]*pTerm[j];
      }
      // DC offset
      for(i=0; i<N; i++)
        m_ppdAlfCorr[N][i] += pTerm[i];
      m_ppdAlfCorr[N][N]   += 1;
      m_ppdAlfCorr[N][N+1] += pOrg[x+y*iOrgStride];
    }
  }

  for(j=0; j<N-1; j++)
    for(i=j+1; i<N; i++)
      m_ppdAlfCorr[i][j] = m_ppdAlfCorr[j][i];

  delete[] pTerm;
  pTerm = NULL;
}

Void TEncAdaptiveLoopFilter::xCalcCorrelationFuncBlock(Pel* pOrg, Pel* pCmp, Int iTap, Int iWidth, Int iHeight, Int iOrgStride, Int iCmpStride)
{
  //Patch should be extended before this point................
  //ext_offset  = tap>>1;

  Int N      = (iTap*iTap+1)>>1;
  Int offset = iTap>>1;

  const Int* pFiltPos;

  switch(iTap)
  {
  case 5:
    pFiltPos = m_aiSymmetricArray5x5;
    break;
  case 7:
    pFiltPos = m_aiSymmetricArray7x7;
    break;
  case 9:
    pFiltPos = m_aiSymmetricArray9x9;
    break;
  default:
		pFiltPos = m_aiSymmetricArray9x9;
    assert(0);
    break;
  }

  Pel* pTerm = new Pel[N];

  Int i, j;

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      i = 0;
      ::memset(pTerm, 0, sizeof(Pel)*N);
      for(Int yy=y-offset; yy<=y+offset; yy++)
      {
        for(Int xx=x-offset; xx<=x+offset; xx++)
        {
          pTerm[pFiltPos[i]] += pCmp[xx + yy*iCmpStride];
          i++;
        }
      }

      for(j=0; j<N; j++)
      {
        m_ppdAlfCorr[j][j] += pTerm[j]*pTerm[j];
        for(i=j+1; i<N; i++)
          m_ppdAlfCorr[j][i] += pTerm[j]*pTerm[i];

        // DC offset
        m_ppdAlfCorr[j][N]   += pTerm[j];
        m_ppdAlfCorr[j][N+1] += pOrg[x+y*iOrgStride]*pTerm[j];
      }
      // DC offset
      for(i=0; i<N; i++)
        m_ppdAlfCorr[N][i] += pTerm[i];
      m_ppdAlfCorr[N][N]   += 1;
      m_ppdAlfCorr[N][N+1] += pOrg[x+y*iOrgStride];
    }
  }

  delete[] pTerm;
  pTerm = NULL;
}

UInt64 TEncAdaptiveLoopFilter::xCalcSSD(Pel* pOrg, Pel* pCmp, Int iWidth, Int iHeight, Int iStride )
{
  UInt64 uiSSD = 0;
  Int x, y;

  UInt uiShift = g_uiBitIncrement<<1;
  Int iTemp;

  for( y = 0; y < iHeight; y++ )
  {
    for( x = 0; x < iWidth; x++ )
    {
      iTemp = pOrg[x] - pCmp[x]; uiSSD += ( iTemp * iTemp ) >> uiShift;
    }
    pOrg += iStride;
    pCmp += iStride;
  }

	return uiSSD;;
}

Int TEncAdaptiveLoopFilter::xGauss(Double **a, Int N)
{
  Int i, j, k;
  Double t;

#if ALF_FIX
  for(k=0; k<N; k++)
  {
    if (a[k][k] <0.000001)
        return 1;
  }
#endif

  for(k=0; k<N-1; k++)
  {
    for(i=k+1;i<N; i++)
    {
      t=a[i][k]/a[k][k];
      for(j=k+1; j<=N; j++)
      {
        a[i][j] -= t * a[k][j];
        if(i==j && fabs(a[i][j])<0.000001) return 1;
      }
    }
  }
  for(i=N-1; i>=0; i--)
  {
    t = a[i][N];
    for(j=i+1; j<N; j++)
      t -= a[i][j] * a[j][N];
    a[i][N] = t / a[i][i];
  }
  return 0;
}

Void TEncAdaptiveLoopFilter::xFilterCoefQuickSort( Double *coef_data, Int *coef_num, Int upper, Int lower )
{
  Double mid, tmp_data;
  Int i, j, tmp_num;

  i = upper;
  j = lower;
  mid = coef_data[(lower+upper)>>1];
  do
  {
    while( coef_data[i] < mid ) i++;
    while( mid < coef_data[j] ) j--;
    if( i <= j )
    {
      tmp_data = coef_data[i];
      tmp_num  = coef_num[i];
      coef_data[i] = coef_data[j];
      coef_num[i]  = coef_num[j];
      coef_data[j] = tmp_data;
      coef_num[j]  = tmp_num;
      i++;
      j--;
    }
  } while( i <= j );
  if ( upper < j ) xFilterCoefQuickSort(coef_data, coef_num, upper, j);
  if ( i < lower ) xFilterCoefQuickSort(coef_data, coef_num, i, lower);
}

Void TEncAdaptiveLoopFilter::xQuantFilterCoef(Double* h, Int* qh, Int tap, int bit_depth)
{
  Int i, N;
  Int max_value, min_value;
  Double dbl_total_gain;
  Int total_gain, q_total_gain;
  Int upper, lower;
  Double *dh;
  Int    *nc;
  const Int    *pFiltMag;

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
		pFiltMag = m_aiSymmetricMag9x9;
    assert(0);
    break;
  }

  N = (tap*tap+1)>>1;

  dh = new Double[N];
  nc = new Int[N];

  max_value =   (1<<(1+ALF_NUM_BIT_SHIFT))-1;
  min_value = 0-(1<<(1+ALF_NUM_BIT_SHIFT));

  dbl_total_gain=0.0;
  q_total_gain=0;
  for(i=0; i<N; i++)
  {
    if(h[i]>=0.0)
      qh[i] =  (Int)( h[i]*(1<<ALF_NUM_BIT_SHIFT)+0.5);
    else
      qh[i] = -(Int)(-h[i]*(1<<ALF_NUM_BIT_SHIFT)+0.5);

    dh[i] = (Double)qh[i]/(Double)(1<<ALF_NUM_BIT_SHIFT) - h[i];
    dh[i]*=pFiltMag[i];
    dbl_total_gain += h[i]*pFiltMag[i];
    q_total_gain   += qh[i]*pFiltMag[i];
    nc[i] = i;
  }

  // modification of quantized filter coefficients
  total_gain = (Int)(dbl_total_gain*(1<<ALF_NUM_BIT_SHIFT)+0.5);

  if( q_total_gain != total_gain )
  {
    xFilterCoefQuickSort(dh, nc, 0, N-1);
    if( q_total_gain > total_gain )
    {
      upper = N-1;
      while( q_total_gain > total_gain+1 )
      {
        i = nc[upper%N];
        qh[i]--;
        q_total_gain -= pFiltMag[i];
        upper--;
      }
      if( q_total_gain == total_gain+1 )
      {
        if(dh[N-1]>0)
          qh[N-1]--;
        else
        {
          i=nc[upper%N];
          qh[i]--;
          qh[N-1]++;
        }
      }
    }
    else if( q_total_gain < total_gain )
    {
      lower = 0;
      while( q_total_gain < total_gain-1 )
      {
        i=nc[lower%N];
        qh[i]++;
        q_total_gain += pFiltMag[i];
        lower++;
      }
      if( q_total_gain == total_gain-1 )
      {
        if(dh[N-1]<0)
          qh[N-1]++;
        else
        {
          i=nc[lower%N];
          qh[i]++;
          qh[N-1]--;
        }
      }
    }
  }

  // set of filter coefficients
  for(i=0; i<N; i++)
  {
    qh[i] = Max(min_value,Min(max_value, qh[i]));
  }

  // DC offset
//  max_value = Min(  (1<<(3+Max(img_bitdepth_luma,img_bitdepth_chroma)))-1, (1<<14)-1);
//  min_value = Max( -(1<<(3+Max(img_bitdepth_luma,img_bitdepth_chroma))),  -(1<<14)  );
  max_value = Min(  (1<<(3+g_uiBitDepth + g_uiBitIncrement))-1, (1<<14)-1);
  min_value = Max( -(1<<(3+g_uiBitDepth + g_uiBitIncrement)),  -(1<<14)  );

  qh[N] =  (h[N]>=0.0)? (Int)( h[N]*(1<<(ALF_NUM_BIT_SHIFT-bit_depth+8)) + 0.5) : -(Int)(-h[N]*(1<<(ALF_NUM_BIT_SHIFT-bit_depth+8)) + 0.5);
  qh[N] = Max(min_value,Min(max_value, qh[N]));

  delete[] dh;
  dh = NULL;

  delete[] nc;
  nc = NULL;
}

Void TEncAdaptiveLoopFilter::xClearFilterCoefInt(Int* qh, Int N)
{
	// clear
  memset( qh, 0, sizeof( Int ) * N );

  // center pos
  qh[N-2]  = 1<<ALF_NUM_BIT_SHIFT;
}

Void TEncAdaptiveLoopFilter::xCalcRDCost(ALFParam* pAlfParam, UInt64& ruiRate, UInt64 uiDist, Double& rdCost)
{
  if(pAlfParam != NULL)
  {
    Int* piTmpCoef;
    piTmpCoef = new Int[ALF_MAX_NUM_COEF];

    memcpy(piTmpCoef, pAlfParam->coeff, sizeof(Int)*pAlfParam->num_coeff);

    predictALFCoeff(pAlfParam);

    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam(pAlfParam);

    if(pAlfParam->cu_control_flag)
    {
#if TSB_ALF_HEADER
      m_pcEntropyCoder->encodeAlfCtrlParam(pAlfParam);
#else
      xEncodeCUAlfCtrlFlags();
#endif
    }
    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
    memcpy(pAlfParam->coeff, piTmpCoef, sizeof(int)*pAlfParam->num_coeff);
    delete[] piTmpCoef;
    piTmpCoef = NULL;
  }
  else
  {
    ruiRate = 1;
  }

  rdCost      = (Double)(ruiRate) * m_dLambdaLuma + (Double)(uiDist);
}

#if WIENER_3_INPUT
#if ALF_MEM_PATCH
Void TEncAdaptiveLoopFilter::xCalcRDCost_precision(imgpel* ImgOrg, imgpel* imgCmp, Int width, Int height, ALFParam* pAlfParam, Int64& iRate, Int64& iDist, Double& rdCost, Int Stride)
#else    
Void TEncAdaptiveLoopFilter::xCalcRDCost_precision(imgpel** ImgOrg, imgpel** imgCmp, Int width, Int height, ALFParam* pAlfParam, Int64& iRate, Int64& iDist, Double& rdCost)
#endif
{
  Int64  tmp;
  Int64  iAdd   = (Int64)((g_uiBitIncrement>0)?(1<<(g_uiBitIncrement-1)):0);
  
#if ALF_MEM_PATCH
  imgpel* pImgOrg;
  imgpel* pimgCmp;
#endif  
  if(pAlfParam != NULL)
  {
    Int* piTmpCoef;
    piTmpCoef = new Int[ALF_MAX_NUM_COEF];

    memcpy(piTmpCoef, pAlfParam->coeff, sizeof(Int)*pAlfParam->num_coeff);

    predictALFCoeff(pAlfParam);

    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam(pAlfParam);

    if(pAlfParam->cu_control_flag)
    {
#if TSB_ALF_HEADER
      m_pcEntropyCoder->encodeAlfCtrlParam(pAlfParam);
#else
      xEncodeCUAlfCtrlFlags();
#endif
    }
    iRate = m_pcEntropyCoder->getNumberOfWrittenBits();
    memcpy(pAlfParam->coeff, piTmpCoef, sizeof(int)*pAlfParam->num_coeff);
    delete[] piTmpCoef;
    piTmpCoef = NULL;
  }
  else
  {
    iRate = 1;
  }

  iDist=0;
  
#if ALF_MEM_PATCH
  pImgOrg = ImgOrg;
  pimgCmp = imgCmp;
#endif  
  for (Int j=0;j<height;j++)
  {
    for (Int i=0;i<width;i++)
    {
#if ALF_MEM_PATCH
      tmp = (((Int64)pImgOrg[i]+iAdd)>>(Int64)g_uiBitIncrement);
      tmp-= (((Int64)pimgCmp[i]+iAdd)>>(Int64)g_uiBitIncrement);      
#else      
      tmp = (((Int64)ImgOrg[j][i]+iAdd)>>(Int64)g_uiBitIncrement);
      tmp-= (((Int64)imgCmp[j][i]+iAdd)>>(Int64)g_uiBitIncrement);
#endif
      tmp*=tmp;
      iDist+=tmp;
    }
#if ALF_MEM_PATCH
    pImgOrg +=Stride;
    pimgCmp +=Stride;
#endif    
  }
  
  rdCost      = (Double)(iRate) * m_dLambdaLuma + (Double)(iDist);  
}
#endif


Void TEncAdaptiveLoopFilter::xCalcRDCost(TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost)
{
  if(pAlfParam != NULL)
  {
    Int* piTmpCoef;
    piTmpCoef = new Int[ALF_MAX_NUM_COEF];

    memcpy(piTmpCoef, pAlfParam->coeff, sizeof(Int)*pAlfParam->num_coeff);

    predictALFCoeff(pAlfParam);

    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam(pAlfParam);

    if(pAlfParam->cu_control_flag)
    {
#if TSB_ALF_HEADER
      m_pcEntropyCoder->encodeAlfCtrlParam(pAlfParam);
#else
      xEncodeCUAlfCtrlFlags();
#endif
    }
    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
    memcpy(pAlfParam->coeff, piTmpCoef, sizeof(int)*pAlfParam->num_coeff);
    delete[] piTmpCoef;
    piTmpCoef = NULL;
  }
  else
  {
    ruiRate = 1;
  }

  ruiDist     = xCalcSSD(pcPicOrg->getLumaAddr(), pcPicCmp->getLumaAddr(), pcPicOrg->getWidth(), pcPicOrg->getHeight(), pcPicOrg->getStride());
  rdCost      = (Double)(ruiRate) * m_dLambdaLuma + (Double)(ruiDist);
}

Void TEncAdaptiveLoopFilter::xCalcRDCostChroma(TComPicYuv* pcPicOrg, TComPicYuv* pcPicCmp, ALFParam* pAlfParam, UInt64& ruiRate, UInt64& ruiDist, Double& rdCost)
{
  if(pAlfParam->chroma_idc)
  {
    Int* piTmpCoef;
    piTmpCoef = new Int[ALF_MAX_NUM_COEF_C];

    memcpy(piTmpCoef, pAlfParam->coeff_chroma, sizeof(Int)*pAlfParam->num_coeff_chroma);

    predictALFCoeffChroma(pAlfParam);

    m_pcEntropyCoder->resetEntropy();
    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeAlfParam(pAlfParam);

    if(pAlfParam->cu_control_flag)
    {
#if TSB_ALF_HEADER
      m_pcEntropyCoder->encodeAlfCtrlParam(pAlfParam);
#else
      xEncodeCUAlfCtrlFlags();
#endif
    }
    ruiRate = m_pcEntropyCoder->getNumberOfWrittenBits();
    memcpy(pAlfParam->coeff_chroma, piTmpCoef, sizeof(int)*pAlfParam->num_coeff_chroma);
    delete[] piTmpCoef;
    piTmpCoef = NULL;
  }
  ruiDist = 0;
  ruiDist += xCalcSSD(pcPicOrg->getCbAddr(), pcPicCmp->getCbAddr(), (pcPicOrg->getWidth()>>1), (pcPicOrg->getHeight()>>1), pcPicOrg->getCStride());
  ruiDist += xCalcSSD(pcPicOrg->getCrAddr(), pcPicCmp->getCrAddr(), (pcPicOrg->getWidth()>>1), (pcPicOrg->getHeight()>>1), pcPicOrg->getCStride());
  rdCost  = (Double)(ruiRate) * m_dLambdaChroma + (Double)(ruiDist);
}

Void TEncAdaptiveLoopFilter::xFilteringFrameLuma(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Bool bStoreCorr)
{
  Int    i, tap, N, err_code;
  Int* qh;
  Int    j, k;
  
  tap  = m_pcTempAlfParam->tap;
  N    = m_pcTempAlfParam->num_coeff;
  qh   = m_pcTempAlfParam->coeff;

  // initialize correlation
  for(i=0; i<N; i++)
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*(N+1));

  if(bStoreCorr)
  {
    // store correlation per minimum size cu
    for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
    for( UInt uiIdx = 0; uiIdx < m_pcPic->getNumPartInCU() ; uiIdx+=4 )
    {
      TComDataCU* pcCU = m_pcPic->getCU(uiCUAddr);
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if (uiLPelX >= pcPicOrg->getWidth() || uiTPelY >= pcPicOrg->getHeight())
      {
        continue;
      }
      CorrBlk &ppuiBlkCorr = m_puiCUCorr[uiCUAddr][(uiIdx>>2)]; 

      for(j=0; j<N; j++)
        memset(ppuiBlkCorr[j], 0, sizeof(UInt)*(N+1-j));      

      Pel* pOrg = pcPicOrg->getLumaAddr(uiCUAddr, uiIdx);
      Pel* pCmp = pcPicDec->getLumaAddr(uiCUAddr, uiIdx);
      xCalcStoredCorrelationFuncBlock(pOrg, pCmp, ppuiBlkCorr, tap, m_uiSCUWidth, m_uiSCUHeight, pcPicOrg->getStride(), pcPicDec->getStride());
      
      for(j=0; j<N; j++)
        for(k=j; k<N+1; k++)
          m_ppdAlfCorr[j][k] += (Double) ppuiBlkCorr[j][k-j];
    }
    for(j=0; j<N-1; j++)
      for(k=j+1; k<N; k++)
        m_ppdAlfCorr[k][j] = m_ppdAlfCorr[j][k];
  }
  else
  {
    Pel* pOrg = pcPicOrg->getLumaAddr();
    Pel* pCmp = pcPicDec->getLumaAddr();

    xCalcCorrelationFunc(pOrg, pCmp, tap, pcPicOrg->getWidth(), pcPicOrg->getHeight(), pcPicOrg->getStride(), pcPicDec->getStride());
  }

  err_code = xGauss(m_ppdAlfCorr, N);

  if(err_code)
  {
    xClearFilterCoefInt(qh, N);
  }
  else
  {
    for(i=0; i<N; i++)
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][N];

    xQuantFilterCoef(m_pdDoubleAlfCoeff, qh, tap, g_uiBitDepth + g_uiBitIncrement);
  }

  xFrame(pcPicDec, pcPicRest, qh, tap);
}

Void TEncAdaptiveLoopFilter::xFilteringFrameChroma(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  Int    i, tap, N, err_code;
  Int* qh;

  tap  = m_pcTempAlfParam->tap_chroma;
  N    = m_pcTempAlfParam->num_coeff_chroma;
  qh   = m_pcTempAlfParam->coeff_chroma;

  // initialize correlation
  for(i=0; i<N; i++)
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*(N+1));

  if ((m_pcTempAlfParam->chroma_idc>>1)&0x01)
  {
    Pel* pOrg = pcPicOrg->getCbAddr();
    Pel* pCmp = pcPicDec->getCbAddr();

    xCalcCorrelationFunc(pOrg, pCmp, tap, (pcPicOrg->getWidth()>>1), (pcPicOrg->getHeight()>>1), pcPicOrg->getCStride(), pcPicDec->getCStride());
  }
  if ((m_pcTempAlfParam->chroma_idc)&0x01)
  {
    Pel* pOrg = pcPicOrg->getCrAddr();
    Pel* pCmp = pcPicDec->getCrAddr();

    xCalcCorrelationFunc(pOrg, pCmp, tap, (pcPicOrg->getWidth()>>1), (pcPicOrg->getHeight()>>1), pcPicOrg->getCStride(), pcPicDec->getCStride());
  }

  err_code = xGauss(m_ppdAlfCorr, N);

  if(err_code)
  {
    xClearFilterCoefInt(qh, N);
  }
  else
  {
    for(i=0; i<N; i++)
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][N];

    xQuantFilterCoef(m_pdDoubleAlfCoeff, qh, tap, g_uiBitDepth + g_uiBitIncrement);
  }


  if ((m_pcTempAlfParam->chroma_idc>>1)&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, qh, tap, 0);
  }
  if ((m_pcTempAlfParam->chroma_idc)&0x01)
  {
    xFrameChroma(pcPicDec, pcPicRest, qh, tap, 1);
  }

  if(m_pcTempAlfParam->chroma_idc<3)
  {
    if(m_pcTempAlfParam->chroma_idc==1)
    {
      pcPicDec->copyToPicCb(pcPicRest);
    }
    if(m_pcTempAlfParam->chroma_idc==2)
    {
      pcPicDec->copyToPicCr(pcPicRest);
    }
  }

}

Void TEncAdaptiveLoopFilter::xReDesignFilterCoeff(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, Bool bReadCorr)
{
  Int i, j, k;

  Int tap = m_pcTempAlfParam->tap;
  Int N = m_pcTempAlfParam->num_coeff;
  Int* qh = m_pcTempAlfParam->coeff;
  // initialize correlation
  for(i=0; i<N; i++)
    memset(m_ppdAlfCorr[i], 0, sizeof(Double)*(N+1));

  xReadOrCalcCorrFromCUs(pcPicOrg, pcPicDec, bReadCorr);

  for(j=0; j<N-1; j++)
    for(k=j+1; k<N; k++)
      m_ppdAlfCorr[k][j] = m_ppdAlfCorr[j][k];

  Int err_code = xGauss(m_ppdAlfCorr, N);

  if(err_code)
  {
    xClearFilterCoefInt(qh, N);
  }
  else
  {
    for(i=0; i<N; i++)
      m_pdDoubleAlfCoeff[i] = m_ppdAlfCorr[i][N];

    xQuantFilterCoef(m_pdDoubleAlfCoeff, qh, tap, g_uiBitDepth + g_uiBitIncrement);
  }
}

#if TSB_ALF_HEADER
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlags(UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, ALFParam *pAlfParam)
#else
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlags(UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist)
#endif
{
  ruiDist = 0;
#if TSB_ALF_HEADER
  pAlfParam->num_alf_cu_flag = 0;
#endif

  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
#if TSB_ALF_HEADER
    xSetCUAlfCtrlFlag(pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist, pAlfParam);
#else
    xSetCUAlfCtrlFlag(pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
#endif
  }
}

#if TSB_ALF_HEADER
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, ALFParam *pAlfParam)
#else
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist)
#endif
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
    UInt uiQNumParts = ( m_pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
#if TSB_ALF_HEADER
        xSetCUAlfCtrlFlag(pcCU, uiAbsPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist, pAlfParam);
#else
        xSetCUAlfCtrlFlag(pcCU, uiAbsPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
#endif
    }
    return;
  }

  if( uiDepth > uiAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    return;
  }

  UInt uiCUAddr = pcCU->getAddr();
  UInt64 uiRecSSD = 0;
  UInt64 uiFiltSSD = 0;

  Int iWidth;
  Int iHeight;
  UInt uiSetDepth;

  if (uiDepth > uiAlfCtrlDepth && pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    iWidth = g_uiMaxCUWidth >> uiAlfCtrlDepth;
    iHeight = g_uiMaxCUHeight >> uiAlfCtrlDepth;

    uiRPelX   = uiLPelX + iWidth  - 1;
    uiBPelY   = uiTPelY + iHeight - 1;

    if( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() )
    {
      iWidth = pcCU->getSlice()->getSPS()->getWidth() - uiLPelX;
    }

    if( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() )
    {
      iHeight = pcCU->getSlice()->getSPS()->getHeight() - uiTPelY;
    }

    uiSetDepth = uiAlfCtrlDepth;
  }
  else
  {
    iWidth = pcCU->getWidth(uiAbsPartIdx);
    iHeight = pcCU->getHeight(uiAbsPartIdx);
    uiSetDepth = uiDepth;
  }

  Pel* pOrg = pcPicOrg->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pRec = pcPicDec->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pFilt = pcPicRest->getLumaAddr(uiCUAddr, uiAbsPartIdx);

  uiRecSSD  += xCalcSSD( pOrg, pRec,  iWidth, iHeight, pcPicOrg->getStride() );
  uiFiltSSD += xCalcSSD( pOrg, pFilt, iWidth, iHeight, pcPicOrg->getStride() );

  if (uiFiltSSD < uiRecSSD)
  {
    ruiDist += uiFiltSSD;
    pcCU->setAlfCtrlFlagSubParts(1, uiAbsPartIdx, uiSetDepth);
#if TSB_ALF_HEADER
    pAlfParam->alf_cu_flag[pAlfParam->num_alf_cu_flag]=1;
#endif
  }
  else
  {
    ruiDist += uiRecSSD;
    pcCU->setAlfCtrlFlagSubParts(0, uiAbsPartIdx, uiSetDepth);
#if TSB_ALF_HEADER
    pAlfParam->alf_cu_flag[pAlfParam->num_alf_cu_flag]=0;
#endif
  }
#if TSB_ALF_HEADER
  pAlfParam->num_alf_cu_flag++;
#endif
}

Void TEncAdaptiveLoopFilter::xCopyDecToRestCUs(TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
{
  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
    xCopyDecToRestCU(pcCU, 0, 0, pcPicDec, pcPicRest);
  }
}

Void TEncAdaptiveLoopFilter::xCopyDecToRestCU(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest)
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
    UInt uiQNumParts = ( m_pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
        xCopyDecToRestCU(pcCU, uiAbsPartIdx, uiDepth+1, pcPicDec, pcPicRest);
    }
    return;
  }

  if (!pcCU->getAlfCtrlFlag(uiAbsPartIdx))
  {
    UInt uiCUAddr = pcCU->getAddr();

    Int iWidth = pcCU->getWidth(uiAbsPartIdx);
    Int iHeight = pcCU->getHeight(uiAbsPartIdx);

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

#if QC_ALF
#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi,
#else    
Void TEncAdaptiveLoopFilter::xcollectStatCodeFilterCoeffForce0(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, 
#endif
                                     int filters_per_group, int bitsVarBin[])
{
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal, 
    *pDepthInt=NULL, kMinTab[MAX_SQR_FILT_LENGTH], bitsCoeffScan[MAX_SCAN_VAL][MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;    
#if WIENER_3_INPUT
  pDepthInt = depth;
#else
  pDepthInt=pDepthIntTab[fl-2];
#endif
  maxScanVal=0;
  for (i=0; i<sqrFiltLength; i++)
  {
    maxScanVal=max(maxScanVal, pDepthInt[i]);
  }

  // vlc for all
  memset(bitsCoeffScan, 0, MAX_SCAN_VAL * MAX_EXP_GOLOMB * sizeof(int));
  for(ind=0; ind<filters_per_group; ++ind){	
    for(i = 0; i < sqrFiltLength; i++){	     
      scanPos=pDepthInt[i]-1;
      coeffVal=abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (k=1; k<15; k++){
        bitsCoeffScan[scanPos][k] += lengthGolomb(coeffVal, k);
      }
    }
  }

  minBitsKStart=0;
  minKStart = -1;
  for (k=1; k<8; k++){ 
    bitsKStart=0; kStart=k;
    for (scanPos=0; scanPos<maxScanVal; scanPos++){
      kMin=kStart; minBits=bitsCoeffScan[scanPos][kMin];

      if (bitsCoeffScan[scanPos][kStart+1]<minBits){
        kMin=kStart+1; minBits=bitsCoeffScan[scanPos][kMin];
      }
      kStart=kMin;
      bitsKStart+=minBits;
    }
    if (bitsKStart<minBitsKStart || k==1){
      minBitsKStart=bitsKStart;
      minKStart=k;
    }
  }

  kStart = minKStart; 
  for (scanPos=0; scanPos<maxScanVal; scanPos++)
  {
    kMin=kStart; minBits=bitsCoeffScan[scanPos][kMin];

    if (bitsCoeffScan[scanPos][kStart+1]<minBits)
    {
      kMin = kStart+1; 
      minBits = bitsCoeffScan[scanPos][kMin];
    }
   
    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  for(ind=0; ind<filters_per_group; ++ind){
    bitsVarBin[ind]=0;
    for(i = 0; i < sqrFiltLength; i++){	
      scanPos=pDepthInt[i]-1;
      bitsVarBin[ind] += lengthGolomb(abs(pDiffQFilterCoeffIntPP[ind][i]), kMinTab[scanPos]);
    }
  }
}

Void TEncAdaptiveLoopFilter::xdecideCoeffForce0(int codedVarBins[NO_VAR_BINS], double errorForce0Coeff[], double errorForce0CoeffTab[NO_VAR_BINS][2], int bitsVarBin[NO_VAR_BINS], double lambda, int filters_per_fr)
{
  int filtNo;
  double lagrangianDiff;
  int ind;

  errorForce0Coeff[0]=errorForce0Coeff[1]=0;
  for (ind=0; ind<16; ind++) codedVarBins[ind]=0;

  for(filtNo=0; filtNo<filters_per_fr; filtNo++)
  {
    // No coeffcient prediction bits used
#if ENABLE_FORCECOEFF0
    lagrangianDiff=errorForce0CoeffTab[filtNo][0]-(errorForce0CoeffTab[filtNo][1]+lambda*bitsVarBin[filtNo]);
    codedVarBins[filtNo]=(lagrangianDiff>0)? 1 : 0;
    errorForce0Coeff[0]+=errorForce0CoeffTab[filtNo][codedVarBins[filtNo]];
    errorForce0Coeff[1]+=errorForce0CoeffTab[filtNo][1];
#else
    lagrangianDiff=errorForce0CoeffTab[filtNo][0]-(errorForce0CoeffTab[filtNo][1]+lambda*bitsVarBin[filtNo]);
    codedVarBins[filtNo]= 1;
    errorForce0Coeff[0]+=errorForce0CoeffTab[filtNo][codedVarBins[filtNo]];
    errorForce0Coeff[1]+=errorForce0CoeffTab[filtNo][1];
#endif
  }   
}

#if WIENER_3_INPUT
double TEncAdaptiveLoopFilter::xfindBestCoeffCodMethod(int codedVarBins[NO_VAR_BINS], int *forceCoeff0, int prec_rec, int prec_pred, int prec_resi,
                              int **filterCoeffSymQuant, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi,
#else                              
double TEncAdaptiveLoopFilter::xfindBestCoeffCodMethod(int codedVarBins[NO_VAR_BINS], int *forceCoeff0, 
                              int **filterCoeffSymQuant, int fl, int sqrFiltLength, 
#endif                              
                              int filters_per_fr, double errorForce0CoeffTab[NO_VAR_BINS][2], 
                              double *errorQuant, double lambda)

{
  int bitsVarBin[NO_VAR_BINS], createBistream, coeffBits, coeffBitsForce0;
  double errorForce0Coeff[2], lagrangianForce0, lagrangian;

#if WIENER_3_INPUT
  xcollectStatCodeFilterCoeffForce0(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi,
#else                                    
  xcollectStatCodeFilterCoeffForce0(filterCoeffSymQuant, fl, sqrFiltLength,  
#endif
    filters_per_fr, bitsVarBin);

  xdecideCoeffForce0(codedVarBins, errorForce0Coeff, errorForce0CoeffTab, bitsVarBin, lambda, filters_per_fr);
  
#if WIENER_3_INPUT
  coeffBitsForce0 = xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi,
#else      
  coeffBitsForce0 = xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, 
#endif
    filters_per_fr, codedVarBins, createBistream=0, tempALFp);

#if WIENER_3_INPUT
  coeffBits = xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi, filters_per_fr,
#else                                    
  coeffBits = xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, filters_per_fr, 
#endif
    createBistream=0, tempALFp);

  lagrangianForce0=errorForce0Coeff[0]+lambda*coeffBitsForce0;
  lagrangian=errorForce0Coeff[1]+lambda*coeffBits;
  if (lagrangianForce0<lagrangian)
  {
    *errorQuant=errorForce0Coeff[0];
    *forceCoeff0=1;
    return(lagrangianForce0);
  }
  else
  {
    *errorQuant=errorForce0Coeff[1];
    *forceCoeff0=0;
    return(lagrangian);
  }
}

#if WIENER_3_INPUT
Int TEncAdaptiveLoopFilter::xsendAllFiltersPPPred(int **FilterCoeffQuant, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int prec_rec, int prec_pred, int prec_resi,
#else
Int TEncAdaptiveLoopFilter::xsendAllFiltersPPPred(int **FilterCoeffQuant, int fl, int sqrFiltLength, 
#endif
                         int filters_per_group, int createBistream, ALFParam* ALFp)
{
  int ind, bit_ct = 0, bit_ct0 = 0, i;
  int predMethod = 0;
  int force0 = 0;
#if !ALF_MEM_PATCH
  static int **diffFilterCoeffQuant;
  static int first = 1;
#endif
#if WIENER_3_INPUT
  int pred_rec, pred_pred, pred_resi;  
#if !ALF_MEM_PATCH
  static int **FilterCoeffQuant_predicted;
#endif
  int mid_rec =sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-2;
  int mid_pred=mid_rec+sqrFiltLength_resi+sqrFiltLength_pred;
  int mid_resi=mid_rec+sqrFiltLength_resi;
  int one=1<<(ALFp->filter_precision_table_shift[prec_rec]);
  int tapTab[3] = {22, 14, 8};
  int filt_size;
#endif  
  Int64 Newbit_ct;
  
#if !ALF_MEM_PATCH
  if(first == 1)
  {
    initMatrix_int(&diffFilterCoeffQuant, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
    first = 0;
  }
#endif
#if WIENER_3_INPUT
#if !ALF_MEM_PATCH  
  get_mem2Dint   (&FilterCoeffQuant_predicted, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
#endif
  
  for(ind = 0; ind < NO_VAR_BINS; ind++)
  {
    pred_rec=one;
    if (prec_pred>prec_rec)
    {
      pred_pred=(one-FilterCoeffQuant[ind][mid_rec])<<(prec_pred-prec_rec);
    }
    else
    {
      pred_pred=(one-FilterCoeffQuant[ind][mid_rec])>>(prec_rec-prec_pred);
    }
    if (prec_resi>prec_pred)
    {
      pred_resi=(FilterCoeffQuant[ind][mid_pred])<<(prec_resi-prec_pred);
    }
    else
    {
      pred_resi=(FilterCoeffQuant[ind][mid_pred])>>(prec_pred-prec_resi);
    }
    
    for(int j = 0; j < MAX_SQR_FILT_LENGTH; j++)
    {
      FilterCoeffQuant_predicted[ind][j]=FilterCoeffQuant[ind][j];
    }
    
    FilterCoeffQuant_predicted[ind][mid_resi] -= pred_resi;
    FilterCoeffQuant_predicted[ind][mid_pred] -= pred_pred;
    FilterCoeffQuant_predicted[ind][mid_rec ] -= pred_rec;    
  }
  
  bit_ct0 = xcodeFilterCoeff(FilterCoeffQuant_predicted, fl, sqrFiltLength, filters_per_group, 0);
#else
  bit_ct0 = xcodeFilterCoeff(FilterCoeffQuant, fl, sqrFiltLength, filters_per_group, 0);
#endif

  for(ind = 0; ind < filters_per_group; ++ind)
  {		
    if(ind == 0)
    {
      for(i = 0; i < sqrFiltLength; i++)
#if WIENER_3_INPUT
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuant_predicted[ind][i];
#else      
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuant[ind][i];
#endif
    }
    else
    {
      for(i = 0; i < sqrFiltLength; i++)
#if WIENER_3_INPUT
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuant_predicted[ind][i] - FilterCoeffQuant_predicted[ind-1][i];
#else      
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuant[ind][i] - FilterCoeffQuant[ind-1][i];
#endif
    }
  }

  if(xcodeFilterCoeff(diffFilterCoeffQuant, fl, sqrFiltLength, filters_per_group, 0) >= bit_ct0)
  {
    predMethod = 0;  
    if(filters_per_group > 1)
      bit_ct += lengthPredFlags(force0, predMethod, NULL, 0, createBistream);
#if WIENER_3_INPUT
    bit_ct += xcodeFilterCoeff(FilterCoeffQuant_predicted, fl, sqrFiltLength, filters_per_group, createBistream);
#else    
    bit_ct += xcodeFilterCoeff(FilterCoeffQuant, fl, sqrFiltLength, filters_per_group, createBistream);
#endif
  }
  else
  {
    predMethod = 1;
    if(filters_per_group > 1)
      bit_ct += lengthPredFlags(force0, predMethod, NULL, 0, createBistream);
    bit_ct += xcodeFilterCoeff(diffFilterCoeffQuant, fl, sqrFiltLength, filters_per_group, createBistream);
  }
  
  ALFp->forceCoeff0 = 0;
  ALFp->filters_per_group_diff = filters_per_group;
  ALFp->filters_per_group = filters_per_group;
  ALFp->predMethod = predMethod;
  ALFp->num_coeff = sqrFiltLength;
#if WIENER_3_INPUT
  ALFp->filter_precision[0]=prec_rec;
  ALFp->filter_precision[1]=prec_pred;
  ALFp->filter_precision[2]=prec_resi;
  
  ALFp->num_coeff_pred = sqrFiltLength_pred;
  ALFp->num_coeff_resi = sqrFiltLength_resi;
  if (fl == 2) ALFp->realfiltNo=2;
  else if (fl == 3) ALFp->realfiltNo=1;
  else ALFp->realfiltNo=0;
#else  
  if (ALFp->num_coeff == 8) ALFp->realfiltNo=2;
  else if (ALFp->num_coeff == 14) ALFp->realfiltNo=1;
  else ALFp->realfiltNo=0;
#endif  
  
#if WIENER_3_INPUT
  filt_size = tapTab[ALFp->realfiltNo];
  
  if      (ALFp->num_coeff_pred == 1) ALFp->tap_pred = 1;
  else if (ALFp->num_coeff_pred == 3) ALFp->tap_pred = 3;
  else if (ALFp->num_coeff_pred == 7) ALFp->tap_pred = 5;
  else if (ALFp->num_coeff_pred ==13) ALFp->tap_pred = 7;
  else if (ALFp->num_coeff_pred ==21) ALFp->tap_pred = 9;

  if      (ALFp->num_coeff_resi == 1) ALFp->tap_resi = 1;
  else if (ALFp->num_coeff_resi == 3) ALFp->tap_resi = 3;
  else if (ALFp->num_coeff_resi == 7) ALFp->tap_resi = 5;
  else if (ALFp->num_coeff_resi ==13) ALFp->tap_resi = 7;
  else if (ALFp->num_coeff_resi ==21) ALFp->tap_resi = 9;
#endif
  
  for(ind = 0; ind < filters_per_group; ++ind)
  {		
    for(i = 0; i < sqrFiltLength; i++)
    {
      if (predMethod) ALFp->coeffmulti[ind][i] = diffFilterCoeffQuant[ind][i];
#if WIENER_3_INPUT
      else ALFp->coeffmulti[ind][i] = FilterCoeffQuant_predicted[ind][i];
#else      
      else ALFp->coeffmulti[ind][i] = FilterCoeffQuant[ind][i];
#endif
    }
  }
  m_pcDummyEntropyCoder->codeFiltCountBit(ALFp, &Newbit_ct);
  
#if (WIENER_3_INPUT && !ALF_MEM_PATCH)
  free_mem2Dint (FilterCoeffQuant_predicted);
#endif
    
//  return(bit_ct);
  return ((Int)Newbit_ct);
}


#if WIENER_3_INPUT
Int TEncAdaptiveLoopFilter::xsendAllFiltersPPPredForce0(int **FilterCoeffQuant, int fl, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int prec_rec, int prec_pred, int prec_resi, int filters_per_group, 
#else    
Int TEncAdaptiveLoopFilter::xsendAllFiltersPPPredForce0(int **FilterCoeffQuant, int fl, int sqrFiltLength, int filters_per_group, 
#endif
                               int codedVarBins[NO_VAR_BINS], int createBistream, ALFParam* ALFp)
{
  int ind, bit_ct=0, bit_ct0, i, j;
  int filters_per_group_temp, filters_per_group_diff;
  int chosenPred = 0;
#if !ALF_MEM_PATCH
  static int **diffFilterCoeffQuant, **FilterCoeffQuantTemp, first = 1;
#endif
#if WIENER_3_INPUT
  int mid_rec =sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-2;
  int mid_pred=mid_rec+sqrFiltLength_resi+sqrFiltLength_pred;
  int mid_resi=mid_rec+sqrFiltLength_resi;
  int one=1<<(ALFp->filter_precision_table_shift[prec_rec]);
  int tapTab[3] = {22, 14, 8};
  int pred_rec;
  int pred_pred;
  int pred_resi;
#endif    
  int force0 = 1;
  Int64 Newbit_ct;

#if !ALF_MEM_PATCH
  if(first == 1)
  {
    initMatrix_int(&diffFilterCoeffQuant, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
    initMatrix_int(&FilterCoeffQuantTemp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
    first = 0;
  }
#endif  

  i = 0;
  for(ind = 0; ind < filters_per_group; ind++)
  {		
    if(codedVarBins[ind] == 1)
    {
      for(j = 0; j < sqrFiltLength; j++)
        FilterCoeffQuantTemp[i][j]=FilterCoeffQuant[ind][j];
#if WIENER_3_INPUT      
      pred_rec=one;
      if (prec_pred>prec_rec)
      {
        pred_pred=(one-FilterCoeffQuantTemp[i][mid_rec])<<(prec_pred-prec_rec);
      }
      else
      {
        pred_pred=(one-FilterCoeffQuantTemp[i][mid_rec])>>(prec_rec-prec_pred);
      }
      if (prec_resi>prec_pred)
      {
        pred_resi=(FilterCoeffQuantTemp[i][mid_pred])<<(prec_resi-prec_pred);
      }
      else
      {
        pred_resi=(FilterCoeffQuantTemp[i][mid_pred])>>(prec_pred-prec_resi);
      }
      
      FilterCoeffQuantTemp[i][mid_resi] -= pred_resi;
      FilterCoeffQuantTemp[i][mid_pred] -= pred_pred;
      FilterCoeffQuantTemp[i][mid_rec ] -= pred_rec;
#endif
      i++;
    }
  }
  filters_per_group_diff = filters_per_group_temp = i;

  for(ind = 0; ind < filters_per_group; ++ind)
  {		
    if(ind == 0)
    {
      for(i = 0; i < sqrFiltLength; i++)
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuantTemp[ind][i];
    }
    else
    {
      for(i = 0; i < sqrFiltLength; i++)
        diffFilterCoeffQuant[ind][i] = FilterCoeffQuantTemp[ind][i] - FilterCoeffQuantTemp[ind-1][i];
    }
  }

  if(!((filters_per_group_temp == 0) && (filters_per_group == 1)))
  {
    bit_ct0 = xcodeFilterCoeff(FilterCoeffQuantTemp, fl, sqrFiltLength, filters_per_group_temp, 0);

    if(xcodeFilterCoeff(diffFilterCoeffQuant, fl, sqrFiltLength, filters_per_group_diff, 0) >= bit_ct0)
    {
      chosenPred = 0;
      bit_ct += lengthPredFlags(force0, chosenPred, codedVarBins, filters_per_group, createBistream);
      bit_ct += xcodeFilterCoeff(FilterCoeffQuantTemp, fl, sqrFiltLength, filters_per_group_temp, createBistream);
    }
    else
    {
      chosenPred = 1;
      bit_ct += lengthPredFlags(force0, chosenPred, codedVarBins, filters_per_group, createBistream);
      bit_ct += xcodeFilterCoeff(diffFilterCoeffQuant, fl, sqrFiltLength, filters_per_group_temp, createBistream);
    }
  }
  ALFp->forceCoeff0 = 1;
  ALFp->predMethod = chosenPred;
  ALFp->filters_per_group_diff = filters_per_group_diff;
  ALFp->filters_per_group = filters_per_group;
  ALFp->num_coeff = sqrFiltLength;
#if WIENER_3_INPUT
  ALFp->filter_precision[0]=prec_rec;
  ALFp->filter_precision[1]=prec_pred;
  ALFp->filter_precision[2]=prec_resi;  
  ALFp->num_coeff_pred = sqrFiltLength_pred;
  ALFp->num_coeff_resi = sqrFiltLength_resi;
  if (fl == 2) ALFp->realfiltNo=2;
  else if (fl == 3) ALFp->realfiltNo=1;
  else ALFp->realfiltNo=0;
#else
  if (ALFp->num_coeff == 8) ALFp->realfiltNo=2;
  else if (ALFp->num_coeff == 14) ALFp->realfiltNo=1;
  else ALFp->realfiltNo=0;  
#endif  
  
#if WIENER_3_INPUT
  if      (ALFp->num_coeff_pred == 1) ALFp->tap_pred = 1;
  else if (ALFp->num_coeff_pred == 3) ALFp->tap_pred = 3;
  else if (ALFp->num_coeff_pred == 7) ALFp->tap_pred = 5;
  else if (ALFp->num_coeff_pred ==13) ALFp->tap_pred = 7;
  else if (ALFp->num_coeff_pred ==21) ALFp->tap_pred = 9;

  if      (ALFp->num_coeff_resi == 1) ALFp->tap_resi = 1;
  else if (ALFp->num_coeff_resi == 3) ALFp->tap_resi = 3;
  else if (ALFp->num_coeff_resi == 7) ALFp->tap_resi = 5;
  else if (ALFp->num_coeff_resi ==13) ALFp->tap_resi = 7;
  else if (ALFp->num_coeff_resi ==21) ALFp->tap_resi = 9;
#endif
  
  for(ind = 0; ind < filters_per_group; ++ind)
  {		
    ALFp->codedVarBins[ind] = codedVarBins[ind];
  }
  for(ind = 0; ind < filters_per_group_diff; ++ind)
  {		
	for(i = 0; i < sqrFiltLength; i++)
	{
      if (chosenPred) ALFp->coeffmulti[ind][i] = diffFilterCoeffQuant[ind][i];
	  else ALFp->coeffmulti[ind][i] = FilterCoeffQuantTemp[ind][i];
	}
  }
  m_pcDummyEntropyCoder->codeFiltCountBit(ALFp, &Newbit_ct);

  return ((Int)Newbit_ct);
}

//filtNo==-1/realfiltNo, noFilters=filters_per_frames, realfiltNo=filtNo
Int TEncAdaptiveLoopFilter::xcodeAuxInfo(int filtNo, int noFilters, int varIndTab[NO_VAR_BINS], int frNo, int createBitstream,int realfiltNo, ALFParam* ALFp)
{
  int i, filterPattern[NO_VAR_BINS], startSecondFilter=0, bitCt=0, codePrediction;
  Int64 NewbitCt;

  codePrediction = 0;

  //send realfiltNo (tap related)
  ALFp->realfiltNo = realfiltNo;
  ALFp->filtNo = filtNo;
  
  if(filtNo >= 0)
  {
    // decide startSecondFilter and filterPattern
	if(noFilters > 1)
    {
      memset(filterPattern, 0, NO_VAR_BINS * sizeof(int)); 
      for(i = 1; i < NO_VAR_BINS; ++i)
	  {
        if(varIndTab[i] != varIndTab[i-1])
        {
          filterPattern[i] = 1;
          startSecondFilter = i;
        }
	  }
	  memcpy (ALFp->filterPattern, filterPattern, NO_VAR_BINS * sizeof(int));
	  ALFp->startSecondFilter = startSecondFilter;
    }

	//send noFilters (filters_per_frame)
	//0: filters_per_frame = 1
    //1: filters_per_frame = 2
    //2: filters_per_frame > 2 (exact number from filterPattern)

    ALFp->noFilters = min(noFilters-1,2);
	if (noFilters<=0) printf("error\n");
  }
  m_pcDummyEntropyCoder->codeAuxCountBit(ALFp, &NewbitCt);
  bitCt = (int) NewbitCt;
  return(bitCt);
}

Int   TEncAdaptiveLoopFilter::xcodeFilterCoeff(int **pDiffQFilterCoeffIntPP, int fl, int sqrFiltLength, 
                    int filters_per_group, int createBitstream)
{
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal, len = 0,
    *pDepthInt=NULL, kMinTab[MAX_SQR_FILT_LENGTH], bitsCoeffScan[MAX_SCAN_VAL][MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;
#if WIENER_3_INPUT
  pDepthInt = depth;
#else
  pDepthInt = pDepthIntTab[fl-2];
#endif
  maxScanVal = 0;
  for(i = 0; i < sqrFiltLength; i++)
    maxScanVal = max(maxScanVal, pDepthInt[i]);

  // vlc for all
  memset(bitsCoeffScan, 0, MAX_SCAN_VAL * MAX_EXP_GOLOMB * sizeof(int));
  for(ind=0; ind<filters_per_group; ++ind){	
    for(i = 0; i < sqrFiltLength; i++){	     
      scanPos=pDepthInt[i]-1;
      coeffVal=abs(pDiffQFilterCoeffIntPP[ind][i]);
      for (k=1; k<15; k++){
        bitsCoeffScan[scanPos][k]+=lengthGolomb(coeffVal, k);
      }
    }
  }

  minBitsKStart = 0;
  minKStart = -1;
  for(k = 1; k < 8; k++)
  { 
    bitsKStart = 0; 
    kStart = k;
    for(scanPos = 0; scanPos < maxScanVal; scanPos++)
    {
      kMin = kStart; 
      minBits = bitsCoeffScan[scanPos][kMin];

      if(bitsCoeffScan[scanPos][kStart+1] < minBits)
      {
        kMin = kStart + 1; 
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if((bitsKStart < minBitsKStart) || (k == 1))
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart; 
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
    kMin = kStart; 
    minBits = bitsCoeffScan[scanPos][kMin];

    if(bitsCoeffScan[scanPos][kStart+1] < minBits)
    {
      kMin = kStart + 1; 
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  // Coding parameters
//  len += lengthFilterCodingParams(minKStart, maxScanVal, kMinTab, createBitstream);
  len += (3 + maxScanVal);

  // Filter coefficients
  len += lengthFilterCoeffs(sqrFiltLength, filters_per_group, pDepthInt, pDiffQFilterCoeffIntPP, 
    kMinTab, createBitstream);

  return len;
}

Int TEncAdaptiveLoopFilter::lengthGolomb(int coeffVal, int k)
{
  int m = 2 << (k - 1);
  int q = coeffVal / m;
  if(coeffVal != 0)
    return(q + 2 + k);
  else
    return(q + 1 + k);
}

Int TEncAdaptiveLoopFilter::lengthPredFlags(int force0, int predMethod, int codedVarBins[NO_VAR_BINS], 
                   int filters_per_group, int createBitstream)
{
  int bit_cnt = 0;

  if(force0)
    bit_cnt = 2 + filters_per_group;
  else
    bit_cnt = 2;
  return bit_cnt;
  
}
//important
Int TEncAdaptiveLoopFilter::lengthFilterCoeffs(int sqrFiltLength, int filters_per_group, int pDepthInt[], 
                      int **FilterCoeff, int kMinTab[], int createBitstream)
{
  int ind, scanPos, i;
  int bit_cnt = 0;

  for(ind = 0; ind < filters_per_group; ++ind)
  {
    for(i = 0; i < sqrFiltLength; i++)
    {	
      scanPos = pDepthInt[i] - 1;
      bit_cnt += lengthGolomb(abs(FilterCoeff[ind][i]), kMinTab[scanPos]);
    }
  }
  return bit_cnt;
}

#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xEncALFLuma_qc ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost )
#else    
Void   TEncAdaptiveLoopFilter::xEncALFLuma_qc ( TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost )
#endif
{
//pcPicDec: extended decoded
//pcPicRest: original decoded: filtered signal will be stored

  UInt64  uiRate;
  UInt64  uiDist;
  Double dCost;
  Int    Height = pcPicOrg->getHeight();
  Int    Width = pcPicOrg->getWidth();
  Int    LumaStride = pcPicOrg->getStride();
#if ALF_MEM_PATCH
  imgpel* pOrg = (imgpel*) pcPicOrg->getLumaAddr();
  imgpel* pRest = (imgpel*) pcPicRest->getLumaAddr();
  imgpel* pDec = (imgpel*) pcPicDec->getLumaAddr();
#if WIENER_3_INPUT
  imgpel* pPred = (imgpel*) pcPicPred->getLumaAddr();
  imgpel* pResi = (imgpel*) pcPicResi->getLumaAddr();
#endif
#else
  Pel* pOrg = pcPicOrg->getLumaAddr();
  Pel* pRest = pcPicRest->getLumaAddr();
#if WIENER_3_INPUT
  Pel* pResi = pcPicResi->getLumaAddr();
  Pel* pPred = pcPicPred->getLumaAddr();
#endif
#endif

  Int tap               = ALF_MIN_NUM_TAP;
  m_pcTempAlfParam->tap = tap;
#if WIENER_3_INPUT
  m_pcTempAlfParam->filter_precision[0] = 5;
  m_pcTempAlfParam->filter_precision[1] = 5;
  m_pcTempAlfParam->filter_precision[2] = 5;
  m_pcTempAlfParam->tap_pred = ALF_MIN_NUM_TAP_PQ;
  m_pcTempAlfParam->tap_resi = ALF_MIN_NUM_TAP_PQ;
  m_pcTempAlfParam->num_coeff = (Int)(tap*tap/4 + 2 + m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 +  m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1);
  m_pcTempAlfParam->num_coeff_pred = (Int)(m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1);
  m_pcTempAlfParam->num_coeff_resi = (Int)(m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1);
#else
  m_pcTempAlfParam->num_coeff = (Int)tap*tap/4 + 2; 
#endif


#if ALF_MEM_PATCH
  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
    { 
      maskImg[i][j] = 1;
    }
  calcVar(varImg, pDec, 9/2, VAR_SIZE, Height, Width, LumaStride);
#if WIENER_3_INPUT
  xFirstFilteringFrameLuma(pOrg, pDec, pRest, pResi, pPred, m_pcTempAlfParam, m_pcTempAlfParam->tap, LumaStride); 
#else  
  xFirstFilteringFrameLuma(pOrg, pDec, pRest, m_pcTempAlfParam, m_pcTempAlfParam->tap, LumaStride); 
#endif
#else
  //move pcPicOrg and pcPicDec and pcPicRest to imgY_org and imgY_ext and imgY_rest 
  for (Int i=0; i<Height; i++)
	for (Int j=0; j<Width; j++)
    {
      imgY_org[i][j]=pOrg[j + i*LumaStride];
      imgY_rest[i][j]=pRest[j + i*LumaStride];
#if WIENER_3_INPUT
      imgY_resi[i][j]=(pResi[j + i*LumaStride]);
      imgY_pred[i][j]=(pPred[j + i*LumaStride]);
#endif      
      maskImg[i][j] = 1;
    }
  padImage(imgY_rest, imgY_ext, 4, Height, Width);
#if WIENER_3_INPUT
  padImage(imgY_pred, imgY_pext, 4, Height, Width);
  padImage(imgY_resi, imgY_rext, 4, Height, Width);
#endif
  
  calcVar(varImg, imgY_ext, 9/2, VAR_SIZE, Height, Width);

#if WIENER_3_INPUT
  xFirstFilteringFrameLuma(imgY_org, imgY_ext, imgY_rest, imgY_rext, imgY_pext, m_pcTempAlfParam, m_pcTempAlfParam->tap);
#else  
  xFirstFilteringFrameLuma(imgY_org, imgY_ext, imgY_rest, m_pcTempAlfParam, m_pcTempAlfParam->tap); 
#endif

  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
	{
	  pRest[j + i*LumaStride]=imgY_rest[i][j];
	}
#endif

  xCalcRDCost(pcPicOrg, pcPicRest, m_pcTempAlfParam, uiRate, uiDist, dCost); // change this function final coding 

  if( dCost < rdMinCost)
  {
    ruiMinRate = uiRate;
    ruiMinDist = uiDist;
    rdMinCost = dCost;
    copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam); 
  }  
}
#if ALF_MEM_PATCH
#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xFirstFilteringFrameLuma(imgpel* ImgOrg, imgpel* ImgDec, imgpel* ImgRest, imgpel* ImgResi, imgpel* ImgPred, ALFParam* ALFp, Int tap, Int Stride)
{
  xstoreInBlockMatrix(ImgOrg, ImgDec, ImgResi, ImgPred, tap, ALFp->tap_pred, ALFp->tap_resi, Stride);
  xFilteringFrameLuma_qc(ImgOrg, ImgDec, ImgRest, ImgResi, ImgPred, ALFp, tap, Stride);  
}
#else
Void   TEncAdaptiveLoopFilter::xFirstFilteringFrameLuma(imgpel* ImgOrg, imgpel* ImgDec, imgpel* ImgRest, ALFParam* ALFp, Int tap, Int Stride)
{
  xstoreInBlockMatrix(ImgOrg, ImgDec, tap, Stride);
  xFilteringFrameLuma_qc(ImgOrg, ImgDec, ImgRest, ALFp, tap, Stride);
}
#endif

#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xstoreInBlockMatrix(imgpel* ImgOrg, imgpel* ImgDec, imgpel* ImgResi, imgpel* ImgPred, Int tap, Int tap_pred, Int tap_resi, Int Stride)
#else    
Void   TEncAdaptiveLoopFilter::xstoreInBlockMatrix(imgpel* ImgOrg, imgpel* ImgDec, Int tap, Int Stride)
#endif
{
  Int i,j,k,l,varInd,ii,jj;
  Int x, y;
  Int fl =tap/2;
#if WIENER_3_INPUT
  Int fl_pred =(tap_pred-1)/2;
  Int fl_resi =(tap_resi-1)/2;
  Int resi_offset, pred_offset;
  Int sqrFiltLength=(((tap*tap)/4 + 1) + 1) + ((tap_pred*tap_pred)/4 + 1) + ((tap_resi*tap_resi)/4 + 1);
#else    
  Int sqrFiltLength=(((tap*tap)/4 + 1) + 1);
#endif
  Int fl2=9/2; //extended size at each side of the frame
  Int ELocal[MAX_SQR_FILT_LENGTH];
  Int yLocal;
  Int *p_pattern;
#if WIENER_3_INPUT
  Int *p_pattern_pred;
  Int *p_pattern_resi;
#endif
  Int filtNo =2; 
  double **E,*yy;
  Int count_valid=0;
  if (tap==9)
    filtNo =0;
  else if (tap==7)
    filtNo =1;

  p_pattern= patternTab[filtNo];
#if WIENER_3_INPUT
  p_pattern_pred= patternTab_pr[fl_pred];
  p_pattern_resi= patternTab_pr[fl_resi];
#endif

  memset( pixAcc, 0,sizeof(double)*NO_VAR_BINS);
  for (varInd=0; varInd<NO_VAR_BINS; varInd++)
  {
#if WIENER_3_INPUT
    memset(yGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#else
    memset(yGlobalSym[filtNo][varInd],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#endif    
    for (k=0; k<sqrFiltLength; k++)
    {
#if WIENER_3_INPUT
      memset(EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd][k],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#else
      memset(EGlobalSym[filtNo][varInd][k],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#endif      
    }
  }
  for (i = fl2; i < im_height+fl2; i++)
  {
	for (j = fl2; j < im_width+fl2; j++)
	{	  
	  if (maskImg[i-fl2][j-fl2] == 1)
	  {
		count_valid++;
	  }
	}
  }

  if (1)
  {
    Int j;
	for (i=0,y=fl2; i<im_height; i++,y++)
	{
	  for (j=0,x=fl2; j<im_width; j++,x++)
	  {
		if (maskImg[i][j] == 0 && count_valid > 0)
		{

		}
		else
		{
		  varInd=min(varImg[i][j], NO_VAR_BINS-1);
		  k=0; 
		  memset(ELocal, 0, sqrFiltLength*sizeof(int));
		  for (ii=-fl; ii<0; ii++)
		  {
			for (jj=-fl-ii; jj<=fl+ii; jj++)
			{  
			  ELocal[p_pattern[k++]]+=(ImgDec[(i+ii)*Stride + (j+jj)]+ImgDec[(i-ii)*Stride + (j-jj)]);
			}
		  }
	  for (jj=-fl; jj<0; jj++)
  	    ELocal[p_pattern[k++]]+=(ImgDec[(i)*Stride + (j+jj)]+ImgDec[(i)*Stride + (j-jj)]);
	  ELocal[p_pattern[k++]]+=ImgDec[(i)*Stride + (j)];
#if WIENER_3_INPUT
         // for residual
          resi_offset = k;
          k=0;
          for (ii=-fl_resi; ii<0; ii++)
          {
            for (jj=-fl_resi-ii; jj<=fl_resi+ii; jj++)
            {
              ELocal[p_pattern_resi[k++] + resi_offset]+=(int)(ImgResi[(i+ii)*Stride + (j+jj)]+ImgResi[(i-ii)*Stride + (j-jj)])-(Int)(2*g_uiIBDI_MAX_Q);
            }
          }
          for (jj=-fl_resi; jj<0; jj++)
            ELocal[p_pattern_resi[k++] + resi_offset]+=(Int)(ImgResi[(i)*Stride + (j+jj)]+ImgResi[(i)*Stride + (j-jj)])-(Int)(2*g_uiIBDI_MAX_Q);
          ELocal[p_pattern_resi[k++] + resi_offset]+=(Int)ImgResi[(i)*Stride + (j)]-(Int)g_uiIBDI_MAX_Q;

          // for prediction
          pred_offset = k + resi_offset;
          k=0;
          for (ii=-fl_pred; ii<0; ii++)
          {
            for (jj=-fl_pred-ii; jj<=fl_pred+ii; jj++)
            {
              ELocal[p_pattern_pred[k++] + pred_offset]+=(ImgPred[(i+ii)*Stride + (j+jj)]+ImgPred[(i-ii)*Stride + (j-jj)]);
            }
          }
          for (jj=-fl_pred; jj<0; jj++)
            ELocal[p_pattern_pred[k++] + pred_offset]+=(ImgPred[(i)*Stride + (j+jj)]+ImgPred[(i)*Stride + (j-jj)]);
          ELocal[p_pattern_pred[k++] + pred_offset]+=ImgPred[(i)*Stride + (j)];          
#endif                    
    	  ELocal[sqrFiltLength-1]=1;
  	  yLocal=ImgOrg[(i)*Stride + (j)];

 	  pixAcc[varInd]+=(yLocal*yLocal);
#if WIENER_3_INPUT
          E= EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
          yy= yGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
#else             
	  E= EGlobalSym[filtNo][varInd];
	  yy= yGlobalSym[filtNo][varInd];
#endif
	  for (k=0; k<sqrFiltLength; k++)
	  {
	    for (l=k; l<sqrFiltLength; l++)
				E[k][l]+=(double)(ELocal[k]*ELocal[l]);
			  yy[k]+=(double)(ELocal[k]*yLocal);
		  }
		}
	  }
	}
  }
  // Matrix EGlobalSeq is symmetric, only part of it is calculated
  for (varInd=0; varInd<NO_VAR_BINS; varInd++)
  {
#if WIENER_3_INPUT
    double **pE = EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
#else 
    double **pE = EGlobalSym[filtNo][varInd];
#endif    
    for (k=1; k<sqrFiltLength; k++)
    {
      for (l=0; l<k; l++)
	  {
        pE[k][l]=pE[l][k];
      }
    }
  }
}

#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xFilteringFrameLuma_qc(imgpel* ImgOrg, imgpel* imgY_pad, imgpel* ImgFilt, imgpel* ImgResi, imgpel* ImgPred, ALFParam* ALFp, Int tap, Int Stride)
#else
Void   TEncAdaptiveLoopFilter::xFilteringFrameLuma_qc(imgpel* ImgOrg, imgpel* imgY_pad, imgpel* ImgFilt, ALFParam* ALFp, Int tap, Int Stride)
#endif
{
  int  filtNo,filters_per_fr;
#if WIENER_3_INPUT
  int  filtNo_pred;
  int  filtNo_resi;
  int  precision_best[3];
  Int64 iRate;
  Int64 iDist;
  Double dCost=0;
  Double dCost_min=0;
  Int start=0;
#endif
  static double **ySym, ***ESym;
  int lambda_val = (Int) m_dLambdaLuma;
  
  lambda_val = lambda_val * (1<<(2*g_uiBitIncrement));
  if (tap==9)
    filtNo =0;
  else if (tap==7)
    filtNo =1;
  else
    filtNo=2;
#if WIENER_3_INPUT
  if (ALFp->tap_pred==9)
    filtNo_pred = 4;
  else if (ALFp->tap_pred==7)
    filtNo_pred = 3;
  else if (ALFp->tap_pred==5)
    filtNo_pred = 2;
  else if (ALFp->tap_pred==3)
    filtNo_pred = 1;
  else
    filtNo_pred = 0;
  
  if (ALFp->tap_resi==9)
    filtNo_resi = 4;
  else if (ALFp->tap_resi==7)
    filtNo_resi = 3;
  else if (ALFp->tap_resi==5)
    filtNo_resi = 2;
  else if (ALFp->tap_resi==3)
    filtNo_resi = 1;
  else
    filtNo_resi = 0;
     
  ESym=EGlobalSym[filtNo+filtNo_pred*(NO_TEST_FILT+2)+filtNo_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)];
  ySym=yGlobalSym[filtNo+filtNo_pred*(NO_TEST_FILT+2)+filtNo_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)];

#if WIENER_3_INPUT_FAST
  if (adapt_precision==1)
  {
#endif 
  for (Int ii=0;ii<FILTER_PRECISION_TABLE_NUMBER;ii++)
  {
    ALFp->filter_precision[0]=ii;
    ALFp->filter_precision[1]=ii;
    ALFp->filter_precision[2]=ii;
    
    xfindBestFilterVarPred(ySym, ESym, pixAcc, filterCoeffSym, g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, &filters_per_fr, 
                         varIndTab, NULL, varImg, maskImg, NULL, lambda_val, 
                         ALFp->filter_precision[0],
                         ALFp->filter_precision[1],
                         ALFp->filter_precision[2],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
    
    // g_filterCoeffPrevSelected = g_filterCoeffSym
    xcalcPredFilterCoeff(filtNo, filtNo_pred, filtNo_resi);
    
    //filter the frame with g_filterCoeffPrevSelected
    xfilterFrame_en(imgY_pad, ImgFilt, ImgResi, ImgPred, filtNo, filtNo_resi, filtNo_pred, 
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[2]],
                           Stride);
    
    xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, varIndTab, filters_per_fr, 0, ALFp);
    
    xCalcRDCost_precision(ImgOrg, ImgFilt, im_width, im_height, ALFp, iRate, iDist, dCost, Stride);
    if (dCost<dCost_min || start==0 )
    {
      start++;
      dCost_min=dCost;
      precision_best[0]=ALFp->filter_precision[0];
      precision_best[1]=ALFp->filter_precision[1];
      precision_best[2]=ALFp->filter_precision[2];
    }
  }
#if WIENER_3_INPUT_FAST
  }
  else
  {
    precision_best[0]=5;
    precision_best[1]=5;
    precision_best[2]=5;
  }  
#endif
//Take best
  ALFp->filter_precision[0]=precision_best[0];
  ALFp->filter_precision[1]=precision_best[1];
  ALFp->filter_precision[2]=precision_best[2];
  

  xfindBestFilterVarPred(ySym, ESym, pixAcc, filterCoeffSym, g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, &filters_per_fr, 
                        varIndTab, NULL, varImg, maskImg, NULL, lambda_val, 
                        ALFp->filter_precision[0],
                        ALFp->filter_precision[1],
                        ALFp->filter_precision[2],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
  
  // g_filterCoeffPrevSelected = g_filterCoeffSym
  xcalcPredFilterCoeff(filtNo, filtNo_pred, filtNo_resi);
  
  //filter the frame with g_filterCoeffPrevSelected
  xfilterFrame_en(imgY_pad, ImgFilt, ImgResi, ImgPred, filtNo, filtNo_resi, filtNo_pred, 
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[2]], 
                          Stride);
  
  xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, varIndTab, filters_per_fr, 0, ALFp);  
#else  
  
  ESym=EGlobalSym[filtNo];
  ySym=yGlobalSym[filtNo];

  xfindBestFilterVarPred(ySym, ESym, pixAcc, filterCoeffSym, g_filterCoeffSymQuant, filtNo, &filters_per_fr,
  			varIndTab, NULL, varImg, maskImg, NULL, lambda_val);

	// g_filterCoeffPrevSelected = g_filterCoeffSym
    xcalcPredFilterCoeff(filtNo);

	 //filter the frame with g_filterCoeffPrevSelected
  xfilterFrame_en(imgY_pad, ImgFilt, filtNo, Stride);

  xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, varIndTab, filters_per_fr,0, ALFp);
#endif
}

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xfilterFrame_en(imgpel* ImgDec, imgpel* ImgRest, imgpel* ImgResi, imgpel* ImgPred, int filtNo, int filtNo_resi, int filtNo_pred, int bits_rec, int bits_pred, int bits_resi, int Stride)
#else    
Void TEncAdaptiveLoopFilter::xfilterFrame_en(imgpel* ImgDec, imgpel* ImgRest,int filtNo, int Stride)
#endif
{
  int i,j,ii,jj,y,x;
  int  *pattern;
  int fl, fl_temp, sqrFiltLength;
  int pixelInt;
#if WIENER_3_INPUT
  int offset     = (1<<(bits_rec - 1));
  int shift_pred = (1<<(bits_rec - bits_pred));
  int shift_resi = (1<<(bits_rec - bits_resi));
  int length = 0;
  int flTab_pr[5] = {0,1,2,3,4};
#else  
  int offset = (1<<(NUM_BITS - 2));
#endif
  pattern=patternTab_filt[filtNo];
  fl_temp=flTab[filtNo];
  sqrFiltLength=MAX_SQR_FILT_LENGTH;  fl=FILTER_LENGTH/2;

  for (y=0, i = fl; i < im_height+fl; i++, y++)
  {
    for (x=0, j = fl; j < im_width+fl; j++, x++)
    {
      int varInd=varImg[i-fl][j-fl];
      imgpel *im1,*im2;
      int *coef = filterCoeffPrevSelected[varInd];
      pattern=patternTab_filt[filtNo];
      pixelInt= filterCoeffPrevSelected[varInd][sqrFiltLength-1];     
#if WIENER_3_INPUT
      fl_temp=flTab[filtNo];
#endif      
      for (ii=-fl_temp; ii<0; ii++)
      {
        im1= &(ImgDec[(y+ii)*Stride + x-fl_temp-ii]);
        im2= &(ImgDec[(y-ii)*Stride + x+fl_temp+ii]);
        for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
          pixelInt+=((*im1+ *im2)*coef[*(pattern++)]);
      }
      im1= &(ImgDec[y*Stride + x-fl_temp]);
      im2= &(ImgDec[y*Stride + x+fl_temp]);	
      for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
        pixelInt+=((*im1+ *im2)*coef[*(pattern++)]);
      pixelInt+=(ImgDec[y*Stride + x]*coef[*(pattern++)]);
#if WIENER_3_INPUT
     // add Resi & Pred
      length = (FILTER_LENGTH * FILTER_LENGTH)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_resi];
      fl_temp=flTab_pr[filtNo_resi];
      for (ii=-fl_temp; ii<0; ii++)
      {
        im1= &(ImgResi[(y+ii)*Stride + x-fl_temp-ii]);
        im2= &(ImgResi[(y-ii)*Stride + x+fl_temp+ii]);
        for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
          pixelInt+=(((Int)(*im1+ *im2)-(Int)(g_uiIBDI_MAX_Q*2))*coef[(*(pattern++))+length]*shift_resi);
      }
      im1= &(ImgResi[y*Stride + x-fl_temp]);
      im2= &(ImgResi[y*Stride + x+fl_temp]);
      for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
        pixelInt+=(((Int)(*im1+ *im2)-(Int)(g_uiIBDI_MAX_Q*2))*coef[(*(pattern++))+length]*shift_resi);
      pixelInt+=(((Int)ImgResi[y*Stride + x]-(Int)g_uiIBDI_MAX_Q)*coef[(*(pattern++))+length]*shift_resi);

      length += (FILTER_LENGTH_RESI * FILTER_LENGTH_RESI)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_pred];
      fl_temp=flTab_pr[filtNo_pred];
      for (ii=-fl_temp; ii<0; ii++)
      {
        im1= &(ImgPred[(y+ii)*Stride + x-fl_temp-ii]);
        im2= &(ImgPred[(y-ii)*Stride + x+fl_temp+ii]);
        for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
          pixelInt+=((*im1+ *im2)*coef[(*(pattern++))+length]*shift_pred);
      }
      im1= &(ImgPred[y*Stride + x-fl_temp]);
      im2= &(ImgPred[y*Stride + x+fl_temp]);     
      for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
        pixelInt+=((*im1+ *im2)*coef[(*(pattern++))+length]*shift_pred);
      pixelInt+=(ImgPred[y*Stride + x]*coef[(*(pattern++))+length]*shift_pred);
      
      pixelInt=(int)((pixelInt+offset) >> bits_rec );
#else      
      pixelInt=(int)((pixelInt+offset) >> (NUM_BITS - 1));
#endif      
      ImgRest[y*Stride + x] = Clip3(0, g_uiIBDI_MAX, pixelInt);
    }
  }
}
#endif

#if !ALF_MEM_PATCH
#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xFirstFilteringFrameLuma(imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgRest, imgpel** ImgResi, imgpel** ImgPred, ALFParam* ALFp, Int tap)
{
  xstoreInBlockMatrix(ImgOrg, ImgDec, ImgResi, ImgPred, tap, ALFp->tap_pred, ALFp->tap_resi);
  xFilteringFrameLuma_qc(ImgOrg, ImgDec, ImgRest, ImgResi, ImgPred, ALFp, tap);
}
#else
Void   TEncAdaptiveLoopFilter::xFirstFilteringFrameLuma(imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgRest, ALFParam* ALFp, Int tap)
{
  xstoreInBlockMatrix(ImgOrg, ImgDec, tap);
  xFilteringFrameLuma_qc(ImgOrg, ImgDec, ImgRest, ALFp, tap);
}
#endif
#endif

#if !ALF_MEM_PATCH
#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xFilteringFrameLuma_qc(imgpel** ImgOrg, imgpel** imgY_pad, imgpel** ImgFilt, imgpel** ImgResi, imgpel** ImgPred, ALFParam* ALFp, Int tap)
#else
Void   TEncAdaptiveLoopFilter::xFilteringFrameLuma_qc(imgpel** ImgOrg, imgpel** imgY_pad, imgpel** ImgFilt, ALFParam* ALFp, Int tap)
#endif
{
  int  filtNo,filters_per_fr;
#if WIENER_3_INPUT
  int  filtNo_pred;
  int  filtNo_resi;
  int  precision_best[3];
  Int64 iRate;
  Int64 iDist;
  Double dCost=0;
  Double dCost_min=0;
  Int start=0;
#endif  
  static double **ySym, ***ESym;
  int lambda_val = (Int) m_dLambdaLuma;
  lambda_val = lambda_val * (1<<(2*g_uiBitIncrement));
  if (tap==9)
    filtNo =0;
  else if (tap==7)
    filtNo =1;
  else
    filtNo=2;
#if WIENER_3_INPUT
  if (ALFp->tap_pred==9)
    filtNo_pred = 4;
  else if (ALFp->tap_pred==7)
    filtNo_pred = 3;
  else if (ALFp->tap_pred==5)
    filtNo_pred = 2;
  else if (ALFp->tap_pred==3)
    filtNo_pred = 1;
  else
    filtNo_pred = 0;
  
  if (ALFp->tap_resi==9)
    filtNo_resi = 4;
  else if (ALFp->tap_resi==7)
    filtNo_resi = 3;
  else if (ALFp->tap_resi==5)
    filtNo_resi = 2;
  else if (ALFp->tap_resi==3)
    filtNo_resi = 1;
  else
    filtNo_resi = 0;
  
  ESym=EGlobalSym[filtNo+filtNo_pred*(NO_TEST_FILT+2)+filtNo_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)];
  ySym=yGlobalSym[filtNo+filtNo_pred*(NO_TEST_FILT+2)+filtNo_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)];

#if WIENER_3_INPUT_FAST
  if (adapt_precision==1)
  {
#endif
  for (Int ii=0;ii<FILTER_PRECISION_TABLE_NUMBER;ii++)
  {
    ALFp->filter_precision[0]=ii;
    ALFp->filter_precision[1]=ii;
    ALFp->filter_precision[2]=ii;
    
    xfindBestFilterVarPred(ySym, ESym, pixAcc, g_filterCoeffSym, g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, &filters_per_fr, 
                         varIndTab, imgY_rec, varImg, maskImg, imgY_pad, lambda_val, 
                         ALFp->filter_precision[0],
                         ALFp->filter_precision[1],
                         ALFp->filter_precision[2],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                         ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
    
    // g_filterCoeffPrevSelected = g_filterCoeffSym
    xcalcPredFilterCoeff(filtNo, filtNo_pred, filtNo_resi);
    
    //filter the frame with g_filterCoeffPrevSelected
    xfilterFrame_en(imgY_pad, ImgFilt, ImgResi, ImgPred, filtNo, filtNo_resi, filtNo_pred, 
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                           ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
    
    xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, varIndTab, filters_per_fr, 0, ALFp);
    
    xCalcRDCost_precision(ImgOrg, ImgFilt, im_width, im_height, ALFp, iRate, iDist, dCost);
    if (dCost<dCost_min || start==0 )
    {
      start++;
      dCost_min=dCost;
      precision_best[0]=ALFp->filter_precision[0];
      precision_best[1]=ALFp->filter_precision[1];
      precision_best[2]=ALFp->filter_precision[2];
    }
  }
#if WIENER_3_INPUT_FAST
  }
  else
  {
    precision_best[0]=5;
    precision_best[1]=5;
    precision_best[2]=5;
  }  
#endif
//Take best
  ALFp->filter_precision[0]=precision_best[0];
  ALFp->filter_precision[1]=precision_best[1];
  ALFp->filter_precision[2]=precision_best[2];
  

  xfindBestFilterVarPred(ySym, ESym, pixAcc, g_filterCoeffSym, g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, &filters_per_fr, 
                        varIndTab, NULL, varImg, maskImg, NULL, lambda_val, 
                        ALFp->filter_precision[0],
                        ALFp->filter_precision[1],
                        ALFp->filter_precision[2],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                        ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
  
  // g_filterCoeffPrevSelected = g_filterCoeffSym
  xcalcPredFilterCoeff(filtNo, filtNo_pred, filtNo_resi);
  
  //filter the frame with g_filterCoeffPrevSelected
  xfilterFrame_en(imgY_pad, ImgFilt, ImgResi, ImgPred, filtNo, filtNo_resi, filtNo_pred, 
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[0]],
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[1]],
                          ALFp->filter_precision_table_shift[ALFp->filter_precision[2]]);
  
  xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, filtNo_pred, filtNo_resi, varIndTab, filters_per_fr, 0, ALFp);  
#else
  ESym=EGlobalSym[filtNo];  
  ySym=yGlobalSym[filtNo];

  xfindBestFilterVarPred(ySym, ESym, pixAcc, g_filterCoeffSym, g_filterCoeffSymQuant, filtNo, &filters_per_fr,
  			  varIndTab, NULL, varImg, maskImg, NULL, lambda_val);

  // g_filterCoeffPrevSelected = g_filterCoeffSym
  xcalcPredFilterCoeff(filtNo);

  xfilterFrame_en(imgY_pad, ImgFilt, filtNo);

  xcodeFiltCoeff(g_filterCoeffSymQuant, filtNo, varIndTab, filters_per_fr,0, ALFp);
#endif
}

#if WIENER_3_INPUT
Void   TEncAdaptiveLoopFilter::xstoreInBlockMatrix(imgpel** ImgOrg, imgpel** ImgDec, imgpel** ImgResi, imgpel** ImgPred, Int tap, Int tap_pred, Int tap_resi)
#else    
Void   TEncAdaptiveLoopFilter::xstoreInBlockMatrix(imgpel** ImgOrg, imgpel** ImgDec, Int tap)
#endif
{
  Int i,j,k,l,varInd,ii,jj;
  Int x, y;
  Int fl =tap/2;
#if WIENER_3_INPUT
  Int fl_pred =(tap_pred-1)/2;
  Int fl_resi =(tap_resi-1)/2;
  Int resi_offset, pred_offset;
  Int sqrFiltLength=(((tap*tap)/4 + 1) + 1) + ((tap_pred*tap_pred)/4 + 1) + ((tap_resi*tap_resi)/4 + 1);
#else
  Int sqrFiltLength=(((tap*tap)/4 + 1) + 1);
#endif
  Int fl2=9/2; //extended size at each side of the frame
  Int ELocal[MAX_SQR_FILT_LENGTH];
  Int yLocal;
  Int *p_pattern;
#if WIENER_3_INPUT
  Int *p_pattern_pred;
  Int *p_pattern_resi;
#endif  
  Int filtNo =2; 
  double **E,*yy;
  Int count_valid=0;
  if (tap==9)
    filtNo =0;
  else if (tap==7)
    filtNo =1;

  p_pattern= patternTab[filtNo];
#if WIENER_3_INPUT
  p_pattern_pred= patternTab_pr[fl_pred];
  p_pattern_resi= patternTab_pr[fl_resi];
#endif
  memset( pixAcc, 0,sizeof(double)*NO_VAR_BINS);
  for (varInd=0; varInd<NO_VAR_BINS; varInd++)
  {
#if WIENER_3_INPUT
    memset(yGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#else    
    memset(yGlobalSym[filtNo][varInd],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#endif    
    for (k=0; k<sqrFiltLength; k++)
    {
#if WIENER_3_INPUT
      memset(EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd][k],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#else      
      memset(EGlobalSym[filtNo][varInd][k],0,sizeof(double)*MAX_SQR_FILT_LENGTH);
#endif
    }
  }
  for (i = fl2; i < im_height+fl2; i++)
  {
	for (j = fl2; j < im_width+fl2; j++)
	{	  
	  if (maskImg[i-fl2][j-fl2] == 1)
	  {
		count_valid++;
	  }
	}
  }
   
   

  if (1)
  {
    Int j;
	for (i=0,y=fl2; i<im_height; i++,y++)
	{
	  for (j=0,x=fl2; j<im_width; j++,x++)
	  {
		if (maskImg[i][j] == 0 && count_valid > 0)
		{

		}
		else
		{
		  varInd=min(varImg[i][j], NO_VAR_BINS-1);
		  k=0; 
		  memset(ELocal, 0, sqrFiltLength*sizeof(int));
		  for (ii=-fl; ii<0; ii++)
		  {
			for (jj=-fl-ii; jj<=fl+ii; jj++)
			{  
			  ELocal[p_pattern[k++]]+=(ImgDec[y+ii][x+jj]+ImgDec[y-ii][x-jj]);
			}
		  }
          for (jj=-fl; jj<0; jj++)
            ELocal[p_pattern[k++]]+=(ImgDec[y][x+jj]+ImgDec[y][x-jj]);
          ELocal[p_pattern[k++]]+=ImgDec[y][x];
#if WIENER_3_INPUT
         // for residual
          resi_offset = k;
          k=0;
          for (ii=-fl_resi; ii<0; ii++)
          {
            for (jj=-fl_resi-ii; jj<=fl_resi+ii; jj++)
            {
              ELocal[p_pattern_resi[k++] + resi_offset]+=(int)(ImgResi[y+ii][x+jj]+ImgResi[y-ii][x-jj])-(Int)(2*g_uiIBDI_MAX_Q);
            }
          }
          for (jj=-fl_resi; jj<0; jj++)
            ELocal[p_pattern_resi[k++] + resi_offset]+=(Int)(ImgResi[y][x+jj]+ImgResi[y][x-jj])-(Int)(2*g_uiIBDI_MAX_Q);
          ELocal[p_pattern_resi[k++] + resi_offset]+=(Int)ImgResi[y][x]-(Int)g_uiIBDI_MAX_Q;

          // for prediction
          pred_offset = k + resi_offset;
          k=0;
          for (ii=-fl_pred; ii<0; ii++)
          {
            for (jj=-fl_pred-ii; jj<=fl_pred+ii; jj++)
            {
              ELocal[p_pattern_pred[k++] + pred_offset]+=(ImgPred[y+ii][x+jj]+ImgPred[y-ii][x-jj]);
            }
          }
          for (jj=-fl_pred; jj<0; jj++)
            ELocal[p_pattern_pred[k++] + pred_offset]+=(ImgPred[y][x+jj]+ImgPred[y][x-jj]);
          ELocal[p_pattern_pred[k++] + pred_offset]+=ImgPred[y][x];          
#endif          
          ELocal[sqrFiltLength-1]=1;
          yLocal=ImgOrg[i][j];
  
          pixAcc[varInd]+=(yLocal*yLocal);
#if WIENER_3_INPUT
          E= EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
          yy= yGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
#else          
          E= EGlobalSym[filtNo][varInd];
          yy= yGlobalSym[filtNo][varInd];
#endif
          for (k=0; k<sqrFiltLength; k++)
          {
            for (l=k; l<sqrFiltLength; l++)
				E[k][l]+=(double)(ELocal[k]*ELocal[l]);
			  yy[k]+=(double)(ELocal[k]*yLocal);
		  }
		}
	  }
	}
  }
  // Matrix EGlobalSeq is symmetric, only part of it is calculated
  for (varInd=0; varInd<NO_VAR_BINS; varInd++)
  {
#if WIENER_3_INPUT
    double **pE = EGlobalSym[filtNo+fl_pred*(NO_TEST_FILT+2)+fl_resi*(NO_TEST_FILT+2)*(NO_TEST_FILT+2)][varInd];
#else 
    double **pE = EGlobalSym[filtNo][varInd];
#endif
    for (k=1; k<sqrFiltLength; k++)
    {
      for (l=0; l<k; l++)
	  {
        pE[k][l]=pE[l][k];
      }
    }
  }
}
#endif
							
#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, int **filterCoeffSym, int **filterCoeffSymQuant, int filtNo, int filtNo_pred, int filtNo_resi, int *filters_per_fr_best, int varIndTab[], imgpel **imgY_rec, imgpel **varImg, imgpel **maskImg, imgpel **imgY_pad, double lambda_val, int prec_rec, int prec_pred, int prec_resi, int shift_rec, int shift_pred, int shift_resi)
#else
Void TEncAdaptiveLoopFilter::xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, int **filterCoeffSym, int **filterCoeffSymQuant, int filtNo, int *filters_per_fr_best, int varIndTab[], imgpel **imgY_rec, imgpel **varImg, imgpel **maskImg, imgpel **imgY_pad, double lambda_val)
#endif
{
  int filters_per_fr, firstFilt, coded, forceCoeff0,
    interval[NO_VAR_BINS][2], intervalBest[NO_VAR_BINS][2];
  int i, k, varInd;
  static double ***E_temp, **y_temp, *pixAcc_temp;
  static int **FilterCoeffQuantTemp;
  double  error, lambda, lagrangian, lagrangianMin;

  int fl, sqrFiltLength;
#if WIENER_3_INPUT
  int sqrFiltLength_pred, sqrFiltLength_resi;
  int flMax=flTab[0];
#else  
  int *pattern, *patternMap, *weights;
#endif
  int numBits, coeffBits;
  double errorForce0CoeffTab[NO_VAR_BINS][2];
  int  codedVarBins[NO_VAR_BINS], createBistream /*, forceCoeff0 */;
  int  usePrevFilt[NO_VAR_BINS], usePrevFiltDefault[NO_VAR_BINS];
  static int first=0;

  for (i = 0; i < NO_VAR_BINS; i++)
    usePrevFiltDefault[i]=usePrevFilt[i]=1;
  lambda = lambda_val;
  sqrFiltLength=MAX_SQR_FILT_LENGTH;  fl=FILTER_LENGTH/2;

  if (first==0)
  {
    initMatrix3D_double(&E_temp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
    initMatrix_double(&y_temp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
    pixAcc_temp = (double *) calloc(NO_VAR_BINS, sizeof(double));
    initMatrix_int(&FilterCoeffQuantTemp, NO_VAR_BINS, MAX_SQR_FILT_LENGTH);
    first=1;
  }

#if WIENER_3_INPUT  
  sqrFiltLength=sqrFiltLengthTab[filtNo]+sqrFiltLengthTab_pr[filtNo_pred]+sqrFiltLengthTab_pr[filtNo_resi];   
  sqrFiltLength_pred=sqrFiltLengthTab_pr[filtNo_pred];
  sqrFiltLength_resi=sqrFiltLengthTab_pr[filtNo_resi];
#else
  sqrFiltLength=sqrFiltLengthTab[filtNo];   
#endif
  
  fl=flTab[filtNo];
#if WIENER_3_INPUT
  for (i=0; i<sqrFiltLengthTab[filtNo]-1; i++)
    weights[i]=weightsTab[filtNo][i];
  k=i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_resi]; i++)
    weights[k+i]=weightsTab_pr[filtNo_resi][i];
  k=k+i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_pred]; i++)
    weights[k+i]=weightsTab_pr[filtNo_pred][i];

  weights[k+i] = weightsTab[filtNo][sqrFiltLengthTab[filtNo]-1]; //DC

  for (i=0; i<sqrFiltLengthTab[filtNo]-1; i++)
    depth[i]=pDepthIntTab[2-filtNo][i];
  k=i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_resi]; i++)
    depth[k+i]=pDepthIntTab_pr[filtNo_resi][i];
  k=k+i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_pred]; i++)
    depth[k+i]=pDepthIntTab_pr[filtNo_pred][i];
  depth[k+i] = pDepthIntTab[2-filtNo][sqrFiltLengthTab[filtNo]-1]; //DC
#else
  weights=weightsTab[filtNo];
  patternMap=patternMapTab[filtNo];
  pattern=patternTab[filtNo];
#endif

  memcpy(pixAcc_temp,pixAcc,sizeof(double)*NO_VAR_BINS);
  for (varInd=0; varInd<NO_VAR_BINS; varInd++)
  {
     memcpy(y_temp[varInd],ySym[varInd],sizeof(double)*sqrFiltLength);
     for (k=0; k<sqrFiltLength; k++)
		memcpy(E_temp[varInd][k],ESym[varInd][k],sizeof(double)*sqrFiltLength);
  }
 
  // zero all variables 
  memset(varIndTab,0,sizeof(int)*NO_VAR_BINS);

  for(i = 0; i < NO_VAR_BINS; i++)
  {
	memset(filterCoeffSym[i],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
	memset(filterCoeffSymQuant[i],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
  }

  firstFilt=1;  lagrangianMin=0;
  filters_per_fr=NO_FILTERS;
  
  while(filters_per_fr>=1)
  {
    findFilterGroupingError(E_temp, y_temp, pixAcc_temp, interval, sqrFiltLength, filters_per_fr);
#if WIENER_3_INPUT    
    findFilterCoeff(E_temp, y_temp, pixAcc_temp, filterCoeffSym, filterCoeffSymQuant, interval,
      varIndTab, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, filters_per_fr, weights, shift_rec, shift_pred, shift_resi,  errorForce0CoeffTab);
    
    lagrangian=xfindBestCoeffCodMethod(codedVarBins, &forceCoeff0, prec_rec, prec_pred, prec_resi, filterCoeffSymQuant, fl, 
      sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, filters_per_fr, errorForce0CoeffTab, &error, lambda);
#else    
    findFilterCoeff(E_temp, y_temp, pixAcc_temp, filterCoeffSym, filterCoeffSymQuant, interval,
      varIndTab, sqrFiltLength, filters_per_fr, weights, numBits=NUM_BITS,  errorForce0CoeffTab);
    
    lagrangian=xfindBestCoeffCodMethod(codedVarBins, &forceCoeff0, filterCoeffSymQuant, fl, 
      sqrFiltLength, filters_per_fr, errorForce0CoeffTab, &error, lambda);
#endif    
    
    if (lagrangian<lagrangianMin || firstFilt==1)
    {
	  firstFilt=0;
	  lagrangianMin=lagrangian;

	  (*filters_per_fr_best)=filters_per_fr;
	  memcpy(intervalBest, interval, NO_VAR_BINS*2*sizeof(int));
	}
    filters_per_fr--;
  }

#if WIENER_3_INPUT    
  findFilterCoeff(E_temp, y_temp, pixAcc_temp, filterCoeffSym, filterCoeffSymQuant, intervalBest,
	varIndTab, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, (*filters_per_fr_best), weights, shift_rec, shift_pred, shift_resi,  errorForce0CoeffTab);
  
  xfindBestCoeffCodMethod(codedVarBins, &forceCoeff0, prec_rec, prec_pred, prec_resi, filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi,
        (*filters_per_fr_best), errorForce0CoeffTab, &error, lambda);
#else
  findFilterCoeff(E_temp, y_temp, pixAcc_temp, filterCoeffSym, filterCoeffSymQuant, intervalBest,
        varIndTab, sqrFiltLength, (*filters_per_fr_best), weights, numBits=NUM_BITS, errorForce0CoeffTab);
  
  xfindBestCoeffCodMethod(codedVarBins, &forceCoeff0, filterCoeffSymQuant, fl, sqrFiltLength, 
        (*filters_per_fr_best), errorForce0CoeffTab, &error, lambda);
#endif
  
  coded=1;
  if (forceCoeff0==1 && (*filters_per_fr_best)==1)
  {
	coded=0;
	coeffBits = xcodeAuxInfo(-1, (*filters_per_fr_best), varIndTab, 0, createBistream=0,filtNo, tempALFp);
  }
  else
  {
	coeffBits = xcodeAuxInfo(filtNo, (*filters_per_fr_best), varIndTab, 0, createBistream=0,filtNo, tempALFp);
  }

  if (forceCoeff0==0)
  {
#if WIENER_3_INPUT
    coeffBits += xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi,
      (*filters_per_fr_best), createBistream=0, tempALFp);
#else    
    coeffBits += xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, 
      (*filters_per_fr_best), createBistream=0, tempALFp);
#endif
  }
  else
  {
	if ((*filters_per_fr_best)==1)
	{
	  for(varInd=0; varInd<(*filters_per_fr_best); varInd++)
	  {
		memset(filterCoeffSym[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
		memset(filterCoeffSymQuant[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
	  }
    }
    else
    {
#if WIENER_3_INPUT
      coeffBits += xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi,
            (*filters_per_fr_best), codedVarBins, createBistream=0, tempALFp);
#else      
      coeffBits += xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, 
            (*filters_per_fr_best), codedVarBins, createBistream=0, tempALFp);
#endif  
      for(varInd=0; varInd<(*filters_per_fr_best); varInd++)
      {
        if (codedVarBins[varInd]==0)
		{
			memset(filterCoeffSym[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
			memset(filterCoeffSymQuant[varInd],0,sizeof(int)*MAX_SQR_FILT_LENGTH);
		}
      }
    }
  }  
}

#if !ALF_MEM_PATCH
#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xfilterFrame_en(imgpel** ImgDec, imgpel** ImgRest, imgpel** ImgResi, imgpel** ImgPred, int filtNo, int filtNo_resi, int filtNo_pred, int bits_rec, int bits_pred, int bits_resi)
#else
Void TEncAdaptiveLoopFilter::xfilterFrame_en(imgpel** ImgDec, imgpel** ImgRest,int filtNo)
#endif
{
  int i,j,ii,jj;
  int  *pattern; 
  int fl, fl_temp, sqrFiltLength;
  int pixelInt;  
#if WIENER_3_INPUT
  int offset     = (1<<(bits_rec - 1));
  int shift_pred = (1<<(bits_rec - bits_pred));
  int shift_resi = (1<<(bits_rec - bits_resi));
  int length = 0;
  int flTab_pr[5] = {0,1,2,3,4};
#else  
  int offset = (1<<(NUM_BITS - 2));
#endif
    
  pattern=patternTab_filt[filtNo];
  fl_temp=flTab[filtNo];
  sqrFiltLength=MAX_SQR_FILT_LENGTH;  fl=FILTER_LENGTH/2;

  for (i = fl; i < im_height+fl; i++)
  {
    for (j = fl; j < im_width+fl; j++)
	{
		int varInd=varImg[i-fl][j-fl];
		imgpel *im1,*im2;
		int *coef = g_filterCoeffPrevSelected[varInd];
		pattern=patternTab_filt[filtNo];
#if WIENER_3_INPUT
                fl_temp=flTab[filtNo];
#endif
		pixelInt=g_filterCoeffPrevSelected[varInd][sqrFiltLength-1]; 
		for (ii=-fl_temp; ii<0; ii++)
		{
		  im1= &(ImgDec[i+ii][j-fl_temp-ii]);
		  im2= &(ImgDec[i-ii][j+fl_temp+ii]);
		  for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
			  pixelInt+=((*im1+ *im2)*coef[*(pattern++)]);
		}
		im1= &(ImgDec[i][j-fl_temp]);
		im2= &(ImgDec[i][j+fl_temp]);	
		for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
              pixelInt+=((*im1+ *im2)*coef[*(pattern++)]);
      pixelInt+=(ImgDec[i][j]*coef[*(pattern++)]);

#if WIENER_3_INPUT
     // add Resi & Pred
      length = (FILTER_LENGTH * FILTER_LENGTH)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_resi];
      fl_temp=flTab_pr[filtNo_resi];
      for (ii=-fl_temp; ii<0; ii++)
      {
        im1= &(ImgResi[i+ii][j-fl_temp-ii]);
        im2= &(ImgResi[i-ii][j+fl_temp+ii]);
        for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
          pixelInt+=(((Int)(*im1+ *im2)-(Int)(g_uiIBDI_MAX_Q*2))*coef[(*(pattern++))+length]*shift_resi);
      }
      im1= &(ImgResi[i][j-fl_temp]);
      im2= &(ImgResi[i][j+fl_temp]);
      for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
        pixelInt+=(((Int)(*im1+ *im2)-(Int)(g_uiIBDI_MAX_Q*2))*coef[(*(pattern++))+length]*shift_resi);
      pixelInt+=(((Int)ImgResi[i][j]-(Int)g_uiIBDI_MAX_Q)*coef[(*(pattern++))+length]*shift_resi);

      length += (FILTER_LENGTH_RESI * FILTER_LENGTH_RESI)/2 + 1;
      pattern=patternTab_filt_pr[filtNo_pred];
      fl_temp=flTab_pr[filtNo_pred];
      for (ii=-fl_temp; ii<0; ii++)
      {
        im1= &(ImgPred[i+ii][j-fl_temp-ii]);
        im2= &(ImgPred[i-ii][j+fl_temp+ii]);
        for (jj=-fl_temp-ii; jj<=fl_temp+ii; jj++,im1++,im2--)
          pixelInt+=((*im1+ *im2)*coef[(*(pattern++))+length]*shift_pred);
      }
      im1= &(ImgPred[i][j-fl_temp]);
      im2= &(ImgPred[i][j+fl_temp]);
      for (jj=-fl_temp; jj<0; jj++,im1++,im2--)
        pixelInt+=((*im1+ *im2)*coef[(*(pattern++))+length]*shift_pred);
      pixelInt+=(ImgPred[i][j]*coef[(*(pattern++))+length]*shift_pred);
      
      pixelInt=(int)((pixelInt+offset) >> bits_rec );
#else      
      pixelInt=(int)((pixelInt+offset) >> (NUM_BITS - 1));
#endif
//		ImgRest[i-fl][j-fl] = Clip3(0, (1 << (g_uiBitDepth+g_uiBitIncrement))-1, pixelInt);
      ImgRest[i-fl][j-fl] = Clip3(0, g_uiIBDI_MAX, pixelInt);
    }
  }
}
#endif

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xcalcPredFilterCoeff(int filtNo, int filtNo_pred, int filtNo_resi)
{
  int *patternMap, *patternMap_pred, *patternMap_resi,varInd, i, k;

  int length = (FILTER_LENGTH*FILTER_LENGTH)/2 + 1;
  int length_pred = (FILTER_LENGTH_PRED*FILTER_LENGTH_PRED)/2 + 1;
  int length_resi = (FILTER_LENGTH_RESI*FILTER_LENGTH_RESI)/2 + 1;        
  
  if (filtNo_resi<2)
    length_resi = (3*3)/2 + 1;
  if (filtNo_pred<2)
    length_pred = (3*3)/2 + 1;
  
  patternMap=patternMapTab[filtNo];
  patternMap_resi=patternMapTab_pr[filtNo_resi];
  patternMap_pred=patternMapTab_pr[filtNo_pred];
  // filter for reconstructions
  for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
  {
    k=0;
    for(i = 0; i < length; i++)
    {
      if (patternMap[i]>0)
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i]=filterCoeffSym[varIndTab[varInd]][k];
#else
        g_filterCoeffPrevSelected[varInd][i]=g_filterCoeffSym[varIndTab[varInd]][k];
#endif      
        k++;
      }
      else
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i]=0;
#else
        g_filterCoeffPrevSelected[varInd][i]=0;
#endif        
      }
    }

    for(i = 0; i < length_resi; i++)
    {
      if (patternMap_resi[i]>0)
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i+length]=filterCoeffSym[varIndTab[varInd]][k];
#else
        g_filterCoeffPrevSelected[varInd][i+length]=g_filterCoeffSym[varIndTab[varInd]][k];
#endif        
        k++;
      }
      else
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i+length]=0;
#else
        g_filterCoeffPrevSelected[varInd][i+length]=0;
#endif
      }
    }

    for(i = 0; i < length_pred; i++)
    {
      if (patternMap_pred[i]>0)
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i+length+length]=filterCoeffSym[varIndTab[varInd]][k];
#else
        g_filterCoeffPrevSelected[varInd][i+length+length]=g_filterCoeffSym[varIndTab[varInd]][k];
#endif
        k++;
      }
      else
      {
#if ALF_MEM_PATCH
        filterCoeffPrevSelected[varInd][i+length+length]=0;
#else
        g_filterCoeffPrevSelected[varInd][i+length+length]=0;
#endif
      }
    }
#if ALF_MEM_PATCH
    filterCoeffPrevSelected[varInd][MAX_SQR_FILT_LENGTH-1]=filterCoeffSym[varIndTab[varInd]][k];//DC
#else
    g_filterCoeffPrevSelected[varInd][MAX_SQR_FILT_LENGTH-1]=g_filterCoeffSym[varIndTab[varInd]][k];//DC
#endif
  }
}
#else
Void TEncAdaptiveLoopFilter::xcalcPredFilterCoeff(int filtNo)
{
  int *patternMap, varInd, i, k;

  patternMap=patternMapTab[filtNo];
  for(varInd=0; varInd<NO_VAR_BINS; ++varInd)
  {		
	k=0;
	for(i = 0; i < MAX_SQR_FILT_LENGTH; i++)
	{
	  if (patternMap[i]>0)
	  {
#if ALF_MEM_PATCH
		filterCoeffPrevSelected[varInd][i]=filterCoeffSym[varIndTab[varInd]][k];
#else
		g_filterCoeffPrevSelected[varInd][i]=g_filterCoeffSym[varIndTab[varInd]][k];
#endif
		k++;
	  }
	  else
	  {
#if ALF_MEM_PATCH
		  filterCoeffPrevSelected[varInd][i]=0;
#else
		  g_filterCoeffPrevSelected[varInd][i]=0;
#endif
	  }
	}
  }
}
#endif

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xcodeFiltCoeff(int **filterCoeffSymQuant, int filtNo,  int filtNo_pred,  int filtNo_resi, int varIndTab[], int filters_per_fr_best, int frNo,ALFParam* ALFp)
#else
Void TEncAdaptiveLoopFilter::xcodeFiltCoeff(int **filterCoeffSymQuant, int filtNo, int varIndTab[], int filters_per_fr_best, int frNo, ALFParam* ALFp)
#endif
{
#if WIENER_3_INPUT
  int varInd, i, forceCoeff0, codedVarBins[NO_VAR_BINS], coeffBits, createBistream, fl=flTab[filtNo], coded;
  int sqrFiltLength = sqrFiltLengthTab[filtNo] + sqrFiltLengthTab_pr[filtNo_pred] + sqrFiltLengthTab_pr[filtNo_resi];
  int sqrFiltLength_pred = sqrFiltLengthTab_pr[filtNo_pred];
  int sqrFiltLength_resi = sqrFiltLengthTab_pr[filtNo_resi]; 
  int prec_rec =ALFp->filter_precision[0];
  int prec_pred=ALFp->filter_precision[1];
  int prec_resi=ALFp->filter_precision[2];
#else
  int varInd, i, forceCoeff0, codedVarBins[NO_VAR_BINS], coeffBits, createBistream,   sqrFiltLength=sqrFiltLengthTab[filtNo], 
    fl=flTab[filtNo], coded;
#endif
    
  ALFp->filters_per_group_diff = filters_per_fr_best;
  ALFp->filters_per_group = filters_per_fr_best;

  for(varInd=0; varInd<filters_per_fr_best; varInd++)
  {
    codedVarBins[varInd]=0;
    for(i = 0; i < MAX_SQR_FILT_LENGTH; i++)
	{
      if (filterCoeffSymQuant[varInd][i] != 0)
	  {
        codedVarBins[varInd]=1;
        break;
      }
    }
  }
  memcpy (ALFp->codedVarBins, codedVarBins, sizeof(int)*NO_VAR_BINS);
  forceCoeff0=0;
  for(varInd=0; varInd<filters_per_fr_best; varInd++)
  {
    if (codedVarBins[varInd] == 0)
	{
      forceCoeff0=1;
      break;
    }
  }

  coded=1;
  if (forceCoeff0==1 && filters_per_fr_best==1)
  {
    coded=0;
    coeffBits = xcodeAuxInfo(-1, filters_per_fr_best, varIndTab, frNo, createBistream=1,filtNo, ALFp);
  }
  else
  {
    coeffBits = xcodeAuxInfo(filtNo, filters_per_fr_best, varIndTab, frNo, createBistream=1,filtNo, ALFp);
  }
  
  ALFp->forceCoeff0 = forceCoeff0;
  ALFp->predMethod = 0;
  ALFp->num_coeff = sqrFiltLength;
#if WIENER_3_INPUT
  ALFp->num_coeff_pred = sqrFiltLength_pred;
  ALFp->num_coeff_resi = sqrFiltLength_resi;
#endif  
  ALFp->realfiltNo=filtNo;
  if (filters_per_fr_best <= 1)
  {
	ALFp->forceCoeff0 = 0;
	ALFp->predMethod = 0;
  }

  if (forceCoeff0==0) 
  {
#if WIENER_3_INPUT
    coeffBits += xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi,
#else    
    coeffBits += xsendAllFiltersPPPred(filterCoeffSymQuant, fl, sqrFiltLength, 
#endif
    filters_per_fr_best, createBistream=1, ALFp);
  }
  else if (filters_per_fr_best>1)
  {
#if WIENER_3_INPUT
    coeffBits += xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, prec_rec, prec_pred, prec_resi,
#else    
    coeffBits += xsendAllFiltersPPPredForce0(filterCoeffSymQuant, fl, sqrFiltLength, 
#endif
      filters_per_fr_best, codedVarBins, createBistream=1, ALFp);
  }
}



#if TSB_ALF_HEADER
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlags_qc(UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, ALFParam *pAlfParam)
#else
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlags_qc(UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist)
#endif
{
  ruiDist = 0;
#if TSB_ALF_HEADER
  pAlfParam->num_alf_cu_flag = 0;
#endif

  for( UInt uiCUAddr = 0; uiCUAddr < m_pcPic->getNumCUsInFrame() ; uiCUAddr++ )
  {
    TComDataCU* pcCU = m_pcPic->getCU( uiCUAddr );
#if TSB_ALF_HEADER
    xSetCUAlfCtrlFlag_qc(pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist, pAlfParam);
#else
    xSetCUAlfCtrlFlag_qc(pcCU, 0, 0, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
#endif
  }
}

#if TSB_ALF_HEADER
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag_qc(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist, ALFParam *pAlfParam)
#else
Void TEncAdaptiveLoopFilter::xSetCUAlfCtrlFlag_qc(TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiAlfCtrlDepth, TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiDist)
#endif
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
    UInt uiQNumParts = ( m_pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getWidth() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getHeight() ) )
#if TSB_ALF_HEADER
        xSetCUAlfCtrlFlag_qc(pcCU, uiAbsPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist, pAlfParam);
#else
        xSetCUAlfCtrlFlag_qc(pcCU, uiAbsPartIdx, uiDepth+1, uiAlfCtrlDepth, pcPicOrg, pcPicDec, pcPicRest, ruiDist);
#endif
    }
    return;
  }

  if( uiDepth > uiAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    return;
  }

  UInt uiCUAddr = pcCU->getAddr();
  UInt64 uiRecSSD = 0;
  UInt64 uiFiltSSD = 0;

  Int iWidth;
  Int iHeight;
  UInt uiSetDepth;

  if (uiDepth > uiAlfCtrlDepth && pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, uiAlfCtrlDepth))
  {
    iWidth = g_uiMaxCUWidth >> uiAlfCtrlDepth;
    iHeight = g_uiMaxCUHeight >> uiAlfCtrlDepth;

    uiRPelX   = uiLPelX + iWidth  - 1;
    uiBPelY   = uiTPelY + iHeight - 1;

    if( uiRPelX >= pcCU->getSlice()->getSPS()->getWidth() )
    {
      iWidth = pcCU->getSlice()->getSPS()->getWidth() - uiLPelX;
    }

    if( uiBPelY >= pcCU->getSlice()->getSPS()->getHeight() )
    {
      iHeight = pcCU->getSlice()->getSPS()->getHeight() - uiTPelY;
    }

    uiSetDepth = uiAlfCtrlDepth;
  }
  else
  {
    iWidth = pcCU->getWidth(uiAbsPartIdx);
    iHeight = pcCU->getHeight(uiAbsPartIdx);
    uiSetDepth = uiDepth;
  }

  Pel* pOrg = pcPicOrg->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pRec = pcPicDec->getLumaAddr(uiCUAddr, uiAbsPartIdx);
  Pel* pFilt = pcPicRest->getLumaAddr(uiCUAddr, uiAbsPartIdx);

  uiRecSSD  += xCalcSSD( pOrg, pRec,  iWidth, iHeight, pcPicOrg->getStride() );
  uiFiltSSD += xCalcSSD( pOrg, pFilt, iWidth, iHeight, pcPicOrg->getStride() );

  if (uiFiltSSD < uiRecSSD)
  {
    ruiDist += uiFiltSSD;
    pcCU->setAlfCtrlFlagSubParts(1, uiAbsPartIdx, uiSetDepth);
#if TSB_ALF_HEADER
    pAlfParam->alf_cu_flag[pAlfParam->num_alf_cu_flag]=1;
#endif
	for (int i=uiTPelY ;i<=min(uiBPelY,(unsigned int)(pcPicOrg->getHeight()-1))  ;i++)
	  for (int j=uiLPelX ;j<=min(uiRPelX,(unsigned int)(pcPicOrg->getWidth()-1)) ;j++)
	  { 
		maskImg[i][j]=1;
	  }
  }
  else
  {
    ruiDist += uiRecSSD;
    pcCU->setAlfCtrlFlagSubParts(0, uiAbsPartIdx, uiSetDepth);
#if TSB_ALF_HEADER
    pAlfParam->alf_cu_flag[pAlfParam->num_alf_cu_flag]=0;
#endif
	for (int i=uiTPelY ;i<=min(uiBPelY,(unsigned int)(pcPicOrg->getHeight()-1))  ;i++)
	  for (int j=uiLPelX ;j<=min(uiRPelX,(unsigned int)(pcPicOrg->getWidth()-1)) ;j++)
	  { 
		maskImg[i][j]=0;
	  }
  }
#if TSB_ALF_HEADER
  pAlfParam->num_alf_cu_flag++;
#endif
}

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xReDesignFilterCoeff_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, Bool bReadCorr)
#else
Void TEncAdaptiveLoopFilter::xReDesignFilterCoeff_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, Bool bReadCorr)
#endif
{
  Int tap = m_pcTempAlfParam->tap;
  Int    Height = pcPicOrg->getHeight();
  Int    Width = pcPicOrg->getWidth();
  Int    LumaStride = pcPicOrg->getStride();
#if ALF_MEM_PATCH
  imgpel* pOrg = (imgpel*)pcPicOrg->getLumaAddr();
  imgpel* pDec = (imgpel*)pcPicDec->getLumaAddr();
  imgpel* pRest = (imgpel*)pcPicRest->getLumaAddr();
#if WIENER_3_INPUT
  imgpel* pResi = (imgpel*)pcPicResi->getLumaAddr();
  imgpel* pPred = (imgpel*)pcPicPred->getLumaAddr();
   
  xFirstFilteringFrameLuma(pOrg, pDec, pRest, pResi, pPred, m_pcTempAlfParam, tap, LumaStride); 
#else  
  xFirstFilteringFrameLuma(pOrg, pDec, pRest, m_pcTempAlfParam, tap, LumaStride); 
#endif
#else
  Pel* pOrg = pcPicOrg->getLumaAddr();
  Pel* pDec = pcPicDec->getLumaAddr();
  Pel* pRest = pcPicRest->getLumaAddr();
#if WIENER_3_INPUT
  Pel* pResi = pcPicResi->getLumaAddr();
  Pel* pPred = pcPicPred->getLumaAddr();
#endif  
  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
    {
      imgY_org[i][j]=pOrg[j + i*LumaStride];
      imgY_rest[i][j]=pDec[j + i*LumaStride];
#if WIENER_3_INPUT
      imgY_resi[i][j]=pResi[j + i*LumaStride];
      imgY_pred[i][j]=pPred[j + i*LumaStride];
#endif      
    }
    
  padImage(imgY_rest, imgY_ext, 4, Height, Width);
#if WIENER_3_INPUT
  padImage(imgY_pred, imgY_pext, 4, Height, Width);
  padImage(imgY_resi, imgY_rext, 4, Height, Width);
  xFirstFilteringFrameLuma(imgY_org, imgY_ext, imgY_rest, imgY_rext, imgY_pext, m_pcTempAlfParam, tap);
#else
  xFirstFilteringFrameLuma(imgY_org, imgY_ext, imgY_rest, m_pcTempAlfParam, tap);
#endif
  for (Int i=0; i<Height; i++)
    for (Int j=0; j<Width; j++)
    {
	  pRest[j + i*LumaStride]=imgY_rest[i][j];
    }
#endif
}

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xCUAdaptiveControl_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
#else    
Void TEncAdaptiveLoopFilter::xCUAdaptiveControl_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
#endif
{
  m_pcEntropyCoder->setAlfCtrl(true);

  UInt uiBestDepth = 0;

  ALFParam cFrmAlfParam;
  allocALFParam(&cFrmAlfParam);
  copyALFParam(&cFrmAlfParam, m_pcBestAlfParam);
  for (UInt uiDepth = 0; uiDepth < g_uiMaxCUDepth; uiDepth++)
  {
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiDepth);
    pcPicRest->copyToPicLuma(m_pcPicYuvTmp);
    copyALFParam(m_pcTempAlfParam, &cFrmAlfParam);
    m_pcTempAlfParam->cu_control_flag = 1;

    for (UInt uiRD = 0; uiRD <= ALF_NUM_OF_REDESIGN; uiRD++)
    {
      if (uiRD)
      {
        // re-design filter coefficients
#if WIENER_3_INPUT
        xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, true); //use filtering of mine
#else        
        xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, true); //use filtering of mine
#endif
      }

      UInt64 uiRate, uiDist;
      Double dCost;
	  //m_pcPicYuvTmp: filtered signal, pcPicDec: orig reconst
#if TSB_ALF_HEADER
      xSetCUAlfCtrlFlags_qc(uiDepth, pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam); //set up varImg here
#else
      xSetCUAlfCtrlFlags_qc(uiDepth, pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist); //set up varImg here
#endif

      xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);

      if (dCost < rdMinCost)
      {
        uiBestDepth = uiDepth;
        rdMinCost = dCost;
        ruiMinDist = uiDist;
        ruiMinRate = uiRate;
        m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);
        copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
		//save maskImg
        xCopyTmpAlfCtrlFlagsFrom();
      }
    }
  }

  if (m_pcBestAlfParam->cu_control_flag)
  {
    m_pcEntropyCoder->setAlfCtrl(true);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(uiBestDepth);
    xCopyTmpAlfCtrlFlagsTo();
    m_pcPicYuvBest->copyToPicLuma(pcPicRest);//copy m_pcPicYuvBest to pcPicRest
    xCopyDecToRestCUs(pcPicDec, pcPicRest); //pcPicRest = pcPicDec
  }
  else
  {
    m_pcEntropyCoder->setAlfCtrl(false);
    m_pcEntropyCoder->setMaxAlfCtrlDepth(0);
  }
  freeALFParam(&cFrmAlfParam);
}

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::xFilterTapDecision_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, TComPicYuv* pcPicPred, TComPicYuv* pcPicResi, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
#else
Void TEncAdaptiveLoopFilter::xFilterTapDecision_qc(TComPicYuv* pcPicOrg, TComPicYuv* pcPicDec, TComPicYuv* pcPicRest, UInt64& ruiMinRate, UInt64& ruiMinDist, Double& rdMinCost)
#endif
{
  // restriction for non-referenced B-slice
  if (m_eSliceType == B_SLICE && m_iPicNalReferenceIdc == 0)
  {
    return;
  }

  UInt64 uiRate, uiDist;
  Double dCost;
#if WIENER_3_INPUT
  int changed=0;
#endif
#if (WIENER_3_INPUT && WIENER_3_INPUT_FAST)
  int tap_rec_best  = ALF_MIN_NUM_TAP;
  int tap_pred_best = ALF_MIN_NUM_TAP_PQ;
  int tap_resi_best = ALF_MIN_NUM_TAP_PQ;
#endif
  if (m_pcBestAlfParam->cu_control_flag)
  {
    xCopyTmpAlfCtrlFlagsFrom();
  }

  Bool bChanged = false;
#if (WIENER_3_INPUT && WIENER_3_INPUT_FAST)
  adapt_precision=0;
  for (Int iTap = ALF_MIN_NUM_TAP; iTap <= ALF_MAX_NUM_TAP; iTap += 2)
  {
    for (Int iTap_pred = ALF_MIN_NUM_TAP_PQ; iTap_pred <= iTap; iTap_pred += 2)
    {
      for (Int iTap_resi = ALF_MIN_NUM_TAP_PQ; iTap_resi <= iTap; iTap_resi += 2)
      {
	copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
        m_pcTempAlfParam->tap = iTap;
        m_pcTempAlfParam->tap_pred = iTap_pred;
        m_pcTempAlfParam->tap_resi = iTap_resi;
        m_pcTempAlfParam->num_coeff = (Int)(m_pcTempAlfParam->tap*m_pcTempAlfParam->tap/4 + 2 + m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 +  m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1);
        m_pcTempAlfParam->num_coeff_pred = (Int)(m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 );
        m_pcTempAlfParam->num_coeff_resi = (Int)(m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1 );
        if (m_pcTempAlfParam->cu_control_flag)
        {
          xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
#if TSB_ALF_HEADER
          xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam);
#else
          xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist);
#endif          
          xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);
        }
        else
        {
          Int    Height = pcPicOrg->getHeight();
          Int    Width  = pcPicOrg->getWidth();
          for (Int i=0; i<Height; i++)
            for (Int j=0; j<Width; j++)
            {
              maskImg[i][j] = 1;
            }
          xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
          xCalcRDCost(pcPicOrg, m_pcPicYuvTmp, m_pcTempAlfParam, uiRate, uiDist, dCost);
        }
   
        if (dCost < rdMinCost)
        {
          tap_rec_best =iTap;
          tap_pred_best=iTap_pred;
          tap_resi_best=iTap_resi;          
          changed=1;          
          rdMinCost = dCost;
          ruiMinDist = uiDist;
          ruiMinRate = uiRate;
          m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);
          copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
          bChanged = true;
          if (m_pcTempAlfParam->cu_control_flag)
          {
            xCopyTmpAlfCtrlFlagsFrom();
          }
        }
      }
    }  
  }
  adapt_precision=1;
  copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
  m_pcTempAlfParam->tap      = tap_rec_best;
  m_pcTempAlfParam->tap_pred = tap_pred_best;
  m_pcTempAlfParam->tap_resi = tap_resi_best;
  m_pcTempAlfParam->num_coeff = (Int)(m_pcTempAlfParam->tap*m_pcTempAlfParam->tap/4 + 2 + m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 +  m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1);
  m_pcTempAlfParam->num_coeff_pred = (Int)(m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 );
  m_pcTempAlfParam->num_coeff_resi = (Int)(m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1 );
  if (m_pcTempAlfParam->cu_control_flag)
  {
    xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
#if TSB_ALF_HEADER
    xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam);
#else
    xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist);
#endif          
    xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);
  }
  else
  {
    Int    Height = pcPicOrg->getHeight();
    Int    Width = pcPicOrg->getWidth();
    for (Int i=0; i<Height; i++)
      for (Int j=0; j<Width; j++)
      {
        maskImg[i][j] = 1;
      }
    xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
    xCalcRDCost(pcPicOrg, m_pcPicYuvTmp, m_pcTempAlfParam, uiRate, uiDist, dCost);
  }

  if (dCost < rdMinCost)
  {
    rdMinCost = dCost;
    ruiMinDist = uiDist;
    ruiMinRate = uiRate;
    m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);    
    changed=1;
    copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
    bChanged = true;
    if (m_pcTempAlfParam->cu_control_flag)
    {
      xCopyTmpAlfCtrlFlagsFrom();
    }
  }
  
#else  
  for (Int iTap = ALF_MIN_NUM_TAP; iTap <= ALF_MAX_NUM_TAP; iTap += 2)
  {
#if WIENER_3_INPUT
    for (Int iTap_pred = ALF_MIN_NUM_TAP_PQ; iTap_pred <= ALF_MAX_NUM_TAP; iTap_pred += 2)
    {
      for (Int iTap_resi = ALF_MIN_NUM_TAP_PQ; iTap_resi <= ALF_MAX_NUM_TAP; iTap_resi += 2)
      {        
#endif
    copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
    m_pcTempAlfParam->tap = iTap;
#if WIENER_3_INPUT
    m_pcTempAlfParam->tap_pred = iTap_pred;
    m_pcTempAlfParam->tap_resi = iTap_resi;
    m_pcTempAlfParam->num_coeff = (Int)(iTap*iTap/4 + 2 + m_pcTempAlfParam->tap_pred*m_pcTempAlfParam->tap_pred/4 + 1 +  m_pcTempAlfParam->tap_resi*m_pcTempAlfParam->tap_resi/4 + 1);
    m_pcTempAlfParam->num_coeff_pred = (Int)(iTap_pred*iTap_pred/4 + 1 );
    m_pcTempAlfParam->num_coeff_resi = (Int)(iTap_resi*iTap_resi/4 + 1 );
#else    
    m_pcTempAlfParam->num_coeff = (Int)(iTap*iTap/4) + 2; 
#endif
    if (m_pcTempAlfParam->cu_control_flag)
    {
#if WIENER_3_INPUT
      xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
#else 
      xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, false);
#endif
#if TSB_ALF_HEADER
      xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist, m_pcTempAlfParam);
#else
      xSetCUAlfCtrlFlags_qc(m_pcEntropyCoder->getMaxAlfCtrlDepth(), pcPicOrg, pcPicDec, m_pcPicYuvTmp, uiDist);
#endif
      xCalcRDCost(m_pcTempAlfParam, uiRate, uiDist, dCost);
    }
    else
    {
#if ALF_MEM_PATCH
      Int    Height = pcPicOrg->getHeight();
      Int    Width = pcPicOrg->getWidth();
      for (Int i=0; i<Height; i++)
	for (Int j=0; j<Width; j++)
	{
	  maskImg[i][j] = 1;
	}
#if WIENER_3_INPUT
      xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
#else
      xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, false);
#endif
#else
#if WIENER_3_INPUT      
      Int    Height = pcPicOrg->getHeight();
      Int    Width = pcPicOrg->getWidth();
      for (Int i=0; i<Height; i++)
        for (Int j=0; j<Width; j++)
        {
          maskImg[i][j] = 1;
        }
      xReDesignFilterCoeff_qc(pcPicOrg, pcPicDec, m_pcPicYuvTmp, pcPicPred, pcPicResi, false);
#else
	  Int    Height = pcPicOrg->getHeight();
	  Int    Width = pcPicOrg->getWidth();
	  Int    LumaStride = pcPicOrg->getStride();
	  Pel* pOrg = pcPicOrg->getLumaAddr();
	  Pel* pDec = pcPicDec->getLumaAddr();
	  Pel* pRest = m_pcPicYuvTmp->getLumaAddr();
	  
	  for (Int i=0; i<Height; i++)
		for (Int j=0; j<Width; j++)
		{
		  imgY_org[i][j]=pOrg[j + i*LumaStride];
		  imgY_rest[i][j]=pDec[j + i*LumaStride];
		  maskImg[i][j] = 1;
		}
      padImage(imgY_rest, imgY_ext, 4, Height, Width);
	  xFirstFilteringFrameLuma(imgY_org, imgY_ext, imgY_rest, m_pcTempAlfParam, m_pcTempAlfParam->tap); 
	  for (Int i=0; i<Height; i++)
		for (Int j=0; j<Width; j++)
		{
//		  pRest[j + i*LumaStride]=Clip3(0, g_uiIBDI_MAX, imgY_rest[i][j]);
		  pRest[j + i*LumaStride]=imgY_rest[i][j];
		}
#endif
#endif

	  xCalcRDCost(pcPicOrg, m_pcPicYuvTmp, m_pcTempAlfParam, uiRate, uiDist, dCost);
    }

    if (dCost < rdMinCost)
    {
#if WIENER_3_INPUT
      changed=1;
#endif
      rdMinCost = dCost;
      ruiMinDist = uiDist;
      ruiMinRate = uiRate;
      m_pcPicYuvTmp->copyToPicLuma(m_pcPicYuvBest);
      copyALFParam(m_pcBestAlfParam, m_pcTempAlfParam);
      bChanged = true;
      if (m_pcTempAlfParam->cu_control_flag)
      {
        xCopyTmpAlfCtrlFlagsFrom();
      }
    }
  }
#if WIENER_3_INPUT
    }
  }
#endif
  
#endif
  if (m_pcBestAlfParam->cu_control_flag)
  {
    xCopyTmpAlfCtrlFlagsTo();
    if (bChanged)
    {
      m_pcPicYuvBest->copyToPicLuma(pcPicRest);
      xCopyDecToRestCUs(pcPicDec, pcPicRest);
    }
  }
#if WIENER_3_INPUT
  else if (changed==1)
#else
  else if (m_pcBestAlfParam->tap > ALF_MIN_NUM_TAP)
#endif
  {
    m_pcPicYuvBest->copyToPicLuma(pcPicRest);
  }

  copyALFParam(m_pcTempAlfParam, m_pcBestAlfParam);
}


#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
Int TEncAdaptiveLoopFilter::gnsCholeskyDec(double **inpMatr, double outMatr[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], int noEq)
{ 
  int 
    i, j, k;     /* Looping Variables */
  double 
    scale;       /* scaling factor for each row */
  double 
    invDiag[MAX_SQR_FILT_LENGTH];  /* Vector of the inverse of diagonal entries of outMatr */


  /*
   *  Cholesky decomposition starts
   */

  for(i = 0; i < noEq; i++)
    for(j = i; j < noEq; j++)
    {
      /* Compute the scaling factor */
      scale=inpMatr[i][j];
      if ( i > 0) for( k = i - 1 ; k >= 0 ; k--)
        scale -= outMatr[k][j] * outMatr[k][i];

      /* Compute i'th row of outMatr */
      if(i==j)
	  {
        if(scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
		{
			return(0);
        }
        else              /* Normal operation */
          invDiag[i] =  1.0/(outMatr[i][i]=sqrt(scale));
      }
      else
	  {
        outMatr[i][j] = scale*invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }                    
    }
    return(1); /* Signal that Cholesky factorization is successfully performed */
}


Void TEncAdaptiveLoopFilter::gnsTransposeBacksubstitution(double U[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], double rhs[], double x[], int order)
{
  int 
    i,j;              /* Looping variables */
  double 
    sum;              /* Holds backsubstitution from already handled rows */

  /* Backsubstitution starts */
  x[0] = rhs[0]/U[0][0];               /* First row of U'                   */
  for (i = 1; i < order; i++){         /* For the rows 1..order-1           */

    for (j = 0, sum = 0.0; j < i; j++) /* Backsubst already solved unknowns */
      sum += x[j]*U[j][i];

    x[i]=(rhs[i] - sum)/U[i][i];       /* i'th component of solution vect.  */
  }
}



Void  TEncAdaptiveLoopFilter::gnsBacksubstitution(double R[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH], double z[MAX_SQR_FILT_LENGTH], int R_size, double A[MAX_SQR_FILT_LENGTH])
{
  int
    i, j;

  double
    sum;

  R_size--;

  A[R_size] = z[R_size] / R[R_size][R_size];

  for (i = R_size-1; i >= 0; i--) {

    for (j = i+1, sum = 0.0; j <= R_size; j++)
      sum += R[i][j] * A[j];

    A[i] = (z[i] - sum) / R[i][i];

  }
}


Int TEncAdaptiveLoopFilter::gnsSolveByChol(double **LHS, double *rhs, double *x, int noEq)
{ 
  double aux[MAX_SQR_FILT_LENGTH];     /* Auxiliary vector */
  double U[MAX_SQR_FILT_LENGTH][MAX_SQR_FILT_LENGTH];    /* Upper triangular Cholesky factor of LHS */
  int  i, singular;          /* Looping variable */
  
  /* The equation to be solved is LHSx = rhs */
  
  /* Compute upper triangular U such that U'*U = LHS */
  if(gnsCholeskyDec(LHS, U, noEq)) /* If Cholesky decomposition has been successful */
  {
    singular=1;
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
     * Solve U'*aux = rhs for aux
     */
    gnsTransposeBacksubstitution(U, rhs, aux, noEq);         

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution(U, aux, noEq, x);   

  }
  else /* LHS was singular */ 
  {
    singular=0;

    /* Regularize LHS */
    for(i=0; i<noEq; i++)
      LHS[i][i] += REG;
    /* Compute upper triangular U such that U'*U = regularized LHS */
    singular = gnsCholeskyDec(LHS, U, noEq);
    /* Solve  U'*aux = rhs for aux */  
    gnsTransposeBacksubstitution(U, rhs, aux, noEq);   

    /* Solve U*x = aux for x */
    gnsBacksubstitution(U, aux, noEq, x);
  }  
  return(singular);
}


//////////////////////////////////////////////////////////////////////////////////////////


Void TEncAdaptiveLoopFilter::add_A(double **Amerged, double ***A, int start, int stop, int size)
{ 
  int
    i, j, ind;          /* Looping variable */

  for (i=0; i<size; i++){
    for (j=0; j<size; j++){
      Amerged[i][j]=0;
      for (ind=start; ind<=stop; ind++){
        Amerged[i][j]+=A[ind][i][j];
      }
    }
  }
}

Void TEncAdaptiveLoopFilter::add_b(double *bmerged, double **b, int start, int stop, int size)
{ 
  int
    i, ind;          /* Looping variable */

  for (i=0; i<size; i++){
    bmerged[i]=0;
    for (ind=start; ind<=stop; ind++){
      bmerged[i]+=b[ind][i];
    }
  }
}
  
double TEncAdaptiveLoopFilter::add_pixAcc(double *pixAcc, int start, int stop)
{ 
  int ind;
  double pixAccMerged=0;

  for (ind=start; ind<=stop; ind++){
    pixAccMerged+=pixAcc[ind];
  }
  return(pixAccMerged);
}


double TEncAdaptiveLoopFilter::calculateErrorCoeffProvided(double **A, double *b, double *c, int size)
{
  int i, j;
  double error, sum=0;

  error=0;
  for (i=0; i<size; i++)   //diagonal
  {
    sum=0;
	for (j=i+1; j<size; j++)
	  sum+=(A[j][i]+A[i][j])*c[j];
	error+=(A[i][i]*c[i]+sum-2*b[i])*c[i];
  }

  return(error);
}


double TEncAdaptiveLoopFilter::calculateErrorCoeffProvidedInt(double **A, double *b, int *c, int size,int *pattern)
{
  int i, j;
  double error, sum=0;
  double factor = 1.0/(double)(1<<(NUM_BITS-1));
  double d[MAX_SQR_FILT_LENGTH];
  error=0;

  for (i=0; i<size; i++)
  {
	d[i]=c[i]*factor;
  }

  for (i=0; i<size-1; i++)   //diagonal
  {
    sum=0;
	for (j=i+1; j<size-1; j++)
		sum+=(A[j][i]+A[i][j])*d[pattern[j]];
	sum+=(A[size-1][i]+A[i][size-1])*d[MAX_SQR_FILT_LENGTH-1];
	error+=(A[i][i]*d[pattern[i]]+sum-2*b[i])*d[pattern[i]];
  }
  i=size-1;
  error+=(A[i][i]*d[MAX_SQR_FILT_LENGTH-1]-2*b[i])*d[MAX_SQR_FILT_LENGTH-1];

  return(error);
}



double TEncAdaptiveLoopFilter::calculateErrorAbs(double **A, double *b, double y, int size)
{
  int i;
  double error, sum;
#if ALF_MEM_PATCH
  double c[MAX_SQR_FILT_LENGTH];
#else
  static double *c;
  static int first=0;
#endif

#if !ALF_MEM_PATCH
  if(first == 0)
  {
    first = 1;
    c = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));
  }
#endif

  gnsSolveByChol(A, b, c, size);

  sum=0;
  for (i=0; i<size; i++)
  {
    sum+=c[i]*b[i];
  }
  error=y-sum;

  return(error);
}

double TEncAdaptiveLoopFilter::mergeFiltersGreedy(double **yGlobalSeq, double ***EGlobalSeq, double *pixAccGlobalSeq, int intervalBest[NO_VAR_BINS][2], int sqrFiltLength, int noIntervals)
{
#if ALF_MEM_PATCH
  int first, ind, ind1, ind2, i, j, bestToMerge ;
  double error, error1, error2, errorMin;
  static double pixAcc_temp, error_tab[NO_VAR_BINS],error_comb_tab[NO_VAR_BINS];
  static int indexList[NO_VAR_BINS], available[NO_VAR_BINS], noRemaining;
#else
  int first, ind, ind1, ind2, i, j, bestToMerge ;
  double error, error1, error2, errorMin;
  static double **y_merged, ***E_merged, *pixAcc_merged, *y_temp, **E_temp, pixAcc_temp, error_tab[NO_VAR_BINS],error_comb_tab[NO_VAR_BINS];
  static int init=0, indexList[NO_VAR_BINS], available[NO_VAR_BINS], noRemaining;

  if(init == 0)
  {
    initMatrix_double(&E_temp, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
    y_temp = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));

    initMatrix3D_double(&E_merged, NO_VAR_BINS, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
    initMatrix_double(&y_merged, NO_VAR_BINS, MAX_SQR_FILT_LENGTH); 
    pixAcc_merged = (double *) calloc(NO_VAR_BINS, sizeof(double));
    init = 1;
  }
#endif
  if (noIntervals == NO_FILTERS)
  {
	  noRemaining=NO_VAR_BINS;
	  for (ind=0; ind<NO_VAR_BINS; ind++)
	  {
		indexList[ind]=ind; 
		available[ind]=1;
		pixAcc_merged[ind]=pixAccGlobalSeq[ind];
		memcpy(y_merged[ind],yGlobalSeq[ind],sizeof(double)*sqrFiltLength);
		for (i=0; i<sqrFiltLength; i++)
		{
	      memcpy(E_merged[ind][i],EGlobalSeq[ind][i],sizeof(double)*sqrFiltLength);
		}
	  }
  }
  // Try merging different matrices
      if (noIntervals == NO_FILTERS)
	  {
		  for (ind=0; ind<NO_VAR_BINS; ind++)
		  {
			  error_tab[ind]=calculateErrorAbs(E_merged[ind], y_merged[ind], pixAcc_merged[ind], sqrFiltLength);
		  }
		  for (ind=0; ind<NO_VAR_BINS-1; ind++)
		  {
			ind1=indexList[ind];
			ind2=indexList[ind+1];

			error1=error_tab[ind1];
			error2=error_tab[ind2];

			pixAcc_temp=pixAcc_merged[ind1]+pixAcc_merged[ind2];
			for (i=0; i<sqrFiltLength; i++)
			{
			  y_temp[i]=y_merged[ind1][i]+y_merged[ind2][i];
			  for (j=0; j<sqrFiltLength; j++)
			  {
				E_temp[i][j]=E_merged[ind1][i][j]+E_merged[ind2][i][j];
			  }
			}
			error_comb_tab[ind1]=calculateErrorAbs(E_temp, y_temp, pixAcc_temp, sqrFiltLength)-error1-error2;
		  }
	  }
	  while (noRemaining>noIntervals)
	  {
		errorMin=0; first=1;
		bestToMerge = 0;
		for (ind=0; ind<noRemaining-1; ind++)
		{
			error = error_comb_tab[indexList[ind]];
			if ((error<errorMin || first==1))
			{
			  errorMin=error;
			  bestToMerge=ind;
			  first=0;
			}
		}
		ind1=indexList[bestToMerge];
		ind2=indexList[bestToMerge+1];
		pixAcc_merged[ind1]+=pixAcc_merged[ind2];
		for (i=0; i<sqrFiltLength; i++)
		{
			y_merged[ind1][i]+=y_merged[ind2][i];
			for (j=0; j<sqrFiltLength; j++)
			{
			  E_merged[ind1][i][j]+=E_merged[ind2][i][j];
			}
		}
        available[ind2]=0;

		//update error tables
        error_tab[ind1]=error_comb_tab[ind1]+error_tab[ind1]+error_tab[ind2];
		if (indexList[bestToMerge] > 0)
		{
			ind1=indexList[bestToMerge-1];
			ind2=indexList[bestToMerge];
			error1=error_tab[ind1];
			error2=error_tab[ind2];
			pixAcc_temp=pixAcc_merged[ind1]+pixAcc_merged[ind2];
			for (i=0; i<sqrFiltLength; i++)
			{
				y_temp[i]=y_merged[ind1][i]+y_merged[ind2][i];
				for (j=0; j<sqrFiltLength; j++)
				{
					E_temp[i][j]=E_merged[ind1][i][j]+E_merged[ind2][i][j];
				}
			}
			error_comb_tab[ind1]=calculateErrorAbs(E_temp, y_temp, pixAcc_temp, sqrFiltLength)-error1-error2;
		}
		if (indexList[bestToMerge+1] < NO_VAR_BINS-1)
		{
			ind1=indexList[bestToMerge];
			ind2=indexList[bestToMerge+2];
			error1=error_tab[ind1];
			error2=error_tab[ind2];
			pixAcc_temp=pixAcc_merged[ind1]+pixAcc_merged[ind2];
			for (i=0; i<sqrFiltLength; i++)
			{
				y_temp[i]=y_merged[ind1][i]+y_merged[ind2][i];
				for (j=0; j<sqrFiltLength; j++)
				{
					E_temp[i][j]=E_merged[ind1][i][j]+E_merged[ind2][i][j];
				}
			}
			error_comb_tab[ind1]=calculateErrorAbs(E_temp, y_temp, pixAcc_temp, sqrFiltLength)-error1-error2;
		}

		ind=0;
		for (i=0; i<NO_VAR_BINS; i++)
		{
			if (available[i]==1)
			{
				indexList[ind]=i;
				ind++;
			}
		}
		noRemaining--;
	  }
	

  errorMin=0;
  for (ind=0; ind<noIntervals; ind++)
  {
    errorMin+=error_tab[indexList[ind]];
  }

  for (ind=0; ind<noIntervals-1; ind++)
  {
    intervalBest[ind][0]=indexList[ind]; intervalBest[ind][1]=indexList[ind+1]-1;
  }

  intervalBest[noIntervals-1][0]=indexList[noIntervals-1]; 
  intervalBest[noIntervals-1][1]=NO_VAR_BINS-1;

  return(errorMin);
}



double TEncAdaptiveLoopFilter::findFilterGroupingError(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int intervalBest[NO_VAR_BINS][2], int sqrFiltLength, int filters_per_fr)
{
  double error;

  // find best filters for each frame group
    error = 0;
    error += mergeFiltersGreedy(yGlobalSeq, EGlobalSeq, pixAccGlobalSeq, intervalBest, sqrFiltLength, filters_per_fr);

   return(error);
}

#if WIENER_3_INPUT
Void TEncAdaptiveLoopFilter::roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int factor_rec, int factor_pred, int factor_resi)
#else
Void TEncAdaptiveLoopFilter::roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor)
#endif
{
  int i;
  double diff; 
  int diffInt, sign; 
#if WIENER_3_INPUT
  for(i = 0; i < sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
  {
    sign               = (FilterCoeff[i]>0) ?  1: -1; 
    diff               = FilterCoeff[i]*sign; 
    diffInt            = (int)(diff*(double)factor_rec+0.5); 
    FilterCoeffQuan[i] = diffInt*sign;  
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i<sqrFiltLength-sqrFiltLength_pred-1; i++)
  {
    sign               = (FilterCoeff[i]>0) ?  1: -1; 
    diff               = FilterCoeff[i]*sign; 
    diffInt            = (int)(diff*(double)factor_resi+0.5);
    FilterCoeffQuan[i] = diffInt*sign;  
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-1; i<sqrFiltLength-1; i++)
  {
    sign               = (FilterCoeff[i]>0) ?  1: -1; 
    diff               = FilterCoeff[i]*sign; 
    diffInt            = (int)(diff*(double)factor_pred+0.5); 
    FilterCoeffQuan[i] = diffInt*sign;  
  }
  for (i=sqrFiltLength-1; i<sqrFiltLength; i++)
  {
    sign               = (FilterCoeff[i]>0) ?  1: -1; 
    diff               = FilterCoeff[i]*sign; 
    diffInt            = (int)(diff*(double)factor_rec+0.5); 
    FilterCoeffQuan[i] = diffInt*sign;  
  }  
#else
  for(i = 0; i < sqrFiltLength; i++)
  {
    sign               = (FilterCoeff[i]>0) ?  1: -1; 
    diff               = FilterCoeff[i]*sign; 
    diffInt            = (int)(diff*(double)factor+0.5); 
    FilterCoeffQuan[i] = diffInt*sign;  
  }
#endif
}







#if WIENER_3_INPUT    
Double TEncAdaptiveLoopFilter::QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int *weights, int shift_rec, int shift_pred, int shift_resi)
#else    
Double TEncAdaptiveLoopFilter::QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, int sqrFiltLength, int *weights, int bit_depth)
#endif
{
  
  double error;

#if !ALF_MEM_PATCH
  static int init=0, *filterCoeffQuantMod;
#endif
#if WIENER_3_INPUT
  int factor_rec  = (1<<(shift_rec )), i; 
  int factor_pred = (1<<(shift_pred)); 
  int factor_resi = (1<<(shift_resi));
  int not_solvable_by_cholesky=0;
  int too_large_coefficients=0;
  int identical_lines=0;
#else
  int factor = (1<<(bit_depth-1)), i; 
#endif
  int quantCoeffSum, minInd, targetCoeffSumInt, k, diff;
  double targetCoeffSum, errMin;

#if !ALF_MEM_PATCH
  if(init == 0)
  {
    filterCoeffQuantMod = (int *) calloc(MAX_SQR_FILT_LENGTH, sizeof(int));
    init = 1;
  }
#endif
#if WIENER_3_INPUT
  for (Int k = 0; k < sqrFiltLength; k++)
  {
    for (Int l = k+1; l < sqrFiltLength; l++)
    {
      identical_lines=1;
      for (Int j = 0; j < sqrFiltLength; j++)
      {
        Double diff=E[k][j]-E[l][j];
        diff*=diff;
        if (diff>0.0001)
        {
          identical_lines=0;
          break;
        }
      }
      if (1 == identical_lines)
      {
        for (Int j = 0; j < sqrFiltLength; j++)
        {
          E[l][j] = 0.0;
          E[j][l] = 0.0;
          y[l]    = 0.0;
        }
      }
    }
  }
#endif
  
  gnsSolveByChol(E, y, filterCoeff, sqrFiltLength);
  
#if WIENER_3_INPUT
  for (i=0;i<sqrFiltLength;i++)
  {
    if (isnan(filterCoeff[i]))//check for NaN
    {
      not_solvable_by_cholesky=1;
      break;
    }
  }
  if (not_solvable_by_cholesky)
  {
    sgaus(E, filterCoeff, y, sqrFiltLength);
  }
  too_large_coefficients=0;
  for (i=0;i<sqrFiltLength;i++)
  {
    if (filterCoeff[i]>100.0 || filterCoeff[i]<-100.0)
    {
      too_large_coefficients=1;
      break;
    }
  }
  if (1 == too_large_coefficients)
  {
    for (i=0;i<sqrFiltLength;i++)
    {
      filterCoeff[i]=0.0;
    }
  }
#endif  
  
    
  targetCoeffSum=0;
#if WIENER_3_INPUT
  for (i=0; i<sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
  {
    targetCoeffSum+=(weights[i]*filterCoeff[i]*factor_rec);
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i<sqrFiltLength-sqrFiltLength_pred-1; i++)
  {
    targetCoeffSum+=(weights[i]*filterCoeff[i]*factor_resi);
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-1; i<sqrFiltLength-1; i++)
  {
    targetCoeffSum+=(weights[i]*filterCoeff[i]*factor_pred);
  }  
  targetCoeffSum+=(weights[sqrFiltLength-1]*filterCoeff[sqrFiltLength-1]*factor_rec);
  
  targetCoeffSumInt=ROUND(targetCoeffSum);
  roundFiltCoeff(filterCoeffQuant, filterCoeff, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, factor_rec, factor_pred, factor_resi);
#else  
  for (i=0; i<sqrFiltLength; i++)
  {
    targetCoeffSum+=(weights[i]*filterCoeff[i]*factor);
  }
  targetCoeffSumInt=ROUND(targetCoeffSum);
  roundFiltCoeff(filterCoeffQuant, filterCoeff, sqrFiltLength, factor);
#endif  
  quantCoeffSum=0;
  for (i=0; i<sqrFiltLength; i++)
  {
    quantCoeffSum+=weights[i]*filterCoeffQuant[i];
  }

#if (ALF_MEM_PATCH || WIENER_3_INPUT)
  int count=0;
  while(quantCoeffSum!=targetCoeffSumInt && count < 10)
#else
  while(quantCoeffSum!=targetCoeffSumInt)
#endif
  {
#if WIENER_3_INPUT
    count++;
#endif
    if (quantCoeffSum>targetCoeffSumInt)
	{
      diff=quantCoeffSum-targetCoeffSumInt;
      errMin=0; minInd=-1;
      for (k=0; k<sqrFiltLength; k++){
        if (weights[k]<=diff){
          for (i=0; i<sqrFiltLength; i++){
            filterCoeffQuantMod[i]=filterCoeffQuant[i];
          }
          filterCoeffQuantMod[k]--;
#if WIENER_3_INPUT
          for (i=0; i<sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_rec;
          }
          for (i=sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i<sqrFiltLength-sqrFiltLength_pred-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_resi;
          }
          for (i=sqrFiltLength-sqrFiltLength_pred-1; i<sqrFiltLength-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_pred;
          }  
          filterCoeff[sqrFiltLength-1]=(double)filterCoeffQuantMod[sqrFiltLength-1]/(double)factor_rec;
#else          
          for (i=0; i<sqrFiltLength; i++){
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor;
          }
#endif
          error=calculateErrorCoeffProvided(E, y, filterCoeff, sqrFiltLength);
          if (error<errMin || minInd==-1){
            errMin=error;
            minInd=k;
          }
        } // if (weights(k)<=diff){
      } // for (k=0; k<sqrFiltLength; k++){
      filterCoeffQuant[minInd]--;
    }
    else{
      diff=targetCoeffSumInt-quantCoeffSum;
      errMin=0; minInd=-1;
      for (k=0; k<sqrFiltLength; k++){
        if (weights[k]<=diff){
          for (i=0; i<sqrFiltLength; i++){
            filterCoeffQuantMod[i]=filterCoeffQuant[i];
          }
          filterCoeffQuantMod[k]++;
#if WIENER_3_INPUT
          for (i=0; i<sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_rec;
          }
          for (i=sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i<sqrFiltLength-sqrFiltLength_pred-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_resi;
          }
          for (i=sqrFiltLength-sqrFiltLength_pred-1; i<sqrFiltLength-1; i++)
          {
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor_pred;
          }  
          filterCoeff[sqrFiltLength-1]=(double)filterCoeffQuantMod[sqrFiltLength-1]/(double)factor_rec;          
#else          
          for (i=0; i<sqrFiltLength; i++){
            filterCoeff[i]=(double)filterCoeffQuantMod[i]/(double)factor;
          }
#endif
          error=calculateErrorCoeffProvided(E, y, filterCoeff, sqrFiltLength);
          if (error<errMin || minInd==-1){
            errMin=error;
            minInd=k;
          }
        } // if (weights(k)<=diff){
      } // for (k=0; k<sqrFiltLength; k++){
      filterCoeffQuant[minInd]++;
    }

    quantCoeffSum=0;
    for (i=0; i<sqrFiltLength; i++){
      quantCoeffSum+=weights[i]*filterCoeffQuant[i];
    }
  }

#if (ALF_MEM_PATCH || WIENER_3_INPUT)
  if (count == 10)
  {
    for (i=0; i<sqrFiltLength; i++)
   {
    filterCoeffQuant[i] = 0;
    }
  }
#endif
#if WIENER_3_INPUT  
  for (i=0; i<sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
  {
    filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor_rec;
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i<sqrFiltLength-sqrFiltLength_pred-1; i++)
  {
    filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor_resi;
  }
  for (i=sqrFiltLength-sqrFiltLength_pred-1; i<sqrFiltLength-1; i++)
  {
    filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor_pred;
  }  
  filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor_rec;
#else
  for (i=0; i<sqrFiltLength; i++)
  {
    filterCoeff[i]=(double)filterCoeffQuant[i]/(double)factor;
  }
#endif  
   error=calculateErrorCoeffProvided(E, y, filterCoeff, sqrFiltLength);
  return(error);

}


#if WIENER_3_INPUT    
Double TEncAdaptiveLoopFilter::findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq, int **filterCoeffQuantSeq, int intervalBest[NO_VAR_BINS][2], int varIndTab[NO_VAR_BINS], int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int filters_per_fr, int *weights, int shift_rec, int shift_pred, int shift_resi, double errorTabForce0Coeff[NO_VAR_BINS][2])
#else
Double TEncAdaptiveLoopFilter::findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq, int **filterCoeffQuantSeq, int intervalBest[NO_VAR_BINS][2], int varIndTab[NO_VAR_BINS], int sqrFiltLength, int filters_per_fr, int *weights, int bit_depth, double errorTabForce0Coeff[NO_VAR_BINS][2])
#endif    
{
#if ALF_MEM_PATCH
  static double pixAcc_temp;
#else
  static int init = 0;
  static double **E_temp, *y_temp, *filterCoeff, pixAcc_temp;
  static int *filterCoeffQuant;
#endif
  double error;
  int k, filtNo;


#if !ALF_MEM_PATCH
  if(init == 0)
  {
    initMatrix_double(&E_temp, MAX_SQR_FILT_LENGTH, MAX_SQR_FILT_LENGTH);
    y_temp = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));
    filterCoeff = (double *) calloc(MAX_SQR_FILT_LENGTH, sizeof(double));
    filterCoeffQuant = (int *) calloc(MAX_SQR_FILT_LENGTH, sizeof(int));
    init = 1;
  }
#endif

  error = 0;
  for(filtNo = 0; filtNo < filters_per_fr; filtNo++)
  {
    add_A(E_temp, EGlobalSeq, intervalBest[filtNo][0], intervalBest[filtNo][1], sqrFiltLength);
    add_b(y_temp, yGlobalSeq, intervalBest[filtNo][0], intervalBest[filtNo][1], sqrFiltLength);

    pixAcc_temp = 0;    
    for(k = intervalBest[filtNo][0]; k <= intervalBest[filtNo][1]; k++)
      pixAcc_temp += pixAccGlobalSeq[k];
    
    // Find coeffcients
#if WIENER_3_INPUT    
    errorTabForce0Coeff[filtNo][1] = pixAcc_temp + QuantizeIntegerFilterPP(filterCoeff, filterCoeffQuant, E_temp, y_temp, sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, weights, shift_rec, shift_pred, shift_resi);
#else    
    errorTabForce0Coeff[filtNo][1] = pixAcc_temp + QuantizeIntegerFilterPP(filterCoeff, filterCoeffQuant, E_temp, y_temp, sqrFiltLength, weights, bit_depth);
#endif    
    errorTabForce0Coeff[filtNo][0] = pixAcc_temp;
    error += errorTabForce0Coeff[filtNo][1];

    for(k = 0; k < sqrFiltLength; k++)
    {
      filterCoeffSeq[filtNo][k] = filterCoeffQuant[k];
      filterCoeffQuantSeq[filtNo][k] = filterCoeffQuant[k];
    }
  }
  
  for(filtNo = 0; filtNo < filters_per_fr; filtNo++)
    for(k = intervalBest[filtNo][0]; k <= intervalBest[filtNo][1]; k++)
      varIndTab[k] = filtNo;

  return(error);
}



#endif
#endif




#endif //WIENER_3_INPUT

#if WIENER_3_INPUT    
//-----------------------------------------------------------------------------
/*!
* \brief Solve Wiener Hopf equations
*
* \param[in] **a                  % pointer to 2D autocorrelation matrix
* \param[in]  *x                  % pointer to 1D coefficient vector
* \param[in]  *b                  % pointer to 1D crosscorrelation vector
* \param[in]   n                  % dimension of matrix and vectors
*/
//-----------------------------------------------------------------------------
void TEncAdaptiveLoopFilter::sgaus (double **a, double *x, double *b, int n)
{
  int i, j, k;
  double q, s;

  for (k = 0; k < n - 1; k++)
  {
    if (a[k][k] != 0.0)
    {
      for (i = k + 1; i < n; i++)
      {
        q = a[i][k] / a[k][k];
        for (j = k + 1; j <= i; j++)
          a[i][j] -= q * a[j][k];
        b[i] -= q * b[k];
      }
    }
  }

  if (a[n - 1][n - 1] == 0.0)   
    x[n - 1] = 0.0;
  else
    x[n - 1] = b[n - 1] / a[n - 1][n - 1];
  for (k = n - 2; k >= 0; k--)
  {
    if (a[k][k] == 0.0)
    {                          
      x[k] = 0.0;
    }
    else
    {
      s = b[k];
      for (j = k + 1; j < n; j++)
        s -= a[j][k] * x[j];
      x[k] = s / a[k][k];
    }
  }
}
#endif
