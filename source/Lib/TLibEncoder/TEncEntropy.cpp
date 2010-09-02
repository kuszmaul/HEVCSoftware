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

/** \file     TEncEntropy.cpp
    \brief    entropy encoder class
*/

#include "TEncEntropy.h"

Void TEncEntropy::setEntropyCoder ( TEncEntropyIf* e, TComSlice* pcSlice )
{
  m_pcEntropyCoderIf = e;
  m_pcEntropyCoderIf->setSlice ( pcSlice );
}

Void TEncEntropy::encodeSliceHeader ( TComSlice* pcSlice )
{
  m_pcEntropyCoderIf->codeSliceHeader( pcSlice );
  return;
}

Void TEncEntropy::encodeTerminatingBit      ( UInt uiIsLast )
{
  m_pcEntropyCoderIf->codeTerminatingBit( uiIsLast );

  return;
}

Void TEncEntropy::encodeSliceFinish()
{
  m_pcEntropyCoderIf->codeSliceFinish();
}

Void TEncEntropy::encodePPS( TComPPS* pcPPS )
{
  m_pcEntropyCoderIf->codePPS( pcPPS );
  return;
}

Void TEncEntropy::encodeSPS( TComSPS* pcSPS )
{
  m_pcEntropyCoderIf->codeSPS( pcSPS );
  return;
}

Void TEncEntropy::encodeSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
#if HHI_MRG
  if ( pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    return;
  }
#endif

  if( bRD )
    uiAbsPartIdx = 0;

  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  m_pcEntropyCoderIf->codeSkipFlag( pcCU, uiAbsPartIdx );
}
#if QC_ALF
#include "../TLibCommon/TypeDef.h"
#include "../TLibCommon/TComAdaptiveLoopFilter.h"
Void TEncEntropy::codeFiltCountBit(ALFParam* pAlfParam, Int64* ruiRate)
{
    resetEntropy();
    resetBits();
    codeFilt(pAlfParam);
    *ruiRate = getNumberOfWrittenBits();
    resetEntropy();
    resetBits();
}

Void TEncEntropy::codeAuxCountBit(ALFParam* pAlfParam, Int64* ruiRate)
{
    resetEntropy();
    resetBits();
    codeAux(pAlfParam);
    *ruiRate = getNumberOfWrittenBits();
    resetEntropy();
    resetBits();
}

Void TEncEntropy::codeAux(ALFParam* pAlfParam)
{
#if ENABLE_FORCECOEFF0
  if (pAlfParam->filtNo>=0) m_pcEntropyCoderIf->codeAlfFlag(1);
  else m_pcEntropyCoderIf->codeAlfFlag(0);
#endif
  Int FiltTab[3] = {9, 7, 5};
  Int Tab = FiltTab[pAlfParam->realfiltNo];
//  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->realfiltNo); 
  m_pcEntropyCoderIf->codeAlfUvlc((Tab-5)/2); 
#if WIENER_3_INPUT
  m_pcEntropyCoderIf->codeAlfUvlc((pAlfParam->tap_pred-1)/2); 
  m_pcEntropyCoderIf->codeAlfUvlc((pAlfParam->tap_resi-1)/2); 
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[0]);
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[0]-pAlfParam->filter_precision[1]);
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[0]-pAlfParam->filter_precision[2]);
#endif
  if (pAlfParam->filtNo>=0)
  {
    if(pAlfParam->realfiltNo >= 0)
    {
      // filters_per_fr
      m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->noFilters);
  
      if(pAlfParam->noFilters == 1)
      {
        m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->startSecondFilter);
      }
      else if (pAlfParam->noFilters == 2)
      {
        for (int i=1; i<NO_VAR_BINS; i++) m_pcEntropyCoderIf->codeAlfFlag (pAlfParam->filterPattern[i]);
      }
    }
  }
}

Int TEncEntropy::lengthGolomb(int coeffVal, int k)
{
  int m = 2 << (k - 1);
  int q = coeffVal / m;
  if(coeffVal != 0)
    return(q + 2 + k);
  else
    return(q + 1 + k);
}

Int TEncEntropy::codeFilterCoeff(ALFParam* ALFp)
{
  int filters_per_group = ALFp->filters_per_group_diff;
  int sqrFiltLength = ALFp->num_coeff;
  int filtNo = ALFp->realfiltNo;
  int flTab[]={9/2, 7/2, 5/2};
  int fl = flTab[filtNo];
  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal, len = 0,
    *pDepthInt=NULL, kMinTab[MAX_SQR_FILT_LENGTH], bitsCoeffScan[MAX_SCAN_VAL][MAX_EXP_GOLOMB],
    minKStart, minBitsKStart, bitsKStart;
#if WIENER_3_INPUT
  int two_codes=0;
  int bitsCoeffScan_pr[MAX_SCAN_VAL][MAX_EXP_GOLOMB];
  int kMinTab_pr[MAX_SQR_FILT_LENGTH];
  int maxScanVal_pr;
  int sqrFiltLength_pred = ALFp->num_coeff_pred;
  int sqrFiltLength_resi = ALFp->num_coeff_resi;
  int filtNo_resi = (ALFp->tap_resi-1)/2;
  int filtNo_pred = (ALFp->tap_pred-1)/2;
  int sqrFiltLengthTab_pr[NO_TEST_FILT+2]={SQR_FILT_LENGTH_1SYM, SQR_FILT_LENGTH_3SYM, SQR_FILT_LENGTH_5SYM-1, SQR_FILT_LENGTH_7SYM-1, SQR_FILT_LENGTH_9SYM-1}; 
  int sqrFiltLengthTab[NO_TEST_FILT]={SQR_FILT_LENGTH_9SYM, SQR_FILT_LENGTH_7SYM, SQR_FILT_LENGTH_5SYM};

  for (i=0; i<sqrFiltLengthTab[filtNo]-1; i++)
    depth[i]=pDepthIntTab[2-filtNo][i];               
  k=i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_resi]; i++)
    depth[k+i]=pDepthIntTab_pr[filtNo_resi][i];               
  k=k+i;
  for (i=0; i<sqrFiltLengthTab_pr[filtNo_pred]; i++)
    depth[k+i]=pDepthIntTab_pr[filtNo_pred][i];
  depth[k+i] = pDepthIntTab[2-filtNo][sqrFiltLengthTab[filtNo]-1]; //DC

  pDepthInt = depth;
#else
  pDepthInt = pDepthIntTab[fl-2];
#endif
  maxScanVal = 0;
#if WIENER_3_INPUT
  maxScanVal_pr = 0;
  if (ALFp->num_coeff > WIENER_3_INPUT_ADAPTIVE_GOLOMB_TS && ALFp->filter_precision[0]>WIENER_3_INPUT_ADAPTIVE_GOLOMB_TP)
  {
    two_codes=1;
  }
  if (two_codes)
  {
    for(i = 0; i < sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++)
      maxScanVal = max(maxScanVal, pDepthInt[i]);
    maxScanVal = max(maxScanVal, pDepthInt[sqrFiltLength-1]);//DC
    for(i = sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i < sqrFiltLength-1; i++)
      maxScanVal_pr = max(maxScanVal_pr, pDepthInt[i]);
  }
  else
  {
#endif
  for(i = 0; i < sqrFiltLength; i++)
    maxScanVal = max(maxScanVal, pDepthInt[i]);
#if WIENER_3_INPUT
  }
#endif      
  
  // vlc for all
  memset(bitsCoeffScan  , 0, MAX_SCAN_VAL * MAX_EXP_GOLOMB * sizeof(int));
#if WIENER_3_INPUT
  memset(bitsCoeffScan_pr, 0, MAX_SCAN_VAL * MAX_EXP_GOLOMB * sizeof(int));
#endif  
  for(ind=0; ind<filters_per_group; ++ind){	
#if WIENER_3_INPUT
    if (two_codes)
    {
      for(i = 0; i < sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i++){            
        scanPos=pDepthInt[i]-1;
        coeffVal=abs(ALFp->coeffmulti[ind][i]);
        for (k=1; k<15; k++){
          bitsCoeffScan[scanPos][k]+=lengthGolomb(coeffVal, k);
        }
      }
      scanPos=pDepthInt[sqrFiltLength-1]-1;//DC
      coeffVal=abs(ALFp->coeffmulti[ind][sqrFiltLength-1]);
      for (k=1; k<15; k++){
        bitsCoeffScan[scanPos][k]+=lengthGolomb(coeffVal, k);
      }
        

      for(i = sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1; i < sqrFiltLength-1; i++){
        scanPos=pDepthInt[i]-1;
        coeffVal=abs(ALFp->coeffmulti[ind][i]);
        for (k=1; k<15; k++){
          bitsCoeffScan_pr[scanPos][k]+=lengthGolomb(coeffVal, k);
        }
      }      
    }
    else
    {    
#endif
      for(i = 0; i < sqrFiltLength; i++){	     
        scanPos=pDepthInt[i]-1;
        coeffVal=abs(ALFp->coeffmulti[ind][i]);
        for (k=1; k<15; k++){
          bitsCoeffScan[scanPos][k]+=lengthGolomb(coeffVal, k);
        }
      }
#if WIENER_3_INPUT
    }
#endif      
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
  ALFp->minKStart = minKStart;
  ALFp->maxScanVal = maxScanVal;
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
     ALFp->kMinTab[scanPos] = kMinTab[scanPos];
  len += writeFilterCodingParams(minKStart, maxScanVal, kMinTab);
#if WIENER_3_INPUT
  if (two_codes)
  {
    minBitsKStart = 0;
    minKStart = -1;
    for(k = 1; k < 8; k++)
    { 
      bitsKStart = 0; 
      kStart = k;
      for(scanPos = 0; scanPos < maxScanVal_pr; scanPos++)
      {
        kMin = kStart; 
        minBits = bitsCoeffScan_pr[scanPos][kMin];
  
        if(bitsCoeffScan_pr[scanPos][kStart+1] < minBits)
        {
          kMin = kStart + 1; 
          minBits = bitsCoeffScan_pr[scanPos][kMin];
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
    for(scanPos = 0; scanPos < maxScanVal_pr; scanPos++)
    {
      kMin = kStart; 
      minBits = bitsCoeffScan_pr[scanPos][kMin];
  
      if(bitsCoeffScan_pr[scanPos][kStart+1] < minBits)
      {
        kMin = kStart + 1; 
        minBits = bitsCoeffScan_pr[scanPos][kMin];
      }
  
      kMinTab_pr[scanPos] = kMin;
      kStart = kMin;
    }
    // Coding parameters
    ALFp->minKStart_pr = minKStart;
    ALFp->maxScanVal_pr = maxScanVal_pr;
    for(scanPos = 0; scanPos < maxScanVal_pr; scanPos++)
      ALFp->kMinTab_pr[scanPos] = kMinTab_pr[scanPos];
    len += writeFilterCodingParams(minKStart, maxScanVal_pr, kMinTab_pr);  
  }
#endif
  // Filter coefficients
#if WIENER_3_INPUT
  len += writeFilterCoeffs(sqrFiltLength, sqrFiltLength_pred, sqrFiltLength_resi, filters_per_group, pDepthInt, ALFp->coeffmulti, kMinTab, kMinTab_pr, two_codes);
#else  
  len += writeFilterCoeffs(sqrFiltLength, filters_per_group, pDepthInt, ALFp->coeffmulti, kMinTab);
#endif

  return len;
}

Int TEncEntropy::writeFilterCodingParams(int minKStart, int maxScanVal, int kMinTab[])
{
  int scanPos;
  int golombIndexBit;
  int kMin;

  // Golomb parameters
  m_pcEntropyCoderIf->codeAlfUvlc(minKStart - 1);

  kMin = minKStart; 
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
    golombIndexBit = (kMinTab[scanPos] != kMin)? 1: 0;

    assert(kMinTab[scanPos] <= kMin + 1);

    m_pcEntropyCoderIf->codeAlfFlag(golombIndexBit);
    kMin = kMinTab[scanPos];
  }    

  return 0;
}

#if WIENER_3_INPUT
Int TEncEntropy::writeFilterCoeffs(int sqrFiltLength, int sqrFiltLength_pred, int sqrFiltLength_resi, int filters_per_group, int pDepthInt[], 
                      int **FilterCoeff, int kMinTab[], int kMinTab_pr[], int two_codes)
#else                                   
Int TEncEntropy::writeFilterCoeffs(int sqrFiltLength, int filters_per_group, int pDepthInt[], 
                      int **FilterCoeff, int kMinTab[])
#endif
{
  int ind, scanPos, i;
#if WIENER_3_INPUT
  int tmp, tmp2;
#endif
  for(ind = 0; ind < filters_per_group; ++ind)
  {
#if WIENER_3_INPUT
    int recon_filt = sqrFiltLength-sqrFiltLength_pred-sqrFiltLength_resi-1;
    int resi_filt = sqrFiltLength_resi;
    int pred_filt = sqrFiltLength_pred;
    int recon_zeroflag = 1;
    int resi_zeroflag = 1;
    int pred_zeroflag = 1;
    for(i = 0; i < recon_filt; i++)
    {
      if (FilterCoeff[ind][i]!= 0) recon_zeroflag = 0;
    }
    for(i = recon_filt; i < recon_filt + resi_filt; i++)
    {
      if (FilterCoeff[ind][i]!= 0) resi_zeroflag = 0;
    }
    for(i = recon_filt + resi_filt; i < recon_filt + resi_filt + pred_filt; i++)
    {
      if (FilterCoeff[ind][i]!= 0) pred_zeroflag = 0;
    }

    m_pcEntropyCoderIf->codeAlfFlag(recon_zeroflag);
    m_pcEntropyCoderIf->codeAlfFlag(resi_zeroflag);
    m_pcEntropyCoderIf->codeAlfFlag(pred_zeroflag);
    if (two_codes)
    {
      for(i = 0; i < recon_filt; i++)
      {
        if (!recon_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
        }
      }
      for(i = recon_filt; i < recon_filt + resi_filt; i++)
      {
        if (!resi_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab_pr[scanPos]);
        }
      }
      for(i = recon_filt + resi_filt; i < recon_filt + resi_filt + pred_filt; i++)
      {
        if (!pred_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab_pr[scanPos]);
        }
      }
  	
      i = sqrFiltLength-1;
      scanPos = pDepthInt[i] - 1;
      golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
    }
    else
    {
      for(i = 0; i < recon_filt; i++)
      {
        if (!recon_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
        }
      }
      for(i = recon_filt; i < recon_filt + resi_filt; i++)
      {
        if (!resi_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
        }
      }
      for(i = recon_filt + resi_filt; i < recon_filt + resi_filt + pred_filt; i++)
      {
        if (!pred_zeroflag)
        {
          scanPos = pDepthInt[i] - 1;
          golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
        }
      }
    
      i = sqrFiltLength-1;
      scanPos = pDepthInt[i] - 1;
      golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
    }
#else
    for(i = 0; i < sqrFiltLength; i++)
    {
      scanPos = pDepthInt[i] - 1;
      golombEncode(FilterCoeff[ind][i], kMinTab[scanPos]);
    }
#endif
  }
  return 0;
}

Int TEncEntropy::golombEncode(int coeff, int k)
{
  int q, i, m;
  int symbol = abs(coeff);

  m = (int)pow(2.0, k);
  q = symbol / m;

  for (i = 0; i < q; i++)
    m_pcEntropyCoderIf->codeAlfFlag(1);
  m_pcEntropyCoderIf->codeAlfFlag(0);
      // write one zero

  for(i = 0; i < k; i++)
  {
    m_pcEntropyCoderIf->codeAlfFlag(symbol & 0x01);
    symbol >>= 1;
  }

  if(coeff != 0)
  {
    int sign = (coeff > 0)? 1: 0;
    m_pcEntropyCoderIf->codeAlfFlag(sign);
  }
  return 0;
}

Void TEncEntropy::codeFilt(ALFParam* pAlfParam)
{
  if(pAlfParam->filters_per_group > 1)
  {
#if ENABLE_FORCECOEFF0
    m_pcEntropyCoderIf->codeAlfFlag (pAlfParam->forceCoeff0);
    if (pAlfParam->forceCoeff0)
    {
      for (int i=0; i<pAlfParam->filters_per_group; i++) 
        m_pcEntropyCoderIf->codeAlfFlag (pAlfParam->codedVarBins[i]);
    }
#endif 
    m_pcEntropyCoderIf->codeAlfFlag (pAlfParam->predMethod);
  }
  codeFilterCoeff (pAlfParam);
}
Void  print(ALFParam* pAlfParam)
{
  Int i=0;
  Int ind=0;
  Int FiltLengthTab[] = {22, 14, 8}; //0:9tap
  Int FiltLength = FiltLengthTab[pAlfParam->realfiltNo];

  printf("set of params\n");
  printf("realfiltNo:%d\n", pAlfParam->realfiltNo);
  printf("filtNo:%d\n", pAlfParam->filtNo);
  printf("filterPattern:");
  for (i=0; i<NO_VAR_BINS; i++) printf("%d ", pAlfParam->filterPattern[i]);
  printf("\n");
  
  printf("startSecondFilter:%d\n", pAlfParam->startSecondFilter);
  printf("noFilters:%d\n", pAlfParam->noFilters);
  printf("varIndTab:");
  for (i=0; i<NO_VAR_BINS; i++) printf("%d ", pAlfParam->varIndTab[i]);
  printf("\n");
  printf("filters_per_group_diff:%d\n", pAlfParam->filters_per_group_diff);
  printf("filters_per_group:%d\n", pAlfParam->filters_per_group);
  printf("codedVarBins:");
  for (i=0; i<NO_VAR_BINS; i++) printf("%d ", pAlfParam->codedVarBins[i]);
  printf("\n");
  printf("forceCoeff0:%d\n", pAlfParam->forceCoeff0);
  printf("predMethod:%d\n", pAlfParam->predMethod);

  for (ind=0; ind<pAlfParam->filters_per_group_diff; ind++)
  {
    printf("coeffmulti(%d):", ind);
	for (i=0; i<FiltLength; i++) printf("%d ", pAlfParam->coeffmulti[ind][i]);
	printf("\n");
  }

  printf("minKStart:%d\n", pAlfParam->minKStart);  
  printf("maxScanVal:%d\n", pAlfParam->maxScanVal);  
  printf("kMinTab:");
  for(Int scanPos = 0; scanPos < pAlfParam->maxScanVal; scanPos++)
  {
     printf("%d ", pAlfParam->kMinTab[scanPos]);
  }
  printf("\n");

  printf("chroma_idc:%d\n", pAlfParam->chroma_idc);  
  printf("tap_chroma:%d\n", pAlfParam->tap_chroma);  
  printf("chroma_coeff:");
  for(Int scanPos = 0; scanPos < pAlfParam->num_coeff_chroma; scanPos++)
  {
     printf("%d ", pAlfParam->coeff_chroma[scanPos]);
  }
  printf("\n");
}
#endif


#if HHI_MRG
Void TEncEntropy::encodeMergeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }
  
  m_pcEntropyCoderIf->codeMergeFlag( pcCU, uiAbsPartIdx );
}
  
Void TEncEntropy::encodeMergeIndex( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeMergeIndex( pcCU, uiAbsPartIdx );
}
#endif

#if (WIENER_3_INPUT && !QC_ALF)
Int TEncEntropy::lengthGolomb(int coeffVal, int k)
{
  int m = 2 << k;
  int q = coeffVal / m;
  if(coeffVal != 0)
    return(q + k + 1);
  else
    return(q + k);

}

Void TEncEntropy::encodeAlfParam_control_data(ALFParam* pAlfParam, Int component)
{
  m_pcEntropyCoderIf->codeAlfUvlc( (pAlfParam->filter_length_RD_rec [component] -1)/2 );
  m_pcEntropyCoderIf->codeAlfUvlc( (pAlfParam->filter_length_RD_pred[component] -1)/2 );
  m_pcEntropyCoderIf->codeAlfUvlc( (pAlfParam->filter_length_RD_qpe [component] -1)/2 );
  
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[component][0]);
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[component][0]-pAlfParam->filter_precision[component][1]);
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->filter_precision[component][0]-pAlfParam->filter_precision[component][2]);
}
  
Void TEncEntropy::encodeAlfParam_golomb(ALFParam* pAlfParam, Int component)
{
  Int size_rec    = pAlfParam->filter_length_RD_rec [component];
  Int size_pred   = pAlfParam->filter_length_RD_pred[component];
  Int size_qpe    = pAlfParam->filter_length_RD_qpe [component];
  Int prec        = pAlfParam->filter_precision[component][0];
  Int prec2       = pAlfParam->filter_precision[component][1];
  Int length      = size_rec *size_rec + size_pred*size_pred+ size_qpe *size_qpe + 1;
  
  if (length>19 && prec>4)//only worth for a lot of coefficients of higher precision
  {
    m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->golomb_enable[component]);
    if (0 != pAlfParam->golomb_enable[component])
    {
      m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->golomb_code[component][0]);
      if (length>39 && size_rec<5 && prec2>4)//only worth for a lot of coefficients of higher precision
      {
        m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->golomb_code[component][1]);
      }
    }
  }
}

Void TEncEntropy::encodeAlfParam_coeffs(ALFParam* pAlfParam, Int component)
{
  Int m,n,i;
  Int *i_p;
  Int *i_p_dont_care;
  Int tmp=0, tmp2=0;
  
  Int bits;
  Int k_min, bits_min;
  
  Int *symbols_to_code_rec;
  Int *symbols_to_code_pred;
  Int *symbols_to_code_qpe;
  Int *p_symbols_to_code_rec;
  Int *p_symbols_to_code_pred;
  Int *p_symbols_to_code_qpe;
  
  Int size_rec    = pAlfParam->filter_length_RD_rec [component];
  Int size_pred   = pAlfParam->filter_length_RD_pred[component];
  Int size_qpe    = pAlfParam->filter_length_RD_qpe [component];
  Int length_rec  = size_rec *size_rec +1;
  Int length_pred = size_pred*size_pred;
  Int length_qpe  = size_qpe *size_qpe;
  Int length      = length_rec+length_pred+length_qpe;
  
  Int prec        = pAlfParam->filter_precision[component][0];
  Int prec2       = pAlfParam->filter_precision[component][1];
  
  
  i_p=(pAlfParam->coeffs)[component];
  i_p_dont_care=(pAlfParam->dont_care)[component];
  
  symbols_to_code_rec  = new Int[length_rec ];
  symbols_to_code_pred = new Int[length_pred];
  symbols_to_code_qpe  = new Int[length_qpe ];
  p_symbols_to_code_rec  = &(symbols_to_code_rec [0]);
  p_symbols_to_code_pred = &(symbols_to_code_pred[0]);
  p_symbols_to_code_qpe  = &(symbols_to_code_qpe [0]);
  
  
  for (m=0;m<size_rec;m++)
  {
    for (n=0;n<size_rec;n++)
    {
      if (m==(size_rec-1)/2 && n==(size_rec-1)/2)
      {
        if (*i_p_dont_care == 0)
        {
          tmp=(*i_p)-pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][0]];
        }
        else
        {
          tmp=0;
        }
        *p_symbols_to_code_rec++ = tmp;
      }
      else
      {
        if (*i_p_dont_care == 0)
        {
          *p_symbols_to_code_rec++ = *i_p;
        }
        else
        {
          *p_symbols_to_code_rec++ = 0;
        }
      }
      i_p++;
      i_p_dont_care++;
    }
  }
  
  for (m=0;m<size_pred;m++)
  {
    for (n=0;n<size_pred;n++)
    {
      if (m==(size_pred-1)/2 && n==(size_pred-1)/2)
      {
        if (*i_p_dont_care == 0)
        {
          tmp*=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][1]];
          tmp/=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][0]];
          *p_symbols_to_code_pred++ =  (*i_p) + tmp;
          tmp2=(*i_p);
        }
        else
        {
          *p_symbols_to_code_pred++ =  0;
          tmp2=(-tmp);
        }
      }
      else
      {
        if (*i_p_dont_care == 0)
        {
          *p_symbols_to_code_pred++ =  *i_p;
        }
        else
        {
          *p_symbols_to_code_pred++ =  0;
        }
      }
      i_p++;
      i_p_dont_care++;
    }
  }
  
  for (m=0;m<size_qpe;m++)
  {
    for (n=0;n<size_qpe;n++)
    {
      if (m==(size_qpe-1)/2 && n==(size_qpe-1)/2)
      {
        if (*i_p_dont_care == 0)
        {
          tmp2*=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][2]];
          tmp2/=pAlfParam->filter_precision_table[pAlfParam->filter_precision[component][1]];
          *p_symbols_to_code_qpe++ =  (*i_p)-tmp2;
        }
        else
        {
          *p_symbols_to_code_qpe++ =  0;
        }
      }
      else
      {
        if (*i_p_dont_care == 0)
        {
          *p_symbols_to_code_qpe++ =  *i_p;
        }
        else
        {
          *p_symbols_to_code_qpe++ =  0;
        }
      }
      i_p++;
      i_p_dont_care++;
    }
  }

  if (*i_p_dont_care == 0)
  {
    *p_symbols_to_code_rec++ =  *i_p;
  }
  else
  {
    *p_symbols_to_code_rec++ =  0;
  }
  
  
  if (pAlfParam->golomb_enable[component] && length>19 && prec>4)
  {
    Int code=0;
    for (m=0;m<length_rec;m++)
    {
      m_pcEntropyCoderIf->golombEncode(symbols_to_code_rec[m], pAlfParam->golomb_code[component][0]);
    }
    if (length>39 && size_rec<5 && prec2>4)//only worth for a lot of coefficients of higher precision
    {
      code=1;
    }
    for (m=0;m<length_pred;m++)
    {
      m_pcEntropyCoderIf->golombEncode(symbols_to_code_pred[m], pAlfParam->golomb_code[component][code]);
    }
    for (m=0;m<length_qpe;m++)
    {
      m_pcEntropyCoderIf->golombEncode(symbols_to_code_qpe[m], pAlfParam->golomb_code[component][code]);
    }
  }
  else
  {
    for (m=0;m<length_rec;m++)
    {
      m_pcEntropyCoderIf->codeAlfSvlc(symbols_to_code_rec[m]);
    }
    for (m=0;m<length_pred;m++)
    {
      m_pcEntropyCoderIf->codeAlfSvlc(symbols_to_code_pred[m]);
    }
    for (m=0;m<length_qpe;m++)
    {
      m_pcEntropyCoderIf->codeAlfSvlc(symbols_to_code_qpe[m]);
    }    
  }
    
  delete[] symbols_to_code_rec;
  delete[] symbols_to_code_pred;
  delete[] symbols_to_code_qpe;
}

Void TEncEntropy::encodeAlfParam(ALFParam* pAlfParam)
{
  m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->alf_flag);
  if (!pAlfParam->alf_flag)
    return;
  
  for (Int c=0; c<3; c++)
  {
    m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->enable_flag[c]);
    if (pAlfParam->enable_flag[c])
    {
      encodeAlfParam_control_data(pAlfParam, c);
      encodeAlfParam_golomb      (pAlfParam, c);
      encodeAlfParam_coeffs      (pAlfParam, c);
    }
  }
}
#else
Void TEncEntropy::encodeAlfParam(ALFParam* pAlfParam)
{
#if HHI_ALF
  m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->alf_flag);
  if (!pAlfParam->alf_flag)
    return;
  Int pos;
  Int iCenterPos ;

  // filter parameters for luma
  // horizontal filter
  AlfFilter *pHorizontalFilter = &(pAlfParam->acHorizontalAlfFilter[0]) ;
  iCenterPos = ( pHorizontalFilter->iFilterSymmetry == 0 ) ? (pHorizontalFilter->iFilterLength + 1) >> 1 : pHorizontalFilter->iNumOfCoeffs - 1 ;

  m_pcEntropyCoderIf->codeAlfUvlc( (pHorizontalFilter->iFilterLength-ALF_MIN_LENGTH)/2 );
  m_pcEntropyCoderIf->codeAlfUvlc( pHorizontalFilter->iFilterSymmetry );

  Int iCoeff;
  for(pos=0; pos < pHorizontalFilter->iNumOfCoeffs; pos++)
  {
    iCoeff = pHorizontalFilter->aiQuantFilterCoeffs[pos] ;
    m_pcEntropyCoderIf->codeAlfCoeff(iCoeff,pHorizontalFilter->iFilterLength, pos );
  }
#if ALF_DC_CONSIDERED
  m_pcEntropyCoderIf->codeAlfDc( pHorizontalFilter->aiQuantFilterCoeffs[ pHorizontalFilter->iNumOfCoeffs ] );
#endif
  // vertical filter
  AlfFilter *pVerticalFilter = &(pAlfParam->acVerticalAlfFilter[0]) ;

  m_pcEntropyCoderIf->codeAlfUvlc( (pVerticalFilter->iFilterLength-ALF_MIN_LENGTH)/2 );
  m_pcEntropyCoderIf->codeAlfUvlc( pVerticalFilter->iFilterSymmetry );

  iCenterPos = ( pVerticalFilter->iFilterSymmetry == 0 ) ? (pVerticalFilter->iFilterLength + 1) >> 1 : pVerticalFilter->iNumOfCoeffs - 1 ;
  for(pos=0; pos < pVerticalFilter->iNumOfCoeffs ; pos++ )
  {
    iCoeff = pVerticalFilter->aiQuantFilterCoeffs[pos] ;
    m_pcEntropyCoderIf->codeAlfCoeff(iCoeff,pVerticalFilter->iFilterLength, pos );
  }
#if ALF_DC_CONSIDERED
  m_pcEntropyCoderIf->codeAlfDc( pVerticalFilter->aiQuantFilterCoeffs[ pVerticalFilter->iNumOfCoeffs ] );
#endif
  // filter parameters for chroma
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->chroma_idc);
  for(Int iPlane = 1; iPlane <3; iPlane++)
  {
    if(pAlfParam->chroma_idc&iPlane)
    {
      m_pcEntropyCoderIf->codeAlfUvlc( pAlfParam->aiPlaneFilterMapping[iPlane] ) ;
      if( pAlfParam->aiPlaneFilterMapping[iPlane] == iPlane )
      {
        // horizontal filter
        pHorizontalFilter = &(pAlfParam->acHorizontalAlfFilter[iPlane]) ;
        iCenterPos = ( pHorizontalFilter->iFilterSymmetry == 0 ) ? (pHorizontalFilter->iFilterLength + 1) >> 1 : pHorizontalFilter->iNumOfCoeffs - 1 ;

        m_pcEntropyCoderIf->codeAlfUvlc( (pHorizontalFilter->iFilterLength-ALF_MIN_LENGTH)/2 );
        m_pcEntropyCoderIf->codeAlfUvlc( pHorizontalFilter->iFilterSymmetry );

        for(pos=0; pos < pHorizontalFilter->iNumOfCoeffs; pos++)
        {
          iCoeff = pHorizontalFilter->aiQuantFilterCoeffs[pos] ;
          m_pcEntropyCoderIf->codeAlfCoeff(iCoeff,pHorizontalFilter->iFilterLength, pos );
        }
#if ALF_DC_CONSIDERED
        m_pcEntropyCoderIf->codeAlfDc( pHorizontalFilter->aiQuantFilterCoeffs[ pHorizontalFilter->iNumOfCoeffs ] );
#endif
        // vertical filter
        pVerticalFilter = &(pAlfParam->acVerticalAlfFilter[iPlane]) ;

        m_pcEntropyCoderIf->codeAlfUvlc( (pVerticalFilter->iFilterLength-ALF_MIN_LENGTH)/2 );
        m_pcEntropyCoderIf->codeAlfUvlc( pVerticalFilter->iFilterSymmetry );

        iCenterPos = ( pVerticalFilter->iFilterSymmetry == 0 ) ? (pVerticalFilter->iFilterLength + 1) >> 1 : pVerticalFilter->iNumOfCoeffs - 1 ;
        for(pos=0; pos < pVerticalFilter->iNumOfCoeffs ; pos++ )
        {
          iCoeff = pVerticalFilter->aiQuantFilterCoeffs[pos] ;
    			m_pcEntropyCoderIf->codeAlfCoeff(iCoeff,pVerticalFilter->iFilterLength, pos );
        }
#if ALF_DC_CONSIDERED
        m_pcEntropyCoderIf->codeAlfDc( pVerticalFilter->aiQuantFilterCoeffs[ pVerticalFilter->iNumOfCoeffs ] );
#endif
      }
    }
  }

  // region control parameters for luma
  m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->cu_control_flag);
  if (pAlfParam->cu_control_flag)
  {
    assert( (pAlfParam->cu_control_flag && m_pcEntropyCoderIf->getAlfCtrl()) || (!pAlfParam->cu_control_flag && !m_pcEntropyCoderIf->getAlfCtrl()));
    m_pcEntropyCoderIf->codeAlfFlag( pAlfParam->bSeparateQt );
    m_pcEntropyCoderIf->codeAlfCtrlDepth();
  }
#else
  m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->alf_flag);
  if (!pAlfParam->alf_flag)
    return;
  Int pos;
#if QC_ALF  
  codeAux(pAlfParam);
  codeFilt(pAlfParam);
#else
  // filter parameters for luma
  m_pcEntropyCoderIf->codeAlfUvlc((pAlfParam->tap-5)/2);
  for(pos=0; pos<pAlfParam->num_coeff; pos++)
  {
    m_pcEntropyCoderIf->codeAlfSvlc(pAlfParam->coeff[pos]);
  }
#endif

  // filter parameters for chroma
  m_pcEntropyCoderIf->codeAlfUvlc(pAlfParam->chroma_idc);
  if(pAlfParam->chroma_idc)
  {
    m_pcEntropyCoderIf->codeAlfUvlc((pAlfParam->tap_chroma-5)/2);

    // filter coefficients for chroma
    for(pos=0; pos<pAlfParam->num_coeff_chroma; pos++)
    {
      m_pcEntropyCoderIf->codeAlfSvlc(pAlfParam->coeff_chroma[pos]);
    }
  }

  // region control parameters for luma
  m_pcEntropyCoderIf->codeAlfFlag(pAlfParam->cu_control_flag);
  if (pAlfParam->cu_control_flag)
  {
    assert( (pAlfParam->cu_control_flag && m_pcEntropyCoderIf->getAlfCtrl()) || (!pAlfParam->cu_control_flag && !m_pcEntropyCoderIf->getAlfCtrl()));
    m_pcEntropyCoderIf->codeAlfCtrlDepth();
  }
#endif
}
#endif

#if HHI_ALF
Void TEncEntropy::encodeAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD, Bool bSeparateQt )
{
  if( bRD )
    uiAbsPartIdx = 0;

  if( bSeparateQt )
  {
    m_pcEntropyCoderIf->codeAlfQTCtrlFlag( pcCU, uiAbsPartIdx );
  }
  else
  {
    m_pcEntropyCoderIf->codeAlfCtrlFlag( pcCU, uiAbsPartIdx );
  }
}

Void TEncEntropy::encodeAlfQTSplitFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth, Bool bRD )
{
  if( bRD )
      uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeAlfQTSplitFlag( pcCU, uiAbsPartIdx, uiDepth, uiMaxDepth );
}
#else
Void TEncEntropy::encodeAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeAlfCtrlFlag( pcCU, uiAbsPartIdx );
}
#endif

Void TEncEntropy::encodePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }

#if HHI_MRG
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    return;
  }
#endif

  if (pcCU->isSkipped( uiAbsPartIdx ))
    return;

  m_pcEntropyCoderIf->codePredMode( pcCU, uiAbsPartIdx );
}

// Split mode
Void TEncEntropy::encodeSplitFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
}

Void TEncEntropy::encodePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

#if HHI_MRG
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    return;
  }
#endif

  if ( pcCU->isSkip( uiAbsPartIdx ) )
    return;

  m_pcEntropyCoderIf->codePartSize( pcCU, uiAbsPartIdx, uiDepth );
}

#if HHI_RQT
Void TEncEntropy::xEncodeTransformSubdiv( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiInnerQuadIdx )
{
  const UInt uiSubdiv = pcCU->getTransformIdx( uiAbsPartIdx ) + pcCU->getDepth( uiAbsPartIdx ) > uiDepth;
  const UInt uiLog2TrafoSize = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()]+2 - uiDepth;

#if HHI_RQT_INTRA
  if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA && pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_NxN && uiDepth == pcCU->getDepth(uiAbsPartIdx) )
  {
    assert( uiSubdiv );
  }
  else
#endif
  if( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() );
    m_pcEntropyCoderIf->codeTransformSubdivFlag( uiSubdiv, uiDepth );
  }

#if HHI_RQT_CHROMA_CBF_MOD
  if( pcCU->getPredictionMode(uiAbsPartIdx) != MODE_INTRA && uiLog2TrafoSize <= pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    const UInt uiTrDepthCurr = uiDepth - pcCU->getDepth( uiAbsPartIdx );
    const Bool bFirstCbfOfCU = uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() || uiTrDepthCurr == 0;
    if( bFirstCbfOfCU || uiLog2TrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepthCurr - 1 ) )
      {
        m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepthCurr );
      }
      if( bFirstCbfOfCU || pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepthCurr - 1 ) )
      {
        m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepthCurr );
      }
    }
    else if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
    {
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepthCurr ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U, uiTrDepthCurr - 1 ) );
      assert( pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepthCurr ) == pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V, uiTrDepthCurr - 1 ) );
    }
  }
#endif

  if( uiSubdiv )
  {
    ++uiDepth;
    const UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1);
    xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 0 );
    uiAbsPartIdx += uiQPartNum;
    xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 1 );
    uiAbsPartIdx += uiQPartNum;
    xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 2 );
    uiAbsPartIdx += uiQPartNum;
    xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 3 );
  }
  else
  {
    {
      DTRACE_CABAC_V( g_nSymbolCounter++ );
      DTRACE_CABAC_T( "\tTrIdx: abspart=" );
      DTRACE_CABAC_V( uiAbsPartIdx );
      DTRACE_CABAC_T( "\tdepth=" );
      DTRACE_CABAC_V( uiDepth );
      DTRACE_CABAC_T( "\ttrdepth=" );
      DTRACE_CABAC_V( pcCU->getTransformIdx( uiAbsPartIdx ) );
      DTRACE_CABAC_T( "\n" );
    }
    UInt uiLumaTrMode, uiChromaTrMode;
    pcCU->convertTransIdx( uiAbsPartIdx, pcCU->getTransformIdx( uiAbsPartIdx ), uiLumaTrMode, uiChromaTrMode );
    m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, TEXT_LUMA, uiLumaTrMode );
#if HHI_RQT_CHROMA_CBF_MOD
    if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA )
#endif
    {
      Bool bCodeChroma = true;
      if( uiLog2TrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
      {
        UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( uiDepth - 1 ) << 1 );
        bCodeChroma  = ( ( uiAbsPartIdx % uiQPDiv ) == 0 );
      }
      if( bCodeChroma )
      {
        m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, uiChromaTrMode );
        m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, uiChromaTrMode );
      }
    }
  }
}
#endif

// transform index
Void TEncEntropy::encodeTransformIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
#if HHI_RQT
  assert( !bRD ); // parameter bRD can be removed
#endif
  if( bRD )
    uiAbsPartIdx = 0;

#if HHI_RQT
  if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
    DTRACE_CABAC_V( g_nSymbolCounter++ )
    DTRACE_CABAC_T( "\tdecodeTransformIdx()\tCUDepth=" )
    DTRACE_CABAC_V( uiDepth )
    DTRACE_CABAC_T( "\n" )
    xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 0 );
  }
  else
#endif
  m_pcEntropyCoderIf->codeTransformIdx( pcCU, uiAbsPartIdx, uiDepth );
}

// ROT index
Void TEncEntropy::encodeROTindex  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

    if (pcCU->getPredictionMode( uiAbsPartIdx )==MODE_INTRA)
    {
      if( ( pcCU->getCbf(uiAbsPartIdx, TEXT_LUMA) + pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_U) + pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_V) ) == 0 )
      {
        return;
      }
      m_pcEntropyCoderIf->codeROTindex( pcCU, uiAbsPartIdx, bRD );
  }
}

// CIP index
Void TEncEntropy::encodeCIPflag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool bRD )
{
  if( bRD )
  uiAbsPartIdx = 0;

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    m_pcEntropyCoderIf->codeCIPflag( pcCU, uiAbsPartIdx, bRD );
  }
}

// Intra direction for Luma
Void TEncEntropy::encodeIntraDirModeLuma  ( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
#if ANG_INTRA
  if ( pcCU->angIntraEnabledPredPart(uiAbsPartIdx) )
    m_pcEntropyCoderIf->codeIntraDirLumaAng( pcCU, uiAbsPartIdx );
  else
    m_pcEntropyCoderIf->codeIntraDirLumaAdi( pcCU, uiAbsPartIdx );
#else
  m_pcEntropyCoderIf->codeIntraDirLumaAdi( pcCU, uiAbsPartIdx );
#endif
}

#if PLANAR_INTRA
Void TEncEntropy::encodePlanarInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if ( pcCU->getSlice()->isInterB() )
    return;

  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codePlanarInfo( pcCU, uiAbsPartIdx );
}
#endif

#if HHI_AIS
// BB: Intra ref. samples filtering for Luma
Void TEncEntropy::encodeIntraFiltFlagLuma ( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  // DC (mode 2) always uses DEFAULT_IS so no signaling needed
  // (no g_aucIntraModeOrder[][] mapping needed because mode 2 always mapped to 2)
  if( (pcCU->getSlice()->getSPS()->getUseAIS()) && (pcCU->getLumaIntraDir( uiAbsPartIdx ) != 2) )
    m_pcEntropyCoderIf->codeIntraFiltFlagLumaAdi( pcCU, uiAbsPartIdx );
}
#endif

// Intra direction for Chroma
Void TEncEntropy::encodeIntraDirModeChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeIntraDirChroma( pcCU, uiAbsPartIdx );
}

Void TEncEntropy::encodePredInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

#if HHI_MRG
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    return;
  }
#endif

  if (pcCU->isSkip( uiAbsPartIdx ))
  {
    if (pcCU->getSlice()->isInterB())
    {
      encodeInterDir(pcCU, uiAbsPartIdx, bRD);
    }
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 ) //if ( ref. frame list0 has at least 1 entry )
    {
      encodeMVPIdx( pcCU, uiAbsPartIdx, REF_PIC_LIST_0);
    }
    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 ) //if ( ref. frame list1 has at least 1 entry )
    {
      encodeMVPIdx( pcCU, uiAbsPartIdx, REF_PIC_LIST_1);
    }
    return;
  }

  PartSize eSize = pcCU->getPartitionSize( uiAbsPartIdx );

  if( pcCU->isIntra( uiAbsPartIdx ) )                                 // If it is Intra mode, encode intra prediction mode.
  {
    if( eSize == SIZE_NxN )                                         // if it is NxN size, encode 4 intra directions.
    {
      UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;
      // if it is NxN size, this size might be the smallest partition size.
      encodeIntraDirModeLuma( pcCU, uiAbsPartIdx                  );
      encodeIntraDirModeLuma( pcCU, uiAbsPartIdx + uiPartOffset   );
      encodeIntraDirModeLuma( pcCU, uiAbsPartIdx + uiPartOffset*2 );
      encodeIntraDirModeLuma( pcCU, uiAbsPartIdx + uiPartOffset*3 );
#if HHI_AIS
      //BB: intra ref. samples filtering flag
      encodeIntraFiltFlagLuma( pcCU, uiAbsPartIdx                  );
      encodeIntraFiltFlagLuma( pcCU, uiAbsPartIdx + uiPartOffset   );
      encodeIntraFiltFlagLuma( pcCU, uiAbsPartIdx + uiPartOffset*2 );
      encodeIntraFiltFlagLuma( pcCU, uiAbsPartIdx + uiPartOffset*3 );
      //
#endif
      encodeIntraDirModeChroma( pcCU, uiAbsPartIdx, bRD );
    }
    else                                                              // if it is not NxN size, encode 1 intra directions
    {
      encodeIntraDirModeLuma  ( pcCU, uiAbsPartIdx );
#if HHI_AIS
      //BB: intra ref. samples filtering flag
      encodeIntraFiltFlagLuma ( pcCU, uiAbsPartIdx );
      //
#endif
      encodeIntraDirModeChroma( pcCU, uiAbsPartIdx, bRD );
    }
  }
  else                                                                // if it is Inter mode, encode motion vector and reference index
  {
    encodeInterDir( pcCU, uiAbsPartIdx, bRD );

    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )       // if ( ref. frame list0 has at least 1 entry )
    {
      encodeRefFrmIdx ( pcCU, uiAbsPartIdx, REF_PIC_LIST_0, bRD );
      encodeMvd       ( pcCU, uiAbsPartIdx, REF_PIC_LIST_0, bRD );
      encodeMVPIdx    ( pcCU, uiAbsPartIdx, REF_PIC_LIST_0      );
    }

    if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )       // if ( ref. frame list1 has at least 1 entry )
    {
      encodeRefFrmIdx ( pcCU, uiAbsPartIdx, REF_PIC_LIST_1, bRD );
      encodeMvd       ( pcCU, uiAbsPartIdx, REF_PIC_LIST_1, bRD );
      encodeMVPIdx    ( pcCU, uiAbsPartIdx, REF_PIC_LIST_1      );
    }
  }
}

#if HHI_MRG
Void TEncEntropy::encodeMergeInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if ( !pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    return;
  }

  if ( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  if( bRD )
    uiAbsPartIdx = 0;

  // find left and top vectors. take vectors from PUs to the left and above.
  TComMvField cMvFieldNeighbours[4]; // above ref_list_0, above ref_list_1, left ref_list_0, left ref_list_1
  UInt uiNeighbourInfo;
  UChar uhInterDirNeighbours[2];
  pcCU->getInterMergeCandidates( uiAbsPartIdx, cMvFieldNeighbours, uhInterDirNeighbours, uiNeighbourInfo );
  
  if ( uiNeighbourInfo )
  {
    // at least one merge candidate exists
    encodeMergeFlag( pcCU, uiAbsPartIdx, bRD );
  }
  else
  {
    assert( !pcCU->getMergeFlag( uiAbsPartIdx ) );
  }

  if ( !pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    // CU is not merged
    return;
  }

  if ( uiNeighbourInfo == 3 )
  {
    // different merge candidates exist. write Merge Index
    encodeMergeIndex( pcCU, uiAbsPartIdx, bRD );
  }
  
}
#endif

Void TEncEntropy::encodeInterDir( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  assert( !pcCU->isIntra( uiAbsPartIdx ) );
  assert( !pcCU->isSkip( uiAbsPartIdx ) || pcCU->getSlice()->isInterB());

  if( bRD )
    uiAbsPartIdx = 0;

  if ( !pcCU->getSlice()->isInterB() )
  {
    return;
  }

  UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;

  switch ( pcCU->getPartitionSize( uiAbsPartIdx ) )
  {
  case SIZE_2Nx2N:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      break;
    }

  case SIZE_2NxN:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      uiAbsPartIdx += uiPartOffset << 1;
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      break;
    }

  case SIZE_Nx2N:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      uiAbsPartIdx += uiPartOffset;
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      break;
    }

  case SIZE_NxN:
    {
      for ( Int iPartIdx = 0; iPartIdx < 4; iPartIdx++ )
      {
        m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
        uiAbsPartIdx += uiPartOffset;
      }
      break;
    }
  case SIZE_2NxnU:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx + (uiPartOffset>>1) );
      break;
    }
  case SIZE_2NxnD:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx + (uiPartOffset<<1) + (uiPartOffset>>1) );
      break;
    }
  case SIZE_nLx2N:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx + (uiPartOffset>>2) );
      break;
    }
  case SIZE_nRx2N:
    {
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx );
      m_pcEntropyCoderIf->codeInterDir( pcCU, uiAbsPartIdx + uiPartOffset + (uiPartOffset>>2) );
      break;
    }
  default:
    break;
  }

  return;
}

Void TEncEntropy::encodeMVPIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;

  switch ( pcCU->getPartitionSize( uiAbsPartIdx ) )
  {
  case SIZE_2Nx2N:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
      {
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_2NxN:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
      {
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      uiAbsPartIdx += uiPartOffset << 1;
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
      {
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_Nx2N:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
      {
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
      }

      uiAbsPartIdx += uiPartOffset;
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
      {
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
      }

      break;
    }

  case SIZE_NxN:
    {
      for ( Int iPartIdx = 0; iPartIdx < 4; iPartIdx++ )
      {
        if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        {
          m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );
        }
        uiAbsPartIdx += uiPartOffset;
      }
      break;
    }
  case SIZE_2NxnU:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>1);
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_2NxnD:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset<<1) + (uiPartOffset>>1);
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nLx2N:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>2);
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nRx2N:
    {
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += uiPartOffset + (uiPartOffset>>2);
      if ( (pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList )) && (pcCU->getMVPNum(eRefList, uiAbsPartIdx)> 1) && (pcCU->getAMVPMode(uiAbsPartIdx) == AM_EXPL) )
        m_pcEntropyCoderIf->codeMVPIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  default:
    break;
  }

  return;

}

Void TEncEntropy::encodeRefFrmIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD )
{
  assert( !pcCU->isIntra( uiAbsPartIdx ) );

  if( bRD )
    uiAbsPartIdx = 0;

#ifdef QC_AMVRES
  if (pcCU->isSkip( uiAbsPartIdx ))
  {
	  return;
  }
  else if ( ( pcCU->getSlice()->getNumRefIdx( eRefList ) == 1 ))
  {
	  if ((pcCU->getSlice()->getSymbolMode() != 0) || (!pcCU->getSlice()->getSPS()->getUseAMVRes()))
		  return;
  }
#else
  if ( ( pcCU->getSlice()->getNumRefIdx( eRefList ) == 1 ) || pcCU->isSkip( uiAbsPartIdx ) )
  {
    return;
  }
#endif
  UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;

  switch ( pcCU->getPartitionSize( uiAbsPartIdx ) )
  {
  case SIZE_2Nx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_2NxN:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
      }

      uiAbsPartIdx += uiPartOffset << 1;
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_Nx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
      }

      uiAbsPartIdx += uiPartOffset;
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_NxN:
    {
      for ( Int iPartIdx = 0; iPartIdx < 4; iPartIdx++ )
      {
        if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        {
          m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );
        }
        uiAbsPartIdx += uiPartOffset;
      }
      break;
    }
  case SIZE_2NxnU:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>1);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_2NxnD:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset<<1) + (uiPartOffset>>1);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nLx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>2);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nRx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += uiPartOffset + (uiPartOffset>>2);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeRefFrmIdx( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  default:
    break;
  }

  return;
}

Void TEncEntropy::encodeMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList, Bool bRD )
{
  assert( !pcCU->isIntra( uiAbsPartIdx ) );

  if( bRD )
    uiAbsPartIdx = 0;

  if ( pcCU->isSkip( uiAbsPartIdx ) )
  {
    return;
  }

  UInt uiPartOffset = ( pcCU->getPic()->getNumPartInCU() >> ( pcCU->getDepth(uiAbsPartIdx) << 1 ) ) >> 2;

  switch ( pcCU->getPartitionSize( uiAbsPartIdx ) )
  {
  case SIZE_2Nx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_2NxN:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
      }

      uiAbsPartIdx += uiPartOffset << 1;
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_Nx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
      }

      uiAbsPartIdx += uiPartOffset;
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
      {
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
      }
      break;
    }

  case SIZE_NxN:
    {
      for ( Int iPartIdx = 0; iPartIdx < 4; iPartIdx++ )
      {
        if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        {
          m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );
        }
        uiAbsPartIdx += uiPartOffset;
      }
      break;
    }
  case SIZE_2NxnU:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>1);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_2NxnD:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset<<1) + (uiPartOffset>>1);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nLx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += (uiPartOffset>>2);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  case SIZE_nRx2N:
    {
      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      uiAbsPartIdx += uiPartOffset + (uiPartOffset>>2);

      if ( pcCU->getInterDir( uiAbsPartIdx ) & ( 1 << eRefList ) )
        m_pcEntropyCoderIf->codeMvd( pcCU, uiAbsPartIdx, eRefList );

      break;
    }
  default:
    break;
  }

  return;
}

#if HHI_RQT
Void TEncEntropy::encodeQtCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth )
{
  m_pcEntropyCoderIf->codeQtCbf( pcCU, uiAbsPartIdx, eType, uiTrDepth );
}

Void TEncEntropy::encodeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx )
{
  m_pcEntropyCoderIf->codeTransformSubdivFlag( uiSymbol, uiCtx );
}
#endif

// Coded block flag
Void TEncEntropy::encodeCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  m_pcEntropyCoderIf->codeCbf( pcCU, uiAbsPartIdx, eType, uiTrDepth );
}

// dQP
Void TEncEntropy::encodeQP( TComDataCU* pcCU, UInt uiAbsPartIdx, Bool bRD )
{
  if( bRD )
    uiAbsPartIdx = 0;

  if ( pcCU->getSlice()->getSPS()->getUseDQP() )
  {
    m_pcEntropyCoderIf->codeDeltaQP( pcCU, uiAbsPartIdx );
  }
}


// texture
Void TEncEntropy::xEncodeCoeff( TComDataCU* pcCU, TCoeff* pcCoeff, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiTrIdx, UInt uiCurrTrIdx, TextType eType, Bool bRD )
{
  if ( pcCU->getCbf( uiAbsPartIdx, eType, uiTrIdx ) )
  {
#if HHI_RQT
    UInt uiLumaTrMode, uiChromaTrMode;
    pcCU->convertTransIdx( uiAbsPartIdx, pcCU->getTransformIdx( uiAbsPartIdx ), uiLumaTrMode, uiChromaTrMode );
    const UInt uiStopTrMode = eType == TEXT_LUMA ? uiLumaTrMode : uiChromaTrMode;

    assert( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() || uiStopTrMode == uiCurrTrIdx ); // as long as quadtrees are not used for residual transform

    if( uiTrIdx == uiStopTrMode )
#else
    if( uiCurrTrIdx == uiTrIdx )
#endif
    {
#if HHI_RQT
      assert( !bRD ); // parameter bRD can be removed

      UInt uiLog2TrSize = g_aucConvertToBit[ pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth ] + 2;
      if( eType != TEXT_LUMA && uiLog2TrSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
      {
        UInt uiQPDiv = pcCU->getPic()->getNumPartInCU() >> ( ( uiDepth - 1 ) << 1 );
        if( ( uiAbsPartIdx % uiQPDiv ) != 0 )
        {
          return;
        }
        uiWidth  <<= 1;
        uiHeight <<= 1;
      }
#endif
#if QC_MDDT
      m_pcEntropyCoderIf->codeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiWidth, uiHeight, uiDepth, eType, pcCU->getLumaIntraDir( uiAbsPartIdx ), bRD );
#else
      m_pcEntropyCoderIf->codeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiWidth, uiHeight, uiDepth, eType, bRD );
#endif
    }
    else
    {
#if HHI_RQT
      {
        DTRACE_CABAC_V( g_nSymbolCounter++ );
        DTRACE_CABAC_T( "\tgoing down\tdepth=" );
        DTRACE_CABAC_V( uiDepth );
        DTRACE_CABAC_T( "\ttridx=" );
        DTRACE_CABAC_V( uiTrIdx );
        DTRACE_CABAC_T( "\n" );
      }
#endif
      if( uiCurrTrIdx <= uiTrIdx )
#if HHI_RQT
        assert( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() );
#else
        assert(0);
#endif

      UInt uiSize;
      uiWidth  >>= 1;
      uiHeight >>= 1;
      uiSize = uiWidth*uiHeight;
      uiDepth++;
      uiTrIdx++;

      UInt uiQPartNum = pcCU->getPic()->getNumPartInCU() >> (uiDepth << 1);
      UInt uiIdx      = uiAbsPartIdx;

      m_pcEntropyCoderIf->codeCbf( pcCU, uiIdx, eType, uiTrIdx );
      xEncodeCoeff( pcCU, pcCoeff, uiIdx, uiDepth, uiWidth, uiHeight, uiTrIdx, uiCurrTrIdx, eType, bRD ); pcCoeff += uiSize; uiIdx += uiQPartNum;

      m_pcEntropyCoderIf->codeCbf( pcCU, uiIdx, eType, uiTrIdx );
      xEncodeCoeff( pcCU, pcCoeff, uiIdx, uiDepth, uiWidth, uiHeight, uiTrIdx, uiCurrTrIdx, eType, bRD ); pcCoeff += uiSize; uiIdx += uiQPartNum;

      m_pcEntropyCoderIf->codeCbf( pcCU, uiIdx, eType, uiTrIdx );
      xEncodeCoeff( pcCU, pcCoeff, uiIdx, uiDepth, uiWidth, uiHeight, uiTrIdx, uiCurrTrIdx, eType, bRD ); pcCoeff += uiSize; uiIdx += uiQPartNum;

      m_pcEntropyCoderIf->codeCbf( pcCU, uiIdx, eType, uiTrIdx );
      xEncodeCoeff( pcCU, pcCoeff, uiIdx, uiDepth, uiWidth, uiHeight, uiTrIdx, uiCurrTrIdx, eType, bRD );
#if HHI_RQT
      {
        DTRACE_CABAC_V( g_nSymbolCounter++ );
        DTRACE_CABAC_T( "\tgoing up\n" );
      }
#endif
    }
  }
}

Void TEncEntropy::encodeCoeff( TComDataCU* pcCU, TCoeff* pCoeff, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight, UInt uiMaxTrMode, UInt uiTrMode, TextType eType, Bool bRD )
{
  xEncodeCoeff( pcCU, pCoeff, uiAbsPartIdx, uiDepth, uiWidth, uiHeight, uiTrMode, uiMaxTrMode, eType, bRD );
}

Void TEncEntropy::encodeCoeff( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight )
{
  UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  UInt uiLumaOffset   = uiMinCoeffSize*uiAbsPartIdx;
  UInt uiChromaOffset = uiLumaOffset>>2;

  UInt uiLumaTrMode, uiChromaTrMode;
  pcCU->convertTransIdx( uiAbsPartIdx, pcCU->getTransformIdx(uiAbsPartIdx), uiLumaTrMode, uiChromaTrMode );

  if( pcCU->isIntra(uiAbsPartIdx) )
  {
#if HHI_RQT_INTRA
    if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
    {
      DTRACE_CABAC_V( g_nSymbolCounter++ )
      DTRACE_CABAC_T( "\tdecodeTransformIdx()\tCUDepth=" )
      DTRACE_CABAC_V( uiDepth )
      DTRACE_CABAC_T( "\n" )
      xEncodeTransformSubdiv( pcCU, uiAbsPartIdx, uiDepth, 0 );
    }
#endif

#if QC_MDDT
	m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_LUMA, 0);
	m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_CHROMA_U, 0);
	m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_CHROMA_V, 0);

#if HHI_ALLOW_ROT_SWITCH
    if (pcCU->getSlice()->getSPS()->getUseROT() && uiWidth >= 16)
#else
    if (uiWidth >= 16)
#endif
		encodeROTindex( pcCU, uiAbsPartIdx, uiDepth );

	xEncodeCoeff( pcCU, pcCU->getCoeffY()  + uiLumaOffset,   uiAbsPartIdx, uiDepth, uiWidth,    uiHeight,    0, uiLumaTrMode,   TEXT_LUMA     );
	xEncodeCoeff( pcCU, pcCU->getCoeffCb() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_U );
	xEncodeCoeff( pcCU, pcCU->getCoeffCr() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_V );
#else
    m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_LUMA, 0);
    xEncodeCoeff( pcCU, pcCU->getCoeffY()  + uiLumaOffset,   uiAbsPartIdx, uiDepth, uiWidth,    uiHeight,    0, uiLumaTrMode,   TEXT_LUMA     );

    m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_CHROMA_U, 0);
    xEncodeCoeff( pcCU, pcCU->getCoeffCb() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_U );

    m_pcEntropyCoderIf->codeCbf(pcCU, uiAbsPartIdx, TEXT_CHROMA_V, 0);
    xEncodeCoeff( pcCU, pcCU->getCoeffCr() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_V );
#endif
  }
  else
  {
    m_pcEntropyCoderIf->codeCbf( pcCU, uiAbsPartIdx, TEXT_LUMA, 0 );
    m_pcEntropyCoderIf->codeCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_U, 0 );
    m_pcEntropyCoderIf->codeCbf( pcCU, uiAbsPartIdx, TEXT_CHROMA_V, 0 );

#if HHI_RQT
    if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() || pcCU->getCbf(uiAbsPartIdx, TEXT_LUMA, 0) || pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_U, 0) || pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_V, 0) )
#else
    if( pcCU->getCbf(uiAbsPartIdx, TEXT_LUMA, 0) || pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_U, 0) || pcCU->getCbf(uiAbsPartIdx, TEXT_CHROMA_V, 0) )
#endif
      encodeTransformIdx( pcCU, uiAbsPartIdx, pcCU->getDepth(uiAbsPartIdx) );

    xEncodeCoeff( pcCU, pcCU->getCoeffY()  + uiLumaOffset,   uiAbsPartIdx, uiDepth, uiWidth,    uiHeight,    0, uiLumaTrMode,   TEXT_LUMA     );
    xEncodeCoeff( pcCU, pcCU->getCoeffCb() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_U );
    xEncodeCoeff( pcCU, pcCU->getCoeffCr() + uiChromaOffset, uiAbsPartIdx, uiDepth, uiWidth>>1, uiHeight>>1, 0, uiChromaTrMode, TEXT_CHROMA_V );
  }
}
#if QC_MDDT
Void TEncEntropy::encodeCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoeff, UInt uiAbsPartIdx, UInt uiTrWidth, UInt uiTrHeight, UInt uiDepth, TextType eType, Bool bRD )
{ // This is for Transform unit processing. This may be used at mode selection stage for Inter.
  UInt uiMode;
  if(eType == TEXT_LUMA && pcCU->isIntra( uiAbsPartIdx ) )
    uiMode = pcCU->getLumaIntraDir( uiAbsPartIdx );
  else
    uiMode = REG_DCT;

  m_pcEntropyCoderIf->codeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiTrWidth, uiTrHeight, uiDepth, eType, uiMode, bRD );
}
#else
Void TEncEntropy::encodeCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoeff, UInt uiAbsPartIdx, UInt uiTrWidth, UInt uiTrHeight, UInt uiDepth, TextType eType, Bool bRD )
{ // This is for Transform unit processing. This may be used at mode selection stage for Inter.
  m_pcEntropyCoderIf->codeCoeffNxN( pcCU, pcCoeff, uiAbsPartIdx, uiTrWidth, uiTrHeight, uiDepth, eType, bRD );
}
#endif


Void TEncEntropy::estimateBit (estBitsSbacStruct* pcEstBitsSbac, UInt uiWidth, TextType eTType)
{
  UInt uiCTXIdx;

  switch(uiWidth)
  {
  case  2: uiCTXIdx = 6; break;
  case  4: uiCTXIdx = 5; break;
  case  8: uiCTXIdx = 4; break;
  case 16: uiCTXIdx = 3; break;
  case 32: uiCTXIdx = 2; break;
  case 64: uiCTXIdx = 1; break;
  default: uiCTXIdx = 0; break;
  }

  eTType = eTType == TEXT_LUMA ? TEXT_LUMA : TEXT_CHROMA;

  m_pcEntropyCoderIf->estBit ( pcEstBitsSbac, uiCTXIdx, eTType );
}

#ifdef QC_SIFO
Void TEncEntropy::encodeSwitched_Filters(TComSlice* pcSlice,TComPrediction *m_cPrediction)
{
	m_pcEntropyCoderIf->encodeSwitched_Filters(pcSlice,m_cPrediction);
}
#endif
