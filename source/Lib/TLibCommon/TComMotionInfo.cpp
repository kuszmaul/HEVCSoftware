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

/** \file     TComMotionInfo.cpp
    \brief    motion information handling classes
*/

#include <memory.h>
#include "TComMotionInfo.h"
#include "assert.h"
#include <stdlib.h>

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Create / destroy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::create( UInt uiNumPartition )
{
  m_pcMv     = ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_pcMvd    = ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_piRefIdx = (    Int* )xMalloc( Int,    uiNumPartition );

#ifdef GEOM
  m_pcMvGeoPart0      =  ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_pcMvGeoPart1      =  ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_pcMvdGeoPart0     =  ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_pcMvdGeoPart1     =  ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_piRefIdxGeoPart0  =  (    Int* )xMalloc( Int,    uiNumPartition );
  m_piRefIdxGeoPart1  =  (    Int* )xMalloc( Int,    uiNumPartition );
#ifdef QC_AMVRES
  m_bMVResGeoPart0    =  (   Bool* )xMalloc( Bool,   uiNumPartition );
  m_bMVResGeoPart1    =  (   Bool* )xMalloc( Bool,   uiNumPartition );
#endif
#endif

#ifdef QC_AMVRES
  m_bMVRes     = (    Bool* )xMalloc( Bool,    uiNumPartition );
#endif

  m_uiNumPartition = uiNumPartition;
}

Void TComCUMvField::destroy()
{
  if( m_pcMv )
  {
    xFree( m_pcMv );     m_pcMv     = NULL;
  }
  if( m_pcMvd )
  {
    xFree( m_pcMvd );    m_pcMvd    = NULL;
  }
  if( m_piRefIdx )
  {
    xFree( m_piRefIdx ); m_piRefIdx = NULL;
  }

#ifdef GEOM
  if(m_pcMvGeoPart0)
  {
    xFree( m_pcMvGeoPart0 );     m_pcMvGeoPart0     = NULL;
  }
  if(m_pcMvGeoPart1)
  {
    xFree( m_pcMvGeoPart1 );     m_pcMvGeoPart1     = NULL;
  }
  if(m_pcMvdGeoPart0)
  {
    xFree( m_pcMvdGeoPart0 );     m_pcMvdGeoPart0     = NULL;
  }
  if(m_pcMvdGeoPart1)
  {
    xFree( m_pcMvdGeoPart1 );     m_pcMvdGeoPart1     = NULL;
  }
  if( m_piRefIdxGeoPart0 )
  {
    xFree( m_piRefIdxGeoPart0 );  m_piRefIdxGeoPart0 = NULL;
  }
  if( m_piRefIdxGeoPart1 )
  {
    xFree( m_piRefIdxGeoPart1 );  m_piRefIdxGeoPart1 = NULL;
  }
#ifdef QC_AMVRES
  if( m_bMVResGeoPart0 )
  {
    xFree( m_bMVResGeoPart0 );  m_bMVResGeoPart0 = NULL;
  }
  if( m_bMVResGeoPart1 )
  {
    xFree( m_bMVResGeoPart1 );  m_bMVResGeoPart1 = NULL;
  }
#endif
#endif 

#ifdef QC_AMVRES
  if( m_bMVRes )
  {
    xFree( m_bMVRes ); m_bMVRes = NULL;
  }
#endif
}

// --------------------------------------------------------------------------------------------------------------------
// Clear / copy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::clearMv( Int iPartAddr, UInt uiDepth )
{
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  for ( Int i = iNumPartition - 1; i >= 0; i-- )
  {
    m_pcMv[ i ].setZero();
  }
}

Void TComCUMvField::clearMvd( Int iPartAddr, UInt uiDepth )
{
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  for ( Int i = iNumPartition - 1; i >= 0; i-- )
  {
    m_pcMvd[ i ].setZero();
  }
}

Void TComCUMvField::clearMvField()
{
  for ( Int i = m_uiNumPartition - 1; i >= 0; i-- )
  {
    m_pcMv    [ i ].setZero();
    m_pcMvd   [ i ].setZero();
    m_piRefIdx[ i ] = NOT_VALID;
#ifdef GEOM
	m_pcMvGeoPart0     [ i ].setZero();
	m_pcMvGeoPart1     [ i ].setZero();
	m_pcMvdGeoPart0    [ i ].setZero();
	m_pcMvdGeoPart1    [ i ].setZero();
	m_piRefIdxGeoPart0 [ i ] = NOT_VALID;
	m_piRefIdxGeoPart1 [ i ] = NOT_VALID;
#ifdef QC_AMVRES
    m_bMVResGeoPart0   [ i ] = false;
    m_bMVResGeoPart1   [ i ] = false;
#endif
#endif

#ifdef QC_AMVRES
	m_bMVRes[ i ] = false;
#endif
  }
}

Void TComCUMvField::copyFrom( TComCUMvField* pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )
{
  Int iSizeInTComMv = sizeof( TComMv ) * iNumPartSrc;

  memcpy( m_pcMv     + iPartAddrDst, pcCUMvFieldSrc->getMv(),     iSizeInTComMv );
  memcpy( m_pcMvd    + iPartAddrDst, pcCUMvFieldSrc->getMvd(),    iSizeInTComMv );
  memcpy( m_piRefIdx + iPartAddrDst, pcCUMvFieldSrc->getRefIdx(), sizeof( Int ) * iNumPartSrc );
#ifdef GEOM
  memcpy( m_pcMvGeoPart0 + iPartAddrDst, pcCUMvFieldSrc->getMvGeoPart0(),     iSizeInTComMv );
  memcpy( m_pcMvGeoPart1 + iPartAddrDst, pcCUMvFieldSrc->getMvGeoPart1(),     iSizeInTComMv );
  memcpy( m_pcMvdGeoPart0 + iPartAddrDst, pcCUMvFieldSrc->getMvdGeoPart0(),    iSizeInTComMv );
  memcpy( m_pcMvdGeoPart1 + iPartAddrDst, pcCUMvFieldSrc->getMvdGeoPart1(),    iSizeInTComMv );
  memcpy( m_piRefIdxGeoPart0 + iPartAddrDst, pcCUMvFieldSrc->getRefIdxGeoPart0(), sizeof( Int ) * iNumPartSrc );
  memcpy( m_piRefIdxGeoPart1 + iPartAddrDst, pcCUMvFieldSrc->getRefIdxGeoPart1(), sizeof( Int ) * iNumPartSrc );
#ifdef QC_AMVRES
  memcpy( m_bMVResGeoPart0 + iPartAddrDst, pcCUMvFieldSrc->getMVResGeoPart0(), sizeof( Bool ) * iNumPartSrc );
  memcpy( m_bMVResGeoPart1 + iPartAddrDst, pcCUMvFieldSrc->getMVResGeoPart1(), sizeof( Bool ) * iNumPartSrc );
#endif
#endif
#ifdef QC_AMVRES
  memcpy( m_bMVRes + iPartAddrDst, pcCUMvFieldSrc->getMVRes(), sizeof( Bool ) * iNumPartSrc );
#endif

}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst )
{
  Int iSizeInTComMv = sizeof( TComMv ) * m_uiNumPartition;

  memcpy( pcCUMvFieldDst->getMv()     + iPartAddrDst, m_pcMv,     iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvd()    + iPartAddrDst, m_pcMvd,    iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdx() + iPartAddrDst, m_piRefIdx, sizeof( Int ) * m_uiNumPartition );

#ifdef GEOM
  memcpy( pcCUMvFieldDst->getMvGeoPart0()     + iPartAddrDst, m_pcMvGeoPart0,     iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvGeoPart1()     + iPartAddrDst, m_pcMvGeoPart1,     iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvdGeoPart0()    + iPartAddrDst, m_pcMvdGeoPart0,    iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvdGeoPart1()    + iPartAddrDst, m_pcMvdGeoPart1,    iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdxGeoPart0() + iPartAddrDst, m_piRefIdxGeoPart0, sizeof( Int ) * m_uiNumPartition );
  memcpy( pcCUMvFieldDst->getRefIdxGeoPart1() + iPartAddrDst, m_piRefIdxGeoPart1, sizeof( Int ) * m_uiNumPartition );
#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVResGeoPart0() + iPartAddrDst, m_bMVResGeoPart0, sizeof( Bool ) * m_uiNumPartition );
  memcpy( pcCUMvFieldDst->getMVResGeoPart1() + iPartAddrDst, m_bMVResGeoPart1, sizeof( Bool ) * m_uiNumPartition );
#endif
#endif

#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVRes()    + iPartAddrDst, m_bMVRes , sizeof( Bool ) * m_uiNumPartition );
#endif
}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart )
{
  Int iSizeInTComMv = sizeof( TComMv ) * uiNumPart;
  Int iOffset = uiOffset + iPartAddrDst;

  memcpy( pcCUMvFieldDst->getMv()     + iOffset, m_pcMv     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvd()    + iOffset, m_pcMvd    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdx() + iOffset, m_piRefIdx + uiOffset, sizeof( Int ) * uiNumPart );
#ifdef GEOM
  memcpy( pcCUMvFieldDst->getMvGeoPart0()     + iOffset, m_pcMvGeoPart0     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvGeoPart1()     + iOffset, m_pcMvGeoPart1     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvdGeoPart0()    + iOffset, m_pcMvdGeoPart0    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvdGeoPart1()    + iOffset, m_pcMvdGeoPart1    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdxGeoPart0() + iOffset, m_piRefIdxGeoPart0 + uiOffset, sizeof( Int ) * uiNumPart );
  memcpy( pcCUMvFieldDst->getRefIdxGeoPart1() + iOffset, m_piRefIdxGeoPart1 + uiOffset, sizeof( Int ) * uiNumPart );
#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVResGeoPart0() + iOffset, m_bMVResGeoPart0 + uiOffset, sizeof( Bool ) * uiNumPart );
  memcpy( pcCUMvFieldDst->getMVResGeoPart1() + iOffset, m_bMVResGeoPart1 + uiOffset, sizeof( Bool ) * uiNumPart );
#endif
#endif

#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVRes()+ iPartAddrDst,  m_bMVRes + uiOffset, sizeof( Bool ) * uiNumPart );
#endif
}

Void TComCUMvField::copyMvTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst )
{
  memcpy( pcCUMvFieldDst->getMv() + iPartAddrDst, m_pcMv, sizeof( TComMv ) * m_uiNumPartition );
}

// --------------------------------------------------------------------------------------------------------------------
// Set
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::setAllMv( TComMv& rcMv, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  TComMv* pcMv = m_pcMv + iPartAddr;
  register TComMv cMv = rcMv;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pcMv[ i ] = cMv;
        pcMv[ i + uiOffset ] = cMv;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMv;
        }

        pi = pcMv + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMv;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMv;
        }
        pi = pcMv + ( iNumPartition - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        TComMv* pi3 = pcMv + (iCurrPartNumQ>>1);
        TComMv* pi4 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
          pi3[i] = cMv;
          pi4[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }

        pi  = pcMv + (iCurrPartNumQ>>1);
        pi2 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }

        pi  = pcMv + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pcMv + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ>>1);
        TComMv* pi3 = pcMv + (iCurrPartNumQ<<1);
        TComMv* pi4 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
          pi3[i] = cMv;
          pi4[i] = cMv;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllMvd( TComMv& rcMvd, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  TComMv* pcMvd = m_pcMvd + iPartAddr;
  register TComMv cMvd = rcMvd;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pcMvd[ i ] = cMvd;
        pcMvd[ i + uiOffset ] = cMvd;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMvd;
        }

        pi = pcMvd + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMvd;
        }
        pi = pcMvd + ( iNumPartition - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        TComMv* pi3 = pcMvd + (iCurrPartNumQ>>1);
        TComMv* pi4 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
          pi3[i] = cMvd;
          pi4[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }

        pi  = pcMvd + (iCurrPartNumQ>>1);
        pi2 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }

        pi  = pcMvd + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pcMvd + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ>>1);
        TComMv* pi3 = pcMvd + (iCurrPartNumQ<<1);
        TComMv* pi4 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
          pi3[i] = cMvd;
          pi4[i] = cMvd;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  Int* piRefIdx = m_piRefIdx + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        piRefIdx[ i ] = iRefIdx;
        piRefIdx[ i + uiOffset ] = iRefIdx;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = iRefIdx;
        }

        pi = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = iRefIdx;
        }
        pi = piRefIdx + iNumPartition - iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        Int* pi3 = piRefIdx + (iCurrPartNumQ>>1);
        Int* pi4 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
          pi3[i] = iRefIdx;
          pi4[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }

        pi  = piRefIdx + (iCurrPartNumQ>>1);
        pi2 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }

        pi  = piRefIdx + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = piRefIdx + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ>>1);
        Int* pi3 = piRefIdx + (iCurrPartNumQ<<1);
        Int* pi4 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
          pi3[i] = iRefIdx;
          pi4[i] = iRefIdx;
        }
      }
      break;
    }
#ifdef GEOM 
 case SIZE_GEO:
    assert (0);
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
#endif

  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllMvField ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  setAllMv( rcMv, eCUMode, iPartAddr, iPartIdx, uiDepth);
  setAllRefIdx(iRefIdx, eCUMode,iPartAddr,iPartIdx,uiDepth);
#ifdef QC_AMVRES
  setAllMVRes(false, eCUMode,iPartAddr,iPartIdx,uiDepth);
#endif
  return;
}


#ifdef GEOM

UInt TComCUMvField:: GeoPartDetection (UInt uiUnitIdxY, UInt uiUnitIdxX, UInt uiUnitHeight, UInt uiUnitWidth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock )
{
  Int ** aaiMbPartitionMask_table = pcGeometricPartitionBlock->getMbPartitionMaskPointerFromTable(uiEdgeIndex);
  UInt uiSum = 0;

  for (UInt uiPelIdxY = 0; uiPelIdxY < uiUnitHeight; uiPelIdxY ++ )
    for (UInt uiPelIdxX = 0; uiPelIdxX < uiUnitWidth; uiPelIdxX ++ )
      uiSum += aaiMbPartitionMask_table [uiUnitIdxY * uiUnitHeight + uiPelIdxY] [uiUnitIdxX * uiUnitWidth + uiPelIdxX];

  if (uiSum > uiUnitHeight*uiUnitWidth / 2)
    return 0;
  else
    return 1;

}

UInt * TComCUMvField:: getZscanToRasterTable (UInt uiTrueDepth) 
{
  switch (uiTrueDepth)
  {
  case 0:
    return g_auiZscanToRaster; 
  case 1:
    return g_auiZscanToRasterDepth1;
  case 2: 
    return g_auiZscanToRasterDepth2;
  default: 
    {
      printf ("currently only support 3 depths");
      exit (-1);
    }		 
  }
}

Void TComCUMvField::setAllMv( TComMv& rcMv, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock  )
{
  Int i;
  TComMv* pcMv = m_pcMv + iPartAddr;
  register TComMv cMv = rcMv;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  UInt uiWidthInUnit, uiHeightInUnit, uiUnitWidth, uiUnitHeight;
  UInt uiMbSize;
  UInt uiGeoPart;
  UInt * puiZscanToRaster;

  uiMbSize = pcGeometricPartitionBlock->getMbSize ();
  uiWidthInUnit = uiHeightInUnit = (UInt) (sqrt((Double) iNumPartition));
  uiUnitWidth   = uiUnitHeight   = uiMbSize/uiHeightInUnit;

  uiGeoPart = ( eParIdxGeo == PART_GEO_0) ? 0: 1; 
  if (  eCUMode != SIZE_GEO )
    assert(0);

  puiZscanToRaster = getZscanToRasterTable (uiTrueDepth);

  for ( i = 0 ; i <= iNumPartition - 1; i++ )
  {
    UInt uiUnitIdxY, uiUnitIdxX, uiGeoPartDetected;

    uiUnitIdxY =  puiZscanToRaster[i]/uiWidthInUnit;
    uiUnitIdxX =  puiZscanToRaster[i]%uiWidthInUnit;
    uiGeoPartDetected = GeoPartDetection (uiUnitIdxY, uiUnitIdxX, uiUnitHeight, uiUnitWidth, uiEdgeIndex, pcGeometricPartitionBlock);

    if ( uiGeoPart == uiGeoPartDetected)
      pcMv[ i ]  = cMv;
  }

}


Void TComCUMvField::setAllMvd( TComMv& rcMvd, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock  )
{
  Int i;
  TComMv* pcMvd = m_pcMvd + iPartAddr;
  register TComMv cMvd = rcMvd;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);
  if (  eCUMode != SIZE_GEO )
    assert(0);

  UInt uiWidthInUnit, uiHeightInUnit, uiUnitWidth, uiUnitHeight;
  UInt uiMbSize;
  UInt uiGeoPart;
  UInt * puiZscanToRaster;

  uiMbSize = pcGeometricPartitionBlock->getMbSize ();
  uiWidthInUnit = uiHeightInUnit = (UInt) (sqrt((Double) iNumPartition));
  uiUnitWidth   = uiUnitHeight   = uiMbSize/uiHeightInUnit;

  uiGeoPart = ( eParIdxGeo == PART_GEO_0) ? 0: 1; 
  puiZscanToRaster = getZscanToRasterTable (uiTrueDepth);

  for ( i = 0 ; i <= iNumPartition - 1; i++ )
  {
    UInt uiUnitIdxY, uiUnitIdxX, uiGeoPartDetected;

    uiUnitIdxY =  puiZscanToRaster[i]/uiWidthInUnit;
    uiUnitIdxX =  puiZscanToRaster[i]%uiWidthInUnit;
    uiGeoPartDetected = GeoPartDetection (uiUnitIdxY, uiUnitIdxX, uiUnitHeight, uiUnitWidth, uiEdgeIndex, pcGeometricPartitionBlock);

    if ( uiGeoPart == uiGeoPartDetected)
      pcMvd[ i ]  = cMvd;
  }
}


Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock  )
{
  Int i;
  Int* piRefIdx = m_piRefIdx + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  if (  eCUMode != SIZE_GEO )
    assert(0);

  UInt uiWidthInUnit, uiHeightInUnit, uiUnitWidth, uiUnitHeight;
  UInt uiMbSize;
  UInt uiGeoPart;
  UInt * puiZscanToRaster;

  uiMbSize = pcGeometricPartitionBlock->getMbSize ();
  uiWidthInUnit = uiHeightInUnit = (UInt) (sqrt((Double) iNumPartition));
  uiUnitWidth   = uiUnitHeight   = uiMbSize/uiHeightInUnit;

  uiGeoPart = ( eParIdxGeo == PART_GEO_0) ? 0: 1; 
  puiZscanToRaster = getZscanToRasterTable (uiTrueDepth);

  for ( i = 0 ; i <= iNumPartition - 1; i++ )
  {
    UInt uiUnitIdxY, uiUnitIdxX, uiGeoPartDetected;

    uiUnitIdxY =  puiZscanToRaster[i]/uiWidthInUnit;
    uiUnitIdxX =  puiZscanToRaster[i]%uiWidthInUnit;
    uiGeoPartDetected = GeoPartDetection (uiUnitIdxY, uiUnitIdxX, uiUnitHeight, uiUnitWidth, uiEdgeIndex, pcGeometricPartitionBlock);

    if ( uiGeoPart == uiGeoPartDetected)
      piRefIdx[ i ]  = iRefIdx;
  }

}

Void TComCUMvField::setAllMvField ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock   )
{
  setAllMv    ( rcMv,   eCUMode, iPartAddr, eParIdxGeo, uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock);
  setAllRefIdx(iRefIdx, eCUMode, iPartAddr, eParIdxGeo, uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock);
#ifdef QC_AMVRES
  setAllMVRes(false, eCUMode,iPartAddr,eParIdxGeo,uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock);
#endif
  return;
}
Void    TComCUMvField::setAllMvGeo       ( TComMv& rcMv ,   PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth )
{
  Int i;
  TComMv* pcMvPart0 = m_pcMvGeoPart0+iPartAddr;
  TComMv* pcMvPart1 = m_pcMvGeoPart1+iPartAddr;

  register TComMv cMv = rcMv;


  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  assert( eCUMode == SIZE_GEO );

  if (eParIdxGeo == PART_GEO_0 )
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      pcMvPart0[ i ]  = cMv;
  }
  else
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      pcMvPart1[ i ]  = cMv;
  }
}

Void    TComCUMvField::setAllMvdGeo      ( TComMv& rcMvd,   PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth )
{
  Int i;
  TComMv* pcMvdPart0 = m_pcMvdGeoPart0 + iPartAddr;
  TComMv* pcMvdPart1 = m_pcMvdGeoPart1 + iPartAddr;

  register TComMv cMvd = rcMvd;

  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  assert( eCUMode == SIZE_GEO );

  if (eParIdxGeo == PART_GEO_0 )
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      pcMvdPart0[ i ] = cMvd;
  }
  else
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      pcMvdPart1[ i ] = cMvd;
  }  
}

Void   TComCUMvField::setAllRefIdxGeo   ( Int     iRefIdx, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth )
{
  Int i;
  Int* piRefIdxPart0 = m_piRefIdxGeoPart0 + iPartAddr;
  Int* piRefIdxPart1 = m_piRefIdxGeoPart1 + iPartAddr;

  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  assert( eCUMode == SIZE_GEO );

  if (eParIdxGeo == PART_GEO_0)
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      piRefIdxPart0[ i ]  = iRefIdx;
  }    
  else
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      piRefIdxPart1[ i ]  = iRefIdx;
  }
}

Void    TComCUMvField::setAllMvFieldGeo  ( TComMv& rcMv,  Int     iRefIdx,  PartSize eMbMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth )
{
  setAllMvGeo    ( rcMv,    eMbMode, iPartAddr, eParIdxGeo, uiDepth);
  setAllRefIdxGeo(iRefIdx,  eMbMode, iPartAddr, eParIdxGeo, uiDepth);

  /* //Does not support AMVR for GEO yet.
  #ifdef QC_AMVRES
  setAllMVRes(false, eCUMode,iPartAddr,iPartIdx,uiDepth);
  #endif
  */
}
#endif


#ifdef QC_AMVRES
#ifdef GEOM
Void TComCUMvField::setAllMvField_AMVRes ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock )
{
  setAllMv( rcMv, eCUMode, iPartAddr, eParIdxGeo, uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock );
  setAllRefIdx(iRefIdx, eCUMode,iPartAddr,eParIdxGeo,uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock );
#ifdef QC_AMVRES
  setAllMVRes(!rcMv.isHAM(), eCUMode,iPartAddr,eParIdxGeo,uiDepth, uiTrueDepth,  uiEdgeIndex, pcGeometricPartitionBlock );
#endif
  return;
}

Void TComCUMvField::setAllMVRes( Bool bMVRes, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth, UInt uiTrueDepth, UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock )
{
  Int i;
  Bool* pbMVRes = m_bMVRes + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  if (  eCUMode != SIZE_GEO )
    assert(0);

  UInt uiWidthInUnit, uiHeightInUnit, uiUnitWidth, uiUnitHeight;
  UInt uiMbSize;
  UInt uiGeoPart;
  UInt * puiZscanToRaster;

  uiMbSize = pcGeometricPartitionBlock->getMbSize ();
  uiWidthInUnit = uiHeightInUnit = (UInt) (sqrt((Double) iNumPartition));
  uiUnitWidth   = uiUnitHeight   = uiMbSize/uiHeightInUnit;

  uiGeoPart = ( eParIdxGeo == PART_GEO_0) ? 0: 1; 
  puiZscanToRaster = getZscanToRasterTable (uiTrueDepth);

  for ( i = 0 ; i <= iNumPartition - 1; i++ )
  {
    UInt uiUnitIdxY, uiUnitIdxX, uiGeoPartDetected;

    uiUnitIdxY =  puiZscanToRaster[i]/uiWidthInUnit;
    uiUnitIdxX =  puiZscanToRaster[i]%uiWidthInUnit;
    uiGeoPartDetected = GeoPartDetection (uiUnitIdxY, uiUnitIdxX, uiUnitHeight, uiUnitWidth, uiEdgeIndex, pcGeometricPartitionBlock);

    if ( uiGeoPart == uiGeoPartDetected)
      pbMVRes[ i ] = bMVRes;
  }
}
Void TComCUMvField::setAllMVResGeo( Bool bMVRes, PartSize eCUMode, Int iPartAddr, ParIdxGEO eParIdxGeo, UInt uiDepth )
{
  Int i;
  Bool* bMVResGeoPart0 = m_bMVResGeoPart0 + iPartAddr;
  Bool* bMVResGeoPart1 = m_bMVResGeoPart1 + iPartAddr;

  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  assert( eCUMode == SIZE_GEO );

  if (eParIdxGeo == PART_GEO_0)
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      bMVResGeoPart0[ i ]  = bMVRes;
  }    
  else
  {
    for ( i = iNumPartition - 1; i >= 0; i-- )
      bMVResGeoPart1[ i ]  = bMVRes;
  }

}
#endif

Void TComCUMvField::setAllMvField_AMVRes ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  setAllMv( rcMv, eCUMode, iPartAddr, iPartIdx, uiDepth);
  setAllRefIdx(iRefIdx, eCUMode,iPartAddr,iPartIdx,uiDepth);
  setAllMVRes(!rcMv.isHAM(), eCUMode,iPartAddr,iPartIdx,uiDepth);
  return;
}
Void TComCUMvField::setAllMVRes( Bool bMVRes, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  Bool* pbMVRes = m_bMVRes + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pbMVRes[ i ] = bMVRes;
        pbMVRes[ i + uiOffset ] = bMVRes;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = bMVRes;
        }

        pi = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = bMVRes;
        }
        pi = pbMVRes + iNumPartition - iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        Bool* pi3 = pbMVRes + (iCurrPartNumQ>>1);
        Bool* pi4 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
          pi3[i] = bMVRes;
          pi4[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }

        pi  = pbMVRes + (iCurrPartNumQ>>1);
        pi2 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }

        pi  = pbMVRes + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pbMVRes + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ>>1);
        Bool* pi3 = pbMVRes + (iCurrPartNumQ<<1);
        Bool* pi4 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
          pi3[i] = bMVRes;
          pi4[i] = bMVRes;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}


#endif

