/*! ====================================================================================================================
 * \file
    TDecBinCoderMultiPIPE.h
 *  \brief
    Copyright information.
 *  \par Copyright statements
    HEVC (JCTVC cfp)

    This software, including its corresponding source code, object code and executable, may only be used for
    (1) research purposes or (2) for evaluation for standardisation purposes within the joint collaborative team on
    video coding for HEVC , provided that this copyright notice and this corresponding notice appear in all copies,
    and that the name of Research in Motion Limited not be used in advertising or publicity without specific, written
    prior permission.  This software, as defined above, is provided as a proof-of-concept and for demonstration
    purposes only; there is no representation about the suitability of this software, as defined above, for any purpose.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
    USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    TRADEMARKS, Product and company names mentioned herein may be the trademarks of their respective owners.
    Any rights not expressly granted herein are reserved.

    Copyright (C) 2010 by Research in Motion Limited, Canada
    All rights reserved.

 *  \par Full Contact Information
    Standards & Licensing Department      (standards-ipr@rim.com)
    Research in Motion
    122 West John Carpenter Parkway
    Irving, TX 75039, USA

 * ====================================================================================================================
 */

/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and   contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2010, FRAUNHOFER HHI
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of FRAUNHOFER HHI
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

/** \file     TDecBinCoderMultiPIPE.h
    \brief    binary entropy decoder for multi-partition PIPE
*/

#ifndef __TDEC_BIN_CODER_MULTI_PIPE__
#define __TDEC_BIN_CODER_MULTI_PIPE__


#include "../TLibCommon/TComBitBuffer.h"
#include "TDecBinCoder.h"
#include "TDecPIPETables.h"


class TDecBinMultiPIPE : public TDecBinIf
{
public:
  TDecBinMultiPIPE ();
  ~TDecBinMultiPIPE();

  Void  init              ( TComBitstream* pcTComBitstream );
  Void  uninit            ();

  Void  start             ();
  Void  finish            ();

  Void  decodeBin         ( UInt& ruiBin, ContextModel& rcCtxModel );
  Void  decodeBinEP       ( UInt& ruiBin                           );
  Void  decodeBinTrm      ( UInt& ruiBin                           );

  Void  setBalancedCPUs( UInt ui ) { m_uiBalancedCPUs = ui; }
  UInt  getBalancedCPUs() { return m_uiBalancedCPUs; }

private:
  Void  xDecode           ( UInt uiIdx, UInt uiBits );
  Void  xDecodeEP         ();
  UInt  xDecodePartSize   ( TComBitstream* pcTComBitstream ); 

  const UInt*     m_pacState2Idx;
  TComBitstream*  m_pcTComBitstream;
  TComBitBuffer   m_acBinBuffer[ NUM_V2V_CODERS ];

  UInt m_uiBalancedCPUs;
};


#endif

