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

/** \file     TAppDecCfg.cpp
    \brief    Decoder configuration class
*/

#include "TAppDecCfg.h"
#if MC_MEMORY_ACCESS_CALC
#include <string>
#endif //MC_MEMORY_ACCESS_CALC

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param argc number of arguments
    \param argv array of arguments
 */
Bool TAppDecCfg::parseCfg( Int argc, Char* argv[] )
{
  // set preferences
  m_apcOpt->setVerbose();
  m_apcOpt->autoUsagePrint(true);
  
  // set usage
  m_apcOpt->addUsage( "options: (if only -b is specified, YUV writing is skipped)" );
  m_apcOpt->addUsage( "  -b  bitstream file name" );
  m_apcOpt->addUsage( "  -o  decoded YUV output file name" );
#if DCM_SKIP_DECODING_FRAMES
  m_apcOpt->addUsage( "  -s  number of frames to skip before random access" );
#endif
  m_apcOpt->addUsage( "  -d  bit depth of YUV output file (use 0 for native depth)" );
#if MC_MEMORY_ACCESS_CALC
  m_apcOpt->addUsage( "  -R  Memory compression ratio (in decimal or fractional form)" );
  m_apcOpt->addUsage( "  -N  Memory compression unit height" );
  m_apcOpt->addUsage( "  -M  Memory compression unit width" );
  m_apcOpt->addUsage( "  -r  Chroma memory compression ratio (in decimal or fractional form)" );
  m_apcOpt->addUsage( "  -n  Chroma memory compression unit height" );
  m_apcOpt->addUsage( "  -m  Chroma memory compression unit width" );
#endif //MC_MEMORY_ACCESS_CALC  

  // set command line option strings/characters
  m_apcOpt->setCommandOption( 'b' );
  m_apcOpt->setCommandOption( 'o' );
#if DCM_SKIP_DECODING_FRAMES
  m_apcOpt->setCommandOption( 's' );
#endif
  m_apcOpt->setCommandOption( 'd' );
#if MC_MEMORY_ACCESS_CALC
  m_apcOpt->setCommandOption( 'R' );
  m_apcOpt->setCommandOption( 'N' );
  m_apcOpt->setCommandOption( 'M' );
  m_apcOpt->setCommandOption( 'r' );
  m_apcOpt->setCommandOption( 'n' );
  m_apcOpt->setCommandOption( 'm' );
#endif //MC_MEMORY_ACCESS_CALC

  // command line parsing
  m_apcOpt->processCommandArgs( argc, argv );
  if( ! m_apcOpt->hasOptions() || !m_apcOpt->getValue( 'b' ) )
  {
    m_apcOpt->printUsage();
    delete m_apcOpt;
    m_apcOpt = NULL;
    return false;
  }
  
  // set configuration
  xSetCfgCommand( m_apcOpt );
  
  return true;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

#if MC_MEMORY_ACCESS_CALC
Bool confirmPara(Bool bflag, const char* message);
#endif //MC_MEMORY_ACCESS_CALC

/** \param pcOpt option handling class
 */
Void TAppDecCfg::xSetCfgCommand   ( TAppOption* pcOpt )
{
  m_pchBitstreamFile = m_pchReconFile = NULL;
  
  if ( pcOpt->getValue( 'b' ) ) m_pchBitstreamFile = pcOpt->getValue( 'b' );
  if ( pcOpt->getValue( 'o' ) ) m_pchReconFile     = pcOpt->getValue( 'o' );
#if DCM_SKIP_DECODING_FRAMES
  m_iSkipFrame = 0;
  if ( pcOpt->getValue( 's' ) ) m_iSkipFrame       = atoi(pcOpt->getValue( 's' ));
#endif
  m_outputBitDepth = 0;
  if ( pcOpt->getValue( 'd' ) ) m_outputBitDepth   = atoi(pcOpt->getValue( 'd' ));
#if MC_MEMORY_ACCESS_CALC
  if ( pcOpt->getValue( 'R' ) ) 
  {
    std::string optval = pcOpt->getValue( 'R' );
    if (optval.find('/') != string::npos) {
      string::size_type pos_div = optval.find('/');
      m_cLumaMemCmpParam.iCmpRatioNum = atoi(optval.substr(0, pos_div).c_str());
      m_cLumaMemCmpParam.iCmpRatioDenom = atoi(optval.substr(pos_div+1).c_str());
    } else {
      double dMemCmpRatio = atof(optval.c_str());
      m_cLumaMemCmpParam.iCmpRatioNum = static_cast<int>(dMemCmpRatio*1000 + 0.499);
      m_cLumaMemCmpParam.iCmpRatioDenom = 1000;
    }
  }
  if ( pcOpt->getValue( 'M' ) )  m_cLumaMemCmpParam.iUnitWidth = atoi(pcOpt->getValue( 'M' ));
  if ( pcOpt->getValue( 'N' ) )  m_cLumaMemCmpParam.iUnitHeight = atoi(pcOpt->getValue( 'N' ));

  // Chroma
  m_cChromaMemCmpParam = m_cLumaMemCmpParam;
  if ( pcOpt->getValue( 'r' ) ) 
  {
    std::string optval = pcOpt->getValue( 'r' );
    if (optval.find('/') != string::npos) {
      string::size_type pos_div = optval.find('/');
      m_cChromaMemCmpParam.iCmpRatioNum = atoi(optval.substr(0, pos_div).c_str());
      m_cChromaMemCmpParam.iCmpRatioDenom = atoi(optval.substr(pos_div+1).c_str());
    } else {
      double dMemCmpRatio = atof(optval.c_str());
      m_cChromaMemCmpParam.iCmpRatioNum = static_cast<int>(dMemCmpRatio*1000 + 0.499);
      m_cChromaMemCmpParam.iCmpRatioDenom = 1000;
    }
  }
  if ( pcOpt->getValue( 'm' ) )  m_cChromaMemCmpParam.iUnitWidth = atoi(pcOpt->getValue( 'm' ));
  if ( pcOpt->getValue( 'n' ) )  m_cChromaMemCmpParam.iUnitHeight = atoi(pcOpt->getValue( 'n' ));

  bool check_failed = false;
#define xConfirmPara(a,b) check_failed |= confirmPara(a,b)
  xConfirmPara( m_cLumaMemCmpParam.iCmpRatioNum     <= 0,                                  "Memory compression ratio must be in the range of (0.0, 1.0]" );
  xConfirmPara( m_cLumaMemCmpParam.iCmpRatioDenom   <  m_cLumaMemCmpParam.iCmpRatioNum,    "Memory compression ratio must be in the range of (0.0, 1.0]" );
  xConfirmPara( m_cLumaMemCmpParam.iUnitWidth       <= 0,                                  "Memory compression unit width must be positive" );
  xConfirmPara( m_cLumaMemCmpParam.iUnitHeight      <= 0,                                  "Memory compression unit height must be positive" );
  xConfirmPara( m_cChromaMemCmpParam.iCmpRatioNum   <= 0,                                  "Memory compression ratio must be in the range of (0.0, 1.0]" );
  xConfirmPara( m_cChromaMemCmpParam.iCmpRatioDenom <  m_cChromaMemCmpParam.iCmpRatioNum,  "Memory compression ratio must be in the range of (0.0, 1.0]" );
  xConfirmPara( m_cChromaMemCmpParam.iUnitWidth     <= 0,                                  "Memory compression unit width must be positive" );
  xConfirmPara( m_cChromaMemCmpParam.iUnitHeight    <= 0,                                  "Memory compression unit height must be positive" );

#undef xConfirmPara
  if (check_failed) {
    exit(EXIT_FAILURE);
  }
#endif //MC_MEMORY_ACCESS_CALC
}

#if MC_MEMORY_ACCESS_CALC
Bool confirmPara(Bool bflag, const char* message)
{
  if (!bflag)
    return false;

  printf("Error: %s\n",message);
  return true;
}
#endif //MC_MEMORY_ACCESS_CALC
