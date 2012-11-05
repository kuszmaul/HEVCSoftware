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

/** \file     Debug.h
    \brief    Defines types and objects for environment-variable-based debugging and feature control
*/

#ifndef __DEBUG__
#define __DEBUG__

#include <iostream>
#include <iomanip>
#include <string>
#include <list>
#include <stdlib.h>
#include <sstream>
#include <TLibCommon/CommonDef.h>

// ---------------------------------------------------------------------------------------------- //

//constant print-out macro

#define PRINT_CONSTANT(NAME, NAME_WIDTH, VALUE_WIDTH) std::cout << std::setw(NAME_WIDTH) << #NAME << " = " << std::setw(VALUE_WIDTH) << NAME << std::endl;

// ---------------------------------------------------------------------------------------------- //

// ---- Environment variables for test/debug ---- //

class EnvVar
{
private:
  std::string m_sName;
  std::string m_sHelp;
  std::string m_sVal;
  double      m_dVal;
  int         m_iVal;
  bool        m_bSet;

public:

  static std::list< std::pair<std::string, std::string> > &getEnvVarList();
  static std::list<EnvVar*>                               &getEnvVarInUse();
  static void printEnvVar();
  static void printEnvVarInUse();

  EnvVar(const std::string &sName, const std::string &sDefault, const std::string &sHelp);

  double              getDouble()   const       { return m_dVal;    }
  int                 getInt()      const       { return m_iVal;    }
  const std::string  &getString()   const       { return m_sVal;    }
  bool                isSet()       const       { return m_bSet;    }
  bool                isTrue()      const       { return m_iVal!=0; }
  const std::string  &getName()     const       { return m_sName;   }

};


// ---------------------------------------------------------------------------------------------- //

// ---- Control switches for debugging and feature control ---- //

namespace DebugOptionList
{
  extern EnvVar DebugSBAC;
  extern EnvVar DebugRQT;
  extern EnvVar DebugPred;
  extern EnvVar ForceLumaMode;
  extern EnvVar ForceChromaMode;
  extern EnvVar CopyLumaToChroma444;
  extern EnvVar SwapCbCrOnLoading;
}

namespace ToolOptionList
{
  extern EnvVar AllChromaFormatsUseSameTUStructureAs420;
  extern EnvVar IntraNxNCUChromaPUSplitMode;
  extern EnvVar DoubleHeightCoefficientGroups422;
  extern EnvVar ReducedChromaIntraModeSet;
  extern EnvVar CombinedLumaChromaIntraModeSearch;
  extern EnvVar EncoderInitialIntraModePreEstDMChroma;
  extern EnvVar EncoderFastIntraModeSearchOverAllComponents;
  extern EnvVar EncoderFullRateDistortionSearchOverAllComponents;
  extern EnvVar AdditionalTrialEncodeChromaIntraModeSearch;
  extern EnvVar ChromaIntraReferenceSampleFiltering;
  extern EnvVar Get444LMChromaReferenceSamplesFrom1stColumn;
  extern EnvVar Chroma422IntraAngleScaling;
  extern EnvVar Chroma422IntraDCDoubleWeightAboveSamples;
  extern EnvVar Chroma422IntraPlanarSingleStageCalculation;
  extern EnvVar SetIntraChromaEdgeFilter422;
  extern EnvVar SetIntraChromaDCFilter422;
  extern EnvVar SetIntraChromaEdgeFilter444;
  extern EnvVar SetIntraChromaDCFilter444;
  extern EnvVar UseLumaFilterForChromaQuarterSampleInterpolation;
  extern EnvVar EnableMDDTFor444Chroma;
  extern EnvVar SingleTransformSkipFlagForAllChannels444;
  extern EnvVar Chroma422QuantiserAdjustment;
  extern EnvVar Chroma422QuantiserAdjustmentMethod;
  extern EnvVar UseTransformDepthFor444ChromaCBFContextSelection;
  extern EnvVar AdditionalChromaQpMappingTables;
  extern EnvVar Chroma422SignificanceMapContextGrid;
  extern EnvVar PatternSigCtxMissingGroupsSameAsAvailableGroups;
  extern EnvVar LumaMDCSMode;
  extern EnvVar LumaMDCSAngleLimit;
  extern EnvVar LumaMDCSMaximumWidth;
  extern EnvVar LumaMDCSMaximumHeight;
  extern EnvVar ChromaMDCSMode;
  extern EnvVar ChromaMDCSAngleLimit;
  extern EnvVar ChromaMDCSMaximumWidth;
  extern EnvVar ChromaMDCSMaximumHeight;
  extern EnvVar NonSubsampledChromaUseLumaMDCSSizeLimits;
}

// ---------------------------------------------------------------------------------------------- //

Void printECFMacroSettings();

// ---------------------------------------------------------------------------------------------- //

//Debugging

extern Bool g_bFinalEncode;
extern UInt g_debugCounter;
extern Bool g_printDebug;
extern Void* g_debugAddr;


Void printSBACCoeffData(  const UInt          lastX,
                          const UInt          lastY,
                          const UInt          width,
                          const UInt          height,
                          const UInt          chan,
                          const UInt          absPart,
                          const UInt          scanIdx,
                          const TCoeff *const pCoeff,
                          const Bool          finalEncode=true
                        );


Void printCbfArray( class TComDataCU* pcCU  );

template <typename ValueType>
Void printBlock(const ValueType    *const source,
                const UInt                width,
                const UInt                height,
                const UInt                stride,
                const UInt                outputValueWidth = 0,         //if set to 0, the maximum output width will be calculated and used
                const Int                 shiftLeftBy      = 0,         //set a negative value to right-shift instead
                const Bool                printAverage     = false,     //Also print the average of the values in the block
                const Bool                onlyPrintEdges   = false,     //print only the top row and left column for printing prediction reference samples
                      std::ostream      & stream           = std::cout)
{
  //find the maximum output width
  UInt outputWidth = 0;

  if (outputWidth == 0)
  {
    for (UInt y = 0; y < height; y++)
      for (UInt x = 0; x < width; x++)
      {
        const ValueType value                 = (onlyPrintEdges && ((x == 0) || (y == 0))) ? 0 : leftShift(source[(y * stride) + x], shiftLeftBy);
        const ValueType absoluteValue         = std::abs(value);
              ValueType minimumIncrementValue = 10;
              UInt      currentWidth          = 1;

        while (minimumIncrementValue <= absoluteValue) { minimumIncrementValue *= 10; currentWidth++; }

        if ((value) < 0) currentWidth++; //for the minus sign

        if (currentWidth > outputWidth) outputWidth = currentWidth;
      }

    outputWidth++; //so the numbers don't run into each other
  }

  //------------------

  ValueType valueSum   = 0;

  for (UInt y = 0; y < height; y++)
  {
    for (UInt x = 0; x < width; x++)
    {
      ValueType value = 0;

      if (!onlyPrintEdges || (x == 0) || (y == 0))
      {
        value     = leftShift(source[(y * stride) + x], shiftLeftBy);
        valueSum += value;
      }

      stream << std::setw(outputWidth) << value;
    }
    stream << "\n";
  }

  const Int valueCount = onlyPrintEdges ? Int((width + height) - 1) : Int(width * height);
  if (printAverage) stream << "Average: " << (valueSum / valueCount) << "\n";
  stream << "\n";
}


template <typename T>
Void printBlockToStream( std::ostream &ss, const char *pLinePrefix, const T * blkSrc, const UInt width, const UInt height, const UInt stride, const UInt subBlockWidth=0, const UInt subBlockHeight=0, const UInt defWidth=3 )
{
  for (UInt y=0; y<height; y++)
  {
    if (subBlockHeight!=0 && (y%subBlockHeight)==0 && y!=0)
      ss << pLinePrefix << '\n';

    ss << pLinePrefix;
    for (UInt x=0; x<width; x++)
    {
      if (subBlockWidth!=0 && (x%subBlockWidth)==0 && x!=0)
        ss << std::setw(defWidth+2) << "";

      ss << std::setw(defWidth) << blkSrc[y*stride + x] << ' ';
    }
    ss << '\n';
  }
}

class TComYuv;
Void printBlockToStream( std::ostream &ss, const char *pLinePrefix, TComYuv &src, const UInt numSubBlocksAcross=1, const UInt numSubBlocksUp=1, const UInt defWidth=3 );

// ---------------------------------------------------------------------------------------------- //

//String manipulation functions for aligning and wrapping printed text

std::string splitOnSettings(const std::string &input);

std::string lineWrap(const std::string &input, const UInt maximumLineLength);

std::string indentNewLines(const std::string &input, const UInt indentBy);

// ---------------------------------------------------------------------------------------------- //

#endif /* __DEBUG__ */
