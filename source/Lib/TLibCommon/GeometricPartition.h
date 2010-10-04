/* ====================================================================================================================

The copyright in this software is being made available under the License included below.
This software may be subject to other third party and 	contributor rights, including patent rights, and no such
rights are granted under this license.

Copyright (c) 2010, QUALCOMM INC.
Copyright (c) 2010, TECHNICOLOR INC.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted only for
the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
promoting such standards. The following conditions are required to be met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and
the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
the following disclaimer in the documentation and/or other materials provided with the distribution.
* The names of QUALCOMM INC and TECHNICOLOR INC may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/
#if !defined(GEOMETRIC_PARTITION_H)
#define GEOMETRIC_PARTITION_H

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TypeDef.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

enum Err  
{
  m_nOK,        
  m_nERR,         
};

#if defined( _DEBUG ) || defined( DEBUG )
  #if !defined( _DEBUG )
    #define _DEBUG
  #endif
  #if !defined( DEBUG )
    #define DEBUG
  #endif
#endif

#define ERR_CLASS Err

#define RNOKR( exp, retVal )        \
{                                   \
  if( m_nOK != ( exp ) ) \
  {                                 \
    assert( 0 );                    \
    return retVal;                  \
  }                                 \
}

#if defined( _DEBUG ) || defined( DEBUG )
  #define CHECK( exp )      assert( exp )
#else  // _DEBUG
  #define CHECK( exp )      ((void)( exp ))
#endif // _DEBUG


#ifndef M_PI 
#define M_PI 3.14159265358979323846     /* pi */
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880  /* sqrt(2) */
#endif


#define MAX_MASK_AMPLITUDE 65536

#define MAX_ANGLE_RANGE 360             //Total scan of degrees required to well cover the space of lines

#if CFG_TMM_AC_GEO_REDUCE_DISTANCE_LIWEI //GLW:
#define MAX_DISTANCE_RANGE M_SQRT2/4.0*static_cast<float>(m_uiMbSize)*(1+0.2) // originally it was 0.3
#else 
#ifdef GEOM
#define MAX_DISTANCE_RANGE (m_uiMbSize>>1) 
#else
#define MAX_DISTANCE_RANGE M_SQRT2/2.0*static_cast<float>(m_uiMbSize)  // half diagonal of a MB
#endif
#endif

#define MAX_NUMBER_OF_EDGE_PARAMETERS 2  //2 for linear edges, 3  for quadratic edges

#define PARAMETER_TOLERANCE 0.0001
#define ANGLE_TOLERANCE  PARAMETER_TOLERANCE            //Tolreance value within the interval of which, two angles are considered identical
#define DISTANCE_TOLERANCE  PARAMETER_TOLERANCE        //Tolreance value within the interval of which, two radius are considered identical


#define DE_ALIASING_BORDERS_COMPENSATION 1 //OSCAR_QQ FLAG TO ENABLE DE-ALIASING ON THE BOUNDARIES OF BORDERS WHEN MOTION COMPENSATING

enum PixLayrIdxGEOMB
{
    INNER_LAYER   = 0,
    BOUNDARY_LAYER   = 1,
    OUTER_LAYER   =  2
};

enum ParIdxGEO
{
  PART_GEO_0 = 0,
  PART_GEO_1 = 1,
};

//CLASS TO HANDLE PARTITONS IN SQUARED BLOCKS
class  GeometricPartitionBlock{

  //Functions to be used in Mb (sub-macroblock) Squared Partitions
public:
  GeometricPartitionBlock();
  ~GeometricPartitionBlock();

  //Init & Destroy
  Int initMbPartitionMask (UInt uiMacroblockSize, Bool bDec);        //Generate the Edge Mask buffer
  Int initMbEdgesDictionary (float fQuantizationStepAngle, float QuantizationStepDistance, Int iSkipDirectional=1,UInt uiChromaMb=0);     //Generate the list containing the set of quantized parameters that determine the edge templates (QuantizationStepAngle: e.g. 45, QuantizationStepDistance 8) SkipDirectional indicates that vertical and horizontal partitions are skiped.
//#ifdef GEOM
  void get_mask(int **mask, char **filter, char **filterUV, Char **geo_mv_predictor, int rho, int theta_idx, float theta_step, int blocksize);
  Int initMbPartitionMaskTable(GeometricPartitionBlock* m_cGeometricPartitionInter_Luma);
//#endif
  Int initMbPartitionMaskTable(Bool bDec = false);
  Int destroyMemoryBuffers ();                                           //Simply reset matrix and buffers: free them.

  //Making The partions and related functions
  Int makeMbGeoLinearPartition (float fLineAngle, float fLineDistance, Bool bDec = false);      //Generate Mask from explicit parameters;
  Int makeMbGeoLinearPartition (UInt uiIndex, Bool bDec = false);                                //Generate Mask from indexed parameters;
  Int **getMbPartitionMaskPointer(){ CHECK(m_aaiMbPartitionMask!=NULL); return m_aaiMbPartitionMask;}
  Int makeComplementaryMbPartitionMask();
  Int **getComplementaryMbPartitionMaskPointer(){CHECK(m_aaiComplementaryMbPartitionMask!=NULL); return m_aaiComplementaryMbPartitionMask;}
  Int makeComplementaryMbPartitionMaskFromTable(Int iIndex);

  //Parameter recovery related functions
  UInt   getMbSize(){return m_uiMbSize;}
  float getQStepAngle(){return m_fQStepAngle;}
  float getQStepDistance(){return m_fQStepDistance;}
  Int  getAngleRange(float *afanglerange);
  Int  getDistanceRange(float *afdistancerange);
  float getLineDistance(UInt uiEdgeIndex) {return m_aafSetOfEdgeParameters[uiEdgeIndex][0];}
  float getLineAngle(UInt uiEdgeIndex) {return m_aafSetOfEdgeParameters[uiEdgeIndex][1];}
  UInt   getNumberOfQuantizedEdges(){return m_uiNumberOfQuantizedEdges;}
  Int  getSetOfEdgeParameters(float *fEdgeParameters,Int inumber);     //Function to get the line parameters of edge # inumber: fEdgeParameters is array of parameters, fEdgeParameters[0]=RADIUS ,fEdgeParameters[1]=ANGLE
  UInt  getNumberOfAnglesAt0Radius(){return m_uiAnglesEvery180;}     //Number of Angles for 180 line degrees at Radius=0;
  UInt  getNumberOfAnglesEvery360(){return m_uiAnglesEvery360;}     //Number of Angles for 360 out of Radius=0.
  UInt  getNumberOfTotalRadius(){return m_uiTotalRadius;}


  //Table related functions
  Int **getMbPartitionMaskPointerFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaaiMbPartitionMask_table!=NULL); return m_aaaiMbPartitionMask_table[uiIndex]; }
  Int **getMbComplementaryPartitionMaskPointerFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaaiMbComplementaryPartitionMask_table!=NULL); return m_aaaiMbComplementaryPartitionMask_table[uiIndex]; }
#ifdef GEOM
  Char **getMbMVPMaskPointerFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaacMbMVPMask_table!=NULL); return m_aaacMbMVPMask_table[uiIndex]; }
#ifdef OBMC
  Char **getMbObmcMaskPointerFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaacMbObmcMask_table!=NULL); return m_aaacMbObmcMask_table[uiIndex]; }
  Char **getMbObmcMaskPointerChromaFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaacMbObmcMaskChroma_table!=NULL); return m_aaacMbObmcMaskChroma_table[uiIndex]; }
#endif
#ifdef GEOM_SPEED
  Bool ***getMotionMaskPointerFromTable(UInt uiIndex){CHECK(uiIndex<m_uiNumberOfQuantizedEdges); CHECK(m_aaaabMotionMask_table!=NULL); return m_aaaabMotionMask_table[uiIndex]; }
  Bool getNumPartitions(UInt& uiDepth, UInt& uiNumPartitions);
  Void MakeMotionBlockMask(Int** aaiMbPartitionMask, Bool*** aaabMotionMask);
  Void RecurMakeMotionBlockMask(Int** aaiMbPartitionMask, Bool*** aaabMotionMask, UInt uiOffsetX, UInt uiOffsetY, UInt uiPartAddr, UInt uiBlockSize, UInt uiNumPartitions, UInt uiDepth, UInt uiMaxDepth);
#endif
#endif//GEOM

//Part for reducing complexity
  Int ** getMbMaskStartStopTable(UInt uiEdgeIndex,UInt uiIscomplementary){CHECK(uiEdgeIndex<m_uiNumberOfQuantizedEdges); return uiIscomplementary? m_aaaiMbCompMask_start_stop_p[uiEdgeIndex] : m_aaaiMbMask_start_stop_p[uiEdgeIndex];};
  Int makeBinMbGeoLinearPartition_fromStartStopTable(UInt uiEdgeIndex);
  Int makeBinMbGeoLinearCompPartition_fromStartStopTable(UInt uiEdgeIndex);
  
private:
  float XYPlane(float fX, float fY, float fa, float fb, float fc){return fa*fX+fb*fY-fc;}
  float YLineOfX(float fX, float fa, float fb, float fc){return (fc-fa*fX)/fb;}
  float XLineOfY(float fY, float fa, float fb, float fc){return (fc-fb*fY)/fa;}
  
  //MATRIX HANDLING FUNCTIONS
  Int AllocateMatrix(Char** &aacMatrixPointer,UInt uiSize_row,UInt uiSize_col);
  Int AllocateMatrix(Int** &aaiMatrixPointer,UInt uiSize_row,UInt uiSize_col);
  Int AllocateMatrix(float** &aafMatrixPointer,UInt uiSize_row,UInt uiSize_col);
  Int FreeMatrix(Char** &aacMatrixPointer);
  Int FreeMatrix(Int** &aaiMatrixPointer);
  Int FreeMatrix(float** &aafMatrixPointer);
#ifdef GEOM_SPEED
  Int AllocateMatrix(Bool***& aaabMatrixPointer,UInt uiSize_row,UInt uiSize_col,UInt uiZ);
  Int FreeMatrix(Bool***& aaabMatrixPointer, UInt uiSize_row);
#endif
    
  Int **m_aaiMbPartitionMask;          //Pointer to the mask matrix
  Int **m_aaiComplementaryMbPartitionMask;

  Int   m_iMAX_mask_amplitude;         //Value defining the logical "1" in m_aaiMbPartitionMask values.
  UInt  m_uiMbSize;                     //Macroblock Size

  UInt  m_uiAnglesEvery360;          //Number of Quantized Angles Every 360 degrees at 0 Radius   
  UInt  m_uiAnglesEvery180;          //Number of Quantized Angles Every 180 degrees at 0 Radius

  UInt  m_uiTotalRadius;             //Total Number of Quantized Radius.
  float m_fQStepAngle;               //Quantization Step for Angle data
  float m_fQStepDistance;            //Quantization Step for Distance data
  float m_afAngleRange[2];            //Fixed value array (constant) AngleRange[0]=0, AngleRange[1]=360
  float m_afDistanceRange[2];         //Fixed value array (constant) DistanceRange[0]=-sqrt(2)*MbSize, DistanceRange[1]=sqrt(2)*MbSize
  Int   m_iSkipDirectional;           //Indicates if 16x8 or 8x16 like modes have been discarted.

  UInt m_uiNumberOfQuantizedEdges;     //Number stating the length of the array where the possible combintations of quantized parameters are hold.
  float **m_aafSetOfEdgeParameters;    //Matrix of size NumberOfQuantizedEdges x MAX_NUMBER_OF_EDGE_PARAMETERS where the quantized parameters of edges are kept.

  //////////////////////
  //Look-up table for precomputed edges.
  Int ***m_aaaiMbPartitionMask_table;   //Pointer to the mask matrix look-up table
  Int ***m_aaaiMbComplementaryPartitionMask_table;   //For best efficiency, a ComplementaryPartitionMask_table should be also available.
#ifdef GEOM
  Char **m_aacMbObmcMask;
  Char **m_aacMbObmcMaskChroma;
  Char ***m_aaacMbObmcMask_table;
  Char ***m_aaacMbObmcMaskChroma_table;
  Char **m_aacMbMVPMask;
  Char ***m_aaacMbMVPMask_table;
#ifdef GEOM_SPEED
  Bool ***m_aaabMotionMask;
  Bool ****m_aaaabMotionMask_table;
#endif
#endif

//Part for reducing complexity
  Int ***m_aaaiMbMask_start_stop_p;
  Int ***m_aaaiMbCompMask_start_stop_p;
  Int **m_aaiMbMask_start_stop_p;
  Int **m_aaiMbCompMask_start_stop_p;  
};

  Int isBlockInGEOpartition(Int iBlock ,UInt uiEdgeIndex, GeometricPartitionBlock *pcGeometricPartitionBlock,UInt uiIsComplementaryPartition);

//////////////////////////////////////////////////////////
//CLASS TO HANDLE ALL DIFFERENT PARTITON MODES
class  GeometricPartition{

public:
  GeometricPartition(Bool bDec = false);
  ~GeometricPartition();

  static Int create( GeometricPartition*& rpcGeometricPartition, Bool bDec = false );
  Int destroy();

  Int initEdgeDictionaries(float fQuantizationStepAngle, float fQuantizationStepDistance);
  Int initLookUpTables(Bool bDec = false);

  //GeometricPartitionBlock *getGeometricPartition64x64Inter_Luma(){return m_cGeometricPartition64x64Inter_Luma;}
  //GeometricPartitionBlock *getGeometricPartition64x64Inter_Chroma(){return m_cGeometricPartition64x64Inter_Chroma;}

  GeometricPartitionBlock *getGeometricPartition32x32Inter_Luma(){return m_cGeometricPartition32x32Inter_Luma;}
  GeometricPartitionBlock *getGeometricPartition32x32Inter_Chroma(){return m_cGeometricPartition32x32Inter_Chroma;}


  GeometricPartitionBlock *getGeometricPartition16x16Inter_Luma(){return m_cGeometricPartition16x16Inter_Luma;}
  GeometricPartitionBlock *getGeometricPartition16x16Inter_Chroma(){return m_cGeometricPartition16x16Inter_Chroma;}
  
private:
  //Basic Block sizes

  //GeometricPartitionBlock* m_cGeometricPartition64x64Inter_Luma;
  //GeometricPartitionBlock* m_cGeometricPartition64x64Inter_Chroma;

  GeometricPartitionBlock* m_cGeometricPartition32x32Inter_Luma;
  GeometricPartitionBlock* m_cGeometricPartition32x32Inter_Chroma;

  GeometricPartitionBlock* m_cGeometricPartition16x16Inter_Luma;
  GeometricPartitionBlock* m_cGeometricPartition16x16Inter_Chroma;
  
  Bool m_bDictionairesInitialized; //Flag variables to allow only one time init
  Bool m_bLookUpTablesInitialized; //Flag variables to allow only one time init
  
};
#endif //GEOMETRIC_PARTITION_H

