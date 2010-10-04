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


#include "GeometricPartition.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "TComRom.h"

#ifndef ROUND
#define ROUND(x)    (static_cast<Int>(floorf(static_cast<float>(x)+static_cast<float>(0.5))))
#endif
int cotLUT32[32]= {
  668927563, 41184, 19777, 12260, 8192, 5474, 3393, 1629, 0, -1629, -3393, -5474, -8192, -12260, -19777,  -41184,
  -668927563, 41184, 19777, 12260, 8192, 5474, 3393, 1629, 0, -1629, -3393, -5474, -8192, -12260, -19777,  -41184
};
int inv_sinLUT32[32]= {
  668927563, 41991, 21407, 14745, 11585, 9852, 8867, 8352, 8192, 8352, 8867, 9852, 11585, 14745, 21407, 41991, 
  -668927563,-41991,-21407,-14745,-11585,-9852,-8867,-8352,-8192,-8352,-8867,-9852,-11585,-14745,-21407,-41991  
};

//Constructor
GeometricPartitionBlock::GeometricPartitionBlock(){

  m_aaiMbPartitionMask=NULL;
  m_aaiComplementaryMbPartitionMask=NULL;

  m_iMAX_mask_amplitude=0;
  m_uiMbSize=0;

  m_fQStepAngle=0;
  m_fQStepDistance=0;

  m_afAngleRange[0]=0;
  m_afAngleRange[1]=0;

  m_afDistanceRange[0]=0;
  m_afDistanceRange[1]=0;

  m_uiNumberOfQuantizedEdges=0;

  m_uiAnglesEvery360=0;
  m_uiAnglesEvery180=0;
  m_uiTotalRadius=0;

  m_aafSetOfEdgeParameters=NULL;
  m_aaaiMbPartitionMask_table=NULL;
  m_aaaiMbComplementaryPartitionMask_table=NULL;

#ifdef GEOM
  m_aacMbObmcMask=NULL;
  m_aacMbObmcMaskChroma=NULL;
  m_aacMbMVPMask=NULL;
#ifdef GEOM_SPEED
  m_aaabMotionMask=NULL;
  m_aaaabMotionMask_table=NULL;
#endif
  m_aaacMbMVPMask_table=NULL;
#endif

  m_aaaiMbMask_start_stop_p=NULL;
  m_aaaiMbCompMask_start_stop_p=NULL;
  m_aaiMbMask_start_stop_p=NULL;
  m_aaiMbCompMask_start_stop_p=NULL;

}

GeometricPartitionBlock::~GeometricPartitionBlock(){

  destroyMemoryBuffers();
}

//Function to initialize a Mb partition mask with input the macroblock size.
Int GeometricPartitionBlock::initMbPartitionMask(UInt uiMacroblockSize, Bool bDec){

  CHECK(uiMacroblockSize>0 && (uiMacroblockSize%2)==0);  //Check MacroblockSize to be positive and even

  //Free the Mask if already allocated
  if(m_aaiMbPartitionMask!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbPartitionMask),m_nERR);
  }


  //Free the complementary mask
  if(m_aaiComplementaryMbPartitionMask!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiComplementaryMbPartitionMask),m_nERR);
  }

  //Allocate the Mask (and complementary)
  m_uiMbSize=uiMacroblockSize;
  RNOKR(AllocateMatrix(m_aaiMbPartitionMask,m_uiMbSize,m_uiMbSize),m_nERR);

  m_iMAX_mask_amplitude=MAX_MASK_AMPLITUDE;
  RNOKR(AllocateMatrix(m_aaiComplementaryMbPartitionMask,m_uiMbSize,m_uiMbSize),m_nERR);
  
#ifdef GEOM
  RNOKR(AllocateMatrix(m_aacMbMVPMask,2,NUM_GEO_MVP),m_nERR);
  RNOKR(AllocateMatrix(m_aacMbObmcMask,m_uiMbSize,m_uiMbSize),m_nERR);
  RNOKR(AllocateMatrix(m_aacMbObmcMaskChroma,m_uiMbSize/2,m_uiMbSize/2),m_nERR);
#ifdef GEOM_SPEED
  if(bDec == false)
  {
    UInt uiDepth, uiNumPartitions;
    if(getNumPartitions(uiDepth, uiNumPartitions))
    {            
      RNOKR(AllocateMatrix(m_aaabMotionMask,2,(g_uiMaxCUDepth - uiDepth)*3+1, uiNumPartitions),m_nERR);
    }
  }
#endif
#endif

//Allocate start_stop line vectors
if(m_aaiMbMask_start_stop_p!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbMask_start_stop_p),m_nERR);
}
if(m_aaiMbCompMask_start_stop_p!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbCompMask_start_stop_p),m_nERR);
}

RNOKR(AllocateMatrix(m_aaiMbMask_start_stop_p,m_uiMbSize,2),m_nERR);
RNOKR(AllocateMatrix(m_aaiMbCompMask_start_stop_p,m_uiMbSize,2),m_nERR);

  return 0;
}

//Generate the whole set of Dictionary Edges
Int GeometricPartitionBlock::initMbEdgesDictionary (float fQuantizationStepAngle, float fQuantizationStepDistance, Int iSkipDirectional, UInt uiChromaMb){

  CHECK(fQuantizationStepAngle>=ANGLE_TOLERANCE && fQuantizationStepDistance>=DISTANCE_TOLERANCE);
  CHECK(m_aafSetOfEdgeParameters==NULL);


  //Setup variables
  m_fQStepAngle=fQuantizationStepAngle;
  m_fQStepDistance=fQuantizationStepDistance;

  m_afAngleRange[0]=0;
  m_afAngleRange[1]=MAX_ANGLE_RANGE;

  m_afDistanceRange[0]=0;
  m_afDistanceRange[1]=(float)MAX_DISTANCE_RANGE;

  m_uiAnglesEvery360=0;
  m_uiAnglesEvery180=0;
  m_uiTotalRadius=0;

  Int iskiped_angles_at_0=0;

  Int uiTestAnglesEvery360=0;

  m_iSkipDirectional=iSkipDirectional;
 
  Int Maximum_length_parameters=(static_cast<Int>(ROUND((m_afDistanceRange[1]-m_afDistanceRange[0])/m_fQStepDistance))+1)*(static_cast<Int>(ROUND((m_afAngleRange[1]-m_afAngleRange[0])/m_fQStepAngle))+1);

  //Free the List if already allocated
  if(m_aafSetOfEdgeParameters!=NULL && m_uiNumberOfQuantizedEdges>0){
    RNOKR(FreeMatrix(m_aafSetOfEdgeParameters),m_nERR);
  }

  RNOKR(AllocateMatrix(m_aafSetOfEdgeParameters,Maximum_length_parameters,MAX_NUMBER_OF_EDGE_PARAMETERS),m_nERR);

  //Set up Container for Dictionary of edge parameters
  float present_angle=m_afAngleRange[0];
  float present_radius=m_afDistanceRange[0];

  m_uiNumberOfQuantizedEdges=0;
  while(present_radius<m_afDistanceRange[1]){
    while(present_angle<m_afAngleRange[1]){

      int valid_edge=1;

      ////////////////////////////////////////////////
      //Check if edge is valid
      if((present_radius-0)<DISTANCE_TOLERANCE){//If in the center, do not accept tree like partitions
 

        if(present_angle<180.0){//Since it is symetric, higher than 180 has no sense
  
          float critical_angles[3]={0.0,90.0,180.0};//Since it is symetric, higher than 180 has no sense
          
          Int num_critical_angles=0;    
          if(m_iSkipDirectional){  
              
            while((critical_angles[num_critical_angles]-m_afAngleRange[1])<-ANGLE_TOLERANCE){
                num_critical_angles++;
                if(num_critical_angles==3) break;
            }
            
          }
          else{
              num_critical_angles=1;
              critical_angles[0]=critical_angles[1]=critical_angles[2]=180.0;
          }  

          for(Int index=0;index<num_critical_angles;index++){
            if(fabs(present_angle-critical_angles[index])<ANGLE_TOLERANCE){
              valid_edge=0;
              iskiped_angles_at_0++;
              break;
            }
          }
        }
        else{
          valid_edge=0;
        }

      }
      else{//If not in the center, do not accept radius outside the MB.

      }//End of Checks and validity TESTS
      ///////////////////////////////////////////////////////////

      //If valid, record edge parameters
      if(valid_edge){
        m_aafSetOfEdgeParameters[m_uiNumberOfQuantizedEdges][0]=present_radius;//Set PARAMETERS
        m_aafSetOfEdgeParameters[m_uiNumberOfQuantizedEdges][1]=present_angle;
        m_uiNumberOfQuantizedEdges++;

        if((present_radius-0)<DISTANCE_TOLERANCE){//FIND OUT THE NUMBER OF ANGLES PRESENT AT RADIUS 0
          m_uiAnglesEvery180++;
        }
        else{

          uiTestAnglesEvery360++;
        }
      }

      present_angle+=m_fQStepAngle;
    }

    present_radius+=m_fQStepDistance;
    present_angle=m_afAngleRange[0];

    if((present_radius-0)>=DISTANCE_TOLERANCE){  //CHECK THAT THE RULE FOR HALF NUM. OF ANGLES AT RADIUS 0 is accomplished.
      CHECK((((Int)m_uiAnglesEvery180+iskiped_angles_at_0)*2)>=(Int)uiTestAnglesEvery360);
    }

    if(uiTestAnglesEvery360>(Int)m_uiAnglesEvery360)
    {
      m_uiAnglesEvery360=uiTestAnglesEvery360;
    }

    uiTestAnglesEvery360=0;

    m_uiTotalRadius++;
  }

  return 0;

}


Int GeometricPartitionBlock::destroyMemoryBuffers (){

  //Delete Array of quantized parameters
  if(m_aafSetOfEdgeParameters!=NULL && m_uiNumberOfQuantizedEdges>0){
      RNOKR(FreeMatrix(m_aafSetOfEdgeParameters),m_nERR);
  }       
  

  //Free matrices used to indicate start and stop of columns
  if(m_aaiMbMask_start_stop_p!=NULL && m_uiNumberOfQuantizedEdges>0){
      RNOKR(FreeMatrix(m_aaiMbMask_start_stop_p),m_nERR);
  }
  
  if(m_aaiMbCompMask_start_stop_p!=NULL && m_uiNumberOfQuantizedEdges>0){
      RNOKR(FreeMatrix(m_aaiMbCompMask_start_stop_p),m_nERR);
  }

  if(m_aaaiMbPartitionMask_table!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<(Int)m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaaiMbPartitionMask_table[index]),m_nERR);

    delete [] m_aaaiMbPartitionMask_table;

    m_aaaiMbPartitionMask_table=NULL;
  }

  if(m_aaaiMbComplementaryPartitionMask_table!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<(Int)m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaaiMbComplementaryPartitionMask_table[index]),m_nERR);

    delete [] m_aaaiMbComplementaryPartitionMask_table;

    m_aaaiMbComplementaryPartitionMask_table=NULL;

  }

  //Delete Masks
  if(m_aaiMbPartitionMask!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbPartitionMask),m_nERR);
  }
  if(m_aaiComplementaryMbPartitionMask!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiComplementaryMbPartitionMask),m_nERR);
  }
#ifdef GEOM
  if(m_aacMbObmcMask!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aacMbObmcMask),m_nERR);
  }
  if(m_aacMbObmcMaskChroma!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aacMbObmcMaskChroma),m_nERR);
  }
  if(m_aaacMbObmcMask_table!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaacMbObmcMask_table[index]),m_nERR);

    delete [] m_aaacMbObmcMask_table;

    m_aaacMbObmcMask_table=NULL;
  }
  if(m_aaacMbObmcMaskChroma_table!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaacMbObmcMaskChroma_table[index]),m_nERR);

    delete [] m_aaacMbObmcMaskChroma_table;

    m_aaacMbObmcMaskChroma_table=NULL;
  }
  
  if(m_aacMbMVPMask!=NULL){
    RNOKR(FreeMatrix(m_aacMbMVPMask),m_nERR);
  }
  if(m_aaacMbMVPMask_table!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaacMbMVPMask_table[index]),m_nERR);

    delete [] m_aaacMbMVPMask_table;

    m_aaacMbMVPMask_table=NULL;
  }
#ifdef GEOM_SPEED
  if(m_aaabMotionMask!=NULL){
    RNOKR(FreeMatrix(m_aaabMotionMask,2),m_nERR);
    m_aaabMotionMask=NULL;
  }
  if(m_aaaabMotionMask_table!=NULL && m_uiNumberOfQuantizedEdges>0){
    for(Int index=0;index<m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaaabMotionMask_table[index],2),m_nERR);

    delete [] m_aaaabMotionMask_table;
    m_aaaabMotionMask_table=NULL;
  }
#endif
#endif
  //Part for reducing complexity
  if(m_aaiMbMask_start_stop_p!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbMask_start_stop_p),m_nERR);
  }
  if(m_aaiMbCompMask_start_stop_p!=NULL && m_uiMbSize>0){
    RNOKR(FreeMatrix(m_aaiMbCompMask_start_stop_p),m_nERR);
  }

  if(m_aaaiMbMask_start_stop_p!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<(Int)m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaaiMbMask_start_stop_p[index]),m_nERR);

    delete [] m_aaaiMbMask_start_stop_p;
  }
  if(m_aaaiMbCompMask_start_stop_p!=NULL && m_uiNumberOfQuantizedEdges>0){

    for(Int index=0;index<(Int)m_uiNumberOfQuantizedEdges;index++) RNOKR(FreeMatrix(m_aaaiMbCompMask_start_stop_p[index]),m_nERR);

    delete [] m_aaaiMbCompMask_start_stop_p;
  }

  m_uiNumberOfQuantizedEdges=0;
  m_uiMbSize=0;

  return 0;
}

#ifdef GEOM_SPEED
Bool GeometricPartitionBlock::getNumPartitions(UInt& uiDepth, UInt& uiNumPartitions)
{
  if(g_uiMaxCUWidth < m_uiMbSize)
    return false;

  UInt uiSize = g_uiMaxCUWidth;
  uiDepth = 0;
  while (uiSize > m_uiMbSize)
  {
    uiSize >>= 1;
    uiDepth++;
  }
  
  if(g_uiMaxCUDepth <= uiDepth)
    return false;

  uiNumPartitions = 1<<( ( g_uiMaxCUDepth - uiDepth )<<1 );
  return true;
}
Void GeometricPartitionBlock::RecurMakeMotionBlockMask(Int** aaiMbPartitionMask, Bool*** aaabMotionMask, UInt uiOffsetX, UInt uiOffsetY, UInt uiPartAddr, UInt uiBlockSize, UInt uiNumPartitions, UInt uiDepth, UInt uiMaxDepth)
{
  UInt i;
  UInt uiCurrPartNumQ = uiNumPartitions>>2;
  UInt uiPartIdx=0;
  
  for(Int iSubLev=0; iSubLev<4; iSubLev++) //CU partition    
  {        
    Int iMvLev = uiDepth * 3 + iSubLev; // sublev 0 will overwrite sublev 3 of the uiDepth-1
    Int iPartSizeX = uiBlockSize>>(iSubLev/2);
    Int iPartSizeY = uiBlockSize>>(iSubLev%2);
    Bool bEmbeddedFlag=true;
    
    uiPartIdx = 0;
    for( Int y=uiOffsetY; y<uiOffsetY+uiBlockSize; y+=iPartSizeY )
    {
      for( Int x=uiOffsetX; x<uiOffsetX+uiBlockSize; x+=iPartSizeX )
      {                       
        for(Int segm=0; segm<2; segm++)
        {
          bEmbeddedFlag = true;
          for(Int jj=y; jj<y+iPartSizeY; jj++)
          {
            for(Int ii=x; ii<x+iPartSizeX; ii++)
            {
              if(aaiMbPartitionMask[jj][ii]!=1-segm)
                bEmbeddedFlag = false;            
            }                   
          }

          Bool* pb = aaabMotionMask[segm][iMvLev]+uiPartAddr;

          switch ( iSubLev )
          {
          case 0://SIZE_2Nx2N:
            for (i = 0; i < uiNumPartitions; i++)
              pb[i] = bEmbeddedFlag;
            break;
          case 1://SIZE_2NxN:
            pb += ( uiPartIdx == 0 )? 0 : uiNumPartitions >> 1;
            for (i = 0; i < (uiNumPartitions >> 1); i++)            
              pb[i] = bEmbeddedFlag;
            break;
          case 2://SIZE_Nx2N:
            {
              pb += ( uiPartIdx == 0 )? 0 : uiCurrPartNumQ;
              Bool* pb2 = pb + ( uiNumPartitions >> 1 );
              for (i = 0; i < uiCurrPartNumQ; i++)
              {
                pb [i] = bEmbeddedFlag;
                pb2[i] = bEmbeddedFlag;
              }
            }
            break;
          case 3://SIZE_NxN:
            pb +=  uiCurrPartNumQ * uiPartIdx;
            for (i = 0; i < uiCurrPartNumQ; i++)
              pb [i] = bEmbeddedFlag;
            break;
          default:
            assert(0);
          }          
        }
        uiPartIdx++;
      }
    }
  }

  UChar uhNextDepth = uiDepth+1;
  //if( uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
  if( uhNextDepth < uiMaxDepth )
  {
    UInt  uiNextBlockSize = uiBlockSize >> 1;
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
    {
      UInt uiNextOffsetX = uiOffsetX + (uiPartUnitIdx%2) * uiNextBlockSize;
      UInt uiNextOffsetY = uiOffsetY + (uiPartUnitIdx/2) * uiNextBlockSize;

      RecurMakeMotionBlockMask( aaiMbPartitionMask, aaabMotionMask, uiNextOffsetX, uiNextOffsetY, uiPartAddr+uiCurrPartNumQ*uiPartUnitIdx, uiNextBlockSize, uiCurrPartNumQ, uhNextDepth, uiMaxDepth );
    }
  }
}
Void GeometricPartitionBlock::MakeMotionBlockMask(Int** aaiMbPartitionMask, Bool*** aaabMotionMask)
{
  UInt uiDepth, uiNumPartitions;  
  if(!getNumPartitions(uiDepth, uiNumPartitions))//checked
    return;

  CHECK(m_aaabMotionMask!=NULL);
    
  RecurMakeMotionBlockMask(aaiMbPartitionMask, aaabMotionMask, 0, 0, 0, m_uiMbSize, uiNumPartitions, 0, g_uiMaxCUDepth-uiDepth);//checked
}
#endif

#ifdef GEOM
void GeometricPartitionBlock::get_mask(int **mask, char **filter, char **filterUV, char **geo_mv_predictor, int rho, int theta_idx, float theta_step, int blocksize)
{
  int i, x,y, segm, pred;
  int sx_i,sy_i;
  int rx;
  //a  b  c  d  e  f g
  static int x0[]={0, 0, 3, 0, 0, 2, 0}; //(x0,y0) and (x1,y1) are the coordinators in a 4x4 block
  static int y0[]={3, 3, 3, 3, 1, 3, 0};
  static int x1[]={0, 1, 3, 0, 0, 3, 0};
  static int y1[]={2, 3, 3, 3, 0, 3, 0};
  int coor[4];
  int blocksizeUV;
  int ii, jj, num[2];
  int xn[]={-1,0, 0,1};//4 connect
  int yn[]={ 0,1,-1,0};//4 connect

  assert(theta_step == 11.25);
  //coor is used to convert the coordinators in a 4x4 block to NxN block
  if(blocksize == 8 || blocksize == 16 || blocksize==32 || blocksize==64)
  {
    coor[0] = 0; coor[1] = 3; coor[2] = blocksize-4; coor[3] = blocksize-1;
  }
  else
  {
    printf ("blocisize is not correct!\n");
    exit(-1);
  }

  for(x=0;x<blocksize;x++)
  {
    for(y=0;y<blocksize;y++)
    {
      sx_i = (x*2-blocksize+1);
      sy_i = (y*2-blocksize+1);

      if (theta_idx ==0)
        rx = sx_i-rho*2;
      else if (theta_idx==16)
        rx = -sx_i-rho*2;
      else
      {
        rx = (sx_i*cotLUT32[theta_idx]+sy_i*8192-rho*2*inv_sinLUT32[theta_idx]);
        if (inv_sinLUT32[theta_idx]<0)
          rx=-rx;
      }

      if (rx < 0)
      {
        mask[y][x]=0;
      }
      else
      {
        mask[y][x]=1;
      }
    }
  }


  int flip = 0;
  int part0, part1, start0_x, start1_x;
  for(y=0;y<blocksize;y++)
  { 
    part0=0;
    part1=0;
    start0_x = -1; // starting location of partition 0
    start1_x = -1; // starting location of partition 1
    for(x=0;x<blocksize;x++)
    {
      if(mask[y][x] == 0 && part0==0)
      {
        part0=1;
        start0_x = x;
      }

      if(mask[y][x] == 1 && part1==0)
      {
        part1=1;
        start1_x = x;
      }
    }

    if(start0_x != -1 && start1_x != -1)
    {
      if(start0_x > start1_x)
      {
        flip = 1;
        break;
      }
    }
  }

  //0 -> 1; 1->0
  if(flip == 1)
  {
    for(x=0;x<blocksize;x++)
    { 
      for(y=0;y<blocksize;y++)
        mask[y][x] = 1 - mask[y][x];
    }
  }

  for(segm=0; segm<2; segm++)
  {
    for(pred=0; pred<NUM_GEO_MVP;pred++)
    {
      if(mask[coor[y0[pred]]] [coor[x0[pred]]] == 1-segm && mask[coor[y1[pred]]] [coor[x1[pred]]] == 1-segm)
      {
        geo_mv_predictor[segm][pred]  = 1;        
      }
      else
      {
        geo_mv_predictor[segm][pred]  = 0;        
      }
    }
  }

  for(x=0;x<blocksize;x++)
  { 
    for(y=0;y<blocksize/2;y++)
    {
      int temp = mask[y][x];
      mask[y][x] = mask[blocksize-1-y][x];
      mask[blocksize-1-y][x] = temp;
    }
  }


  //for(x=0;x<blocksize;x+=4)
  //{
  //  for(y=0;y<blocksize;y+=4)
  //  {
  //    segm = mask[y][x];
  //    mask4x4[y][x] = segm;

  //    for(i=x; i<x+4; i++)
  //    {
  //      for(j=y; j<y+4; j++)
  //      {
  //        if(mask[j][i] != segm)
  //        {
  //          mask4x4[y][x] = 2;
  //          break;
  //        }
  //      }
  //      if(mask4x4[y][x] == 2)
  //        break;
  //    }
  //  }
  //}

  for(x=0;x<blocksize;x++)//OBMC
  {
    for(y=0;y<blocksize;y++)
    {
      num[0] = 0; num[1] = 0;

      for(i=0; i<4; i++)//4-connect
      {
        ii = x+xn[i];
        if(ii<0)          ii=0;
        if(ii>=blocksize) ii=blocksize-1;
        jj = y+yn[i];
        if(jj<0)          jj=0;
        if(jj>=blocksize) jj=blocksize-1;
        num[mask[jj][ii]]++;
      }

      if(num[0]>0 && num[1]>0)
      {
        filter[y][x] = 1+mask[y][x];
      }
      else
        filter[y][x] = 0;
    }
  }

  blocksizeUV = blocksize/2;
  for(x=0;x<blocksizeUV;x++)
  {
    for(y=0;y<blocksizeUV;y++)
    {
      num[0] = 0; num[1] = 0;
      for(i=0; i<4; i++)
      {
        ii = x+xn[i];
        if(ii<0)          ii=0;
        if(ii>=blocksizeUV) ii=blocksizeUV-1;
        jj = y+yn[i];
        if(jj<0)          jj=0;
        if(jj>=blocksizeUV) jj=blocksizeUV-1;
        num[mask[jj<<1][ii<<1]]++;
      }

      if(num[0]>0 && num[1]>0)
      {
        filterUV[y][x] = 1+mask[y<<1][x<<1];
      }
      else
        filterUV[y][x] = 0;
    }
  }

  //if( rho == 0 && theta_idx == 4)
  //{
  //  for(y=0;y<blocksize;y++)
  //  {
  //    for(x=0;x<blocksize;x++)
  //    {
  //      printf("%d ", mask[y][x]);
  //    }
  //    printf("\n");
  //  }
  //}
}
#endif//
//Generates a GeoLinearPartion mask based on function f=x\cos(\theta)+y\sin(\theta)-\pho
Int GeometricPartitionBlock::makeMbGeoLinearPartition (float fLineAngle, float fLineDistance, Bool bDec){

  CHECK(m_aaiMbPartitionMask!=NULL && m_uiMbSize>0); //Check the existence of a the output matrix
#ifdef GEOM
  CHECK(m_aacMbMVPMask!=NULL);
  CHECK(m_aacMbObmcMask!=NULL);
  CHECK(m_aacMbObmcMaskChroma!=NULL);
#endif
  CHECK(fLineAngle>=0 && fLineAngle<360);
  CHECK(fLineDistance>=0 && fLineDistance<=M_SQRT2*m_uiMbSize);

#ifdef GEOM
  get_mask(m_aaiMbPartitionMask, m_aacMbObmcMask, m_aacMbObmcMaskChroma, m_aacMbMVPMask, (Int)fLineDistance, (Int)(fLineAngle/THETASTEP), THETASTEP, m_uiMbSize);
#ifdef GEOM_SPEED
  if (bDec == false)
  {
    MakeMotionBlockMask(m_aaiMbPartitionMask, m_aaabMotionMask);//checked
  }
#endif
#endif//GEOM

    for(Int y_variable=0;y_variable<(Int)m_uiMbSize;y_variable++){
//Part for reducing complexity
    Int iPartition_label=0;
    m_aaiMbMask_start_stop_p[y_variable][0]=m_aaiMbMask_start_stop_p[y_variable][1]=-1; //Start with impossible Val
    m_aaiMbCompMask_start_stop_p[y_variable][0]=m_aaiMbCompMask_start_stop_p[y_variable][1]=-1; //Start with impossible Val

        for(Int x_variable=0;x_variable<(Int)m_uiMbSize;x_variable++){

            switch(x_variable){
                case 0:
                    if(m_aaiMbPartitionMask[y_variable][x_variable]>=(m_iMAX_mask_amplitude>>1)){
                        m_aaiMbMask_start_stop_p[y_variable][0]=0;
                        iPartition_label=1;
                    }
                    else{
                        m_aaiMbCompMask_start_stop_p[y_variable][0]=0;
                        iPartition_label=0;
                    }
                    break;
                default:
                    if(iPartition_label==1){
                        if(m_aaiMbPartitionMask[y_variable][x_variable]<(m_iMAX_mask_amplitude>>1)){
                            m_aaiMbMask_start_stop_p[y_variable][1]=x_variable;     
                            m_aaiMbCompMask_start_stop_p[y_variable][0]=x_variable;
                            iPartition_label=0;
                        }
                    }
                    else{
                        if(m_aaiMbPartitionMask[y_variable][x_variable]>=(m_iMAX_mask_amplitude>>1)){
                            m_aaiMbMask_start_stop_p[y_variable][0]=x_variable;     
                            m_aaiMbCompMask_start_stop_p[y_variable][1]=x_variable;
                            iPartition_label=1;
                        }
                    }     
                    break;
            }
            if(x_variable==(m_uiMbSize-1)){
                if(iPartition_label==1){    
                    m_aaiMbMask_start_stop_p[y_variable][1]=m_uiMbSize;
                    m_aaiMbMask_start_stop_p[y_variable][0]=m_aaiMbMask_start_stop_p[y_variable][0]==-1? m_uiMbSize : m_aaiMbMask_start_stop_p[y_variable][0];

                    CHECK(m_aaiMbCompMask_start_stop_p[y_variable][1]==-1 || (m_aaiMbCompMask_start_stop_p[y_variable][0]!=-1 && m_aaiMbCompMask_start_stop_p[y_variable][1]!=-1));
   
                    m_aaiMbCompMask_start_stop_p[y_variable][0]=(m_aaiMbCompMask_start_stop_p[y_variable][0]==-1? m_uiMbSize : m_aaiMbCompMask_start_stop_p[y_variable][0]);
                    m_aaiMbCompMask_start_stop_p[y_variable][1]=(m_aaiMbCompMask_start_stop_p[y_variable][1]==-1? m_uiMbSize : m_aaiMbCompMask_start_stop_p[y_variable][1]);
                }
                else{
                    m_aaiMbCompMask_start_stop_p[y_variable][1]=m_uiMbSize;
                    m_aaiMbCompMask_start_stop_p[y_variable][0]=(m_aaiMbCompMask_start_stop_p[y_variable][0]==-1? m_uiMbSize : m_aaiMbCompMask_start_stop_p[y_variable][0]);
                  
                    CHECK(m_aaiMbCompMask_start_stop_p[y_variable][1]==-1 || (m_aaiMbCompMask_start_stop_p[y_variable][0]!=-1 && m_aaiMbCompMask_start_stop_p[y_variable][1]!=-1));
                  
                    m_aaiMbMask_start_stop_p[y_variable][0]=(m_aaiMbMask_start_stop_p[y_variable][0]==-1? m_uiMbSize : m_aaiMbMask_start_stop_p[y_variable][0]);
                    m_aaiMbMask_start_stop_p[y_variable][1]=(m_aaiMbMask_start_stop_p[y_variable][1]==-1? m_uiMbSize : m_aaiMbMask_start_stop_p[y_variable][1]);
                }
            }
        }
        
        CHECK(m_aaiMbCompMask_start_stop_p[y_variable][0]!=-1 && m_aaiMbCompMask_start_stop_p[y_variable][1]!=-1);
        CHECK(m_aaiMbMask_start_stop_p[y_variable][0]!=-1 && m_aaiMbMask_start_stop_p[y_variable][1]!=-1);
    }

  
  return 0;
}

//Generate MbGeoLinearPartition mask of uiIndex
Int GeometricPartitionBlock::makeMbGeoLinearPartition (UInt uiIndex, Bool bDec){

  CHECK(uiIndex<m_uiNumberOfQuantizedEdges);

  float LineDistance=m_aafSetOfEdgeParameters[uiIndex][0];
  float LineAngle=m_aafSetOfEdgeParameters[uiIndex][1];

  return makeMbGeoLinearPartition (LineAngle,LineDistance, bDec);

}

//Generate MbPartitionMaskTable of all quantized edges
Int GeometricPartitionBlock::initMbPartitionMaskTable(GeometricPartitionBlock* m_cGeometricPartitionInter_Luma){

  m_uiNumberOfQuantizedEdges = m_cGeometricPartitionInter_Luma->m_uiNumberOfQuantizedEdges;
  //CHECK(m_uiNumberOfQuantizedEdges>0 && m_aafSetOfEdgeParameters!=NULL && m_uiMbSize>0);
  CHECK(m_aaaiMbPartitionMask_table==NULL);
  CHECK(m_aaiMbPartitionMask!=NULL);

  CHECK(m_aaaiMbComplementaryPartitionMask_table==NULL);
  CHECK(m_aaiComplementaryMbPartitionMask!=NULL);

  //Generate array of matrix
  m_aaaiMbPartitionMask_table=new Int**[m_uiNumberOfQuantizedEdges];
  m_aaaiMbComplementaryPartitionMask_table=new Int**[m_uiNumberOfQuantizedEdges];
  
    //Part for reducing complexity
  m_aaaiMbMask_start_stop_p=new Int**[m_uiNumberOfQuantizedEdges];
  m_aaaiMbCompMask_start_stop_p=new Int**[m_uiNumberOfQuantizedEdges];
  

  for(register Int index=0;index<m_uiNumberOfQuantizedEdges;index++){

    //Create memory buffer to hold mask
    m_aaaiMbPartitionMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbPartitionMask_table[index],m_uiMbSize,m_uiMbSize),m_nERR);

    m_aaaiMbComplementaryPartitionMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbComplementaryPartitionMask_table[index],m_uiMbSize,m_uiMbSize),m_nERR);


#ifdef GEOM
    for(Int y=0; y<m_uiMbSize; y++)
      for(Int x=0; x<m_uiMbSize; x++)
      {
        m_aaiMbPartitionMask[y][x] = m_cGeometricPartitionInter_Luma->m_aaaiMbPartitionMask_table[index][y<<1][x<<1];
        m_aaiComplementaryMbPartitionMask[y][x] = m_cGeometricPartitionInter_Luma->m_aaaiMbComplementaryPartitionMask_table[index][y<<1][x<<1];
      }
#else
    makeMbGeoLinearPartition(index);
    makeComplementaryMbPartitionMask();
#endif
    memcpy(m_aaaiMbPartitionMask_table[index][0],m_aaiMbPartitionMask[0],sizeof(Int)*m_uiMbSize*m_uiMbSize);
    memcpy(m_aaaiMbComplementaryPartitionMask_table[index][0],m_aaiComplementaryMbPartitionMask[0],sizeof(Int)*m_uiMbSize*m_uiMbSize);


      //set of tables to allocate the start-stop values for the mask partitions of each partition

//Part for reducing complexity
    m_aaaiMbMask_start_stop_p[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbMask_start_stop_p[index],m_uiMbSize,2),m_nERR);
    m_aaaiMbCompMask_start_stop_p[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbCompMask_start_stop_p[index],m_uiMbSize,2),m_nERR);

    memcpy(m_aaaiMbMask_start_stop_p[index][0],m_aaiMbMask_start_stop_p[0],sizeof(Int)*2*m_uiMbSize);
    memcpy(m_aaaiMbCompMask_start_stop_p[index][0],m_aaiMbCompMask_start_stop_p[0],sizeof(Int)*2*m_uiMbSize);
    
  }

  return 0;

}

Int GeometricPartitionBlock::initMbPartitionMaskTable(Bool bDec){

  CHECK(m_uiNumberOfQuantizedEdges>0 && m_aafSetOfEdgeParameters!=NULL && m_uiMbSize>0);
  CHECK(m_aaaiMbPartitionMask_table==NULL);
  CHECK(m_aaiMbPartitionMask!=NULL);

  CHECK(m_aaaiMbComplementaryPartitionMask_table==NULL);
  CHECK(m_aaiComplementaryMbPartitionMask!=NULL);

  //Generate array of matrix
  m_aaaiMbPartitionMask_table=new Int**[m_uiNumberOfQuantizedEdges];
  m_aaaiMbComplementaryPartitionMask_table=new Int**[m_uiNumberOfQuantizedEdges];
  
#ifdef GEOM
  m_aaacMbObmcMask_table=new Char**[m_uiNumberOfQuantizedEdges];
  m_aaacMbObmcMaskChroma_table=new Char**[m_uiNumberOfQuantizedEdges];
  m_aaacMbMVPMask_table=new Char**[m_uiNumberOfQuantizedEdges];
#ifdef GEOM_SPEED
  UInt uiDepth, uiNumPartitions;
  Bool bCreatMotionMask;
  if(bDec == false)
  {
    bCreatMotionMask = getNumPartitions(uiDepth, uiNumPartitions);
    if(bCreatMotionMask)
      m_aaaabMotionMask_table=new Bool***[m_uiNumberOfQuantizedEdges];
  }
#endif
#endif


    //Part for reducing complexity
  m_aaaiMbMask_start_stop_p=new Int**[m_uiNumberOfQuantizedEdges];
  m_aaaiMbCompMask_start_stop_p=new Int**[m_uiNumberOfQuantizedEdges];
  

  for(register Int index=0;index<(Int)m_uiNumberOfQuantizedEdges;index++){

    //Create memory buffer to hold mask
    m_aaaiMbPartitionMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbPartitionMask_table[index],m_uiMbSize,m_uiMbSize),m_nERR);

    m_aaaiMbComplementaryPartitionMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbComplementaryPartitionMask_table[index],m_uiMbSize,m_uiMbSize),m_nERR);

#ifdef GEOM
    m_aaacMbObmcMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaacMbObmcMask_table[index],m_uiMbSize,m_uiMbSize),m_nERR);

    m_aaacMbObmcMaskChroma_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaacMbObmcMaskChroma_table[index],m_uiMbSize/2,m_uiMbSize/2),m_nERR);

    m_aaacMbMVPMask_table[index]=NULL;
    RNOKR(AllocateMatrix(m_aaacMbMVPMask_table[index],2,NUM_GEO_MVP),m_nERR);
#ifdef GEOM_SPEED
    if(bDec == false)
    {
      if(bCreatMotionMask)
      {
        m_aaaabMotionMask_table[index]=NULL;
        RNOKR(AllocateMatrix(m_aaaabMotionMask_table[index],2,(g_uiMaxCUDepth - uiDepth)*3+1, uiNumPartitions),m_nERR);
      }
    }
#endif
#endif

    makeMbGeoLinearPartition(index, bDec);
    makeComplementaryMbPartitionMask();

    memcpy(m_aaaiMbPartitionMask_table[index][0],m_aaiMbPartitionMask[0],sizeof(Int)*m_uiMbSize*m_uiMbSize);
    memcpy(m_aaaiMbComplementaryPartitionMask_table[index][0],m_aaiComplementaryMbPartitionMask[0],sizeof(Int)*m_uiMbSize*m_uiMbSize);

#ifdef GEOM
    memcpy(m_aaacMbObmcMask_table[index][0],m_aacMbObmcMask[0],sizeof(Char)*m_uiMbSize*m_uiMbSize);
    memcpy(m_aaacMbObmcMaskChroma_table[index][0],m_aacMbObmcMaskChroma[0],sizeof(Char)*m_uiMbSize/2*m_uiMbSize/2);
    memcpy(m_aaacMbMVPMask_table[index][0],m_aacMbMVPMask[0],sizeof(Char)*2*NUM_GEO_MVP);
#ifdef GEOM_SPEED
    if(bDec == false)
    {
      if(bCreatMotionMask)
      {
        memcpy(m_aaaabMotionMask_table[index][0][0],m_aaabMotionMask[0][0],sizeof(Bool)*2*((g_uiMaxCUDepth - uiDepth)*3+1)*uiNumPartitions);
      }
    }
#endif
#endif
      //set of tables to allocate the start-stop values for the mask partitions of each partition

//Part for reducing complexity
    m_aaaiMbMask_start_stop_p[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbMask_start_stop_p[index],m_uiMbSize,2),m_nERR);
    m_aaaiMbCompMask_start_stop_p[index]=NULL;
    RNOKR(AllocateMatrix(m_aaaiMbCompMask_start_stop_p[index],m_uiMbSize,2),m_nERR);

    memcpy(m_aaaiMbMask_start_stop_p[index][0],m_aaiMbMask_start_stop_p[0],sizeof(Int)*2*m_uiMbSize);
    memcpy(m_aaaiMbCompMask_start_stop_p[index][0],m_aaiMbCompMask_start_stop_p[0],sizeof(Int)*2*m_uiMbSize);
    
  }

  return 0;

}

Int GeometricPartitionBlock::AllocateMatrix(float**& aafMatrixPointer,UInt uiSize_row,UInt uiSize_col){

  CHECK(aafMatrixPointer==NULL);
  CHECK(uiSize_row>0 && uiSize_col>0);

  aafMatrixPointer=new float*[uiSize_row];       
  aafMatrixPointer[0]=new float[uiSize_row*uiSize_col];

  //Build matrix, assigning pointers
  for(Int index=1;index<(Int)uiSize_row;index++){
    aafMatrixPointer[index]=&aafMatrixPointer[0][index*uiSize_col];
  }

  return 0;
}
#ifdef GEOM_SPEED
Int GeometricPartitionBlock::AllocateMatrix(Bool***& aaabMatrixPointer,UInt uiSize_row,UInt uiSize_col,UInt uiZ){

  CHECK(aaabMatrixPointer==NULL);
  CHECK(uiSize_row>0 && uiSize_col>0 && uiZ>0);

  aaabMatrixPointer=new Bool**[uiSize_row];  
  for(Int index=0;index<uiSize_row;index++){
    aaabMatrixPointer[index]=new Bool*[uiSize_col];
  }
   
  aaabMatrixPointer[0][0] = new Bool[uiSize_row*uiSize_col*uiZ];
  //Build matrix, assigning pointers
  for(Int index1=0;index1<uiSize_row;index1++){
    for(Int index2=0;index2<uiSize_col;index2++){
      if(index1!=0 || index2!=0)
        aaabMatrixPointer[index1][index2]=&aaabMatrixPointer[0][0][(index1*uiSize_col+index2)*uiZ];
    }
  }

  return 0;
}

Int GeometricPartitionBlock::FreeMatrix(Bool***& aaabMatrixPointer, UInt uiSize_row){

  CHECK(aaabMatrixPointer!=NULL);

  delete [] aaabMatrixPointer[0][0]; //Delete the continuous Data container.
  
  for(UInt i=0; i<uiSize_row; i++)
    delete [] aaabMatrixPointer[i];

  delete [] aaabMatrixPointer; //Delete the array containing the pointers to the rows.

  aaabMatrixPointer=NULL;

  return 0;
}
#endif


Int GeometricPartitionBlock::AllocateMatrix(Char**& aacMatrixPointer,UInt uiSize_row,UInt uiSize_col){

  CHECK(aacMatrixPointer==NULL);
  CHECK(uiSize_row>0 && uiSize_col>0);

  aacMatrixPointer=new Char*[uiSize_row];       
  aacMatrixPointer[0]=new Char[uiSize_row*uiSize_col];

  //Build matrix, assigning pointers
  for(Int index=1;index<uiSize_row;index++){
    aacMatrixPointer[index]=&aacMatrixPointer[0][index*uiSize_col];
  }

  return 0;
}
Int GeometricPartitionBlock::AllocateMatrix(Int**& aaiMatrixPointer,UInt uiSize_row,UInt uiSize_col){

  CHECK(aaiMatrixPointer==NULL);
  CHECK(uiSize_row>0 && uiSize_col>0);

  aaiMatrixPointer=new Int*[uiSize_row];       
  aaiMatrixPointer[0]=new Int[uiSize_row*uiSize_col];

  //Build matrix, assigning pointers
  for(Int index=1;index<uiSize_row;index++){
    aaiMatrixPointer[index]=&aaiMatrixPointer[0][index*uiSize_col];
  }

  return 0;
}
Int GeometricPartitionBlock::FreeMatrix(Char**& aacMatrixPointer){

  CHECK(aacMatrixPointer!=NULL);

  delete [] aacMatrixPointer[0]; //Delete the continuous Data container.
  delete [] aacMatrixPointer; //Delete the array containing the pointers to the rows.

  aacMatrixPointer=NULL;

  return 0;
}

Int GeometricPartitionBlock::FreeMatrix(Int**& aaiMatrixPointer){

  CHECK(aaiMatrixPointer!=NULL);

  delete [] aaiMatrixPointer[0]; //Delete the continuous Data container.
  delete [] aaiMatrixPointer; //Delete the array containing the pointers to the rows.

  aaiMatrixPointer=NULL;

  return 0;
}

Int GeometricPartitionBlock::FreeMatrix(float**& aaiMatrixPointer){

  CHECK(aaiMatrixPointer!=NULL);

  delete [] aaiMatrixPointer[0]; //Delete the continuous Data container.
  delete [] aaiMatrixPointer; //Delete the array containing the pointers to the rows.

  aaiMatrixPointer=NULL;

  return 0;
}

//Generate complementaryMask table for all quantized edges
Int GeometricPartitionBlock::makeComplementaryMbPartitionMask(){

  CHECK(m_aaiComplementaryMbPartitionMask!=NULL && m_aaiMbPartitionMask!=NULL);
  CHECK(m_uiMbSize>0);

  Int iMatrixSize=m_uiMbSize*m_uiMbSize;

  for(Int index=0;index<iMatrixSize;index++)
  {
    m_aaiComplementaryMbPartitionMask[0][index]= 1 - m_aaiMbPartitionMask[0][index];
  }

  return 0;
}


Int GeometricPartitionBlock::makeComplementaryMbPartitionMaskFromTable(Int iIndex){

  CHECK(m_aaaiMbPartitionMask_table!=NULL && m_aaiComplementaryMbPartitionMask!=NULL);
  CHECK(m_uiMbSize>0);
  CHECK(iIndex<(Int)m_uiNumberOfQuantizedEdges);

  Int iMatrixSize=m_uiMbSize*m_uiMbSize;

  for(Int index=0;index<iMatrixSize;index++)
  {
    m_aaiComplementaryMbPartitionMask[0][index]=m_iMAX_mask_amplitude-m_aaaiMbPartitionMask_table[iIndex][0][index];
  }

  return 0;
}

//Output mask table as binary value
Int GeometricPartitionBlock::makeBinMbGeoLinearPartition_fromStartStopTable(UInt uiEdgeIndex)
{
    
    CHECK(m_aaiMbPartitionMask!=NULL && m_uiMbSize>0);
    CHECK(uiEdgeIndex<m_uiNumberOfQuantizedEdges);
    
    Int **aiLimits=getMbMaskStartStopTable(uiEdgeIndex,0);
    
    for (Int iRows=0;iRows<(Int)m_uiMbSize;iRows++)
        for(Int iCols=0; iCols<(Int)m_uiMbSize;iCols++){
            if(iCols>=aiLimits[iRows][0] && iCols<aiLimits[iRows][1]){
                m_aaiMbPartitionMask[iRows][iCols]=1;
            }
            else{
                m_aaiMbPartitionMask[iRows][iCols]=0;
            }
        }
        
        return 0;
}


Int GeometricPartitionBlock::makeBinMbGeoLinearCompPartition_fromStartStopTable(UInt uiEdgeIndex){
    
    CHECK(m_aaiComplementaryMbPartitionMask!=NULL && m_uiMbSize>0);
    CHECK(uiEdgeIndex<m_uiNumberOfQuantizedEdges);
    
    Int **aiLimits=getMbMaskStartStopTable(uiEdgeIndex,1);
    
    for (Int iRows=0;iRows<(Int)m_uiMbSize;iRows++)
        for(Int iCols=0; iCols<(Int)m_uiMbSize;iCols++){
            if(iCols>=aiLimits[iRows][0] && iCols<aiLimits[iRows][1]){
                m_aaiComplementaryMbPartitionMask[iRows][iCols]=1;
            }   
            else{
                m_aaiComplementaryMbPartitionMask[iRows][iCols]=0;
            }
        }
        
        return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// FROM HERE: PROCEDURES FROM THE GeometricPartition Class, general container for holding the different instances of
// the class hereabove present

GeometricPartition::GeometricPartition(Bool bDec)
{
  //m_cGeometricPartition64x64Inter_Luma   =new GeometricPartitionBlock;
  //m_cGeometricPartition64x64Inter_Chroma =new GeometricPartitionBlock;

  m_cGeometricPartition32x32Inter_Luma   =new GeometricPartitionBlock;
  m_cGeometricPartition32x32Inter_Chroma =new GeometricPartitionBlock;

  m_cGeometricPartition16x16Inter_Luma   =new GeometricPartitionBlock;
  m_cGeometricPartition16x16Inter_Chroma =new GeometricPartitionBlock;

  


  //Init variables

  //m_cGeometricPartition64x64Inter_Luma->initMbPartitionMask(64);
  //m_cGeometricPartition64x64Inter_Chroma->initMbPartitionMask(32);


  m_cGeometricPartition32x32Inter_Luma->initMbPartitionMask(32, bDec);
  m_cGeometricPartition32x32Inter_Chroma->initMbPartitionMask(16, bDec);

  m_cGeometricPartition16x16Inter_Luma->initMbPartitionMask(16, bDec);
  m_cGeometricPartition16x16Inter_Chroma->initMbPartitionMask(8, bDec);
  
  m_bDictionairesInitialized=false;
  m_bLookUpTablesInitialized=false;
}

GeometricPartition::~GeometricPartition()
{
  //delete m_cGeometricPartition64x64Inter_Luma;
  //delete m_cGeometricPartition64x64Inter_Chroma;

  delete m_cGeometricPartition32x32Inter_Luma;
  delete m_cGeometricPartition32x32Inter_Chroma;

  delete m_cGeometricPartition16x16Inter_Luma;
  delete m_cGeometricPartition16x16Inter_Chroma;
}

Int  GeometricPartition::initEdgeDictionaries(float fQuantizationStepAngle, float fQuantizationStepDistance)
{
    
  if(m_bDictionairesInitialized) return 0;
    
  Int nErrval;

  // ----------------------------------  64 X 64 -------------------------------------------------
  //64x64 INTER GEO Modes
  //nErrval=m_cGeometricPartition64x64Inter_Luma->initMbEdgesDictionary(fQuantizationStepAngle*2,fQuantizationStepDistance, 1);
  //if(nErrval!=0) return nErrval;

  //nErrval=m_cGeometricPartition64x64Inter_Chroma->initMbEdgesDictionary(fQuantizationStepAngle*2,fQuantizationStepDistance*(float)0.5,1,1);
  //if(nErrval!=0) return nErrval;
  //
  //CHECK(m_cGeometricPartition64x64Inter_Chroma->getNumberOfQuantizedEdges()==m_cGeometricPartition64x64Inter_Luma->getNumberOfQuantizedEdges());

  // ----------------------------------  32 X 32 -------------------------------------------------
  //32x32 INTER GEO Modes
  nErrval=m_cGeometricPartition32x32Inter_Luma->initMbEdgesDictionary(fQuantizationStepAngle,fQuantizationStepDistance, 1);
  if(nErrval!=0) return nErrval;

  //nErrval=m_cGeometricPartition32x32Inter_Chroma->initMbEdgesDictionary(fQuantizationStepAngle*2,fQuantizationStepDistance/2,1,1);
  //if(nErrval!=0) return nErrval;
  //
  //CHECK(m_cGeometricPartition32x32Inter_Chroma->getNumberOfQuantizedEdges()==m_cGeometricPartition32x32Inter_Luma->getNumberOfQuantizedEdges());

  // ----------------------------------  16 X 16 -------------------------------------------------
  nErrval=m_cGeometricPartition16x16Inter_Luma->initMbEdgesDictionary(fQuantizationStepAngle,fQuantizationStepDistance);
  if(nErrval!=0) return nErrval;

  //nErrval=(m_cGeometricPartition16x16Inter_Chroma->initMbEdgesDictionary(fQuantizationStepAngle,fQuantizationStepDistance/2,1,1));
  //if(nErrval!=0) return nErrval;
  //
  //CHECK(m_cGeometricPartition16x16Inter_Chroma->getNumberOfQuantizedEdges()==m_cGeometricPartition16x16Inter_Luma->getNumberOfQuantizedEdges());
  
  
  m_bDictionairesInitialized=true; //Don't allow extra init
  
  return 0;

}

Int GeometricPartition::initLookUpTables(Bool bDec)
{
  
  if(m_bLookUpTablesInitialized) return 0;
    
  Int nErrval;

  nErrval=m_cGeometricPartition32x32Inter_Luma->initMbPartitionMaskTable(bDec);
  
  CHECK(nErrval==0);
  
#ifdef GEOM
  nErrval=m_cGeometricPartition32x32Inter_Chroma->initMbPartitionMaskTable(m_cGeometricPartition32x32Inter_Luma);
#else
  nErrval=m_cGeometricPartition32x32Inter_Chroma->initMbPartitionMaskTable();
#endif
  CHECK(nErrval==0);

  nErrval=m_cGeometricPartition16x16Inter_Luma->initMbPartitionMaskTable(bDec);

  CHECK(nErrval==0);

#ifdef GEOM
  nErrval=m_cGeometricPartition16x16Inter_Chroma->initMbPartitionMaskTable(m_cGeometricPartition16x16Inter_Luma);
#else
  nErrval=m_cGeometricPartition16x16Inter_Chroma->initMbPartitionMaskTable();
#endif

  CHECK(nErrval==0);
  
  m_bLookUpTablesInitialized=true; //Don't allow extra init

  return 0;
}

Int
GeometricPartition::create( GeometricPartition*& rpcGeometricPartition, Bool bDec )
{
  rpcGeometricPartition = new GeometricPartition (bDec);
  if( NULL == rpcGeometricPartition )
  {
    printf("no memory to allocate");
    exit(-1);
  }

  return 0;
}

Int
GeometricPartition::destroy()
{

  delete this;

  return 0;
}


