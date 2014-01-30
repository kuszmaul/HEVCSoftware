# the SOURCE definiton lets you move your makefile to another position
CONFIG 				= CONSOLE

# set directories to your wanted values
SRC_DIR				= ../../../../source/App/TAppDecoder
INC_DIR				= ../../../../source/Lib
LIB_DIR				= ../../../../lib
BIN_DIR				= ../../../../bin

SRC_DIR1		=
SRC_DIR2		=
SRC_DIR3		=
SRC_DIR4		=

USER_INC_DIRS	= -I$(SRC_DIR) 
USER_LIB_DIRS	=

# intermediate directory for object files
OBJ_DIR				= ./objects

# set executable name
PRJ_NAME			= TAppDecoderAnalyser

# defines to set
DEFS				= -DMSYS_LINUX -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DMSYS_UNIX_LARGEFILE -DRExt__DECODER_DEBUG_BIT_STATISTICS=1

# set objects
OBJS          		= 	\
					$(OBJ_DIR)/decmain.o \
					$(OBJ_DIR)/TAppDecCfg.o \
					$(OBJ_DIR)/TAppDecTop.o \

# set libs to link with
LIBS				= -ldl

DEBUG_LIBS			=
RELEASE_LIBS		=

STAT_LIBS			= -lpthread
DYN_LIBS			=


DYN_DEBUG_LIBS		= -lTLibDecoderAnalyserd -lTLibCommond -lTLibVideoIOd -lTAppCommond
DYN_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderAnalyserd.a $(LIB_DIR)/libTLibCommond.a $(LIB_DIR)/libTLibVideoIOd.a $(LIB_DIR)/libTAppCommond.a
STAT_DEBUG_LIBS		= -lTLibDecoderAnalyserStaticd -lTLibCommonStaticd -lTLibVideoIOStaticd -lTAppCommonStaticd
STAT_DEBUG_PREREQS		= $(LIB_DIR)/libTLibDecoderAnalyserStaticd.a $(LIB_DIR)/libTLibCommonStaticd.a $(LIB_DIR)/libTLibVideoIOStaticd.a $(LIB_DIR)/libTAppCommonStaticd.a

DYN_RELEASE_LIBS	= -lTLibDecoderAnalyser -lTLibCommon -lTLibVideoIO -lTAppCommon
DYN_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoderAnalyser.a $(LIB_DIR)/libTLibCommon.a $(LIB_DIR)/libTLibVideoIO.a $(LIB_DIR)/libTAppCommon.a
STAT_RELEASE_LIBS	= -lTLibDecoderAnalyserStatic -lTLibCommonStatic -lTLibVideoIOStatic -lTAppCommonStatic
STAT_RELEASE_PREREQS	= $(LIB_DIR)/libTLibDecoderAnalyserStatic.a $(LIB_DIR)/libTLibCommonStatic.a $(LIB_DIR)/libTLibVideoIOStatic.a $(LIB_DIR)/libTAppCommonStatic.a


# name of the base makefile
MAKE_FILE_NAME		= ../../common/makefile.base

# include the base makefile
include $(MAKE_FILE_NAME)
