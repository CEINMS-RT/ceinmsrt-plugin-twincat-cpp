///////////////////////////////////////////////////////////////////////////////
// AdsTalkerServices.h

#pragma once

#include "TcServices.h"

const ULONG DrvID_AdsTalker = 0x3F000000;
#define SRVNAME_ADSTALKER "AdsTalker"

///<AutoGeneratedContent id="ClassIDs">
const CTCID CID_AdsTalkerCAdsTalker = {0xf7282152,0x786b,0x4e67,{0xa9,0xfd,0xc4,0x5f,0x6e,0xf6,0xa1,0xa7}};
///</AutoGeneratedContent>

///<AutoGeneratedContent id="ParameterIDs">
const PTCID PID_AdsTalkerParameter = 0x00000001;
///</AutoGeneratedContent>

///<AutoGeneratedContent id="DataTypes">
typedef struct _AdsTalkerParameter
{
	ULONG data1;
	ULONG data2;
	double data3;
} AdsTalkerParameter, *PAdsTalkerParameter;

typedef struct _AdsTalkerInputs
{
	double TorqueLeftAnkle;
	double PositionLeftAnkle;
	double PositionRightAnkle;
	double TorqueRightAnkle;
} AdsTalkerInputs, *PAdsTalkerInputs;

typedef struct _AdsTalkerOutputs
{
	double TorqueCommandLeftAnkle;
	double TorqueCommandRightAnkle;
} AdsTalkerOutputs, *PAdsTalkerOutputs;

///</AutoGeneratedContent>



///<AutoGeneratedContent id="DataAreaIDs">
#define ADI_AdsTalkerInputs 0
#define ADI_AdsTalkerOutputs 1
///</AutoGeneratedContent>

///<AutoGeneratedContent id="InterfaceIDs">
///</AutoGeneratedContent>
