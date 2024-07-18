///////////////////////////////////////////////////////////////////////////////
// AdsTalkerDriver.h

#ifndef __ADSTALKERDRIVER_H__
#define __ADSTALKERDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ADSTALKERDRV_NAME        "ADSTALKER"
#define ADSTALKERDRV_Major       1
#define ADSTALKERDRV_Minor       0

#define DEVICE_CLASS CAdsTalkerDriver

#include "ObjDriver.h"

class CAdsTalkerDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ADSTALKERDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ADSTALKERDRV)
	VxD_Service( ADSTALKERDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ADSTALKERDRIVER_H__