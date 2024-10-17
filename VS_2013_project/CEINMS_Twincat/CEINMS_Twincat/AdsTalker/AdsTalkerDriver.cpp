///////////////////////////////////////////////////////////////////////////////
// AdsTalkerDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "AdsTalkerDriver.h"
#include "AdsTalkerClassFactory.h"

DECLARE_GENERIC_DEVICE(ADSTALKERDRV)

IOSTATUS CAdsTalkerDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CAdsTalkerClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CAdsTalkerDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CAdsTalkerDriver::ADSTALKERDRV_GetVersion( )
{
	return( (ADSTALKERDRV_Major << 8) | ADSTALKERDRV_Minor );
}

