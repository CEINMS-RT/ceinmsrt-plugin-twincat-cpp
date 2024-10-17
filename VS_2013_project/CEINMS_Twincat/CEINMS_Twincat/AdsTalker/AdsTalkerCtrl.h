///////////////////////////////////////////////////////////////////////////////
// AdsTalkerCtrl.h

#ifndef __ADSTALKERCTRL_H__
#define __ADSTALKERCTRL_H__

#include <atlbase.h>
#include <atlcom.h>

#define ADSTALKERDRV_NAME "ADSTALKER"

#include "resource.h"       // main symbols
#include "AdsTalkerW32.h"
#include "TcBase.h"
#include "AdsTalkerClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CAdsTalkerCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CAdsTalkerCtrl, &CLSID_AdsTalkerCtrl>
	, public IAdsTalkerCtrl
	, public ITcOCFCtrlImpl<CAdsTalkerCtrl, CAdsTalkerClassFactory>
{
public:
	CAdsTalkerCtrl();
	virtual ~CAdsTalkerCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ADSTALKERCTRL)
DECLARE_NOT_AGGREGATABLE(CAdsTalkerCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CAdsTalkerCtrl)
	COM_INTERFACE_ENTRY(IAdsTalkerCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ADSTALKERCTRL_H__
