#ifndef __SERIALPORTENUM_H__
#define __SERIALPORTENUM_H__

#include "stdio.h"
#include "stdlib.h"
#include "malloc.h"
#include "crtdbg.h"
#include "setupapi.h"


#define MAX_NAME_PORTS 256
#define RegDisposition_OpenExisting (0x00000001) // open key only if exists
#define CM_REGISTRY_HARDWARE (0x00000000)

typedef DWORD (WINAPI *CM_Open_DevNode_Key)(DWORD, DWORD, DWORD, DWORD, ::PHKEY, DWORD);


class CSerialPortEnum 
{
public:

  CSerialPortEnum();
  ~CSerialPortEnum();

  bool BeginEnum();
  bool EnumNext(char *portName, int& portNumber);
  void EndEnum();

private:

  HANDLE hDeviceInfoSet;

  static HANDLE CSerialPortEnum::beginEnumeratePorts(VOID);
  static bool CSerialPortEnum::enumeratePortsNext(HANDLE DeviceInfoSet, char *lpBuffer, int& portNo);
  static void CSerialPortEnum::endEnumeratePorts(HANDLE DeviceInfoSet);
};
#endif // __SERIALPORTENUM_H__

