#include "stdafx.h"
#include "setupapi.h"
#include "SerialPortEnum.h"

CSerialPortEnum::CSerialPortEnum()
{
  hDeviceInfoSet = INVALID_HANDLE_VALUE;
}

CSerialPortEnum::~CSerialPortEnum()
{
  _ASSERT(INVALID_HANDLE_VALUE == hDeviceInfoSet); //failed to call EndEnum if this asserts
}

bool CSerialPortEnum::BeginEnum()
{
  _ASSERT (INVALID_HANDLE_VALUE == hDeviceInfoSet);

  hDeviceInfoSet = beginEnumeratePorts();

  return (INVALID_HANDLE_VALUE != hDeviceInfoSet);
}

bool CSerialPortEnum::EnumNext(char *portName, int& portNumber)
{
  int portNo;

  char buffer[MAX_NAME_PORTS];

  _ASSERT (INVALID_HANDLE_VALUE != hDeviceInfoSet);

  if (!enumeratePortsNext(hDeviceInfoSet, buffer, portNo))
    return false;

  sprintf_s(portName, MAX_NAME_PORTS, "%s", buffer);
  portNumber = portNo;

  return true;
}

void CSerialPortEnum::EndEnum()
{
  endEnumeratePorts(hDeviceInfoSet);
  hDeviceInfoSet = INVALID_HANDLE_VALUE;
}

HANDLE CSerialPortEnum::beginEnumeratePorts()
{
  _GUID* buf;
  HDEVINFO deviceInfoSet;
  BOOL guidTest=FALSE;
  DWORD requiredSize = 0;

  guidTest = SetupDiClassGuidsFromNameA("Ports", 0, 0, &requiredSize);

  if (requiredSize < 1)
    return INVALID_HANDLE_VALUE;

  buf = (_GUID*)malloc(requiredSize * sizeof(GUID));

  //get GUID of the ports class:
  guidTest = SetupDiClassGuidsFromNameA("Ports", buf, requiredSize * sizeof(GUID), &requiredSize);

  if (!guidTest)
    return INVALID_HANDLE_VALUE;

  //get devices within ports class:
  deviceInfoSet = SetupDiGetClassDevs(buf, NULL, NULL, DIGCF_PRESENT);

  free(buf);

  return deviceInfoSet;
}

bool CSerialPortEnum::enumeratePortsNext(HANDLE deviceInfoSet, char* lpBuffer, int& portNo)
{
  int         res1;
  int         numport;
  static int  numDev = 0;
  static      HINSTANCE cfgMan;
  
  unsigned char devName[MAX_NAME_PORTS] = {0};
  SP_DEVINFO_DATA deviceInfoData = {0};
  static CM_Open_DevNode_Key openDevNodeKey = NULL;

  deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

  if (!deviceInfoSet || !lpBuffer)
    return false;

  if (openDevNodeKey == NULL) 
  {
    cfgMan = LoadLibrary(TEXT("cfgmgr32.dll"));

    if (!cfgMan)
      return false;
      
    openDevNodeKey = (CM_Open_DevNode_Key)GetProcAddress(cfgMan, "CM_Open_DevNode_Key");

    if (!openDevNodeKey) 
    {
      FreeLibrary(cfgMan);
      
      return false;
    }

    numDev = 0;
  }

  while (TRUE) 
  {
    HKEY keyDevice;
    DWORD len;

    res1 = SetupDiEnumDeviceInfo(deviceInfoSet, numDev, &deviceInfoData);

    if (!res1) 
    {
      SetupDiDestroyDeviceInfoList(deviceInfoSet);
      FreeLibrary(cfgMan);
      openDevNodeKey = NULL;

      return false;
    }

    res1 = openDevNodeKey(deviceInfoData.DevInst, KEY_QUERY_VALUE, 0,
                          RegDisposition_OpenExisting, &keyDevice, CM_REGISTRY_HARDWARE);

    if (res1 != ERROR_SUCCESS)
      return false;

    len = MAX_NAME_PORTS;

    res1 = RegQueryValueEx(keyDevice, (LPCWSTR)"portname", NULL, NULL, devName, &len);

    RegCloseKey(keyDevice);

    if (res1 != ERROR_SUCCESS)
      return false;

    numDev++;
    
    if (_memicmp(devName, "com", 3))
      continue;

    numport = atoi((const char*)(devName + 3));

    if (numport > 0 && numport <= 256) 
    {
      strcpy_s(lpBuffer, MAX_NAME_PORTS, (const char*)devName);
      portNo = numport;
      return true;
    }

    FreeLibrary(cfgMan);
    openDevNodeKey = NULL;
    return false;
  }
}

void CSerialPortEnum::endEnumeratePorts(HANDLE deviceInfoSet)
{
  SetupDiDestroyDeviceInfoList(deviceInfoSet);
}
