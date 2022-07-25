#pragma once
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS // some CString constructors will be explicit

#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <atlbase.h>
#include <iostream>

#import "C:\Program Files\SolidWorks Corp\SolidWorks\sldworks.tlb" raw_interfaces_only, raw_native_types, no_namespace, named_guids  //the SOLIDWORKS type library
#import "C:\Program Files\SolidWorks Corp\SolidWorks\swconst.tlb"  raw_interfaces_only, raw_native_types, no_namespace, named_guids  //the SOLIDWORKS constant type libraryl
#import "C:\Program Files\SolidWorks Corp\SolidWorks\Simulation\cosworks.tlb" raw_interfaces_only, raw_native_types, no_namespace, named_guids // COSMOS maybe...