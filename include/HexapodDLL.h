#pragma once

#include "HexapodErrors.h"
//#include "C:\Users\HMMS\Documents\Corey\Hexapod\API\HexapodSDK\Samples\Cpp\Sample2_Cpp_MFC_DLL\Config.h"

// For a DLL designed to be called by other programming languages, we want
// to use __stdcall, and the C calling conventions rather than C++.
#ifdef __cplusplus
extern "C" {
#endif

#define HEXAPODDLL_CALLCONV __stdcall

// Export the names when building the DLL, and import the names when using the DLL.
#ifdef BUILD_HEXAPODDLL
#define HEXAPODDLL_API __declspec(dllexport)
#else
#define HEXAPODDLL_API __declspec(dllimport)
#endif


// Name and version
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetAssemblyName(
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetVersion(
				wchar_t* VersionBuffer, int BufferSize);

// Find devices
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPFindDevices(int DeviceNumber, int* pPortCount);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetDeviceAddress(int DeviceNumber, int index,
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetDeviceSerialNumber(int DeviceNumber, int index,
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetDeviceStatus(int DeviceNumber, int index, int* Status);

// Open and connect a device, and related properties.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPOpen(int* pDeviceNumber, const wchar_t* DeviceName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPClose(int DeviceNumber);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPConnect(int DeviceNumber, int Simulate,
			wchar_t* Address, wchar_t* SerialNumber);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPDisconnect(int DeviceNumber);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetConnected(int DeviceNumber, BOOL* Connected);
// Simulated is true if either opened with simulate parameter true, or device file has plugin of simulate.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetSimulated(int DeviceNumber, BOOL* Simulated);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetAddress(int DeviceNumber,
			wchar_t* Buffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetSerialNumber(int DeviceNumber,
			wchar_t* Buffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetConnectedAddress(int DeviceNumber,
			wchar_t* Buffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetConnectedSerialNumber(int DeviceNumber,
			wchar_t* Buffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetAddress(int DeviceNumber,
			wchar_t* Buffer);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetSerialNumber(int DeviceNumber,
			wchar_t* Buffer);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetReadCacheTime(int DeviceNumber, double* pCacheSeconds);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetReadCacheTime(int DeviceNumber, double cacheSeconds);

// Velocity
// Number from 0 to 1, applies to all axes.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetVelocity(int DeviceNumber, double* pVelocity);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetVelocity(int DeviceNumber, double velocity);

// Moves
// Units are mm and radians
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPHome(int DeviceNumber);

HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPTranslate(int DeviceNumber, BOOL Execute,
							const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double X, double Y, double Z);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPRotate(int DeviceNumber, BOOL Execute,
							const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double Rx, double Ry, double Rz);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPMoveTuple(int DeviceNumber, BOOL Execute,
							const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double* PositionTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPMoveMatrix(int DeviceNumber, BOOL Execute,
							const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double* PositionMatrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPMovePosition(int DeviceNumber, BOOL Execute,
							const wchar_t* PositionName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPStop(int DeviceNumber);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetMaxMoveDistanceTuple(int DeviceNumber,
							int PositionType, const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double* PositionTuple, double* NegDistance, double* PosDistance);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetMaxMoveDistanceMatrix(int DeviceNumber,
							int PositionType, const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							const double* const Matrix, double* NegDistance, double* PosDistance);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetMaxMoveOneDirectionTuple(int DeviceNumber,
							int PositionType, const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							double* StartPositionTuple, int AxisIndex, int Direction,
							double* Distance);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetMaxMoveOneDirectionMatrix(int DeviceNumber,
							int PositionType, const wchar_t* ObjectCS, const wchar_t* ReferenceCS,
							const double* const StartPositionMatrix, int AxisIndex,
							int Direction, double* Distance);

// Get and set leg lengths
// Leglengths are mm relative to leg home position.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPMoveLegs(int DeviceNumber, const double* LegLength,
							const int LegLengthSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionLegs(int DeviceNumber,
							int PositionType, double* LegLength, int LegLengthSize);

// Size in mm of one step of motor
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLegStepSize(int DeviceNumber,
							double* StepSize, int ArraySize);
// Leg minimum and maximum position.
// Position at home is 0.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetMinMaxPositionLegs(int DeviceNumber,
							double* MinPosition, double* MaxPosition, int ArraySize);
// Length between joints when legs are at home (position 0)
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetHomeLegLength(int DeviceNumber,
							double* LegLength, int LegLengthSize);
// Return joint locations for base and platform.
// Returns 18 numbers, X,Y,Z for 6 joints.
// Relative to Base or Platform CS.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetBaseJointLocation(int DeviceNumber,
							double* Location, int LocationSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPlatformJointLocation(int DeviceNumber,
							double* Location, int LocationSize);

// Return platform limits
// This is the min and max values of platform relative to base.
// See <PlatformLimitLower> and <PlatformLimitUpper> in the device file.
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPlatformLimits(int DeviceNumber,
							double* NegLimitTuple, double* PosLimitTuple);

// Get Position
// Units are mm and radians
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionMatrix(int DeviceNumber, int PositionType,
				const wchar_t* ObjectCS, const wchar_t* ReferenceCS, double* PositionMatrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionTuple(int DeviceNumber, int PositionType,
				const wchar_t* ObjectCS, const wchar_t* ReferenceCS, double* PositionTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetIsPositionValid(int DeviceNumber, int PositionType, BOOL* pIsValid);

// Get status
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetIsHomed(int DeviceNumber, BOOL* pIsHomed);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetIsHoming(int DeviceNumber, BOOL* pIsHoming);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetIsMoving(int DeviceNumber, BOOL* pIsMoving);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetInLimitSwitch(int DeviceNumber, BOOL* pInLimitSwitch);

// Simulate motion velocity
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetSimulateMotionVelocity(int DeviceNumber, BOOL* pSimulateMotionVelocity);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetSimulateMotionVelocity(int DeviceNumber, BOOL SimulateMotionVelocity);

// Coordinate systems
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSExists(int DeviceNumber, const wchar_t* Name, BOOL* pExists);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPCreateCS(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ReferenceCSName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPDeleteCS(int DeviceNumber, const wchar_t* Name);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPRenameCS(int DeviceNumber, const wchar_t* OldName,
				const wchar_t* NewName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetCSTransformTuple(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ReferenceCSName, const double* const TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetCSTransformMatrix(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ReferenceCSName, const double* const Matrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSTransformTuple(int DeviceNumber, const wchar_t* Name,
				double* TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSTransformMatrix(int DeviceNumber, const wchar_t* Name,
				double* TransformMatrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSCount(int DeviceNumber, int* pCount);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSName(int DeviceNumber, int CSNumber,
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSIsMovable(int DeviceNumber, const wchar_t* Name,
				int* pIsMovable);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSIsReadOnly(int DeviceNumber, const wchar_t* Name,
				int* pIsReadOnly);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSReference(int DeviceNumber, const wchar_t* Name,
				wchar_t* ReferenceNameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetCSDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* Description);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetCSDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* DescriptionBuffer, int BufferSize);

// Named Positions
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionExists(int DeviceNumber, const wchar_t* Name, BOOL* pExists);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPCreatePosition(int DeviceNumber, const wchar_t* Name);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPDeletePosition(int DeviceNumber, const wchar_t* Name);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPRenamePosition(int DeviceNumber, const wchar_t* OldName,
				const wchar_t* NewName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetPositionTransformTuple(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ObjectCSName, const wchar_t* ReferenceCSName, const double* const TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetPositionTransformMatrix(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ObjectCSName, const wchar_t* ReferenceCSName, const double* const Matrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionTransformTuple(int DeviceNumber, const wchar_t* Name,
				double* TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionTransformMatrix(int DeviceNumber, const wchar_t* Name,
				double* TransformMatrix);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionCount(int DeviceNumber, int* pCount);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionName(int DeviceNumber, int PositionNumber,
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionReference(int DeviceNumber, const wchar_t* Name,
				wchar_t* ReferenceNameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionObject(int DeviceNumber, const wchar_t* Name,
				wchar_t* ObjectNameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetPositionDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* Description);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPositionDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* DescriptionBuffer, int BufferSize);

// Limits
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitExists(int DeviceNumber, const wchar_t* Name, BOOL* pExists);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPCreateLimit(int DeviceNumber, const wchar_t* Name);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPDeleteLimit(int DeviceNumber, const wchar_t* Name);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPRenameLimit(int DeviceNumber, const wchar_t* OldName,
				const wchar_t* NewName);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetLimit(int DeviceNumber, const wchar_t* Name,
				const wchar_t* ObjectCSName, const wchar_t* ReferenceCSName,
				const double* const NegLimitTuple, const double* const PosLimitTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetNegLimit(int DeviceNumber, const wchar_t* Name,
				double* TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetPosLimit(int DeviceNumber, const wchar_t* Name,
				double* TaitBryanTuple);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitCount(int DeviceNumber, int* pCount);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitName(int DeviceNumber, int LimitNumber,
				wchar_t* NameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitReference(int DeviceNumber, const wchar_t* Name,
				wchar_t* ReferenceNameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitObject(int DeviceNumber, const wchar_t* Name,
				wchar_t* ObjectNameBuffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetLimitDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* Description);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetLimitDescription(int DeviceNumber, const wchar_t* Name,
				wchar_t* DescriptionBuffer, int BufferSize);

// Errors
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetErrorString(MHPError MHPErrorNumber,
				wchar_t* Buffer, int BufferSize);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPGetAsyncError(int DeviceNumber, wchar_t* Buffer, int BufferSize);

// Logging
HEXAPODDLL_API const wchar_t* HEXAPODDLL_CALLCONV MHPGetLogSeverityLevel(int DeviceNumber);
HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetLogSeverityLevel(int DeviceNumber, const wchar_t* Level);

HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPSetLoggingEnabled(int DeviceNumber, BOOL enabled);
HEXAPODDLL_API BOOL HEXAPODDLL_CALLCONV MHPGetLoggingEnabled(int DeviceNumber);

HEXAPODDLL_API MHPError HEXAPODDLL_CALLCONV MHPLogText(int DeviceNumber, const wchar_t* LevelText,
			const wchar_t* Text);

#ifdef __cplusplus
}  // end extern "C"
#endif
