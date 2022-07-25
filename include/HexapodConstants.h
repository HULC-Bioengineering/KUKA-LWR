#pragma once

// Types of positions. Used in GetPosition() function.
// 	Actual		The current position. If no feedback on the motors, this
//				is the same as the commanded position.
//	Commmanded	The position currently commanded to the motors.
//	Executed	The commanded position when finished with any currently
//				ongoing move. If not moving, this is the same as
//				the commanded position.
//	Virtual		The commanded position after any virtual moves are
//				performed. If you execute all pending non-executed
//				moves, the system would be at this commanded position.
#define MHPPositionActual 		0
#define MHPPositionCommanded	1
#define MHPPositionExecuted		2
#define MHPPositionVirtual		3

// Number of legs.
// All arrays of leg lengths must be this size.
#define MHPLegCount 6

// Number of elements in TaitBryanTuple
#define TaitBryanTupleSize 6

// Port connection status
#define MHPPortStatusConnected		0
#define MHPPortStatusAvailable		1
#define MHPPortStatusBusy			2
#define MHPPortStatusNoResponse		3

