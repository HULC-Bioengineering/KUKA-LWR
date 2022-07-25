#KUKA LWR Structure
The project KUKA LWR has a lot of compiling and linking that goes on behind the scenes, hidden, waiting, lurking. Here I will outline all the necessary Visual Studio project property changes which has brought us to our clean .sln which contains only our singular KUKA project.

#Project Layout
The root folder: `KUKA LWR` is such:
* calibration
  * 980039-FRI-Driver.init
  * FT13036Nano25ReCalibrated.cal
* experiments
* external - External files
 * ATICombinedDAQFT/ - Load cell calibration, bias, transformation, etc.
 * boost_1_57_0/ - Libraries that work well with the C++ Standard Library
 * DSPFiltersComplete/ - C++ filters library
 * eigen/ - Rotations and transformations
 * FRILibrary/ - Simple user interface to the KUKA Light-Weight Robot IV
 * ReflexesII/ - Cartesian and joint path planning
 * cpplint.py - Google C++ linter
* include - Header (.h) files
  * Config.h
  * LWR.h
  * LoadCell.h
  * MotionParser.h
  * Nano25E.h
  * PID.h
  * Scripts.h
  * Statistics.h
  * utils.h
* libs - Compiled static libraries (.lib) usually from external files
  * release 
    * ATICombinedDAQFT.lib
    * DSPFilters.lib
    * FastResearchLibrary.lib
    * KUKA.lib
    * LoadCell.lib
    * MotionParser.lib
    * PID.lib
    * Statistics.lib
    * TypeIRML.lib - Reflex motion library
    * pthreadVC.lib  - Threading library needed by FastResearchLibrary
* src - Source (.cpp) files
  * LWR.cpp
  * MotionParser.cpp
  * Nano25E.cpp
  * PID.cpp
  * Scripts
    * FollowPath.cpp
    * FreeMovementPoseLoadRecord.cpp
    * GravityCompensatedCartesianCollaboration.cpp
    * NavByBendingForces.cpp
    * Record.cpp
    * Replay.cpp
  * execute.cpp
  * LoadCell.cpp
  * Statistics.cpp
  * Utils.cpp
* vsprojects - Visual Studio solution (.sln) and project (.vcxproj)
  * KUKA.sln
  * KUKA
    * KUKA.vcxproj
    * KUKA.vcxproj.filters
    * KUKA.vcxproj.user
    * KUKA.sln
  * LoadCell
    * LoadCell.vcxproj
    * LoadCell.vcxproj.filters
  * MotionParser
    * MotionParser.vcxproj
    * MotionParser.vcxproj.filters
    * MotionParser.vcxproj.user
  * PID
    * PID.vcxproj
    * PID.vcxproj.filters
  * Scripts
    * Scripts.vcxproj
    * Scripts.vcxproj.filters
    * Scripts.vcxproj.user

Note, some of the contents in each folder might have changed from the time of writing.

Lovely, we have our project all nicely organized. But how does our project (KUKA.vcxproj) compile and link against with this organization? Great of you to ask, lets find out.

#Library Directories, Compiling, and the Linker
We use relative paths, with the exception to the NI-DAQmx library as this must be installed on your system/Program Files. For more information on setting up the NI-DAQmx see the NI-DAQmx.md. Relative paths entail that our project stays cute and self contained. 

Compiling uses the dynamic runtime library (\Md) for windowsx32 release.

Our (.sln) consists of multiple projects which are compiled into static library (.lib) files, with the exception of pthreadVC which is compiled into a dynamically linked library (.dll). Projects:

* ATICombinedDAQFT [EXTERNAL]
* DSPFilters [EXTERNAL]
* FastResearchLibrary [EXTERNAL]
* KUKA
* LoadCell
* MotionParser
* PID
* pthread [EXTERNAL]
* Scripts
* Statistics
* TypeIRML [EXTERNAL]

## KUKA

###VC++->Include Directories
```
..\..\external\DSPFiltersComplete\shared\DSPFilters\include;
..\..\external\FRILibrary\include;
..\..\external\FRILibrary\Windows\pthread;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\include;
```

###VC++->Library Directories
```
..\..\external\FRILibrary\Windows\pthread\Release\;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\libs\release;
```

###VC++->Source Directories
```
..\..\src;
```

###C/C++->General->Additional Include Directories
```
..\..\external\eigen;
..\..\external\boost_1_57_0\boost_1_57_0;
```

###Linker->General->Additional Library Directories
```
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
```

###Linker->Input->Additional Dependencies
```
PID.lib;
pthreadVC.lib;
TypeIRML.lib;
FastResearchLibrary.lib;
```

## LoadCell

###VC++->Include Directories
```
..\..\external\DSPFiltersComplete\shared\DSPFilters\include;
..\..\external\ATICombinedDAQFT;
..\..\include;
```

###VC++->Library Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\lib\msvc;
..\..\libs\release;
```

###VC++->Source Directories
```
..\..\src;
```

###C/C++->General->Additional Include Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include;
```

###Linker->Input->Additional Dependencies
```
ATICombinedDAQFT.lib;
NIDAQmx.lib;
```

## MotionParser & Statistics

###VC++->Include Directories
```
..\..\external\eigen;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\include;
```

###VC++->Library Directories
```
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
```

###VC++->Source Directories
```
..\..\src;
..\..\eigen;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
```

###C/C++->General->Additional Include Directories
```
..\..\external\boost_1_57_0\boost_1_57_0;
```

###Linker->General->Additional Library Directories
```
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
```

## PID

###VC++->Include Directories
```
..\..\include;
```

###VC++->Source Directories
```
..\..\src;
```

## Scripts

###VC++->Include Directories
```
..\..\external\FRILibrary\Windows\pthread;
..\..\external\FRILibrary\include;
..\..\external\eigen;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\external\DSPFiltersComplete\shared\DSPFilters\include;
..\..\external\ATICombinedDAQFT;
..\..\include;
```

###VC++->Library Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\lib\msvc;
..\..\libs\release;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
```

###VC++->Source Directories
```
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\external\eigen\Eigen\src;
..\..\src\Scripts;
```

###C/C++->General->Additional Include Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include;
..\..\external\boost_1_57_0\boost_1_57_0;
%(AdditionalIncludeDirectories)
```

###Linker->General->Additional Library Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include;
..\..\external\ATICombinedDAQFT;
..\..\external\boost_1_57_0\boost_1_57_0;
```

###Linker->Input->Additional Dependencies
```
ATICombinedDAQFT.lib;
NIDAQmx.lib;
TypeIRML.lib;
DSPFilters.lib;
FastResearchLibrary.lib;
KUKA.lib;
MotionParser.lib;
PID.lib;
Statistics.lib;
pthreadVC.lib;
LoadCell.lib;
```

Its just that easy!
