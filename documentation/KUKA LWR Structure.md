#KUKA LWR Structure
The project KUKA LWR has a lot of compiling and linking that goes on behind the scenes, hidden, waiting, lurking. Here I will outline all the necessary Visual Studio project property changes which has brought us to our clean .sln which contains only our singular KUKA project.

#Project Layout
The root folder: `KUKA LWR`. Inside this folder we have the following folders:
* external - External files
 * ATICombinedDAQFT/ - Load cell calibration, bias, transformation, etc.
 * boost_1_57_0/ - Libraries that work well with the C++ Standard Library
 * FRILibrary/ - Simple user interface to the KUKA Light-Weight Robot IV
 * cpplint.py - Google C++ linter
* include - Header (.h) files
  * LoadCell.h
  * LWR.h
  * LWRLogParser.h
  * Utils.h
* libs - Compiled static libraries (.lib) usually from external files
 * ATICombinedDAQFT.lib
 * FastResearchLibrary.h
 * pthreadVC.h - Threading library needed by FastResearchLibrary
 * TypeIRML.h - Reflex motion library
* src - Source (.cpp) files
 * execute.cpp
 * LoadCell.cpp
 * LWR.cpp
 * LWRLogParser.cpp
 * Utils.cpp
* vsprojects - Visual Studio solution (.sln) and project (.vcxproj)
 * KUKA.sln
 * KUKA/ - KUKA project folder

Note, some of the contents in each folder might have changed from the time of writing.

Lovely, we have our project all nicely organized. But how does our project (KUKA.vcxproj) compile and link against with this organization? Great of you to ask, lets find out.
#Library Directories, Compiling, and the Linker
We use relative paths, with the exception to the NI-DAQmx library as this must be installed on your system/Program Files. For more information on setting up the NI-DAQmx see the NI-DAQmx.md. Relative paths entail that our project stays cute and self contained.

##VC++->Include Directories
```
..\..\external\FRILibrary\include;
..\..\external\FRILibrary\Windows\pthread;
..\..\external\ATICombinedDAQFT;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
..\..\include;
$(VC_IncludePath);
$(WindowsSDK_IncludePath);
```

##VC++->Library Directories
```
..\..\external\FRILibrary\Windows\pthread;
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
< path >\National Instruments\NI-DAQ\DAQmx ANSI C Dev\lib\msvc;
..\..\libs;
$(LibraryPath)
```

##VC++->Source Directories
```
..\..\src;
$(SourcePath)
```

##C/C++->General->Additional Include Directories
```
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include;
..\..\external\boost_1_57_0\boost_1_57_0;
%(AdditionalIncludeDirectories)
```
##Linker->General->Additional Library Directories
```
..\..\external\boost_1_57_0\boost_1_57_0\stage\lib;
%(AdditionalLibraryDirectories)
```

##Linker->Input->Additional Dependencies
```
ATICombinedDAQFT.lib;
pthreadVC.lib;
TypeIRML.lib;
FastResearchLibrary.lib;
pthreadVC.lib;
NIDAQmx.lib;
kernel32.lib;user32.lib;gdi32.lib;winspool.lib;comdlg32.lib;advapi32.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;odbc32.lib;odbccp32.lib;
%(AdditionalDependencies)
```

Its just that easy!