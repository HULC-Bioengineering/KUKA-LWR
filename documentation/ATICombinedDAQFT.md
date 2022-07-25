#ATICombinedDAQFT
You've come to the right place if you're looking to get access to the ATICombinedFT library in your project; `#include <ftconfig.h>`

#Installation
The [ATICombinedDAQFT] download called "ATICombinedDAQFT Class Library and Windows Demo Source Code". Unzip and place this folder somewhere important to you/your project.
[ATICombinedDAQFT]:http://www.ati-ia.com/Products/ft/software/daq_software.aspx

Lets then open and build that .sln in Visual Studio which should create the lib file: ATICombinedDAQFT.lib in the projects /Release folder. Move that file to a place of interest.

#Slight Modification
We need to modify the files: `dom.h` and `dom.c`. The DOM_Exception is being re-declaired in multiple files and will throw a compilation error without these changes. (took forever to find this issue).
###Dom.h
Line 54 should be changed to `extern unsigned short DOM_Exception`
###Dom.c
Line 42 should be changed to `unsigned short DOM_Exception`

#Library Directories, Compiling, and the Linker
To add the library into a Visual Studio project we must compile and link as follows

##VC++->Include Directories
< path >/ATICombinedDAQFT;

##Linker->Additional Library Directories
ATICombinedDAQFT.lib

Now we should be able to `#include <ftconfig.h>`. ATI provides 2 really solid examples for calibration, biasing, and transforming in their [ATIDAQ C Library]
[ATIDAQ C Library]:http://www.ati-ia.com/Products/ft/software/daq_software.aspx