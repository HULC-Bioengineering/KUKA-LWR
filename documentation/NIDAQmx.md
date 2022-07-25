#NIDAQmx
Since I absolutely can't stand LabView we are going to be using the DAQmx ANSI C Dev libraries directly. Our current setup has an ATI load cell plugged into an NI-DAQ 
which then plugs in via USB. If we could go from ATI load cell to USB we would but unfortunately that's not the case. 
Though the NI software is very impressive compared to ATI so we can be thankful of that.

To begin download and install the NIDAQmx library (version 14 at the time of writing).

#Library Directories, Compiling, and the Linker
To add the library into a Visual Studio project we must compile and link as follows

##VC++->Library Directories
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\lib\msvc

##C/C++->General->Additional Library Directories
C:\Program Files %28x86%29\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include
##Linker->Additional Library Directories
NIDAQmx.lib

Now we should be able to `#include <NIDAQmx.h>`. NI provides a wack of examples in the folder: National Instruments/NI-DAQ/Examples. 
The bulk of our code is modeled off the example: Analog Out/Generate Voltage/Cont Gen Volt Wfm-Ext Clk/VC_ContGen_IntClk_AnlgStart