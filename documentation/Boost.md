# Boost
Boost is awesome. You should definitely install it to make life in C++ much easier.

## Installiation
Go to boost.org and download the latest version of boost (1.56.0 at the time of writing this). Once downloaded, open the command prompt and change your current directory to the Boost root directory. Then, type the following commands:

    bootstrap
    .\b2

The first command prepares the Boost.Build system for use. The second command invokes Boost.Build to build the separately-compiled Boost libraries.

Lets knowledgify your IDE with boost!

## IDE Knowledgification

Add the file path ```<boost_root>/<boost_version>/stage/libs/``` to:

    Properties->VC++ Directories->Include Directories
    Properties->VC++ Directories->Library Directories
    Properties->Linker->General->Additional Library Directories
    
Add the file path ```<boost_root>/<boost_version>/``` to:

    Properties->C/C++->Additional Include Directories
    
And lastly, don't you dare precompile those headers (*Not Using Precompiled Headers*).

    Properties->C/C++->Precompile Headers->Precompiled Header      
