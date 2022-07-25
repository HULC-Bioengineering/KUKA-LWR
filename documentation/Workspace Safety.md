# Workspace Safety

There are two types of workspaces: cartesian, and axis-specific. Cartesian allows you to make a safety cube of sorts, while axis specific allows you to limit ranges of motion on certain axis. Measurements are taken from the tool center point (TCP).

## Monitor Rob. Position
Monitoring menu

    Monitor->Rob.Position

We can have a live look at the robots Cartesian or Axis-specific values. Real-time goodness.

## Safety Workspace

Log on with *Expert* permissions. 

    Configure->User group->Expert
    
Open the Monitoring working envelope configuration menu

    Configure->Miscellaneous->Monitoring working envelope->Configuration

Select either *Cartesian* or *Axis spec*.


### Mode    

Enter your information and for mode select *OUTSIDE_STOP*. This mode will hault the KUKA if we try and move out of our workspace. There is no variable checking, callbacks, or any overhead on our part; the robot will just stop. Technically the KUKA is watching a *Signal* variable which toggles from FALSE to TRUE (*default*) when the KUKA exceeds our safety limit(s). In *OUTSIDE* mode the robot won't automatically stop so it's left up to us how we want to interpret the signal variable.


## Freebird

In *Expert* mode we have the ability to override the safety limits in T1 teaching mode and let the KUKA roam free.