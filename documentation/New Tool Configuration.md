# New Tool Configuration
Two step calibration:

1. coordinate system
2. payload data


## Coordinate system
Log on with *Expert* permissions. 

    Configure->User group->Expert

Open the XYZ 4-Point menu

    Setup->Measure->Tool->XYZ 4-Point


Select a Tool no. (should not be in use) and name your tool then click *Continue*. Next we will put the robot into 3 different orientations maintaining the reference point of the tool remaining in the same position. KUKA software will calculate the new tool coordinate system and offset from the end effector place for us automatically. 

## Payload Data
Log on with *Expert* permissions. 

    Configure->User group->Expert

Open the Payload data menu:

    Setup->Measure->Tool->Payload data
    
Select tool no. from the last step. Tool name, X,Y,Z should be populated for the tool selected. Click *Continue*. Enter the load data for the tool (Mass, Center of mass (X,Y,Z), and the Orientation (A,B,C) of the Moment of intertia (JX, JY, JZ). For simple purposes just put in the mass (kg) and leave the rest at 0.


## Select Your Newly Configured Tool

Select this tool configuration globally (by selecting from main menu) or locally (with a KUKA program open):

    Configure->Set tool/base
    
Yay you're calibrated. As a test throw this bad boy in gravity compensation mode and ensure your tool does not float or sink.