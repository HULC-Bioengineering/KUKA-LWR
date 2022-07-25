# Gravity Compensation Mode
Gravity compensation mode can only be activated while in T1 or T2 teaching mode.

Gravity compensation mode allows the operator to move/hand orient the robot. Practically it drops the stiffness and dampening in each joint. It must also account for the weight of the tool as its weight adds forces/moments to the joints. Without a properly calibrated tool (weight, center of mass, mass moment of inertia) the robot will not freely float in space but rather rise or fall. **Calibrate your tool properly before using gravity compensation mode for best results**

Gravity compensation mode will activate in viscous mode for unstable starting orientations. Listed below is a stable starting orientation example:

    ptp {a1 0, a2 135, a3 -90, a4 0, a5 0, a6 0, e1 0}
