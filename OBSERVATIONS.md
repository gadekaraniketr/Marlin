# Homing
- To establish the "Machine Zero" (0,0) coordinate.
- Physically move the axes to your desired (0,0) position and send G92 X0 Y0 to tell Marlin "this is zero".

# Parking
- Used at the end of a job or when pausing a print to move the toolhead out of the way so you can access the bed.
- Inbuilt parking needs Z axis to be enabled but we can create parking command GXXX Xxxx Yxxx Fxxx

# Shapes
## Circle
- G2 X[X_Value_mm] Y[Y_Value_mm] I[Radius_mm] J0 P[Turns-1]

# Lead In/Out
- 