Header header

Nmea_Time time    
Nmea_Pos  pos
int32     status        # < Status (A = active or V = void)    
float64   speed         # < Speed over the ground in knots
float64   direction     # < Track angle in degrees True
float64   declination   # < Magnetic variation degrees (Easterly var. subtracts from true course)
int32     declin_ew     # < [E]ast or [W]est
int32     mode          # < Mode indicator of fix type (A = autonomous, D = differential, E = estimated, N = not valid, S = simulator)

