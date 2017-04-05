Header header
Nmea_Time time    
Nmea_Pos pos    
uint32  sig            # < GPS quality indicator (0 = Invalid  1 = Fix  2 = Differential, 3 = Sensitive) */
uint32  satinuse       # < Number of satellites in use (not those in view)
float64 HDOP           # < Horizontal dilution of precision
float64 elv            # < Antenna altitude above/below mean sea level (geoid)
int32   elv_units      # < [M]eters (Antenna height unit)
float64 diff           # < Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level. '-' = geoid is below WGS-84 ellipsoid)
int32   diff_units     # < [M]eters (Units of geoidal separation)
float64 dgps_age       # < Time in seconds since last DGPS update
uint32  dgps_sid       # < DGPS station ID number
