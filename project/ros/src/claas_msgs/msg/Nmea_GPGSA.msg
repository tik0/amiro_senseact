Header header

int32    fix_mode       # < Mode (M = Manual, forced to operate in 2D or 3D  A = Automatic, 3D/2D)
uint32   fix_type       # < Type, used for navigation (1 = Fix not available  2   D  3   D)
uint32[] sat_prn        # < PRNs of satellites used in position fix (null for unused fields) */
float64  PDOP           # < Dilution of precision
float64  HDOP           # < Horizontal dilution of precision
float64  VDOP           # < Vertical dilution of precision
