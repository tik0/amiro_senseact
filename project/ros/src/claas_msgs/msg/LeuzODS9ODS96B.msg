# Single scan of the LEUZ 1D scanner

Header header            # timestamp and frame

uint32 distance_mm           # Distance in mm
bool   distance_in_range     # True, if distance is valid
bool   error_signal_too_week # True, if signal is too weak
bool   device_error_signal   # True, if device has error
bool   device_error_laser    # True, if laser has encountered an error
bool   distance_out_of_range # True, if measurement is out of range
