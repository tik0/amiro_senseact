# Baumer radar scan

Header header               # timestamp and frame

int32   timestamp           # System time at start of long range measurement [ms] Cast to unsigned 32 bit integer.
int32   temperature         # Internal temperature [Celsius]
int32   distance            # measured in [um]
int32   gain_I              # Gain stage for I data 0,1,2 or 3. Cast to ANSI/IEEE 754-1985 single precision float.
int32   gain_Q              # Gain stage for Q data 0,1,2 or 3. Cast to ANSI/IEEE 754-1985 single precision float.
int32[] fft_Data            # FFT bin size = 180.86 mm. Cast to ANSI/IEEE 754-1985 single precision float.
int32[] distances           # List of up to 10 detected distances [um]
int32   num_valid_distances # Number of valid values in distance list
int32[] I_Data              # Raw sample data imaginary part. Cast to ANSI/IEEE 754-1985 single precision float.
int32[] Q_Data              # Raw sample data real part. Cast to ANSI/IEEE 754-1985 single precision float.
string  uniqueIdentifier
