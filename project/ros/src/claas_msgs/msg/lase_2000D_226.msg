# Single scan from a planar LASE 2000D 226 laser range-finder

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.

uint32 scannumber        # Raw consecutive number
uint32 timestamp         # Raw timestamp by scanner
uint32 systemtemperature # Raw temperature
string uniqueIdentifier  # The IP of the device

uint32[] distance        # Range data [1/10mm]
uint32[] pulsewidth      # duration in [ps]
