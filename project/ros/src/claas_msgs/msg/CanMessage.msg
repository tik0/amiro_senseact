# Single message from CAN

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.

uint32  canId            # Raw CAN identifier
uint8[] canMessage       # Raw CAN message
uint32  canDlc           # Amount of payload in bytes
