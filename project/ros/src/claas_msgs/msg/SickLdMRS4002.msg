# Single SickLDMRS scan

Header header                   # timestamp and frame

uint32 magicWord                # 0xAFFEC0C2
uint32 sizePrevMsg              # 4 Byte  
uint32 sizeMsg                  # 4 Byte  
uint32 reserved                 # 4 Byte
uint32 deviceId                 # 4 Byte
uint32 dataType                 # 0x2202
uint64 timeMsgSent              # 8 Byte total = 32 Byte, 24 Byte without optional
                                
uint32 measureNumber            # 4 Byte
uint32 sensorStatus             # fixed32
uint32 synchroPhase             # 4 Byte
uint64 timestartMeasure         # 8 Byte  
uint64 timeendMeasure           # 8 Byte
uint32 angularStepsPerRotation  # 4 Byte
int32  startAngle               # 4 Byte
int32  endAngle                 # 4 Byte
uint32 numberMeasuredPoints     # 4 Byte
int32  reserved_3               # 4 Byte
int32  reserved_2               # 4 Byte
int32  reserved_4               # 4 Byte
int32  reserved_1               # 4 Byte
int32  reserved_5               # 4 Byte
int32  reserved_6               # 4 Byte
int32  reserved_7               # 4 Byte total = 72 Byte, 44 Byte without optional


uint32[] layer                  # 1 .. 1000 Points a 8 Byte = 8000 Byte
uint32[] echo                   # 1 .. 1000 Points a 8 Byte = 8000 Byte
uint32[] flags                  # 1 .. 1000 Points a 8 Byte = 8000 Byte
uint32[] horizontalAngle        # 1 .. 1000 Points a 8 Byte = 8000 Byte
uint32[] radialDistance         # 1 .. 1000 Points a 8 Byte = 8000 Byte
uint32[] echoPulseWidth         # 1 .. 1000 Points a 8 Byte = 8000 Byte

uint32[] reserved_8             # Take care of this reserved value 

string uniqueIdentifier         # addtional Values