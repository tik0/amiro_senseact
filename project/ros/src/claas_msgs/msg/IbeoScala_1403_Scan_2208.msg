Header header

string uniqueIdentifier   # Header
uint32 sizePrevMsg           
uint32 sizeMsg           
uint32 reserved       
uint32 scalaDeviceId         
uint32 dataType          
uint64 timeMsgSent 

uint32 scanNumber          # Measured Data
uint32 scannerType      
int32  scannerStatus       
uint32 angleTicksRotation   
uint32 scanFlags    
int32  mountingYaw 
int32  mountingPitch 
int32  mountingRoll 
int32  mounting_X 
int32  mounting_Y 
int32  mounting_Z 
uint32 deviceID 
int32  reserved_1   
uint64 scanStartTime 
uint64 scanEndTime 
int32  startAngle 
int32  subFlags   
uint32 mirrorSide 
uint32 reserved_2   
int32  mirrorTilt 
uint32 numberScanPoints 

uint32[] echo               # List of measured points
uint32[] layer      
uint32[] scanPointFlags      
uint32[] horizontalAngle    
uint32[] radialDistance    
uint32[] echoPulseWidth    
