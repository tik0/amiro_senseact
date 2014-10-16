#!/bin/bash
./sense_ir -o /ir
./trigger_cleanup --inscopeIRData /ir --remoteSocketServer 129.70.141.228 --outscopeRemoteCleanup /tabletop/cleanup
