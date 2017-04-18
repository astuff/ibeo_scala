Header header
time scan_start
time scan_end
bool fused_scan
uint8 mirror_side
uint8 coordinate_system
uint16 scan_number
ScannerInfo[] scanner_info_list
ScanPoint2205[] scan_point_list

# mirror_side values
uint8 FRONT=0
uint8 REAR=1

# coordinate_system values
uint8 SCANNER=0
uint8 VEHICLE=1
