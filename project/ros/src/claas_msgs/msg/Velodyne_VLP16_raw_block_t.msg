uint32 header                                  # < Block id: UPPER_BANK or LOWER_BANK
uint32 rotation                                # < 0-35999, divide by 100 to get degrees
Velodyne_VLP16_laser_return_t[] laser_returns  # laser_return_t  laser_returns[SCANS_PER_BLOCK] 32;