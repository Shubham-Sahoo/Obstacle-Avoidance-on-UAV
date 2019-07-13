All the configurations is listed in the document attached.

These parameters should be checked before attempting autonomous indoor flying.

## Parameter    -   Value   -      Description

CBRK_GPSFAIL - 240024 - Allows flight without GPS-data.

CBRK_IOSAFETY- 22027 - Allows flight with I/O connections.

CBRK_USB_CHK - 197848 - Allows flight with USB connected.

SYS_MC_EST_GROUP - Motion Capture - Allows the Pixhawk to recive mocap position.

ATT_EXT_HDG_M - local_position_estimator OR ekf2 - Position estimator, lpe required for offboard control.Only available if SYS_MC_EST_GROUP is set to lpe.

MAV_1_CONFIG - TELEM2 - Sets mavlink to use the TELEM2 port.

MAV_1_MODE - Onboard - Required for mavlink.

MAV_1_RATE - 921600  - Baudrate for mavlink port.


