# ArduPilot-RL  Project

[<img src="http://www.radiolink.com.cn/firmware/wiki/crossflight_readme.png" alt="crossflight" width="350px" />](https://radiolink.com/crossflight) 

The ArduPlot-RL project  is modified from the [ArduPilot project](https://github.com/ArduPilot/ardupilot) to support [HDSC](https://www.xhsc.com.cn/Productlist/list.aspx?lcid=9&cid=9&key=HC32F4A0) 's HC32 series chips. At the same time, the project is specially designed for the Radiolink crossflight,so crossflight basically has the same functions as ardupilot. However, it should be noted that some HC32 driver .c file of the submodule ChibiOS and some cpp file of the "AP_HAL_ChibiOS" folder are both packaged as .a static libraries.

## The static library

- libHAL_ChibiOS_libs.a

  Except for "stdio.cpp" file, all .cpp files in the "libraries/AP_HAL_ChibiOS" directory are packaged as libHAL_ChibiOS_libs.a static library. 

  The "libHAL_ChibiOS_libs.a" file is stored in the  ["libraries/AP_HAL_ChibiOS"](https://github.com/radiolinkW/ArduPilot-RL/tree/Copter-4.3/libraries/AP_HAL_ChibiOS) directory.

- libch.a

  All .c file in the "libraries/AP_HAL_ChibiOS/hwdef/common" and "modules/ChibiOS/os/hal/ports/HC32" directories are packaged as libch.a static library.

  The "libch.a" file is stored in the  ["modules/ChibiOS"](https://github.com/radiolinkW/ChibiOS) directory.

## Building Firmware

Building instructions can refer to : [Building ArduPilot](https://github.com/radiolinkW/ArduPilot-RL/blob/Copter-4.3/BUILD.md) 

The name of hwdef.dat directory for the crossflight board is ["Crossflight"](https://github.com/radiolinkW/ArduPilot-RL/tree/Copter-4.3/libraries/AP_HAL_ChibiOS/hwdef/Crossflight) , as shown in the following figure: ![dat file](http://www.radiolink.com.cn/firmware/wiki/dat_readme.png)

Therefore, the Command to configure Crossflight board is as follows:

./waf configure --board Crossflight --bootloader