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

## Building bootloader

Building instructions can refer to : [Building bootloader](https://github.com/radiolinkW/ArduPilot-RL/blob/Bootloader/Tools/AP_Bootloader/README.md) 

The name of hwdef.dat directory for the crossflight board is ["Crossflight"](https://github.com/radiolinkW/ArduPilot-RL/tree/Copter-4.3/libraries/AP_HAL_ChibiOS/hwdef/Crossflight) , as shown in the following figure: ![dat file](http://www.radiolink.com.cn/firmware/wiki/dat_readme.png)

Therefore, the Command to configure Crossflight board is as follows:

./waf configure --board Crossflight --bootloader

**Note:**

When updating the submodule, please confirm that the "pyuavcan" submodule has been successfully cloned, otherwise a compilation bootloader error will occur.As shown in the following figure:
![pyuavcan](http://www.radiolink.com.cn/firmware/wiki/readme_bl/pyuavcan.png)

An empty pyuavcan folder indicates that the cloning of the submodule was unsuccessful,so we need to manually update and clone the module. First, enter the libcanard directory, as shown in the following figure:
![libcanard](http://www.radiolink.com.cn/firmware/wiki/readme_bl/libcannard.png)

enter "git reset --hard HEAD" to update the libcanard module workspace, and enter "git submodule update --init --recursive " to update the pyuavcan submodule. When all files appear in the pyuavcan directory, as shown in the following figure:

 ![pyuavcany](http://www.radiolink.com.cn/firmware/wiki/readme_bl/pyuavcany.png)

this indicates that the pyuavcan submodule is been successfully updated,and at this point, the bootloader can be compiled.

Meanwhile, if compiling the bootloader also prompts other errors, please continue to update the submodules.

## Upload bootloader

Due to the lack of dfu mode on the main chip HC32 of the crossflight, the bootloader cannot be uploaded through USB. At this point, it is necessary to upload the bootloader through a serial tool such as "CH340".

### Connection

 <img src="http://www.radiolink.com.cn/firmware/wiki/readme_bl/Boot1.png" alt="boot1" width="300px" />
The above is the line sequence diagram of the upload port.
 <img src="http://www.radiolink.com.cn/firmware/wiki/readme_bl/MCU.png" alt="connect" width="400px" />  

 ![connect2](http://www.radiolink.com.cn/firmware/wiki/readme_bl/connect.png)

Connect the wires as shown in the above figure.

### Download APP

Click on [XHSC](https://www.radiolink.com.cn/firmware/wiki/XHSC_ISP.zip), download the "XHSC ISP" app.

### Upload

- enter boot mode

  Before powering on the crossflight, connect the boo0 pin to 3.3v,

   <img src="http://www.radiolink.com.cn/firmware/wiki/readme_bl/boot0.png" alt="boot0" width="400px" />

   At this time, power on the crossflight and it will enter boot mode.

- upload bootloader

Open the "XHSC" app

 <img src="http://www.radiolink.com.cn/firmware/wiki/readme_bl/XHSC.png" alt="XHSC" width="600px" />

1.Select "HC32F4A0" as the target MCU.

2.Select the compiled "AP_Bootloader.bin" file.

3.Select the COM port for connecting CH340 to the computer.

4.Select "Chip Erase".

5.Tick these three options.

6.Finally upload the bootloader to the crossflight board.