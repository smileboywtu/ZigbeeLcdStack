# Wireless LCD Stack

Embeded Sensor network based on cc2530:

+ LCD
+ Camera
+ ADXL345

# Version

- Z-Stack 2.5.3a

- IAR 8.2

# Tools

You need to use the IAR workspace to open this project, this project contain all the ZIGBEE pro stack
You don't need to install the TI's z-stack again. just copy the stack into the disk and use IAR open it.

All thing is easy and all files just prepared for you.
  
# Description

## Content

You may interesting with the content it contains. Here I just give you some information.

Here are the projects:

+ Coordinator Monitor
+ Router Project
+ LCD Project
+ Camera Project
+ Vibrate(ADXL345)

## LCD Project

This project is the main project for the z-stack, this project has won big prize in
the final contest of the Up-Tech Cap.

When you use this project you should at least own two CC2530 device: one is as the 
coordinator and burn the coordinator project binary, the other is burned with this 
project, all two projects contained in the stack.

You may need a LCD device, this device you can buy from the [DWIN](http://www.dwin.com.cn/). If you get an lcd in hand, make sure it use the two wire UART mode.

![Alt text](https://gd3.alicdn.com/bao/uploaded/i3/T1AUuiFFhaXXXXXXXX_!!0-item_pic.jpg "LCD Device")

For other LCDs, you may need to create the z-stack your own.

For the user whole use other LCDs but with the UART control interface, you may need to change the content command. As for content command, you should look at the docs for
your LCD, this is usually specific.

## Camera Project

So when you use this project to get and transfer images, you need a camera, which is
also special ones.Know more information about the camera from [here](https://item.taobao.com/item.htm?spm=a230r.1.14.71.XsMDNL&id=37639069215&ns=1&abbucket=14#detail).

![Alt text](https://img.alicdn.com/imgextra/i3/61516653/T2laWmXTxXXXXXXXXX-61516653.jpg  "UART JPEG Camera")

## Vibrate Project

This project just use the ADXL345 to get the vibrate. know more things about the ADXL345 from [here](http://www.analog.com/en/products/mems/mems-accelerometers/adxl345.html).

Because CC2530 is actually is a 8051 core, so there isn't an IIC interface, so I just use an Arduino UNO as a wire connect ADXL345 and CC2530.

# More

You also can look at the *Embeded-Monitor* Project and this project is the PC front end of this ZIGBEE stack.

# Contact

You may get some bugs or need help, please email me at 294101042@qq.com.
