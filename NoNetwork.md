# ROS2 Communication on Windows 10 IoT Core without a Network

To enable Loopback for UDP DDS on Windows 10 IoT Core when there is no network available (no Ethernet connected and no Wifi configured), by default, ROS2 nodes will not be able to communicate with eachother or themselves.  

To work around this, we can leverage the Microsoft Loopback Adapter and a USB Wifi dongle.  

## Get Microsoft Loopback Adapter

First you'll need to install the Microsoft Loopback Adapter on a desktop computer using these steps (from Windows 10):

1. Right click on Windows Icon in bottom left corner and select Device Manager

1. Select Network Adapters node

1. Select Action > Add legacy hardware to invoke the Add Hardware Wizard

1. Click Next to continue

1. Select *Install the hardware that I manually select from a list (Advanced)* option and click Next

1. Select Network adapters and click Next

1. Wait for list to populate.  Select `Microsoft` as a Manufacturer and `Microsft KM-TEST Loopback Adapter` as a Model.  Click Next.

1. Click Next to install

The files you'll need should be found in C:\WINDOWS\system32\DriverStore\FileRepository\netloop.inf_amd64_9af4c1272e95e55d.

## Configure Windows 10 IoT Core

1. Copy loop.sys and netloop.inf to your Window 10 IoT Core device (for these instructions, we'll assume you've copied them into C:\data\loopback)

1. Run these commands to install and enable Microsoft Loopback Adapter:

     ```
     devcon -r install \data\loop\netloop.inf msloop
     devcon enable @ROOT\DEVCON\0000
     ```
1. Restart your device