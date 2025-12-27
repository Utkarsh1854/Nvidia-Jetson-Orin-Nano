## Connecting Raspberry Pi Camera to Jetson
**1. Software Configuration**<br>
NVIDIA’s Jetpack doesn't always auto-detect CSI cameras because the pins can be multiplexed for different functions. You need to tell the kernel what is plugged in:
```
sudo /opt/nvidia/jetson-io/jetson-io.py
```
- Select Configure Jetson 24-pin expansion header (or the CSI connector option, depending on your JetPack version).
- Select Configure for compatible hardware.
- Select the Camera IMX219/IMX477 (this is the sensor in the RPi camera).
- Save and reboot. If you don't reboot, the driver won't load.

**2. Testing the Stream**<br>
- We are going to use GStreamer. It is one of the efficient ways to handle CSI data on Jetson because it uses the hardware ISP (Image Signal Processor):
```
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
'video/x-raw(memory:NVMM), width=3840, height=2160, framerate=30/1' ! \
nvvidconv ! \
'video/x-raw(memory:NVMM), width=1920, height=1080' ! \
nvoverlaysink
```
- The "Headless" Command (If you have no monitor)<br>
If you are just testing if the camera works via SSH and want to see the framerate output in your terminal (without seeing the actual video).
```
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
'video/x-raw(memory:NVMM), width=3840, height=2160, framerate=30/1' ! \
nvvidconv ! \
'video/x-raw(memory:NVMM), width=1920, height=1080' ! \
fpsdisplaysink video-sink=fakesink text-overlay=false
```
- Run this command to see what the system thinks is connected:
```
v4l2-ctl --list-devices
```
- If a previous camera attempt crashed, the argus daemon might be "holding" the camera resource, preventing a new "FrameConsumer" from being created. Reset the daemon:
```
sudo systemctl restart argus-daemon
```
