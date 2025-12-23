# Nvidia-Jetson-Orin-Nano
## **Install NoMachine on NVIDIA Jetson (ARM64)**
This guide demonstrates how to download and install NoMachine on an NVIDIA Jetson device and configure a dummy display to enable NoMachine to function without a physical monitor.<br>
Prerequisites:<br>
NVIDIA Jetson device (ARM64)<br>
Ubuntu-based JetPack<br>
Internet connection<br>

**1. Download NoMachine (ARM64)**<br>
Download the ARM64 .deb package:<br>
```
wget https://download.nomachine.com/download/9.3/Arm/nomachine_9.3.7_1_arm64.deb
```

**2. Install NoMachine**<br>
Install the downloaded package:
```
sudo dpkg -i nomachine_9.3.7_1_arm64.deb
```
If you see dependency errors, fix them with:
```
sudo apt --fix-broken install
```

**3. Install Dummy Display (Headless Setup)**<br>
If your Jetson runs without a monitor, you need a dummy X11 display.<br>
Open the Xorg config file:
```
sudo nano /etc/X11/xorg.conf
```
Paste the following content:
```
Section "Device"
    Identifier "Configured Video Device"
    Driver "dummy"
EndSection

Section "Monitor"
    Identifier "Configured Monitor"
    HorizSync 31.5-48.5
    VertRefresh 50-70
EndSection

Section "Screen"
    Identifier "Default Screen"
    Monitor "Configured Monitor"
    Device "Configured Video Device"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1280x720"
    EndSubSection
EndSection
```

**4. Reboot the System**
```
sudo reboot
```

