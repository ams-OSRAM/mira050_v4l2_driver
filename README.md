# ams-mira050 Linux driver

## Installation

### Driver

Compile the driver using the Makefile

```
make
```

Place it in the kernel modules folder.

```
sudo cp mira050.ko /lib/modules/$(uname -r)/updates
```

Resolve modules dependencies.

```
sudo depmod -a
```

### Device tree

Compile the device tree overlay matching your platform and plugin board from the `dts` folder.

```
cd dts/rpi
sudo dtc mira050-overlay.dts -o /boot/firmware/overlays/mira050.dtbo
```

Set the device tree overlay in your platform. This may differ from platform to platform. Please refer to your platform documentation.

This is how to set the device tree overlay for Raspberry Pi.

echo "dtoverlay=mira050" | sudo tee -a /boot/firmware/config.txt
```
alternatively, add the line `dtoverlay=mira050` at the end of the file.

There is a device tree argument for a __monochrome image sensor__.

Use: `dtoverlay=mira050,mono` instead.

Finally, run `sudo reboot` to test your changes.

