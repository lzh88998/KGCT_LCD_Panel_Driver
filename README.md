# KGCT LCD Panel Driver



## Component Description

- panel-novatek-nt35516.c

Raspberry Pi OS driver. Need to add to Raspberry Pi Linux kernel re-build and update the kernel to take effect. Please refer to this [link](https://www.raspberrypi.com/documentation/computers/linux_kernel.html) for details.

- vc4-kms-dsi-nt35516-overlay.dts

Raspberry Pi [device tree overlay component](https://www.raspberrypi.com/documentation/computers/configuration.html#part2). Need to compile to dbto object and copy to firmware location. Building Raspberry Pi kernel will deal with this process automatically.

## Add files to linux kernel

- Copy the .c file to linux kernel path drivers/gpu/drm/panel/

- Modify the Makefile and Kconfig in that path adding the following lines to Makefile

```
obj-$(CONFIG_DRM_PANEL_NOVATEK_NT35516) += panel-novatek-nt35516.o
```

- Adding the following lines to Kconfig

```
config DRM_PANEL_NOVATEK_NT35516
        tristate "Novatek NT35516 RGB panel driver"
        depends on OF
        depends on DRM_MIPI_DSI
        depends on BACKLIGHT_CLASS_DEVICE
        help
          Say Y here if you want to enable support for the panels built
          around the Novatek NT35516 display controller
```

- Copy the .dts file to linux kernel path arch/arm/boot/dts/overlays/

- Modify the Makefile in that path adding the following lines to Makefile dtbo part

```
vc4-kms-dsi-nt35516.dtbo
```
