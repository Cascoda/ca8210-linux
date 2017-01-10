# ca8210-linux
Linux kernel driver for Cascoda's CA-8210 IEEE 802.15.4 transceiver.

The master branch supports kernel 4.8. The backports branch can be built for kernel 4.1 onwards.

Building:
The Linux kbuild system must be used to build the driver as a kernel module. From [kernel.org](https://www.kernel.org/doc/Documentation/kbuild/modules.txt):

*The command to build an external module is:*
```
$ make -C <path_to_kernel_src> M=$PWD
```
*The kbuild system knows that an external module is being built due to the "M=\<dir\>" option given in the command.*

*To build against the running kernel use:*
```
$ make -C /lib/modules/`uname -r`/build M=$PWD
```
*Then to install the module(s) just built, add the target "modules_install" to the command:*
```
$ make -C /lib/modules/`uname -r`/build M=$PWD modules_install
```
To enable the debugfs interface you must add the CONFIG_IEEE802154_CA8210_DEBUGFS flag to the module's CFLAGS when building the module, i.e.
```
$ make CFLAGS_ca8210.o:=-DCONFIG_IEEE802154_CA8210_DEBUGFS=1 -C /lib/modules/`uname -r`/build M=$PWD
```
The built module can be found at /lib/modules/\`uname -r\`/extra/**ca8210sm** and can be loaded with the command  
```
sudo modprobe ca8210sm
```

---
Information about the hardware is specified in the Linux device tree. The device tree file will be platform specific but for any tree a section must be added within the section for the relevant spi bus for the ca8210. See Docs/devicetree/ca8210.txt for more info.

---
In order to expose the ca8210 debug node the Linux debug file system must be mounted. debugfs can be mounted with the command:
```
mount -t debugfs none /sys/kernel/debug
```
