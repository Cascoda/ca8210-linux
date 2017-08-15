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
Information about the hardware is specified in the Linux device tree. The device tree file will be platform specific but for any tree a section must be added within the section for the relevant spi bus for the CA-8210. See Docs/devicetree/ca8210.txt for more info.

---
This is a softMAC implementation of a CA-8210 driver for Linux. This means it is using the built-in MAC layer provided by the Linux 802.15.4 stack. The CA-8210 does have its own comprehensive MAC layer built in but the stack support for hardMAC devices is less mature. A hardMAC implementation will ultimately supercede this version.

---
When started, the driver will create the required interfaces for use with the networking stack. For more information please see [linux-wpan](http://wpan.cakelab.org/#_how_to_8217_s). There is also the option of using a debug interface which will bypass the networking stack in order to use the CA-821x API directly. This API is documented in section 5 of the [CA-8210 datasheet](http://www.cascoda.com/wp/wp-content/uploads/CA-8210_datasheet_1016.pdf). Using the driver in this way will allow access to the entirety of the features provided by the 802.15.4 MAC, some of which are not yet supported in the Linux implementation. Cascoda provides a project that can be used as a base for user-space applications that wish to use the CA-8210 in this way called [ca8210-kernel-exchange](https://github.com/Cascoda/ca8210-kernel-exchange).

---
In order to expose the ca8210 debug node the Linux debug file system must be mounted. debugfs can be mounted with the command:
```
mount -t debugfs none /sys/kernel/debug
```
