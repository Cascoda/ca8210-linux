# ca8210-linux
Linux kernel drivers for Cascoda's CA-8210 IEEE 802.15.4 transceiver.

The master branch supports kernel 4.4.

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
The built module can be found at /lib/modules/\`uname -r\`/extra/**ca8210sm** and can be loaded with the command  
```
sudo modprobe ca8210sm
```

---
Information about the hardware is specified in the Linux device tree. The device tree file will be platform specific but for any tree a section must be added within the section for the relevant spi bus for the ca8210 similar to the following:

```
ca8210@0 {  /* chip select number */  
	compatible = "cascoda,ca8210";  
	reg = <0>;  
	spi-max-frequency = <3000000>;                  /* max of 4000000 */  
	spi-cpol;                                       /* clock polarity 1, clock phase 0 */  
	reset-gpio = <&gpio1 1 GPIO_ACTIVE_HIGH>;       /* GPIOs dependent on specific hardware */  
	irq-gpio = <&gpio1 2 GPIO_ACTIVE_HIGH>;
	extclock-enable;                                /* if external clock provided by ca8210 is desired*/  
	extclock-freq = 16000000;                       /* frequency as specified in ca8210 datasheet */  
	extclock-gpio = 2;                              /* output gpio as specified in ca8210 datasheet */  
};
```
