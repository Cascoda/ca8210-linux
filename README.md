# ca8210-linux
Linux kernel drivers for Cascoda's CA-8210 IEEE 802.15.4 transceiver

Building:
The Linux kbuild system must be used to build the driver as a kernel module. From [kernel.org](https://www.kernel.org/doc/Documentation/kbuild/modules.txt):

    The command to build an external module is:

	  $ make -C <path_to_kernel_src> M=$PWD

	The kbuild system knows that an external module is being built
	due to the "M=<dir>" option given in the command.

	To build against the running kernel use:

	  $ make -C /lib/modules/`uname -r`/build M=$PWD

	Then to install the module(s) just built, add the target
	"modules_install" to the command:

	  $ make -C /lib/modules/`uname -r`/build M=$PWD modules_install
	  
Information about the hardware is specified in the Linux device tree. The device tree file will be platform specific but for any tree a section must be added within the section for the relevant spi bus for the ca8210 similar to the following:

ca8210@**_chip select number_** {  
&nbsp;&nbsp;&nbsp;&nbsp;compatible = "cascoda,ca8210";
&nbsp;&nbsp;&nbsp;&nbsp;reg = <0>;  
&nbsp;&nbsp;&nbsp;&nbsp;spi-max-frequency = <**_max of 4000000_**>;  
&nbsp;&nbsp;&nbsp;&nbsp;spi-cpol;  
&nbsp;&nbsp;&nbsp;&nbsp;reset-gpio = <&**_gpio bank and number_** GPIO_ACTIVE_HIGH>;  
&nbsp;&nbsp;&nbsp;&nbsp;irq-gpio = <&**_gpio bank and number_** GPIO_ACTIVE_HIGH>;  
&nbsp;&nbsp;&nbsp;&nbsp;extclock-enable **(if external clock provided by ca8210 is desired)**  
&nbsp;&nbsp;&nbsp;&nbsp;extclock-freq = <**_frequency as specified in ca8210 datasheet_**>;  
&nbsp;&nbsp;&nbsp;&nbsp;extclock-gpio = <**_output gpio as specified in ca8210 datasheet_**>;  
};
