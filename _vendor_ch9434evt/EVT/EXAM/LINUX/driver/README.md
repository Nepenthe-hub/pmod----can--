ch9434 SPI/I2C Linux驱动程序
===========================
此资料包包含以下部分：Linux端SPI/I2C设备驱动程序、GPIO应用库与演示程序。

此资料包适用于如下芯片的接口扩展功能：

CH9434A/M：SPI转UART/GPIO

CH9434D：SPI/I2C转UART/CAN/GPIO

## 注意事项

对于**CH9434D**芯片，支持SPI或I2C作为控制接口、此外因引脚之间存在功能复用，可根据芯片应用方式在ch943x.h文件中修改适配。

```c
CH9434D SPI或I2C接口选择
如果选择SPI作为控制接口，修改如下：
#define USE_SPI_MODE // SPI接口
如果选择I2C作为控制接口，修改如下：
#define USE_I2C_MODE // I2C接口

注意：只能定义其中一个宏！
```

```c
CH9434D 时钟选择
如使用外部时钟，修改如下：
#define EXTERN_CLOCK // 外部时钟
如使用内部时钟，修改如下：
#define INTERNAL_CLOCK // 内部时钟

注意：只能定义其中一个宏！
```

```c
CH9434D TNOW0~3功能使能，对应UART0~3的RS485自动方向控制脚使能
如配置UART0~3为RS485串口，修改如下：
/******** CH9434D TNOW function enable *******/
#define CH9434D_TNOW0_ON
#define CH9434D_TNOW1_ON
#define CH9434D_TNOW2_ON
#define CH9434D_TNOW3_ON
    
注意：无需使能为TNOW功能的引脚，可注释如上宏定义，或启用undef，如：#undef CH9434D_TNOW0_ON
```

```c
CH9434D CAN功能引脚使能
如使用CAN功能，定义如下宏：
/******** CH9434D CAN function switch *******/
#define CH9434D_CAN_ON

注意：如果不使用CAN功能，或CAN_TX/CAN_RX引脚用作其他功能，可注释如上宏定义，或启用undef，或改为 #undef CH9434D_CAN_ON
```

对于**CH9434A/M**，TNOW0~3的RS485自动方向控制脚使能，可做如下修改：

```c
/******** CH9434A/M TNOW function enable *******/
#define CH943X_TNOW0_ON
#define CH943X_TNOW1_ON
#define CH943X_TNOW2_ON
#define CH943X_TNOW3_ON

注意：无需使能为TNOW功能的引脚，可注释如上宏定义，或启用undef，如：#undef CH943X_TNOW0_ON
```

驱动移植方法
---------------------------------------

编译驱动程序，Makefile示例：

```
KERNEL_DIR=xxx (Linux内核绝对路径)
ARCH=xxx
CROSS_COMPILE=xxx (指定交叉编译工具链)
export  ARCH  CROSS_COMPILE
DRIVERNAME := ch943x_module
obj-m := $(DRIVERNAME).o
$(DRIVERNAME)-y := ch943x.o ch943x_ctrl.o ch943x_gpio.o ch943x_serial.o ch943x_can.o
all:
	$(MAKE) EXTRA_CFLAGS=-fno-pic -C $(KERNEL_DIR) M=$(CURDIR) modules
.PHONY:clean
clean:
	$(MAKE)  -C $(KERNEL_DIR) M=$(CURDIR) clean
```

编译后出现ch943x_module.ko文件。

SPI接口dts示例：

```
&spiX{
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <xxx>;
    pinctrl-1 = <xxx>;
    cs-gpios = <xxx>;

    ch943x@0 {
        compatible = "wch,ch943x";
        reg = <0>;
        spi-max-frequency = <2000000>; 
        interrupt-parent = <&gpiox>;
        interrupts = <xxx IRQ_TYPE_LEVEL_LOW>;
    };
};
```

I2C接口dts示例：

```
&i2c0{
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <xxx>;
    #address-cells = <1>;
    #size-cells = <0>;
    clock-frequency = <1000000>;

    ch943x@xx {
        compatible = "wch,ch943x";
        reg = <xx>;
        status = "okay";
    };
};
```

执行insmod ch943x_module.ko即可加载驱动程序。

此处仅展示驱动程序动态编译方式，静态编译可参考各SoC厂商SDK相关说明。



如有任何问题，您可以将反馈信息发送至邮箱: tech@wch.cn
