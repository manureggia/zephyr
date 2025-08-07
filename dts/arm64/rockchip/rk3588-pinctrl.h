#ifndef ZEPHYR_DTS_ARM_ROCKCHIP_RK3588_PINCTRL_H_
#define ZEPHYR_DTS_ARM_ROCKCHIP_RK3588_PINCTRL_H_

#define RK_FUNC_GPIO    0
#define RK_FUNC_1       1
#define RK_FUNC_2       2
#define RK_FUNC_3       3
#define RK_FUNC_4       4
#define RK_PULL_NONE    0
#define RK_PULL_UP      1
#define RK_PULL_DOWN    2
#define RK_DRIVE_2MA    0
#define RK_DRIVE_4MA    1
#define RK_DRIVE_8MA    2
#define RK_DRIVE_12MA   3

#define RK_PINCFG(bank, pin, func, pull, drive)         \
    ((((bank) & 0x7) << 20) | (((pin) & 0x1F) << 16) |  \
     (((drive) & 0xF) << 8) | (((pull) & 0x7) << 4)  |  \
     ((func) & 0x7))

#endif /* ZEPHYR_DTS_ARM_ROCKCHIP_RK3588_PINCTRL_H_ */