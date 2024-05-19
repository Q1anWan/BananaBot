#ifndef DL_H723_H
#define DL_H723_H
#ifdef __cplusplus
extern "C" {
#endif
/*
	使用自定分散加载文件时
	不指定: 使用普通RAM
	SRAM_SET_DTCM: 默认 保存进DTCM
    SRAM_SET_RAM_D: Stored in DRAM, without flash, no init value!!
*/
#define SRAM_SET_DTCM    __attribute__((section(".RAM_DTCM")))
#define SRAM_SET_RAM_D1  __attribute__((section(".RAM_D1")))
#define SRAM_SET_RAM_D2  __attribute__((section(".RAM_D2")))
#define SRAM_SET_RAM_D3  __attribute__((section(".RAM_D3")))

#ifdef __cplusplus
}
#endif
#endif