/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Freescale Semiconductor, Inc.
 */

#include <config.h>

/* DDR script */
.macro eval1a_ddr3_setting
	ldr r0, =IOMUXC_BASE_ADDR
	ldr r1, =0x000C0000
	str r1, [r0, #0x4B4]

	ldr r1, =0x00000000
	str r1, [r0, #0x4AC]

	ldr r1, =0x00000028
	str r1, [r0, #0x27C]
	str r1, [r0, #0x250]
	str r1, [r0, #0x24C]
	str r1, [r0, #0x490]
	str r1, [r0, #0x288]

	ldr r1, =0x00000000
	str r1, [r0, #0x270]

	ldr r1, =0x00000028
	str r1, [r0, #0x260]
	str r1, [r0, #0x264]
	str r1, [r0, #0x4A0]

	ldr r1, =0x00020000
	str r1, [r0, #0x494]

	ldr r1, =0x00000028
	str r1, [r0, #0x280]
	str r1, [r0, #0x284]

	ldr r1, =0x00020000
	str r1, [r0, #0x4B0]

	ldr r1, =0x00000028
	str r1, [r0, #0x498]
	str r1, [r0, #0x4A4]
	str r1, [r0, #0x244]
	str r1, [r0, #0x248]

	ldr r0, =MMDC_P0_BASE_ADDR
	ldr r1, =0x00008000
	str r1, [r0, #0x1C]

	ldr r1, =0xA1390003
	str r1, [r0, #0x800]

	ldr r1, =0x000A0025
	str r1, [r0, #0x80C]

	ldr r1, =0x000B000B
	str r1, [r0, #0x810]

	ldr r1, =0x01580158
	str r1, [r0, #0x83C]

	ldr r1, =0x00000000
	str r1, [r0, #0x840]

	ldr r1, =0x40403436
	str r1, [r0, #0x848]

	ldr r1, =0x4040302C
	str r1, [r0, #0x850]
	
	ldr r1, =0x33333333
	str r1, [r0, #0x81C]
	str r1, [r0, #0x820]
	ldr r1, =0xF3333333
	str r1, [r0, #0x82C]
	str r1, [r0, #0x830]
	
	ldr r1, =0x00944009
	str r1, [r0, #0x8C0]

	ldr r1, =0x00000800
	str r1, [r0, #0x8B8]
	ldr r1, =0x0002002D
	str r1, [r0, #0x004]
	ldr r1, =0x1B333030
	str r1, [r0, #0x008]

	ldr r1, =0x3F4352F3
	str r1, [r0, #0x00C]

	ldr r1, =0xB66D0B63
	str r1, [r0, #0x010]

	ldr r1, =0x01FF00DB
	str r1, [r0, #0x014]

	ldr r1, =0x00211740
	str r1, [r0, #0x018]

	ldr r1, =0x00008000
	str r1, [r0, #0x01C]

	ldr r1, =0x000026D2
	str r1, [r0, #0x02C]

	ldr r1, =0x00431023
	str r1, [r0, #0x030]

	ldr r1, =0x00000047
	str r1, [r0, #0x040]

	ldr r1, =0x83180000
	str r1, [r0, #0x000]

	ldr r1, =0x00400000
	str r1, [r0, #0x890]
	ldr r1, =0x02008032
	str r1, [r0, #0x01C]
	ldr r1, =0x00008033
	str r1, [r0, #0x01C]
	ldr r1, =0x00408031
	str r1, [r0, #0x01C]
	ldr r1, =0x15208030
	str r1, [r0, #0x01C]
	ldr r1, =0x04008040
	str r1, [r0, #0x01C]

	ldr r1, =0x0200803A
	str r1, [r0, #0x01C]

	ldr r1, =0x0000803B
	str r1, [r0, #0x01C]

	ldr r1, =0x00408039
	str r1, [r0, #0x01C]

	ldr r1, =0x15208038
	str r1, [r0, #0x01C]

	ldr r1, =0x04008048
	str r1, [r0, #0x01C]


	ldr r1, =0x00007800
	str r1, [r0, #0x020]
	ldr r1, =0x00000117
	str r1, [r0, #0x818]
	ldr r1, =0x0002552D
	str r1, [r0, #0x004]
	ldr r1, =0x00011006
	str r1, [r0, #0x404]
	ldr r1, =0x00000000
	str r1, [r0, #0x01C]
.endm

.macro imx6_clock_gating
	ldr r0, =CCM_BASE_ADDR
	ldr r1, =0xFFFFFFFF
	str r1, [r0, #0x68]
	str r1, [r0, #0x6C]
	str r1, [r0, #0x70]
	str r1, [r0, #0x74]
	str r1, [r0, #0x78]
	str r1, [r0, #0x7C]
	str r1, [r0, #0x80]
.endm

.macro imx6_qos_setting
.endm

.macro imx6_ddr_setting
	eval1a_ddr3_setting
.endm

/* include the common plugin code here */
#include <asm/arch/mx6_plugin.S>
