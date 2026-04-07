/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * This is a pinmux/gpio controller for the X1E.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/gpio.h>
#include <sys/bitstring.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <dev/gpio/gpiobusvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_pinctrl.h>

#include "qcom_tlmm_var.h"
#include "qcom_tlmm_pin.h"
#include "qcom_tlmm_debug.h"

#include "qcom_tlmm_x1e_reg.h"
#include "qcom_tlmm_x1e_hw.h"

#include "gpio_if.h"

#define	DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | \
	    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)

/* gpio 0 .. 237 - normal GPIO pins */
/* gpio 238 - UFS_RESET */
/* gpio 239 .. 241 - SDC2 */

#define QCOM_TLMM_X1E80100_GPIO_PINS     238

static const struct qcom_tlmm_gpio_mux x1e80100_muxes[] = {
	GDEF(0, "qup0_se0", "ibi_i3c", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(1, "qup0_se0", "ibi_i3c", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(2, "qup0_se0", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(3, "qup0_se0", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(4, "qup0_se1", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(5, "qup0_se1", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(6, "qup0_se1", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(7, "qup0_se1", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(8, "qup0_se2", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(9, "qup0_se2", NULL, "atest_usb", "ddr_pxi0", NULL, NULL, NULL, NULL, NULL),
	GDEF(10, "qup0_se2", NULL, "atest_usb", "ddr_pxi1", NULL, NULL, NULL, NULL, NULL),
	GDEF(11, "qup0_se2", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(12, "qup0_se3", "qup0_se7", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(13, "qup0_se3", "qup0_se7", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(14, "qup0_se3", "qup0_se7", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(15, "qup0_se3", "qup0_se7", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(16, "qup0_se4", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(17, "qup0_se4", "qup0_se2", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(18, "qup0_se4", "qup0_se2", "phase_flag", NULL, "qdss_cti", NULL, NULL, NULL, NULL),
	GDEF(19, "qup0_se4", "qup0_se2", "phase_flag", NULL, "qdss_cti", NULL, NULL, NULL, NULL),
	GDEF(20, "qup0_se5", NULL, "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(21, "qup0_se5", "qup0_se3", NULL, "phase_flag", NULL, NULL, NULL, NULL, NULL),
	GDEF(22, "qup0_se5", "qup0_se3", NULL, "phase_flag", NULL, NULL, NULL, NULL, NULL),
	GDEF(23, "qup0_se5", "qup0_se3", "phase_flag", NULL, "qdss_cti", NULL, NULL, NULL, NULL),
	GDEF(24, "qup0_se6", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(25, "qup0_se6", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(26, "qup0_se6", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(27, "qup0_se6", "phase_flag", NULL, "qdss_cti", NULL, NULL, NULL, NULL, NULL),
	GDEF(28, "pll_bist", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(29, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(30, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(31, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(32, "qup1_se0", "ibi_i3c", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(33, "qup1_se0", "ibi_i3c", "qup1_se3", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(34, "qup1_se0", "qup1_se3", "tsense_pwm1", "tsense_pwm2", "tsense_pwm3", "tsense_pwm4", NULL, NULL, NULL),
	GDEF(35, "qup1_se0", "qup1_se3", "pll_clk", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(36, "qup1_se1", "ibi_i3c", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(37, "qup1_se1", "ibi_i3c", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(38, "qup1_se1", "vsense_trigger", "atest_usb", "ddr_pxi0", NULL, NULL, NULL, NULL, NULL),
	GDEF(39, "qup1_se1", "sys_throttle", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(40, "qup1_se2", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(41, "qup1_se2", "atest_usb", "ddr_pxi1", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(42, "qup1_se2", "jitter_bist", "atest_usb", "ddr_pxi2", NULL, NULL, NULL, NULL, NULL),
	GDEF(43, "qup1_se2", NULL, "atest_usb", "ddr_pxi2", NULL, NULL, NULL, NULL, NULL),
	GDEF(44, "qup1_se3", NULL, "atest_usb", "ddr_pxi3", NULL, NULL, NULL, NULL, NULL),
	GDEF(45, "qup1_se3", "cmu_rng3", NULL, "atest_usb", "ddr_pxi3", NULL, NULL, NULL, NULL),
	GDEF(46, "qup1_se3", "cmu_rng2", NULL, "atest_usb", "ddr_pxi4", NULL, NULL, NULL, NULL),
	GDEF(47, "qup1_se3", "cmu_rng1", NULL, "atest_usb", "ddr_pxi4", NULL, NULL, NULL, NULL),
	GDEF(48, "qup1_se4", "cmu_rng0", NULL, "atest_usb", "ddr_pxi5", NULL, NULL, NULL, NULL),
	GDEF(49, "qup1_se4", "qup1_se2", NULL, "atest_usb", "ddr_pxi5", NULL, NULL, NULL, NULL),
	GDEF(50, "qup1_se4", "qup1_se2", NULL, "atest_usb", "ddr_pxi6", NULL, NULL, NULL, NULL),
	GDEF(51, "qup1_se4", "qup1_se2", "dbg_out", "atest_usb", "ddr_pxi6", NULL, NULL, NULL, NULL),
	GDEF(52, "qup1_se5", "qup1_se7", "atest_usb", "ddr_pxi7", NULL, NULL, NULL, NULL, NULL),
	GDEF(53, "qup1_se5", "qup1_se7", NULL, "atest_usb", "ddr_pxi7", NULL, NULL, NULL, NULL),
	GDEF(54, "qup1_se5", "qup1_se7", "ddr_bist", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(55, "qup1_se5", "qup1_se7", "ddr_bist", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(56, "qup1_se6", "ddr_bist", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(57, "qup1_se6", "ddr_bist", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(58, "qup1_se6", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(59, "qup1_se6", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(60, "aoss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(61, "aoss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(62, "aoss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(63, "aoss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(64, "qup2_se0", "gcc_gp2", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(65, "qup2_se0", "qup2_se3", "tgu_ch1", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(66, "qup2_se0", "qup2_se3", "tgu_ch2", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(67, "qup2_se0", "qup2_se3", "tgu_ch3", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(68, "qup2_se1", "ibi_i3c", "tgu_ch4", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(69, "qup2_se1", "ibi_i3c", "tgu_ch5", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(70, "qup2_se1", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(71, "qup2_se1", "gcc_gp1", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(72, "qup2_se2", "gcc_gp1", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(73, "qup2_se2", "gcc_gp2", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(74, "qup2_se2", "gcc_gp3", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(75, "qup2_se2", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(76, "qup2_se3", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(77, "qup2_se3", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(78, "qup2_se3", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(79, "qup2_se3", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(80, "qup2_se4", "tgu_ch7", "atest_usb", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(81, "qup2_se4", "qup2_se2", "tgu_ch0", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(82, "qup2_se4", "qup2_se2", "gcc_gp3", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(83, "qup2_se4", "qup2_se2", "tgu_ch6", "atest_usb", NULL, NULL, NULL, NULL, NULL),
	GDEF(84, "qup2_se5", "qup2_se7", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(85, "qup2_se5", "qup2_se7", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(86, "qup2_se5", "qup2_se7", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(87, "qup2_se5", "qup2_se7", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(88, "qup2_se6", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(89, "qup2_se6", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(90, "qup2_se6", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(91, "qup2_se6", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(92, "tmess_prng0", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(93, "tmess_prng1", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(94, "sys_throttle", "tmess_prng2", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(95, "tmess_prng3", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(96, "cam_mclk", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(97, "cam_mclk", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(98, "cam_mclk", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(99, "cam_mclk", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(100, "cam_aon", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(101, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(102, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(103, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(104, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(105, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(106, "cci_i2c", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(107, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(108, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(109, "cci_timer0", "mdp_vsync4", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(110, "cci_timer1", "mdp_vsync5", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(111, "cci_timer2", "cci_async", "mdp_vsync6", "qdss_gpio", NULL, NULL, NULL, NULL, NULL),
	GDEF(112, "cci_timer3", "cci_async", "mdp_vsync7", "qdss_gpio", NULL, NULL, NULL, NULL, NULL),
	GDEF(113, "cci_timer4", "cci_async", "mdp_vsync8", "qdss_gpio", NULL, NULL, NULL, NULL, NULL),
	GDEF(114, "mdp_vsync0", "mdp_vsync1", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(115, "mdp_vsync3", "mdp_vsync2", "edp1_lcd", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(116, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(117, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(118, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(119, "edp0_hot", "edp1_lcd", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(120, "edp1_hot", "edp0_lcd", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(121, "usb0_phy", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(122, "usb0_dp", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(123, "usb1_phy", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(124, "usb1_dp", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(125, "usb2_phy", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(126, "usb2_dp", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(127, "qspi0_clk", "sdc4_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(128, "qspi00", "sdc4_data0", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(129, "qspi01", "sdc4_data1", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(130, "qspi02", "sdc4_data2", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(131, "qspi03", "sdc4_data3", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(132, "qspi0_cs0", "sdc4_cmd", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(133, "qspi0_cs1", "tb_trig", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(134, "audio_ext", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(135, "i2s0_sck", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(136, "i2s0_data0", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(137, "i2s0_data1", "tb_trig", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(138, "i2s0_ws", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(139, "i2s1_sck", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(140, "i2s1_data0", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(141, "i2s1_ws", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(142, "i2s1_data1", "audio_ext", "audio_ref", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(143, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(144, "pcie3_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(145, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(146, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(147, "pcie4_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(148, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(149, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(150, "pcie5_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(151, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(152, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(153, "pcie6a_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(154, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(155, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(156, "pcie6b_clk", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(157, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(158, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(159, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(160, "RESOUT_GPIO", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(161, "qdss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(162, "sd_write", "qdss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(163, "usb0_sbrx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(164, "usb0_sbtx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(165, "usb0_sbtx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(166, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(167, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(168, "eusb0_ac", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(169, "eusb3_ac", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(170, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(171, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(172, "usb1_sbrx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(173, "usb1_sbtx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(174, "usb1_sbtx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(175, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(176, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(177, "eusb1_ac", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(178, "eusb6_ac", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(179, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(180, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(181, "usb2_sbrx", "prng_rosc3", "phase_flag", NULL, "atest_char", NULL, NULL, NULL, NULL),
	GDEF(182, "usb2_sbtx", "prng_rosc2", "phase_flag", NULL, "atest_char3", NULL, NULL, NULL, NULL),
	GDEF(183, "usb2_sbtx", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(184, "phase_flag", NULL, "atest_char1", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(185, "phase_flag", NULL, "atest_char0", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(186, "eusb2_ac", "prng_rosc0", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(187, "eusb5_ac", "cri_trng", "phase_flag", NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(188, "prng_rosc1", "phase_flag", NULL, "atest_char2", NULL, NULL, NULL, NULL, NULL),
	GDEF(189, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(190, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(191, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(192, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(193, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(194, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(195, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(196, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(197, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(198, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(199, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(200, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(201, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(202, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(203, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(204, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(205, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(206, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(207, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(208, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(209, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(210, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(211, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(212, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(213, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(214, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(215, NULL, "qdss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(216, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(217, NULL, "qdss_cti", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(218, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(219, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(220, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(221, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(222, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(223, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(224, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(225, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(226, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(227, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(228, NULL, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(229, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(230, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(231, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(232, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(233, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(234, "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(235, "aon_cci", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(236, "aon_cci", "qdss_gpio", NULL, NULL, NULL, NULL, NULL, NULL, NULL),
	GDEF(237, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
#if 0
	GDEF(238] = UFS_RESET(ufs_reset, 0xf9000),
	GDEF(239] = SDC_QDSD_PINGROUP(sdc2_clk, 0xf2000, 14, 6),
	GDEF(240] = SDC_QDSD_PINGROUP(sdc2_cmd, 0xf2000, 11, 3),
	GDEF(241] = SDC_QDSD_PINGROUP(sdc2_data, 0xf2000, 9, 0),
#endif
	GDEF(-1),
};

static const uint32_t qcom_tlmm_x1e_reserved_pins[] = {
	44, 45, 46, 47,
	72, 73, 74, 75,
};

static struct qcom_tlmm_hw_callbacks qcom_tlmm_x1e_hw_callbacks = {
	.qcom_tlmm_hw_pin_set_function = qcom_tlmm_x1e_hw_pin_set_function,
	.qcom_tlmm_hw_pin_get_function = qcom_tlmm_x1e_hw_pin_get_function,
	.qcom_tlmm_hw_pin_set_oe_output = qcom_tlmm_x1e_hw_pin_set_oe_output,
	.qcom_tlmm_hw_pin_set_oe_input = qcom_tlmm_x1e_hw_pin_set_oe_input,
	.qcom_tlmm_hw_pin_get_oe_state = qcom_tlmm_x1e_hw_pin_get_oe_state,
	.qcom_tlmm_hw_pin_set_output_value = qcom_tlmm_x1e_hw_pin_set_output_value,
	.qcom_tlmm_hw_pin_get_output_value = qcom_tlmm_x1e_hw_pin_get_output_value,
	.qcom_tlmm_hw_pin_get_input_value = qcom_tlmm_x1e_hw_pin_get_input_value,
	.qcom_tlmm_hw_pin_toggle_output_value = qcom_tlmm_x1e_hw_pin_toggle_output_value,
	.qcom_tlmm_hw_pin_set_pupd_config = qcom_tlmm_x1e_hw_pin_set_pupd_config,
	.qcom_tlmm_hw_pin_get_pupd_config = qcom_tlmm_x1e_hw_pin_get_pupd_config,
	.qcom_tlmm_hw_pin_set_drive_strength = qcom_tlmm_x1e_hw_pin_set_drive_strength,
	.qcom_tlmm_hw_pin_get_drive_strength = qcom_tlmm_x1e_hw_pin_get_drive_strength,
	.qcom_tlmm_hw_pin_set_vm = qcom_tlmm_x1e_hw_pin_set_vm,
	.qcom_tlmm_hw_pin_get_vm = qcom_tlmm_x1e_hw_pin_get_vm,
	.qcom_tlmm_hw_pin_set_open_drain = qcom_tlmm_x1e_hw_pin_set_open_drain,
	.qcom_tlmm_hw_pin_get_open_drain = qcom_tlmm_x1e_hw_pin_get_open_drain,
};

/* TODO: move to a header file */
extern void qcom_tlmm_x1e_attach(struct qcom_tlmm_softc *sc);

void
qcom_tlmm_x1e_attach(struct qcom_tlmm_softc *sc)
{

	sc->gpio_npins = QCOM_TLMM_X1E80100_GPIO_PINS;
	sc->gpio_muxes = &x1e80100_muxes[0];
	sc->sc_ap_reserved.pins = qcom_tlmm_x1e_reserved_pins;
	sc->sc_ap_reserved.npins = nitems(qcom_tlmm_x1e_reserved_pins);
	sc->sc_hw = &qcom_tlmm_x1e_hw_callbacks;
}
