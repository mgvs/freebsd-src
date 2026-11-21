/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Oleksandr Zhabotynskyi <alexlp@mgvs.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
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
 * StarFive JH7110 USB 2.0 PHY ("starfive,jh7110-usb-phy").
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/phy/phy_usb.h>
#include <dev/syscon/syscon.h>

#include "phynode_if.h"
#include "phynode_usb_if.h"
#include "syscon_if.h"

#define	USB2PHY_CLK_MODE		0x00
#define	 USB2PHY_CLK_RX_NORMAL_PWR	(1u << 1)
#define	USB2PHY_LS_KEEPALIVE		0x04
#define	 USB2PHY_LS_KEEPALIVE_EN	(1u << 4)

#define	USB2PHY_REFCLK_RATE		125000000

/* In the SYS syscon (not referenced by the PHY binding). */
#define	SYS_SYSCON_USB_SPLIT		0x18
#define	 SYS_SYSCON_USB_PDRSTN_SPLIT	(1u << 17)

struct jh7110_usbphy_softc {
	device_t		dev;
	struct resource		*res;
	clk_t			clk_125m;
	clk_t			clk_app;
	struct syscon		*sys_syscon;
};

struct jh7110_usbphy_node_sc {
	struct phynode_usb_sc	usb_sc;
	int			mode;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-usb-phy",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int
jh7110_usbphy_enable(struct phynode *phynode, bool enable)
{
	struct jh7110_usbphy_softc *sc;
	int error;

	sc = device_get_softc(phynode_get_device(phynode));

	if (!enable)
		return (clk_disable(sc->clk_app));

	error = clk_set_freq(sc->clk_125m, USB2PHY_REFCLK_RATE,
	    CLK_SET_ROUND_DOWN);
	if (error != 0) {
		device_printf(sc->dev, "cannot set 125m clock: %d\n", error);
		return (error);
	}
	error = clk_enable(sc->clk_app);
	if (error != 0) {
		device_printf(sc->dev, "cannot enable app_125m clock: %d\n",
		    error);
		return (error);
	}
	WR4(sc, USB2PHY_CLK_MODE,
	    RD4(sc, USB2PHY_CLK_MODE) | USB2PHY_CLK_RX_NORMAL_PWR);

	return (0);
}

static int
jh7110_usbphy_set_mode(struct phynode *phynode, int mode)
{
	struct jh7110_usbphy_softc *sc;
	struct jh7110_usbphy_node_sc *nsc;
	uint32_t val;

	sc = device_get_softc(phynode_get_device(phynode));
	nsc = phynode_get_softc(phynode);

	switch (mode) {
	case PHY_USB_MODE_HOST:
	case PHY_USB_MODE_OTG:
	case PHY_USB_MODE_DEVICE:
		break;
	default:
		return (EINVAL);
	}

	/* Low-speed keep-alive is generated in host and OTG roles only. */
	val = RD4(sc, USB2PHY_LS_KEEPALIVE);
	if (mode == PHY_USB_MODE_DEVICE)
		val &= ~USB2PHY_LS_KEEPALIVE_EN;
	else
		val |= USB2PHY_LS_KEEPALIVE_EN;
	WR4(sc, USB2PHY_LS_KEEPALIVE, val);

	SYSCON_MODIFY_4(sc->sys_syscon, SYS_SYSCON_USB_SPLIT,
	    SYS_SYSCON_USB_PDRSTN_SPLIT, SYS_SYSCON_USB_PDRSTN_SPLIT);

	nsc->mode = mode;
	return (0);
}

static int
jh7110_usbphy_get_mode(struct phynode *phynode, int *mode)
{
	struct jh7110_usbphy_node_sc *nsc;

	nsc = phynode_get_softc(phynode);
	*mode = nsc->mode;
	return (0);
}

static phynode_method_t jh7110_usbphy_node_methods[] = {
	PHYNODEUSBMETHOD(phynode_enable,	jh7110_usbphy_enable),
	PHYNODEUSBMETHOD(phynode_usb_set_mode,	jh7110_usbphy_set_mode),
	PHYNODEUSBMETHOD(phynode_usb_get_mode,	jh7110_usbphy_get_mode),
	PHYNODEUSBMETHOD_END
};

DEFINE_CLASS_1(jh7110_usbphy_node, jh7110_usbphy_node_class,
    jh7110_usbphy_node_methods, sizeof(struct jh7110_usbphy_node_sc),
    phynode_usb_class);

static int
jh7110_usbphy_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 USB 2.0 PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_usbphy_attach(device_t dev)
{
	struct jh7110_usbphy_softc *sc;
	struct phynode_usb_init_def init;
	struct phynode *phynode;
	struct jh7110_usbphy_node_sc *nsc;
	phandle_t node, snode;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, node, "125m", &sc->clk_125m) != 0 ||
	    clk_get_by_ofw_name(dev, node, "app_125m", &sc->clk_app) != 0) {
		device_printf(dev, "cannot get clocks\n");
		goto fail;
	}

	snode = ofw_bus_find_compatible(OF_finddevice("/"),
	    "starfive,jh7110-sys-syscon");
	if (snode <= 0 ||
	    syscon_get_by_ofw_node(dev, snode, &sc->sys_syscon) != 0) {
		device_printf(dev, "cannot get SYS syscon\n");
		goto fail;
	}

	bzero(&init, sizeof(init));
	init.phynode_init_def.id = 0;
	init.phynode_init_def.ofw_node = node;
	init.std_param.usb_mode = PHY_USB_MODE_UNKNOWN;
	phynode = phynode_usb_create(dev, &jh7110_usbphy_node_class, &init);
	if (phynode == NULL) {
		device_printf(dev, "cannot create PHY\n");
		goto fail;
	}
	nsc = phynode_get_softc(phynode);
	nsc->mode = PHY_USB_MODE_UNKNOWN;
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "cannot register PHY\n");
		goto fail;
	}

	return (0);

fail:
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
	return (ENXIO);
}

static device_method_t jh7110_usbphy_methods[] = {
	DEVMETHOD(device_probe,		jh7110_usbphy_probe),
	DEVMETHOD(device_attach,	jh7110_usbphy_attach),

	DEVMETHOD_END
};

static DEFINE_CLASS_0(jh7110_usbphy, jh7110_usbphy_driver,
    jh7110_usbphy_methods, sizeof(struct jh7110_usbphy_softc));

EARLY_DRIVER_MODULE(jh7110_usbphy, simplebus, jh7110_usbphy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
