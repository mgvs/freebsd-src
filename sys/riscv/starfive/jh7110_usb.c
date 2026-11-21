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
 * StarFive JH7110 USB wrapper ("starfive,jh7110-usb").
 *
 * The wrapper holds a Cadence USB3 dual-role controller ("cdns,usb3"
 * child node, handled by cdns3_xhci_fdt.c).  Configure the controller's
 * strap and PHY/PLL bits in the STG syscon, enable its clocks and release
 * its resets, then attach the child like a simplebus.  Host role only.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/simplebus.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/syscon/syscon.h>

#include "syscon_if.h"

/* USB mode register, at the offset given in "starfive,stg-syscon". */
#define	STG_USB_STRAP_MASK	0x00070000	/* bits 18:16 */
#define	 STG_USB_STRAP_HOST	0x00020000
#define	 STG_USB_STRAP_DEVICE	0x00040000
#define	STG_USB_SUSPENDM_HOST	0x00080000
#define	STG_USB_SUSPENDM_BYPS	0x00100000
#define	STG_USB_PLL_EN		0x00400000
#define	STG_USB_REFCLK_MODE	0x00800000

static int
jh7110_usb_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "starfive,jh7110-usb"))
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 USB");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_usb_attach(device_t dev)
{
	struct simplebus_softc *sc;
	struct syscon *stg;
	const char **names;
	phandle_t node, child;
	pcell_t cells[2];
	char mode[16];
	clk_t clk;
	hwreset_t rst;
	int error, i, n;

	node = ofw_bus_get_node(dev);

	if (OF_getprop(node, "dr_mode", mode, sizeof(mode)) > 0 &&
	    strcmp(mode, "host") != 0) {
		device_printf(dev, "dr_mode \"%s\" not supported\n", mode);
		return (ENXIO);
	}

	if (syscon_get_by_ofw_property(dev, node, "starfive,stg-syscon",
	    &stg) != 0 || OF_getencprop(node, "starfive,stg-syscon", cells,
	    sizeof(cells)) != sizeof(cells)) {
		device_printf(dev, "cannot get starfive,stg-syscon\n");
		return (ENXIO);
	}

	/* Strap the controller to host before its clocks and resets. */
	SYSCON_MODIFY_4(stg, cells[1], 0,
	    STG_USB_SUSPENDM_BYPS | STG_USB_PLL_EN | STG_USB_REFCLK_MODE);
	SYSCON_MODIFY_4(stg, cells[1], STG_USB_STRAP_MASK, STG_USB_STRAP_HOST);
	SYSCON_MODIFY_4(stg, cells[1], STG_USB_SUSPENDM_HOST,
	    STG_USB_SUSPENDM_HOST);

	/*
	 * Every clock and reset in the node is needed: an access to the
	 * controller with any of them off stalls the bus.
	 */
	n = ofw_bus_string_list_to_array(node, "clock-names", &names);
	if (n > 0)
		OF_prop_free(names);
	for (i = 0; i < n; i++) {
		if ((error = clk_get_by_ofw_index(dev, node, i, &clk)) != 0 ||
		    (error = clk_enable(clk)) != 0) {
			device_printf(dev, "cannot enable clock %d: %d\n", i,
			    error);
			return (ENXIO);
		}
	}
	n = ofw_bus_string_list_to_array(node, "reset-names", &names);
	if (n > 0)
		OF_prop_free(names);
	for (i = 0; i < n; i++) {
		if ((error = hwreset_get_by_ofw_idx(dev, node, i, &rst)) != 0 ||
		    (error = hwreset_deassert(rst)) != 0) {
			device_printf(dev, "cannot deassert reset %d: %d\n", i,
			    error);
			return (ENXIO);
		}
	}

	fdt_pinctrl_configure_by_name(dev, "default");

	/* Our softc is the simplebus one (see kobj_instance_offset()). */
	simplebus_init(dev, node);
	sc = device_get_softc_class(dev, &simplebus_driver);
	if (simplebus_fill_ranges(node, sc) < 0) {
		device_printf(dev, "cannot parse ranges\n");
		return (ENXIO);
	}
	for (child = OF_child(node); child > 0; child = OF_peer(child))
		simplebus_add_device(dev, child, 0, NULL, -1, NULL);
	bus_attach_children(dev);

	return (0);
}

static device_method_t jh7110_usb_methods[] = {
	DEVMETHOD(device_probe,		jh7110_usb_probe),
	DEVMETHOD(device_attach,	jh7110_usb_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(jh7110_usb, jh7110_usb_driver, jh7110_usb_methods, 0,
    simplebus_driver);
DRIVER_MODULE(jh7110_usb, simplebus, jh7110_usb_driver, 0, 0);
