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
 * Cadence USB3 dual-role controller ("cdns,usb3"), host role only.
 *
 * The controller has three register windows: "otg" (role control),
 * "xhci" (a standard xHCI) and "dev" (the device controller, unused here).
 * Request the host role from the OTG block, wait for the xHCI to become
 * ready and attach xhci(4) to the "xhci" window.  SoC glue (clocks, resets,
 * straps) is done by the parent node's driver.
 *
 * Only the cdns3 "v1" OTG register layout (DID 0x000402xx) is supported.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sx.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/phy/phy_usb.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_util.h>
#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/xhci.h>
#include <dev/usb/controller/xhcireg.h>
#include <dev/usb/controller/generic_xhci.h>

/* OTG block, cdns3 v1 layout. */
#define	CDNS3_OTG_DID			0x00
#define	 CDNS3_OTG_DID_V1(d)		(((d) & 0xffffff00) == 0x00040200)
#define	CDNS3_OTG_RID			0x04
#define	CDNS3_OTG_CMD			0x10
#define	 CDNS3_OTGCMD_HOST_BUS_REQ	(1u << 1)
#define	 CDNS3_OTGCMD_OTG_DIS		(1u << 3)
#define	CDNS3_OTG_STS			0x14
#define	 CDNS3_OTGSTS_STRAP(s)		(((s) >> 12) & 0x7)
#define	 CDNS3_STRAP_GADGET		0x4
#define	 CDNS3_OTGSTS_XHCI_READY	(1u << 26)
#define	CDNS3_OTG_IEN			0x20
#define	CDNS3_OTG_SIMULATE		0x40

#define	CDNS3_XHCI_READY_TIMEOUT	100000	/* us */

/* Cadence-specific registers in the xHCI window. */
#define	CDNS3_XECP_PORT_CAP		0x8000
#define	 CDNS3_LPM_2_STB_SWITCH_EN	(1u << 25)
#define	CDNS3_XECP_AUX_CTRL1		0x8120
#define	 CDNS3_CFG_RXDET_P3_EN		(1u << 15)

struct cdns3_xhci_softc {
	struct xhci_softc	xsc;	/* must be first */
	struct resource		*otg_res;
	int			otg_rid;
	phy_t			usb2_phy;
	phy_t			usb3_phy;
};

static struct ofw_compat_data compat_data[] = {
	{ "cdns,usb3",	1 },
	{ NULL,		0 }
};

static int
cdns3_xhci_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Cadence USB3 DRD controller (host)");
	return (BUS_PROBE_DEFAULT);
}

static int
cdns3_xhci_phys_enable(device_t dev, phandle_t node)
{
	struct cdns3_xhci_softc *sc = device_get_softc(dev);
	int error;

	if (phy_get_by_ofw_name(dev, node, "cdns3,usb2-phy",
	    &sc->usb2_phy) == 0 && (error = phy_enable(sc->usb2_phy)) != 0) {
		device_printf(dev, "cannot enable USB 2.0 PHY: %d\n", error);
		return (error);
	}
	if (phy_get_by_ofw_name(dev, node, "cdns3,usb3-phy",
	    &sc->usb3_phy) == 0 && (error = phy_enable(sc->usb3_phy)) != 0) {
		device_printf(dev, "cannot enable USB 3.0 PHY: %d\n", error);
		return (error);
	}
	return (0);
}

static int
cdns3_xhci_host_on(device_t dev)
{
	struct cdns3_xhci_softc *sc = device_get_softc(dev);
	uint32_t did, sts;
	int i;

	did = bus_read_4(sc->otg_res, CDNS3_OTG_DID);
	if (!CDNS3_OTG_DID_V1(did)) {
		device_printf(dev, "unsupported controller, DID %#x\n", did);
		return (ENXIO);
	}
	sts = bus_read_4(sc->otg_res, CDNS3_OTG_STS);
	if (CDNS3_OTGSTS_STRAP(sts) == CDNS3_STRAP_GADGET) {
		device_printf(dev, "controller strapped to device role\n");
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(dev, "DID %#x, RID %#x, strap %u\n", did,
		    bus_read_4(sc->otg_res, CDNS3_OTG_RID),
		    CDNS3_OTGSTS_STRAP(sts));

	bus_write_4(sc->otg_res, CDNS3_OTG_SIMULATE, 1);
	bus_write_4(sc->otg_res, CDNS3_OTG_IEN, 0);
	bus_write_4(sc->otg_res, CDNS3_OTG_CMD,
	    CDNS3_OTGCMD_HOST_BUS_REQ | CDNS3_OTGCMD_OTG_DIS);

	for (i = 0; i < CDNS3_XHCI_READY_TIMEOUT / 10; i++) {
		sts = bus_read_4(sc->otg_res, CDNS3_OTG_STS);
		if ((sts & CDNS3_OTGSTS_XHCI_READY) != 0)
			break;
		DELAY(10);
	}
	if ((sts & CDNS3_OTGSTS_XHCI_READY) == 0) {
		device_printf(dev, "timeout waiting for xHCI ready\n");
		return (ETIMEDOUT);
	}

	if (sc->usb2_phy != NULL)
		phy_usb_set_mode(sc->usb2_phy, PHY_USB_MODE_HOST);
	if (sc->usb3_phy != NULL)
		phy_usb_set_mode(sc->usb3_phy, PHY_USB_MODE_HOST);

	return (0);
}

static int
cdns3_xhci_detach(device_t dev)
{
	struct cdns3_xhci_softc *sc = device_get_softc(dev);
	int error;

	error = generic_xhci_detach(dev);
	if (error != 0)
		return (error);

	if (sc->otg_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->otg_rid,
		    sc->otg_res);
		sc->otg_res = NULL;
	}
	if (sc->usb3_phy != NULL) {
		phy_disable(sc->usb3_phy);
		phy_release(sc->usb3_phy);
		sc->usb3_phy = NULL;
	}
	if (sc->usb2_phy != NULL) {
		phy_disable(sc->usb2_phy);
		phy_release(sc->usb2_phy);
		sc->usb2_phy = NULL;
	}
	return (0);
}

static int
cdns3_xhci_attach(device_t dev)
{
	struct cdns3_xhci_softc *csc = device_get_softc(dev);
	struct xhci_softc *sc = &csc->xsc;
	phandle_t node;
	char mode[16];
	int xhci_rid, irq_rid, error;

	node = ofw_bus_get_node(dev);
	if (OF_getprop(node, "dr_mode", mode, sizeof(mode)) > 0 &&
	    strcmp(mode, "host") != 0 && strcmp(mode, "otg") != 0) {
		device_printf(dev, "dr_mode \"%s\" not supported\n", mode);
		return (ENXIO);
	}
	if (ofw_bus_find_string_index(node, "reg-names", "otg",
	    &csc->otg_rid) != 0 ||
	    ofw_bus_find_string_index(node, "reg-names", "xhci",
	    &xhci_rid) != 0 ||
	    ofw_bus_find_string_index(node, "interrupt-names", "host",
	    &irq_rid) != 0) {
		device_printf(dev, "missing otg/xhci registers or host IRQ\n");
		return (ENXIO);
	}

	csc->otg_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &csc->otg_rid, RF_ACTIVE);
	if (csc->otg_res == NULL) {
		device_printf(dev, "cannot map OTG registers\n");
		return (ENXIO);
	}

	if (cdns3_xhci_phys_enable(dev, node) != 0 ||
	    cdns3_xhci_host_on(dev) != 0)
		goto fail;

	/* The rest follows generic_xhci_attach(), with our resource ids. */
	sc->sc_bus.parent = dev;
	sc->sc_bus.devices = sc->sc_devices;
	sc->sc_bus.devices_max = XHCI_MAX_DEVICES;

	sc->sc_io_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &xhci_rid,
	    RF_ACTIVE);
	if (sc->sc_io_res == NULL) {
		device_printf(dev, "cannot map xHCI registers\n");
		goto fail;
	}
	sc->sc_io_tag = rman_get_bustag(sc->sc_io_res);
	sc->sc_io_hdl = rman_get_bushandle(sc->sc_io_res);
	sc->sc_io_size = rman_get_size(sc->sc_io_res);

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "cannot allocate host IRQ\n");
		goto fail;
	}

	sc->sc_bus.bdev = device_add_child(dev, "usbus", DEVICE_UNIT_ANY);
	if (sc->sc_bus.bdev == NULL) {
		device_printf(dev, "cannot add USB bus\n");
		goto fail;
	}
	device_set_ivars(sc->sc_bus.bdev, &sc->sc_bus);
	strlcpy(sc->sc_vendor, "Cadence", sizeof(sc->sc_vendor));
	device_set_desc(sc->sc_bus.bdev, "Cadence USB3 xHCI");

	error = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, (driver_intr_t *)xhci_interrupt, sc, &sc->sc_intr_hdl);
	if (error != 0) {
		device_printf(dev, "cannot set up IRQ: %d\n", error);
		sc->sc_intr_hdl = NULL;
		goto fail;
	}

	if (xhci_init(sc, dev, 0) != 0) {
		device_printf(dev, "xHCI init failed\n");
		goto fail;
	}
	if (xhci_start_controller(sc) != 0) {
		device_printf(dev, "xHCI start failed\n");
		goto fail;
	}

	/* Rx detect in P3, LPM -> standby clock switch (cdns3 specific). */
	bus_write_4(sc->sc_io_res, CDNS3_XECP_AUX_CTRL1,
	    bus_read_4(sc->sc_io_res, CDNS3_XECP_AUX_CTRL1) |
	    CDNS3_CFG_RXDET_P3_EN);
	bus_write_4(sc->sc_io_res, CDNS3_XECP_PORT_CAP,
	    bus_read_4(sc->sc_io_res, CDNS3_XECP_PORT_CAP) |
	    CDNS3_LPM_2_STB_SWITCH_EN);

	if (device_probe_and_attach(sc->sc_bus.bdev) != 0) {
		device_printf(dev, "USB bus attach failed\n");
		goto fail;
	}

	return (0);

fail:
	cdns3_xhci_detach(dev);
	return (ENXIO);
}

static device_method_t cdns3_xhci_methods[] = {
	DEVMETHOD(device_probe,		cdns3_xhci_probe),
	DEVMETHOD(device_attach,	cdns3_xhci_attach),
	DEVMETHOD(device_detach,	cdns3_xhci_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	DEVMETHOD_END
};

/* Named "xhci" so that usbus(4) attaches to it. */
static driver_t cdns3_xhci_driver = {
	"xhci",
	cdns3_xhci_methods,
	sizeof(struct cdns3_xhci_softc),
};

DRIVER_MODULE(cdns3_xhci, simplebus, cdns3_xhci_driver, 0, 0);
MODULE_DEPEND(cdns3_xhci, usb, 1, 1, 1);
MODULE_DEPEND(cdns3_xhci, xhci, 1, 1, 1);
