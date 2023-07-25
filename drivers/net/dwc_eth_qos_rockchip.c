// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2023 Collabora, Ltd.
 *
 * Rockchip DWMAC4 QOS specific glue layer
 */
#define DEBUG
#include <common.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3588.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <phy.h>
#include <reset.h>
#include <syscon.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#include "dwc_eth_qos.h"

static int eqos_set_tx_clk_speed_rk3588(struct udevice *dev);
static const u32 rk3588_instance_regs[] = {
	0xfe1b0000, /* gmac0 */
	0xfe1c0000, /* gmac1 */
	0x0, /* sentinel */
};

struct eqos_rockchip_priv {
	bool clock_input;
	int instance_id;
	u32 tx_delay;
	u32 rx_delay;
};

static int eqos_start_clks_rk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	int ret;

	debug("%s(dev=%p):\n", __func__, dev);

	if (clk_valid(&eqos->clk_slave_bus)) {
		ret = clk_enable(&eqos->clk_slave_bus);
		if (ret < 0)
			pr_info("clk_enable(clk_slave_bus) failed: %d", ret);
	}

	if (clk_valid(&eqos->clk_master_bus)) {
		ret = clk_enable(&eqos->clk_master_bus);
		if (ret < 0)
			pr_info("clk_enable(clk_master_bus) failed: %d", ret);
	}

	if (clk_valid(&eqos->clk_ck) && !eqos->clk_ck_enabled) {
		ret = clk_enable(&eqos->clk_ck);
		if (ret < 0)
			pr_info("clk_enable(clk_ck) failed: %d", ret);
		else
			eqos->clk_ck_enabled = true;
	}

	if (clk_valid(&eqos->clk_ptp_ref)) {
		ret = clk_enable(&eqos->clk_ptp_ref);
		if (ret < 0)
			pr_info("clk_enable(clk_ptp_ref) failed: %d", ret);
	}

	if (clk_valid(&eqos->clk_tx)) {
		ret = clk_enable(&eqos->clk_tx);
		if (ret < 0)
			pr_info("clk_enable(clk_tx) failed: %d", ret);
	}
//eqos_set_tx_clk_speed_rk3588(dev);
	debug("%s: OK\n", __func__);
	return 0;
}

static int eqos_start_clks_rk3588(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk = eqos->priv;
	struct rk3588_php_grf *php_grf;
	unsigned int val, mask;

	enum {
		RK3588_GMAC_CLK_SELECT_SHIFT = 0x4,
		RK3588_GMAC_CLK_SELECT_MASK = BIT(4),
		RK3588_GMAC_CLK_SELECT_CRU = BIT(4),
		RK3588_GMAC_CLK_SELECT_IO = 0,
	};

	debug("%s(dev=%p):\n", __func__, dev);

	php_grf = syscon_get_first_range(ROCKCHIP_SYSCON_PHP_GRF);
	val = rk->clock_input ? RK3588_GMAC_CLK_SELECT_IO :
				RK3588_GMAC_CLK_SELECT_CRU;
	mask = RK3588_GMAC_CLK_SELECT_MASK;

	/* For the second instance, registers are shifted */
	if (rk->instance_id == 1) {
		val <<= 5;
		mask <<= 5;
	}

	rk_clrsetreg(&php_grf->clk_con1, mask, val);

	debug("%s: OK\n", __func__);
	return eqos_start_clks_rk(dev);
}
static int eqos_stop_clks_rk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);

	debug("%s(dev=%p):\n", __func__, dev);

	if (clk_valid(&eqos->clk_tx))
		clk_disable(&eqos->clk_tx);
	if (clk_valid(&eqos->clk_ptp_ref))
		clk_disable(&eqos->clk_ptp_ref);
	if (clk_valid(&eqos->clk_ck) && eqos->clk_ck_enabled)
		clk_disable(&eqos->clk_ck);
	if (clk_valid(&eqos->clk_master_bus))
		clk_disable(&eqos->clk_master_bus);
	if (clk_valid(&eqos->clk_slave_bus))
		clk_disable(&eqos->clk_slave_bus);

	debug("%s: OK\n", __func__);
	return 0;
}

static void eqos_set_to_rgmii_rk3588(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk = eqos->priv;
	unsigned int rx_enable, rx_enable_mask, tx_enable, tx_enable_mask;
	unsigned int intf_sel, intf_sel_mask;
	unsigned int clk_mode, clk_mode_mask;
	unsigned int rx_delay;
	struct rk3588_php_grf *php_grf;
	struct rk3588_sys_grf *grf;
	void *offset_con;
printf("set to rgmii %x %x \n",rk->rx_delay, rk->tx_delay);
//rk->rx_delay = 0; rk->tx_delay=0;
	enum {
		RK3588_GMAC_PHY_INTF_SEL_SHIFT = 3,
		RK3588_GMAC_PHY_INTF_SEL_MASK = GENMASK(5, 3),
		RK3588_GMAC_PHY_INTF_SEL_RGMII = BIT(3),

		RK3588_RXCLK_DLY_ENA_GMAC_MASK = BIT(3),
		RK3588_RXCLK_DLY_ENA_GMAC_DISABLE = 0,
		RK3588_RXCLK_DLY_ENA_GMAC_ENABLE = BIT(3),

		RK3588_TXCLK_DLY_ENA_GMAC_MASK = BIT(2),
		RK3588_TXCLK_DLY_ENA_GMAC_DISABLE = 0,
		RK3588_TXCLK_DLY_ENA_GMAC_ENABLE = BIT(2),
	};

	enum {
		RK3588_CLK_RX_DL_CFG_GMAC_SHIFT = 0x8,
		RK3588_CLK_RX_DL_CFG_GMAC_MASK = GENMASK(15, 8),

		RK3588_CLK_TX_DL_CFG_GMAC_SHIFT = 0x0,
		RK3588_CLK_TX_DL_CFG_GMAC_MASK = GENMASK(7, 0),
	};

	enum {
		RK3588_GMAC_CLK_RGMII_MODE_SHIFT = 0x0,
		RK3588_GMAC_CLK_RGMII_MODE_MASK = BIT(0),
		RK3588_GMAC_CLK_RGMII_MODE = 0x0,
	};

	grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	php_grf = syscon_get_first_range(ROCKCHIP_SYSCON_PHP_GRF);

	if (rk->rx_delay < 0) {
		rx_enable = RK3588_RXCLK_DLY_ENA_GMAC_DISABLE;
		rx_delay = 0;
printf("OOPS !\n");
	} else {
		rx_enable = RK3588_RXCLK_DLY_ENA_GMAC_ENABLE;
		rx_delay = rk->rx_delay << RK3588_CLK_RX_DL_CFG_GMAC_SHIFT;
	}

	if (rk->instance_id == 1) {
		offset_con = &grf->soc_con9;
		rx_enable = rx_enable << 2;
printf("rx enable = %x\n", rx_enable);
		rx_enable_mask = RK3588_RXCLK_DLY_ENA_GMAC_MASK << 2;
		tx_enable = RK3588_TXCLK_DLY_ENA_GMAC_ENABLE << 2;
		tx_enable_mask = RK3588_TXCLK_DLY_ENA_GMAC_MASK << 2;
		intf_sel = RK3588_GMAC_PHY_INTF_SEL_RGMII << 6;
		intf_sel_mask = RK3588_GMAC_PHY_INTF_SEL_MASK << 6;
		clk_mode = RK3588_GMAC_CLK_RGMII_MODE << 5;
		clk_mode_mask = RK3588_GMAC_CLK_RGMII_MODE_MASK << 5;
	} else {
		offset_con = &grf->soc_con8;
		rx_enable_mask = RK3588_RXCLK_DLY_ENA_GMAC_MASK;
		tx_enable = RK3588_TXCLK_DLY_ENA_GMAC_ENABLE;
		tx_enable_mask = RK3588_TXCLK_DLY_ENA_GMAC_MASK;
		intf_sel = RK3588_GMAC_PHY_INTF_SEL_RGMII;
		intf_sel_mask = RK3588_GMAC_PHY_INTF_SEL_MASK;
		clk_mode = RK3588_GMAC_CLK_RGMII_MODE;
		clk_mode_mask = RK3588_GMAC_CLK_RGMII_MODE_MASK;
	}

	rk_clrsetreg(offset_con,
		     RK3588_CLK_TX_DL_CFG_GMAC_MASK |
		     RK3588_CLK_RX_DL_CFG_GMAC_MASK,
		     rk->tx_delay << RK3588_CLK_TX_DL_CFG_GMAC_SHIFT |
		     rx_delay);

printf("offset_con %x\n", rk->tx_delay << RK3588_CLK_TX_DL_CFG_GMAC_SHIFT |           
                     rx_delay);
	rk_clrsetreg(&grf->soc_con7, tx_enable_mask | rx_enable_mask,
		     tx_enable | rx_enable);
printf("SOC CON7 %x %x\n",tx_enable_mask | rx_enable_mask, tx_enable| rx_enable);

	rk_clrsetreg(&php_grf->gmac_con0, intf_sel_mask, intf_sel);

printf("phpgrf gmac_con0 %x\n", intf_sel);
	rk_clrsetreg(&php_grf->clk_con1, clk_mode_mask, clk_mode);
printf("phpgrf clk-con1 %x\n", clk_mode);
}

static int eqos_set_mac_speed_rk3588(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk = eqos->priv;
	struct rk3588_php_grf *php_grf;
	phy_interface_t interface;
	unsigned int div, div_mask;

	enum {
		RK3588_GMAC_CLK_RGMII_DIV_SHIFT = 2,
		RK3588_GMAC_CLK_RGMII_DIV_MASK = GENMASK(3, 2),
		RK3588_GMAC_CLK_RGMII_DIV1 = 0,
		RK3588_GMAC_CLK_RGMII_DIV5 = GENMASK(3, 2),
		RK3588_GMAC_CLK_RGMII_DIV50 = BIT(3),
		RK3588_GMA_CLK_RMII_DIV2 = BIT(2),
		RK3588_GMAC_CLK_RMII_DIV20 = 0,
	};

	debug("%s(dev=%p):\n", __func__, dev);

	interface = eqos->config->interface(dev);

	php_grf = syscon_get_first_range(ROCKCHIP_SYSCON_PHP_GRF);

	switch (eqos->phy->speed) {
	case 10:
		if (interface == PHY_INTERFACE_MODE_RMII)
			div = RK3588_GMAC_CLK_RMII_DIV20;
		else
			div = RK3588_GMAC_CLK_RGMII_DIV50;
		break;
	case 100:
		if (interface == PHY_INTERFACE_MODE_RMII)
			div = RK3588_GMA_CLK_RMII_DIV2;
		else
			div = RK3588_GMAC_CLK_RGMII_DIV5;
		break;
	case 1000:
		if (interface != PHY_INTERFACE_MODE_RMII) {printf("setting mac speed 1000\n");
			div = RK3588_GMAC_CLK_RGMII_DIV1;}
		else
			return -EINVAL;
		break;
	default:
		debug("Unknown phy speed: %d\n", eqos->phy->speed);
		return -EINVAL;
	}

	if (rk->instance_id == 1) {
		div <<= 5;
		div_mask = RK3588_GMAC_CLK_RGMII_DIV_MASK << 5;
	}

	rk_clrsetreg(&php_grf->clk_con1, div_mask, div);

	debug("%s: OK\n", __func__);
	return 0;
}


/* Clock rates */
#define RGMII_1000_NOM_CLK_FREQ			(250 * 1000 * 1000UL)
#define RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ	 (50 * 1000 * 1000UL)
#define RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ	  (5 * 1000 * 1000UL)

static int eqos_set_tx_clk_speed_rk3588(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk = eqos->priv;
	phy_interface_t interface;
	ulong rate;

	debug("%s(dev=%p):\n", __func__, dev);

	interface = eqos->config->interface(dev);

	eqos_set_mac_speed_rk3588(dev);

	switch (interface) {
	case PHY_INTERFACE_MODE_RGMII:
		/* Set to RGMII mode */
printf("RGMII simple %d\n", PHY_INTERFACE_MODE_RGMII);
		eqos_set_to_rgmii_rk3588(dev);
		/*
		 * If the gmac clock is from internal pll, need to set and
		 * check the return value for gmac clock at RGMII mode. If
		 * the gmac clock is from external source, the clock rate
		 * is not set, because of it is bypassed.
		 */

		if (!rk->clock_input && clk_valid(&eqos->clk_master_bus)) {
			rate = clk_set_rate(&eqos->clk_master_bus, 125000000);
			if (rate != 125000000)
				return -EINVAL;
		}
		break;
#if 0
	case PHY_INTERFACE_MODE_RGMII_ID:
		/* Set to RGMII mode */
		if (ops->set_to_rgmii) {
			pdata->tx_delay = 0;
			pdata->rx_delay = 0;
printf("RGMII ID\n");
			ops->set_to_rgmii(pdata);
		} else
			return -EPERM;

		if (!pdata->clock_input) {
			rate = clk_set_rate(&clk, 125000000);
			if (rate != 125000000)
				return -EINVAL;
		}
		break;

	case PHY_INTERFACE_MODE_RMII:
		/* Set to RMII mode */
		if (ops->set_to_rmii)
			ops->set_to_rmii(pdata);
		else
			return -EPERM;

		if (!pdata->clock_input) {
			rate = clk_set_rate(&clk, 50000000);
			if (rate != 50000000)
				return -EINVAL;
		}
		break;

	case PHY_INTERFACE_MODE_RGMII_RXID:
		 /* Set to RGMII_RXID mode */
		if (ops->set_to_rgmii) {
			pdata->tx_delay = 0;
			pdata->rx_delay = -1;
printf("RXID ?!\n");
			ops->set_to_rgmii(pdata);
		} else
			return -EPERM;

		if (!pdata->clock_input) {
			rate = clk_set_rate(&clk, 125000000);
			if (rate != 125000000)
				return -EINVAL;
		}
		break;

	case PHY_INTERFACE_MODE_RGMII_TXID:
		/* Set to RGMII_TXID mode */
		if (ops->set_to_rgmii) {
			pdata->rx_delay = 0;
printf("RGMII TXID\n");
			ops->set_to_rgmii(pdata);
		} else
			return -EPERM;

		if (!pdata->clock_input) {
			rate = clk_set_rate(&clk, 125000000);
			if (rate != 125000000)
				return -EINVAL;
		}
		break;
#endif
	default:
		debug("NO interface defined!\n");
		return -ENXIO;
	}

	debug("%s: OK\n", __func__);
	return 0;
}

static int eqos_probe_resources_rk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk;
	phy_interface_t interface;
	int ret;

	debug("%s(dev=%p):\n", __func__, dev);

	interface = eqos->config->interface(dev);

	if (interface == PHY_INTERFACE_MODE_NA) {
		pr_err("Invalid PHY interface\n");
		return -EINVAL;
	}

	eqos->priv = devm_kmalloc(dev, sizeof(struct eqos_rockchip_priv),
				  GFP_KERNEL);
	if (!eqos->priv)
		return -ENOMEM;
	rk = eqos->priv;

	rk->clock_input = true;
	if (!strcmp(dev_read_string(dev, "clock_in_out"), "output"))
		rk->clock_input = false;

	rk->tx_delay = dev_read_u32_default(dev, "tx_delay", 0x30);
	rk->rx_delay = dev_read_u32_default(dev, "rx_delay", 0x10);

	ret = clk_get_by_name(dev, "pclk_mac", &eqos->clk_slave_bus);
	if (ret)
		printf("clk_get_by_name(pclk_mac) failed: %d", ret);

	ret = clk_get_by_name(dev, "stmmaceth", &eqos->clk_master_bus);
	if (ret)
		printf("clk_get_by_name(stmmaceth) failed: %d", ret);

			clk_set_rate(&eqos->clk_master_bus, 125000000);
	ret = clk_get_by_name(dev, "aclk_mac", &eqos->clk_ck);
	if (ret)
		printf("clk_get_by_name(aclk_mac) failed: %d", ret);

	ret = clk_get_by_name(dev, "ptp_ref", &eqos->clk_ptp_ref);
	if (ret)
		printf("clk_get_by_name(ptp_ref) failed: %d", ret);

	ret = clk_get_by_name(dev, "clk_mac_ref", &eqos->clk_tx);
	if (ret)
		printf("clk_get_by_name(clk_mac_ref) failed: %d", ret);

	debug("%s: OK\n", __func__);
	return 0;
}

static int eqos_probe_resources_rk3588(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eqos_rockchip_priv *rk;
	int ret;
	int i = 0;

	ret = eqos_probe_resources_rk(dev);
	if (ret)
		return ret;

	rk = eqos->priv;
	while (rk3588_instance_regs[i]) {
		if (rk3588_instance_regs[i] == eqos->regs) {
			rk->instance_id = i;
			break;
		}
		i++;
	}
	printf("Instance id = %d\n", rk->instance_id);

	return ret;
}

static int eqos_remove_resources_rk(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);

	debug("%s(dev=%p):\n", __func__, dev);

	if (clk_valid(&eqos->clk_tx))
		clk_free(&eqos->clk_tx);
	if (clk_valid(&eqos->clk_ptp_ref))
		clk_free(&eqos->clk_ptp_ref);
	if (clk_valid(&eqos->clk_ck))
		clk_free(&eqos->clk_ck);
	if (clk_valid(&eqos->clk_master_bus))
		clk_free(&eqos->clk_master_bus);
	if (clk_valid(&eqos->clk_slave_bus))
		clk_free(&eqos->clk_slave_bus);

	reset_free(&eqos->reset_ctl);

	debug("%s: OK\n", __func__);
	return 0;
}

static struct eqos_ops eqos_rk3588_ops = {
	.eqos_inval_desc = eqos_inval_desc_generic,
	.eqos_flush_desc = eqos_flush_desc_generic,
	.eqos_inval_buffer = eqos_inval_buffer_generic,
	.eqos_flush_buffer = eqos_flush_buffer_generic,
	.eqos_probe_resources = eqos_probe_resources_rk3588,
	.eqos_remove_resources = eqos_remove_resources_rk,
	.eqos_stop_resets = eqos_null_ops,
	.eqos_start_resets = eqos_null_ops,
	.eqos_stop_clks = eqos_stop_clks_rk,
	.eqos_start_clks = eqos_start_clks_rk3588,
	.eqos_calibrate_pads = eqos_null_ops,
	.eqos_disable_calibration = eqos_null_ops,
	.eqos_set_tx_clk_speed = eqos_set_tx_clk_speed_rk3588,
	.eqos_get_enetaddr = eqos_null_ops,
};

struct eqos_config __maybe_unused eqos_rk3588_config = {
	.reg_access_always_ok = false,
	.mdio_wait = 10000,
	.swr_wait = 200,
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED,
	.config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_100_150,
	.axi_bus_width = EQOS_AXI_WIDTH_128,
	.interface = dev_read_phy_mode,
	.ops = &eqos_rk3588_ops,
};
