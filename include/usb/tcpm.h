/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2017 Google, Inc
 * Copyright 2024 Collabora
 */

#ifndef __LINUX_USB_TCPM_H
#define __LINUX_USB_TCPM_H

#include <dm/of.h>
#include <linux/bitops.h>
#include "pd.h"

enum typec_cc_status {
	TYPEC_CC_OPEN,
	TYPEC_CC_RA,
	TYPEC_CC_RD,
	TYPEC_CC_RP_DEF,
	TYPEC_CC_RP_1_5,
	TYPEC_CC_RP_3_0,
};

enum typec_cc_polarity {
	TYPEC_POLARITY_CC1,
	TYPEC_POLARITY_CC2,
};

enum tcpm_transmit_status {
	TCPC_TX_SUCCESS = 0,
	TCPC_TX_DISCARDED = 1,
	TCPC_TX_FAILED = 2,
};

enum tcpm_transmit_type {
	TCPC_TX_SOP = 0,
	TCPC_TX_SOP_PRIME = 1,
	TCPC_TX_SOP_PRIME_PRIME = 2,
	TCPC_TX_SOP_DEBUG_PRIME = 3,
	TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
	TCPC_TX_HARD_RESET = 5,
	TCPC_TX_CABLE_RESET = 6,
	TCPC_TX_BIST_MODE_2 = 7
};

/**
 * struct tcpc_dev - Port configuration and callback functions
 * @fwnode:	Pointer to port fwnode
 * @get_vbus:	Called to read current VBUS state
 * @set_cc:	Called to set value of CC pins
 * @get_cc:	Called to read current CC pin values
 * @set_polarity:
 *		Called to set polarity
 * @set_vconn:	Called to enable or disable VCONN
 * @set_vbus:	Called to enable or disable VBUS
 * @set_pd_rx:	Called to enable or disable reception of PD messages
 * @set_roles:	Called to set power and data roles
 * @start_toggling:
 *		Optional; if supported by hardware, called to start dual-role
 *		toggling or single-role connection detection. Toggling stops
 *		automatically if a connection is established.
 * @pd_transmit:Called to transmit PD message
 * @poll_event:
 * 		After the PD chip driver is loaded, the callback function will be
 * 		called to poll what events have been triggered.
 * @enter_low_power_mode:
 * 		Optional; the pd chip enters low power mode.
 */
struct tcpc_dev {
	ofnode connector_node;
	int (*init)(struct tcpc_dev *dev);
	int (*get_vbus)(struct tcpc_dev *dev);
	int (*set_cc)(struct tcpc_dev *dev, enum typec_cc_status cc);
	int (*get_cc)(struct tcpc_dev *dev, enum typec_cc_status *cc1,
		      enum typec_cc_status *cc2);
	int (*set_polarity)(struct tcpc_dev *dev,
			    enum typec_cc_polarity polarity);
	int (*set_vconn)(struct tcpc_dev *dev, bool on);
	int (*set_vbus)(struct tcpc_dev *dev, bool on, bool charge);
	int (*set_pd_rx)(struct tcpc_dev *dev, bool on);
	int (*set_roles)(struct tcpc_dev *dev, bool attached,
			 enum typec_role role, enum typec_data_role data);
	int (*start_toggling)(struct tcpc_dev *dev,
			      enum typec_port_type port_type,
			      enum typec_cc_status cc);
	int (*pd_transmit)(struct tcpc_dev *dev, enum tcpm_transmit_type type,
			   const struct pd_message *msg, unsigned int negotiated_rev);
	void (*poll_event)(struct tcpc_dev *dev);
	int (*enter_low_power_mode)(struct tcpc_dev *dev, bool attached, bool pd_capable);
};

struct dm_tcpm_ops {
	int (*get_voltage)(struct udevice *dev);
	int (*get_current)(struct udevice *dev);
	const char* (*get_state)(struct udevice *dev);
};

/* API for drivers */
struct tcpm_port;
struct tcpm_port *tcpm_port_init(struct udevice *dev, struct tcpc_dev *tcpc);
void tcpm_poll_event(struct tcpm_port *port);
void tcpm_vbus_change(struct tcpm_port *port);
void tcpm_cc_change(struct tcpm_port *port);
void tcpm_pd_receive(struct tcpm_port *port,
		     const struct pd_message *msg);
void tcpm_pd_transmit_complete(struct tcpm_port *port,
			       enum tcpm_transmit_status status);
void tcpm_pd_hard_reset(struct tcpm_port *port);
int tcpm_get_current(struct tcpm_port *port);
int tcpm_get_voltage(struct tcpm_port *port);
const char* tcpm_get_state(struct tcpm_port *port);

/* API for boards */
int tcpm_get(const char *name, struct udevice **devp);
int tcpm_print_info(struct udevice *devp);

#endif /* __LINUX_USB_TCPM_H */
