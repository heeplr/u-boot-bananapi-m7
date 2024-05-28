// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2017 Google, Inc
 *
 * USB Power Delivery protocol stack.
 */

#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/devres.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <time.h>
#include <usb/tcpm.h>

DECLARE_GLOBAL_DATA_PTR;

#define FOREACH_STATE(S)			\
	S(INVALID_STATE),			\
	S(TOGGLING),			\
	S(SRC_UNATTACHED),			\
	S(SRC_ATTACH_WAIT),			\
	S(SRC_ATTACHED),			\
	S(SRC_STARTUP),				\
	S(SRC_SEND_CAPABILITIES),		\
	S(SRC_SEND_CAPABILITIES_TIMEOUT),	\
	S(SRC_NEGOTIATE_CAPABILITIES),		\
	S(SRC_TRANSITION_SUPPLY),		\
	S(SRC_READY),				\
	S(SRC_WAIT_NEW_CAPABILITIES),		\
						\
	S(SNK_UNATTACHED),			\
	S(SNK_ATTACH_WAIT),			\
	S(SNK_DEBOUNCED),			\
	S(SNK_ATTACHED),			\
	S(SNK_STARTUP),				\
	S(SNK_DISCOVERY),			\
	S(SNK_DISCOVERY_DEBOUNCE),		\
	S(SNK_DISCOVERY_DEBOUNCE_DONE),		\
	S(SNK_WAIT_CAPABILITIES),		\
	S(SNK_NEGOTIATE_CAPABILITIES),		\
	S(SNK_TRANSITION_SINK),			\
	S(SNK_TRANSITION_SINK_VBUS),		\
	S(SNK_READY),				\
						\
	S(HARD_RESET_SEND),			\
	S(HARD_RESET_START),			\
	S(SRC_HARD_RESET_VBUS_OFF),		\
	S(SRC_HARD_RESET_VBUS_ON),		\
	S(SNK_HARD_RESET_SINK_OFF),		\
	S(SNK_HARD_RESET_WAIT_VBUS),		\
	S(SNK_HARD_RESET_SINK_ON),		\
						\
	S(SOFT_RESET),				\
	S(SOFT_RESET_SEND),			\
						\
	S(DR_SWAP_ACCEPT),			\
	S(DR_SWAP_CHANGE_DR),			\
						\
	S(ERROR_RECOVERY),			\
	S(PORT_RESET),				\
	S(PORT_RESET_WAIT_OFF)

#define GENERATE_ENUM(e)	e
#define GENERATE_STRING(s)	#s
#define TCPM_POLL_EVENT_TIME_OUT 2000

enum tcpm_state {
	FOREACH_STATE(GENERATE_ENUM)
};

static const char * const tcpm_states[] = {
	FOREACH_STATE(GENERATE_STRING)
};

enum pd_msg_request {
	PD_MSG_NONE = 0,
	PD_MSG_CTRL_REJECT,
	PD_MSG_CTRL_WAIT,
	PD_MSG_CTRL_NOT_SUPP,
	PD_MSG_DATA_SINK_CAP,
	PD_MSG_DATA_SOURCE_CAP,
};

struct tcpm_port {
	struct udevice *dev;
	enum typec_port_type typec_type;
	int typec_prefer_role;
	struct tcpc_dev	*tcpc;

	enum typec_role vconn_role;
	enum typec_role pwr_role;
	enum typec_data_role data_role;

	struct typec_partner *partner;

	enum typec_cc_status cc_req;
	enum typec_cc_status cc1;
	enum typec_cc_status cc2;
	enum typec_cc_polarity polarity;

	bool attached;
	bool connected;
	int poll_event_cnt;
	enum typec_port_type port_type;

	/*
	 * Set to true when vbus is greater than VSAFE5V min.
	 * Set to false when vbus falls below vSinkDisconnect max threshold.
	 */
	bool vbus_present;

	/*
	 * Set to true when vbus is less than VSAFE0V max.
	 * Set to false when vbus is greater than VSAFE0V max.
	 */
	bool vbus_vsafe0v;

	bool vbus_never_low;
	bool vbus_source;
	bool vbus_charge;

	int try_role;

	enum pd_msg_request queued_message;

	enum tcpm_state enter_state;
	enum tcpm_state prev_state;
	enum tcpm_state state;
	enum tcpm_state delayed_state;
	unsigned long delay_ms;

	bool state_machine_running;

	bool tx_complete;
	enum tcpm_transmit_status tx_status;

	unsigned int negotiated_rev;
	unsigned int message_id;
	unsigned int caps_count;
	unsigned int hard_reset_count;
	bool pd_capable;
	bool explicit_contract;
	unsigned int rx_msgid;

	/* Partner capabilities/requests */
	u32 sink_request;
	u32 source_caps[PDO_MAX_OBJECTS];
	unsigned int nr_source_caps;
	u32 sink_caps[PDO_MAX_OBJECTS];
	unsigned int nr_sink_caps;

	/*
	 * whether to wait for the Type-C device to send the DR_SWAP Message flag
	 * For Type-C device with Dual-Role Power and Dual-Role Data, the port side
	 * is used as sink + ufp, then the tcpm framework needs to wait for Type-C
	 * device to initiate DR_swap Message.
	 */
	bool wait_dr_swap_message;

	/* Local capabilities */
	u32 src_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_src_pdo;
	u32 snk_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_snk_pdo;

	unsigned int operating_snk_mw;
	bool update_sink_caps;

	/* Requested current / voltage to the port partner */
	u32 req_current_limit;
	u32 req_supply_voltage;
	/* Actual current / voltage limit of the local port */
	u32 current_limit;
	u32 supply_voltage;

	/* port belongs to a self powered device */
	bool self_powered;

	unsigned long delay_target;
};

struct pd_rx_event {
	struct tcpm_port *port;
	struct pd_message msg;
};

static const char * const pd_rev[] = {
	[PD_REV10]		= "rev1",
	[PD_REV20]		= "rev2",
	[PD_REV30]		= "rev3",
};

static bool tcpm_cc_is_sink(enum typec_cc_status cc)
{
	return ((cc) == TYPEC_CC_RP_DEF ||
		(cc) == TYPEC_CC_RP_1_5 ||
		(cc) == TYPEC_CC_RP_3_0);
}

static bool tcpm_port_is_sink(struct tcpm_port *port)
{
	return ((tcpm_cc_is_sink(port->cc1) &&
		 !tcpm_cc_is_sink(port->cc2)) ||
		(tcpm_cc_is_sink(port->cc2) &&
		 !tcpm_cc_is_sink(port->cc1)));
}

static bool tcpm_cc_is_source(enum typec_cc_status cc)
{
	return ((cc) == TYPEC_CC_RD);
}

static bool tcpm_port_is_source(struct tcpm_port *port)
{
	return ((tcpm_cc_is_source((port)->cc1) &&
		 !tcpm_cc_is_source((port)->cc2)) ||
		(tcpm_cc_is_source((port)->cc2) &&
		 !tcpm_cc_is_source((port)->cc1)));
}

static bool tcpm_try_src(struct tcpm_port *port)
{
	return ((port)->try_role == TYPEC_SOURCE &&
		(port)->port_type == TYPEC_PORT_DRP);
}

static enum tcpm_state tcpm_default_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->try_role == TYPEC_SINK)
			return SNK_UNATTACHED;
		else if (port->try_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
	} else if (port->port_type == TYPEC_PORT_SNK) {
		return SNK_UNATTACHED;
	}
	return SRC_UNATTACHED;
}

static bool tcpm_port_is_disconnected(struct tcpm_port *port)
{
	return (!port->attached && port->cc1 == TYPEC_CC_OPEN &&
		port->cc2 == TYPEC_CC_OPEN) ||
	       (port->attached && ((port->polarity == TYPEC_POLARITY_CC1 &&
				    port->cc1 == TYPEC_CC_OPEN) ||
				   (port->polarity == TYPEC_POLARITY_CC2 &&
				    port->cc2 == TYPEC_CC_OPEN)));
}

static void tcpm_set_cc(struct tcpm_port *port, enum typec_cc_status cc)
{
	dev_dbg(port->dev, "TCPM: set cc = %d\n", cc);
	port->cc_req = cc;
	port->tcpc->set_cc(port->tcpc, cc);
}

/*
 * Determine RP value to set based on maximum current supported
 * by a port if configured as source.
 * Returns CC value to report to link partner.
 */
static enum typec_cc_status tcpm_rp_cc(struct tcpm_port *port)
{
	const u32 *src_pdo = port->src_pdo;
	int nr_pdo = port->nr_src_pdo;
	int i;

	/*
	 * Search for first entry with matching voltage.
	 * It should report the maximum supported current.
	 */
	for (i = 0; i < nr_pdo; i++) {
		const u32 pdo = src_pdo[i];

		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == 5000) {
			unsigned int curr = pdo_max_current(pdo);

			if (curr >= 3000)
				return TYPEC_CC_RP_3_0;
			else if (curr >= 1500)
				return TYPEC_CC_RP_1_5;
			return TYPEC_CC_RP_DEF;
		}
	}

	return TYPEC_CC_RP_DEF;
}

static void tcpm_check_and_run_delayed_work(struct tcpm_port *port);

static int tcpm_pd_transmit(struct tcpm_port *port,
			    enum tcpm_transmit_type type,
			    const struct pd_message *msg)
{
	int ret;
	int timeout = PD_T_TCPC_TX_TIMEOUT;

	if (msg)
		dev_dbg(port->dev, "TCPM: PD TX, header: %#x\n",
			le16_to_cpu(msg->header));
	else
		dev_dbg(port->dev, "TCPM: PD TX, type: %#x\n", type);

	port->tx_complete = false;
	ret = port->tcpc->pd_transmit(port->tcpc, type, msg, port->negotiated_rev);
	if (ret < 0)
		return ret;

	while ((timeout > 0) && (!port->tx_complete)) {
		port->tcpc->poll_event(port->tcpc);
		udelay(1000);
		timeout--;
		tcpm_check_and_run_delayed_work(port);
	}

	if (!timeout) {
		dev_err(port->dev, "TCPM: PD transmit data timeout\n");
		return -ETIMEDOUT;
	}

	switch (port->tx_status) {
	case TCPC_TX_SUCCESS:
		port->message_id = (port->message_id + 1) & PD_HEADER_ID_MASK;
		break;
	case TCPC_TX_DISCARDED:
		ret = -EAGAIN;
		break;
	case TCPC_TX_FAILED:
	default:
		ret = -EIO;
		break;
	}

	return ret;
}

void tcpm_pd_transmit_complete(struct tcpm_port *port,
			       enum tcpm_transmit_status status)
{
	dev_dbg(port->dev, "TCPM: PD TX complete, status: %u\n", status);
	port->poll_event_cnt = 0;
	port->tx_status = status;
	port->tx_complete = true;
}
EXPORT_SYMBOL_GPL(tcpm_pd_transmit_complete);

static int tcpm_set_polarity(struct tcpm_port *port,
			     enum typec_cc_polarity polarity)
{
	int ret;

	dev_dbg(port->dev, "TCPM: set polarity = %d\n", polarity);

	ret = port->tcpc->set_polarity(port->tcpc, polarity);
	if (ret < 0)
		return ret;

	port->polarity = polarity;

	return 0;
}

static int tcpm_set_vconn(struct tcpm_port *port, bool enable)
{
	int ret;

	dev_dbg(port->dev, "TCPM: set vconn = %d\n", enable);

	ret = port->tcpc->set_vconn(port->tcpc, enable);
	if (!ret)
		port->vconn_role = enable ? TYPEC_SOURCE : TYPEC_SINK;

	return ret;
}

static u32 tcpm_get_current_limit(struct tcpm_port *port)
{
	enum typec_cc_status cc;
	u32 limit;

	cc = port->polarity ? port->cc2 : port->cc1;
	switch (cc) {
	case TYPEC_CC_RP_1_5:
		limit = 1500;
		break;
	case TYPEC_CC_RP_3_0:
		limit = 3000;
		break;
	case TYPEC_CC_RP_DEF:
	default:
		limit = 0;
		break;
	}

	return limit;
}

static int tcpm_set_current_limit(struct tcpm_port *port, u32 max_ma, u32 mv)
{
	int ret = -EOPNOTSUPP;

	dev_info(port->dev, "TCPM: set voltage limit = %u mV\n", mv);
	dev_info(port->dev, "TCPM: set current limit = %u mA\n", max_ma);

	port->supply_voltage = mv;
	port->current_limit = max_ma;

	return ret;
}

static int tcpm_set_attached_state(struct tcpm_port *port, bool attached)
{
	return port->tcpc->set_roles(port->tcpc, attached, port->pwr_role,
				     port->data_role);
}

static int tcpm_set_roles(struct tcpm_port *port, bool attached,
			  enum typec_role role, enum typec_data_role data)
{
	int ret;

	ret = port->tcpc->set_roles(port->tcpc, attached, role, data);
	if (ret < 0)
		return ret;

	port->pwr_role = role;
	port->data_role = data;

	return 0;
}

static int tcpm_pd_send_source_caps(struct tcpm_port *port)
{
	struct pd_message msg;
	int i;

	memset(&msg, 0, sizeof(msg));

	if (!port->nr_src_pdo) {
		/* No source capabilities defined, sink only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SOURCE_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  port->nr_src_pdo);
	}

	for (i = 0; i < port->nr_src_pdo; i++)
		msg.payload[i] = cpu_to_le32(port->src_pdo[i]);

	return tcpm_pd_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_pd_send_sink_caps(struct tcpm_port *port)
{
	struct pd_message msg;
	unsigned int i;

	memset(&msg, 0, sizeof(msg));

	if (!port->nr_snk_pdo) {
		/* No sink capabilities defined, source only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SINK_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  port->nr_snk_pdo);
	}

	for (i = 0; i < port->nr_snk_pdo; i++)
		msg.payload[i] = cpu_to_le32(port->snk_pdo[i]);

	return tcpm_pd_transmit(port, TCPC_TX_SOP, &msg);
}

static void tcpm_state_machine(struct tcpm_port *port);

static void tcpm_timer_uninit(struct tcpm_port *port)
{
	port->delay_target = 0;
}

static void tcpm_timer_init(struct tcpm_port *port, uint32_t ms)
{
	unsigned long time_us = ms * 1000;

	port->delay_target = timer_get_us() + time_us;
}

static void tcpm_check_and_run_delayed_work(struct tcpm_port *port)
{
	/* no delayed state changes scheduled */
	if (port->delay_target == 0)
		return;

	/* it's not yet time */
	if (timer_get_us() < port->delay_target)
		return;

	tcpm_timer_uninit(port);
	tcpm_state_machine(port);
}

static void mod_tcpm_delayed_work(struct tcpm_port *port, unsigned int delay_ms)
{
	if (delay_ms) {
		tcpm_timer_init(port, delay_ms);
	} else {
		tcpm_timer_uninit(port);
		tcpm_state_machine(port);
	}
}

static void tcpm_set_state(struct tcpm_port *port, enum tcpm_state state,
			   unsigned int delay_ms)
{
	if (delay_ms) {
		dev_dbg(port->dev, "TCPM: pending state change %s -> %s @ %u ms [%s]\n",
			tcpm_states[port->state], tcpm_states[state], delay_ms,
			pd_rev[port->negotiated_rev]);
		port->delayed_state = state;
		mod_tcpm_delayed_work(port, delay_ms);
		port->delay_ms = delay_ms;
	} else {
		dev_dbg(port->dev, "TCPM: state change %s -> %s\n",
			tcpm_states[port->state], tcpm_states[state]);
		port->delayed_state = INVALID_STATE;
		port->prev_state = port->state;
		port->state = state;
		/*
		 * Don't re-queue the state machine work item if we're currently
		 * in the state machine and we're immediately changing states.
		 * tcpm_state_machine_work() will continue running the state
		 * machine.
		 */
		if (!port->state_machine_running)
			mod_tcpm_delayed_work(port, 0);
	}
}

static void tcpm_set_state_cond(struct tcpm_port *port, enum tcpm_state state,
				unsigned int delay_ms)
{
	if (port->enter_state == port->state)
		tcpm_set_state(port, state, delay_ms);
	else
		dev_dbg(port->dev, "TCPM: skipped %sstate change %s -> %s [%u ms], context state %s [%s]\n",
			delay_ms ? "delayed " : "",
			tcpm_states[port->state], tcpm_states[state],
			delay_ms, tcpm_states[port->enter_state],
			pd_rev[port->negotiated_rev]);
}

static void tcpm_queue_message(struct tcpm_port *port,
			       enum pd_msg_request message)
{
	port->queued_message = message;
	mod_tcpm_delayed_work(port, 0);
}

enum pdo_err {
	PDO_NO_ERR,
	PDO_ERR_NO_VSAFE5V,
	PDO_ERR_VSAFE5V_NOT_FIRST,
	PDO_ERR_PDO_TYPE_NOT_IN_ORDER,
	PDO_ERR_FIXED_NOT_SORTED,
	PDO_ERR_VARIABLE_BATT_NOT_SORTED,
	PDO_ERR_DUPE_PDO,
	PDO_ERR_PPS_APDO_NOT_SORTED,
	PDO_ERR_DUPE_PPS_APDO,
};

static const char * const pdo_err_msg[] = {
	[PDO_ERR_NO_VSAFE5V] =
	" err: source/sink caps should at least have vSafe5V",
	[PDO_ERR_VSAFE5V_NOT_FIRST] =
	" err: vSafe5V Fixed Supply Object Shall always be the first object",
	[PDO_ERR_PDO_TYPE_NOT_IN_ORDER] =
	" err: PDOs should be in the following order: Fixed; Battery; Variable",
	[PDO_ERR_FIXED_NOT_SORTED] =
	" err: Fixed supply pdos should be in increasing order of their fixed voltage",
	[PDO_ERR_VARIABLE_BATT_NOT_SORTED] =
	" err: Variable/Battery supply pdos should be in increasing order of their minimum voltage",
	[PDO_ERR_DUPE_PDO] =
	" err: Variable/Batt supply pdos cannot have same min/max voltage",
	[PDO_ERR_PPS_APDO_NOT_SORTED] =
	" err: Programmable power supply apdos should be in increasing order of their maximum voltage",
	[PDO_ERR_DUPE_PPS_APDO] =
	" err: Programmable power supply apdos cannot have same min/max voltage and max current",
};

static enum pdo_err tcpm_caps_err(struct tcpm_port *port, const u32 *pdo,
				  unsigned int nr_pdo)
{
	unsigned int i;

	/* Should at least contain vSafe5v */
	if (nr_pdo < 1)
		return PDO_ERR_NO_VSAFE5V;

	/* The vSafe5V Fixed Supply Object Shall always be the first object */
	if (pdo_type(pdo[0]) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(pdo[0]) != VSAFE5V)
		return PDO_ERR_VSAFE5V_NOT_FIRST;

	for (i = 1; i < nr_pdo; i++) {
		if (pdo_type(pdo[i]) < pdo_type(pdo[i - 1])) {
			return PDO_ERR_PDO_TYPE_NOT_IN_ORDER;
		} else if (pdo_type(pdo[i]) == pdo_type(pdo[i - 1])) {
			enum pd_pdo_type type = pdo_type(pdo[i]);

			switch (type) {
			/*
			 * The remaining Fixed Supply Objects, if
			 * present, shall be sent in voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_FIXED:
				if (pdo_fixed_voltage(pdo[i]) <=
				    pdo_fixed_voltage(pdo[i - 1]))
					return PDO_ERR_FIXED_NOT_SORTED;
				break;
			/*
			 * The Battery Supply Objects and Variable
			 * supply, if present shall be sent in Minimum
			 * Voltage order; lowest to highest.
			 */
			case PDO_TYPE_VAR:
			case PDO_TYPE_BATT:
				if (pdo_min_voltage(pdo[i]) <
				    pdo_min_voltage(pdo[i - 1]))
					return PDO_ERR_VARIABLE_BATT_NOT_SORTED;
				else if ((pdo_min_voltage(pdo[i]) ==
					  pdo_min_voltage(pdo[i - 1])) &&
					 (pdo_max_voltage(pdo[i]) ==
					  pdo_max_voltage(pdo[i - 1])))
					return PDO_ERR_DUPE_PDO;
				break;
			/*
			 * The Programmable Power Supply APDOs, if present,
			 * shall be sent in Maximum Voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_APDO:
				if (pdo_apdo_type(pdo[i]) != APDO_TYPE_PPS)
					break;

				if (pdo_pps_apdo_max_voltage(pdo[i]) <
				    pdo_pps_apdo_max_voltage(pdo[i - 1]))
					return PDO_ERR_PPS_APDO_NOT_SORTED;
				else if (pdo_pps_apdo_min_voltage(pdo[i]) ==
					  pdo_pps_apdo_min_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_voltage(pdo[i]) ==
					  pdo_pps_apdo_max_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_current(pdo[i]) ==
					  pdo_pps_apdo_max_current(pdo[i - 1]))
					return PDO_ERR_DUPE_PPS_APDO;
				break;
			default:
				dev_err(port->dev, "TCPM: Unknown pdo type\n");
			}
		}
	}

	return PDO_NO_ERR;
}

static int tcpm_validate_caps(struct tcpm_port *port, const u32 *pdo,
			      unsigned int nr_pdo)
{
	enum pdo_err err_index = tcpm_caps_err(port, pdo, nr_pdo);

	if (err_index != PDO_NO_ERR) {
		dev_err(port->dev, "TCPM:%s\n", pdo_err_msg[err_index]);
		return -EINVAL;
	}

	return 0;
}

/*
 * PD (data, control) command handling functions
 */
static inline enum tcpm_state ready_state(struct tcpm_port *port)
{
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_READY;
	else
		return SNK_READY;
}

static int tcpm_pd_send_control(struct tcpm_port *port,
				enum pd_ctrl_msg_type type);

static void tcpm_pd_data_request(struct tcpm_port *port,
				 const struct pd_message *msg)
{
	enum pd_data_msg_type type = pd_header_type_le(msg->header);
	unsigned int cnt = pd_header_cnt_le(msg->header);
	unsigned int rev = pd_header_rev_le(msg->header);
	unsigned int i;

	switch (type) {
	case PD_DATA_SOURCE_CAP:
		for (i = 0; i < cnt; i++)
			port->source_caps[i] = le32_to_cpu(msg->payload[i]);

		port->nr_source_caps = cnt;

		tcpm_validate_caps(port, port->source_caps,
				   port->nr_source_caps);

		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just do nothing in that scenario.
		 */
		if (rev == PD_REV10)
			break;

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		if ((pdo_type(port->source_caps[0]) == PDO_TYPE_FIXED) &&
		    (port->source_caps[0] & PDO_FIXED_DUAL_ROLE) &&
		    (port->source_caps[0] & PDO_FIXED_DATA_SWAP)) {
			/* Dual role power and data, eg: self-powered Type-C */
			port->wait_dr_swap_message = true;
		} else {
			/* Non-Dual role power, eg: adapter */
			port->wait_dr_swap_message = false;
		}

		/*
		 * This message may be received even if VBUS is not
		 * present. This is quite unexpected; see USB PD
		 * specification, sections 8.3.3.6.3.1 and 8.3.3.6.3.2.
		 * However, at the same time, we must be ready to
		 * receive this message and respond to it 15ms after
		 * receiving PS_RDY during power swap operations, no matter
		 * if VBUS is available or not (USB PD specification,
		 * section 6.5.9.2).
		 * So we need to accept the message either way,
		 * but be prepared to keep waiting for VBUS after it was
		 * handled.
		 */
		tcpm_set_state(port, SNK_NEGOTIATE_CAPABILITIES, 0);
		break;
	case PD_DATA_REQUEST:
		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just reject in that scenario.
		 */
		if (rev == PD_REV10) {
			tcpm_queue_message(port, PD_MSG_CTRL_REJECT);
			break;
		}

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		port->sink_request = le32_to_cpu(msg->payload[0]);

		tcpm_set_state(port, SRC_NEGOTIATE_CAPABILITIES, 0);
		break;
	case PD_DATA_SINK_CAP:
		/* We don't do anything with this at the moment... */
		for (i = 0; i < cnt; i++)
			port->sink_caps[i] = le32_to_cpu(msg->payload[i]);

		port->nr_sink_caps = cnt;
		break;
	default:
		break;
	}
}

static void tcpm_pd_ctrl_request(struct tcpm_port *port,
				 const struct pd_message *msg)
{
	enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
	enum tcpm_state next_state;

	switch (type) {
	case PD_CTRL_GOOD_CRC:
	case PD_CTRL_PING:
		break;
	case PD_CTRL_GET_SOURCE_CAP:
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_queue_message(port, PD_MSG_DATA_SOURCE_CAP);
			break;
		default:
			tcpm_queue_message(port, PD_MSG_CTRL_REJECT);
			break;
		}
		break;
	case PD_CTRL_GET_SINK_CAP:
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_queue_message(port, PD_MSG_DATA_SINK_CAP);
			break;
		default:
			tcpm_queue_message(port, PD_MSG_CTRL_REJECT);
			break;
		}
		break;
	case PD_CTRL_GOTO_MIN:
		break;
	case PD_CTRL_PS_RDY:
		switch (port->state) {
		case SNK_TRANSITION_SINK:
			if (port->vbus_present) {
				tcpm_set_current_limit(port,
						       port->req_current_limit,
						       port->req_supply_voltage);
				port->explicit_contract = true;
				tcpm_set_state(port, SNK_READY, 0);
			} else {
				/*
				 * Seen after power swap. Keep waiting for VBUS
				 * in a transitional state.
				 */
				tcpm_set_state(port,
					       SNK_TRANSITION_SINK_VBUS, 0);
			}
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_REJECT:
	case PD_CTRL_WAIT:
	case PD_CTRL_NOT_SUPP:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			/* USB PD specification, Figure 8-43 */
			if (port->explicit_contract)
				next_state = SNK_READY;
			else
				next_state = SNK_WAIT_CAPABILITIES;

			tcpm_set_state(port, next_state, 0);
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_ACCEPT:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			tcpm_set_state(port, SNK_TRANSITION_SINK, 0);
			break;
		case SOFT_RESET_SEND:
			port->message_id = 0;
			port->rx_msgid = -1;
			if (port->pwr_role == TYPEC_SOURCE)
				next_state = SRC_SEND_CAPABILITIES;
			else
				next_state = SNK_WAIT_CAPABILITIES;
			tcpm_set_state(port, next_state, 0);
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_SOFT_RESET:
		tcpm_set_state(port, SOFT_RESET, 0);
		break;
	case PD_CTRL_DR_SWAP:
		if (port->port_type != TYPEC_PORT_DRP) {
			tcpm_queue_message(port, PD_MSG_CTRL_REJECT);
			break;
		}
		/*
		 * XXX
		 * 6.3.9: If an alternate mode is active, a request to swap
		 * alternate modes shall trigger a port reset.
		 */
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_set_state(port, DR_SWAP_ACCEPT, 0);
			break;
		default:
			tcpm_queue_message(port, PD_MSG_CTRL_WAIT);
			break;
		}
		break;
	case PD_CTRL_PR_SWAP:
	case PD_CTRL_VCONN_SWAP:
	case PD_CTRL_GET_SOURCE_CAP_EXT:
	case PD_CTRL_GET_STATUS:
	case PD_CTRL_FR_SWAP:
	case PD_CTRL_GET_PPS_STATUS:
	case PD_CTRL_GET_COUNTRY_CODES:
		/* Currently not supported */
		dev_err(port->dev, "TCPM: Currently not supported type %#x\n", type);
		tcpm_queue_message(port, PD_MSG_CTRL_NOT_SUPP);
		break;
	default:
		dev_err(port->dev, "TCPM: Unrecognized ctrl message type %#x\n", type);
		break;
	}
}

static void tcpm_pd_rx_handler(struct tcpm_port *port,
			       struct pd_rx_event *event)
{
	const struct pd_message *msg = &event->msg;
	unsigned int cnt = pd_header_cnt_le(msg->header);
	bool remote_is_host, local_is_host;

	dev_dbg(port->dev, "TCPM: PD RX, header: %#x [%d]\n",
		le16_to_cpu(msg->header), port->attached);

	if (port->attached) {
		enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
		unsigned int msgid = pd_header_msgid_le(msg->header);

		/*
		 * USB PD standard, 6.6.1.2:
		 * "... if MessageID value in a received Message is the
		 * same as the stored value, the receiver shall return a
		 * GoodCRC Message with that MessageID value and drop
		 * the Message (this is a retry of an already received
		 * Message). Note: this shall not apply to the Soft_Reset
		 * Message which always has a MessageID value of zero."
		 */
		if (msgid == port->rx_msgid && type != PD_CTRL_SOFT_RESET)
			goto done;
		port->rx_msgid = msgid;

		/*
		 * If both ends believe to be DFP/host, we have a data role
		 * mismatch.
		 */
		remote_is_host = !!(le16_to_cpu(msg->header) & PD_HEADER_DATA_ROLE);
		local_is_host = port->data_role == TYPEC_HOST;
		if (remote_is_host == local_is_host) {
			dev_err(port->dev, "TCPM: data role mismatch, initiating error recovery\n");
			tcpm_set_state(port, ERROR_RECOVERY, 0);
		} else {
			if (cnt)
				tcpm_pd_data_request(port, msg);
			else
				tcpm_pd_ctrl_request(port, msg);
		}
	}

done:
	kfree(event);
}

void tcpm_pd_receive(struct tcpm_port *port, const struct pd_message *msg)
{
	struct pd_rx_event *event;

	port->poll_event_cnt = 0;
	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (!event)
		return;

	event->port = port;
	memcpy(&event->msg, msg, sizeof(*msg));
	tcpm_pd_rx_handler(port, event);
}
EXPORT_SYMBOL_GPL(tcpm_pd_receive);

static int tcpm_pd_send_control(struct tcpm_port *port,
				enum pd_ctrl_msg_type type)
{
	struct pd_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(type, port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 0);

	return tcpm_pd_transmit(port, TCPC_TX_SOP, &msg);
}

/*
 * Send queued message without affecting state.
 * Return true if state machine should go back to sleep,
 * false otherwise.
 */
static bool tcpm_send_queued_message(struct tcpm_port *port)
{
	enum pd_msg_request queued_message;

	do {
		queued_message = port->queued_message;
		port->queued_message = PD_MSG_NONE;

		switch (queued_message) {
		case PD_MSG_CTRL_WAIT:
			tcpm_pd_send_control(port, PD_CTRL_WAIT);
			break;
		case PD_MSG_CTRL_REJECT:
			tcpm_pd_send_control(port, PD_CTRL_REJECT);
			break;
		case PD_MSG_CTRL_NOT_SUPP:
			tcpm_pd_send_control(port, PD_CTRL_NOT_SUPP);
			break;
		case PD_MSG_DATA_SINK_CAP:
			tcpm_pd_send_sink_caps(port);
			break;
		case PD_MSG_DATA_SOURCE_CAP:
			tcpm_pd_send_source_caps(port);
			break;
		default:
			break;
		}
	} while (port->queued_message != PD_MSG_NONE);

	return false;
}

static int tcpm_pd_check_request(struct tcpm_port *port)
{
	u32 pdo, rdo = port->sink_request;
	unsigned int max, op, pdo_max, index;
	enum pd_pdo_type type;

	index = rdo_index(rdo);
	if (!index || index > port->nr_src_pdo)
		return -EINVAL;

	pdo = port->src_pdo[index - 1];
	type = pdo_type(pdo);
	switch (type) {
	case PDO_TYPE_FIXED:
	case PDO_TYPE_VAR:
		max = rdo_max_current(rdo);
		op = rdo_op_current(rdo);
		pdo_max = pdo_max_current(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;

		if (type == PDO_TYPE_FIXED)
			dev_dbg(port->dev, "TCPM: Requested %u mV, %u mA for %u / %u mA\n",
				pdo_fixed_voltage(pdo), pdo_max, op, max);
		else
			dev_dbg(port->dev, "TCPM: Requested %u -> %u mV, %u mA for %u / %u mA\n",
				pdo_min_voltage(pdo), pdo_max_voltage(pdo),
				pdo_max, op, max);
		break;
	case PDO_TYPE_BATT:
		max = rdo_max_power(rdo);
		op = rdo_op_power(rdo);
		pdo_max = pdo_max_power(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;
		dev_info(port->dev, "TCPM: Requested %u -> %u mV, %u mW for %u / %u mW\n",
			 pdo_min_voltage(pdo), pdo_max_voltage(pdo),
			 pdo_max, op, max);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define min_power(x, y) min(pdo_max_power(x), pdo_max_power(y))
#define min_current(x, y) min(pdo_max_current(x), pdo_max_current(y))

static int tcpm_pd_select_pdo(struct tcpm_port *port, int *sink_pdo,
			      int *src_pdo)
{
	unsigned int i, j, max_src_mv = 0, min_src_mv = 0, max_mw = 0,
		     max_mv = 0, src_mw = 0, src_ma = 0, max_snk_mv = 0,
		     min_snk_mv = 0;
	int ret = -EINVAL;

	/*
	 * Select the source PDO providing the most power which has a
	 * matchig sink cap.
	 */
	for (i = 0; i < port->nr_source_caps; i++) {
		u32 pdo = port->source_caps[i];
		enum pd_pdo_type type = pdo_type(pdo);

		switch (type) {
		case PDO_TYPE_FIXED:
			max_src_mv = pdo_fixed_voltage(pdo);
			min_src_mv = max_src_mv;
			break;
		case PDO_TYPE_BATT:
		case PDO_TYPE_VAR:
			max_src_mv = pdo_max_voltage(pdo);
			min_src_mv = pdo_min_voltage(pdo);
			break;
		case PDO_TYPE_APDO:
			continue;
		default:
			dev_err(port->dev, "TCPM: Invalid source PDO type, ignoring\n");
			continue;
		}

		switch (type) {
		case PDO_TYPE_FIXED:
		case PDO_TYPE_VAR:
			src_ma = pdo_max_current(pdo);
			src_mw = src_ma * min_src_mv / 1000;
			break;
		case PDO_TYPE_BATT:
			src_mw = pdo_max_power(pdo);
			break;
		case PDO_TYPE_APDO:
			continue;
		default:
			dev_err(port->dev, "TCPM: Invalid source PDO type, ignoring\n");
			continue;
		}

		for (j = 0; j < port->nr_snk_pdo; j++) {
			pdo = port->snk_pdo[j];

			switch (pdo_type(pdo)) {
			case PDO_TYPE_FIXED:
				max_snk_mv = pdo_fixed_voltage(pdo);
				min_snk_mv = max_snk_mv;
				break;
			case PDO_TYPE_BATT:
			case PDO_TYPE_VAR:
				max_snk_mv = pdo_max_voltage(pdo);
				min_snk_mv = pdo_min_voltage(pdo);
				break;
			case PDO_TYPE_APDO:
				continue;
			default:
				dev_err(port->dev, "TCPM: Invalid sink PDO type, ignoring\n");
				continue;
			}

			if (max_src_mv <= max_snk_mv && min_src_mv >= min_snk_mv) {
				/* Prefer higher voltages if available */
				if ((src_mw == max_mw && min_src_mv > max_mv) ||
				    src_mw > max_mw) {
					*src_pdo = i;
					*sink_pdo = j;
					max_mw = src_mw;
					max_mv = min_src_mv;
					ret = 0;
				}
			}
		}
	}

	return ret;
}

static int tcpm_pd_build_request(struct tcpm_port *port, u32 *rdo)
{
	unsigned int mv, ma, mw, flags;
	unsigned int max_ma, max_mw;
	enum pd_pdo_type type;
	u32 pdo, matching_snk_pdo;
	int src_pdo_index = 0;
	int snk_pdo_index = 0;
	int ret;

	ret = tcpm_pd_select_pdo(port, &snk_pdo_index, &src_pdo_index);
	if (ret < 0)
		return ret;

	pdo = port->source_caps[src_pdo_index];
	matching_snk_pdo = port->snk_pdo[snk_pdo_index];
	type = pdo_type(pdo);

	switch (type) {
	case PDO_TYPE_FIXED:
		mv = pdo_fixed_voltage(pdo);
		break;
	case PDO_TYPE_BATT:
	case PDO_TYPE_VAR:
		mv = pdo_min_voltage(pdo);
		break;
	default:
		dev_err(port->dev, "TCPM: Invalid PDO selected!\n");
		return -EINVAL;
	}

	/* Select maximum available current within the sink pdo's limit */
	if (type == PDO_TYPE_BATT) {
		mw = min_power(pdo, matching_snk_pdo);
		ma = 1000 * mw / mv;
	} else {
		ma = min_current(pdo, matching_snk_pdo);
		mw = ma * mv / 1000;
	}

	flags = RDO_USB_COMM | RDO_NO_SUSPEND;

	/* Set mismatch bit if offered power is less than operating power */
	max_ma = ma;
	max_mw = mw;
	if (mw < port->operating_snk_mw) {
		flags |= RDO_CAP_MISMATCH;
		if (type == PDO_TYPE_BATT &&
		    (pdo_max_power(matching_snk_pdo) > pdo_max_power(pdo)))
			max_mw = pdo_max_power(matching_snk_pdo);
		else if (pdo_max_current(matching_snk_pdo) >
			 pdo_max_current(pdo))
			max_ma = pdo_max_current(matching_snk_pdo);
	}

	dev_dbg(port->dev, "TCPM: cc=%d cc1=%d cc2=%d vbus=%d vconn=%s polarity=%d\n",
		port->cc_req, port->cc1, port->cc2, port->vbus_source,
		port->vconn_role == TYPEC_SOURCE ? "source" : "sink",
		port->polarity);

	if (type == PDO_TYPE_BATT) {
		*rdo = RDO_BATT(src_pdo_index + 1, mw, max_mw, flags);

		dev_info(port->dev, "TCPM: requesting PDO %d: %u mV, %u mW%s\n",
			 src_pdo_index, mv, mw,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	} else {
		*rdo = RDO_FIXED(src_pdo_index + 1, ma, max_ma, flags);

		dev_info(port->dev, "TCPM: requesting PDO %d: %u mV, %u mA%s\n",
			 src_pdo_index, mv, ma,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	}

	port->req_current_limit = ma;
	port->req_supply_voltage = mv;

	return 0;
}

static int tcpm_pd_send_request(struct tcpm_port *port)
{
	struct pd_message msg;
	int ret;
	u32 rdo;

	ret = tcpm_pd_build_request(port, &rdo);
	if (ret < 0)
		return ret;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(PD_DATA_REQUEST,
				  port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 1);
	msg.payload[0] = cpu_to_le32(rdo);

	return tcpm_pd_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_set_vbus(struct tcpm_port *port, bool enable)
{
	int ret;

	if (enable && port->vbus_charge)
		return -EINVAL;

	dev_dbg(port->dev, "TCPM: set vbus = %d charge = %d\n",
		enable, port->vbus_charge);

	ret = port->tcpc->set_vbus(port->tcpc, enable, port->vbus_charge);
	if (ret < 0)
		return ret;

	port->vbus_source = enable;
	return 0;
}

static int tcpm_set_charge(struct tcpm_port *port, bool charge)
{
	int ret;

	if (charge && port->vbus_source)
		return -EINVAL;

	if (charge != port->vbus_charge) {
		dev_dbg(port->dev, "TCPM: set vbus = %d charge = %d\n",
			port->vbus_source, charge);
		ret = port->tcpc->set_vbus(port->tcpc, port->vbus_source,
					   charge);
		if (ret < 0)
			return ret;
	}
	port->vbus_charge = charge;
	return 0;
}

static bool tcpm_start_toggling(struct tcpm_port *port, enum typec_cc_status cc)
{
	int ret;

	if (!port->tcpc->start_toggling)
		return false;

	dev_dbg(port->dev, "TCPM: Start toggling\n");
	ret = port->tcpc->start_toggling(port->tcpc, port->port_type, cc);
	return ret == 0;
}

static int tcpm_init_vbus(struct tcpm_port *port)
{
	int ret;

	ret = port->tcpc->set_vbus(port->tcpc, false, false);
	port->vbus_source = false;
	port->vbus_charge = false;
	return ret;
}

static int tcpm_init_vconn(struct tcpm_port *port)
{
	int ret;

	ret = port->tcpc->set_vconn(port->tcpc, false);
	port->vconn_role = TYPEC_SINK;
	return ret;
}

static void tcpm_typec_connect(struct tcpm_port *port)
{
	if (!port->connected)
		port->connected = true;
}

static int tcpm_src_attach(struct tcpm_port *port)
{
	enum typec_cc_polarity polarity =
				port->cc2 == TYPEC_CC_RD ? TYPEC_POLARITY_CC2
							 : TYPEC_POLARITY_CC1;
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(port, polarity);
	if (ret < 0)
		return ret;

	ret = tcpm_set_roles(port, true, TYPEC_SOURCE, TYPEC_HOST);
	if (ret < 0)
		return ret;

	ret = port->tcpc->set_pd_rx(port->tcpc, true);
	if (ret < 0)
		goto out_disable_mux;

	/*
	 * USB Type-C specification, version 1.2,
	 * chapter 4.5.2.2.8.1 (Attached.SRC Requirements)
	 * Enable VCONN only if the non-RD port is set to RA.
	 */
	if ((polarity == TYPEC_POLARITY_CC1 && port->cc2 == TYPEC_CC_RA) ||
	    (polarity == TYPEC_POLARITY_CC2 && port->cc1 == TYPEC_CC_RA)) {
		ret = tcpm_set_vconn(port, true);
		if (ret < 0)
			goto out_disable_pd;
	}

	ret = tcpm_set_vbus(port, true);
	if (ret < 0)
		goto out_disable_vconn;

	port->pd_capable = false;

	port->partner = NULL;

	port->attached = true;

	return 0;

out_disable_vconn:
	tcpm_set_vconn(port, false);
out_disable_pd:
	port->tcpc->set_pd_rx(port->tcpc, false);
out_disable_mux:
	dev_err(port->dev, "TCPM: CC connected in %s as DFP\n",
		polarity ? "CC2" : "CC1");
	return 0;
}

static void tcpm_typec_disconnect(struct tcpm_port *port)
{
	if (port->connected) {
		port->partner = NULL;
		port->connected = false;
	}
}

static void tcpm_reset_port(struct tcpm_port *port)
{
	tcpm_timer_uninit(port);
	tcpm_typec_disconnect(port);
	port->poll_event_cnt = 0;
	port->wait_dr_swap_message = false;
	port->attached = false;
	port->pd_capable = false;

	/*
	 * First Rx ID should be 0; set this to a sentinel of -1 so that
	 * we can check tcpm_pd_rx_handler() if we had seen it before.
	 */
	port->rx_msgid = -1;

	port->tcpc->set_pd_rx(port->tcpc, false);
	tcpm_init_vbus(port);	/* also disables charging */
	tcpm_init_vconn(port);
	tcpm_set_current_limit(port, 0, 0);
	tcpm_set_polarity(port, TYPEC_POLARITY_CC1);
	tcpm_set_attached_state(port, false);
	port->nr_sink_caps = 0;
}

static void tcpm_detach(struct tcpm_port *port)
{
	if (tcpm_port_is_disconnected(port))
		port->hard_reset_count = 0;

	if (!port->attached)
		return;

	tcpm_reset_port(port);
}

static void tcpm_src_detach(struct tcpm_port *port)
{
	tcpm_detach(port);
}

static int tcpm_snk_attach(struct tcpm_port *port)
{
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(port, port->cc2 != TYPEC_CC_OPEN ?
				TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1);
	if (ret < 0)
		return ret;

	ret = tcpm_set_roles(port, true, TYPEC_SINK, TYPEC_DEVICE);
	if (ret < 0)
		return ret;

	port->pd_capable = false;

	port->partner = NULL;

	port->attached = true;
	dev_info(port->dev, "TCPM: CC connected in %s as UFP\n",
		 port->cc1 != TYPEC_CC_OPEN ? "CC1" : "CC2");

	return 0;
}

static void tcpm_snk_detach(struct tcpm_port *port)
{
	tcpm_detach(port);
}

static inline enum tcpm_state hard_reset_state(struct tcpm_port *port)
{
	if (port->hard_reset_count < PD_N_HARD_RESET_COUNT)
		return HARD_RESET_SEND;
	if (port->pd_capable)
		return ERROR_RECOVERY;
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_UNATTACHED;
	if (port->state == SNK_WAIT_CAPABILITIES)
		return SNK_READY;
	return SNK_UNATTACHED;
}

static inline enum tcpm_state unattached_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->pwr_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
		else
			return SNK_UNATTACHED;
	} else if (port->port_type == TYPEC_PORT_SRC) {
		return SRC_UNATTACHED;
	}

	return SNK_UNATTACHED;
}

static void run_state_machine(struct tcpm_port *port)
{
	int ret;

	port->enter_state = port->state;
	switch (port->state) {
	case TOGGLING:
		break;
	/* SRC states */
	case SRC_UNATTACHED:
		tcpm_src_detach(port);
		if (tcpm_start_toggling(port, tcpm_rp_cc(port))) {
			tcpm_set_state(port, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(port, tcpm_rp_cc(port));
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(port, SNK_UNATTACHED, PD_T_DRP_SNK);
		break;
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_ATTACHED, PD_T_CC_DEBOUNCE);
		break;

	case SRC_ATTACHED:
		ret = tcpm_src_attach(port);
		/*
		 * Currently, vbus control is not implemented,
		 * and the SRC detection process cannot be fully implemented.
		 */
		tcpm_set_state(port, SRC_READY, 0);
		break;
	case SRC_STARTUP:
		port->caps_count = 0;
		port->negotiated_rev = PD_MAX_REV;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;
		tcpm_set_state(port, SRC_SEND_CAPABILITIES, 0);
		break;
	case SRC_SEND_CAPABILITIES:
		port->caps_count++;
		if (port->caps_count > PD_N_CAPS_COUNT) {
			tcpm_set_state(port, SRC_READY, 0);
			break;
		}
		ret = tcpm_pd_send_source_caps(port);
		if (ret < 0) {
			tcpm_set_state(port, SRC_SEND_CAPABILITIES,
				       PD_T_SEND_SOURCE_CAP);
		} else {
			/*
			 * Per standard, we should clear the reset counter here.
			 * However, that can result in state machine hang-ups.
			 * Reset it only in READY state to improve stability.
			 */
			/* port->hard_reset_count = 0; */
			port->caps_count = 0;
			port->pd_capable = true;
			tcpm_set_state_cond(port, SRC_SEND_CAPABILITIES_TIMEOUT,
					    PD_T_SEND_SOURCE_CAP);
		}
		break;
	case SRC_SEND_CAPABILITIES_TIMEOUT:
		/*
		 * Error recovery for a PD_DATA_SOURCE_CAP reply timeout.
		 *
		 * PD 2.0 sinks are supposed to accept src-capabilities with a
		 * 3.0 header and simply ignore any src PDOs which the sink does
		 * not understand such as PPS but some 2.0 sinks instead ignore
		 * the entire PD_DATA_SOURCE_CAP message, causing contract
		 * negotiation to fail.
		 *
		 * After PD_N_HARD_RESET_COUNT hard-reset attempts, we try
		 * sending src-capabilities with a lower PD revision to
		 * make these broken sinks work.
		 */
		if (port->hard_reset_count < PD_N_HARD_RESET_COUNT) {
			tcpm_set_state(port, HARD_RESET_SEND, 0);
		} else if (port->negotiated_rev > PD_REV20) {
			port->negotiated_rev--;
			port->hard_reset_count = 0;
			tcpm_set_state(port, SRC_SEND_CAPABILITIES, 0);
		} else {
			tcpm_set_state(port, hard_reset_state(port), 0);
		}
		break;
	case SRC_NEGOTIATE_CAPABILITIES:
		ret = tcpm_pd_check_request(port);
		if (ret < 0) {
			tcpm_pd_send_control(port, PD_CTRL_REJECT);
			if (!port->explicit_contract) {
				tcpm_set_state(port,
					       SRC_WAIT_NEW_CAPABILITIES, 0);
			} else {
				tcpm_set_state(port, SRC_READY, 0);
			}
		} else {
			tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
			tcpm_set_state(port, SRC_TRANSITION_SUPPLY,
				       PD_T_SRC_TRANSITION);
		}
		break;
	case SRC_TRANSITION_SUPPLY:
		/* XXX: regulator_set_voltage(vbus, ...) */
		tcpm_pd_send_control(port, PD_CTRL_PS_RDY);
		port->explicit_contract = true;
		tcpm_set_state_cond(port, SRC_READY, 0);
		break;
	case SRC_READY:
		port->hard_reset_count = 0;

		tcpm_typec_connect(port);
		break;
	case SRC_WAIT_NEW_CAPABILITIES:
		/* Nothing to do... */
		break;

	/* SNK states */
	case SNK_UNATTACHED:
		tcpm_snk_detach(port);
		if (tcpm_start_toggling(port, TYPEC_CC_RD)) {
			tcpm_set_state(port, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(port, TYPEC_CC_RD);
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(port, SRC_UNATTACHED, PD_T_DRP_SRC);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			tcpm_set_state(port, SNK_DEBOUNCED,
				       PD_T_CC_DEBOUNCE);
		else if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_UNATTACHED,
				       PD_T_CC_DEBOUNCE);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_UNATTACHED, PD_T_PD_DEBOUNCE);
		else if (port->vbus_present)
			tcpm_set_state(port, SNK_ATTACHED, 0);
		else
			/* Wait for VBUS, but not forever */
			tcpm_set_state(port, PORT_RESET, PD_T_PS_SOURCE_ON);
		break;
	case SNK_ATTACHED:
		ret = tcpm_snk_attach(port);
		if (ret < 0)
			tcpm_set_state(port, SNK_UNATTACHED, 0);
		else
			tcpm_set_state(port, SNK_STARTUP, 0);
		break;
	case SNK_STARTUP:
		port->negotiated_rev = PD_MAX_REV;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;
		tcpm_set_state(port, SNK_DISCOVERY, 0);
		break;
	case SNK_DISCOVERY:
		if (port->vbus_present) {
			tcpm_set_current_limit(port,
					       tcpm_get_current_limit(port),
					       5000);
			tcpm_set_charge(port, true);
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
			break;
		}
		/*
		 * For DRP, timeouts differ. Also, handling is supposed to be
		 * different and much more complex (dead battery detection;
		 * see USB power delivery specification, section 8.3.3.6.1.5.1).
		 */
		tcpm_set_state(port, hard_reset_state(port),
			       port->port_type == TYPEC_PORT_DRP ?
					PD_T_DB_DETECT : PD_T_NO_RESPONSE);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		tcpm_set_state(port, SNK_DISCOVERY_DEBOUNCE_DONE,
			       PD_T_CC_DEBOUNCE);
		break;
	case SNK_DISCOVERY_DEBOUNCE_DONE:
		tcpm_set_state(port, unattached_state(port), 0);
		break;
	case SNK_WAIT_CAPABILITIES:
		ret = port->tcpc->set_pd_rx(port->tcpc, true);
		if (ret < 0) {
			tcpm_set_state(port, SNK_READY, 0);
			break;
		}
		/*
		 * If VBUS has never been low, and we time out waiting
		 * for source cap, try a soft reset first, in case we
		 * were already in a stable contract before this boot.
		 * Do this only once.
		 */
		if (port->vbus_never_low) {
			port->vbus_never_low = false;
			tcpm_set_state(port, SOFT_RESET_SEND,
				       PD_T_SINK_WAIT_CAP);
		} else {
			tcpm_set_state(port, hard_reset_state(port),
				       PD_T_SINK_WAIT_CAP);
		}
		break;
	case SNK_NEGOTIATE_CAPABILITIES:
		port->pd_capable = true;
		port->hard_reset_count = 0;
		ret = tcpm_pd_send_request(port);
		if (ret < 0) {
			/* Let the Source send capabilities again. */
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
		} else {
			tcpm_set_state_cond(port, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		}
		break;
	case SNK_TRANSITION_SINK:
	case SNK_TRANSITION_SINK_VBUS:
		tcpm_set_state(port, hard_reset_state(port),
			       PD_T_PS_TRANSITION);
		break;
	case SNK_READY:
		port->update_sink_caps = false;
		tcpm_typec_connect(port);
		/*
		 * Here poll_event_cnt is cleared, waiting for self-powered Type-C devices
		 * to send DR_swap Messge until 1s (TCPM_POLL_EVENT_TIME_OUT * 500us)timeout
		 */
		if (port->wait_dr_swap_message)
			port->poll_event_cnt = 0;

		break;

	/* Hard_Reset states */
	case HARD_RESET_SEND:
		tcpm_pd_transmit(port, TCPC_TX_HARD_RESET, NULL);
		tcpm_set_state(port, HARD_RESET_START, 0);
		port->wait_dr_swap_message = false;
		break;
	case HARD_RESET_START:
		port->hard_reset_count++;
		port->tcpc->set_pd_rx(port->tcpc, false);
		port->nr_sink_caps = 0;
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(port, SRC_HARD_RESET_VBUS_OFF,
				       PD_T_PS_HARD_RESET);
		else
			tcpm_set_state(port, SNK_HARD_RESET_SINK_OFF, 0);
		break;
	case SRC_HARD_RESET_VBUS_OFF:
		tcpm_set_vconn(port, true);
		tcpm_set_vbus(port, false);
		tcpm_set_roles(port, port->self_powered, TYPEC_SOURCE,
			       TYPEC_HOST);
		tcpm_set_state(port, SRC_HARD_RESET_VBUS_ON, PD_T_SRC_RECOVER);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_vconn(port, true);
		tcpm_set_vbus(port, true);
		port->tcpc->set_pd_rx(port->tcpc, true);
		tcpm_set_attached_state(port, true);
		tcpm_set_state(port, SRC_UNATTACHED, PD_T_PS_SOURCE_ON);
		break;
	case SNK_HARD_RESET_SINK_OFF:
		tcpm_set_vconn(port, false);
		if (port->pd_capable)
			tcpm_set_charge(port, false);
		tcpm_set_roles(port, port->self_powered, TYPEC_SINK,
			       TYPEC_DEVICE);
		/*
		 * VBUS may or may not toggle, depending on the adapter.
		 * If it doesn't toggle, transition to SNK_HARD_RESET_SINK_ON
		 * directly after timeout.
		 */
		tcpm_set_state(port, SNK_HARD_RESET_SINK_ON, PD_T_SAFE_0V);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		/* Assume we're disconnected if VBUS doesn't come back. */
		tcpm_set_state(port, SNK_UNATTACHED,
			       PD_T_SRC_RECOVER_MAX + PD_T_SRC_TURN_ON);
		break;
	case SNK_HARD_RESET_SINK_ON:
		/* Note: There is no guarantee that VBUS is on in this state */
		/*
		 * XXX:
		 * The specification suggests that dual mode ports in sink
		 * mode should transition to state PE_SRC_Transition_to_default.
		 * See USB power delivery specification chapter 8.3.3.6.1.3.
		 * This would mean to
		 * - turn off VCONN, reset power supply
		 * - request hardware reset
		 * - turn on VCONN
		 * - Transition to state PE_Src_Startup
		 * SNK only ports shall transition to state Snk_Startup
		 * (see chapter 8.3.3.3.8).
		 * Similar, dual-mode ports in source mode should transition
		 * to PE_SNK_Transition_to_default.
		 */
		if (port->pd_capable) {
			tcpm_set_current_limit(port,
					       tcpm_get_current_limit(port),
					       5000);
			tcpm_set_charge(port, true);
		}
		tcpm_set_attached_state(port, true);
		tcpm_set_state(port, SNK_STARTUP, 0);
		break;

	/* Soft_Reset states */
	case SOFT_RESET:
		port->message_id = 0;
		port->rx_msgid = -1;
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(port, SRC_SEND_CAPABILITIES, 0);
		else
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
		break;
	case SOFT_RESET_SEND:
		port->message_id = 0;
		port->rx_msgid = -1;
		if (tcpm_pd_send_control(port, PD_CTRL_SOFT_RESET))
			tcpm_set_state_cond(port, hard_reset_state(port), 0);
		else
			tcpm_set_state_cond(port, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		break;

	/* DR_Swap states */
	case DR_SWAP_ACCEPT:
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		tcpm_set_state_cond(port, DR_SWAP_CHANGE_DR, 0);
		break;
	case DR_SWAP_CHANGE_DR:
		if (port->data_role == TYPEC_HOST) {
			tcpm_set_roles(port, true, port->pwr_role,
				       TYPEC_DEVICE);
		} else {
			tcpm_set_roles(port, true, port->pwr_role,
				       TYPEC_HOST);
		}
		/* DR_swap process complete, wait_dr_swap_message is cleared */
		port->wait_dr_swap_message = false;
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case ERROR_RECOVERY:
		tcpm_set_state(port, PORT_RESET, 0);
		break;
	case PORT_RESET:
		tcpm_reset_port(port);
		if (port->self_powered)
			tcpm_set_cc(port, TYPEC_CC_OPEN);
		else
			tcpm_set_cc(port, tcpm_default_state(port) == SNK_UNATTACHED ?
				    TYPEC_CC_RD : tcpm_rp_cc(port));
		tcpm_set_state(port, PORT_RESET_WAIT_OFF,
			       PD_T_ERROR_RECOVERY);
		break;
	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(port,
			       tcpm_default_state(port),
			       port->vbus_present ? PD_T_PS_SOURCE_OFF : 0);
		break;
	default:
		dev_err(port->dev, "TCPM: Unexpected port state %d\n", port->state);
		break;
	}
}

static void tcpm_state_machine(struct tcpm_port *port)
{
	enum tcpm_state prev_state;

	mutex_lock(&port->lock);
	port->state_machine_running = true;

	if (port->queued_message && tcpm_send_queued_message(port))
		goto done;

	/* If we were queued due to a delayed state change, update it now */
	if (port->delayed_state) {
		dev_dbg(port->dev, "TCPM: state change %s -> %s [delayed %ld ms]\n",
			tcpm_states[port->state],
			tcpm_states[port->delayed_state], port->delay_ms);
		port->prev_state = port->state;
		port->state = port->delayed_state;
		port->delayed_state = INVALID_STATE;
	}

	/*
	 * Continue running as long as we have (non-delayed) state changes
	 * to make.
	 */
	do {
		prev_state = port->state;
		run_state_machine(port);
		if (port->queued_message)
			tcpm_send_queued_message(port);
	} while (port->state != prev_state && !port->delayed_state);

done:
	port->state_machine_running = false;
	mutex_unlock(&port->lock);
}

static void _tcpm_cc_change(struct tcpm_port *port, enum typec_cc_status cc1,
			    enum typec_cc_status cc2)
{
	enum typec_cc_status old_cc1, old_cc2;
	enum tcpm_state new_state;

	old_cc1 = port->cc1;
	old_cc2 = port->cc2;
	port->cc1 = cc1;
	port->cc2 = cc2;

	dev_dbg(port->dev, "TCPM: CC1: %u -> %u, CC2: %u -> %u [state %s, polarity %d, %s]\n",
		old_cc1, cc1, old_cc2, cc2, tcpm_states[port->state],
		port->polarity,
		tcpm_port_is_disconnected(port) ? "disconnected" : "connected");

	switch (port->state) {
	case TOGGLING:
		if (tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_ATTACH_WAIT, 0);
		else if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		break;
	case SRC_UNATTACHED:
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SRC_UNATTACHED, 0);
		else if (cc1 != old_cc1 || cc2 != old_cc2)
			tcpm_set_state(port, SRC_ATTACH_WAIT, 0);
		break;
	case SRC_ATTACHED:
	case SRC_SEND_CAPABILITIES:
	case SRC_READY:
		if (tcpm_port_is_disconnected(port) ||
		    !tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_UNATTACHED, 0);
		break;
	case SNK_UNATTACHED:
		if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			new_state = SNK_DEBOUNCED;
		else if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else
			break;
		if (new_state != port->delayed_state)
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else if (port->vbus_present)
			new_state = tcpm_try_src(port) ? INVALID_STATE : SNK_ATTACHED;
		else
			new_state = SNK_UNATTACHED;
		if (new_state != port->delayed_state)
			tcpm_set_state(port, SNK_DEBOUNCED, 0);
		break;
	case SNK_READY:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, unattached_state(port), 0);
		else if (!port->pd_capable &&
			 (cc1 != old_cc1 || cc2 != old_cc2))
			tcpm_set_current_limit(port,
					       tcpm_get_current_limit(port),
					       5000);
		break;

	case SNK_DISCOVERY:
		/* CC line is unstable, wait for debounce */
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_DISCOVERY_DEBOUNCE, 0);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore CC changes here.
		 */
		break;
	default:
		/*
		 * While acting as sink and auto vbus discharge is enabled, Allow disconnect
		 * to be driven by vbus disconnect.
		 */
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, unattached_state(port), 0);
		break;
	}
}

static void _tcpm_pd_vbus_on(struct tcpm_port *port)
{
	dev_dbg(port->dev, "TCPM: VBUS on event\n");
	port->vbus_present = true;
	/*
	 * When vbus_present is true i.e. Voltage at VBUS is greater than VSAFE5V implicitly
	 * states that vbus is not at VSAFE0V, hence clear the vbus_vsafe0v flag here.
	 */
	port->vbus_vsafe0v = false;

	switch (port->state) {
	case SNK_TRANSITION_SINK_VBUS:
		port->explicit_contract = true;
		tcpm_set_state(port, SNK_READY, 0);
		break;
	case SNK_DISCOVERY:
		tcpm_set_state(port, SNK_DISCOVERY, 0);
		break;
	case SNK_DEBOUNCED:
		tcpm_set_state(port, SNK_ATTACHED, 0);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		tcpm_set_state(port, SNK_HARD_RESET_SINK_ON, 0);
		break;
	case SRC_ATTACHED:
		tcpm_set_state(port, SRC_STARTUP, 0);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_state(port, SRC_STARTUP, 0);
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;

	default:
		break;
	}
}

static void _tcpm_pd_vbus_off(struct tcpm_port *port)
{
	dev_dbg(port->dev, "TCPM: VBUS off event\n");
	port->vbus_present = false;
	port->vbus_never_low = false;
	switch (port->state) {
	case SNK_HARD_RESET_SINK_OFF:
		tcpm_set_state(port, SNK_HARD_RESET_WAIT_VBUS, 0);
		break;
	case HARD_RESET_SEND:
		break;
	case SNK_ATTACH_WAIT:
		tcpm_set_state(port, SNK_UNATTACHED, 0);
		break;

	case SNK_NEGOTIATE_CAPABILITIES:
		break;

	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(port, tcpm_default_state(port), 0);
		break;

	case PORT_RESET:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;

	default:
		if (port->pwr_role == TYPEC_SINK && port->attached)
			tcpm_set_state(port, SNK_UNATTACHED, 0);
		break;
	}
}

static void _tcpm_pd_hard_reset(struct tcpm_port *port)
{
	dev_dbg(port->dev, "TCPM: Received hard reset\n");
	port->poll_event_cnt = 0;

	/* If a hard reset message is received during the port reset process,
	 * we should ignore it, that is, do not set port->state to HARD_RESET_START.
	 */
	if (port->state == PORT_RESET || port->state == PORT_RESET_WAIT_OFF)
		return;

	/*
	 * If we keep receiving hard reset requests, executing the hard reset
	 * must have failed. Revert to error recovery if that happens.
	 */
	tcpm_set_state(port,
		       port->hard_reset_count < PD_N_HARD_RESET_COUNT ?
				HARD_RESET_START : ERROR_RECOVERY,
		       0);
}

void tcpm_cc_change(struct tcpm_port *port)
{
	enum typec_cc_status cc1, cc2;

	port->poll_event_cnt = 0;
	if (port->tcpc->get_cc(port->tcpc, &cc1, &cc2) == 0)
		_tcpm_cc_change(port, cc1, cc2);
}
EXPORT_SYMBOL_GPL(tcpm_cc_change);

void tcpm_vbus_change(struct tcpm_port *port)
{
	bool vbus;

	port->poll_event_cnt = 0;
	vbus = port->tcpc->get_vbus(port->tcpc);
	if (vbus)
		_tcpm_pd_vbus_on(port);
	else
		_tcpm_pd_vbus_off(port);
}
EXPORT_SYMBOL_GPL(tcpm_vbus_change);

void tcpm_pd_hard_reset(struct tcpm_port *port)
{
	port->poll_event_cnt = 0;
	_tcpm_pd_hard_reset(port);
}
EXPORT_SYMBOL_GPL(tcpm_pd_hard_reset);

static void tcpm_init(struct tcpm_port *port)
{
	enum typec_cc_status cc1, cc2;

	port->tcpc->init(port->tcpc);

	tcpm_reset_port(port);

	/*
	 * XXX
	 * Should possibly wait for VBUS to settle if it was enabled locally
	 * since tcpm_reset_port() will disable VBUS.
	 */
	port->vbus_present = port->tcpc->get_vbus(port->tcpc);
	if (port->vbus_present)
		port->vbus_never_low = true;

	/*
	 * 1. When vbus_present is true, voltage on VBUS is already at VSAFE5V.
	 * So implicitly vbus_vsafe0v = false.
	 *
	 * 2. When vbus_present is false and TCPC does NOT support querying
	 * vsafe0v status, then, it's best to assume vbus is at VSAFE0V i.e.
	 * vbus_vsafe0v is true.
	 *
	 * 3. When vbus_present is false and TCPC does support querying vsafe0v,
	 * then, query tcpc for vsafe0v status.
	 */
	if (port->vbus_present)
		port->vbus_vsafe0v = false;
	else
		port->vbus_vsafe0v = true;

	tcpm_set_state(port, tcpm_default_state(port), 0);

	if (port->tcpc->get_cc(port->tcpc, &cc1, &cc2) == 0)
		_tcpm_cc_change(port, cc1, cc2);
}

static int tcpm_fw_get_caps(struct tcpm_port *port)
{
	const char *cap_str;
	ofnode node = port->tcpc->connector_node;
	int ret;
	u32 mw;

	cap_str = ofnode_read_string(node, "power-role");
	if (!cap_str)
		return -EINVAL;

	if (!strcmp("dual", cap_str))
		port->typec_type = TYPEC_PORT_DRP;
	else if (!strcmp("source", cap_str))
		port->typec_type = TYPEC_PORT_SRC;
	else if (!strcmp("sink", cap_str))
		port->typec_type = TYPEC_PORT_SNK;
	else
		return -EINVAL;

	port->port_type = port->typec_type;

	if (port->port_type == TYPEC_PORT_SNK)
		goto sink;

	/* Get source pdos */
	ret = ofnode_read_size(node, "source-pdos") / sizeof(u32);
	if (ret <= 0)
		return -EINVAL;

	port->nr_src_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = ofnode_read_u32_array(node, "source-pdos",
				    port->src_pdo, port->nr_src_pdo);
	if (ret || tcpm_validate_caps(port, port->src_pdo, port->nr_src_pdo))
		return -EINVAL;

	if (port->port_type == TYPEC_PORT_SRC)
		return 0;

	/* Get the preferred power role for DRP */
	cap_str = ofnode_read_string(node, "try-power-role");
	if (!cap_str)
		return -EINVAL;

	if (!strcmp("sink", cap_str))
		port->typec_prefer_role = TYPEC_SINK;
	else if (!strcmp("source", cap_str))
		port->typec_prefer_role = TYPEC_SOURCE;
	else
		return -EINVAL;

	if (port->typec_prefer_role < 0)
		return -EINVAL;
sink:
	/* Get sink pdos */
	ret = ofnode_read_size(node, "sink-pdos") / sizeof(u32);
	if (ret <= 0)
		return -EINVAL;

	port->nr_snk_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = ofnode_read_u32_array(node, "sink-pdos",
				    port->snk_pdo, port->nr_snk_pdo);
	if (ret || tcpm_validate_caps(port, port->snk_pdo, port->nr_snk_pdo))
		return -EINVAL;

	if (ofnode_read_u32_array(node, "op-sink-microwatt", &mw, 1))
		return -EINVAL;
	port->operating_snk_mw = mw / 1000;

	port->self_powered = ofnode_read_bool(node, "self-powered");

	return 0;
}

struct tcpm_port *tcpm_port_init(struct udevice *dev, struct tcpc_dev *tcpc)
{
	struct tcpm_port *port;
	int err;

	if (!dev || !tcpc ||
	    !tcpc->get_vbus || !tcpc->set_cc || !tcpc->get_cc ||
	    !tcpc->set_polarity || !tcpc->set_vconn || !tcpc->set_vbus ||
	    !tcpc->set_pd_rx || !tcpc->set_roles || !tcpc->pd_transmit)
		return ERR_PTR(-EINVAL);

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	port->dev = dev;
	port->tcpc = tcpc;

	err = tcpm_fw_get_caps(port);
	if (err < 0) {
		dev_err(dev, "TCPM: please check the dts config: %d\n", err);
		return ERR_PTR(err);
	}

	port->try_role = port->typec_prefer_role;
	port->port_type = port->typec_type;

	tcpm_init(port);

	dev_info(port->dev, "TCPM: init finished\n");

	return port;
}
EXPORT_SYMBOL_GPL(tcpm_port_init);

void tcpm_poll_event(struct tcpm_port *port)
{
	if (!port->tcpc->get_vbus(port->tcpc))
		return;

	while (port->poll_event_cnt < TCPM_POLL_EVENT_TIME_OUT) {
		if (!port->wait_dr_swap_message &&
		    (port->state == SNK_READY || port->state == SRC_READY))
			break;

		port->tcpc->poll_event(port->tcpc);
		port->poll_event_cnt++;
		udelay(500);
		tcpm_check_and_run_delayed_work(port);
	}

	if (port->state != SNK_READY && port->state != SRC_READY)
		dev_warn(port->dev, "TCPM: exit in state %s\n",
			 tcpm_states[port->state]);

	/*
	 * At this time, call the callback function of the respective pd chip
	 * to enter the low-power mode. In order to reduce the time spent on
	 * the PD chip driver as much as possible, the tcpm framework does not
	 * fully process the communication initiated by the device,so it should
	 * be noted that we can disable the internal oscillator, etc., but do
	 * not turn off the power of the transceiver module, otherwise the
	 * self-powered Type-C device will initiate a Message(eg: self-powered
	 * Type-C hub initiates a SINK capability request(PD_CTRL_GET_SINK_CAP))
	 * and the pd chip cannot reply to GoodCRC, causing the self-powered Type-C
	 * device to switch vbus to vSafe5v, or even turn off vbus.
	 */
	if (port->tcpc->enter_low_power_mode) {
		if (port->tcpc->enter_low_power_mode(port->tcpc,
						     port->attached,
						     port->pd_capable))
			dev_err(port->dev, "TCPM: failed to enter low power\n");
		else
			dev_info(port->dev, "TCPM: PD chip enter low power mode\n");
	}
}
EXPORT_SYMBOL_GPL(tcpm_poll_event);

int tcpm_get_voltage(struct tcpm_port *port)
{
	return port->supply_voltage;
}
EXPORT_SYMBOL_GPL(tcpm_get_voltage);

int tcpm_get_current(struct tcpm_port *port)
{
	return port->current_limit;
}
EXPORT_SYMBOL_GPL(tcpm_get_voltage);

const char *tcpm_get_state(struct tcpm_port *port)
{
	return tcpm_states[port->state];
}
EXPORT_SYMBOL_GPL(tcpm_get_state);

int tcpm_get(const char *name, struct udevice **devp)
{
	return uclass_get_device_by_name(UCLASS_TCPM, name, devp);
}

int tcpm_print_info(struct udevice *dev)
{
	const struct dm_tcpm_ops *ops = dev_get_driver_ops(dev);

	if (!ops)
		return -ENOSYS;

	if (ops->get_state)
		printf("TCPM State: %s\n", ops->get_state(dev));

	if (ops->get_voltage) {
		int mv = ops->get_voltage(dev);

		printf("Voltage:    %2d.%03d V\n", mv / 1000, mv % 1000);
	}

	if (ops->get_current) {
		int ma = ops->get_current(dev);

		printf("Current:    %2d.%03d A\n", ma / 1000, ma % 1000);
	}

	return 0;
}

UCLASS_DRIVER(tcpm) = {
	.id		= UCLASS_TCPM,
	.name		= "tcpm",
};
