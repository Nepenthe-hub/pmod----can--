
#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "ch9434.h"

struct rx_mailbox_info {
	u32 rxmdh; /* high byte of the receiving email address */
	u32 rxmdl; /* low byte of the receiving email address */
	u32 rxmdt; /* receiving email data length and timestamp */
	u32 rxmir; /* receiving email identifier */
};

struct tx_mailbox_info {
	u32 txmdh; /* high byte of the send email address */
	u32 txmdl; /* low byte of the send email address */
	u32 txmdt; /* send email data length and timestamp */
	u32 txmir; /* send email identifier */
};

static int ch943x_can_init(struct ch943x_port *s)
{
	u32 reg_val;
	unsigned long timeout;
	struct can_bittiming *bt = &s->priv->can.bittiming;

	dev_dbg(s->dev, "%s\n", __func__);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
	reg_val &= ~CAN_CTLR_SLEEP;
	ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
	reg_val |= CAN_CTLR_INRQ;
	ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);

	timeout = jiffies + HZ;
	while (((ch943x_canreg_read(s, CH9434D_CAN_STATR) &
		 CAN_STATR_INAK) != CAN_STATR_INAK)) {
		usleep_range(CH943X_DELAY_MS * 1000,
			     CH943X_DELAY_MS * 1000 * 2);
		if (time_after(jiffies, timeout)) {
			dev_err(s->dev,
				"ch9434d didn't enter in conf mode after reset\n");
			return -EBUSY;
		}
	}

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
	reg_val &= ~(CAN_CTLR_TTCM | CAN_CTLR_ABOM | CAN_CTLR_AWUM |
		     CAN_CTLR_RFLM | CAN_CTLR_TXFP);
	reg_val |= CAN_CTLR_NART;
	ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

	dev_dbg(s->dev,
		"bitrate:%d sample_point:%d tq:%d prop_seg:%d phase_seg1:%d phase_seg2:%d sjw:%d brp:%d\n",
		bt->bitrate, bt->sample_point, bt->tq, bt->prop_seg,
		bt->phase_seg1, bt->phase_seg2, bt->sjw, bt->brp);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
	reg_val = ((bt->sjw - 1) << 24) |
		  ((bt->prop_seg + bt->phase_seg1 - 1) << 16) |
		  ((bt->phase_seg2 - 1) << 20) |
		  ((bt->brp - 1) & 0x000001FF);
	ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
	reg_val &= ~CAN_CTLR_INRQ;
	ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

	timeout = jiffies + HZ;
	while (((ch943x_canreg_read(s, CH9434D_CAN_STATR) &
		 CAN_STATR_INAK) == CAN_STATR_INAK)) {
		usleep_range(CH943X_DELAY_MS * 1000,
			     CH943X_DELAY_MS * 1000 * 2);
		if (time_after(jiffies, timeout)) {
			dev_err(s->dev,
				"ch9434d didn't enter in conf mode after reset\n");
			return -EBUSY;
		}
	}

	return 0;
}

void ch943x_can_filterinit(struct ch943x_port *s)
{
	uint32_t filter_num = 0;
	uint32_t filter_bit = 0;
	uint32_t reg_val;

	dev_dbg(s->dev, "%s\n", __func__);

	for (filter_num = 0; filter_num < 1; filter_num++) {
		filter_bit = BIT(filter_num);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FCTLR);
		reg_val |= CAN_FCTLR_FINIT;
		ch943x_canreg_write(s, CH9434D_CAN_FCTLR, reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FWR);
		reg_val &= ~filter_bit;
		ch943x_canreg_write(s, CH9434D_CAN_FWR, reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FSCFGR);
		reg_val |= filter_bit;
		ch943x_canreg_write(s, CH9434D_CAN_FSCFGR, reg_val);

		reg_val = 0x00000000;
		ch943x_canreg_write(s, CH9434D_CAN_FxR1(filter_num),
				    reg_val);
		reg_val = 0x00000000;
		ch943x_canreg_write(s, CH9434D_CAN_FxR2(filter_num),
				    reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FMCFGR);
		reg_val &= ~filter_bit;
		ch943x_canreg_write(s, CH9434D_CAN_FMCFGR, reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FAFIFOR);
		reg_val &= ~filter_bit;
		ch943x_canreg_write(s, CH9434D_CAN_FAFIFOR, reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FWR);
		reg_val |= filter_bit;
		ch943x_canreg_write(s, CH9434D_CAN_FWR, reg_val);

		reg_val = ch943x_canreg_read(s, CH9434D_CAN_FCTLR);
		reg_val &= ~CAN_FCTLR_FINIT;
		ch943x_canreg_write(s, CH9434D_CAN_FCTLR, reg_val);
	}
}

u8 ch943x_handle_can_tx_contmode(struct ch943x_port *s,
				 struct can_frame *frame)
{
	u8 tx_mailbox = 0;
	struct tx_mailbox_info txinfo[2];
	u8 txbuf[32] = { 0 };
	u8 data[16] = { 0 };
	u32 reg_val;
	u32 sid = 0, eid = 0, exide = 0, rtr = 0;
	int retval;

	dev_dbg(s->dev, "%s\n", __func__);

	if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
	     CAN_TSTATR_TME0) == CAN_TSTATR_TME0)
		tx_mailbox = 0;
	else if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
		  CAN_TSTATR_TME1) == CAN_TSTATR_TME1)
		tx_mailbox = 1;
	else if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
		  CAN_TSTATR_TME2) == CAN_TSTATR_TME2)
		tx_mailbox = 2;
	else {
		tx_mailbox = CAN_TxStatus_NoMailBox;
		return tx_mailbox;
	}

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
	reg_val &= CAN_TXMIRx_TXRQ;
	ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4,
			    reg_val);

	exide = (frame->can_id & CAN_EFF_FLAG) ?
			1 :
			0; /* Extended ID Enable */
	if (exide) {
		eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
		sid = (frame->can_id & CAN_EFF_MASK) >>
		      18; /* Standard ID */
	} else {
		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	}
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 :
					       0; /* Remote transmission */

	if (exide)
		txinfo[0].txmir =
			((eid << 3) | CAN_Id_Extended | (rtr << 1));
	else
		txinfo[0].txmir = ((sid << 21) | (rtr << 1));

	txinfo[0].txmdt = (frame->can_dlc & 0x0000000F);
	memcpy(data, frame->data, frame->can_dlc);
	txinfo[0].txmdl = (data[3] << 24) | (data[2] << 16) |
			  (data[1] << 8) | (data[0]);
	txinfo[0].txmdh = (data[7] << 24) | (data[6] << 16) |
			  (data[5] << 8) | (data[4]);

	memcpy(txbuf, &txinfo[0], 16);
	retval = ch943x_canreg_contmode_write(
		s, CH9434D_CAN_TX0WRITE_CONT + tx_mailbox, txbuf, 16);

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
	reg_val |= CAN_TXMIRx_TXRQ;
	ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4,
			    reg_val);

	return tx_mailbox;
}

u8 ch943x_handle_can_tx(struct ch943x_port *s, struct can_frame *frame)
{
	u8 tx_mailbox = 0;
	u32 reg_val;
	u32 sid = 0, eid = 0, exide = 0, rtr = 0;
	u8 data[16] = { 0 };

	dev_dbg(s->dev, "%s\n", __func__);

	exide = (frame->can_id & CAN_EFF_FLAG) ?
			1 :
			0; /* Extended ID Enable */
	if (exide) {
		eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
		sid = (frame->can_id & CAN_EFF_MASK) >>
		      18; /* Standard ID */
	} else {
		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	}
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 :
					       0; /* Remote transmission */

	if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
	     CAN_TSTATR_TME0) == CAN_TSTATR_TME0) {
		tx_mailbox = 0;
	} else if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
		    CAN_TSTATR_TME1) == CAN_TSTATR_TME1) {
		tx_mailbox = 1;
	} else if ((ch943x_canreg_read(s, CH9434D_CAN_TSTATR) &
		    CAN_TSTATR_TME2) == CAN_TSTATR_TME2) {
		tx_mailbox = 2;
	} else {
		tx_mailbox = CAN_TxStatus_NoMailBox;
		return tx_mailbox;
	}

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
	reg_val &= CAN_TXMIRx_TXRQ;
	ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4,
			    reg_val);

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
	if (exide)
		reg_val |= ((eid << 3) | CAN_Id_Extended | (rtr << 1));
	else
		reg_val |= ((sid << 21) | (rtr << 1));
	ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4,
			    reg_val);

	reg_val = ch943x_canreg_read(s,
				     CH9434D_CAN_TXMDTR0 + tx_mailbox * 4);
	reg_val |= (frame->can_dlc & 0x0000000F);
	ch943x_canreg_write(s, CH9434D_CAN_TXMDTR0 + tx_mailbox * 4,
			    reg_val);

	memcpy(data, frame->data, frame->can_dlc);
	reg_val = ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) |
		   (data[0]));
	ch943x_canreg_write(s, CH9434D_CAN_TXMDLR0 + tx_mailbox * 4,
			    reg_val);
	reg_val = ((data[7] << 24) | (data[6] << 16) | (data[5] << 8) |
		   (data[4]));
	ch943x_canreg_write(s, CH9434D_CAN_TXMDHR0 + tx_mailbox * 4,
			    reg_val);

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
	reg_val |= CAN_TXMIRx_TXRQ;
	ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4,
			    reg_val);

	return tx_mailbox;
}

static void ch943x_hw_tx(struct ch943x_port *s, struct can_frame *frame)
{
	u8 tx_mailbox;

	dev_dbg(s->dev, "%s\n", __func__);
#ifdef CAN_TX_CONTMODE
	tx_mailbox = ch943x_handle_can_tx_contmode(s, frame);
#else
	tx_mailbox = ch943x_handle_can_tx(s, frame);
#endif
}

static void ch943x_tx_work_handler(struct work_struct *ws)
{
	struct ch943x_can_priv *priv =
		container_of(ws, struct ch943x_can_priv, tx_work);
	struct ch943x_port *s = priv->s;
	struct net_device *ndev = priv->ndev;
	struct can_frame *frame;

	dev_dbg(s->dev, "%s\n", __func__);

	mutex_lock(&priv->can_lock);

	if (priv->tx_skb) {
		frame = (struct can_frame *)priv->tx_skb->data;
		if (frame->can_dlc > 8)
			frame->can_dlc = 8;
		ch943x_hw_tx(s, frame);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
		can_put_echo_skb(priv->tx_skb, ndev, 0, 0);
#else
		can_put_echo_skb(priv->tx_skb, ndev, 0);
#endif
		priv->tx_skb = NULL;
	}

	mutex_unlock(&priv->can_lock);
}

static int ch943x_hw_sleep(struct ch943x_port *s)
{
	unsigned long timeout;

	dev_dbg(s->dev, "%s\n", __func__);

	ch943x_canreg_write(s, CH9434D_CAN_CTLR, CAN_CTLR_RESET);
	timeout = jiffies + HZ;
	while ((ch943x_canreg_read(s, CH9434D_CAN_STATR) &
		CAN_STATR_SLAK) != CAN_STATR_SLAK) {
		usleep_range(CH943X_DELAY_MS * 1000,
			     CH943X_DELAY_MS * 1000 * 2);
		if (time_after(jiffies, timeout)) {
			dev_err(s->dev,
				"ch9434d didn't enter in conf mode after reset\n");
			return -EBUSY;
		}
	}

	return 0;
}

static void ch943x_clean(struct net_device *ndev)
{
	struct ch943x_can_priv *priv = netdev_priv(ndev);
	struct ch943x_port *s = priv->s;

	dev_dbg(s->dev, "%s\n", __func__);

	dev_kfree_skb(priv->tx_skb);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0))
	if (priv->tx_busy)
		can_free_echo_skb(priv->ndev, 0, NULL);
#else
	if (priv->tx_busy)
		can_free_echo_skb(priv->ndev, 0);
#endif
}

static int ch943x_setup(struct net_device *net, struct ch943x_port *s)
{
	int ret;

	dev_dbg(s->dev, "%s\n", __func__);

	ret = ch943x_can_init(s);
	ch943x_can_filterinit(s);

	return 0;
}

static void ch943x_restart_work_handler(struct work_struct *ws)
{
	struct ch943x_can_priv *priv =
		container_of(ws, struct ch943x_can_priv, restart_work);
	struct ch943x_port *s = priv->s;
	int ret;

	dev_dbg(s->dev, "%s\n", __func__);

	mutex_lock(&priv->can_lock);
	ch943x_hw_sleep(s);
	netif_device_attach(priv->ndev);
	ch943x_clean(priv->ndev);

	ret = ch943x_setup(priv->ndev, s);

	netif_wake_queue(priv->ndev);
	mutex_unlock(&priv->can_lock);
}

#if 0
static int ch943x_power_enable(struct regulator *reg, int enable)
{
    if (IS_ERR_OR_NULL(reg))
        return 0;

    if (enable)
        return regulator_enable(reg);
    else
        return regulator_disable(reg);
}
#endif

static int ch943x_set_mode(struct ch943x_port *s)
{
	struct ch943x_can_priv *priv = s->priv;
	unsigned long timeout;
	u32 reg_val;

	dev_dbg(s->dev, "%s ctrlmode:0x%x\n", __func__,
		priv->can.ctrlmode);

	/* Enable interrupts */
	reg_val = ch943x_canreg_read(s, CH9434D_CAN_INTENR);
	reg_val |=
		(CAN_INTENR_FFIE0 | CAN_INTENR_FOVIE0 | CAN_INTENR_FFIE1 |
		 CAN_INTENR_FOVIE1 | CAN_INTENR_EWGIE | CAN_INTENR_EPVIE |
		 CAN_INTENR_BOFIE | CAN_INTENR_LECIE | CAN_INTENR_ERRIE |
		 CAN_INTENR_WKUIE | CAN_INTENR_SLKIE | CAN_INTENR_FMPIE0 |
		 CAN_INTENR_FMPIE1);
	reg_val |= CAN_INTENR_TMEIE;
	ch943x_canreg_write(s, CH9434D_CAN_INTENR, reg_val);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* Put device into loopback mode */
		reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
		reg_val |= CAN_BTIMR_LBKM;
		ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* Put device into listen-only mode */
		reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
		reg_val |= CAN_BTIMR_SILM;
		ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);
	} else {
		/* Put device into normal mode */
		reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
		reg_val &= ~CAN_CTLR_INRQ;
		ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

		/* Wait for the device to enter normal mode */
		timeout = jiffies + HZ;
		while ((ch943x_canreg_read(s, CH9434D_CAN_STATR) &
			CAN_STATR_INAK) == CAN_STATR_INAK) {
			usleep_range(CH943X_DELAY_MS * 1000,
				     CH943X_DELAY_MS * 1000 * 2);
			if (time_after(jiffies, timeout)) {
				dev_err(s->dev,
					"ch9434d didn't enter in conf mode after reset\n");
				return -EBUSY;
			}
		}
	}

	return 0;
}

static int ch943x_stop(struct net_device *ndev)
{
	struct ch943x_can_priv *priv = netdev_priv(ndev);
	struct ch943x_port *s = priv->s;
	u32 reg_val;

	dev_dbg(s->dev, "%s\n", __func__);

	close_candev(ndev);

	reg_val = 0;
	ch943x_canreg_write(s, CH9434D_CAN_INTENR, reg_val);
	ch943x_hw_sleep(s);
	priv->can.state = CAN_STATE_STOPPED;
	atomic_set(&s->priv->can_isopen, 0);
	return 0;
}

static netdev_tx_t ch943x_start_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct ch943x_can_priv *priv = netdev_priv(ndev);
	struct ch943x_port *s = priv->s;

	dev_dbg(s->dev, "%s\n", __func__);

	if (priv->tx_skb) {
		dev_warn(s->dev, "hard_xmit called while tx busy\n");
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);
	priv->tx_skb = skb;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static const struct ethtool_ops ch943x_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static const struct can_bittiming_const ch943x_bittiming_const = {
	.name = "ch943x_can",
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static int ch943x_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct ch943x_can_priv *priv = netdev_priv(ndev);
	struct ch943x_port *s = priv->s;

	dev_dbg(s->dev, "%s\n", __func__);

	switch (mode) {
	case CAN_MODE_START:
		ch943x_clean(ndev);
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		queue_work(priv->wq, &priv->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/*
 * Open can device
 */
static int ch943x_open(struct net_device *ndev)
{
	struct ch943x_can_priv *priv = netdev_priv(ndev);
	struct ch943x_port *s = priv->s;
	int ret;

	dev_dbg(s->dev, "%s\n", __func__);

	ret = open_candev(ndev);
	if (ret) {
		dev_err(s->dev, "unable to set initial baudrate!\n");
		return ret;
	}

	atomic_set(&s->priv->can_isopen, 1);
	mutex_lock(&priv->can_lock);

	priv->force_quit = 0;
	priv->tx_skb = NULL;

	INIT_WORK(&priv->tx_work, ch943x_tx_work_handler);
	INIT_WORK(&priv->restart_work, ch943x_restart_work_handler);

	ret = ch943x_setup(ndev, s);
	if (ret)
		return -1;

	ret = ch943x_set_mode(s);
	if (ret)
		return -1;

	mutex_unlock(&priv->can_lock);
	return 0;
}

static const struct net_device_ops ch943x_netdev_ops = {
	.ndo_open = ch943x_open,
	.ndo_stop = ch943x_stop,
	.ndo_start_xmit = ch943x_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	.ndo_change_mtu = can_change_mtu,
#endif
};

int ch943x_can_resgister(struct ch943x_port *s)
{
	struct net_device *ndev;
	int ret = 0;

	dev_dbg(s->dev, "%s\n", __func__);

	s->priv = NULL;

	/* Allocate can/net device */
	ndev = alloc_candev(sizeof(struct ch943x_can_priv), 1);
	if (!ndev)
		return -ENOMEM;

	ndev->netdev_ops = &ch943x_netdev_ops;
	ndev->ethtool_ops = &ch943x_ethtool_ops;
	ndev->flags |= IFF_ECHO;

	s->priv = netdev_priv(ndev);
	if (!s->priv) {
		dev_err(s->dev, "Failed to get private data\n");
		return -ENOMEM;
	}

	atomic_set(&s->priv->can_isopen, 0);
	s->priv->s = s;
	s->priv->can.bittiming_const = &ch943x_bittiming_const;
	s->priv->can.do_set_mode = ch943x_do_set_mode;
	s->priv->can.clock.freq = 96000000;
	s->priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
					  CAN_CTRLMODE_LOOPBACK |
					  CAN_CTRLMODE_LISTENONLY;
	s->priv->ndev = ndev;
	mutex_init(&s->priv->can_lock);

	s->priv->wq = alloc_workqueue("ch943x_wq",
				      WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	if (!s->priv->wq) {
		ret = -ENOMEM;
		goto error_probe;
	}

	ret = ch943x_hw_sleep(s);
	if (ret < 0)
		goto error_probe;

	SET_NETDEV_DEV(ndev, s->dev);

	ret = register_candev(ndev);
	if (ret)
		goto error_probe;

	return 0;

error_probe:
	free_candev(ndev);

	return ret;
}

void ch943x_can_remove(struct ch943x_port *s)
{
	unregister_candev(s->priv->ndev);
}

#ifdef CAN_RX_CONTMODE
static void ch943x_hw_rx_contmode(struct ch943x_port *s, int rx_mailbox)
{
	struct ch943x_can_priv *priv = s->priv;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 exide;
	u8 buf[16] = { 0 };
	int i;
	int retlen;
	struct rx_mailbox_info rxinfo[4];
	u8 rxbuf[64];

	dev_dbg(s->dev, "%s\n", __func__);

	retlen = ch943x_canreg_contmode_read(
		s, CH9434D_CAN_RX0READ_CONT + rx_mailbox, rxbuf);
	for (i = 0; i < 4; i++) {
		memcpy(rxinfo + i, rxbuf + (i * 16), 16);
		if ((rxinfo[i].rxmdt & 0x0F) == 0)
			continue;

		skb = alloc_can_skb(priv->ndev, &frame);
		if (!skb) {
			dev_err(s->dev, "cannot allocate RX skb\n");
			priv->ndev->stats.rx_dropped++;
			return;
		}

		exide = 0x04 & rxinfo[i].rxmir;
		if (exide) {
			/* Extended ID format */
			frame->can_id = CAN_EFF_FLAG;
			frame->can_id |=
				((rxinfo[i].rxmir >> 3) & 0x1FFFFFFF);
			/* Remote transmission request */
			if (0x02 & rxinfo[i].rxmir)
				frame->can_id |= CAN_RTR_FLAG;
		} else {
			/* Standard ID format */
			frame->can_id =
				((rxinfo[i].rxmir >> 21) & 0x000007FF);
		}

		/* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
		frame->len = can_cc_dlc2len(rxinfo[i].rxmdt & 0x0F);
#else
		frame->can_dlc = get_can_dlc(rxinfo[i].rxmdt & 0x0F);
#endif
		buf[0] = 0xFF & rxinfo[i].rxmdl;
		buf[1] = 0xFF & (rxinfo[i].rxmdl >> 8);
		buf[2] = 0xFF & (rxinfo[i].rxmdl >> 16);
		buf[3] = 0xFF & (rxinfo[i].rxmdl >> 24);

		buf[4] = 0xFF & rxinfo[i].rxmdh;
		buf[5] = 0xFF & (rxinfo[i].rxmdh >> 8);
		buf[6] = 0xFF & (rxinfo[i].rxmdh >> 16);
		buf[7] = 0xFF & (rxinfo[i].rxmdh >> 24);
		memcpy(frame->data, buf, frame->can_dlc);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
		priv->ndev->stats.rx_bytes += frame->len;
#else
		priv->ndev->stats.rx_bytes += frame->can_dlc;
#endif
		priv->ndev->stats.rx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
		netif_rx(skb);
#else
		netif_rx_ni(skb);
#endif
	}
}
#endif

static void ch943x_hw_rx(struct ch943x_port *s, int rx_mailbox)
{
	struct ch943x_can_priv *priv = s->priv;
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 reg_val;
	u32 exide;
	u8 buf[16] = { 0 };

	dev_dbg(s->dev, "%s\n", __func__);

	skb = alloc_can_skb(priv->ndev, &frame);
	if (!skb) {
		dev_err(s->dev, "cannot allocate RX skb\n");
		priv->ndev->stats.rx_dropped++;
		return;
	}

	reg_val =
		ch943x_canreg_read(s, CH9434D_CAN_RXMIR0 + rx_mailbox * 4);
	exide = 0x04 & reg_val;
	if (exide) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |= ((reg_val >> 3) & 0x1FFFFFFF);
		/* Remote transmission request */
		if ((u8)0x02 & reg_val)
			frame->can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard ID format */
		frame->can_id = ((reg_val >> 21) & 0x000007FF);
	}

	/* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	reg_val = ch943x_canreg_read(s,
				     CH9434D_CAN_RXMDTR0 + rx_mailbox * 4);
	frame->len = can_cc_dlc2len(reg_val & 0x0F);
#else
	reg_val = ch943x_canreg_read(s,
				     CH9434D_CAN_RXMDTR0 + rx_mailbox * 4);
	frame->can_dlc = get_can_dlc(reg_val & 0x0F);
#endif

	reg_val = ch943x_canreg_read(s,
				     CH9434D_CAN_RXMDLR0 + rx_mailbox * 4);
	buf[0] = 0xFF & reg_val;
	buf[1] = 0xFF & (reg_val >> 8);
	buf[2] = 0xFF & (reg_val >> 16);
	buf[3] = 0xFF & (reg_val >> 24);
	reg_val = ch943x_canreg_read(s,
				     CH9434D_CAN_RXMDHR0 + rx_mailbox * 4);
	buf[4] = 0xFF & reg_val;
	buf[5] = 0xFF & (reg_val >> 8);
	buf[6] = 0xFF & (reg_val >> 16);
	buf[7] = 0xFF & (reg_val >> 24);
	memcpy(frame->data, buf, frame->can_dlc);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	priv->ndev->stats.rx_bytes += frame->len;
#else
	priv->ndev->stats.rx_bytes += frame->can_dlc;
#endif

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_RFIFO0 + rx_mailbox);
	reg_val |= CAN_RFIFOx_RFOMx;
	ch943x_canreg_write(s, CH9434D_CAN_RFIFO0 + rx_mailbox, reg_val);

	priv->ndev->stats.rx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

u8 ch943x_get_can_trans_state(struct ch943x_port *s, u8 tx_mailbox)
{
	u32 state = 0;
	u32 reg_val;

	dev_dbg(s->dev, "%s\n", __func__);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_TSTATR);
	switch (tx_mailbox) {
	case 0:
		state = reg_val & (CAN_TSTATR_RQCP0 | CAN_TSTATR_TXOK0 |
				   CAN_TSTATR_TME0);
		break;
	case 1:
		state = reg_val & (CAN_TSTATR_RQCP1 | CAN_TSTATR_TXOK1 |
				   CAN_TSTATR_TME1);
		break;
	case 2:
		state = reg_val & (CAN_TSTATR_RQCP2 | CAN_TSTATR_TXOK2 |
				   CAN_TSTATR_TME2);
		break;
	default:
		return -1;
	}

	switch (state) {
	case (0x0):
		state = CAN_TxStatus_Pending;
		break;
	case (CAN_TSTATR_RQCP0 | CAN_TSTATR_TME0):
	case (CAN_TSTATR_RQCP1 | CAN_TSTATR_TME1):
	case (CAN_TSTATR_RQCP2 | CAN_TSTATR_TME2):
		state = CAN_TxStatus_Failed;
		break;
	case (CAN_TSTATR_RQCP0 | CAN_TSTATR_TXOK0 | CAN_TSTATR_TME0):
	case (CAN_TSTATR_RQCP1 | CAN_TSTATR_TXOK1 | CAN_TSTATR_TME1):
	case (CAN_TSTATR_RQCP2 | CAN_TSTATR_TXOK2 | CAN_TSTATR_TME2):
		state = CAN_TxStatus_Ok;
		break;
	default:
		state = CAN_TxStatus_Failed;
		break;
	}
	return state;
}

static void ch943x_can_err(struct ch943x_port *s, u32 fifo0_state,
			   u32 fifo1_state, u32 err_state)
{
	struct net_device *ndev = s->priv->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	struct net_device_stats *stats = &ndev->stats;
	enum can_state new_state;

	dev_dbg(s->dev, "%s\n", __func__);

	skb = alloc_can_err_skb(ndev, &frame);
	if (!skb)
		return;

	if ((fifo0_state & CAN_RFIFOx_FOVRx) ||
	    (fifo1_state & CAN_RFIFOx_FOVRx)) {
		dev_err(s->dev, "CAN data overrun\n");
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
	}

	/* Update can state */
	if (err_state & CAN_ERRSR_EWGF) {
		new_state = CAN_STATE_ERROR_ACTIVE;
	} else if (err_state & (CAN_ERRSR_EPVF | CAN_ERRSR_REC)) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_RX_PASSIVE;
	} else if (err_state & (CAN_ERRSR_EPVF | CAN_ERRSR_TEC)) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_TX_PASSIVE;
	} else if (err_state & CAN_ERRSR_BOFF) {
		new_state = CAN_STATE_BUS_OFF;
		frame->can_id |= CAN_ERR_BUSOFF;
		s->priv->can.can_stats.bus_off++;
		can_bus_off(ndev);
		ch943x_hw_sleep(s);
	}

	/* Update can state statistics */
	switch (s->priv->can.state) {
	case CAN_STATE_ERROR_ACTIVE:
		if (new_state >= CAN_STATE_ERROR_WARNING &&
		    new_state <= CAN_STATE_BUS_OFF)
			s->priv->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		fallthrough;
#endif
	case CAN_STATE_ERROR_WARNING: /* fallthrough */
		if (new_state >= CAN_STATE_ERROR_PASSIVE &&
		    new_state <= CAN_STATE_BUS_OFF)
			s->priv->can.can_stats.error_passive++;
		break;
	default:
		break;
	}
	s->priv->can.state = new_state;

	netif_rx(skb);
}

void ch943x_can_irq(struct ch943x_port *s)
{
	u32 reg_val;
	u32 state, err_state;
	int i;
	int nums;
	u32 fifo0_state, fifo1_state;
	struct net_device *ndev = s->priv->ndev;

	dev_dbg(s->dev, "%s\n", __func__);

	reg_val = ch943x_canreg_read(s, CH9434D_CAN_STATR);
	if (reg_val & CAN_STATR_ERRI)
		ch943x_canreg_write(s, CH9434D_CAN_STATR,
				    reg_val | CAN_STATR_ERRI);
	if (reg_val & CAN_STATR_WKUI)
		ch943x_canreg_write(s, CH9434D_CAN_STATR,
				    reg_val | CAN_STATR_WKUI);
	if (reg_val & CAN_STATR_SLAKI)
		ch943x_canreg_write(s, CH9434D_CAN_STATR,
				    reg_val | CAN_STATR_SLAKI);

	for (i = 0; i < 3; i++) {
		state = ch943x_get_can_trans_state(s, i);
		if (state == CAN_TxStatus_Ok) {
			ch943x_canreg_write(s, CH9434D_CAN_TSTATR,
					    CAN_TSTATR_RQCP0 |
						    CAN_TSTATR_RQCP1 |
						    CAN_TSTATR_RQCP2);
			ndev->stats.tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
			ndev->stats.tx_bytes +=
				can_get_echo_skb(ndev, 0, NULL);
#else
			ndev->stats.tx_bytes += can_get_echo_skb(ndev, 0);
#endif
			netif_wake_queue(ndev);
		}
	}

	fifo0_state = ch943x_canreg_read(s, CH9434D_CAN_RFIFO0);
	nums = fifo0_state & 0x000000FF;
	if (fifo0_state &
	    (CAN_RFIFOx_FMPx | CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx)) {
		ch943x_canreg_write(s, CH9434D_CAN_RFIFO0,
				    CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx);
#ifdef CAN_RX_CONTMODE
		ch943x_hw_rx_contmode(s, 0);
#else
		for (i = 0; i < nums; i++)
			ch943x_hw_rx(s, 0);
#endif
	}
	reg_val = ch943x_canreg_read(s, CH9434D_CAN_RFIFO1);
	fifo1_state = reg_val;
	nums = reg_val & 0x000000FF;
	if (reg_val &
	    (CAN_RFIFOx_FMPx | CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx)) {
		ch943x_canreg_write(s, CH9434D_CAN_RFIFO1,
				    CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx);
#ifdef CAN_RX_CONTMODE
		ch943x_hw_rx_contmode(s, 0);
#else
		for (i = 0; i < nums; i++)
			ch943x_hw_rx(s, 1);
#endif
	}

	err_state = ch943x_canreg_read(s, CH9434D_CAN_ERRSR);
	if (err_state) {
		if (err_state & (u32)0x70)
			ch943x_canreg_write(s, CH9434D_CAN_ERRSR, 0x70);
		else
			ch943x_canreg_write(s, CH9434D_CAN_ERRSR, 0x00);
	}

	if (fifo0_state & CAN_RFIFOx_FOVRx ||
	    fifo1_state & CAN_RFIFOx_FOVRx || err_state) {
		ch943x_can_err(s, fifo0_state, fifo1_state, err_state);
	}
}
