/*
 * Copyright (C) 2016-17 Freie Universität Berlin
 *               2018 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_netif
 * @{
 *
 * @file
 *
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 *
 * Adapted from @ref gnrc_xbee.c with only minor modifications. So all credit
 * goes to Martine Lenders <m.lenders@fu-berlin.de> and to
 * Hauke Petersen <hauke.petersen@fu-berlin.de>.
 *
 * @}
 */

#include "assert.h"
#include "net/gnrc.h"
#include "luid.h"
#include "lifi.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define BCAST  (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)

static gnrc_pktsnip_t *lifi_adpt_recv(gnrc_netif_t *netif)
{
    netdev_t *dev = netif->dev;
    cc1xxx_rx_info_t rx_info;
    int pktlen;
    gnrc_pktsnip_t *payload;
    gnrc_pktsnip_t *hdr;
    gnrc_netif_hdr_t *netif_hdr;
    lifi_l2hdr_t l2hdr;

    assert(dev);

    /* see how much data there is to process */
    pktlen = dev->driver->recv(dev, NULL, 0, &rx_info);
    if (pktlen <= 0) {
        DEBUG("[cc1xxx-gnrc] recv: no data available to process\n");
        return NULL;
    }

    /* allocate space for the packet in the pktbuf */
    payload = gnrc_pktbuf_add(NULL, NULL, pktlen, LIFI_DEFAULT_PROTOCOL);
    if (payload == NULL) {
        DEBUG("[lifi-gnrc] recv: unable to allocate space in the pktbuf\n");
        /* tell the driver to drop the packet */
        dev->driver->recv(dev, NULL, pktlen, NULL);
        return NULL;
    }

    /* copy the complete including the lifi header into the packet buffer */
    dev->driver->recv(dev, payload->data, pktlen, NULL);

    /* The first two bytes are the layer 2 header */
    l2hdr.dest_addr = ((uint8_t *)payload->data)[0];
    l2hdr.src_addr = ((uint8_t *)payload->data)[1];

    /* crop the layer 2 header from the payload */
    hdr = gnrc_pktbuf_mark(payload, sizeof(lifi_l2hdr_t), GNRC_NETTYPE_UNDEF);
    if (hdr == NULL) {
        DEBUG("[lifi-gnrc] recv: unable to mark lifi header snip\n");
        gnrc_pktbuf_release(payload);
        return NULL;
    }
    gnrc_pktbuf_remove_snip(payload, hdr);

    /* create a netif hdr from the obtained data */
    hdr = gnrc_netif_hdr_build(&l2hdr.src_addr, LIFI_ADDR_SIZE,
                               &l2hdr.dest_addr, LIFI_ADDR_SIZE);
    if (hdr == NULL) {
        DEBUG("[lifi-gnrc] recv: unable to allocate netif header\n");
        gnrc_pktbuf_release(payload);
        return NULL;
    }
    netif_hdr = (gnrc_netif_hdr_t *)hdr->data;
    gnrc_netif_hdr_set_netif(netif_hdr, netif);
    netif_hdr->rssi = GNRC_NETIF_HDR_NO_RSSI;
    netif_hdr->lqi = GNRC_NETIF_HDR_NO_LQI;
    if (l2hdr.dest_addr == LIFI_BCAST_ADDR) {
        netif_hdr->flags = GNRC_NETIF_HDR_FLAGS_BROADCAST;
    }

    DEBUG("[lifi-gnrc] recv: successfully parsed packet\n");

    /* and append the netif header */
    payload = gnrc_pkt_append(payload, hdr);

#ifdef MODULE_NETSTATS_L2
    netif->stats.rx_count++;
    netif->stats.rx_bytes += pktlen;
#endif

    return payload;
}

static int lifi_adpt_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt)
{
    int res;
    size_t size;
    lifi_t *lifi_dev = (lifi_t *)netif->dev;
    gnrc_netif_hdr_t *netif_hdr;
    lifi_l2hdr_t l2hdr;

    /* check device descriptor and packet */
    assert(netif && pkt);

    /* get the payload size and the dst address details */
    size = gnrc_pkt_len(pkt->next);
    DEBUG("[lifi-gnrc] send: payload size of packet is %i\n", (int)size);
    netif_hdr = (gnrc_netif_hdr_t *)pkt->data;

    l2hdr.src_addr = lifi_dev->addr;
    if (netif_hdr->flags & BCAST) {
        l2hdr.dest_addr = LIFI_BCAST_ADDR;
        DEBUG("[lifi-gnrc] send: preparing to send broadcast\n");
#ifdef MODULE_NETSTATS_L2
        netif->stats.tx_mcast_count++;
#endif
    }
    else {
        /* check that destination address is valid */
        assert(netif_hdr->dst_l2addr_len > 0);
        uint8_t *addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
        l2hdr.dest_addr = addr[0];
        DEBUG("[lifi-gnrc] send: preparing to send unicast %02x --> %02x\n",
              (int)l2hdr.src_addr, (int)l2hdr.dest_addr);
#ifdef MODULE_NETSTATS_L2
        netif->stats.tx_unicast_count++;
#endif
    }

    /* now let's send out the stuff */
    iolist_t iolist = {
        .iol_next = (iolist_t *)pkt->next,
        .iol_base = &l2hdr,
        .iol_len = sizeof(l2hdr),
    };

    DEBUG("[lifi-gnrc] send: triggering the drivers send function\n");
    res = netif->dev->driver->send(netif->dev, &iolist);

    gnrc_pktbuf_release(pkt);

    return res;
}

static const gnrc_netif_ops_t lifi_netif_ops = {
    .init = gnrc_netif_default_init,
    .send = lifi_adpt_send,
    .recv = lifi_adpt_recv,
    .get = gnrc_netif_get_from_netdev,
    .set = gnrc_netif_set_from_netdev,
};

int gnrc_netif_lifi_create(gnrc_netif_t *netif, char *stack, int stacksize,
                             char priority, char *name, netdev_t *dev)
{
    return gnrc_netif_create(netif, stack, stacksize, priority, name,
                             dev, &lifi_netif_ops);
}

void __attribute__((weak)) lifi_eui_get(const netdev_t *netdev, uint8_t *eui)
{
    (void)netdev;
    do {
        luid_get(eui, 1);
    }
    while (*eui == LIFI_BCAST_ADDR);
}
