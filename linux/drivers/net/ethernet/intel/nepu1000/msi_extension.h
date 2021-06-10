/**
 * This Extensions add MSI/MSI-X to traditional e1000 devices
 */

#ifndef _MSI_EXTENSION_H_
#define _MSI_EXTENSION_H_

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

/**
 * Bring some macro from e1000.h for e1000e
 * Those are MSI macro
 */
#define FLAG_HAS_MSIX                     BIT(10)
#define FLAG_MSI_ENABLED                  BIT(27)
#define FLAG_MSI_TEST_FAILED              BIT(31)

static irqreturn_t e1000_intr_msi(int __always_unused irq, void *data);