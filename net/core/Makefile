#
# Makefile for the Linux networking core.
#

obj-y := sock.o request_sock.o skbuff.o iovec.o datagram.o stream.o scm.o \
	 gen_stats.o gen_estimator.o net_namespace.o

obj-$(CONFIG_SYSCTL) += sysctl_net_core.o
obj-$(CONFIG_HAS_DMA) += skb_dma_map.o

obj-y		     += dev.o ethtool.o dev_mcast.o dst.o netevent.o \
			neighbour.o rtnetlink.o utils.o link_watch.o filter.o

ifeq ($(CONFIG_ARM),y)
# Warnings are produced by the current arm cross compiler (v4.2.1) causing
# build abort. Version 4.3.1 of the compiler does not create this warning.
# So, remove the -Werror until the compiler can be updated for all build
# platforms to v4.3.1
CFLAGS_REMOVE_neighbour.o = -Werror
endif

obj-$(CONFIG_XFRM) += flow.o
obj-y += net-sysfs.o
obj-$(CONFIG_NET_PKTGEN) += pktgen.o
obj-$(CONFIG_NETPOLL) += netpoll.o
obj-$(CONFIG_NET_DMA) += user_dma.o
obj-$(CONFIG_FIB_RULES) += fib_rules.o
