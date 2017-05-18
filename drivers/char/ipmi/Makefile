#
# Makefile for the ipmi drivers.
#

ipmi_si-y := ipmi_si_intf.o ipmi_kcs_sm.o ipmi_smic_sm.o ipmi_bt_sm.o
ipmi_hisi_lpc_bt-objs := lpc_ipmi.o ipmi_bt_sm.o lpc_bt_sm.o

obj-$(CONFIG_IPMI_HANDLER) += ipmi_msghandler.o
obj-$(CONFIG_IPMI_DEVICE_INTERFACE) += ipmi_devintf.o
obj-$(CONFIG_IPMI_SI) += ipmi_si.o
obj-$(CONFIG_IPMI_SSIF) += ipmi_ssif.o
obj-$(CONFIG_IPMI_POWERNV) += ipmi_powernv.o
obj-$(CONFIG_IPMI_WATCHDOG) += ipmi_watchdog.o
obj-$(CONFIG_IPMI_POWEROFF) += ipmi_poweroff.o
obj-$(CONFIG_ASPEED_BT_IPMI_BMC) += bt-bmc.o
obj-$(CONFIG_IPMI_HISI_LPC_BT) += ipmi_hisi_lpc_bt.o
