ifeq ($(CONFIG_SOUTHBRIDGE_AMD_SB900),y)

ramstage-y += sb900.c
ramstage-y += usb.c
ramstage-y += lpc.c
ramstage-y += sm.c
ramstage-y += ide.c
ramstage-y += sata.c
ramstage-y += hda.c
#ramstage-y += pci.c
ramstage-y += pcie.c
ramstage-$(CONFIG_HAVE_ACPI_TABLES) += fadt.c
ramstage-y += reset.c
ramstage-y += ramtop.c
ramstage-$(CONFIG_USBDEBUG) += enable_usbdebug.c
ramstage-$(CONFIG_HAVE_SMI_HANDLER) += smi.c smi_util.c

romstage-y += smbus.c
romstage-$(CONFIG_USBDEBUG_IN_ROMSTAGE) += enable_usbdebug.c
romstage-y += early_setup.c
romstage-y += ramtop.c

smm-$(CONFIG_HAVE_SMI_HANDLER) += smihandler.c smi_util.c

endif
