bootblock-y += spi.c alternate_cbfs.c
bootblock-y += bootblock.c
bootblock-y += pinmux.c timer.c power.c
# Clock is required for UART
bootblock-$(CONFIG_BOOTBLOCK_CONSOLE) += clock_init.c
bootblock-$(CONFIG_BOOTBLOCK_CONSOLE) += clock.c
ifeq ($(CONFIG_DRIVERS_UART),y)
bootblock-$(CONFIG_BOOTBLOCK_CONSOLE) += uart.c
endif
bootblock-y += wakeup.c
bootblock-y += gpio.c

romstage-y += spi.c alternate_cbfs.c
romstage-y += smp.c
romstage-y += clock.c
romstage-y += clock_init.c
romstage-y += pinmux.c  # required by s3c24x0_i2c and uart.
romstage-y += dmc_common.c
romstage-y += dmc_init_ddr3.c
romstage-y += power.c
romstage-y += timer.c
romstage-$(CONFIG_DRIVERS_UART) += uart.c
romstage-y += wakeup.c
romstage-y += gpio.c
romstage-y += i2c.c
#romstage-y += wdt.c
romstage-y += cbmem.c
romstage-y += trustzone.c

ramstage-y += spi.c alternate_cbfs.c
ramstage-y += clock.c
ramstage-y += clock_init.c
ramstage-y += pinmux.c
ramstage-y += power.c
ramstage-$(CONFIG_DRIVERS_UART) += uart.c
ramstage-y += cpu.c
ramstage-y += tmu.c
ramstage-y += timer.c
ramstage-y += gpio.c
ramstage-y += i2c.c
ramstage-y += dp.c dp_lowlevel.c fimd.c
ramstage-y += usb.c
ramstage-y += cbmem.c

rmodules_$(ARCH-ROMSTAGE-y)-y += timer.c

CPPFLAGS_common += -Isrc/soc/samsung/exynos5420/include/

$(objcbfs)/bootblock.raw.elf: $(objcbfs)/bootblock.elf
	cp $< $@

$(objcbfs)/bootblock.bin: $(objcbfs)/bootblock.raw.bin
	@printf "    BL1, CKSUM $(subst $(obj)/,,$(@))\n"
	util/exynos/variable_cksum.py $< $<.cksum
	cat 3rdparty/cpu/samsung/exynos5420/bl1.bin $<.cksum > $@
