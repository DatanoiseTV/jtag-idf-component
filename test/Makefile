CC ?= cc
CFLAGS = -Wall -Wextra -g -DTEST_HOST -I../components/xmos_jtag/src

.PHONY: all test clean

all: test_xe_parser

test_xe_parser: test_xe_parser.c ../components/xmos_jtag/src/xmos_xe.c ../components/xmos_jtag/src/xmos_regs.h ../components/xmos_jtag/src/xmos_xe.h
	$(CC) $(CFLAGS) -o $@ test_xe_parser.c

test: test_xe_parser
	./test_xe_parser
	@echo ""
	@echo "--- Testing with real XE files (if available) ---"
	@test -f /Users/syso/Downloads/satellite_firmware_apps/satellite1_firmware_fixed_delay.xe && \
		./test_xe_parser /Users/syso/Downloads/satellite_firmware_apps/satellite1_firmware_fixed_delay.xe || true
	@test -f /Users/syso/Downloads/satellite_firmware_apps/satellite1_usb_firmware_ns.xe && \
		./test_xe_parser /Users/syso/Downloads/satellite_firmware_apps/satellite1_usb_firmware_ns.xe || true

clean:
	rm -f test_xe_parser
