BUILD:=build
CC:=clang
CFLAGS:=-Wall -Wextra -Werror -std=c17 -g

.PHONY: all test clean linux windows

all: $(BUILD)/pras3

linux: $(BUILD)/pras3_linux

windows: $(BUILD)/pras3.exe

clean:
	rm -rf $(BUILD)

test: $(BUILD)/pras3
	$(BUILD)/pras3 test

$(BUILD):
	mkdir -p $@

$(BUILD)/pras3: pras3.c | $(BUILD)
	$(CC) $(CFLAGS) $< -o $@

$(BUILD)/pras3_linux: pras3.c | $(BUILD)
	zig cc -target x86_64-linux-musl -D_DEFAULT_SOURCE $(CFLAGS) $< -o $@

$(BUILD)/pras3.exe: pras3.c | $(BUILD)
	zig cc -target x86_64-windows-gnu $(CFLAGS) $< -o $@
