BUILD:=build
CC:=clang
CFLAGS:=-Wall -Wextra -Werror -std=c17 -g

.PHONY: all test clean linux windows

all: $(BUILD)/pras3

linux: $(BUILD)/pras3_linux

windows: $(BUILD)/pras3.exe

archive: $(BUILD)/pras3_linux.zip $(BUILD)/pras3_windows.zip

clean:
	rm -rf $(BUILD)

test: $(BUILD)/pras3
	$(BUILD)/pras3 test

$(BUILD):
	mkdir -p $@

BIG5.TXT:
	curl -O https://unicode.org/Public/MAPPINGS/OBSOLETE/EASTASIA/OTHER/BIG5.TXT

CP949.TXT:
	curl -O https://www.unicode.org/Public/MAPPINGS/VENDORS/MICSFT/WINDOWS/CP949.TXT

GB2312.TXT:
	curl -O https://people.freebsd.org/~perky/i18n/GB2312.TXT

JIS0208.TXT:
	curl -O http://www.unicode.org/Public/MAPPINGS/OBSOLETE/EASTASIA/JIS/JIS0208.TXT

JIS0212.TXT:
	curl -O http://www.unicode.org/Public/MAPPINGS/OBSOLETE/EASTASIA/JIS/JIS0212.TXT

enc.c enc.h: enc.py BIG5.TXT CP949.TXT GB2312.TXT JIS0208.TXT JIS0212.TXT
	python3 enc.py

$(BUILD)/pras3: pras3.c enc.c enc.h | $(BUILD)
	$(CC) $(CFLAGS) $< -o $@

$(BUILD)/pras3_linux: pras3.c | $(BUILD)
	zig cc -target x86_64-linux-musl -D_DEFAULT_SOURCE $(CFLAGS) $< -o $@

$(BUILD)/pras3.exe: pras3.c | $(BUILD)
	zig cc -target x86_64-windows-gnu $(CFLAGS) $< -o $@

$(BUILD)/pras3_linux.zip: $(BUILD)/pras3_linux
	cd $(BUILD); mv pras3_linux /tmp/pras3; pushd /tmp; zip pras3_linux.zip pras3; popd; mv /tmp/pras3_linux.zip .

$(BUILD)/pras3_windows.zip: $(BUILD)/pras3.exe
	cd $(BUILD); zip pras3_windows.zip pras3.exe
