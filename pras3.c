//
// This file is in the public domain.
//
#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef _WIN32
#	define IS_WINDOWS 1
#	include <windows.h>
#else
#	define IS_WINDOWS 0
#	include <fcntl.h>
#	include <termios.h>
#endif

#ifdef __APPLE__
#	define IS_MACOS 1
#else
#	define IS_MACOS 0
#endif

#ifdef __linux__
#	define IS_LINUX 1
#else
#	define IS_LINUX 0
#endif

#if IS_WINDOWS
typedef HANDLE Serial;
#else
typedef int Serial;
#endif

struct OptionalSerial {
	bool has_value;
	Serial serial;
};

struct SerialOptions {
	bool rtscts;
};

#if IS_WINDOWS

struct OptionalSerial Serial_open(char const* path, struct SerialOptions const* options)
{
	HANDLE fd;
	char full_name[12];

	int chars_printed = snprintf(full_name, sizeof(full_name), "\\\\.\\%s", path);
	if (chars_printed >= (int)sizeof(full_name)) {
		fprintf(stderr, "serial port name too long: '%s'\n", path);
		return (struct OptionalSerial){};
	}

	fd = CreateFileA(full_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (fd == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Failed to open serial port '%s' (%d)\n", path, GetLastError());
		return (struct OptionalSerial){};
	}

	if (!SetupComm(fd, 4096, 4096)) {
		fprintf(stderr, "Failed to set serial buffer sizes. (%d)\n", GetLastError());
		return (struct OptionalSerial){};
	}

	{
		COMMTIMEOUTS timeouts = {};
		SetCommTimeouts(fd, &timeouts);
	}

	SetCommMask(fd, EV_ERR);

	DCB tty;
	if (!GetCommState(fd, &tty)) {
		fprintf(stderr, "Failed to get serial state (%d)\n", GetLastError());
		CloseHandle(fd);
		return (struct OptionalSerial){};
	}

	tty.BaudRate = CBR_115200;
	tty.ByteSize = 8;
	tty.Parity = NOPARITY;
	tty.fParity = 0;
	tty.StopBits = ONESTOPBIT;
	tty.fBinary = 1;
	if (options->rtscts) {
		tty.fRtsControl = RTS_CONTROL_HANDSHAKE;
	} else {
		tty.fRtsControl = RTS_CONTROL_DISABLE;
	}
	tty.fOutxCtsFlow = options->rtscts ? 1 : 0;
	tty.fDtrControl = DTR_CONTROL_DISABLE;
	tty.fOutxDsrFlow = 0;
	tty.fOutX = tty.fInX = 0;
	tty.fNull = 0;
	tty.fErrorChar = 0;
	tty.fAbortOnError = 0;
	tty.XonChar = 17;
	tty.XoffChar = 19;

	if (!SetCommState(fd, &tty)) {
		fprintf(stderr, "Failed to set serial state (%d)\n", GetLastError());
		CloseHandle(fd);
		return (struct OptionalSerial){};
	}

	// Flush buffers.
	PurgeComm(fd, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR | PURGE_RXABORT);
	return (struct OptionalSerial){
		.has_value = true,
		.serial = fd,
	};
}

void Serial_close(Serial serial)
{
	CloseHandle(serial);
}

bool Serial_read(Serial serial, uint8_t* buff, size_t size)
{
	DWORD bytes_read;
	BOOL success = ReadFile(serial, buff, size, &bytes_read, NULL);
	return success && (bytes_read == size);
}

bool Serial_write(Serial serial, uint8_t const* buff, size_t size)
{
	assert(size <= SSIZE_MAX);
	DWORD bytes_written;
	BOOL success = WriteFile(serial, buff, size, &bytes_written, NULL);
	return success && (bytes_written == size);
}

#else

#if IS_MACOS
#	define CMSPAR 0
#endif

static struct termios set_termios(struct termios tty, struct SerialOptions const* options)
{
	speed_t const speed = B115200;

	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
	tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);

#	if IS_LINUX
	tty.c_iflag &= ~IUCLC;
#	endif
	tty.c_iflag &= ~PARMRK;

	tty.c_ispeed = tty.c_ospeed = speed;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~(CSTOPB);
	tty.c_iflag &= ~(INPCK | ISTRIP);
	tty.c_cflag &= ~(PARENB | PARODD | CMSPAR);

	if (options->rtscts) {
		tty.c_cflag |= CRTSCTS;
	} else {
		tty.c_cflag &= ~(CRTSCTS);
	}

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	return tty;
}

struct OptionalSerial Serial_open(char const* path, struct SerialOptions const* options)
{
	// speed_t const speed = B115200;
	int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		fprintf(stderr, "Failed to open serial port at '%s' (%d: %s)\n", path, errno, strerror(errno));
		return (struct OptionalSerial){};
	}

	struct termios tty;
	if (0 != tcgetattr(fd, &tty)) {
		fprintf(stderr, "Failed to get serial attributes (%d: %s)\n", errno, strerror(errno));
		close(fd);
		return (struct OptionalSerial){};
	}

	tty = set_termios(tty, options);

	if (0 != tcsetattr(fd, TCSANOW, &tty)) {
		close(fd);
		fprintf(stderr, "Failed to set serial settings (%d: %s)\n", errno, strerror(errno));
		return (struct OptionalSerial){};
	}

	if (options->rtscts) {
		// const int v = TIOCM_RTS;
		// set
		// ioctl(fd, TIOCMBIS, &v);
		// clear
		// ioctl(fd, TIOCMBIC, &v);
	}

	if (0 != tcflush(fd, TCIFLUSH)) {
		close(fd);
		fprintf(stderr, "Failed to flush input buffer(%d: %s)\n", errno, strerror(errno));
		return (struct OptionalSerial){};
	}

	return (struct OptionalSerial){
		.has_value = true,
		.serial = fd,
	};
}

void Serial_close(Serial serial)
{
	close(serial);
}

bool Serial_read(Serial serial, uint8_t* buff, size_t size)
{
	// Even though the fd is in blocking mode, serial is special
	// and can still return early or timeout so we must loop.
	int const fd = serial;
	uint8_t* cursor = buff;
	uint8_t const* end = buff + size;
	while (cursor < end) {
		ssize_t bytes_read = read(fd, cursor, size);
		if (bytes_read == 0) {
			fprintf(stderr, "Read timed out.\n");
			return false;
		} else if (bytes_read < 0) {
			fprintf(stderr, "Error reading %zu bytes (%d: %s)\n", size, errno, strerror(errno));
			return false;
		}
		cursor += bytes_read;
		size -= bytes_read;
	}
	return true;
}

bool Serial_write(Serial serial, uint8_t const* buff, size_t size)
{
	assert(size <= SSIZE_MAX);
	int const fd = serial;
	return (ssize_t)size == write(fd, buff, size);
}

#endif

static size_t jvs_encode_size_max(size_t unencoded_size)
{
	return unencoded_size * 2;
}

static size_t jvs_encode(void const* src, size_t size, uint8_t* dst)
{
	uint8_t* dst_cursor = dst;
	uint8_t const* src_bytes = src;
	for (size_t i = 0; i < size; i++) {
		uint8_t const b = src_bytes[i];
		if (b == 0xd0 || b == 0xe0) {
			*dst_cursor = 0xd0;
			dst_cursor++;
			*dst_cursor = b - 1;
		} else {
			*dst_cursor = b;
		}
		dst_cursor++;
	}
	return dst_cursor - dst;
}

static uint8_t jvs_checksum(void const* buff, size_t size, uint8_t starting_value)
{
	uint8_t const* bytes = buff;
	uint8_t sum = starting_value;
	for (size_t i = 0; i < size; i++) {
		sum += bytes[i];
	}
	return sum;
}

struct Color
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};
// Just ensure the compiler has laid out the bytes as though it was a plain byte array.
_Static_assert(sizeof(struct Color) == 3, "Color struct has unexpected size.");

struct OptionalColor
{
	bool has_value;
	struct Color color;
};

static struct OptionalColor Color_from_string(char const* str)
{
	struct Color color;
	char const* cursor = str;
	char* end;
	long value;

	value = strtol(cursor, &end, 0);
	if (end == cursor) {
		fprintf(stderr, "Could not parse red channel in color string '%s'\n", str);
		return (struct OptionalColor){};
	}
	if (value < 0 || value > 255) {
		fprintf(stderr, "Red color is out of range must be 0-255. Got %ld\n", value);
		return (struct OptionalColor){};
	}
	if (*end != ',') {
		fprintf(stderr, "Expected ',' after red color but got '%c'.\n", *end);
		return (struct OptionalColor){};
	}
	color.r = value;
	cursor = end + 1;

	value = strtol(cursor, &end, 0);
	if (end == cursor) {
		fprintf(stderr, "Could not parse green channel in color string '%s'\n", str);
		return (struct OptionalColor){};
	}
	if (value < 0 || value > 255) {
		fprintf(stderr, "Green color is out of range must be 0-255. Got %ld\n", value);
		return (struct OptionalColor){};
	}
	if (*end != ',') {
		fprintf(stderr, "Expected ',' after green color but got '%c'.\n", *end);
		return (struct OptionalColor){};
	}
	color.g = value;
	cursor = end + 1;

	value = strtol(cursor, &end, 0);
	if (end == cursor) {
		fprintf(stderr, "Could not parse blue channel in color string '%s'\n", str);
		return (struct OptionalColor){};
	}
	if (value < 0 || value > 255) {
		fprintf(stderr, "Blue color is out of range must be 0-255. Got %ld\n", value);
		return (struct OptionalColor){};
	}
	if (*end != '\0') {
		fprintf(stderr, "Unexpected data after color: '%s'.\n", end);
		return (struct OptionalColor){};
	}
	color.b= value;

	return (struct OptionalColor){
		.has_value = true,
		.color = color,
	};
}

static void Color_test(void)
{
	struct OptionalColor c;
	c = Color_from_string("0,");
	assert(!c.has_value);
	c = Color_from_string("0,0");
	assert(!c.has_value);
	c = Color_from_string("0,0,");
	assert(!c.has_value);
	c = Color_from_string("0,0,a");
	assert(!c.has_value);
	c = Color_from_string("0,0,0");
	assert(c.has_value && c.color.r == 0 && c.color.g == 0 && c.color.b == 0);
	c = Color_from_string("1,2,3");
	assert(c.has_value && c.color.r == 1 && c.color.g == 2 && c.color.b == 3);
	c = Color_from_string("1,2,256");
	assert(!c.has_value);
	c = Color_from_string("256,2,2");
	assert(!c.has_value);
	c = Color_from_string("2,256,2");
	assert(!c.has_value);
	c = Color_from_string("0xff,0xff,0xff");
	assert(c.has_value && c.color.r == 0xff && c.color.g == 0xff && c.color.b == 0xff);
}

struct ArgsLED
{
	char const* port;
	struct Color left;
	struct Color right;
	struct Color center;
};

static bool ArgsLED_parse(struct ArgsLED* args, int argc, char** argv)
{
	*args = (struct ArgsLED){};
	if (IS_WINDOWS) {
		args->port = "COM2";
	} else {
		args->port = "/dev/ttyS1";
	}
	struct option const longopts[] = {
		{"port",   required_argument, NULL, 'p'},
		{"color",  required_argument, NULL, 'c'},
		{"left",   required_argument, NULL, '<'},
		{"right",  required_argument, NULL, '>'},
		{"center", required_argument, NULL, '^'},
		{},
	};
	struct OptionalColor maybe_color = {};
	struct OptionalColor maybe_left = {};
	struct OptionalColor maybe_right = {};
	struct OptionalColor maybe_center = {};
	int c;
	optind = 1;
	while (-1 != (c = getopt_long(argc, argv, "", longopts, NULL))) {
		switch (c) {
			case 'p': {
				args->port = optarg;
			} break;
			case 'c': {
				maybe_color = Color_from_string(optarg);
				if (!maybe_color.has_value) {
					fprintf(stderr, "Could not parse color for --color '%s'\n", optarg);
					return false;
				}
			} break;
			case '<': {
				maybe_left = Color_from_string(optarg);
				if (!maybe_left.has_value) {
					fprintf(stderr, "Could not parse color for --left '%s'\n", optarg);
					return false;
				}
			} break;
			case '>': {
				maybe_right = Color_from_string(optarg);
				if (!maybe_right.has_value) {
					fprintf(stderr, "Could not parse color for --right '%s'\n", optarg);
					return false;
				}
			} break;
			case '^': {
				maybe_center = Color_from_string(optarg);
				if (!maybe_center.has_value) {
					fprintf(stderr, "Could not parse color for --center '%s'\n", optarg);
					return false;
				}
			} break;
			default: {
				fprintf(stderr, "Unknown option '%s'\n", argv[optopt]);
				return false;
			} break;
		}
	}
	// Use the main color value to set the sections if they weren't given a value explicitly.
	if (maybe_color.has_value) {
		if (!maybe_left.has_value) {
			maybe_left = maybe_color;
		}
		if (!maybe_right.has_value) {
			maybe_right = maybe_color;
		}
		if (!maybe_center.has_value) {
			maybe_center = maybe_color;
		}
	}
	if (!maybe_left.has_value || !maybe_right.has_value || !maybe_center.has_value) {
		fprintf(stderr, "colors for all sides not specified. use '--color' to give a default\n");
		return false;
	}
	args->left = maybe_left.color;
	args->right = maybe_right.color;
	args->center = maybe_center.color;
	return true;
}

static void print_usage_led(void)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "pras3 led [--port path] [--color x,x,x] [--left x,x,x] [--right x,x,x] [--center x,x,x]\n");
	fprintf(stderr, "--port <path>                  The serial port path.\n");
	fprintf(stderr, "--color <0-255,0-255,0-255>    Default color to set all LEDs to.\n");
	fprintf(stderr, "--left <0-255,0-255,0-255>     The color to set the left side LEDs to.\n");
	fprintf(stderr, "--right <0-255,0-255,0-255>    The color to set the right side LEDs to.\n");
	fprintf(stderr, "--center <0-255,0-255,0-255>   The color to set the right side LEDs to.\n");
}

struct LEDPage {
	struct Color pixels[22];
};

void LEDPage_set(struct LEDPage* page, struct Color left, struct Color center, struct Color right)
{
	// Maps pixels laied out in order of left-center-right to the in-memory placement.
	static const uint8_t location_to_mem[22] = {
		// 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21
		   3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18,  0,  1,  2, 19, 20, 21,
	};
	for (size_t i = 0; i < 6; i++) {
		page->pixels[location_to_mem[i]] = left;
	}
	for (size_t i = 6; i < 6 + 12; i++) {
		page->pixels[location_to_mem[i]] = center;
	}
	for (size_t i = 6 + 12; i < 22; i++) {
		page->pixels[location_to_mem[i]] = right;
	}
}

struct LEDDisplay {
	struct LEDPage pages[3];
};

static bool LED_cmd_send(Serial serial, uint8_t cmd_id, void const* buff, size_t size)
{
	fprintf(stderr, "sending...\n");
	assert(buff);
	assert(size <= UINT8_MAX);

	struct LEDCmd {
		uint8_t dst_node_id;
		uint8_t src_node_id;
		uint8_t payload_len;
		uint8_t command;
	};

	uint8_t const sync = 0xe0;
	uint8_t checksum;
	struct LEDCmd const cmd = {
		.dst_node_id = 0,
		.src_node_id = 0,
		.payload_len = (uint8_t)size + sizeof(checksum),
		.command = cmd_id,
	};

	checksum = jvs_checksum(&cmd, sizeof(cmd), 0);
	checksum = jvs_checksum(buff, size, checksum);

	uint8_t encoded_buf[sizeof(sync) + 2 * (sizeof(cmd) + UINT8_MAX + sizeof(checksum))];
	assert(jvs_encode_size_max(sizeof(cmd) + sizeof(struct LEDDisplay) + sizeof(checksum)) <= sizeof(encoded_buf));
	uint8_t* cursor = encoded_buf;
	*cursor = sync;
	cursor++;
	cursor += jvs_encode(&cmd, sizeof(cmd), cursor);
	cursor += jvs_encode(buff, size, cursor);
	cursor += jvs_encode(&checksum, sizeof(checksum), cursor);
	size_t const encoded_size = cursor - encoded_buf;

	if (!Serial_write(serial, encoded_buf, encoded_size)) {
		fprintf(stderr, "Failed to write %zu bytes for cmd %" PRIu8 "\n", encoded_size, cmd_id);
		return false;
	}
	return true;
}

static bool LED_fade_to_display(Serial serial, struct LEDDisplay const* display)
{
	return LED_cmd_send(serial, 0x83, display, sizeof(*display));
}

static int run_tool_led(int argc, char** argv)
{
	struct ArgsLED args;
	if (!ArgsLED_parse(&args, argc, argv)) {
		print_usage_led();
		return 1;
	}
	struct SerialOptions const options = {};
	struct OptionalSerial maybe_serial = Serial_open(args.port, &options);
	if (!maybe_serial.has_value) {
		fprintf(stderr, "Failed to open LED serial port at '%s'\n", args.port);
		return 1;
	}
	Serial serial = maybe_serial.serial;
	struct LEDDisplay display;
	LEDPage_set(&display.pages[0], args.left, args.center, args.right);
	LEDPage_set(&display.pages[1], args.left, args.center, args.right);
	LEDPage_set(&display.pages[2], args.left, args.center, args.right);
	if (!LED_fade_to_display(serial, &display)) {
		fprintf(stderr, "Failed to fade to LED display\n");
		Serial_close(serial);
		return 1;
	}
	Serial_close(serial);
	return 0;
}

static void print_usage(void)
{
	fprintf(stderr, "pras3 <led|nfc|vfd|test> [options...]\n");
	print_usage_led();
}

static int run_test(void)
{
	Color_test();
	return 0;
}

int main(int argc, char* argv[])
{
	if (argc < 2) {
		fprintf(stderr, "Missing tool name\n");
		print_usage();
		return 1;
	}
	char const* tool = argv[1];
	if (0 == strcmp(tool, "test")) {
		return run_test();
	} else if (0 == strcmp(tool, "led")) {
		return run_tool_led(argc, argv);
	}

	fprintf(stderr, "Unknown tool name '%s'\n", tool);
	return 1;
}
