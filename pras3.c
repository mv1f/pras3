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
#include <time.h>
#include <limits.h>
#include <sys/types.h>
#include <unistd.h>

#define TEST(...) if (!(__VA_ARGS__)) { \
	fprintf(stderr, "%s(%d): %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
	abort(); \
}

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

static void sleep_ns(uint64_t nanoseconds)
{
	// This won't be nanosecond-accurate but we don't really care.
	DWORD const milliseconds = nanoseconds / UINT64_C(1000000);
	Sleep(milliseconds);
}

static uint64_t now_ns(void)
{
	LARGE_INTEGER frequency;
	(void)QueryPerformanceFrequency(&frequency);

	LARGE_INTEGER count;
	(void)QueryPerformanceCounter(&count);

	uint64_t const ns_per_second = UINT64_C(1000000000);
	uint64_t const seconds = count.QuadPart / frequency.QuadPart;
	uint64_t const nanoseconds = ((count.QuadPart % frequency.QuadPart) * ns_per_second) / frequency.QuadPart;

	return seconds * ns_per_second + nanoseconds;
}

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

bool Serial_read(Serial serial, void* buff, size_t size)
{
	DWORD bytes_read;
	BOOL success = ReadFile(serial, buff, size, &bytes_read, NULL);
	return success && (bytes_read == size);
}

bool Serial_write(Serial serial, void const* buff, size_t size)
{
	assert(size <= SSIZE_MAX);
	DWORD bytes_written;
	BOOL success = WriteFile(serial, buff, size, &bytes_written, NULL);
	return success && (bytes_written == size);
}

#else

static void sleep_ns(uint64_t nanoseconds)
{
	uint64_t const ns_per_second = UINT64_C(1000000000);
	struct timespec const t = {
		.tv_sec = nanoseconds / ns_per_second,
		.tv_nsec = nanoseconds % ns_per_second,
	};
	(void)nanosleep(&t, NULL);
}

static uint64_t now_ns(void)
{
	struct timespec t;
	(void)clock_gettime(CLOCK_MONOTONIC_RAW, &t);
	uint64_t const ns_per_second = UINT64_C(1000000000);
	return t.tv_sec * ns_per_second + t.tv_nsec;
}

#if IS_MACOS
#	define CMSPAR 0
#endif

static struct termios set_termios(struct termios tty, struct SerialOptions const* options)
{
	speed_t const speed = B115200;

	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN | ECHOCTL | ECHOKE);
	tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);

#	if IS_LINUX
	tty.c_iflag &= ~IUCLC;
#	endif
	tty.c_iflag &= ~PARMRK;

	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~(CSTOPB);
	tty.c_iflag &= ~(INPCK | ISTRIP);
	tty.c_cflag &= ~(PARENB | PARODD | CMSPAR);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

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

bool Serial_read(Serial serial, void* buff, size_t size)
{
	// Even though the fd is in blocking mode, serial is special
	// and can still return early or timeout so we must loop.
	int const fd = serial;
	fd_set read_fds;
	uint8_t* cursor = buff;
	uint8_t const* end = buff + size;
	while (cursor < end) {
		FD_ZERO(&read_fds);
		FD_SET(fd, &read_fds);
		struct timeval timeout = {
			.tv_sec = 1,
		};
		int ready_count = select(fd + 1, &read_fds, NULL, NULL, &timeout);
		if (ready_count < 0) {
			fprintf(stderr, "Failed to wait for serial port to be ready to read: (%d: %s)\n", errno, strerror(errno));
			return false;
		}
		if (ready_count == 0 || !FD_ISSET(fd, &read_fds)) {
			continue;
		}
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

bool Serial_write(Serial serial, void const* buff, size_t size)
{
	assert(size <= SSIZE_MAX);
	int const fd = serial;
	return (ssize_t)size == write(fd, buff, size);
}

#endif

__attribute__((unused))
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

static bool jvs_read_encoded(Serial serial, void* out_buff, size_t size)
{
	uint8_t* cursor = out_buff;
	while (size) {
		uint8_t byte;
		if (!Serial_read(serial, &byte, sizeof(byte))) {
			fprintf(stderr, "Failed to read JVS byte.\n");
			return false;
		}
		if (byte == 0xd0) {
			if (!Serial_read(serial, &byte, sizeof(byte))) {
				fprintf(stderr, "Failed to read escaped JVS byte.\n");
				return false;
			}
			byte++;
		}
		if (cursor) {
			*cursor = byte;
			cursor++;
		}
		size--;
	}
	return true;
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
	TEST(!c.has_value);
	c = Color_from_string("0,0");
	TEST(!c.has_value);
	c = Color_from_string("0,0,");
	TEST(!c.has_value);
	c = Color_from_string("0,0,a");
	TEST(!c.has_value);
	c = Color_from_string("0,0,0");
	TEST(c.has_value && c.color.r == 0 && c.color.g == 0 && c.color.b == 0);
	c = Color_from_string("1,2,3");
	TEST(c.has_value && c.color.r == 1 && c.color.g == 2 && c.color.b == 3);
	c = Color_from_string("1,2,256");
	TEST(!c.has_value);
	c = Color_from_string("256,2,2");
	TEST(!c.has_value);
	c = Color_from_string("2,256,2");
	TEST(!c.has_value);
	c = Color_from_string("0xff,0xff,0xff");
	TEST(c.has_value && c.color.r == 0xff && c.color.g == 0xff && c.color.b == 0xff);
}

#define BMP_MAX_IMAGE_BYTES(w,h) ((w)*(h)*4)
#define BMP_BYTES_PER_PIXEL 3
#define BMP_ROW_STRIDE(w,bpp) (4 * (((w) * ((bpp)/8) + (4 - 1)) / 4))

static bool bmp_load_(char const* path, int expected_w, int expected_h, void* image_bytes, uint16_t* bpp)
{
	assert(image_bytes);
	assert(bpp);
	FILE* f = fopen(path, "rb");
	if (f == NULL) {
		fprintf(stderr, "failed to open image '%s'.\n", path);
		return false;
	}
	ssize_t bytes_read;

	struct BMPHeader {
		char magic[2];
		uint32_t size;
		uint32_t pad_;
		uint32_t image_data_offset;
		struct {
			uint32_t size;
			int32_t width;
			int32_t height;
			uint16_t planes;
			uint16_t bpp;
			uint32_t compression;
			uint32_t image_size;
			uint32_t horizontal_resolution;
			uint32_t vertical_resolution;
			uint32_t colors_in_palette;
			uint32_t important_colors;
		} bitmap_info;
	} __attribute__((__packed__));

	struct BMPHeader header;
	bytes_read = fread(&header, 1, sizeof(header), f);
	if (bytes_read != sizeof(header)) {
		fprintf(stderr, "faild to read BMP header.\n");
		return false;
	}
	if (header.bitmap_info.bpp != 24 && header.bitmap_info.bpp != 32) {
		fclose(f);
		fprintf(stderr, "BMP is %" PRIu16 " bpp but only RGB (24bpp) and ARGB (32bpp) images are supported.\n", header.bitmap_info.bpp);
		return false;
	}
	*bpp = header.bitmap_info.bpp;
	int const row_size = BMP_ROW_STRIDE(header.bitmap_info.width, header.bitmap_info.bpp);

	int const w = header.bitmap_info.width;
	int const h = header.bitmap_info.height < 0 ? -header.bitmap_info.height : header.bitmap_info.height;
	if (w != expected_w || h != expected_h) {
		fclose(f);
		fprintf(stderr, "Image is %d x %d but it must be %d x %d\n", w, h, expected_w, expected_h);
		return false;
	}
	size_t const total_image_bytes = row_size * h;
	// Seek to the image bytes.
	{
		int err = fseek(f, header.image_data_offset, SEEK_SET);
		(void)err;
		assert(err == 0);
	}
	if (header.bitmap_info.height < 0) {
		// If the height is negative, the rows are already in the expected order.
		bytes_read = fread(image_bytes, 1, total_image_bytes, f);
		if (bytes_read != (ssize_t)total_image_bytes) {
			fclose(f);
			fprintf(stderr, "Faild to read %zu image bytes.\n", total_image_bytes);
			return false;
		}
	} else {
		// If the height is positive we need to flip the rows.
		uint8_t* row = image_bytes + row_size * (h - 1);
		for (int row_i = 0; row_i < h; row_i++) {
			bytes_read = fread(row, 1, row_size, f);
			if (bytes_read != (ssize_t)total_image_bytes) {
				fclose(f);
				fprintf(stderr, "Faild to read %zu image bytes from row %d.\n", total_image_bytes, row_i);
				return false;
			}
			row -= row_size;
		}
	}

	fclose(f);
	return true;
}

struct ArgsLED
{
	char const* port;
	struct Color left;
	struct Color right;
	struct Color center;
	char const* image_path;
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
		{"image",  required_argument, NULL, 'i'},
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
			case 'i': {
				args->image_path = optarg;
			} break;
			default: {
				fprintf(stderr, "Unknown option '%s'\n", argv[optopt]);
				return false;
			} break;
		}
	}
	if (args->image_path) {
		if (maybe_color.has_value) {
			fprintf(stderr, "You can't use --color with --image.\n");
			return false;
		}
		if (maybe_left.has_value) {
			fprintf(stderr, "You can't use --left with --image.\n");
			return false;
		}
		if (maybe_right.has_value) {
			fprintf(stderr, "You can't use --right with --image.\n");
			return false;
		}
		if (maybe_center.has_value) {
			fprintf(stderr, "You can't use --center with --image.\n");
			return false;
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
	if (args->image_path == NULL) {
		if (!maybe_left.has_value || !maybe_right.has_value || !maybe_center.has_value) {
			fprintf(stderr, "colors for all sides not specified. use '--color' to give a default\n");
			return false;
		}
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
	fprintf(stderr, "--image <path>                 Path to a 22 x 1 RGB BMP image to set pixel colors with.\n");
}

struct LEDPage {
	struct Color pixels[22];
};

struct LEDImage {
	uint8_t p[22 * 3];
};

static bool LEDImage_load_(struct LEDImage* image, char const* path)
{
	uint16_t bpp;
	uint8_t image_bytes[22 * 4] = {};
	if (!bmp_load_(path, 22, 1, image_bytes, &bpp)) {
		fprintf(stderr, "failed to load bmp for LEDs.\n");
		return false;
	}
	// Convert to the format we expect.
	if (bpp == 24 || bpp == 32) {
		uint16_t bytes_per_pixel = bpp / 8;
		for (size_t i = 0; i < 22; i++) {
			uint8_t* from_p = &image_bytes[i * bytes_per_pixel];
			uint8_t* to_p = &image->p[i * 3];
			// BGR to RGB order.
			to_p[0] = from_p[2];
			to_p[1] = from_p[1];
			to_p[2] = from_p[0];
		}
	} else {
		fprintf(stderr, "Unexpected BPP for LED image: %" PRIu16 "\n", bpp);
		return false;
	}
	return true;
}

// Maps pixels laied out in order of left-center-right to the in-memory placement.
static uint8_t const led_location_to_mem[22] = {
	//  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21
	   16, 17, 18,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 19, 20, 21
};

void LEDPage_image_set(struct LEDPage* page, struct LEDImage const* image)
{
	for (size_t i = 0; i < 22; i++) {
		uint8_t const* const p = &image->p[i * 3];
		page->pixels[led_location_to_mem[i]] = (struct Color){ p[0], p[1], p[2] };
	}
}

void LEDPage_set(struct LEDPage* page, struct Color left, struct Color center, struct Color right)
{
	for (size_t i = 0; i < 6; i++) {
		page->pixels[led_location_to_mem[i]] = left;
	}
	for (size_t i = 6; i < 6 + 10; i++) {
		page->pixels[led_location_to_mem[i]] = center;
	}
	for (size_t i = 6 + 10; i < 22; i++) {
		page->pixels[led_location_to_mem[i]] = right;
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
	if (args.image_path) {
		struct LEDImage image;
		if (!LEDImage_load_(&image, args.image_path)) {
			fprintf(stderr, "Failed to load LEDImage from '%s'.\n", args.image_path);
			return 1;
		}
		LEDPage_image_set(&display.pages[0], &image);
		LEDPage_image_set(&display.pages[1], &image);
		LEDPage_image_set(&display.pages[2], &image);
	} else {
		LEDPage_set(&display.pages[0], args.left, args.center, args.right);
		LEDPage_set(&display.pages[1], args.left, args.center, args.right);
		LEDPage_set(&display.pages[2], args.left, args.center, args.right);
	}
	if (!LED_fade_to_display(serial, &display)) {
		fprintf(stderr, "Failed to fade to LED display\n");
		Serial_close(serial);
		return 1;
	}
	Serial_close(serial);
	return 0;
}

static void print_usage_nfc(void)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "pras3 nfc [--port path] [--color x,x,x] [--wait_for_specific <hex UID>] [--wait_for_any <count>] [--timeout <seconds>]\n");
	fprintf(stderr, "--port <path>                  The serial port path.\n");
	fprintf(stderr, "--color <0-255,0-255,0-255>    Default color to set all LEDs to.\n");
	fprintf(stderr, "--wait_for_specific <hex uid>  The hex representation of a specific UID of a\n");
	fprintf(stderr, "                               card to wait for.\n");
	fprintf(stderr, "--wait_for_any <count>         Wait for and print <count> unique card UIDs\n");
	fprintf(stderr, "                               before ending.\n");
	fprintf(stderr, "--timeout <seconds>            Stop unconditionally after waiting for <seconds>.\n");
	fprintf(stderr, "                               Default is 0 which will wait forever.\n");
}

struct UID {
	uint8_t bytes[16];
	uint8_t length;
};

bool UID_is_equal(struct UID const* lhs, struct UID const* rhs)
{
	if (lhs->length != rhs->length) {
		return false;
	}
	return 0 == memcmp(lhs->bytes, rhs->bytes, lhs->length);
}

struct UIDString {
	char chars[sizeof(((struct UID*)0)->bytes) * 2 + 1];
};

struct UIDString UIDString_from_UID(struct UID const* uid)
{
	static char const nibble_to_char[16] = {
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
	};
	struct UIDString result = {};
	char* cursor = result.chars;
	for (size_t i = 0; i < uid->length; i++) {
		cursor[0] = nibble_to_char[(0xf0 & uid->bytes[i]) >> 4];
		cursor[1] = nibble_to_char[(0x0f & uid->bytes[i]) >> 0];
		cursor += 2;
	}
	*cursor = '\0';
	return result;
}

struct OptionalUID {
	bool has_value;
	struct UID uid;
};

struct OptionalUID UID_from_string(char const* str)
{
	struct OptionalUID maybe = {};
	char const* cursor = str;
	static uint8_t const char_to_hex[0xff] = {
		['0'] = 0x0,
		['1'] = 0x1,
		['2'] = 0x2,
		['3'] = 0x3,
		['4'] = 0x4,
		['5'] = 0x5,
		['6'] = 0x6,
		['7'] = 0x7,
		['8'] = 0x8,
		['9'] = 0x9,
		['a'] = 0xa,
		['b'] = 0xb,
		['c'] = 0xc,
		['d'] = 0xd,
		['e'] = 0xe,
		['f'] = 0xf,
	};

	size_t char_count = 0;
	uint8_t value = 0;
	size_t const max_uid_len = sizeof(maybe.uid.bytes);
	while (*cursor != '\0' && char_count < max_uid_len * 2) {
		uint8_t const nibble = char_to_hex[(uint8_t)*cursor];
		if (nibble == 0 && *cursor != '0') {
			fprintf(stderr, "Bad character in hex string: '%c'\n", *cursor);
			return (struct OptionalUID){};
		}
		value = (value << 4) | nibble;
		if (char_count & 1) {
			maybe.uid.bytes[maybe.uid.length++] = value;
			value = 0;
		}
		cursor++;
		char_count++;
	}
	if (*cursor != '\0') {
		fprintf(stderr, "UID string is too long. Max length is %zu bytes.\n", max_uid_len);
		return maybe;
	}
	if (char_count & 1) {
		fprintf(stderr, "Odd number of hex characters found but an even number are required\n");
		return maybe;
	} else {
		maybe.has_value = true;
	}
	return maybe;
}

static bool UID_is_in_array(struct UID const* seen_uids, size_t count, struct UID const* uid)
{
	for (size_t i = 0; i < count; i++) {
		if (UID_is_equal(&seen_uids[i], uid)) {
			return true;
		}
	}
	return false;
}

void UID_test(void)
{
	struct OptionalUID maybe_uid;
	// Bad strings.
	// Invalid characters.
	maybe_uid = UID_from_string("xx");
	TEST(!maybe_uid.has_value);
	// Odd number of chars.
	maybe_uid = UID_from_string("1");
	TEST(!maybe_uid.has_value);
	// No leading '0x'.
	maybe_uid = UID_from_string("0xff");
	TEST(!maybe_uid.has_value);
	// Too long.
	maybe_uid = UID_from_string("00112233445566778899aabbccddeeff00");
	TEST(!maybe_uid.has_value);

	// Good strings.
	maybe_uid = UID_from_string("ff");
	TEST(0 == strcmp("ff", UIDString_from_UID(&maybe_uid.uid).chars));
	maybe_uid = UID_from_string("001122aabbcc");
	TEST(0 == strcmp("001122aabbcc", UIDString_from_UID(&maybe_uid.uid).chars));
	maybe_uid = UID_from_string("00112233445566778899aabbccddeeff");
	TEST(0 == strcmp("00112233445566778899aabbccddeeff", UIDString_from_UID(&maybe_uid.uid).chars));
}

struct ArgsNFC {
	char const* port;
	struct OptionalColor color;
	int wait_for_any;
	struct OptionalUID wait_for_specific;
	int timeout;
};

static bool ArgsNFC_parse(struct ArgsNFC* args, int argc, char** argv)
{
	*args = (struct ArgsNFC){};
	if (IS_WINDOWS) {
		args->port = "COM3";
	} else {
		args->port = "/dev/ttyS2";
	}
	struct option const longopts[] = {
		{"port",              required_argument, NULL, 'p'},
		{"color",             required_argument, NULL, 'c'},
		{"wait_for_specific", required_argument, NULL, 's'},
		{"wait_for_any",      required_argument, NULL, 'a'},
		{"timeout",           required_argument, NULL, 't'},
		{},
	};
	int c;
	optind = 1;
	while (-1 != (c = getopt_long(argc, argv, "", longopts, NULL))) {
		switch (c) {
			case 'p': {
				args->port = optarg;
			} break;
			case 'c': {
				args->color = Color_from_string(optarg);
				if (!args->color.has_value) {
					fprintf(stderr, "Could not parse color for --color '%s'\n", optarg);
					return false;
				}
			} break;
			case 's': {
				args->wait_for_specific = UID_from_string(optarg);
				if (!args->wait_for_specific.has_value) {
					fprintf(stderr, "Could not parse UID value: '%s'\n", optarg);
					return false;
				}
			} break;
			case 'a': {
				char* end;
				args->wait_for_any = strtol(optarg, &end, 0);
				if (end != (optarg + strlen(optarg))) {
					fprintf(stderr, "Could not parse count for wait_for_any: '%s'\n", optarg);
					return false;
				}
			} break;
			case 't': {
				char* end;
				args->timeout = strtol(optarg, &end, 0);
				if (end != (optarg + strlen(optarg))) {
					fprintf(stderr, "Could not parse timeout value: '%s'\n", optarg);
					return false;
				}
			} break;
			default: {
				fprintf(stderr, "Unknown option '%s'\n", argv[optopt]);
				return false;
			} break;
		}
	}
	if (args->wait_for_specific.has_value && args->wait_for_any > 0) {
		fprintf(stderr, "--wait_for_any and --wait_for_specific are mutually exclusive. You must only use one.\n");
		return false;
	}
	if (args->wait_for_any < 0) {
		fprintf(stderr, "--wait_for_any must not be negative.\n");
		return false;
	}
	return true;
}

struct NFC {
	Serial serial;
	uint8_t seqence;
};

static bool NFC_cmd_send_(struct NFC* nfc, uint8_t cmd_id, void const* buff, size_t size)
{
	nfc->seqence++;

	uint8_t checksum;
	struct NFCHeader {
		uint8_t addr;
		uint8_t sequence;
		uint8_t command;
		uint8_t payload_length;
	} const header = {
		.addr = 0,
		.sequence = nfc->seqence,
		.command = cmd_id,
		.payload_length = size,
	};
	uint8_t const buffer_length = sizeof(header) + size + sizeof(checksum);

	checksum = jvs_checksum(&buffer_length, sizeof(buffer_length), 0);
	checksum = jvs_checksum(&header, sizeof(header), checksum);
	checksum = jvs_checksum(buff, size, checksum);

	uint8_t encoded_buff[2 * (sizeof(header) + 255 + sizeof(checksum))];

	encoded_buff[0] = 0xe0;
	uint8_t* cursor = encoded_buff + 1;
	cursor += jvs_encode(&buffer_length, sizeof(buffer_length), cursor);
	cursor += jvs_encode(&header, sizeof(header), cursor);
	cursor += jvs_encode(buff, size, cursor);
	cursor += jvs_encode(&checksum, sizeof(checksum), cursor);
	size_t encoded_size = cursor - encoded_buff;

	if (!Serial_write(nfc->serial, encoded_buff, encoded_size)) {
		fprintf(stderr, "Failed to write NFC command to serial\n");
		return false;
	}
	return true;
}

static bool NFC_response_recv_(Serial serial, void* out_payload, size_t* out_size)
{
	uint8_t sync;
	if (!Serial_read(serial, &sync, sizeof(sync))) {
		fprintf(stderr, "Failed to read sync byte\n");
		return false;
	}
	if (sync != 0xe0) {
		fprintf(stderr, "Unexpected sync value 0x%02x\n", sync);
		return false;
	}
	uint8_t size;
	if (!jvs_read_encoded(serial, &size, sizeof(size))) {
		fprintf(stderr, "Failed to read response size\n");
		return false;
	}
	struct NFCResponseHeader {
		uint8_t addr;
		uint8_t seq;
		uint8_t command;
		uint8_t status;
		uint8_t payload_length;
	} header;
	if (!jvs_read_encoded(serial, &header, sizeof(header))) {
		fprintf(stderr, "Failed to read NFC response header.\n");
		return false;
	}
	if (out_size) {
		*out_size = header.payload_length;
	}
	if (!jvs_read_encoded(serial, out_payload, header.payload_length)) {
		fprintf(stderr, "Failed to read NFC response payload.\n");
		return false;
	}
	uint8_t checksum;
	if (!jvs_read_encoded(serial, &checksum, sizeof(checksum))) {
		fprintf(stderr, "Failed to read NFC response checksum.\n");
		return false;
	}
	// We're ignoring the checksum.
	return header.status == 0;
}

enum NFCCardType
{
	kNFCCardType_MIFARE = 1,
	kNFCCardType_FeliCa = 2,
};

static bool NFC_reset(struct NFC* nfc)
{
	uint8_t const cmd = 0x62;
	if (!NFC_cmd_send_(nfc, cmd, NULL, 0)) {
		fprintf(stderr, "Failed to send NFC 'reset' command.\n");
		return false;
	}
	return NFC_response_recv_(nfc->serial, NULL, NULL);
}

static bool NFC_led_get_info(struct NFC* nfc)
{
	uint8_t const cmd = 0xf0;
	if (!NFC_cmd_send_(nfc, cmd, NULL, 0)) {
		fprintf(stderr, "Failed to send NFC 'get info' command.\n");
		return false;
	}
	// We don't really care about the payload at the moment, but it does have one.
	return NFC_response_recv_(nfc->serial, NULL, 0);
}

static bool NFC_led_set_color(struct NFC* nfc, struct Color color)
{
	uint8_t const cmd = 0x81;
	// There is no response to process.
	return NFC_cmd_send_(nfc, cmd, &color, sizeof(color));
}

static bool NFC_radio_on(struct NFC* nfc, uint8_t card_type)
{
	assert(card_type == kNFCCardType_MIFARE || card_type == kNFCCardType_FeliCa);
	uint8_t const cmd = 0x40;
	if (!NFC_cmd_send_(nfc, cmd, &card_type, sizeof(card_type))) {
		fprintf(stderr, "Failed to send 'radio on' command.\n");
		return false;
	}
	return NFC_response_recv_(nfc->serial, NULL, 0);
}

struct NFCCard {
	uint8_t type;
	struct UID uid;
};

struct NFCCardIterator {
	uint8_t bytes[256];
	uint8_t const* cursor;
	uint8_t const* end;
	uint8_t count;
};

static bool NFCCardIterator_next(struct NFCCardIterator* iter, struct NFCCard* card)
{
	if (iter->count == 0) {
		return false;
	}
	uint8_t type;
	memcpy(&type, iter->cursor, sizeof(type));
	iter->cursor += sizeof(type);
	card->type = (type >> 4) & 0xf;
	if (iter->cursor == iter->end) {
		iter->count = 0;
		return false;
	}

	uint8_t size;
	memcpy(&size, iter->cursor, sizeof(size));
	iter->cursor += sizeof(size);
	card->uid.length = size;
	if (iter->cursor + size > iter->end) {
		iter->count = 0;
		return false;
	} else if (size > sizeof(card->uid.bytes)) {
		fprintf(stderr, "Unexpected UID size: %" PRIu8 "\n", size);
		iter->count = 0;
		return false;
	}

	memcpy(card->uid.bytes, iter->cursor, size);
	iter->cursor += size;

	iter->count--;
	return true;
}

static bool NFC_poll(struct NFC* nfc, struct NFCCardIterator* iter)
{
	// Don't poll too fast.
	sleep_ns(150000000);
	uint8_t const cmd = 0x42;
	if (!NFC_cmd_send_(nfc, cmd, NULL, 0)) {
		fprintf(stderr, "Failed to send NFC poll command.\n");
		return false;
	}
	size_t size;
	if (!NFC_response_recv_(nfc->serial, iter->bytes, &size)) {
		fprintf(stderr, "Failed to receive NFC poll response.\n");
		return false;
	}
	iter->cursor = iter->bytes;
	iter->end = iter->bytes + size;
	memcpy(&iter->count, iter->cursor, sizeof(iter->count));
	iter->cursor += sizeof(iter->count);
	if (iter->cursor >= iter->end) {
		iter->count = 0;
	}
	return true;
}

static bool timeout_(uint64_t start_ns, int seconds_to_wait)
{
	if (seconds_to_wait <= 0) {
		// Wait forever...
		return false;
	}
	uint64_t const ns_per_second = UINT64_C(1000000000);
	uint64_t const ns_to_wait = (uint64_t)seconds_to_wait * ns_per_second;
	uint64_t const end_ns = start_ns + ns_to_wait;

	return now_ns() >= end_ns;
}

static int run_tool_nfc(int argc, char** argv)
{
	struct ArgsNFC args;
	if (!ArgsNFC_parse(&args, argc, argv)) {
		print_usage_nfc();
		return 1;
	}

	struct SerialOptions options = {};
	struct OptionalSerial maybe_serial = Serial_open(args.port, &options);
	if (!maybe_serial.has_value) {
		fprintf(stderr, "Failed to open NFC serial connection.\n");
		return false;
	}
	struct NFC nfc = {
		.serial = maybe_serial.serial,
	};
	// If we've already initialized the NFC device once before then the status will return an error
	// so we just ignore it.
	(void)NFC_reset(&nfc);

	if (args.color.has_value) {
		if (!NFC_led_get_info(&nfc)) {
			fprintf(stderr, "Failed to get NFC LED info.\n");
			return false;
		}
		if (!NFC_led_set_color(&nfc, args.color.color)) {
			fprintf(stderr, "Failed to set NFC LED color.\n");
			return false;
		}
	}

	if (args.wait_for_any > 0 || args.wait_for_specific.has_value) {
		struct UID* seen_uids = NULL;
		int seen_uid_count = 0;
		if (args.wait_for_any) {
			seen_uids = malloc(args.wait_for_any * sizeof(seen_uids[0]));
			assert(seen_uids);
		}

		if (!NFC_radio_on(&nfc, kNFCCardType_MIFARE)) {
			free(seen_uids);
			fprintf(stderr, "Failed to turn on NFC radio.\n");
			return false;
		}
		// Give the radio half a second to get started.
		sleep_ns(INT64_C(500000000));

		struct NFCCardIterator iter;
		struct NFCCard card;
		uint64_t const start = now_ns();
		bool did_find_uid = !args.wait_for_specific.has_value;
		while (seen_uid_count < args.wait_for_any || (!did_find_uid && !timeout_(start, args.timeout))) {
			if (!NFC_poll(&nfc, &iter)) {
				free(seen_uids);
				fprintf(stderr, "Failed to poll NFC.\n");
				return false;
			}
			while (NFCCardIterator_next(&iter, &card)) {
				if (UID_is_in_array(seen_uids, seen_uid_count, &card.uid)) {
					continue;
				}

				if (args.wait_for_any) {
					memcpy(seen_uids + seen_uid_count, &card.uid, sizeof(card.uid));
					seen_uid_count++;

					struct UIDString const uid_string = UIDString_from_UID(&card.uid);
					printf("%s\n", uid_string.chars);
				}
				if (args.wait_for_specific.has_value && UID_is_equal(&args.wait_for_specific.uid, &card.uid)) {
					did_find_uid = true;
					break;
				}
			}
		}
		free(seen_uids);
	}

	Serial_close(nfc.serial);
	return 0;
}

#include "enc.c"

static void encode_one_test(
	Encoding encoding,
	wchar_t const* unicode,
	size_t unicode_len,
	unsigned char const* expected_str,
	size_t expected_str_len)
{
	unsigned char* encoded_str;
	int encoded_size;
	encoded_size = encode_string(unicode, unicode_len, encoding, NULL);
	TEST(encoded_size != -1);
	TEST(encoded_size = expected_str_len);
	encoded_str = malloc(encoded_size);
	TEST(encoded_str);
	(void)encode_string(unicode, unicode_len, encoding, encoded_str);
	TEST(0 == memcmp(expected_str, encoded_str, encoded_size - 1));
}

static void Encoding_test(void)
{
	wchar_t const unicode_jp[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x65e5,0x672c,0x8a9e,0x3067,0x3059,0xff83,0xff9e,0xff7d};
	unsigned char const encoded_jp[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x93,0xfa,0x96,0x7b,0x8c,0xea,0x82,0xc5,0x82,0xb7,0xc3,0xde,0xbd};
	wchar_t const unicode_kr[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x6b,0x6f,0x72,0x65,0x61,0x6e,0x3a,0x20,0xd1b5,0xd569,0xd615,0x20,0xd55c,0xae00,0x20,0xcf54,0xb4dc};
	unsigned char const encoded_kr[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x6b,0x6f,0x72,0x65,0x61,0x6e,0x3a,0x20,0xc5,0xeb,0xc7,0xd5,0xc7,0xfc,0x20,0xc7,0xd1,0xb1,0xdb,0x20,0xc4,0xda,0xb5,0xe5};
	wchar_t const unicode_cns[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x73,0x69,0x6d,0x70,0x6c,0x69,0x66,0x69,0x65,0x64,0x20,0x63,0x68,0x69,0x6e,0x65,0x73,0x65,0x3a,0x20,0x7b80,0x4f53,0x4e2d,0x6587};
	unsigned char const encoded_cns[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x73,0x69,0x6d,0x70,0x6c,0x69,0x66,0x69,0x65,0x64,0x20,0x63,0x68,0x69,0x6e,0x65,0x73,0x65,0x3a,0x20,0xbc,0xf2,0xcc,0xe5,0xd6,0xd0,0xce,0xc4};
	wchar_t const unicode_cnt[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x74,0x72,0x61,0x64,0x69,0x74,0x69,0x6f,0x6e,0x61,0x6c,0x20,0x63,0x68,0x69,0x6e,0x65,0x73,0x65,0x3a,0x20,0x4e2d,0x6587,0x6578,0x4f4d,0x5316,0x6280,0x8853,0x63a8,0x5ee3,0x59d4,0x54e1,0x6703};
	unsigned char const encoded_cnt[] = {0x74,0x68,0x69,0x73,0x20,0x69,0x73,0x20,0x74,0x72,0x61,0x64,0x69,0x74,0x69,0x6f,0x6e,0x61,0x6c,0x20,0x63,0x68,0x69,0x6e,0x65,0x73,0x65,0x3a,0x20,0xa4,0xa4,0xa4,0xe5,0xbc,0xc6,0xa6,0xec,0xa4,0xc6,0xa7,0xde,0xb3,0x4e,0xb1,0xc0,0xbc,0x73,0xa9,0x65,0xad,0xfb,0xb7,0x7c};
#	define TEST_ENC(enc, uni, exp) encode_one_test(kEncoding_##enc, uni, sizeof(uni) / sizeof(uni[0]), exp, sizeof(exp) / sizeof(exp[0]))
	TEST_ENC(shift_jis, unicode_jp, encoded_jp);
	TEST_ENC(cp949, unicode_kr, encoded_kr);
	TEST_ENC(gb2312, unicode_cns, encoded_cns);
	TEST_ENC(big5, unicode_cnt, encoded_cnt);
#	undef TEST_ENC
}

#define kVFDImage_width      160
#define kVFDImage_height     32
#define kVFDImage_bytes_high 4

struct VFDImage {
	uint8_t p[kVFDImage_bytes_high * kVFDImage_width];
};

bool VFD_reset(Serial serial)
{
	uint8_t const cmd[] = {0x1b, 0x0b};
	return Serial_write(serial, cmd, sizeof(cmd));
}

bool VFD_turn_on(Serial serial, bool on)
{
	uint8_t const cmd[] = {0x1b, 0x21, on ? 1 : 0};
	return Serial_write(serial, cmd, sizeof(cmd));
}

bool VFD_set_brightness(Serial serial, int brightness)
{
	assert(brightness >= 0 && brightness <= 4);
	uint8_t const cmd[] = {0x1b, 0x20, (uint8_t)brightness};
	return Serial_write(serial, cmd, sizeof(cmd));
}

static uint16_t u16_be(uint16_t s)
{
#if __LITTLE_ENDIAN__
    uint8_t b[2];
    memcpy(b, &s, sizeof(s));
    s = b[0] << 8 | b[1];
    return s;
#else
    return s;
#endif
}

bool VFD_set_text_window(Serial serial, uint16_t x_start, uint16_t x_end, uint8_t y)
{
	struct {
		uint8_t header[2];
		uint16_t x;
		uint8_t y;
		uint16_t x_end;
		uint8_t unused;
	} __attribute__((__packed__)) cmd = {
		.header = {0x1b, 0x40},
		.x = u16_be(x_start),
		.y = y,
		.x_end = u16_be(x_end),
	};
	return Serial_write(serial, &cmd, sizeof(cmd));
}

bool VFD_write_scroll_text(Serial serial, unsigned char const* text, size_t text_len)
{
	if (text_len > 148) {
		fprintf(stderr, "Encoded text string is too long: %zu > 148.\n", text_len);
		return false;
	}
	uint8_t cmd[148 + 3] = {0x1b, 0x50, (uint8_t)text_len};
	memcpy(&cmd[3], text, text_len);
	return Serial_write(serial, cmd, 3 + text_len);
}

bool VFD_set_text_scroll(Serial serial, bool enable)
{
	uint8_t cmd[] = {0x1b, 0};
	if (enable) {
		cmd[1] = 0x51;
	} else {
		cmd[1] = 0x52;
	}
	return Serial_write(serial, cmd, sizeof(cmd));
}

bool VFD_set_slow_text_scroll(Serial serial, bool enable)
{
	uint8_t const cmd[] = {0x1b, 0x41, enable ? 1 : 0};
	return Serial_write(serial, cmd, sizeof(cmd));
}

bool VFD_draw_image(Serial serial, uint16_t x, uint8_t y_byte, uint16_t width, uint8_t bytes_high, struct VFDImage const* image)
{
	struct {
		uint8_t header[2];
		uint16_t x_start;
		uint8_t y_start_byte;
		uint16_t w;
		uint8_t y_end_byte;
		struct VFDImage image;
	} __attribute__((__packed__)) cmd = {
		.header = {0x1b, 0x2e},
		.x_start = u16_be(x),
		.y_start_byte = y_byte,
		.w = u16_be(width),
		.y_end_byte = (y_byte + bytes_high) - 1,
	};
	memcpy(&cmd.image, image, sizeof(*image));
	return Serial_write(serial, &cmd, sizeof(cmd));
}

bool VFD_set_text_encoding(Serial serial, Encoding encoding)
{
	uint8_t enc_cmd;
	switch(encoding) {
		case kEncoding_shift_jis: enc_cmd = 2; break;
		case kEncoding_gb2312: enc_cmd = 0; break;
		case kEncoding_big5: enc_cmd = 1; break;
		case kEncoding_cp949: enc_cmd = 3; break;
		default:
			fprintf(stderr, "Unknown encoding format %d\n", encoding);
			abort();
	}
	uint8_t const cmd[] = {0x1b, 0x32, enc_cmd};
	return Serial_write(serial, cmd, sizeof(cmd));
}

struct ArgsVFD {
	char const* port;
	int brightness;
	char const* image_path;
	unsigned char* text;
	int text_length;
	int slow_scroll;
	Encoding text_encoding;
	bool off;
	bool reset;
};

void ArgsVFD_cleanup(struct ArgsVFD* args)
{
	free(args->text);
}


#if IS_WINDOWS
#include <wchar.h>
static wchar_t* copy_unicode_arg_(char** argv, int arg_i, size_t* unicode_len)
{
	// On windows the args we get are non-unicode so we ignore them and just ask the OS directly.
	(void)argv;
	LPWSTR wide_commandline = GetCommandLineW();
	int argc_w;
	LPWSTR* argv_w = CommandLineToArgvW(wide_commandline, &argc_w);
	assert(argv_w);
	assert(argc_w > arg_i);
	LPWSTR arg = argv_w[arg_i];
	*unicode_len = wcslen(arg);
	size_t buff_size = sizeof(wchar_t) * (*unicode_len + 1);
	wchar_t* result = malloc(buff_size);
	memcpy(result, arg, buff_size);
	LocalFree(argv_w);

	return result;
}

static void free_unicode_arg_(wchar_t* arg)
{
	free(arg);
}

#else

#include <wchar.h>
#include <locale.h>
static wchar_t* copy_unicode_arg_(char** argv, int arg_i, size_t* unicode_len)
{
	char* old_locale = setlocale(LC_ALL, NULL);
	char* locale_result = setlocale(LC_ALL, "");
	if (locale_result == NULL) {
		setlocale(LC_ALL, old_locale);
		return NULL;
	}

	wchar_t* arg_wide = NULL;
	mbstate_t ps = {};
	char const* arg_in = argv[arg_i];
	size_t const arg_in_len = strlen(arg_in);
	arg_wide = malloc(sizeof(arg_wide[0]) * (arg_in_len + 1));
	assert(arg_wide);
	size_t const wide_len = mbsrtowcs(arg_wide, &arg_in, arg_in_len, &ps);
	assert(wide_len != (size_t)-1);

	setlocale(LC_ALL, old_locale);

	*unicode_len = wide_len;
	return arg_wide;
}

static void free_unicode_arg_(wchar_t* arg)
{
	free(arg);
}

#endif

static int find_string_encoding_(wchar_t const* uni_str, size_t uni_str_len, Encoding* out_encoding)
{
	assert(uni_str);
	assert(out_encoding);
	Encoding all_encodings[] = {
		kEncoding_shift_jis,
		kEncoding_big5,
		kEncoding_gb2312,
		kEncoding_cp949,
	};
	for (size_t i = 0; i < sizeof(all_encodings) / sizeof(all_encodings[0]); i++) {
		Encoding const encoding = all_encodings[i];
		int len = encode_string(uni_str, uni_str_len, encoding, NULL);
		if (len != -1) {
			*out_encoding = encoding;
			return len;
		}
	}
	return -1;
}

static bool VFDImage_load_(struct VFDImage* vfd_image, char const* path)
{
	void* image_bytes = malloc(BMP_MAX_IMAGE_BYTES(kVFDImage_width, kVFDImage_height));
	assert(image_bytes);
	uint16_t bpp;
	if (!bmp_load_(path, kVFDImage_width, kVFDImage_height, image_bytes, &bpp)) {
		free(image_bytes);
		fprintf(stderr, "Failed to load VFD BMP.\n");
		return false;
	}

	// Convert BMP image bytes to the VFD format.
	memset(vfd_image->p, 0, sizeof(vfd_image->p));
	uint8_t const* row = image_bytes;
	int stride = BMP_ROW_STRIDE(kVFDImage_width, bpp);
	uint16_t const bytes_per_pixel = bpp / 8;
	for (int y = 0; y < kVFDImage_height; y++) {
		for (int x = 0; x < kVFDImage_width; x++) {
			uint8_t const* p = row + x * bytes_per_pixel;
			size_t const r = (size_t)p[0] * (size_t)212;
			size_t const g = (size_t)p[1] * (size_t)701;
			size_t const b = (size_t)p[2] * (size_t)87;
			uint8_t const brightness = (r + g + b) / (size_t)1000;
			uint8_t v = brightness > 128 ? 1 : 0;
			size_t const col_byte_index = y / 8;
			size_t const bit_index = y % 8;
			v <<= (7 - bit_index);
			vfd_image->p[x * kVFDImage_bytes_high + col_byte_index] |= v;
		}
		row += stride;
	}

	free(image_bytes);
	return true;
}

static void print_usage_vfd(void)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "pras3 vfd [--port path] [--text <string>] [--image <path>] [--brightness <0-4>] [--off] [--reset]\n");
	fprintf(stderr, "--port <path>         The serial port path.\n");
	fprintf(stderr, "--text <string>       String to scroll on the display.\n");
	fprintf(stderr, "--slow_scroll <0,1>   Set to 1 to make the text scroll slower.\n");
	fprintf(stderr, "--image <path>        Path to an RGB BMP image to display on the VFD.\n");
	fprintf(stderr, "                      The image must be exactly 160x32 pixels.\n");
	fprintf(stderr, "--brightness <0-4>    Set the brightness of the VFD. 0 is off. 4 is the max brightest.\n");
	fprintf(stderr, "--off                 Turn off the VFD.\n");
	fprintf(stderr, "--reset               Reset VFD to initial state.\n");
}

static bool ArgsVFD_parse(struct ArgsVFD* args, int argc, char** argv)
{
	*args = (struct ArgsVFD){
		.brightness = -1,
		.slow_scroll = -1,
	};
	if (IS_WINDOWS) {
		args->port = "COM1";
	} else {
		args->port = "/dev/ttyS0";
	}
	struct option const longopts[] = {
		{"port",        required_argument, NULL, 'p'},
		{"text",        required_argument, NULL, 't'},
		{"slow_scroll", required_argument, NULL, 's'},
		{"image",       required_argument, NULL, 'i'},
		{"brightness",  required_argument, NULL, 'b'},
		{"off",         no_argument,       NULL, 'o'},
		{"reset",       no_argument,       NULL, 'r'},
		{},
	};
	int c;
	optind = 1;
	while (-1 != (c = getopt_long(argc, argv, "", longopts, NULL))) {
		switch (c) {
			case 'p': {
				args->port = optarg;
			} break;
			case 't': {
				size_t unicode_length;
				wchar_t* unicode_arg = copy_unicode_arg_(argv, optind - 1, &unicode_length);
				if (unicode_arg == NULL) {
					fprintf(stderr, "Could not copy text argument. Something bad has happend.\n");
					return false;
				}
				args->text_length = find_string_encoding_(unicode_arg, unicode_length, &args->text_encoding);
				if (args->text_length == -1) {
					fprintf(stderr, "Could not encode text string.\n");
					free_unicode_arg_(unicode_arg);
					return false;
				}
				args->text = malloc(args->text_length + 1);
				encode_string(unicode_arg, unicode_length, args->text_encoding, args->text);
				args->text[args->text_length] = '\0';
				free_unicode_arg_(unicode_arg);
			} break;
			case 's': {
				char* end;
				args->slow_scroll = strtol(optarg, &end, 10);
				if (errno != 0 || end == optarg) {
					fprintf(stderr, "Invalid value for slow_scroll. Only 0 or 1 is allowed.\n");
					return false;
				}
			} break;
			case 'i': {
				args->image_path = optarg;
			} break;
			case 'b': {
				char* end;
				args->brightness = strtol(optarg, &end, 10);
				if (errno != 0 || end == optarg) {
					fprintf(stderr, "Invalid value for brightness. Only 0-4 are valid values.\n");
					return false;
				}
			} break;
			case 'o': {
				args->off = true;
			} break;
			case 'r': {
				args->reset = true;
			} break;
			default: {
				fprintf(stderr, "Unknown option '%s'\n", argv[optopt]);
				return false;
			} break;
		}
	}

	return true;
}

int run_tool_vfd(int argc, char** argv)
{
	struct ArgsVFD args;
	if (!ArgsVFD_parse(&args, argc, argv)) {
		fprintf(stderr, "Failed to parse VFD args.\n");
		print_usage_vfd();
		ArgsVFD_cleanup(&args);
		return -1;
	}
	struct VFDImage image;
	if (args.image_path) {
		if (!VFDImage_load_(&image, args.image_path)) {
			fprintf(stderr, "Failed to load BMP image.\n");
			ArgsVFD_cleanup(&args);
			return -1;
		}
	}

	struct SerialOptions const options = {
		.rtscts = true,
	};
	struct OptionalSerial maybe_serial = Serial_open(args.port, &options);
	if (!maybe_serial.has_value) {
		fprintf(stderr, "Failed to open VFD serial connection.\n");
		return false;
	}
	Serial serial = maybe_serial.serial;

	if (args.reset) {
		if (!VFD_reset(serial)) {
			fprintf(stderr, "Failed to reset VFD.\n");
			return false;
		}
	}
	if (!VFD_turn_on(serial, !args.off)) {
		fprintf(stderr, "Failed to turn VFD %s.\n", args.off ? "off" : "on");
		return false;
	}
	if (args.slow_scroll != -1) {
		if (!VFD_set_slow_text_scroll(serial, args.slow_scroll ? true : false)) {
			fprintf(stderr, "Failed to set slow text scroll.\n");
			return false;
		}
	}
	if (args.text) {
		if (args.text_length > 0) {
			VFD_set_text_window(serial, 20, 120, 1);
			VFD_set_text_encoding(serial, args.text_encoding);
			VFD_write_scroll_text(serial, args.text, args.text_length);
			VFD_set_text_scroll(serial, true);
		} else {
			VFD_set_text_scroll(serial, false);
		}
	}
	if (args.image_path) {
		VFD_draw_image(serial, 0, 0, kVFDImage_width, kVFDImage_bytes_high, &image);
	}
	if (args.brightness != -1) {
		VFD_set_brightness(serial, args.brightness);
	}

	Serial_close(serial);
	ArgsVFD_cleanup(&args);
	return 0;
}

static int run_test(void)
{
	Color_test();
	UID_test();
	Encoding_test();
	return 0;
}


static void print_usage(void)
{
	fprintf(stderr, "pras3 <led|nfc|vfd> [options...]\n");
	print_usage_led();
	print_usage_nfc();
	print_usage_vfd();
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
	} else if (0 == strcmp(tool, "nfc")) {
		return run_tool_nfc(argc, argv);
	} else if (0 == strcmp(tool, "vfd")) {
		return run_tool_vfd(argc, argv);
	}

	fprintf(stderr, "Unknown tool name '%s'\n", tool);
	return 1;
}
