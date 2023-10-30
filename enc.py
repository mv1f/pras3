#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional, Tuple
import sys

@dataclass
class Row:
	pointer: int
	codepoint: int

Index = List[Row]


def load_unicode_map(path: str, pointer_i: int=0, unicode_i: int=1) -> Index:
	txt = open(path, "rb").read()
	lines = [l for l in txt.split(b'\x0a') if 0 != len(l) and not l.startswith(b'#')]
	rows = [[c.strip() for c in l.split(b'\x09')] for l in lines]
	rows = [r for r in rows if r[unicode_i]]
	return sorted([Row(int(r[pointer_i], 16), int(r[unicode_i], 16)) for r in rows], key=lambda row: row.codepoint)


index_unicode_big5 = load_unicode_map("BIG5.TXT")
index_unicode_gb2312 = load_unicode_map("GB2312.TXT")
index_unicode_jis0208 = load_unicode_map("JIS0208.TXT", pointer_i=1, unicode_i=2)
index_unicode_jis0212 = load_unicode_map("JIS0212.TXT")
index_unicode_cp949 = load_unicode_map("CP949.TXT")


index_jis0208_flat = [b'\x00'] * (1<<16)
for row in index_unicode_jis0208:
	index = row.codepoint
	assert index < (1 << 16)
	lead = row.pointer >> 8
	trail = row.pointer & 0xff
	if (lead - 0x21) & 1:
		trail = 0x5e + trail - 0x21
	else:
		trail = trail - 0x21
	lead = (lead - 0x21) >> 1
	if lead < 0x1f:
		lead += 0x81
	else:
		lead += 0xc1
	if trail < 0x3f:
		trail += 0x40
	else:
		trail += 0x41
	assert lead > 0
	index_jis0208_flat[index] = bytes([lead, trail])
for i in range(0x80):
	index_jis0208_flat[i] = bytes([0, i])
index_jis0208_flat[0xa5] = bytes([0, 0x5c])
index_jis0208_flat[0x203e] = bytes([0, 0x7e])
for i in range(0xff61, 0xff9f + 1):
	index_jis0208_flat[i] = bytes([0, i - 0xfec0])


def codepoint_to_jis0208_flat(cp: int) -> bytes:
	return index_jis0208_flat[cp]


index_cp949_flat = [b'\x00'] * (1<<16)
for row in index_unicode_cp949:
	index = row.codepoint
	if row.pointer & 0x8000:
		assert index < (1 << 16)
		lead = row.pointer >> 8
		trail = row.pointer & 0xff
		assert lead > 0
		index_cp949_flat[index] = bytes([lead, trail])
for i in range(0x80 + 1):
	index_cp949_flat[i] = bytes([0, i])


def codepoint_to_cp949_flat(cp: int) -> bytes:
	return index_cp949_flat[cp]


index_big5_flat = [b'\x00'] * (1<<16)
for row in index_unicode_big5:
	index = row.codepoint
	if index <= 0x80:
		index_big5_flat[index] = bytes([index])
	else:
		lead = row.pointer >> 8
		trail = row.pointer & 0xff
		assert lead > 0
		index_big5_flat[index] = bytes([lead, trail])
for i in range(0x80 + 1):
	index_big5_flat[i] = bytes([0, i])


def codepoint_to_big5_flat(cp: int) -> bytes:
	return index_big5_flat[cp]


index_gb2312_flat = [b'\x00'] * (1<<16)
for row in index_unicode_gb2312:
	index = row.codepoint
	if index <= 0x80:
		index_gb2312_flat[index] = bytes([index])
	else:
		lead = 0x80 | (row.pointer >> 8)
		trail = 0x80 | (row.pointer & 0xff)
		assert lead > 0
		index_gb2312_flat[index] = bytes([lead, trail])
for i in range(0x80 + 1):
	index_gb2312_flat[i] = bytes([0, i])


def codepoint_to_gb2312_flat(cp: int) -> bytes:
	return index_gb2312_flat[cp]


def test_encoding(encoding_name: str, table: Index, codepoint_to_bytes):
	print(f"testing {encoding_name}")
	skip_count = 0
	bad_encoding_count = 0
	for i, row in enumerate(table):
		cp = row.codepoint
		c = chr(cp)
		encoded = codepoint_to_bytes(cp)
		try:
			encoded_expected = c.encode(encoding_name)
		except UnicodeEncodeError as _:
			print(f"{i}: {skip_count}: can't encode '{c}' 0x{cp:04x}. Skipping.")
			skip_count += 1
			continue
		encoded = encoded if encoded[0] != 0 else bytes([encoded[1]])
		if encoded != encoded_expected:
			print(f"{i}: unexpected encoding for '{c}' (0x{cp:x}) got {encoded} expected {encoded_expected} (0x{cp:04x})")
			bad_encoding_count += 1
	if skip_count:
		print(f"skipped {skip_count}")
	if bad_encoding_count:
		print(f"{bad_encoding_count} bad encodings")


def convert_string(s: str, converter) -> bytes:
	result = b''
	for c in s:
		c = ord(c)
		b = converter(c)
		if b == b'\x00':
			print(f"c=0x{c:02x}")
		if b[0] == 0:
			result += bytes([b[1]])
		else:
			result += b
	return result


def check_conversion(encoding: str, s: str, converter):
	expected = s.encode(encoding)
	converted = convert_string(s, converter)
	if expected != converted:
		print(f"unexpected conversion for {s!r} to {encoding}")
		print(f"expected: {expected}")
		print(f"got:      {converted}")


check_conversion("shift-jis", "this is 日本語ですﾃﾞｽ", codepoint_to_jis0208_flat)
check_conversion("KSC5601","this is korean: 통합형 한글 코드", codepoint_to_cp949_flat)
check_conversion("GB2312","this is simplified chinese: 简体中文", codepoint_to_gb2312_flat)
check_conversion("BIG5", "this is traditional chinese: 中文數位化技術推廣委員會", codepoint_to_big5_flat)
test_encoding("big5", index_unicode_big5, codepoint_to_big5_flat)
test_encoding("gb2312", index_unicode_gb2312, codepoint_to_gb2312_flat)
test_encoding("shift-jis", index_unicode_jis0208, codepoint_to_jis0208_flat)
test_encoding("cp949", index_unicode_cp949, codepoint_to_cp949_flat)

with open("enc.c", "w") as source_file, open("enc.h", "w") as header_file:
	encodings = [
		("shift_jis", index_jis0208_flat),
		("big5", index_big5_flat),
		("gb2312", index_gb2312_flat),
		("cp949", index_cp949_flat)
	]
	header_file.write("""\
#include <string.h>
#include <wchar.h>

typedef enum {
	kEncoding_none,
""")
	for name, _ in encodings:
		header_file.write(f"""\
	kEncoding_{name},
""")
	header_file.write("""\
} Encoding;
""")
	func_decl = "int encode_string(wchar_t const* ustr, size_t ustr_len, Encoding encoding, unsigned char* out_encoded)"
	header_file.write(f"""\
{func_decl};
""")

	# Source file
	source_file.write(f"""\
#include \"enc.h\"

""")
	# Write tables
	for name, table in encodings:
		source_file.write(f"""\
static const unsigned char enc_table_{name}[{len(table)}][2] = {{
""")
		rows_per_line = 32
		for line in range(len(table) // rows_per_line):
			source_file.write("""\
    """)
			start_row = line * rows_per_line
			for row in table[start_row : start_row + rows_per_line]:
				if len(row) == 1 and row == b'\x00':
					source_file.write("{},")
				else:
					source_file.write(f"{{0x{row[0]:02x}, 0x{row[1]:02x}}},")
			source_file.write("\n")
		source_file.write("""\
};

""")
	source_file.write("""\
static int try_encode(wchar_t const* in_str, size_t ustr_len, unsigned char const table[][2], unsigned char* out_encoded)
{
	int out_pos = 0;
	for (size_t i = 0; i < ustr_len; i++) {
		wchar_t const c = in_str[i];
		unsigned char const* row = table[c];
		if (row[0] == 0) {
			if (row[1] == 0) {
				return -1;
			} else if (out_encoded) {
				out_encoded[out_pos] = row[1];
			}
			out_pos++;
		} else {
			if (out_encoded) {
				out_encoded[out_pos + 0] = row[0];
				out_encoded[out_pos + 1] = row[1];
			}
			out_pos += 2;
		}
	}
	return out_pos;
}

""")
	source_file.write(f"""\
{func_decl}
{{
	int result = -1;
""")
	for name, table in encodings:
		source_file.write(f"""\
	if (encoding == kEncoding_{name}) {{
		result = try_encode(ustr, ustr_len, enc_table_{name}, out_encoded);
		if (result != -1) {{
			return result;
		}}
	}}
""")
	source_file.write("""\
	return result;
}
""")
