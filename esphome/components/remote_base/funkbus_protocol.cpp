#include "funkbus_protocol.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::remote_base {

ESPHOME_LOG_TAG(TAG, "remote.funkbus");

// Funkbus wireless remotes (Berker, Gira, Jung), also sold as Instafunk
// developed by Insta GmbH.
// Reverse-engineered protocol: https://github.com/merbanan/rtl_433/blob/master/src/devices/funkbus.c
static constexpr uint32_t PREAMBLE_US = 4000;
static constexpr uint32_t SHORT_US = 500;
static constexpr uint32_t LONG_US = 1000;
static constexpr uint8_t NBITS = 48;
static constexpr uint8_t NBYTES = NBITS / 8;
// Only remote (switch/scene) messages are decoded; other Funkbus device types are not supported.
static constexpr uint8_t TYPE_REMOTE = 0x4;
static constexpr uint8_t SUBTYPE_REMOTE = 0x3;

namespace {

// The frame is packed MSB-first per byte, in the order bits are put on the air (matches rtl_433's bitbuffer).
bool get_bit(const uint8_t *buf, unsigned pos) { return (buf[pos / 8] >> (7 - (pos % 8))) & 1; }

void set_bit(uint8_t *buf, unsigned pos, bool value) {
  if (value) {
    buf[pos / 8] |= 1 << (7 - (pos % 8));
  }
}

// Fields are transmitted LSB-first, unlike the raw MSB-first bit packing above.
uint32_t get_bits_reflect(const uint8_t *buf, unsigned start, unsigned len) {
  uint32_t result = 0;
  for (unsigned i = 0; i < len; i++) {
    if (get_bit(buf, start + i))
      result |= 1UL << i;
  }
  return result;
}

void set_bits_reflect(uint8_t *buf, unsigned start, unsigned len, uint32_t value) {
  for (unsigned i = 0; i < len; i++) {
    set_bit(buf, start + i, (value >> i) & 1);
  }
}

uint8_t parity8(uint8_t byte) {
  byte ^= byte >> 4;
  byte &= 0xF;
  return (0x6996 >> byte) & 1;
}

// 4-bit reversal: the transmitted check nibble is bit-reversed relative to the raw checksum nibble below.
uint8_t reflect4(uint8_t x) {
  x &= 0xF;
  return ((x & 0x1) << 3) | ((x & 0x2) << 1) | ((x & 0x4) >> 1) | ((x & 0x8) >> 3);
}

// LFSR checksum (8-bit mask 0x8C, shifted left by 2 each bit) over the first `len_bits` bits, plus parity.
// Result: bits 0-3 are the check nibble, bit 4 is the parity bit.
uint8_t calc_checksum(const uint8_t *buf, unsigned len_bits) {
  const uint8_t full_bytes = len_bits / 8;
  const uint8_t bits_left = len_bits % 8;

  uint8_t xor_byte = xor_bytes(buf, full_bytes);
  uint8_t mask = 0xFF << (8 - bits_left);
  xor_byte ^= buf[full_bytes] & mask;

  const uint8_t xor_nibble = ((xor_byte & 0xF0) >> 4) ^ (xor_byte & 0x0F);

  uint8_t result = 0;
  if (xor_nibble & 0x8)
    result ^= 0x8C;
  if (xor_nibble & 0x4)
    result ^= 0x32;
  if (xor_nibble & 0x2)
    result ^= 0xC8;
  if (xor_nibble & 0x1)
    result ^= 0x23;

  result &= 0xF;
  result |= parity8(xor_byte) << 4;
  return result;
}

bool peek_symbol(RemoteReceiveData &src, uint32_t width, uint32_t offset) {
  return src.peek_mark(width, offset) || src.peek_space(width, offset);
}

// A frame that ends on a space has that space merged into the idle gap, so it is missing or too long.
bool peek_last_symbol(RemoteReceiveData &src, uint32_t width, uint32_t offset) {
  return peek_symbol(src, width, offset) || !src.is_valid(offset) || src.peek_space_at_least(width, offset);
}

}  // namespace

void FunkbusProtocol::encode(RemoteTransmitData *dst, const FunkbusData &data) {
  dst->set_carrier_frequency(0);
  dst->reserve(2 + NBITS * 2u);

  uint8_t buf[NBYTES] = {0};
  set_bits_reflect(buf, 0, 4, TYPE_REMOTE);
  set_bits_reflect(buf, 4, 4, SUBTYPE_REMOTE);
  set_bits_reflect(buf, 8, 20, data.serial);
  set_bits_reflect(buf, 30, 1, data.battery_low ? 1 : 0);
  set_bits_reflect(buf, 33, 3, data.command);
  set_bits_reflect(buf, 36, 2, data.group);
  set_bits_reflect(buf, 39, 2, data.action);
  set_bits_reflect(buf, 41, 1, data.repeated ? 1 : 0);
  set_bits_reflect(buf, 42, 1, data.longpress ? 1 : 0);

  uint8_t checksum = calc_checksum(buf, 43);
  set_bits_reflect(buf, 43, 1, checksum >> 4);
  set_bits_reflect(buf, 44, 4, reflect4(checksum & 0xF));

  // Differential Manchester / Biphase-Mark: every bit period has a transition at its boundary; a "1" has an
  // additional transition at its midpoint. Since mark/space always alternate, tracking "next call is a mark"
  // and always toggling it after every emitted symbol reproduces this automatically.
  dst->mark(PREAMBLE_US);
  bool mark_next = false;
  for (uint8_t i = 0; i < NBITS; i++) {
    const uint32_t width = get_bit(buf, i) ? SHORT_US : LONG_US;
    const uint8_t symbols = get_bit(buf, i) ? 2 : 1;
    for (uint8_t s = 0; s < symbols; s++) {
      if (mark_next) {
        dst->mark(width);
      } else {
        dst->space(width);
      }
      mark_next = !mark_next;
    }
  }
}

optional<FunkbusData> FunkbusProtocol::decode(RemoteReceiveData src) {
  if (!src.expect_mark(PREAMBLE_US))
    return {};

  uint8_t buf[NBYTES] = {0};
  uint32_t offset = 0;
  for (uint8_t i = 0; i < NBITS; i++) {
    const auto peek_end = i == NBITS - 1 ? peek_last_symbol : peek_symbol;
    if (peek_symbol(src, SHORT_US, offset) && peek_end(src, SHORT_US, offset + 1)) {
      set_bit(buf, i, true);
      offset += 2;
    } else if (peek_end(src, LONG_US, offset)) {
      offset += 1;
    } else {
      return {};
    }
  }
  src.advance(offset);

  if (get_bits_reflect(buf, 0, 4) != TYPE_REMOTE || get_bits_reflect(buf, 4, 4) != SUBTYPE_REMOTE)
    return {};

  const uint32_t parity = get_bits_reflect(buf, 43, 1);
  const uint32_t check = get_bits_reflect(buf, 44, 4);
  const uint8_t checksum = calc_checksum(buf, 43);
  if (check != reflect4(checksum & 0xF) || parity != (checksum >> 4))
    return {};

  FunkbusData out{};
  out.serial = get_bits_reflect(buf, 8, 20);
  out.battery_low = get_bits_reflect(buf, 30, 1) != 0;
  out.command = get_bits_reflect(buf, 33, 3);
  out.group = get_bits_reflect(buf, 36, 2);
  out.action = get_bits_reflect(buf, 39, 2);
  out.repeated = get_bits_reflect(buf, 41, 1) != 0;
  out.longpress = get_bits_reflect(buf, 42, 1) != 0;
  return out;
}

void FunkbusProtocol::dump(const FunkbusData &data) {
  ESP_LOGI(TAG,
           "Received Funkbus: serial=0x%05" PRIX32 ", command=%d, group=%d, action=%d, battery_low=%s, "
           "repeated=%s, longpress=%s",
           data.serial, data.command, data.group, data.action, YESNO(data.battery_low), YESNO(data.repeated),
           YESNO(data.longpress));
}

}  // namespace esphome::remote_base
