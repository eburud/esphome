#include <gtest/gtest.h>
#include <cstdlib>
#include <vector>
#include "esphome/components/remote_base/funkbus_protocol.h"
#include "esphome/core/helpers.h"

namespace esphome::remote_base::testing {

namespace {

constexpr int32_t SHORT_US = 500;
constexpr int32_t LONG_US = 1000;
constexpr int32_t IDLE_US = 10000;
constexpr int32_t PREAMBLE_US = 4000;
constexpr size_t NBITS = 48;

FunkbusData make(uint32_t serial, uint8_t command) {
  FunkbusData out{};
  out.serial = serial;
  out.command = command;
  out.group = 1;
  out.action = 2;
  return out;
}

RawTimings encode(const FunkbusData &data) {
  RemoteTransmitData tx;
  FunkbusProtocol().encode(&tx, data);
  return tx.get_data();
}

optional<FunkbusData> decode(const RawTimings &timings) {
  return FunkbusProtocol().decode(RemoteReceiveData(timings, 25, TOLERANCE_MODE_PERCENTAGE));
}

void expect_same(const optional<FunkbusData> &res, const FunkbusData &data) {
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->serial, data.serial);
  EXPECT_EQ(res->command, data.command);
  EXPECT_EQ(res->group, data.group);
  EXPECT_EQ(res->action, data.action);
  EXPECT_EQ(res->battery_low, data.battery_low);
  EXPECT_EQ(res->repeated, data.repeated);
  EXPECT_EQ(res->longpress, data.longpress);
}

// Read the bits back: two short symbols are a 1, one long symbol is a 0
std::vector<bool> to_bits(const RawTimings &timings) {
  std::vector<bool> bits;
  for (size_t i = 1; i < timings.size(); i += bits.back() ? 2 : 1)
    bits.push_back(std::abs(timings[i]) == SHORT_US);
  return bits;
}

RawTimings from_bits(const std::vector<bool> &bits) {
  RawTimings timings{PREAMBLE_US};
  int32_t level = -1;
  for (bool bit : bits) {
    for (int s = 0; s < (bit ? 2 : 1); s++) {
      timings.push_back(level * (bit ? SHORT_US : LONG_US));
      level = -level;
    }
  }
  return timings;
}

// Fields are sent LSB-first, so a field value is read with its first bit as the lowest bit.
void set_field(std::vector<bool> &bits, size_t start, size_t len, uint32_t value) {
  for (size_t i = 0; i < len; i++)
    bits[start + i] = (value >> i) & 1;
}

// Independent copy of the frame checksum so a frame with an altered header can still be made valid.
void set_valid_checksum(std::vector<bool> &bits) {
  uint8_t buf[6] = {0};
  for (size_t i = 0; i < 43; i++)
    buf[i / 8] |= bits[i] << (7 - i % 8);
  uint8_t x = xor_bytes(buf, 5) ^ (buf[5] & 0xE0);
  x = (x >> 4) ^ (x & 0xF);
  uint8_t sum = 0;
  static constexpr uint8_t MASKS[4] = {0x23, 0xC8, 0x32, 0x8C};
  for (int i = 0; i < 4; i++) {
    if (x & (1 << i))
      sum ^= MASKS[i];
  }
  sum &= 0xF;
  uint8_t parity = 0;
  for (size_t i = 0; i < 43; i++)
    parity ^= bits[i];
  bits[43] = parity;
  // The transmitted check nibble is bit-reversed
  set_field(bits, 44, 4, ((sum & 1) << 3) | ((sum & 2) << 1) | ((sum & 4) >> 1) | ((sum & 8) >> 3));
}

}  // namespace

TEST(FunkbusProtocolTest, RoundTripAllFields) {
  FunkbusData data = make(0xABCDE, 5);
  data.group = 3;
  data.action = 1;
  data.battery_low = true;
  data.repeated = true;
  data.longpress = true;
  expect_same(decode(encode(data)), data);
}

// A receiver loses a trailing space in the idle gap: it is either dropped or reported as idle length
TEST(FunkbusProtocolTest, DecodesFramesEndingOnSpace) {
  unsigned ending_on_space = 0;
  unsigned ending_on_mark = 0;
  for (uint32_t serial = 0; serial < 64; serial++) {
    const FunkbusData data = make(serial, serial & 0x7);
    RawTimings timings = encode(data);
    expect_same(decode(timings), data);
    if (timings.back() > 0) {
      ending_on_mark++;
      continue;
    }
    ending_on_space++;
    timings.back() = -IDLE_US;
    expect_same(decode(timings), data);
    timings.pop_back();
    expect_same(decode(timings), data);
  }
  // Both frame endings must have been exercised
  EXPECT_GT(ending_on_space, 0u);
  EXPECT_GT(ending_on_mark, 0u);
}

TEST(FunkbusProtocolTest, RejectsTruncatedFrame) {
  RawTimings timings = encode(make(0x12345, 4));
  timings.resize(timings.size() - 4);
  EXPECT_FALSE(decode(timings).has_value());
}

TEST(FunkbusProtocolTest, RejectsBadChecksum) {
  std::vector<bool> bits = to_bits(encode(make(0x12345, 4)));
  ASSERT_EQ(bits.size(), NBITS);
  // Bit 8 is the lowest serial bit, so the type check passes and only the checksum can reject the frame
  bits[8] = !bits[8];
  EXPECT_FALSE(decode(from_bits(bits)).has_value());
}

// Bit 43 is the parity bit and bits 44-47 the check nibble; each alone must reject the frame
TEST(FunkbusProtocolTest, RejectsBadParityOrCheckNibble) {
  const std::vector<bool> good = to_bits(encode(make(0x12345, 4)));
  ASSERT_TRUE(decode(from_bits(good)).has_value());
  for (size_t bit : {43u, 44u}) {
    std::vector<bool> bits = good;
    bits[bit] = !bits[bit];
    EXPECT_FALSE(decode(from_bits(bits)).has_value()) << "bit " << bit;
  }
}

TEST(FunkbusProtocolTest, RejectsWrongPreamble) {
  RawTimings timings = encode(make(0x12345, 4));
  timings[0] = PREAMBLE_US / 2;
  EXPECT_FALSE(decode(timings).has_value());
}

// Other Funkbus device types carry a valid checksum but are not remote messages
TEST(FunkbusProtocolTest, RejectsOtherTypeOrSubtype) {
  const std::vector<bool> good = to_bits(encode(make(0x12345, 4)));
  for (size_t field_start : {0u, 4u}) {
    std::vector<bool> bits = good;
    set_field(bits, field_start, 4, 0x5);
    set_valid_checksum(bits);
    EXPECT_FALSE(decode(from_bits(bits)).has_value()) << "field at bit " << field_start;
  }
  // The helper itself must produce frames the decoder accepts, or the cases above prove nothing
  std::vector<bool> same = good;
  set_valid_checksum(same);
  EXPECT_EQ(same, good);
  EXPECT_TRUE(decode(from_bits(same)).has_value());
}

TEST(FunkbusProtocolTest, MatchIgnoresStatusFlags) {
  FunkbusData configured = make(0x12345, 4);
  FunkbusData received = configured;
  received.battery_low = true;
  received.repeated = true;
  EXPECT_TRUE(received == configured);
  received.longpress = true;
  EXPECT_FALSE(received == configured);
}

}  // namespace esphome::remote_base::testing
