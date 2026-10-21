#include <gtest/gtest.h>
#include "esphome/components/remote_base/funkbus_protocol.h"

namespace esphome::remote_base::testing {

namespace {

constexpr int32_t IDLE_US = 10000;

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
  RawTimings timings = encode(make(0x12345, 4));
  // Turn the first data bit (a long symbol for 0, two short symbols for 1) into its opposite
  RawTimings flipped(timings.begin(), timings.begin() + 1);
  size_t i = 1;
  if (std::abs(timings[i]) == 1000) {
    flipped.push_back(timings[i] / 2);
    flipped.push_back(-timings[i] / 2);
    i += 1;
  } else {
    flipped.push_back(timings[i] * 2);
    i += 2;
  }
  // Every later symbol has its level swapped to keep the alternation
  for (; i < timings.size(); i++)
    flipped.push_back(-timings[i]);
  EXPECT_FALSE(decode(flipped).has_value());
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
