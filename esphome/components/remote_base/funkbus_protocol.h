#pragma once

#include "remote_base.h"

namespace esphome::remote_base {

struct FunkbusData {
  uint32_t serial;
  uint8_t command;
  uint8_t group;
  uint8_t action;
  bool battery_low;
  bool repeated;
  bool longpress;

  // battery_low and repeated are status flags, ignored by operator== so a binary sensor keeps matching
  bool operator==(const FunkbusData &rhs) const {
    return serial == rhs.serial && command == rhs.command && group == rhs.group && action == rhs.action &&
           longpress == rhs.longpress;
  }
};

class FunkbusProtocol : public RemoteProtocol<FunkbusData> {
 public:
  void encode(RemoteTransmitData *dst, const FunkbusData &data);
  optional<FunkbusData> decode(RemoteReceiveData src);
  void dump(const FunkbusData &data);
};

DECLARE_REMOTE_PROTOCOL(Funkbus)

}  // namespace esphome::remote_base
