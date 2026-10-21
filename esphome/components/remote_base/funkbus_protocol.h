#pragma once

#include "esphome/core/component.h"
#include "remote_base.h"

#include <cinttypes>

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

template<typename... Ts> class FunkbusAction : public RemoteTransmitterActionBase<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint32_t, serial)
  TEMPLATABLE_VALUE(uint8_t, command)
  TEMPLATABLE_VALUE(uint8_t, group)
  TEMPLATABLE_VALUE(uint8_t, action)
  TEMPLATABLE_VALUE(bool, battery_low)
  TEMPLATABLE_VALUE(bool, repeated)
  TEMPLATABLE_VALUE(bool, longpress)

  void encode(RemoteTransmitData *dst, Ts... x) override {
    FunkbusData data{};
    data.serial = this->serial_.value(x...);
    data.command = this->command_.value(x...);
    data.group = this->group_.value(x...);
    data.action = this->action_.value(x...);
    data.battery_low = this->battery_low_.value(x...);
    data.repeated = this->repeated_.value(x...);
    data.longpress = this->longpress_.value(x...);
    FunkbusProtocol().encode(dst, data);
  }
};

}  // namespace esphome::remote_base
