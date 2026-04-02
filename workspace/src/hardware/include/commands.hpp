#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <sstream>
#include <tuple>
#include <iomanip>
#include <array>

#include "tuple_utils.hpp"

// ---------------------------------------------------------------------------
// Cmd namespace
//
// Sub-namespaces:
//   Cmd::NC   — Stepper (I2C, masked multi-motor) + DC motors
//   Cmd::ESP  — Arm joints (CAN, single-motor per message)
//
// Aliases for brevity:
//   using namespace Cmd;   // gives NC::, ESP::
//   using NC::WriteInst;   // gives WriteInst<NC::SPEED> etc.
// ---------------------------------------------------------------------------

namespace Cmd {

// Write Send/Return ------------------------------------------------
template <auto CMD> struct WriteSendPkt   { using type = void; };
template <auto CMD> struct WriteReturnPkt { using type = void; };

template <auto CMD> struct ReadSendPkt   { using type = void; };
template <auto CMD> struct ReadReturnPkt { using type = void; };

// ── (Stepper + DC) ────────────────────────────────────────────────────────
namespace Teensy {

enum class Write : uint8_t {
  BYTELOSS = 0x00,
  SPEED    = 0x01,
  POSITION = 0x02,
  ACCEL    = 0x03,
  DCVEL    = 0x04,
  DCACCEL  = 0x05,
};

enum class Read : uint8_t {
  BYTELOSS = 0x00,
  SPEED    = 0x01,
  POSITION = 0x02,
  ACCEL    = 0x03,
  DCVEL    = 0x04,
  DCACCEL  = 0x05,
};

}

template <> struct WriteSendPkt<Teensy::Write::BYTELOSS>   { using type = std::tuple<>; };
template <> struct WriteReturnPkt<Teensy::Write::BYTELOSS> { using type = std::tuple<>; };

template <> struct WriteSendPkt<Teensy::Write::SPEED>      { using type = std::tuple<uint8_t, float>; };   // mask, speed
template <> struct WriteReturnPkt<Teensy::Write::SPEED>    { using type = std::tuple<>; };

template <> struct WriteSendPkt<Teensy::Write::POSITION>   { using type = std::tuple<uint8_t, int32_t>; }; // mask, pos
template <> struct WriteReturnPkt<Teensy::Write::POSITION> { using type = std::tuple<>; };

template <> struct WriteSendPkt<Teensy::Write::ACCEL>      { using type = std::tuple<uint8_t, float>; };   // mask, accel
template <> struct WriteReturnPkt<Teensy::Write::ACCEL>    { using type = std::tuple<>; };

template <> struct WriteSendPkt<Teensy::Write::DCVEL>      { using type = std::tuple<float, float>; };
template <> struct WriteReturnPkt<Teensy::Write::DCVEL>    { using type = std::tuple<>; };

template <> struct WriteSendPkt<Teensy::Write::DCACCEL>    { using type = std::tuple<float, float>; };
template <> struct WriteReturnPkt<Teensy::Write::DCACCEL>  { using type = std::tuple<>; };

// Read Send/Return ------------------------------------
template <> struct ReadSendPkt<Teensy::Read::BYTELOSS>   { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::BYTELOSS> { using type = std::tuple<uint32_t, uint32_t>; }; // tx_loss, rx_loss

template <> struct ReadSendPkt<Teensy::Read::SPEED>      { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::SPEED>    { using type = std::tuple<float, float, float, float>; };

template <> struct ReadSendPkt<Teensy::Read::POSITION>   { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::POSITION> { using type = std::tuple<int32_t, int32_t, int32_t, int32_t>; };

template <> struct ReadSendPkt<Teensy::Read::ACCEL>      { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::ACCEL>    { using type = std::tuple<float, float, float, float>; };

template <> struct ReadSendPkt<Teensy::Read::DCVEL>      { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::DCVEL>    { using type = std::tuple<float, float>; };

template <> struct ReadSendPkt<Teensy::Read::DCACCEL>    { using type = std::tuple<>; };
template <> struct ReadReturnPkt<Teensy::Read::DCACCEL>  { using type = std::tuple<float, float>; };


namespace ESP32{

enum class Write : uint8_t {
  BYTELOSS = 0x00,
  SPEED    = 0x01,
  POSITION = 0x02,
  ACCEL    = 0x03,
};

enum class Read : uint8_t {
  BYTELOSS = 0x00,
  SPEED    = 0x01,
  POSITION = 0x02,
  ACCEL    = 0x03,
};

}

template <> struct WriteSendPkt<ESP32::Write::BYTELOSS>   { using type = std::tuple<>; };
template <> struct WriteReturnPkt<ESP32::Write::BYTELOSS> { using type = std::tuple<>; };

template <> struct WriteSendPkt<ESP32::Write::SPEED>      { using type = std::tuple<float>; };
template <> struct WriteReturnPkt<ESP32::Write::SPEED>    { using type = std::tuple<>; };

template <> struct WriteSendPkt<ESP32::Write::POSITION>   { using type = std::tuple<int32_t>; };
template <> struct WriteReturnPkt<ESP32::Write::POSITION> { using type = std::tuple<>; };

template <> struct WriteSendPkt<ESP32::Write::ACCEL>      { using type = std::tuple<float>; };
template <> struct WriteReturnPkt<ESP32::Write::ACCEL>    { using type = std::tuple<>; };


template <> struct ReadSendPkt<ESP32::Read::BYTELOSS>   { using type = std::tuple<>; };
template <> struct ReadReturnPkt<ESP32::Read::BYTELOSS> { using type = std::tuple<uint32_t>; };

template <> struct ReadSendPkt<ESP32::Read::SPEED>      { using type = std::tuple<>; };
template <> struct ReadReturnPkt<ESP32::Read::SPEED>    { using type = std::tuple<float>; };

template <> struct ReadSendPkt<ESP32::Read::POSITION>   { using type = std::tuple<>; };
template <> struct ReadReturnPkt<ESP32::Read::POSITION> { using type = std::tuple<int32_t>; };

template <> struct ReadSendPkt<ESP32::Read::ACCEL>      { using type = std::tuple<>; };
template <> struct ReadReturnPkt<ESP32::Read::ACCEL>    { using type = std::tuple<float>; };

template <auto CMD>
using read_type_t = typename ReadReturnPkt<CMD>::type;

template <auto ID, typename HandlerFunc>
inline bool dispatch_one(const uint8_t *data, size_t size, HandlerFunc &&handle) {
  using payload = typename ReadReturnPkt<ID>::type;
  if (size != tuple_size(payload{})) return false;
  payload p;
  unpack_tuple_from_buffer(p, data);
  handle(std::move(p));
  return true;
}

template <auto CMD, typename F, typename LogFn>
auto make_callback(F&& f, LogFn&& log) {
  return [func = std::forward<F>(f),
          logger = std::forward<LogFn>(log)]
         (const uint8_t* data, size_t size) {
    
    using info_t = read_type_t<CMD>;

    bool ok = dispatch_one<CMD>(
        data, size,
        [&](const info_t& info) {  
          func(info);
        });

    if (!ok) {
      logger("%i size mismatch (got %zu)",
             static_cast<int32_t>(CMD), size);
    }
  };
}

// ── Static commands (Ping etc.) ─────────────────────────────────────────────
enum class Static : uint8_t { Ping = 0x01 };


// CmdInter: maps a command enum value to wire-level metadata
template <uint8_t num, auto u> struct CmdInter {
  constexpr static uint8_t instruction = 0;
  constexpr static uint8_t id   = 0;
  constexpr static bool   hasID = false;
  using sending   = void;
  using returning = void;
};

template <Static ID> struct CmdInter<0, ID> {
  constexpr static uint8_t instruction = static_cast<uint8_t>(ID);
  constexpr static uint8_t id   = 0;
  constexpr static bool   hasID = false;
  using sending   = std::tuple<>;
  using returning = std::tuple<>;
};

// ── CmdInter — maps enum value to wire metadata ─────────────────────────────
template <auto ID> struct CmdInter<2, ID> {
  constexpr static uint8_t instruction = 2;
  constexpr static uint8_t id   = static_cast<uint8_t>(ID);
  constexpr static bool   hasID = true;
  using sending   = typename WriteSendPkt<ID>::type;
  using returning = typename WriteReturnPkt<ID>::type;
};

template <auto ID> struct CmdInter<3, ID> {
  constexpr static uint8_t instruction = 3;
  constexpr static uint8_t id   = static_cast<uint8_t>(ID);
  constexpr static bool   hasID = true;
  using sending   = typename ReadSendPkt<ID>::type;
  using returning = typename ReadReturnPkt<ID>::type;
};
//Write es 2
//Read es 3



// ── Command base ─────────────────────────────────────────────────────────────
struct Command {
  virtual void        pack(uint8_t *buffer) { (void)buffer; }
  virtual std::string info()                { return "";    }
  virtual size_t      getPckSize()          { return 0;     }
  virtual uint8_t     getInst()             { return 0;     }
  virtual uint8_t     getID()               { return 0;     }
  virtual bool        hasID()               { return false; }
  virtual ~Command() = default;
};

// ── GeneralInstruction ───────────────────────────────────────────────────────
template <typename T> struct GeneralInstruction : Command {
  using packet = typename T::sending;

  packet content;
  constexpr static size_t  size = tuple_size(packet{});
  constexpr static uint8_t id   = T::id;

  GeneralInstruction(packet in = packet{}) : content{in} {}

  void pack(uint8_t *buffer) override {
    if constexpr (T::hasID) {
      buffer[0] = static_cast<uint8_t>(id);
      pack_tuple_to_buffer(content, buffer + 1);
    } else {
      pack_tuple_to_buffer(content, buffer);
    }
  }

  std::string info() override {
    std::ostringstream oss;
    oss << "Size: " << getPckSize() << " | ";
    if constexpr (T::hasID) oss << "ID: " << static_cast<int>(id) << " | ";
    oss << "Packet: (";
    std::apply([&oss](auto&&... args) {
      size_t n = 0;
      auto print = [&oss](auto&& v) {
        using U = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<U,uint8_t>  || std::is_same_v<U,int8_t> ||
                      std::is_same_v<U,unsigned char> || std::is_same_v<U,signed char>)
          oss << static_cast<int>(v);
        else oss << v;
      };
      ((print(args), oss << (++n < sizeof...(args) ? ", " : "")), ...);
    }, content);
    oss << ")";
    constexpr auto total = T::hasID ? size + 1 : size;
    std::array<uint8_t, total> buf{};
    pack(buf.data());
    oss << " | Raw: [";
    for (size_t i = 0; i < total; ++i)
      oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
          << static_cast<int>(buf[i]) << (i+1 < total ? " " : "");
    oss << "]";
    return oss.str();
  }

  size_t  getPckSize() override { return T::hasID ? size + 1 : size; }
  uint8_t getID()      override { return id; }
  bool    hasID()      override { return T::hasID; }
  uint8_t getInst()    override { return T::instruction; }
};

// ── Convenient aliases ───────────────────────────────────────────────────────
template <Static T>    using MiscInst     = GeneralInstruction<CmdInter<0, T>>;

template <Teensy::Write T> using WriteInst    = GeneralInstruction<CmdInter<2, T>>;
template <Teensy::Read  T> using ReadInst     = GeneralInstruction<CmdInter<3, T>>;

template <ESP32::Write T> using WriteInstESP = GeneralInstruction<CmdInter<2, T>>;
template <ESP32::Read  T> using ReadInstESP  = GeneralInstruction<CmdInter<3, T>>;

} // namespace Cmd