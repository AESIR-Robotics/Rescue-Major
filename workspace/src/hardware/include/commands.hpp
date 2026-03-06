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
// Declarations / externs
// ---------------------------------------------------------------------------

namespace WriteCommandsNC {
  enum WriteCommand : uint8_t;
  template <WriteCommand CMD> struct packetSend;
  template <WriteCommand CMD> struct packetReturn;
} // namespace WriteCommandsNC

namespace ReadCommandsNC {
  enum ReadCommand : uint8_t;
  template <ReadCommand CMD> struct packetSend;
  template <ReadCommand CMD> struct packetReturn;
  template <ReadCommand ID, typename HandlerFunc>
  bool dispatch_one(const uint8_t *data, size_t size, HandlerFunc &&handle);
} // namespace ReadCommandsNC

namespace CommandsNC {
  enum class StaticCommand : uint8_t;
  template <auto u> struct CmdInter;
  struct Command;
  template <typename T> struct GeneralInstruction;
} // namespace CommandsNC

// ---------------------------------------------------------------------------
// Templates
// ---------------------------------------------------------------------------

// --- WriteCommandsNC --------------------------------------------------------
namespace WriteCommandsNC {

enum WriteCommand : uint8_t { BYTELOSS = 0X00, SPEED = 0x01, POSITION = 0x02, ACCEL = 0x03, DCVEL = 0x04, DCACCEL = 0x05 };

template <WriteCommand CMD> struct packetSend   { using type = void; };
template <WriteCommand CMD> struct packetReturn { using type = void; };

template <> struct packetSend<BYTELOSS>      { using type = std::tuple<>;   }; 
template <> struct packetReturn<BYTELOSS>    { using type = std::tuple<>;   };

template <> struct packetSend<SPEED>      { using type = std::tuple<uint8_t, float>;    }; // motor mask, speed
template <> struct packetReturn<SPEED>    { using type = std::tuple<>;                  };

template <> struct packetSend<POSITION>   { using type = std::tuple<uint8_t, int32_t>;  }; // motor mask, position
template <> struct packetReturn<POSITION> { using type = std::tuple<>;                  };

template <> struct packetSend<ACCEL>   { using type = std::tuple<uint8_t, int32_t>;  }; // motor mask, position
template <> struct packetReturn<ACCEL> { using type = std::tuple<>;                  };

template <> struct packetSend<DCVEL>    { using type = std::tuple<float, float>; };
template <> struct packetReturn<DCVEL>  { using type = std::tuple<>; };

template <> struct packetSend<DCACCEL>    { using type = std::tuple<float, float>; };
template <> struct packetReturn<DCACCEL>  { using type = std::tuple<>; };

} // namespace WriteCommandsNC

// --- ReadCommandsNC ---------------------------------------------------------
namespace ReadCommandsNC {

enum ReadCommand : uint8_t { BYTELOSS = 0X00, SPEED = 0x01, POSITION = 0x02, ACCEL = 0x03, DCVEL = 0x04, DCACCEL = 0x05  };

template <ReadCommand CMD> struct packetSend  { using type = void; };
template <ReadCommand CMD> struct packetReturn { using type = void; };

template <> struct packetSend<BYTELOSS>      { using type = std::tuple<>;   }; 
template <> struct packetReturn<BYTELOSS>    { using type = std::tuple<uint32_t>;   }; // Amount of bytes lost

template <> struct packetSend<SPEED>      { using type = std::tuple<>; };
template <> struct packetReturn<SPEED>    { using type = std::tuple<float, float, float, float>; }; // speed per motor

template <> struct packetSend<POSITION>   { using type = std::tuple<>; };
template <> struct packetReturn<POSITION> {
  using type = std::tuple<int32_t, int32_t, int32_t, int32_t>; // position per motor
};

template <> struct packetSend<ACCEL>   { using type = std::tuple<>;  }; // motor mask, position
template <> struct packetReturn<ACCEL> { using type = std::tuple<int32_t, int32_t, int32_t, int32_t>; };

template <> struct packetSend<DCVEL>    { using type = std::tuple<>; };
template <> struct packetReturn<DCVEL>  { using type = std::tuple<float, float>; };

template <> struct packetSend<DCACCEL>    { using type = std::tuple<>; };
template <> struct packetReturn<DCACCEL>  { using type = std::tuple<float, float>; };

} // namespace ReadCommandsNC

// --- CommandsNC -------------------------------------------------------------
namespace CommandsNC {

enum class StaticCommand : uint8_t {
  Ping = 0x01,
};

// CmdInter: maps a command enum value to wire-level metadata
template <auto u> struct CmdInter {
  constexpr static uint8_t instruction = 0;
  using sending   = void;
  using returning = void;
};

template <StaticCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = static_cast<uint8_t>(ID);
  constexpr static uint8_t id   = 0;
  constexpr static bool   hasID = false;
  using sending   = std::tuple<>;
  using returning = std::tuple<>;
};

template <WriteCommandsNC::WriteCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = 2;
  constexpr static uint8_t id   = ID;
  constexpr static bool   hasID = true;
  using sending   = typename WriteCommandsNC::packetSend<ID>::type;
  using returning = typename WriteCommandsNC::packetReturn<ID>::type;
};

template <ReadCommandsNC::ReadCommand ID> struct CmdInter<ID> {
  constexpr static uint8_t instruction = 3;
  constexpr static uint8_t id   = ID;
  constexpr static bool   hasID = true;
  using sending   = typename ReadCommandsNC::packetSend<ID>::type;
  using returning = typename ReadCommandsNC::packetReturn<ID>::type;
};

// Command: polymorphic base
struct Command {
  virtual void    pack(uint8_t *buffer) { (void)buffer; }
  virtual std::string info()            { return ""; }
  virtual size_t  getPckSize()          { return 0;  }
  virtual uint8_t getInst()             { return 0;  }
  virtual uint8_t getID()               { return 0;  }
  virtual bool    hasID()               { return false; }
  virtual ~Command() = default;
};

// GeneralInstruction: typed concrete command
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

  std::string info() override{
    std::ostringstream oss;
    oss << "Size: " << getPckSize() << " | ";

    if constexpr (T::hasID) {
        oss << "ID: " << static_cast<int>(id) << " | ";
    }

    oss << "Packet: (";

    std::apply([&oss](auto&&... args) {
      size_t n = 0;

      auto print_arg = [&oss](auto&& value) {
          using U = std::decay_t<decltype(value)>;

          if constexpr (std::is_same_v<U, uint8_t> ||
                        std::is_same_v<U, int8_t>  ||
                        std::is_same_v<U, unsigned char> ||
                        std::is_same_v<U, signed char>) {
              oss << static_cast<int>(value);
          } else {
              oss << value;
          }
      };
      
      (void)print_arg;

      ((print_arg(args),
        oss << (++n < sizeof...(args) ? ", " : "")), ...);

    }, content);
    
    constexpr auto totalSize = T::hasID ? size + 1 : size;
    std::array<uint8_t, totalSize> buffer;
    pack(buffer.data());

    oss << " | Raw: [";

    for (size_t i = 0; i < totalSize; ++i) {
        oss << "0x"
            << std::hex
            << std::uppercase
            << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(buffer[i]);

        if (i + 1 < totalSize)
          oss << " ";
    }

    oss << "]";

    return oss.str();

    return oss.str();
  }

  size_t  getPckSize() override { return T::hasID ? size + 1 : size; }
  uint8_t getID()      override { return id; }
  bool    hasID()      override { return T::hasID; }
  uint8_t getInst()    override { return T::instruction; }
};

// Convenient aliases
template <StaticCommand T>
using MiscInst = GeneralInstruction<CmdInter<T>>;

template <WriteCommandsNC::WriteCommand T>
using WriteInst = GeneralInstruction<CmdInter<T>>;

template <ReadCommandsNC::ReadCommand T>
using ReadInst = GeneralInstruction<CmdInter<T>>;

} // namespace CommandsNC

// ---------------------------------------------------------------------------
// Inline definitions
// ---------------------------------------------------------------------------

namespace ReadCommandsNC {

/// Deserialise a read-response payload and invoke handle(tuple).
/// Returns false if the payload size does not match the expected type.
template <ReadCommand ID, typename HandlerFunc>
inline bool dispatch_one(const uint8_t *data, size_t size, HandlerFunc &&handle) {
  using payload = typename packetReturn<ID>::type;

  if (size != tuple_size(payload{})) {
    return false;
  }

  payload p;
  unpack_tuple_from_buffer(p, data);
  handle(std::move(p));
  return true;
}

} // namespace ReadCommandsNC