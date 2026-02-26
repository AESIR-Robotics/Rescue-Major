#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>

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

enum WriteCommand : uint8_t { DIRECTION = 0x00, SPEED = 0x01, POSITION = 0x02 };

template <WriteCommand CMD> struct packetSend  { using type = void; };
template <WriteCommand CMD> struct packetReturn { using type = void; };

template <> struct packetSend<DIRECTION>  { using type = std::tuple<uint8_t, bool>;    }; // motor mask, direction
template <> struct packetReturn<DIRECTION>{ using type = std::tuple<>;                  };

template <> struct packetSend<SPEED>      { using type = std::tuple<uint8_t, float>;   }; // motor mask, speed
template <> struct packetReturn<SPEED>    { using type = std::tuple<>;                  };

template <> struct packetSend<POSITION>   { using type = std::tuple<uint8_t, int32_t>; }; // motor mask, position
template <> struct packetReturn<POSITION> { using type = std::tuple<>;                  };

} // namespace WriteCommandsNC

// --- ReadCommandsNC ---------------------------------------------------------
namespace ReadCommandsNC {

enum ReadCommand : uint8_t { DIRECTION = 0x00, SPEED = 0x01, POSITION = 0x02 };

template <ReadCommand CMD> struct packetSend  { using type = void; };
template <ReadCommand CMD> struct packetReturn { using type = void; };

template <> struct packetSend<DIRECTION>  { using type = std::tuple<>; };
template <> struct packetReturn<DIRECTION>{ using type = std::tuple<bool, bool, bool, bool>; }; // direction per motor

template <> struct packetSend<SPEED>      { using type = std::tuple<>; };
template <> struct packetReturn<SPEED>    { using type = std::tuple<float, float, float, float>; }; // speed per motor

template <> struct packetSend<POSITION>   { using type = std::tuple<>; };
template <> struct packetReturn<POSITION> {
  using type = std::tuple<int32_t, int32_t, int32_t, int32_t>; // position per motor
};

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
  virtual size_t  getPckSize()          { return 0; }
  virtual uint8_t getInst()             { return 0; }
  virtual uint8_t getID()               { return 0; }
  virtual bool    hasID()               { return false; }
  virtual ~Command() = default;
};

// GeneralInstruction: typed concrete command
template <typename T> struct GeneralInstruction : Command {
  using packet = typename T::sending;

  packet info;
  constexpr static size_t  size = tuple_size(packet{});
  constexpr static uint8_t id   = T::id;

  GeneralInstruction(packet in = packet{}) : info{in} {}

  void pack(uint8_t *buffer) override {
    if constexpr (T::hasID) {
      buffer[0] = static_cast<uint8_t>(id);
      pack_tuple_to_buffer(info, buffer + 1);
    } else {
      pack_tuple_to_buffer(info, buffer);
    }
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