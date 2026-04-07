#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <functional>
#include <tuple>
#include <type_traits>
#include <mutex>
//#include <utility>

// ---------------------------------------------------------------------------
// Declarations / externs
// ---------------------------------------------------------------------------

template <typename... Args>
constexpr size_t tuple_size(const std::tuple<Args...> &);

template <typename... TupleArgs, typename... Args>
auto make_msg_from_args(std::tuple<TupleArgs...>, const Args &...args);

template <typename... Args>
void pack_tuple_to_buffer(const std::tuple<Args...> &tuple, uint8_t *buffer);

template <typename... Args>
void unpack_tuple_from_buffer(std::tuple<Args...> &tuple, const uint8_t *buffer);

// ---------------------------------------------------------------------------
// Templates
// ---------------------------------------------------------------------------

// --- Endianness helpers -----------------------------------------------------
// Assumption documented here: float is IEEE 754 single-precision (32-bit) on
// both host and MCU. This is true for all ARM Cortex-M and x86/x64 targets but
// is not guaranteed by the C++ standard.

/// to_le<T>: return the little-endian byte representation of a value.
/// On a little-endian host this is a no-op; on big-endian it byte-swaps.
/// Only arithmetic types (int8, int16, int32, int64, float, bool, …) are supported.
template <typename T>
inline T to_le(T value) {
  static_assert(std::is_arithmetic_v<T>,
                "to_le only supports arithmetic types");
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return value; 
#else
  // Big-endian host: byte-swap using the appropriately-sized builtin.
  if constexpr (sizeof(T) == 1) {
    return value;
  } else if constexpr (sizeof(T) == 2) {
    uint16_t raw;
    std::memcpy(&raw, &value, 2);
    raw = __builtin_bswap16(raw);
    T out;
    std::memcpy(&out, &raw, 2);
    return out;
  } else if constexpr (sizeof(T) == 4) {
    uint32_t raw;
    std::memcpy(&raw, &value, 4);
    raw = __builtin_bswap32(raw);
    T out;
    std::memcpy(&out, &raw, 4);
    return out;
  } else if constexpr (sizeof(T) == 8) {
    uint64_t raw;
    std::memcpy(&raw, &value, 8);
    raw = __builtin_bswap64(raw);
    T out;
    std::memcpy(&out, &raw, 8);
    return out;
  }
#endif
}

/// from_le<T>: interpret a little-endian wire value as host-endian.
/// Symmetric with to_le — byte-swap only on big-endian hosts.
template <typename T>
inline T from_le(T value) {
  return to_le(value);   // byte-swap is its own inverse
}

// --- tuple_size -------------------------------------------------------------

template <typename... Args>
constexpr size_t tuple_size(const std::tuple<Args...> &) {
  return (sizeof(Args) + ... + 0);
}

// --- pack_tuple_to_buffer ---------------------------------------------------
// Serialises each element through to_le before writing to the buffer,
// ensuring little-endian wire format regardless of host endianness.

template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
pack_tuple_recursive(const std::tuple<Args...> &, uint8_t * /*buffer*/,
                     size_t & /*offset*/) {}

template <size_t I = 0, typename... Args>
    std::enable_if_t < I<sizeof...(Args), void>
                       pack_tuple_recursive(const std::tuple<Args...> &tup,
                                            uint8_t *buffer, size_t &offset) {
  using T    = std::decay_t<decltype(std::get<I>(tup))>;
  const T le = to_le(std::get<I>(tup));
  std::memcpy(buffer + offset, &le, sizeof(T));
  offset += sizeof(T);
  pack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void pack_tuple_to_buffer(const std::tuple<Args...> &tuple, uint8_t *buffer) {
  size_t offset = 0;
  pack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

// --- unpack_tuple_from_buffer -----------------------------------------------
// Reads each element from the buffer and passes it through from_le,
// converting from little-endian wire format to host endianness.

template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
unpack_tuple_recursive(std::tuple<Args...> &, const uint8_t * /*buffer*/,
                       size_t & /*offset*/) {}

template <size_t I = 0, typename... Args>
    std::enable_if_t <
    I<sizeof...(Args), void> unpack_tuple_recursive(std::tuple<Args...> &tup,
                                                    const uint8_t *buffer,
                                                    size_t &offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  T raw;
  std::memcpy(&raw, buffer + offset, sizeof(T));
  std::get<I>(tup) = from_le(raw);
  offset += sizeof(T);
  unpack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void unpack_tuple_from_buffer(std::tuple<Args...> &tuple,
                              const uint8_t *buffer) {
  size_t offset = 0;
  unpack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

// --- args_match_tuple -------------------------------------------------------

template <typename Tuple, typename... Args> struct args_match_tuple;

template <typename... Ts, typename... Args>
struct args_match_tuple<std::tuple<Ts...>, Args...> {
  static constexpr bool value = (sizeof...(Ts) == sizeof...(Args)) &&
                                (std::is_constructible_v<Ts, Args> && ...);
};

template <typename... TupleArgs, typename... Args>
auto make_msg_from_args(std::tuple<TupleArgs...>, const Args &...args) {
  static_assert(args_match_tuple<std::tuple<TupleArgs...>, Args...>::value,
                "Argument types do not match tuple types");
  return std::tuple<TupleArgs...>{(static_cast<TupleArgs>(args))...};
}


template <typename Tuple, typename NewType> struct tuple_push_back;

template <typename... Ts, typename NewType>
struct tuple_push_back<std::tuple<Ts...>, NewType> {
  using type = std::tuple<Ts..., NewType>;
};

template <typename Tuple, typename NewType>
using tuple_push_back_t = typename tuple_push_back<Tuple, NewType>::type;

template <typename Tuple, typename NewType> struct tuple_push_front;

template <typename... Ts, typename NewType>
struct tuple_push_front<std::tuple<Ts...>, NewType> {
  using type = std::tuple<NewType, Ts...>;
};

template <typename Tuple, typename NewType>
using tuple_push_front_t = typename tuple_push_front<Tuple, NewType>::type;

template <typename T>
struct Guarded {  
  template <typename F>
  auto with(F &&fn) {
      std::lock_guard<std::mutex> lock{mtx};
      return fn(data);
  }

  T snapshot() const {
      std::lock_guard<std::mutex> lock{mtx};
      return data;
  }
  
  private:

  T data;
  mutable std::mutex mtx;
};