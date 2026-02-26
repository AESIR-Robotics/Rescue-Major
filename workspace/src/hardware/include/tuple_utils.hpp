#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <tuple>
#include <type_traits>
#include <utility>

// ---------------------------------------------------------------------------
// tuple_size: total byte size of all element types in a tuple
// ---------------------------------------------------------------------------
template <typename... Args>
constexpr size_t tuple_size(const std::tuple<Args...> &) {
  return (sizeof(Args) + ... + 0);
}

// ---------------------------------------------------------------------------
// pack_tuple_to_buffer: serialise a tuple into a contiguous byte buffer
// ---------------------------------------------------------------------------
template <size_t I, typename... Args>
std::enable_if_t<I == sizeof...(Args), void>
pack_tuple_recursive(const std::tuple<Args...> &, uint8_t * /*buffer*/,
                     size_t & /*offset*/) {}

template <size_t I = 0, typename... Args>
    std::enable_if_t < I<sizeof...(Args), void>
                       pack_tuple_recursive(const std::tuple<Args...> &tup,
                                            uint8_t *buffer, size_t &offset) {
  using T = std::decay_t<decltype(std::get<I>(tup))>;
  std::memcpy(buffer + offset, &std::get<I>(tup), sizeof(T));
  offset += sizeof(T);
  pack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void pack_tuple_to_buffer(const std::tuple<Args...> &tuple, uint8_t *buffer) {
  size_t offset = 0;
  pack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

// ---------------------------------------------------------------------------
// unpack_tuple_from_buffer: deserialise a tuple from a contiguous byte buffer
// ---------------------------------------------------------------------------
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
  std::memcpy(&std::get<I>(tup), buffer + offset, sizeof(T));
  offset += sizeof(T);
  unpack_tuple_recursive<I + 1, Args...>(tup, buffer, offset);
}

template <typename... Args>
void unpack_tuple_from_buffer(std::tuple<Args...> &tuple,
                              const uint8_t *buffer) {
  size_t offset = 0;
  unpack_tuple_recursive<0, Args...>(tuple, buffer, offset);
}

// ---------------------------------------------------------------------------
// args_match_tuple: check that a parameter pack matches a tuple's types
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// tuple_push_back / tuple_push_front
// ---------------------------------------------------------------------------
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