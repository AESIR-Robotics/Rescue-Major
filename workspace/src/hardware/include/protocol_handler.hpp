#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>

#include "commands.hpp"
#include "crc.hpp"
#include "tuple_utils.hpp"

// micros / stdclock / deadline_t / LogFn come from the transport header,
// included transitively. Declare them here too so this file is self-contained
// when used with a non-I2C transport.
#include <functional>
using micros    = std::chrono::microseconds;
using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;
using LogFn      = std::function<void(const std::string &)>;

// =============================================================================
// Protocol_Handler<Transport>
//
// Transport-agnostic framing layer. Inherits from Transport to gain access to:
//   Transport::writeData(data, length, deadline) → size_t
//   Transport::readData(buffer, length, deadline) → size_t
//   Transport::connected()                        → bool
//   Transport::INTERNAL_BUF_SIZE                  → size_t (constexpr)
//   Transport::transport_error_                   → Transport::Transport_Error
//
// The protocol layer owns:
//   - Command queue (sendQueue / addCommand)
//   - Message framing (header / tail / CRC)
//   - Sync-byte state machine (readPending)
//   - Dispatch to read_callbacks / write_callbacks
//   - Protocol-level error (IO_ERROR)
// =============================================================================

template <typename Transport, typename Identifier>
class Protocol_Handler : public Transport {
public:

  using header = std::tuple<uint8_t, uint8_t, uint8_t>; // sync, inst, length
  using tail   = std::tuple<uint8_t>;                    // CRC

  // callbacks must be non-blocking, O(1) — no locks, no I/O, no heavy computation
  std::unordered_map<Identifier,
                     std::function<void(const uint8_t *, size_t)>> read_callbacks{};
  std::unordered_map<WriteCommandsNC::WriteCommand,
                     std::function<void(const uint8_t *, size_t)>> write_callbacks{};

  // ── Forwarding constructor — passes args to Transport ─────────────────────
  template <typename... Args>
  explicit Protocol_Handler(Args&&... args)
      : Transport(std::forward<Args>(args)...) {
    setupInstructionCallbacks();
  }

  // ── Public API ─────────────────────────────────────────────────────────────

  bool addCommand(std::unique_ptr<CommandsNC::Command> &&input);

  /// Flush the outbound queue. Sends at most one full frame per call.
  bool sendQueue(micros timeout = micros(8000), micros timePerMsg = micros(4000));

  /// Drain incoming messages and dispatch callbacks.
  /// Returns true if at least one message was dispatched.
  bool readPending(micros timeout = micros(8000), micros timePerMsg = micros(4000));

  /// Maximum payload bytes per message — mirrors MAX_PAYLOAD_SIZE on the MCU
  /// (MAX_PACKET_SIZE=64 minus 4 bytes of framing).
  static constexpr size_t MAX_PAYLOAD_SIZE = 60;

private:
  // ── ReadResult — internal to the framing layer ────────────────────────────
  enum class ReadResult {
    OK_DISPATCHED,
    NO_MESSAGE,
    CRC_MISMATCH,
    IO_ERROR,
  };

  void       setupInstructionCallbacks();
  size_t     sendNext(deadline_t deadline);
  ReadResult readOneMessage(micros timePerMsg);
  ReadResult ReadHeader(header &output, deadline_t hdr_dl);
  uint8_t    getMsgCRC(const header &msg_head, uint8_t *pckage, size_t size);
  void       dispatchInput(uint8_t inst, uint8_t *pckg, size_t size);

  // ── Sync state ────────────────────────────────────────────────────────────
  // readPending scans for 0xAA using the global deadline and stores it here.
  // readOneMessage consumes it without re-scanning.
  bool    already_synced_ { false };
  uint8_t synced_byte_    { 0x00  };

  // Diagnostic counters — bytes skipped during sync scan.
  uint32_t lost_bytes_  { 0 };
  uint32_t total_attmp_ { 0 };

  constexpr static unsigned int max_queue { 50 };
  std::queue<std::unique_ptr<CommandsNC::Command>> sending;

  std::unordered_map<uint8_t, std::function<void(const uint8_t *, size_t)>>
      instruction_callback{};

  // Log helpers — delegate to Transport::logXxx (inherited via using or friend).
  // We call them directly since Protocol_Handler inherits Transport's private
  // log members indirectly — instead, re-expose Transport's setLogger and use
  // the same LogFn pattern here for protocol-level messages.
  LogFn proto_log_warn_{};
  LogFn proto_log_error_{};
  LogFn proto_log_info_{};

  template<typename... Args>
  void logInfo (const char *fmt, Args&&... args) const { logDispatch(proto_log_info_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logWarn (const char *fmt, Args&&... args) const { logDispatch(proto_log_warn_,  fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  void logError(const char *fmt, Args&&... args) const { logDispatch(proto_log_error_, fmt, std::forward<Args>(args)...); }
  template<typename... Args>
  static void logDispatch(const LogFn &fn, const char *fmt, Args&&... args) {
    if (!fn) return;
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, std::forward<Args>(args)...);
    fn(buf);
  }

public:
  // Override setLogger to set both Transport and Protocol_Handler loggers.
  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    proto_log_info_  = info;
    proto_log_warn_  = warn;
    proto_log_error_ = error;
    Transport::setLogger(std::move(info), std::move(warn), std::move(error));
  }
};

// =============================================================================
// Convenience alias — the only concrete type callers should use directly
// =============================================================================
#include "i2c_transport.hpp"
template<typename Identifier>
using Protocol_Handler_I2C = Protocol_Handler<I2C_Transport, Identifier>;

#include "can_transport.hpp"
template<typename Identifier>
using Protocol_Handler_CAN = Protocol_Handler<CAN_Transport, Identifier>;

#include "serial_transport.hpp"
template<typename Identifier>
using Protocol_Handler_SERIAL = Protocol_Handler<Serial_Transport, Identifier>;

// =============================================================================
// Template definitions
// =============================================================================

template <typename Transport, typename Identifier>
void Protocol_Handler<Transport, Identifier>::setupInstructionCallbacks() {
  instruction_callback.emplace(1, [](const uint8_t *pckg, size_t size) {
    (void)pckg; (void)size;
  });
  instruction_callback.emplace(2, [this](const uint8_t *pckg, size_t size) {
    uint8_t id = pckg[0];
    auto it = write_callbacks.find(static_cast<WriteCommandsNC::WriteCommand>(id));
    if (it != write_callbacks.end()) it->second(pckg + 1, size - 1);
  });
  instruction_callback.emplace(3, [this](const uint8_t *pckg, size_t size) {
    uint8_t id = pckg[0];
    auto it = read_callbacks.find(static_cast<Identifier>(id));
    if (it != read_callbacks.end()) it->second(pckg + 1, size - 1);
  });
}

template <typename Transport, typename Identifier>
bool Protocol_Handler<Transport, Identifier>::addCommand(
    std::unique_ptr<CommandsNC::Command> &&input) {
  if (sending.size() < max_queue) {
    sending.push(std::move(input));
    return true;
  }
  return false;
}

template <typename Transport, typename Identifier>
bool Protocol_Handler<Transport, Identifier>::sendQueue(micros timeout, micros timePerMsg) {
  if (!this->connected()) return false;

  const deadline_t dl = stdclock::now() + timeout;
  size_t bytes{0};

  constexpr size_t SEND_BUDGET =
      MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{});

  size_t expectedsize{0};
  while (!sending.empty() &&
         (expectedsize = sending.front()->getPckSize() +
                         tuple_size(header{}) + tuple_size(tail{})) &&
         bytes + expectedsize <= SEND_BUDGET &&
         stdclock::now() + timePerMsg < dl) {
    auto sent = sendNext(dl);
    if (sent != expectedsize) {
      return false;
    }
    bytes += sent;
  }
  return true;
}

template <typename Transport, typename Identifier>
size_t Protocol_Handler<Transport, Identifier>::sendNext(deadline_t deadline) {
  if (!this->connected() || sending.empty()) return 0;

  auto size = sending.front()->getPckSize();
  constexpr auto headerSize = tuple_size(header{});
  constexpr auto tailSize   = tuple_size(tail{});

  std::array<uint8_t, MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{})> buffer{};
  const size_t frameSize = size + headerSize + tailSize;

  auto headerSend = make_msg_from_args(header{}, 0xAA, sending.front()->getInst(), size);
  pack_tuple_to_buffer(headerSend, buffer.data());
  sending.front()->pack(buffer.data() + headerSize);

  auto tail_tuple = make_msg_from_args(
      tail{}, calcCRC(buffer.data(), headerSize + size, 0));
  pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + size);

  auto sent = this->writeData(buffer.data(), frameSize, deadline);
  if (sent == frameSize) sending.pop();
  return sent;
}

template <typename Transport, typename Identifier>
bool Protocol_Handler<Transport, Identifier>::readPending(micros timeout, micros timePerMsg) {
  if (!this->connected()) return false;

  const deadline_t dl = stdclock::now() + timeout;
  bool dispatched_any = false;

  while (stdclock::now() < dl) {
    if (!already_synced_) {
      bool found = false;
      uint8_t byte = 0;
      while (stdclock::now() < dl) {
        if (this->readData(&byte, 1, dl) != 1) break;
        if (byte != 0xBB) { lost_bytes_++; }
        total_attmp_++;
        if (byte == 0xAA) { found = true; break; }
      }
      if (!found) {
        logWarn("Could not sync to message HEADER: lost %u, total %u",
                lost_bytes_, total_attmp_);
        total_attmp_ = 0;
        break;
      }
      total_attmp_   = 0;
      synced_byte_   = byte;
      already_synced_ = true;
    }

    if (stdclock::now() + timePerMsg >= dl) break;

    if (lost_bytes_ > 1)
      logWarn("Skipped %u bytes in sync", lost_bytes_ - 1);
    lost_bytes_    = 0;
    already_synced_ = false;

    auto res = readOneMessage(timePerMsg);
    if (res == ReadResult::OK_DISPATCHED) { dispatched_any = true; continue; }
    if (res == ReadResult::NO_MESSAGE)    { break; }
    if (res == ReadResult::CRC_MISMATCH)  { logWarn("CRC Mismatch"); continue; }

    logError("I/O error occurred while reading message");
    return false;
  }
  return dispatched_any;
}

template <typename Transport, typename Identifier>
typename Protocol_Handler<Transport, Identifier>::ReadResult
Protocol_Handler<Transport, Identifier>::readOneMessage(micros timePerMsg) {
  if (!this->connected()) return ReadResult::IO_ERROR;

  const deadline_t msg_dl = stdclock::now() + timePerMsg;

  header msg_head;
  auto state = ReadHeader(msg_head, msg_dl);
  if (state != ReadResult::OK_DISPATCHED) return state;

  const uint8_t inst   = std::get<1>(msg_head);
  const uint8_t length = std::get<2>(msg_head);

  if (length > MAX_PAYLOAD_SIZE) {
    logWarn("Payload length %u exceeds MAX_PAYLOAD_SIZE (%zu) — dropping",
            static_cast<unsigned>(length), MAX_PAYLOAD_SIZE);
    return ReadResult::IO_ERROR;
  }

  std::array<uint8_t, MAX_PAYLOAD_SIZE + 1> out_buffer{};
  auto num = this->readData(out_buffer.data(), length + 1, msg_dl);
  if (num != static_cast<size_t>(length + 1)) return ReadResult::IO_ERROR;

  uint8_t recv_crc       = out_buffer[length];
  auto    calculated_crc = getMsgCRC(msg_head, out_buffer.data(), length);
  if (recv_crc != calculated_crc) return ReadResult::CRC_MISMATCH;

  dispatchInput(inst, out_buffer.data(), length);
  return ReadResult::OK_DISPATCHED;
}

template <typename Transport, typename Identifier>
typename Protocol_Handler<Transport, Identifier>::ReadResult
Protocol_Handler<Transport, Identifier>::ReadHeader(header &output, deadline_t hdr_dl) {
  uint8_t header_buf[sizeof(output)];
  header_buf[0] = synced_byte_;  // 0xAA already validated by readPending

  auto num = this->readData(header_buf + 1, sizeof(header_buf) - 1, hdr_dl);
  if (num != (sizeof(header_buf) - 1)) return ReadResult::IO_ERROR;

  unpack_tuple_from_buffer(output, header_buf);

  if (output == header{0xAA, 0x00, calcCRC({0xAA, 0x00}, 0)})
    return ReadResult::NO_MESSAGE;

  return ReadResult::OK_DISPATCHED;
}

template <typename Transport, typename Identifier>
uint8_t Protocol_Handler<Transport, Identifier>::getMsgCRC(const header &msg_head,
                                                uint8_t *pckage, size_t size) {
  uint8_t tmp[sizeof(header)];
  pack_tuple_to_buffer(msg_head, tmp);
  uint8_t crc = calcCRC(tmp, sizeof(header), 0);
  crc         = calcCRC(pckage, size, crc);
  return crc;
}

template <typename Transport, typename Identifier>
void Protocol_Handler<Transport, Identifier>::dispatchInput(uint8_t inst, uint8_t *pckg,
                                                 size_t size) {
  auto it = instruction_callback.find(inst);
  if (it != instruction_callback.end()) it->second(pckg, size);
}