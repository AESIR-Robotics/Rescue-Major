#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>
#include <list>

#include <memory>
#include <queue>
#include <string>
#include <sys/types.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "commands.hpp"
#include "utils/crc.hpp"
#include "utils/tuple_utils.hpp"
#include "utils/diagnostics.hpp"
#include "utils/logger.hpp"

#include "transport/transport.hpp"
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

template <typename Transport, typename ReadEnum, typename WriteEnum>
class Protocol_Handler : public Transport {
public:

  // Wire format: [0xAA][inst][seq_id][length][payload][CRC]
  using header = std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>; // sync, inst, seq_id, length
  using tail   = std::tuple<uint8_t>;                             // CRC

  // callbacks must be non-blocking, O(1) — no locks, no I/O, no heavy computation
  std::unordered_map<ReadEnum,
                     std::function<void(const uint8_t *, size_t)>> read_callbacks{};
  std::unordered_map<WriteEnum,
                     std::function<void(const uint8_t *, size_t)>> write_callbacks{};

  // ── Forwarding constructor — passes args to Transport ─────────────────────
  template <typename... Args>
  explicit Protocol_Handler(DiagnosticRegistry *reg = nullptr, Args&&... args)
      : Transport(reg, std::forward<Args>(args)...) {
    setupInstructionCallbacks();
    Transport::setEventCallback([this](TransportInterface::Event ev) {
        if (ev == TransportInterface::Event::DISCONNECTED) {
            resetProtocolState();
        }
    });
    // Initialize protocol-level diagnostic keys in the shared statusReport
    Transport::statusReport.with([](Status &s) {
      for (const char *k : {"crc_errors","sync_failures","bytes_skipped",
                            "msgs_dispatched","retransmits","discarded",
                            "coalesced","partial_acks","pending_cmds",
                            "queue_depth","channel_dead","consecutive_unacked"})
        s.values.emplace(k, "0");
    });
  }

  ~Protocol_Handler(){
    Transport::clearDisconnectCallback(); 
  }

  // ── Public API ─────────────────────────────────────────────────────────────

  bool addCommand(std::unique_ptr<Cmd::Command> &&input);
  int msgQueued();
  int msgPending();
  int byteSentFromQueue();
  /// Check retransmission queue — call once per tick before sendQueue.
  /// Returns true if any message was re-queued for retransmission.
  //  bool checkRetransmissions(micros retry_timeout = micros(50000));
  constexpr static micros retry_timeout = micros(50000);

  /// True if the channel is declared disconnected due to too many
  /// consecutive unacked messages.
  bool isChannelDead() const { return channel_dead_; }

  /// Reset the channel-dead flag (call after successful reconnect).
  void resetChannel() {
    channel_dead_         = false;
    consecutive_unacked_  = 0;
    // Drain pending — stale commands from before reconnect are useless
    sending = std::queue<std::unique_ptr<Cmd::Command>>{};
    waitingCmds_.clear();
    cmds_lookout.clear();
    
  }

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

  struct PendingCommand {
    std::unique_ptr<Cmd::Command> cmd;
    uint8_t                       seq_id;
    std::unordered_set<uint8_t>   old_seq;
    uint8_t                       retries_left;
    stdclock::time_point          first_sent;
    stdclock::time_point          last_sent;
  };

  void       setupInstructionCallbacks();
  size_t     sendNext(Cmd::Command *operation, uint8_t seq, deadline_t deadline);
  ReadResult readOneMessage(micros timePerMsg);
  ReadResult ReadHeader(header &output, deadline_t hdr_dl);
  uint8_t    getMsgCRC(const header &msg_head, uint8_t *pckage, size_t size);
  void       dispatchInput(uint8_t inst, uint8_t seq, uint8_t *pckg, size_t size);
  void       processAck(uint8_t id, uint8_t inst, uint8_t seq_id);
  uint8_t    allocSeq();
  bool       coalesceingAddition(std::unique_ptr<Cmd::Command> &&cmd, uint8_t new_seq);


  void resetProtocolState() {
    sending = {}; 

    cmds_lookout.clear();
    waitingCmds_.clear();

    already_synced_ = false;
    synced_byte_ = 0;

    consecutive_unacked_ = 0;
    channel_dead_ = false;
  }

  // ── Sync state ────────────────────────────────────────────────────────────
  // readPending scans for 0xAA using the global deadline and stores it here.
  // readOneMessage consumes it without re-scanning.
  bool    already_synced_ { false };
  uint8_t synced_byte_    { 0x00  };

  // Diagnostic counters — bytes skipped during sync scan.
  uint32_t lost_bytes_  { 0 };
  uint32_t total_attmp_ { 0 };

  std::list<PendingCommand> waitingCmds_;

  using MsgKey = std::pair<uint8_t, uint8_t>;

  struct PairHash {
    std::size_t operator()(const MsgKey& p) const {
        return (static_cast<std::size_t>(p.first) << 8) |
               static_cast<std::size_t>(p.second);
      }
  };

  std::unordered_map<
      MsgKey,
      typename std::list<PendingCommand>::iterator,
      PairHash
  > cmds_lookout;

  uint8_t  next_seq_         { 0 };
  uint8_t  consecutive_unacked_{ 0 };
  bool     channel_dead_     { false };
  static constexpr uint8_t  max_consecutive_unacked_ { 5 };
  static constexpr uint8_t  max_retries_             { 3 };
  static constexpr size_t   max_pending_             { 256 };

  constexpr static unsigned int max_queue { 50 };
  uint8_t bytes_sent{0};
  std::queue<std::unique_ptr<Cmd::Command>> sending;

  std::unordered_map<uint8_t, std::function<void(const uint8_t *, uint8_t, size_t)>>
      instruction_callback{};

  Logger log{};

  // ── Diagnostic counters ─────────────────────────────────────────────────
  uint32_t stat_crc_errors_      { 0 };
  uint32_t stat_sync_failures_   { 0 };
  uint32_t stat_bytes_skipped_   { 0 };
  uint32_t stat_msgs_dispatched_ { 0 };
  uint32_t stat_retransmits_     { 0 };
  uint32_t stat_discarded_       { 0 };
  uint32_t stat_coalesced_       { 0 };
  uint32_t stat_partial_acks_    { 0 };

  void updateStatus() {
    Transport::statusReport.with([this](Status &s) {
      s.values["crc_errors"]      = std::to_string(stat_crc_errors_);
      s.values["sync_failures"]   = std::to_string(stat_sync_failures_);
      s.values["bytes_skipped"]   = std::to_string(stat_bytes_skipped_);
      s.values["msgs_dispatched"] = std::to_string(stat_msgs_dispatched_);
      s.values["retransmits"]     = std::to_string(stat_retransmits_);
      s.values["discarded"]       = std::to_string(stat_discarded_);
      s.values["coalesced"]       = std::to_string(stat_coalesced_);
      s.values["partial_acks"]    = std::to_string(stat_partial_acks_);
      s.values["pending_cmds"]    = std::to_string(waitingCmds_.size());
      s.values["queue_depth"]     = std::to_string(sending.size());
      s.values["channel_dead"]    = channel_dead_ ? "true" : "false";
      s.values["consecutive_unacked"] = std::to_string(consecutive_unacked_);
    });
  }

public:
  // Override setLogger to set both Transport and Protocol_Handler loggers.
  void setLogger(Logger &in_log) {
    log = in_log;
    Transport::setLogger(log);
  }
};

// =============================================================================
// Convenience alias — the only concrete type callers should use directly
// =============================================================================
#include "transport/i2c_transport.hpp"
template<typename ReadID = Cmd::Teensy::Read, typename WriteID = Cmd::Teensy::Write>
using Protocol_Handler_I2C = Protocol_Handler<I2C_Transport, ReadID, WriteID>;


#include "transport/can_transport.hpp"
// CAN_Transport does not use flock — each instance opens its own socket
// and uses a hardware RX filter (rx_id) for isolation. Multiple instances
// on the same interface are safe because the kernel routes frames by ID.
template<typename ReadID = Cmd::ESP32::Read, typename WriteID = Cmd::ESP32::Write>
using Protocol_Handler_CAN = Protocol_Handler<CAN_Transport, ReadID, WriteID>;

#include "transport/serial_transport.hpp"
template<typename ReadID = Cmd::Teensy::Read, typename WriteID = Cmd::Teensy::Write>
using Protocol_Handler_SERIAL = Protocol_Handler<Serial_Transport, ReadID, WriteID>;

// =============================================================================
// Template definitions
// =============================================================================


template <typename Transport, typename ReadEnum, typename WriteEnum>
void Protocol_Handler<Transport, ReadEnum, WriteEnum>::setupInstructionCallbacks() {
  instruction_callback.emplace(1, [this](const uint8_t *pckg, uint8_t seq, size_t size) {
    processAck(0, 1, seq);
    (void)pckg; (void)size;
  });
  instruction_callback.emplace(2, [this](const uint8_t *pckg, uint8_t seq, size_t size) {
    if(size == 0){return;}
    uint8_t id = pckg[0];
    processAck(id, 2, seq);
    auto it = write_callbacks.find(static_cast<WriteEnum>(id));
    if (it != write_callbacks.end()) it->second(pckg + 1, size - 1);
  });
  instruction_callback.emplace(3, [this](const uint8_t *pckg, uint8_t seq, size_t size) {
    if(size == 0){return;}
    uint8_t id = pckg[0];
    processAck(id, 3, seq);
    auto it = read_callbacks.find(static_cast<ReadEnum>(id));
    if (it != read_callbacks.end()) it->second(pckg + 1, size - 1);
  });
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
bool Protocol_Handler<Transport, ReadEnum, WriteEnum>::addCommand(
    std::unique_ptr<Cmd::Command> &&input) {
  if (sending.size() < max_queue) {
    sending.push(std::move(input));
    return true;
  }
  return false;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
inline int Protocol_Handler<Transport, ReadEnum, WriteEnum>::msgQueued() {
  return sending.size();
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
inline int Protocol_Handler<Transport, ReadEnum, WriteEnum>::msgPending() {
  return waitingCmds_.size();
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
inline int Protocol_Handler<Transport, ReadEnum, WriteEnum>::byteSentFromQueue() {
  return bytes_sent;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
bool Protocol_Handler<Transport, ReadEnum, WriteEnum>::sendQueue(micros timeout, micros timePerMsg) {
  if (!this->connected()) return false;

  const deadline_t dl = stdclock::now() + timeout;
  bytes_sent = 0;

  constexpr size_t SEND_BUDGET =
      MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{});

  size_t expectedsize{0};
  while(Transport::canSend() && stdclock::now() + timePerMsg < dl){
    if(!waitingCmds_.empty()){
      auto& elem = waitingCmds_.front();
      auto age_since_send = std::chrono::duration_cast<micros>(stdclock::now() - elem.last_sent);

      if (age_since_send >= retry_timeout) { 
        if(elem.retries_left == 0){
          log.logWarn("Seq=%u exhausted retries (inst=%u id=%u) — discarding",
                elem.seq_id, elem.cmd->getInst(), elem.cmd->getID());

          auto it = cmds_lookout.find({elem.cmd->getInst(), elem.cmd->getID()});
          if (it == cmds_lookout.end()) {
            // Should never happen
            waitingCmds_.erase(waitingCmds_.begin());
          }
          auto list_it = it->second;
          waitingCmds_.erase(list_it);
          cmds_lookout.erase(it);

          ++stat_discarded_;
          ++consecutive_unacked_;
          if (consecutive_unacked_ >= max_consecutive_unacked_) {
            channel_dead_ = true;
            log.logError("Channel declared dead after %u consecutive unacked messages",
                    consecutive_unacked_);
          }
          continue;
        }

        expectedsize = elem.cmd->getPckSize() + tuple_size(header{}) + tuple_size(tail{});
        if(bytes_sent + expectedsize > SEND_BUDGET) break;
        
        uint8_t new_seq = allocSeq();
        auto sent = sendNext(elem.cmd.get(), new_seq, dl);
        bytes_sent += sent;
        if (sent != expectedsize) {updateStatus(); return false;}

        ++stat_retransmits_;
        // Keep old seq in old_seq set so late ACKs still resolve
        elem.old_seq.insert(elem.seq_id);
        elem.seq_id      = new_seq;
        elem.retries_left--;
        elem.last_sent   = stdclock::now();

        // Move to back of list (LRU: oldest retry at front)
        waitingCmds_.splice(waitingCmds_.end(), waitingCmds_, waitingCmds_.begin());

        log.logInfo("Retransmit: new_seq=%u retries_left=%u inst=%u id=%u",
                    new_seq, elem.retries_left,
                    elem.cmd->getInst(), elem.cmd->getID());
        
        continue;
      }
    }

    if(sending.empty()) break;
    expectedsize = sending.front()->getPckSize() + tuple_size(header{}) + tuple_size(tail{});

    if(bytes_sent + expectedsize > SEND_BUDGET) break;
    uint8_t seq = allocSeq();
    auto sent = sendNext(sending.front().get(), seq, dl);
    bytes_sent += sent;

    if (sent != expectedsize) {updateStatus(); return false;}
    coalesceingAddition(std::move(sending.front()), seq);
    sending.pop();
  }
  updateStatus();
  return true;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
size_t Protocol_Handler<Transport, ReadEnum, WriteEnum>::sendNext(Cmd::Command *operation, uint8_t seq, deadline_t deadline) {
  if (!this->connected() || !operation) return 0;

  auto &front = *operation;
  auto  size  = front.getPckSize();
  constexpr auto headerSize = tuple_size(header{});
  constexpr auto tailSize   = tuple_size(tail{});

  std::array<uint8_t, MAX_PAYLOAD_SIZE + tuple_size(header{}) + tuple_size(tail{})> buffer{};
  const size_t frameSize = size + headerSize + tailSize;

  // header: [0xAA][inst][seq_id][length]
  auto headerSend = make_msg_from_args(header{}, 0xAA, front.getInst(), seq,
                                        static_cast<uint8_t>(size));
  pack_tuple_to_buffer(headerSend, buffer.data());
  front.pack(buffer.data() + headerSize);

  auto tail_tuple = make_msg_from_args(
      tail{}, calcCRC(buffer.data(), headerSize + size, 0));
  pack_tuple_to_buffer(tail_tuple, buffer.data() + headerSize + size);

  auto sent = this->writeData(buffer.data(), frameSize, deadline);
  return sent;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
bool Protocol_Handler<Transport, ReadEnum, WriteEnum>::readPending(micros timeout, micros timePerMsg) {
  if (!this->connected()) return false;

  const deadline_t dl = stdclock::now() + timeout;
  bool dispatched_any = false;

  while (stdclock::now() < dl) {
    if (!already_synced_) {
      // Ask transport if there are bytes available before entering the sync
      // scan. For stream transports (I2C, Serial) this is always true — they
      // block in readData. For ring-based transports (Bridge) this avoids a
      // spurious "Could not sync" warning when the ring is simply empty.
      if (!this->hasData()) break;

      bool found = false;
      uint8_t byte = 0;
      while (stdclock::now() < dl) {
        if (this->readData(&byte, 1, dl) != 1) break;
        if (byte != 0xBB) { lost_bytes_++; }
        total_attmp_++;
        if (byte == 0xAA) { found = true; break; }
      }
      if (!found) {
        ++stat_sync_failures_;
        log.logWarn("Could not sync to message HEADER: lost %u, total %u",
                lost_bytes_, total_attmp_);
        total_attmp_ = 0;
        break;
      }
      total_attmp_   = 0;
      synced_byte_   = byte;
      already_synced_ = true;
    }

    if (stdclock::now() + timePerMsg >= dl) break;

    if (lost_bytes_ > 1) {
      stat_bytes_skipped_ += lost_bytes_ - 1;
      log.logWarn("Skipped %u bytes in sync", lost_bytes_ - 1);
    }
    lost_bytes_    = 0;
    already_synced_ = false;

    auto res = readOneMessage(timePerMsg);
    if (res == ReadResult::OK_DISPATCHED) { dispatched_any = true; continue; }
    if (res == ReadResult::NO_MESSAGE)    { break; }
    if (res == ReadResult::CRC_MISMATCH)  { ++stat_crc_errors_; log.logWarn("CRC Mismatch"); continue; }

    log.logError("I/O error occurred while reading message");
    updateStatus();
    return false;
  }
  updateStatus();
  return dispatched_any;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
typename Protocol_Handler<Transport, ReadEnum, WriteEnum>::ReadResult
Protocol_Handler<Transport, ReadEnum, WriteEnum>::readOneMessage(micros timePerMsg) {
  if (!this->connected()) return ReadResult::IO_ERROR;

  const deadline_t msg_dl = stdclock::now() + timePerMsg;

  header msg_head;
  auto state = ReadHeader(msg_head, msg_dl);
  if (state != ReadResult::OK_DISPATCHED) return state;

  const uint8_t inst   = std::get<1>(msg_head);
  const uint8_t seq_id = std::get<2>(msg_head);
  const uint8_t length = std::get<3>(msg_head);

  // Process ACK for this seq_id before dispatching the payload
  // This goes on dispatch output when we discover the id too
  //processAck(1, inst, seq_id);

  if (length > MAX_PAYLOAD_SIZE) {
    log.logWarn("Payload length %u exceeds MAX_PAYLOAD_SIZE (%zu) — dropping",
            static_cast<unsigned>(length), MAX_PAYLOAD_SIZE);
    return ReadResult::IO_ERROR;
  }

  std::array<uint8_t, MAX_PAYLOAD_SIZE + 1> out_buffer{};
  auto num = this->readData(out_buffer.data(), length + 1, msg_dl);
  if (num != static_cast<size_t>(length + 1)) return ReadResult::IO_ERROR;

  uint8_t recv_crc       = out_buffer[length];
  auto    calculated_crc = getMsgCRC(msg_head, out_buffer.data(), length);
  if (recv_crc != calculated_crc) return ReadResult::CRC_MISMATCH;

  dispatchInput(inst, seq_id, out_buffer.data(), length);
  ++stat_msgs_dispatched_;
  return ReadResult::OK_DISPATCHED;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
typename Protocol_Handler<Transport, ReadEnum, WriteEnum>::ReadResult
Protocol_Handler<Transport, ReadEnum, WriteEnum>::ReadHeader(header &output, deadline_t hdr_dl) {
  // header is now 4 bytes: [0xAA][inst][seq_id][length]
  uint8_t header_buf[sizeof(output)];
  header_buf[0] = synced_byte_;  // 0xAA already validated by readPending

  auto num = this->readData(header_buf + 1, sizeof(header_buf) - 1, hdr_dl);
  if (num != (sizeof(header_buf) - 1)) return ReadResult::IO_ERROR;

  unpack_tuple_from_buffer(output, header_buf);

  // Sentinel: [0xAA][0x00][0x00][CRC] — MCU has nothing to send
  // CRC covers first 3 bytes
  if (std::get<1>(output) == 0x00 && std::get<2>(output) == 0x00 &&
      std::get<3>(output) == calcCRC(header_buf, 3, 0))
    return ReadResult::NO_MESSAGE;

  return ReadResult::OK_DISPATCHED;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
uint8_t Protocol_Handler<Transport, ReadEnum, WriteEnum>::getMsgCRC(const header &msg_head,
                                                uint8_t *pckage, size_t size) {
  uint8_t tmp[sizeof(header)];
  pack_tuple_to_buffer(msg_head, tmp);
  // CRC covers all 4 header bytes + payload
  uint8_t crc = calcCRC(tmp, sizeof(header), 0);
  crc         = calcCRC(pckage, size, crc);
  return crc;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
void Protocol_Handler<Transport, ReadEnum, WriteEnum>::dispatchInput(uint8_t inst, uint8_t seq, uint8_t *pckg,
                                                 size_t size) {
  auto it = instruction_callback.find(inst);
  if (it != instruction_callback.end()) it->second(pckg, seq, size);
}
// =============================================================================
// Sequencing implementations
// =============================================================================

template <typename Transport, typename ReadEnum, typename WriteEnum>
uint8_t Protocol_Handler<Transport, ReadEnum, WriteEnum>::allocSeq() {
  uint8_t seq = next_seq_++;
  // Skip 0x00 — reserved as sentinel indicator
  if (next_seq_ == 0) next_seq_ = 1;
  return seq;
}

template <typename Transport, typename ReadEnum, typename WriteEnum>
bool Protocol_Handler<Transport, ReadEnum, WriteEnum>::coalesceingAddition(
    std::unique_ptr<Cmd::Command> &&cmd, uint8_t new_seq) {

  MsgKey newId = {cmd->getInst(), cmd->getID()};
  auto now = stdclock::now();

  auto it = cmds_lookout.find(newId);
  if (it != cmds_lookout.end()) {

    auto list_it = it->second;

    PendingCommand pc;
    pc.seq_id = new_seq;
    pc.old_seq = std::move(list_it->old_seq);
    pc.old_seq.insert(list_it->seq_id);

    pc.retries_left = list_it->retries_left + max_retries_;
    pc.first_sent   = list_it->first_sent;
    pc.last_sent    = now;
    pc.cmd          = std::move(cmd);

    // reemplazar nodo
    *list_it = std::move(pc);

    // mover al final (LRU behavior)
    waitingCmds_.splice(waitingCmds_.end(), waitingCmds_, list_it);
    ++stat_coalesced_;
    return true;
  }

  PendingCommand pc;
  pc.seq_id       = new_seq;
  pc.old_seq      = {};
  pc.retries_left = max_retries_;
  pc.first_sent   = now;
  pc.last_sent    = now;
  pc.cmd          = std::move(cmd);

  waitingCmds_.push_back(std::move(pc));
  auto list_it = std::prev(waitingCmds_.end());

  cmds_lookout.emplace(newId, list_it);

  return false;
}


template <typename Transport, typename ReadEnum, typename WriteEnum>
void Protocol_Handler<Transport, ReadEnum, WriteEnum>::processAck(
    uint8_t id, uint8_t inst, uint8_t seq_id) {

  MsgKey key = {inst, id};

  auto it = cmds_lookout.find(key);
  if (it == cmds_lookout.end()) {
    return; // no existe
  }

  auto list_it = it->second;
  if (list_it == waitingCmds_.end()) return;

  if (list_it->seq_id == seq_id) {
    waitingCmds_.erase(list_it);
    cmds_lookout.erase(it);
    return;
  }

  auto& old = list_it->old_seq;
  auto old_it = old.find(seq_id);

  if (old_it != old.end()) {
    old.erase(old_it);
    ++stat_partial_acks_;
  }

}