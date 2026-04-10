#pragma once

#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <atomic>

// =============================================================================
// Status
// =============================================================================
struct Value {
    std::string key;
    std::string value;
};

struct Status {
    enum Levels { OK, WARN, ERROR, STALE };

    std::string hardware_id;
    std::string message;
    std::string name;
    std::unordered_map<std::string, std::string> values;
    Levels level { OK };
};

class DiagnosticRegistry;

// =============================================================================
// Tracked<T>
// =============================================================================
template <typename T>
class Tracked {
public:
    void set_registry(DiagnosticRegistry* r) { registry_ = r; }

    template <typename F>
    auto with(F &&fn) {
        std::lock_guard<std::mutex> lock{mtx_};

        bool was_updated = updated_.load(std::memory_order_relaxed);
        updated_.store(true, std::memory_order_relaxed);

        if (!was_updated && registry_) {
            notify_registry();  
        }

        return fn(data_);
    }

    T consume() {
        std::lock_guard<std::mutex> lock{mtx_};
        updated_.store(false, std::memory_order_relaxed);
        return data_;
    }

    bool isUpdated() const {
        return updated_.load(std::memory_order_relaxed);
    }

private:
    void notify_registry();

    T data_;
    mutable std::mutex mtx_;
    std::atomic<bool> updated_{ false };
    DiagnosticRegistry* registry_{ nullptr };
};

// =============================================================================
// DiagnosticRegistry
//
// Colector de Tracked<Status>. Guarda punteros crudos — las fuentes deben
// outlive al registry (el nodo outlives sus transportes en uso normal).
//
// =============================================================================
class DiagnosticRegistry {
public:
    void register_source(Tracked<Status>* src) {
        if (!src) return;

        auto [it, inserted] = sources_.insert(src);
        if (!inserted) return;

        src->set_registry(this);
    }

    void notify_dirty(Tracked<Status>* src) {
        std::lock_guard<std::mutex> lock(mtx_);

        if (in_dirty_.insert(src).second) {
            dirty_.push_back(src);
        }
    }

    std::vector<Status> consume_updated() {
        std::vector<Status> out;

        std::lock_guard<std::mutex> lock(mtx_);

        while (!dirty_.empty()) {
            auto* s = dirty_.front();
            dirty_.pop_front();
            in_dirty_.erase(s);

            if (s->isUpdated()) {
                out.push_back(s->consume());
            }
        }

        return out;
    }

    std::vector<Status> snapshot_all() {
        std::vector<Status> out; 

        std::lock_guard<std::mutex> lock(mtx_);

        dirty_.clear();
        in_dirty_.clear();

        out.reserve(sources_.size()); 
        for (auto *s : sources_) {
            out.push_back(s->consume());
        }
        return out; 
    
    }

    size_t amountUpdated() const { return dirty_.size(); }

    size_t size() const { return sources_.size(); }

private:
    std::unordered_set<Tracked<Status>*> sources_;

    std::deque<Tracked<Status>*> dirty_;
    std::unordered_set<Tracked<Status>*> in_dirty_;

    mutable std::mutex mtx_;
};

template <typename T>
void Tracked<T>::notify_registry() {
    registry_->notify_dirty(this);
}