#pragma once

#include <cassert>
#include <list>
#include <mutex>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace cache {

// Forward declaration of the user-facing Cache class.
// Capacity defaults to 1, ThreadSafe defaults to true, Index defaults to 0.
// Index can be used to differentiate instances.
template <typename T, std::size_t Capacity = 1, bool ThreadSafe = true, std::size_t Index = 0>
class Cache;

namespace internal {

// Base template declaration (not defined).
template <typename T, std::size_t Capacity, bool ThreadSafe, std::size_t Index>
class CacheImpl;

// Specialization 1: Capacity == 1 && ThreadSafe == false
// A single-value cache without locking, no key required.
template <typename T, std::size_t Index>
class CacheImpl<T, 1, false, Index> {
 public:
  CacheImpl() = default;

  CacheImpl(const CacheImpl&) = delete;
  CacheImpl& operator=(const CacheImpl&) = delete;

  // Sets the value
  void Set(const T& value) {
    value_ = value;
  }

  // Sets the value using move semantics
  void Set(T&& value) {
    value_ = std::move(value);
  }

  // Retrieves the value
  T Get() const {
    return value_;
  }

  // Clears the stored value.
  void Clear() {
    value_ = T{};
  }

 private:
  T value_;
};

// Specialization 2: Capacity == 1 && ThreadSafe == true
// A single-value cache with locking, no key required.
template <typename T, std::size_t Index>
class CacheImpl<T, 1, true, Index> {
 public:
  CacheImpl() = default;

  CacheImpl(const CacheImpl&) = delete;
  CacheImpl& operator=(const CacheImpl&) = delete;

  // Sets the value with thread safety
  void Set(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
  }

  // Sets the value using move semantics with thread safety
  void Set(T&& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
  }

  // Retrieves the value with thread safety
  T Get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Clears the stored value with thread safety.
  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = T{};
  }

 private:
  T value_;
  mutable std::mutex mutex_;
};

// Specialization 3: Capacity > 1 && ThreadSafe == false
// An LRU cache without locking. Uses std::size_t as key.
template <typename T, std::size_t Capacity, std::size_t Index>
class CacheImpl<T, Capacity, false, Index> {
  static_assert(Capacity > 1, "This specialization is for Capacity > 1 only.");

 public:
  CacheImpl() {
    map_.reserve(Capacity);
  }

  CacheImpl(const CacheImpl&) = delete;
  CacheImpl& operator=(const CacheImpl&) = delete;

  // Sets the value for a key
  void Set(std::size_t key, const T& value) {
    auto it = map_.find(key);
    if (it != map_.end()) {
      it->second->second = value;
      storage_.splice(storage_.begin(), storage_, it->second);
      return;
    }

    if (storage_.size() == Capacity) {
      auto last = std::prev(storage_.end());
      map_.erase(last->first);
      storage_.pop_back();
    }

    storage_.emplace_front(key, value);
    map_[key] = storage_.begin();
  }

  // Sets the value for a key using move semantics
  void Set(std::size_t key, T&& value) {
    auto it = map_.find(key);
    if (it != map_.end()) {
      it->second->second = std::move(value);
      storage_.splice(storage_.begin(), storage_, it->second);
      return;
    }

    if (storage_.size() == Capacity) {
      auto last = std::prev(storage_.end());
      map_.erase(last->first);
      storage_.pop_back();
    }

    storage_.emplace_front(key, std::move(value));
    map_[key] = storage_.begin();
  }

  // Retrieves the value for a key. Throws std::out_of_range if not found.
  T Get(std::size_t key) const {
    auto it = map_.find(key);
    if (it == map_.end()) {
      throw std::out_of_range("Key not found in cache.");
    }
    storage_.splice(storage_.begin(), storage_, it->second);
    return it->second->second;
  }

  // Clears all stored values.
  void Clear() {
    storage_.clear();
    map_.clear();
  }

 private:
  mutable std::list<std::pair<std::size_t, T>> storage_;
  mutable std::unordered_map<std::size_t, typename std::list<std::pair<std::size_t, T>>::iterator> map_;
};

// Specialization 4: Capacity > 1 && ThreadSafe == true
// An LRU cache with locking. Uses std::size_t as key.
template <typename T, std::size_t Capacity, std::size_t Index>
class CacheImpl<T, Capacity, true, Index> {
  static_assert(Capacity > 1, "This specialization is for Capacity > 1 only.");

 public:
  CacheImpl() {
    map_.reserve(Capacity);
  }

  CacheImpl(const CacheImpl&) = delete;
  CacheImpl& operator=(const CacheImpl&) = delete;

  // Sets the value for a key with thread safety
  void Set(std::size_t key, const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_.find(key);
    if (it != map_.end()) {
      it->second->second = value;
      storage_.splice(storage_.begin(), storage_, it->second);
      return;
    }

    if (storage_.size() == Capacity) {
      auto last = std::prev(storage_.end());
      map_.erase(last->first);
      storage_.pop_back();
    }

    storage_.emplace_front(key, value);
    map_[key] = storage_.begin();
  }

  // Sets the value for a key using move semantics with thread safety
  void Set(std::size_t key, T&& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_.find(key);
    if (it != map_.end()) {
      it->second->second = std::move(value);
      storage_.splice(storage_.begin(), storage_, it->second);
      return;
    }

    if (storage_.size() == Capacity) {
      auto last = std::prev(storage_.end());
      map_.erase(last->first);
      storage_.pop_back();
    }

    storage_.emplace_front(key, std::move(value));
    map_[key] = storage_.begin();
  }

  // Retrieves the value for a key with thread safety. Throws std::out_of_range if not found.
  T Get(std::size_t key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_.find(key);
    if (it == map_.end()) {
      throw std::out_of_range("Key not found in cache.");
    }
    storage_.splice(storage_.begin(), storage_, it->second);
    return it->second->second;
  }

  // Clears all stored values with thread safety.
  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    storage_.clear();
    map_.clear();
  }

 private:
  mutable std::list<std::pair<std::size_t, T>> storage_;
  mutable std::unordered_map<std::size_t, typename std::list<std::pair<std::size_t, T>>::iterator> map_;
  mutable std::mutex mutex_;
};

}  // namespace internal

// User-facing Cache class.
// Provides a global singleton instance via Instance().
// Key type is fixed to std::size_t.
// Capacity defaults to 1, ThreadSafe defaults to true, Index defaults to 0.
// Index can be used to differentiate multiple singleton instances for the same T/Capacity/ThreadSafe.
template <typename T, std::size_t Capacity, bool ThreadSafe, std::size_t Index>
class Cache : public internal::CacheImpl<T, Capacity, ThreadSafe, Index> {
  static_assert(Capacity >= 1, "Capacity must be at least 1.");

 public:
  Cache(const Cache&) = delete;
  Cache& operator=(const Cache&) = delete;

  // Returns a global singleton instance of the cache.
  static Cache& Instance() {
    static Cache instance;
    return instance;
  }

 protected:
  Cache() = default;
};

}  // namespace cache
