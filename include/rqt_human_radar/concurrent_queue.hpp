// Copyright 2024 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// taken from https://gist.github.com/CaglayanDokme/2fd4278969cf9683c5e5ad0ca5534020
#pragma once

#include <queue>
#include <mutex>
#include <utility>
#include <condition_variable>

template<class ElementType>
class concurrent_queue
{
public:
  /** Construction **/
  concurrent_queue() = default;

  // Forbid copying and moving
  concurrent_queue(const concurrent_queue &) = delete;
  concurrent_queue(concurrent_queue &&) = delete;
  concurrent_queue operator=(const concurrent_queue &) = delete;
  concurrent_queue & operator=(concurrent_queue &&) = delete;

public:
  /** Methods **/
  void push(const ElementType & element)
  {
    {
      std::lock_guard lock(m_lock);
      m_messages.push(element);
    }

    m_notifier.notify_one();
  }

  void push(ElementType && element)
  {
    {
      std::lock_guard lock(m_lock);
      m_messages.push(std::move(element));
    }

    m_notifier.notify_one();
  }

  template<class ... Args>
  void emplace(Args &&... args)
  {
    {
      std::lock_guard lock(m_lock);
      m_messages.emplace(std::forward<Args>(args)...);
    }

    m_notifier.notify_one();
  }

  [[nodiscard]] ElementType & front()
  {
    std::unique_lock lock(m_lock);

    if (m_messages.empty()) {
      m_notifier.wait(lock, [&] {return !m_messages.empty();});
    }

    return m_messages.front();
  }

  void pop()
  {
    if (!m_messages.empty()) {
      std::lock_guard lock(m_lock);
      m_messages.pop();
    }
  }

  [[nodiscard]] bool empty() const
  {
    return m_messages.empty();
  }

  [[nodiscard]] size_t size() const
  {
    return m_messages.size();
  }

private:
  /** Members **/
  std::queue<ElementType> m_messages;
  std::condition_variable m_notifier;
  std::mutex m_lock;
};
