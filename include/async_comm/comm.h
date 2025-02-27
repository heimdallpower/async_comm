/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2018 Daniel Koch.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file comm.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_COMM_H
#define ASYNC_COMM_COMM_H

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/none.hpp>

#include <async_comm/error_handler.hpp>

namespace async_comm
{

/**
 * @class CommListener
 * @brief Abstract base class for getting comm events via a listener interface
 */
class CommListener
{
public:
  /**
   * @brief Callback for data received
   *
   * @warning The data buffer passed to the callback function will be invalid after the callback function exits. If you
   * want to store the data for later processing, you must copy the data to a new buffer rather than storing the
   * pointer to the buffer.
   *
   * @param buf Address of buffer containing received bytes
   * @param size Number of bytes in the receive buffer
   */
  virtual void receive_callback(const uint8_t * buf, size_t size) = 0;
};

/**
 * @class Comm
 * @brief Abstract base class for an asynchronous communication port
 */
template<typename Impl, typename ErrorHandlerType = DefaultErrorHandler>
class Comm
{
public:
  /**
   * @brief Set up asynchronous communication base class
   * @param message_handler Custom message handler, or omit for default handler
   */
  
  Comm():
  work_{std::make_unique<boost::asio::io_service::work>(io_service_)},
  impl_{io_service_},
  io_thread_{std::thread(boost::bind(&boost::asio::io_service::run, &io_service_))},
  callback_thread_{std::thread(std::bind(&Comm::process_callbacks, this))}
  {}

  ~Comm()
  {
    // send shutdown signal to callback thread
    {
      std::unique_lock<std::mutex> lock(callback_mutex_);
      shutdown_requested_ = true;
    }
    condition_variable_.notify_one();

    if (impl_.is_open())
      impl_.close();
    work_.reset();
    if (io_thread_.joinable())
      io_thread_.join();

    if (callback_thread_.joinable())
      callback_thread_.join();
  }

  bool is_open() { return impl_.is_open(); };

  /**
   * @brief Initializes and opens the port
   * @return True if the port was succesfully initialized
   */
  template<class... Args>
  bool open(Args&&... args)
  {
    try
    {
      impl_.open(std::forward<Args>(args)...);
    }
    catch (boost::system::system_error e)
    {
      message_handler_.on_open(e.code());
      return false;
    }
    async_read();
    return true;
  }

  /**
   * @brief Closes the port
   */
  void close()
  {
    impl_.close();
  };

  /**
   * @brief Send bytes from a buffer over the port
   * @param src Address of the buffer
   * @param len Number of bytes to send
   */
  bool send_bytes(const uint8_t * src, size_t len)
  {
    if (!is_open())
      return false;
  
    std::lock_guard<std::recursive_mutex> lock(write_mutex_);

    for (size_t pos = 0; pos < len; pos += Impl::WRITE_BUFFER_SIZE)
    {
      const size_t num_bytes{(len - pos) > Impl::WRITE_BUFFER_SIZE ? Impl::WRITE_BUFFER_SIZE : (len - pos)};
      write_queue_.emplace_back(src + pos, num_bytes);
    }

    async_write(true);
    return true;
  }

  /**
   * @brief Send bytes from a buffer over the port
   * @param bytes the buffer
   */
  template<size_t Len>
  bool send_bytes(const std::array<uint8_t, Len>& bytes)
  {
    if (!is_open())
      return false;
    
    std::lock_guard<std::recursive_mutex> lock(write_mutex_);
    for (size_t pos = 0; pos < Len; pos += Impl::WRITE_BUFFER_SIZE)
    {
      const size_t num_bytes{(Len - pos) > Impl::WRITE_BUFFER_SIZE ? Impl::WRITE_BUFFER_SIZE : (Len - pos)};
      write_queue_.emplace_back(bytes.data() + pos, num_bytes);
    }
    async_write(true);
    return true;
  }

  /**
   * @brief Send a single byte over the port
   * @param data Byte to send
   */
  inline void send_byte(uint8_t data) { send_bytes(&data, 1); }

  /**
   * @brief Register a callback function for when bytes are received on the port
   *
   * The callback function needs to accept two parameters. The first is of type `const uint8_t*`, and is a constant
   * pointer to the data buffer. The second is of type `size_t`, and specifies the number of bytes available in the
   * buffer.
   *
   * @warning The data buffer passed to the callback function will be invalid after the callback function exits. If you
   * want to store the data for later processing, you must copy the data to a new buffer rather than storing the
   * pointer to the buffer.
   *
   * @param fun Function to call when bytes are received
   */
  void register_receive_callback(std::function<void(const uint8_t*, size_t)> fun)
  {
    receive_callback_ = fun;
  }

  /**
   * @brief Register a listener for when bytes are received on the port
   *
   * The listener must inherit from CommListener and implement the `receive_callback` function.  This is another
   * mechanism for receiving data from the Comm interface without needing to create function pointers. Multiple
   * listeners can be added and all will get the callback
   *
   * @param listener Reference to listener
   */
  void register_listener(CommListener &listener)
  {
    listeners_.push_back(listener);
  }
private:

  void async_read()
  {
    if (!is_open())
      return;

    impl_.do_async_read(
      boost::asio::buffer(read_buffer_, Impl::READ_BUFFER_SIZE),
      boost::bind(
        &Comm::async_read_end,
        this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred
      )
    );
  }

  struct ReadBuffer
  {
    uint8_t data[Impl::READ_BUFFER_SIZE];
    size_t len;

    ReadBuffer(const uint8_t * buf, size_t len) : len(len)
    {
      assert(len <= Impl::READ_BUFFER_SIZE); // only checks in debug mode
      memcpy(data, buf, len);
    }
  };

  struct WriteBuffer
  {
    uint8_t data[Impl::WRITE_BUFFER_SIZE];
    size_t len;
    size_t pos;

    WriteBuffer() : len{0u}, pos{0u} {}

    WriteBuffer(const uint8_t * buf, size_t len) : len{len}, pos{0u}
    {
      assert(len <= Impl::WRITE_BUFFER_SIZE); // only checks in debug mode
      memcpy(data, buf, len);
    }

    const uint8_t * dpos() const { return data + pos; }

    size_t nbytes() const { return len - pos; }
  };

  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      if (error.value() == boost::system::errc::operation_canceled)
        return;
      
      close();
      message_handler_.during_operation(error);
      return;
    }

    {
      std::unique_lock<std::mutex> lock(callback_mutex_);
      read_queue_.emplace_back(read_buffer_, bytes_transferred);
      new_data_ = true;
    }
    condition_variable_.notify_one();

    async_read();
  }

  void async_write(bool check_write_state)
  {
    if (check_write_state && write_in_progress_)
      return;

    std::lock_guard<std::recursive_mutex> lock(write_mutex_);
    if (write_queue_.empty())
      return;

    write_in_progress_ = true;
    WriteBuffer& buffer = write_queue_.front();
    impl_.do_async_write(
      boost::asio::buffer(buffer.dpos(), buffer.nbytes()),
      boost::bind(&Comm::async_write_end,
        this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred
      )
    );
  }

  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      if (error.value() == boost::system::errc::operation_canceled)
        return;
      
      close();
      message_handler_.during_operation(error);
      return;
    }

    std::lock_guard<std::recursive_mutex> lock(write_mutex_);
    if (write_queue_.empty())
    {
      write_in_progress_ = false;
      return;
    }

    WriteBuffer& buffer = write_queue_.front();
    buffer.pos += bytes_transferred;
    if (buffer.nbytes() == 0)
    {
      write_queue_.pop_front();
    }

    if (write_queue_.empty())
      write_in_progress_ = false;
    else
      async_write(false);
  }

  void process_callbacks()
  {
    std::list<ReadBuffer> local_queue;

    while (true)
    {
      // wait for either new data or a shutdown request
      std::unique_lock<std::mutex> lock(callback_mutex_);
      condition_variable_.wait(lock, [&]{ return new_data_ || shutdown_requested_; });

      // if shutdown requested, end thread execution
      if (shutdown_requested_)
      {
        break;
      }

      // move data to local buffer
      local_queue.splice(local_queue.end(), read_queue_);

      // release mutex to allow continued asynchronous read operations
      new_data_ = false;
      lock.unlock();

      // execute callbacks for all new data
      while (!local_queue.empty())
      {
        ReadBuffer buffer = local_queue.front();
        if (receive_callback_)
          receive_callback_(buffer.data, buffer.len);
        for (std::reference_wrapper<CommListener> listener_ref : listeners_)
          listener_ref.get().receive_callback(buffer.data, buffer.len);

        local_queue.pop_front();
      }
    }
  }

  bool new_data_{false};
  bool shutdown_requested_{false};
  bool write_in_progress_{false};

  ErrorHandlerType message_handler_{};
  boost::asio::io_service io_service_{};

  uint8_t read_buffer_[Impl::READ_BUFFER_SIZE]{0u};
  std::list<ReadBuffer> read_queue_{};
  std::mutex callback_mutex_{};
  std::condition_variable condition_variable_{};

  std::list<WriteBuffer> write_queue_{};
  std::recursive_mutex write_mutex_{};

  std::function<void(const uint8_t *, size_t)> receive_callback_{nullptr};
  std::vector<std::reference_wrapper<CommListener>> listeners_{};

  std::unique_ptr<boost::asio::io_service::work> work_;
  Impl impl_;
  std::thread io_thread_;
  std::thread callback_thread_;
};

} // namespace async_comm

#endif // ASYNC_COMM_COMM_H
