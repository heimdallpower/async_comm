/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2019 Rein Appeldoorn.
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
 * @file tcp_client.h
 * @author Rein Appeldoorn <reinzor@gmail.com>
 */

#ifndef ASYNC_COMM_TCP_CLIENT_H
#define ASYNC_COMM_TCP_CLIENT_H

#include <string>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <async_comm/error_handler.hpp>
#include <async_comm/comm.h>

namespace async_comm
{

/**
 * @class TCPClientImpl
 * @brief Asynchronous communication class for a TCP client
 */
template<size_t ReadBufferSize = 1024u, size_t WriteBufferSize = ReadBufferSize>
class TCPClientImpl
{
public:
  static constexpr size_t READ_BUFFER_SIZE{ReadBufferSize};
  static constexpr size_t WRITE_BUFFER_SIZE{WriteBufferSize};

  /**
   * @brief Connect to a TCP socket as a client
   * @param host The host where the TCP server is running
   * @param port The port on which the TCP server is listening
   * @param message_handler Custom message handler, or omit for default handler
   */
  TCPClientImpl(boost::asio::io_service& io_service): socket_(io_service) {}

  bool is_open() const { return socket_.is_open(); }
  void close() { socket_.close(); }
  bool open(
    std::string host = DEFAULT_HOST,
    uint16_t port = DEFAULT_PORT
  )
  {
    using boost::asio::ip::tcp;

    tcp::resolver resolver(socket_.get_io_service());

    endpoint_ = *resolver.resolve({tcp::v4(), host, "", boost::asio::ip::resolver_query_base::numeric_service});
    endpoint_.port(port);
    socket_.open(tcp::v4());

    socket_.connect(endpoint_);

    socket_.set_option(tcp::socket::reuse_address(true));
    socket_.set_option(tcp::socket::send_buffer_size(WRITE_BUFFER_SIZE*1024));
    socket_.set_option(tcp::socket::receive_buffer_size(READ_BUFFER_SIZE*1024));
  }
  void do_async_read
  (
    const boost::asio::mutable_buffers_1 &buffer,
    boost::function<void(const boost::system::error_code&, size_t)> handler
  )
  {
    socket_.async_receive(buffer, handler);
  }

  void do_async_write
  (
    const boost::asio::const_buffers_1 &buffer,
    boost::function<void(const boost::system::error_code&, size_t)> handler
  )
  {
    socket_.async_send(buffer, handler);
  }
private:
  static constexpr auto DEFAULT_HOST = "localhost";
  static constexpr uint16_t DEFAULT_PORT = 16140;

  boost::asio::ip::tcp::socket socket_;
  boost::asio::ip::tcp::endpoint endpoint_;
};

} // namespace async_comm

#endif // ASYNC_COMM_TCP_CLIENT_H
