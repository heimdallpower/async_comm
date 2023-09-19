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
 * @file udp.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_UDP_H
#define ASYNC_COMM_UDP_H

#include <string>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <async_comm/comm.h>

namespace async_comm
{

/**
 * @class UDPImpl
 * @brief Asynchronous communication class for a UDPImpl socket
 */
template<size_t ReadBufferSize = 1024u, size_t WriteBufferSize = ReadBufferSize>
class UDPImpl
{
public:
  static constexpr size_t READ_BUFFER_SIZE{ReadBufferSize};
  static constexpr size_t WRITE_BUFFER_SIZE{WriteBufferSize};

  /**
   * @brief Bind a UDPImpl socket
   * @param bind_host The bind host where this application is listening (usually "localhost")
   * @param bind_port The bind port where this application is listening
   * @param remote_host The remote host to communicate with
   * @param remote_port The port on the remote host
   * @param message_handler Custom message handler, or omit for default handler
   */
  UDPImpl(boost::asio::io_service& io_service):
  socket_(io_service)
  {}

  bool is_open() const { return socket_.is_open(); }
  void close() { socket_.close(); }
  void open(
    std::string bind_host = DEFAULT_BIND_HOST,
    uint16_t bind_port = DEFAULT_BIND_PORT,
    std::string remote_host = DEFAULT_REMOTE_HOST,
    uint16_t remote_port = DEFAULT_REMOTE_PORT
  )
  {
    using boost::asio::ip::udp;

    udp::resolver resolver(socket_.get_io_service());

    bind_endpoint_ = *resolver.resolve({udp::v4(), bind_host, "",
                                        boost::asio::ip::resolver_query_base::numeric_service});
    bind_endpoint_.port(bind_port);

    remote_endpoint_ = *resolver.resolve({udp::v4(), remote_host, "",
                                          boost::asio::ip::resolver_query_base::numeric_service});
    remote_endpoint_.port(remote_port);

    socket_.open(udp::v4());
    socket_.bind(bind_endpoint_);

    socket_.set_option(udp::socket::reuse_address(true));
    socket_.set_option(udp::socket::send_buffer_size(WRITE_BUFFER_SIZE*1024));
    socket_.set_option(udp::socket::receive_buffer_size(READ_BUFFER_SIZE*1024));
  }

  void do_async_read
  (
    const boost::asio::mutable_buffers_1 &buffer,
    boost::function<void(const boost::system::error_code&, size_t)> handler
  )
  {
    socket_.async_receive_from(buffer, remote_endpoint_, handler);
  }
  void do_async_write(
    const boost::asio::const_buffers_1 &buffer,
    boost::function<void(const boost::system::error_code&, size_t)> handler
  )
  {
    socket_.async_send_to(buffer, remote_endpoint_, handler);
  }
private:
  static constexpr auto DEFAULT_BIND_HOST = "localhost";
  static constexpr uint16_t DEFAULT_BIND_PORT = 16140;
  static constexpr auto DEFAULT_REMOTE_HOST = "localhost";
  static constexpr uint16_t DEFAULT_REMOTE_PORT = 16145;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

} // namespace async_comm

#endif // ASYNC_COMM_UDP_H
