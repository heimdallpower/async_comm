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
 * @file udp_hello_world.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 *
 * This example opens two UDPImpl objects listening on different ports on the local host, and then uses each to send a
 * simple "hello world" message to the other.
 */

#include <async_comm/udp.hpp>

#include <cstdint>
#include <cstring>
#include <iostream>

#include <chrono>
#include <thread>
#include <vector>


/**
 * @brief Callback function for the async_comm library
 *
 * Prints the received bytes to stdout.
 *
 * @param buf Received bytes buffer
 * @param len Number of bytes received
 */
void callback(const uint8_t* buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    std::cout << buf[i];
  }
}


int main()
{
  const std::string host1{"localhost"};
  const uint16_t port1{14620};
  const std::string host2{"localhost"};
  const uint16_t port2{14625};
  
  async_comm::UDP<1024> udp1(host1, port1, host2, port2);
  // open UDPImpl ports
  udp1.register_receive_callback(&callback);

  async_comm::UDP<1024> udp2(host2, port2, host1, port1);
  udp2.register_receive_callback(&callback);

  if (!udp1.open() || !udp2.open())
  {
    std::cout << "Failed to open UDPImpl ports" << std::endl;
    return 1;
  }

  // send message one direction
  char message1[] = "hello world 1!";
  udp2.send_bytes((uint8_t*) message1, std::strlen(message1));

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << std::endl << std::flush;

  // send message the other direction
  char message2[] = "hello world 2!";
  udp1.send_bytes((uint8_t*) message2, std::strlen(message2));

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << std::endl << std::flush;

  std::cout.flush();

  // close UDPImpl ports
  udp1.close();
  udp2.close();

  return 0;
}
