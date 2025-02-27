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
 * @file error_handler.hpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_MESSAGE_HANDLER_H
#define ASYNC_COMM_MESSAGE_HANDLER_H

#include <iostream>
#include <string>
#include <boost/system/error_code.hpp>

namespace async_comm
{

/**
 * @class ErrorHandler
 * @brief Abstract base class for message handler
 *
 * The implementations of this class define how messages are displayed, logged,
 * etc. To create custom behavior, derive from this base class and override the
 * pure virtual functions.
 */
class ErrorHandler
{
public:
  virtual void on_open(const boost::system::error_code& code) = 0;
  virtual void during_operation(const boost::system::error_code& code) = 0;
};

/**
 * @class DefaultErrorHandler
 * @brief Default message handler that outputs to stdout and stderr
 */
class DefaultErrorHandler : public ErrorHandler
{
public:
  inline void on_open(const boost::system::error_code &code) override { std::cerr << "[async_comm][on open ERROR]: " << code.message() << std::endl; }
  inline void during_operation(const boost::system::error_code& code) override { std::cerr << "[async_comm][during operation ERROR]: " << code.message() << std::endl; }
};

} // namespace async_comm

#endif // ASYNC_COMM_MESSAGE_HANDLER_H
