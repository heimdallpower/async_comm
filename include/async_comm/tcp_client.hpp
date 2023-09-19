#pragma once
#include <async_comm/comm.h>
#include <async_comm/tcp_client_impl.h>
#include <async_comm/error_handler.hpp>

namespace async_comm
{
template<class ErrorHandlerType = DefaultErrorHandler, size_t ReadBufferSize = 1024, size_t WriteBufferSize = ReadBufferSize>
using TCPClient = async_comm::Comm<async_comm::TCPClientImpl<ReadBufferSize, WriteBufferSize>, ErrorHandlerType>;
} // namespace async_comm
