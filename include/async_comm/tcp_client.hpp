#pragma once
#include <async_comm/comm.h>
#include <async_comm/tcp_client_impl.h>
#include <async_comm/message_handler.h>

namespace async_comm
{
template<class MessageHandlerType = DefaultMessageHandler, size_t ReadBufferSize = 1024, size_t WriteBufferSize = ReadBufferSize>
using TCPClient = async_comm::Comm<async_comm::TCPClientImpl<ReadBufferSize, WriteBufferSize>, DefaultMessageHandler>;
} // namespace async_comm
