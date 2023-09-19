#pragma once
#include <async_comm/comm.h>
#include <async_comm/udp_impl.h>
#include <async_comm/message_handler.h>

namespace async_comm
{
template<class MessageHandlerType = DefaultMessageHandler, size_t ReadBufferSize = 1024, size_t WriteBufferSize = ReadBufferSize>
using UDP = async_comm::Comm<async_comm::UDPImpl<ReadBufferSize, WriteBufferSize>, MessageHandlerType>;
} // namespace async_comm
