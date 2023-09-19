#pragma once
#include <async_comm/comm.h>
#include <async_comm/udp_impl.h>

namespace async_comm
{
template<size_t ReadBufferSize = 1024, size_t WriteBufferSize = ReadBufferSize>
using UDP = async_comm::Comm<async_comm::UDPImpl<ReadBufferSize, WriteBufferSize>>;
} // namespace async_comm
