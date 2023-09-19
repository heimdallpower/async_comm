#pragma once
#include <async_comm/comm.h>
#include <async_comm/serial_impl.h>

namespace async_comm
{
template<size_t ReadBufferSize = 1024, size_t WriteBufferSize = ReadBufferSize>
using Serial = async_comm::Comm<async_comm::SerialImpl<ReadBufferSize, WriteBufferSize>>;
} // namespace async_comm
