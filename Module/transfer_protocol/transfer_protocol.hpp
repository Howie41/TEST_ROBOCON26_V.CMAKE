#ifndef MODULES_TRANSFER_PROTOCOL_HPP
#define MODULES_TRANSFER_PROTOCOL_HPP

//先保存原来的pid里面的宏定义，再取消定义，
//避免污染c++的标志库，等完后再恢复原来的定义
#pragma push_macro("abs")
#undef abs

#include "function.hpp"                      //提供函数包装器
#include "verification_algorithm.hpp"         //提供校验算法

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <memory_resource>
#include <type_traits>
#include <vector>

//数据的传输协议，定义了数据包的格式和校验算法的接口，
//header        2byte               包头     0xAA    0X55
//data_length   2byte               数据长度
//code          2byte               命令码
//verify        2byte               校验码
//data          Nbyte               数据体
//tail          2byte               包尾     0x55    0XAA
namespace gdut {

  //标签结构体类型，用于区分不同的构造函数
  //build_packet_t:我给你bady数据，你帮我构造一个完整的包
  //from_whole_packet_t:我给你一个完整的包，你帮我解析出数据来
struct build_packet_t {};
inline constexpr build_packet_t build_packet;

struct from_whole_packet_t {};
inline constexpr from_whole_packet_t from_whole_packet;

//date_packet类表示一个数据包，包含头部、数据体和尾部，并提供了构造、解析、验证等功能
template <typename VerifyAlgorithm> class data_packet {
  // 静态断言，保证VerifyAlgorithm是verify_algorithm的子类
  static_assert(
      std::is_base_of_v<verify_algorithm<VerifyAlgorithm>, VerifyAlgorithm>,
      "VerifyAlgorithm must be derived class of "
      "verify_algorithm<VerifyAlgorithm>");

public:
  using verify_algorithm_t = VerifyAlgorithm;
  static constexpr std::uint16_t header = 0xAA << 8 | 0x55;
  static constexpr std::uint16_t tail = 0x55 << 8 | 0xAA;
  static constexpr std::size_t header_size =
      sizeof(uint16_t) + sizeof(uint16_t) * 3; // header + size + code + crc
  static constexpr std::size_t tail_size = sizeof(uint16_t);

  //自定义内存资源的接口，默认使用全局的内存资源
  static std::pmr::memory_resource *default_memory_resource() noexcept {
    return std::pmr::get_default_resource();
  }

  //创建一个空包
  data_packet(std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {}

      //构造发送包
  template <std::input_iterator It>
  //code:命令码/功能码，begin和end是数据体的范围，
  // build_packet_t表示这是构造包的构造函数，mr是内存资源
  data_packet(uint16_t code, It begin, It end, build_packet_t,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {
        //静态断断言，保证迭代器的元素大小为1字节，因为校验算法通常是基于字节进行计算的
        //只能用单字节的迭代器来构造包：uint8_t*、const char*、std::vector<uint8_t>::iterator
        // std::array<char, N>::iterator
        //float*、double*等类型的迭代器不行，因为它们的元素大小不是1字节
        //如果需要构造包含非字节数据的包，可以先把数据序列化成字节数组，再用这个构造函数来构造包
    static_assert(sizeof(std::iter_value_t<It>) == 1,
                  "The data size of the iterator must be 1");
                  //包的总大小不能超过65535字节，因为size字段是16位的
    if (std::distance(begin, end) >
        static_cast<decltype(std::distance(begin, end))>(
            std::numeric_limits<uint16_t>::max() - sizeof(uint16_t) * 3)) {
      return; // Body size is too large to fit in a packet
    }
    // head (2 bytes) + size (2 bytes) + code (2 bytes) + crc (2 bytes) + body +
    // tail (2 bytes)
    //计算包和填充包头等
    const uint16_t total_size =
        static_cast<uint16_t>(header_size) +
        static_cast<uint16_t>(std::distance(begin, end)) +
        static_cast<uint16_t>(tail_size);
    m_data.resize(total_size);
    m_data[0] = (header >> 8) & 0xFF;
    m_data[1] = header & 0xFF;
    m_data[2] = (total_size >> 8) & 0xFF;
    m_data[3] = total_size & 0xFF;
    m_data[4] = (code >> 8) & 0xFF;
    m_data[5] = code & 0xFF;
    //有长度能copy
    std::copy(begin, end, m_data.begin() + header_size);
    m_data[total_size - 2] = (tail >> 8) & 0xFF;
    m_data[total_size - 1] = tail & 0xFF;
    //计算校验码并写入包中
    update_verification();
  }

  //构造解析包
  template <std::random_access_iterator It>
  data_packet(It begin, It end, from_whole_packet_t,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(mr) {
        //长度至少要2个字节，因为要有包头
    if (std::distance(begin, end) < 2) {
      return; // Not enough data for packet header
    }
    //检查包头是不是 0xAA55
    if (*begin != ((header >> 8) & 0xFF) || *(begin + 1) != (header & 0xFF)) {
      return; // Invalid packet header
    }
    //检查至少有 header 和 tail
    if (static_cast<std::size_t>(std::distance(begin, end)) <
        header_size + tail_size) {
      return; // Not enough data for header and tail, wait for more data
    }
    //最小包长度是10
    uint16_t size = *(begin + 2) << 8 | *(begin + 3);
    if (size < header_size + tail_size) {
      return; // Invalid packet size
    }
    //如果当前数据不足一个完整包，就返回等待更多数据：
    if (static_cast<std::size_t>(std::distance(begin, end)) < size) {
      return;
    }
    //检查包尾是不是 0x55AA
    if (*(begin + size - 2) != ((tail >> 8) & 0xFF) ||
        *(begin + size - 1) != (tail & 0xFF)) {
      return; // Invalid packet tail
    }
    //复制完整数据包
    m_data.resize(size);
    std::copy(begin, begin + size, m_data.begin());
    //校验失败则清空
    if (!verify_verification()) {
      m_data.clear();
      return;
    }
  }

  ~data_packet() = default;

  //构造函数：可以把一个包复制到另一个内存资源管理的vector里
  data_packet(const data_packet &packet,
              std::pmr::memory_resource *mr = default_memory_resource())
      : m_data(packet.m_data, mr) {}
  data_packet(data_packet &&packet) noexcept
      : m_data(std::move(packet.m_data)) {}

  data_packet &operator=(const data_packet &packet) {
    if (this != std::addressof(packet)) {
      m_data = packet.m_data;
    }
    return *this;
  }

  data_packet &operator=(data_packet &&packet) noexcept {
    if (this != std::addressof(packet)) {
      m_data = std::move(packet.m_data);
    }
    return *this;
  }

  //读取总长度
  [[nodiscard]] uint16_t size() const noexcept {
    if (m_data.size() < 4) {
      return 0;
    }
    return m_data[2] << 8 | m_data[3];
  }

  //读取数据包编号，功能编码
  [[nodiscard]] uint16_t code() const noexcept {
    if (m_data.size() < 6) {
      return 0;
    }
    return m_data[4] << 8 | m_data[5];
  }

  //读取数据包校验码
  [[nodiscard]] uint16_t crc() const noexcept {
    if (m_data.size() < 8) {
      return 0;
    }
    return m_data[6] << 8 | m_data[7];
  }

  //返回数据包数据指针
  [[nodiscard]] const uint8_t *data() const noexcept { return m_data.data(); }

  [[nodiscard]] const uint8_t *begin() const noexcept { return m_data.data(); }

  [[nodiscard]] const uint8_t *end() const noexcept {
    return m_data.data() + m_data.size();
  }

  //body相关接口
  //计算body长度
  [[nodiscard]] uint16_t body_size() const noexcept {
    const uint16_t packet_size = size();
    if (packet_size < header_size + tail_size) {
      return 0;
    }
    return packet_size - header_size - tail_size;
  }

  //返回整个包头的位置
  [[nodiscard]] const uint8_t *body_data() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + header_size;
  }

  //返回整个包体的结尾位置
  [[nodiscard]] const uint8_t *body_begin() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + header_size;
  }

  [[nodiscard]] const uint8_t *body_end() const noexcept {
    if (m_data.size() < header_size) {
      return nullptr;
    }
    return m_data.data() + size();
  }

  //更新校验和
  void update_verification() noexcept {
    if (m_data.size() < header_size + tail_size) {
      return;
    }
    verify_algorithm_t va;
    va.calculate(m_data.begin(), m_data.end(),
                 m_data.begin() + header_size - sizeof(uint16_t));
  }

  //验证校验
  [[nodiscard]] bool verify_verification() const noexcept {
    if (m_data.size() < header_size + tail_size) {
      return false;
    }
    verify_algorithm_t va;
    return va.verify(m_data.begin(), m_data.end(),
                     m_data.begin() + header_size - sizeof(uint16_t));
  }

  explicit operator bool() const noexcept {
    return m_data.size() >= (header_size + tail_size);
  }

private:
  /*
  struct {
    uint8_t header;
    uint16_t size;
    uint16_t code;
    uint16_t verify;
    uint8_t  payload[];
  } header;
  */
  std::pmr::vector<std::uint8_t> m_data;
};

//收发管理器
template <typename VerifyAlgorithm> class packet_manager {
public:
  using packet_t = data_packet<VerifyAlgorithm>;

  packet_manager()
      : m_receive_buffer(std::pmr::get_default_resource()) {}
  ~packet_manager() = default;

  //设置发送字节的函数
  void set_send_function(
      gdut::function<void(const std::uint8_t *, const std::uint8_t *)> func) {
    m_send_function = std::move(func);
  }

  //设置接收回调的函数
  void set_receive_function(gdut::function<void(packet_t)> func) {
    m_receive_function = std::move(func);
  }

//发送数据
  void send(const packet_t &packet) {
    if (m_send_function && packet) {
      m_send_function(packet.data(), packet.data() + packet.size());
    }
  }

  //接收数据流
  template <std::input_iterator It> void   receive(It begin, It end) {
    //把新的数据添加到接收缓冲区中
    //m_receive_buffer.append_range(receive_range<It>{begin, end});
    m_receive_buffer.insert(m_receive_buffer.end(), begin, end);
    //尝试从缓冲区中取出一个完整的数据包  
    while (true) {
      std::pmr::vector<std::uint8_t>::iterator packet_start;
      while (true) {
        //寻找包头
        packet_start =
            std::find(m_receive_buffer.begin(), m_receive_buffer.end(),
                      (packet_t::header >> 8) & 0xFF);
        //尾部为包头，保存
        if (packet_start == m_receive_buffer.end()) {  
          // No header start found. Keep one trailing 0xAA to handle split
          // header across receive chunks: [..., 0xAA] + [0x55, ...].
          if (!m_receive_buffer.empty() &&
              m_receive_buffer.back() == ((packet_t::header >> 8) & 0xFF)) {
            m_receive_buffer.erase(m_receive_buffer.begin(),
                                   m_receive_buffer.end() - 1);
          } else {
            m_receive_buffer.clear();
          }
          return; // No packet header found, wait for more data
        }
        if ((packet_start + 1 != m_receive_buffer.end() &&
             *(packet_start + 1) != (packet_t::header & 0xFF))) {
          m_receive_buffer.erase(m_receive_buffer.begin(), packet_start + 1);
        } else {
          break; // Found potential packet header
        }
      }
      //检查数据包长度
      if (std::distance(packet_start, m_receive_buffer.end()) <
          static_cast<std::ptrdiff_t>(packet_t::header_size)) {
        return; // Not enough data for header, wait for more data
      }
      //读取数据长度
      uint16_t size = *(packet_start + 2) << 8 | *(packet_start + 3);
      //不够长就不要
      if (size < packet_t::header_size + packet_t::tail_size) {
        m_receive_buffer.erase(m_receive_buffer.begin(), packet_start + 1);
        continue; // Invalid size, resync stream
      }
      //数据长度不够就等待数据
      if (std::distance(packet_start, m_receive_buffer.end()) <
          static_cast<std::ptrdiff_t>(size)) {
        return; // Not enough data for whole packet, wait for more data
      }
      //验证包
      packet_t packet{packet_start, packet_start + size, from_whole_packet};
      if (packet) {
        if (m_receive_function) {
          m_receive_function(std::move(packet));
        }
      }
      m_receive_buffer.erase(m_receive_buffer.begin(),
                             packet_start +
                                 size); // Remove processed packet from buffer
    }
  }

protected:
  template <std::input_iterator It> struct receive_range {
    It m_begin;
    It m_end;
    It begin() { return m_begin; }
    It end() { return m_end; }
  };

private:
  gdut::function<void(const std::uint8_t *, const std::uint8_t *)>
      m_send_function;
  gdut::function<void(packet_t)> m_receive_function;
  std::pmr::vector<std::uint8_t> m_receive_buffer;
};

} // namespace gdut

#endif // MODULES_TRANSFER_PROTOCOL_HPP

#pragma pop_macro("abs")
