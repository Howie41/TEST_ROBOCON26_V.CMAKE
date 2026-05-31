#ifndef COMPONENTS_FUNCTION_HPP
#define COMPONENTS_FUNCTION_HPP

#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

//所有东西都在gdut命名空间下，避免和其他库冲突
namespace gdut {

template <std::size_t Value> struct is_power_of_two {
  static constexpr bool value = Value && (Value & (Value - 1)) == 0;//看看是不是2的幂次方，
                          // 2的幂次方在二进制表示中只有一个1，所以Value & (Value - 1)应该为0
                          //为了内存对齐
};

template <std::size_t Value>       //编译时常量，判断是否是2的幂次方
inline constexpr bool is_power_of_two_v = is_power_of_two<Value>::value;

template <typename> struct always_false : std::false_type {};

template <typename T>
inline constexpr bool always_false_v = always_false<T>::value;

//主模板函数对象类，不能直接实例化，必须通过特化来使用
template <typename T, std::size_t StorageSize, std::size_t Alignment>
class basic_function {
  static_assert(always_false_v<T>, "basic_function is a base class template "
                                   "and cannot be instantiated directly.");
};

//R对应的是返回值，Args对应的是参数列表，StorageSize是存储空间大小，Alignment是对齐要求
template <typename R, typename... Args, std::size_t StorageSize,
          std::size_t Alignment>
class basic_function<R(Args...), StorageSize, Alignment> {
  static_assert(is_power_of_two_v<Alignment>,
                "Alignment must be a power of two.");

public:
  basic_function() noexcept = default;
  basic_function(std::nullptr_t) noexcept {}

  basic_function(const basic_function &other) {
    if (other.m_callable) {
      other.m_callable->clone(m_storage);
      m_callable = std::launder(reinterpret_cast<callable *>(m_storage));
    }
  }

  template <typename Func, typename = std::enable_if_t<!std::is_same_v<
                               std::decay_t<Func>, basic_function>>>
  basic_function(Func &&func) {
    emplace(std::forward<Func>(func));
  }

  basic_function(basic_function &&other) noexcept {
    if (other.m_callable) {
      other.m_callable->move(m_storage);
      m_callable = std::launder(reinterpret_cast<callable *>(m_storage));
      other.destroy();
    }
  }

  ~basic_function() noexcept { destroy(); }

  bool valid() const noexcept { return m_callable != nullptr; }

  R operator()(Args... args) {
    if (!valid()) {
      std::terminate();
    }
    return std::invoke(*m_callable, std::forward<Args>(args)...);
  }

  explicit operator bool() const noexcept { return valid(); }

  basic_function &operator=(const basic_function &other) {
    if (this != std::addressof(other)) {
      destroy();
      if (other.m_callable) {
        other.m_callable->clone(m_storage);
        m_callable = std::launder(reinterpret_cast<callable *>(m_storage));
      }
    }
    return *this;
  }

  basic_function &operator=(basic_function &&other) noexcept {
    if (this != std::addressof(other)) {
      destroy();
      if (other.m_callable) {
        other.m_callable->move(m_storage);
        m_callable = std::launder(reinterpret_cast<callable *>(m_storage));
        other.destroy();
      }
    }
    return *this;
  }

  basic_function &operator=(std::nullptr_t) noexcept {
    destroy();
    return *this;
  }

  template <typename Func, typename = std::enable_if_t<!std::is_same_v<
                               std::decay_t<Func>, basic_function>>>
  basic_function &operator=(Func &&func) {
    destroy();
    emplace(std::forward<Func>(func));
    return *this;
  }

  void swap(basic_function &other) noexcept {
    if (this == std::addressof(other)) {
      return;
    }
    basic_function tmp(std::move(other));
    other = std::move(*this);
    *this = std::move(tmp);
  }

  friend void swap(basic_function &lhs, basic_function &rhs) noexcept {
    lhs.swap(rhs);
  }

protected:
  struct callable {
    virtual R operator()(Args... args) = 0;
    virtual void clone(std::byte *storage) = 0;
    virtual void move(std::byte *storage) noexcept = 0;
    virtual ~callable() noexcept = default;
  };

  template <typename Func> struct model : callable {
    Func func;

    model(const Func &fn) : func(fn) {}
    model(Func &&fn) : func(std::move(fn)) {}

    R operator()(Args... args) override {
      return std::invoke(func, std::forward<Args>(args)...);
    }

    void clone(std::byte *storage) override {
      std::construct_at(reinterpret_cast<model<Func> *>(storage), *this);
    }
    void move(std::byte *storage) noexcept override {
      std::construct_at(reinterpret_cast<model<Func> *>(storage),
                        std::move(*this));
    }

    ~model() noexcept override = default;
  };

  template <typename Func> void emplace(Func &&func) {
    static_assert(
        sizeof(model<std::decay_t<Func>>) <= StorageSize,
        "The function object is too large and exceeds the storage space.");
    static_assert(std::is_invocable_r_v<R, std::decay_t<Func>, Args...>,
                  "The function object must be callable and match the "
                  "specified function signature.");
    static_assert(alignof(model<std::decay_t<Func>>) <= Alignment,
                  "The function object requires stricter alignment than "
                  "storage provides.");
    static_assert(std::is_nothrow_move_constructible_v<std::decay_t<Func>>,
                  "The function object must be nothrow move constructible.");
    static_assert(std::is_nothrow_destructible_v<std::decay_t<Func>>,
                  "The function object must be nothrow destructible.");
    std::construct_at(reinterpret_cast<model<std::decay_t<Func>> *>(m_storage),
                      std::forward<Func>(func));
    m_callable = std::launder(reinterpret_cast<callable *>(m_storage));
  }

  void destroy() noexcept {
    if (m_callable) {
      std::destroy_at(m_callable);
      m_callable = nullptr;
    }
  }

private:
  callable *m_callable{nullptr};
  alignas(Alignment) std::byte m_storage[StorageSize];
};

template <typename Func, std::size_t StorageSize = 16,
          std::size_t Alignment = alignof(std::max_align_t)>
using function = basic_function<Func, StorageSize, Alignment>;

} // namespace gdut

#endif // COMPONENTS_FUNCTION_HPP
