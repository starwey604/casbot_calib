#pragma once
#include <charconv>
#include <ostream>
#include <type_traits>
#include <stdexcept>

namespace vendor_text {
// Locale-independent round-trip numbers, avoiding locale/printf conversion in
// high-rate CSV/JSON rows. Seventeen digits retain the original double value.
template<class T> void number(std::ostream& out,T value) {
  char buffer[64];std::to_chars_result result;
  if constexpr(std::is_floating_point_v<T>)result=std::to_chars(buffer,buffer+64,value,std::chars_format::general,17);
  else result=std::to_chars(buffer,buffer+64,static_cast<std::conditional_t<std::is_same_v<T,bool>,int,T>>(value));
  if(result.ec!=std::errc{})throw std::runtime_error("Numeric conversion failed");
  out.write(buffer,result.ptr-buffer);
}
}
