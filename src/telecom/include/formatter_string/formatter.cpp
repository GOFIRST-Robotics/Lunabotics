// Formatter.cpp
// VERSION 1.4.1
#include <vector>
#include <string>
#include <cassert>

#include "formatter_string/formatter.hpp"

// Gen Funcs
inline bool isValidSymbol(char c){
  return (c >= 33 && c <= 45) || c == 47 || (c >= 58 && c <= 64) || (c >= 91 && c <= 96) || (c >= 123 && c <= 126);
}

inline char letter(int i){
  if(i < 0)
    return 21;
  else if(i < 26)
    return (i + 65);
  else
    return (i + 97);
}

inline int number(char c){
  return ((int)c < 91) ? (int)c - 65 : (int)c - 97;
}

std::vector<IV> convert(const val_fmt& from, const val_fmt& to, std::vector<IV> values){
  for(std::string::size_type i = 0; i < values.size(); ++i){
    int f = values[i].v;
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    int t = (f - from.off) * to.scale / from.scale + to.off;
    values[i].v = t < to.min_val ? to.min_val : (t > to.max_val ? to.max_val : t);
  }
  return values;
}

int convert(const val_fmt& from, const val_fmt& to, int f){
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    f = (f - from.off) * to.scale / from.scale + to.off;
    f = f < to.min_val ? to.min_val : (f > to.max_val ? to.max_val : f);
  return f;
}

std::vector<IV_float> convert(const val_fmt& from, const val_fmt& to, std::vector<IV_float> values){
  for(std::string::size_type i = 0; i < values.size(); ++i){
    float f = values[i].v;
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    float t = (f - from.off) * to.scale / from.scale + to.off;
    values[i].v = t < to.min_val ? to.min_val : (t > to.max_val ? to.max_val : t);
  }
  return values;
}

float convert(const val_fmt& from, const val_fmt& to, float f){
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    f = (f - from.off) * to.scale / from.scale + to.off;
    f = f < to.min_val ? to.min_val : (f > to.max_val ? to.max_val : f);
  return f;
}

const val_fmt* Formatter::getFormat(char symbol){
  if(isValidSymbol(symbol))
    for(std::vector<val_fmt>::const_iterator it = formats.begin(); it != formats.end(); ++it){
      if(it->symbol == symbol)
        return &(*it);
    }
  return NULL;
}

const val_fmt* Formatter::getFormat(std::string data_t){
    for(std::vector<val_fmt>::const_iterator it = formats.begin(); it != formats.end(); ++it){
    if(it->data_t == data_t)
      return &(*it);
  }
  return NULL;
}

Formatter::Formatter(std::vector<val_fmt> fmts) : formats(fmts){
  //formats = fmts;
  newMsg = true;
  msg.reserve(300);
  msg = "";
  symbols = "";
  for(val_fmt fmt : formats)
    symbols += fmt.symbol;
}

inline int Formatter::getVal(int out, const val_fmt* fmt){
  out = out * fmt->scale + fmt->off;
  if(out > fmt->max_val)
    return fmt->max_val;
  else if(out < fmt->min_val)
    return fmt->min_val;
  else
    return out;
}

inline int Formatter::getVal(float value, const val_fmt* fmt){
  int out = (int)(value * fmt->scale + fmt->off);
  if(out > fmt->max_val)
    return fmt->max_val;
  else if(out < fmt->min_val)
    return fmt->min_val;
  else
    return out;
}

inline int Formatter::getValue(int val, const val_fmt* fmt){
  return (val - fmt->off) / fmt->scale;
}

inline float Formatter::getValueFloat(int val, const val_fmt* fmt){
  return ((float)(val - fmt->off)) / fmt->scale;
}

void Formatter::add(std::string data_t, const std::vector<IV>& ids_values, std::string apriori_data_t){
  if(newMsg){
    newMsg = false;
    msg = "";
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  const val_fmt* preApp = NULL;
  if(apriori_data_t != ""){
    preApp = getFormat(apriori_data_t);
    assert(preApp);
  }
  for(IV idv : ids_values){
    msg += fmt->symbol;
    msg += letter(idv.i);
    std::string tmp = std::to_string(
        (apriori_data_t == "") ? getVal(idv.v,fmt) : (int)convert(*preApp,*fmt,idv.v));
    msg.append(fmt->bytes - tmp.size(),'0');
    msg += tmp;
  }
}

void Formatter::add(std::string data_t, const std::vector<IV>& ids_values, const val_fmt& apriori_fmt){
  if(newMsg){
    newMsg = false;
    msg = "";
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  for(IV idv : ids_values){
    msg += fmt->symbol;
    msg += letter(idv.i);
    std::string tmp = std::to_string((int)convert(apriori_fmt,*fmt,idv.v));
    msg.append(fmt->bytes - tmp.size(),'0');
    msg += tmp;
  }
}

void Formatter::addFloat(std::string data_t, const std::vector<IV_float>& ids_values, std::string apriori_data_t){
  if(newMsg){
    newMsg = false;
    msg = "";
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  const val_fmt* preApp = NULL;
  if(apriori_data_t != ""){
    preApp = getFormat(apriori_data_t);
    assert(preApp);
  }
  for(IV_float idv : ids_values){
    msg += fmt->symbol;
    msg += letter(idv.i);
    std::string tmp = std::to_string(
        (apriori_data_t == "") ? getVal(idv.v,fmt) : (int)convert(*preApp,*fmt,idv.v));
    msg.append(fmt->bytes - tmp.size(),'0');
    msg += tmp;
  }
}

void Formatter::addFloat(std::string data_t, const std::vector<IV_float>& ids_values, const val_fmt& apriori_fmt){
  if(newMsg){
    newMsg = false;
    msg = "";
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  for(IV_float idv : ids_values){
    msg += fmt->symbol;
    msg += letter(idv.i);
    std::string tmp = std::to_string((int)convert(apriori_fmt,*fmt,idv.v));
    msg.append(fmt->bytes - tmp.size(),'0');
    msg += tmp;
  }
}

std::string Formatter::emit(){
  msg += "\n";
  newMsg = true;
  return msg;
}

std::vector<IV> Formatter::parse(std::string message, std::string data_t){
  std::vector<IV> out;
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt);
  for(std::string::size_type i = 0; i < message.size(); ++i){
    if(message[i] == fmt->symbol){
      out.push_back({
          number(message[i+1]),
          (std::stoi(message.substr(i+2,i+2 + fmt->bytes)) - fmt->off)
            / fmt->scale});
    }
  }
  return out;
}

std::vector<IV> Formatter::parse(std::string message, std::string from_data_t, std::string to_data_t){
  std::vector<IV> out;
  const val_fmt* from_fmt = getFormat(from_data_t);
  assert(from_fmt);
  const val_fmt* to_fmt = getFormat(to_data_t);
  assert(to_fmt);
  for(std::string::size_type i = 0; i < message.size(); ++i){
    if(message[i] == from_fmt->symbol){
      out.push_back({
          number(message[i+1]),
          convert(*from_fmt, *to_fmt, std::stoi(message.substr(i+2,i+2 + from_fmt->bytes)))
      });
    }
  }
  return out;
}

std::vector<IV_float> Formatter::parseFloat(std::string message, std::string data_t){
  std::vector<IV_float> out;
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt);
  for(std::string::size_type i = 0; i < message.size(); ++i){
    if(message[i] == fmt->symbol){
      out.push_back({
          number(message[i+1]),
          ((float)(std::stoi(message.substr(i+2,i+2 + fmt->bytes)) - fmt->off))
            / fmt->scale});
    }
  }
  return out;
}

std::vector<IV_float> Formatter::parseFloat(std::string message, std::string from_data_t, std::string to_data_t){
  std::vector<IV_float> out;
  const val_fmt* from_fmt = getFormat(from_data_t);
  assert(from_fmt);
  const val_fmt* to_fmt = getFormat(to_data_t);
  assert(to_fmt);
  for(std::string::size_type i = 0; i < message.size(); ++i){
    if(message[i] == from_fmt->symbol){
      out.push_back({
          number(message[i+1]),
          convert(*from_fmt,*to_fmt,(float)std::stoi(message.substr(i+2,i+2 + from_fmt->bytes)))
      });
    }
  }
  return out;
}

bool Formatter::hasSymbols(std::string message){
  return message.find_first_of(symbols) != std::string::npos;
}

