// Formatter.cc
// VERSION 1.1.0
#include "Formatter.hh"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Gen Funcs
bool isValidSymbol(char c){
  return (c >= 33 && c <= 45) || c == 47 || (c >= 58 && c <= 64) || (c >= 91 && c <= 96) || (c >= 123 && c <= 126);
}

char letter(int i){
  if(i < 0)
    return 21;
  else if(i < 26)
    return (i + 65);
  else
    return (i + 97);
}

int number(char c){
  return ((int)c < 91) ? (int)c - 65 : (int)c - 97;
}

void Formatter::addChar(char c){
  msg[msgi++] = c;
  msg[msgi] = '\0';
}

int convert(const val_fmt& from, const val_fmt& to, int v){
    long f = v;
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    f = (f - from.off) * to.scale / from.scale + to.off;
    f = f < to.min_val ? to.min_val : (f > to.max_val ? to.max_val : f);
  return f;
}

float convert(const val_fmt& from, const val_fmt& to, float f){
    f = f < from.min_val ? from.min_val : (f > from.max_val ? from.max_val : f);
    f = (f - from.off) * to.scale / from.scale + to.off;
    f = f < to.min_val ? to.min_val : (f > to.max_val ? to.max_val : f);
  return f;
}

const val_fmt* const Formatter::getFormat(char symbol){
  if(isValidSymbol(symbol))
    for(int i = 0; i < numFormats; ++i){
      if(formats[i].symbol == symbol)
        return &formats[i];
    }
  return NULL;
}

const val_fmt* const Formatter::getFormat(const char data_t[]){
  for(int i = 0; i < numFormats; ++i){
    if(strcmp(formats[i].data_t, data_t) == 0)
      return &formats[i];
  }
  return NULL;
}

Formatter::Formatter(int argc, val_fmt argv[]){
  numFormats = argc;
  formats = new val_fmt[argc];
  for(int i=0; i<argc; ++i)
    formats[i] = argv[i];
  newMsg = true;
  msgi = 0;
  memset(msg,'\0',300);
}

Formatter::~Formatter(){
  delete formats;
}

int Formatter::getVal(int in, const val_fmt* fmt){
  long out = in;
  out = out * fmt->scale + fmt->off;
  if(out > fmt->max_val)
    return fmt->max_val;
  else if(out < fmt->min_val)
    return fmt->min_val;
  else
    return out;
}

int Formatter::getVal(float value, const val_fmt* fmt){
  int out = (int)(value * fmt->scale + fmt->off);
  if(out > fmt->max_val)
    return fmt->max_val;
  else if(out < fmt->min_val)
    return fmt->min_val;
  else
    return out;
}

int Formatter::getValue(int val, const val_fmt* fmt){
  return (val - fmt->off) / fmt->scale;
}

float Formatter::getValueFloat(int val, const val_fmt* fmt){
  return ((float)(val - fmt->off)) / fmt->scale;
}

void Formatter::add(const char data_t[], int i, int v, const char apriori_data_t[]){
  IV iv = {i,v};
  add(data_t,iv, apriori_data_t);
}

void Formatter::add(const char data_t[], const IV& idv, const char apriori_data_t[]){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* preApp = getFormat(apriori_data_t);
  assert(preApp);
  add(data_t, idv, *preApp);
}

void Formatter::add(const char data_t[], int i, int v, const val_fmt& apriori_fmt){
  IV iv = {i,v};
  add(data_t,iv,apriori_fmt);
}

void Formatter::add(const char data_t[], const IV& idv, const val_fmt& apriori_fmt){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  addChar(fmt->symbol);
  addChar(letter(idv.i));
  char tmp[15];
  sprintf(tmp,"%d",(int)convert(apriori_fmt,*fmt,idv.v));
  for(int i = 0, len = (unsigned)strlen(tmp); i < (fmt->bytes - len); ++i)
    addChar('0');
  for(int i = 0, len = (unsigned)strlen(tmp); i < len; ++i)
    addChar(tmp[i]);
}

void Formatter::add(const char data_t[], int i, int v){
  IV iv = {i,v};
  add(data_t, iv);
}

void Formatter::add(const char data_t[], const IV& idv){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  addChar(fmt->symbol);
  addChar(letter(idv.i));
  char tmp[15];
  sprintf(tmp,"%d",getVal(idv.v,fmt));
  for(int i = 0, len = (unsigned)strlen(tmp); i < (fmt->bytes - len); ++i)
    addChar('0');
  for(int i = 0, len = (unsigned)strlen(tmp); i < len; ++i)
    addChar(tmp[i]);
}
    
void Formatter::addFloat(const char data_t[], int i, float v, const char apriori_data_t[]){
  IV_float iv = {i,v};
  addFloat(data_t,iv,apriori_data_t);
}

void Formatter::addFloat(const char data_t[], const IV_float& idv, const char apriori_data_t[]){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* preApp = getFormat(apriori_data_t);
  assert(preApp);
  addFloat(data_t, idv, *preApp);
}

void Formatter::addFloat(const char data_t[], int i, float v, const val_fmt& apriori_fmt){
  IV_float iv = {i,v};
  addFloat(data_t,iv,apriori_fmt);
}

void Formatter::addFloat(const char data_t[], const IV_float& idv, const val_fmt& apriori_fmt){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  addChar(fmt->symbol);
  addChar(letter(idv.i));
  char tmp[15];
  sprintf(tmp,"%d",(int)convert(apriori_fmt,*fmt,idv.v));
  for(int i = 0, len = (unsigned)strlen(tmp); i < (fmt->bytes - len); ++i)
    addChar('0');
  for(int i = 0, len = (unsigned)strlen(tmp); i < len; ++i)
    addChar(tmp[i]);
}

void Formatter::addFloat(const char data_t[], int i, float v){
  IV_float iv = {i,v};
  addFloat(data_t,iv);
}

void Formatter::addFloat(const char data_t[], const IV_float& idv){
  if(newMsg){
    newMsg = false;
    msgi = 0;
    memset(msg,'\0',300);
  }
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt); // Check that code runs; this should 
  addChar(fmt->symbol);
  addChar(letter(idv.i));
  char tmp[15];
  sprintf(tmp,"%d",getVal(idv.v,fmt));
  for(int i = 0, len = (unsigned)strlen(tmp); i < (fmt->bytes - len); ++i)
    addChar('0');
  for(int i = 0, len = (unsigned)strlen(tmp); i < len; ++i)
    addChar(tmp[i]);
}

char* Formatter::emit(){
  addChar('\n');
  newMsg = true;
  return msg;
}

IV* Formatter::nextIV(IV_list* &list){
  if(!list)
    return NULL;
  IV* tmp = &(list->iv);
  list = list->next;
  return tmp;
}

IV_float* Formatter::nextIV_float(IV_float_list* &list){
  if(!list)
    return NULL;
  IV_float* tmp = &(list->iv);
  list = list->next;
  return tmp;
}

IV_list* Formatter::parse(char message[], const char data_t[]){
  struct IV_list* out = NULL; //(struct IV_list*) malloc(sizeof(struct IV_list));
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt);
  for(int i = 0, len = (unsigned)strlen(message); i < len; ++i){
    if(message[i] == fmt->symbol){
      struct IV_list* newFirst = (struct IV_list*) malloc(sizeof(struct IV_list));
      char tmp[fmt->bytes + 1];
      strncpy(tmp,&message[i+2],fmt->bytes);
      tmp[fmt->bytes] = '\0';
      const IV iv = {
          number(message[i+1]),
          (atoi(tmp) - fmt->off) / fmt->scale
      };
      newFirst->iv = iv;
      newFirst->next = out; // Move chain down
      out = newFirst;
    }
  }
  return out;
}

IV_list* Formatter::parse(char message[], const char from_data_t[], const char to_data_t[]){
  struct IV_list* out = NULL; //(struct IV_list*) malloc(sizeof(struct IV_list));
  const val_fmt* from_fmt = getFormat(from_data_t);
  assert(from_fmt);
  const val_fmt* to_fmt = getFormat(to_data_t);
  assert(to_fmt);
  for(int i = 0, len = (unsigned)strlen(message); i < len; ++i){
    if(message[i] == from_fmt->symbol){
      struct IV_list* newFirst = (struct IV_list*) malloc(sizeof(struct IV_list));
      char tmp[from_fmt->bytes + 1];
      strncpy(tmp,&message[i+2],from_fmt->bytes);
      tmp[from_fmt->bytes] = '\0';
      const IV iv = {
          number(message[i+1]),
          convert(*from_fmt, *to_fmt, atoi(tmp))
      };
      newFirst->iv = iv;
      newFirst->next = out; // Move chain down
      out = newFirst;
    }
  }
  return out;
}

IV_float_list* Formatter::parseFloat(char message[], const char data_t[]){
  struct IV_float_list* out = NULL; //(struct IV_float_list*) malloc(sizeof(struct IV_float_list));
  const val_fmt* fmt = getFormat(data_t);
  assert(fmt);
  for(int i = 0, len = (unsigned)strlen(message); i < len; ++i){
    if(message[i] == fmt->symbol){
      struct IV_float_list* newFirst = 
        (struct IV_float_list*) malloc(sizeof(struct IV_float_list));
      char tmp[fmt->bytes + 1];
      strncpy(tmp,&message[i+2],fmt->bytes);
      tmp[fmt->bytes] = '\0';
      const IV_float iv = {
          number(message[i+1]),
          ((float)(atoi(tmp) - fmt->off)) / fmt->scale
      };
      newFirst->iv = iv;
      newFirst->next = out; // Move chain down
      out = newFirst;
    }
  }
  return out;
}

IV_float_list* Formatter::parseFloat(char message[], const char from_data_t[], const char to_data_t[]){
  struct IV_float_list* out = NULL; //(struct IV_float_list*) malloc(sizeof(struct IV_float_list));
  const val_fmt* from_fmt = getFormat(from_data_t);
  assert(from_fmt);
  const val_fmt* to_fmt = getFormat(to_data_t);
  assert(to_fmt);
  for(int i = 0, len = (unsigned)strlen(message); i < len; ++i){
    if(message[i] == from_fmt->symbol){
      struct IV_float_list* newFirst = (struct IV_float_list*) malloc(sizeof(struct IV_float_list));
      char tmp[from_fmt->bytes + 1];
      strncpy(tmp,&message[i+2],from_fmt->bytes);
      tmp[from_fmt->bytes] = '\0';
      const IV_float iv = {
          number(message[i+1]),
          convert(*from_fmt, *to_fmt, (float)atoi(tmp))
      };
      newFirst->iv = iv;
      newFirst->next = out; // Move chain down
      out = newFirst;
    }
  }
  return out;
}
