#ifndef FORMATTER_HH
#define FORMATTER_HH
// VERSION 1.1.0

/* Formatter.hh defines a class to parameterize a standard formatter for sending 
 * and recieving messages, usually for the same expected interface. 
 * This class is meant to be c/c++, for use with arduino. Formatter.hpp for c++.
 * This does not require c++11 to use. 
 * A type of communicatable object is parameterized by a val_fmt, which describes 
 * the processing of that type, encoding and decoding it from a stream or buffer. 
 * A formatter for a specific interface is initialized / constructed with a 
 * numbered array of val_fmt structs, to which that interface is limited to. These
 * should be a runtime constant, constant with the program designs, internal. 
 * A group of values to be communicated, such as the speeds for 7 motors, are
 * processed as a vector of IV{ int i; int v} structs, where 'i' is the index
 * or id of the unit, ex motor, and 'v' is a value, expressed as an int, through
 * val_fmt. It can be converted between val_fmts.
 * Each kind of group of values to be communicated is added to a buffer with 
 * 'add', and then the buffer is emptied and returned with 'emit'.
 * Return types were decided to be ints/floats, given the expected ARM arch.
 * Parsing is similar, given a string message: it is handled for each data_t
 * and val_fmt, returned in index - int/float pairs (for simplicity). 
 * The IV values can be cheaply initiated by {0,4}, {1,5}, {2,-3} etc. 
 * Parsing returns a linked list of values extracted, example usage is given
 * below with the function. 
 */

// For each type "data_t" of send/recv -able data, define simple conv:
// Convert from arb. value to a storable val with scale==resolution
// value := (val - off) / scale -> val := value * scale + off
// "data_t" in message has form of:
// char 'symbol' + char bundleID + 'bytes' length chars
// Symbol: printable non-alpha-numeric: ASCII [33,45],47,[58,64],[91,96],[123,126]
struct val_fmt {
  char data_t[20];
  char symbol;
  int bytes;
  int min_val; // Safety checks on 'val' possible values, bounds
  int max_val;
  int off; // Equal to scale*real_offset(float)
  int scale; // NOTE: scale is really just the range, but I don't want to change/cleanup
};

struct IV {
  int i;
  int v;
};

struct IV_float {
  int i;
  float v;
};

#include <stdlib.h>

struct IV_list {
  IV iv;
  struct IV_list* next;
};

struct IV_float_list {
  IV_float iv;
  struct IV_float_list* next;
};

// Generic Conversion Functions, not tied
int convert(const val_fmt& from, const val_fmt& to, int values);
float convert(const val_fmt& from, const val_fmt& to, float values);

class Formatter {
  public: 
    Formatter(int argc, val_fmt argv[]);
    ~Formatter();
    
    // Sending / Encoding:
    // Build up a string to output: apriori conversion optional
    void add(const char data_t[], const IV& ids_values);
    void add(const char data_t[], const IV& ids_values, const char apriori_data_t[]);
    void add(const char data_t[], const IV& ids_values, const val_fmt& apriori_fmt);
    void add(const char data_t[], int i, int v);
    void add(const char data_t[], int i, int v, const char apriori_data_t[]);
    void add(const char data_t[], int i, int v, const val_fmt& apriori_fmt);
    
    // Support for floats
    void addFloat(const char data_t[], const IV_float& ids_values);
    void addFloat(const char data_t[], const IV_float& ids_values, const char apriori_data_t[]);
    void addFloat(const char data_t[], const IV_float& ids_values, const val_fmt& apriori_fmt);
    void addFloat(const char data_t[], int i, float v);
    void addFloat(const char data_t[], int i, float v, const char apriori_data_t[]);
    void addFloat(const char data_t[], int i, float v, const val_fmt& apriori_fmt);
    
    // Emit string. Resets to empty
    char* emit();

    // Receiving / Parsing:
    // Parse from string all values of type data_t
    IV_list* parse(char message[], const char data_t[]);
    IV_list* parse(char message[], const char from_data_t[], const char to_data_t[]);
    
    // Depending on data type, float could be prefered
    IV_float_list* parseFloat(char message[], const char data_t[]);
    IV_float_list* parseFloat(char message[], const char from_data_t[], const char to_data_t[]);
    
    // Iterators on lists
    IV* nextIV(IV_list* &list);
    IV_float* nextIV_float(IV_float_list* &list);
      // Usage, consumes list:
      // IV* ivPtr; 
      // IV_list* list = parse("Message","DataType");
      // while(ivPtr = nextIV(list)){ 
      //   ivPtr->i // ... do something
      //   ivPtr->v // ... do something
      //   free(ivPtr);
      // }

    // Basic conversions
    int getValue(int val, const val_fmt* fmt);
    float getValueFloat(int val, const val_fmt* fmt);
    int getVal(int value, const val_fmt*);
    int getVal(float value, const val_fmt*);
    const val_fmt* const getFormat(char symbol);
    const val_fmt* const getFormat(const char data_t[]);

  private:
    int numFormats;
    val_fmt* formats;
    int msgi;
    char msg[300]; 
    bool newMsg;
    void addChar(char c);

};

#endif
