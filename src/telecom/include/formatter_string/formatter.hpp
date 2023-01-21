#ifndef FORMATTER_HPP
#define FORMATTER_HPP
// VERSION 1.4.1

/* Formatter defines a class to parameterize a standard formatter for sending 
 * and recieving messages, usually for the same expected interface. 
 * A type of communicatable object is parameterized by a val_fmt, which describes 
 * the processing of that type, encoding and decoding it from a stream or buffer. 
 * A formatter for a specific interface is initialized / constructed with a 
 * vector of val_fmt structs, to which that interface is limited to. These
 * should be a runtime constant, constant with the program designs, internal. 
 * A group of values to be communicated, such as the speeds for 7 motors, are
 * processed as a vector of IV{ int i; int v} structs, where 'i' is the index
 * or id of the unit, ex motor, and 'v' is a value, expressed as an int, through
 * val_fmt. It can be converted between val_fmts.
 * Each kind of group of values to be communicated is added to a buffer with 
 * 'add', and then the buffer is emptied and returned with 'emit'.
 * Return types were decided to be floats, given the expected ARM arch.
 * Parsing is similar, given a string message: it is handled for each data_t
 * and val_fmt, returned in index / float pairs (for simplicity). 
 * The vectors can be cheaply initiated by {{0,4},{1,5},{2,-3}} etc. 
 * Floats: a float from -1.0 to 1.0 doesn't need a conv_from format, just
 * apply the proper scaling with addFloat("_msg_fmt", _idv). Get back out
 * as a float with parseFloat(recv_msg, "_msg_fmt"). If you want to conv to
 * a different float scale, then use parseFloat with 2 fmts. If you want to
 * keep the int repr after recv'ing it, use parse() with a second fmt with 
 * scale=1, and shift bytes,min_val,max_val if want signed int else keep same.
 */

#include <string>
#include <vector>

// For each type "data_t" of send/recv -able data, define simple conv:
// Convert from arb. value to a storable val with scale==resolution
// value := (val - off) / scale -> val := value * scale + off
// "data_t" in message has form of:
// char 'symbol' + char bundleID + 'bytes' length chars
// Symbol: printable non-alpha-numeric: ASCII [33,45],47,[58,64],[91,96],[123,126]
struct val_fmt {
  std::string data_t;
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

// Generic Conversion Functions, not tied
std::vector<IV> convert(const val_fmt& from, const val_fmt& to, std::vector<IV> values);
int convert(const val_fmt& from, const val_fmt& to, int values);
std::vector<IV_float> convert(const val_fmt& from, const val_fmt& to, std::vector<IV_float> values);
float convert(const val_fmt& from, const val_fmt& to, float values);

class Formatter {
  public: 
    Formatter(std::vector<val_fmt>);
    
    // Sending / Encoding:
    // Build up a string to output: apriori conversion optional
    void add(std::string data_t, const std::vector<IV>& ids_values, std::string apriori_data_t = "");
    void add(std::string data_t, const std::vector<IV>& ids_values, const val_fmt& apriori_fmt);

    // Support for floats
    void addFloat(std::string data_t, const std::vector<IV_float>& ids_values, std::string apriori_data_t = "");
    void addFloat(std::string data_t, const std::vector<IV_float>& ids_values, const val_fmt& apriori_fmt);
    
    // Emit string. Resets to empty
    std::string emit();

    // Receiving / Parsing:
    // Parse from string all values of type data_t
    std::vector<IV> parse(std::string message, std::string data_t);
    std::vector<IV> parse(std::string message, std::string from_data_t, std::string to_data_t);
    // Depending on data type, float could be prefered
    std::vector<IV_float> parseFloat(std::string message, std::string data_t);
    std::vector<IV_float> parseFloat(std::string message, std::string from_data_t, std::string to_data_t);
    // Check for simply just a text message
    bool hasSymbols(std::string message);

    // Basic conversions
    int getValue(int val, const val_fmt* fmt);
    float getValueFloat(int val, const val_fmt* fmt);
    int getVal(int value, const val_fmt*);
    int getVal(float value, const val_fmt*);
    const val_fmt* getFormat(char symbol);
    const val_fmt* getFormat(std::string data_t);

  private:
    const std::vector<val_fmt> formats;
    std::string msg; 
    bool newMsg;
    std::string symbols;

};

inline bool operator==(const IV& lhs, const IV& rhs){ return lhs.i == rhs.i && lhs.v == rhs.v; }
inline bool operator!=(const IV& lhs, const IV& rhs){ return lhs.i != rhs.i || lhs.v != rhs.v; }

inline bool operator==(const IV_float& lhs, const IV_float& rhs){ return lhs.i == rhs.i && lhs.v == rhs.v; }
inline bool operator!=(const IV_float& lhs, const IV_float& rhs){ return lhs.i != rhs.i || lhs.v != rhs.v; }

#endif
