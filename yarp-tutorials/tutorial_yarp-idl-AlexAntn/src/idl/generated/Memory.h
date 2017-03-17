// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_Memory
#define YARP_THRIFT_GENERATOR_Memory

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class Memory;


class Memory : public yarp::os::Wire {
public:
  Memory();
  virtual std::string get_answer(const std::string& k);
  virtual bool push(const std::string& k, const std::string& v);
  virtual bool show_list();
  virtual bool clear();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
