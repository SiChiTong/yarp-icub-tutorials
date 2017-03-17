// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <Memory.h>
#include <yarp/os/idl/WireTypes.h>



class Memory_get_answer : public yarp::os::Portable {
public:
  std::string k;
  std::string _return;
  void init(const std::string& k);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class Memory_push : public yarp::os::Portable {
public:
  std::string k;
  std::string v;
  bool _return;
  void init(const std::string& k, const std::string& v);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class Memory_show_list : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class Memory_clear : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool Memory_get_answer::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("get_answer",1,2)) return false;
  if (!writer.writeString(k)) return false;
  return true;
}

bool Memory_get_answer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void Memory_get_answer::init(const std::string& k) {
  _return = "";
  this->k = k;
}

bool Memory_push::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("push",1,1)) return false;
  if (!writer.writeString(k)) return false;
  if (!writer.writeString(v)) return false;
  return true;
}

bool Memory_push::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void Memory_push::init(const std::string& k, const std::string& v) {
  _return = false;
  this->k = k;
  this->v = v;
}

bool Memory_show_list::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("show_list",1,2)) return false;
  return true;
}

bool Memory_show_list::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void Memory_show_list::init() {
  _return = false;
}

bool Memory_clear::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clear",1,1)) return false;
  return true;
}

bool Memory_clear::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void Memory_clear::init() {
  _return = false;
}

Memory::Memory() {
  yarp().setOwner(*this);
}
std::string Memory::get_answer(const std::string& k) {
  std::string _return = "";
  Memory_get_answer helper;
  helper.init(k);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string Memory::get_answer(const std::string& k)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool Memory::push(const std::string& k, const std::string& v) {
  bool _return = false;
  Memory_push helper;
  helper.init(k,v);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool Memory::push(const std::string& k, const std::string& v)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool Memory::show_list() {
  bool _return = false;
  Memory_show_list helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool Memory::show_list()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool Memory::clear() {
  bool _return = false;
  Memory_clear helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool Memory::clear()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool Memory::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "get_answer") {
      std::string k;
      if (!reader.readString(k)) {
        reader.fail();
        return false;
      }
      std::string _return;
      _return = get_answer(k);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "push") {
      std::string k;
      std::string v;
      if (!reader.readString(k)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(v)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = push(k,v);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "show_list") {
      bool _return;
      _return = show_list();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "clear") {
      bool _return;
      _return = clear();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> Memory::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("get_answer");
    helpString.push_back("push");
    helpString.push_back("show_list");
    helpString.push_back("clear");
    helpString.push_back("help");
  }
  else {
    if (functionName=="get_answer") {
      helpString.push_back("std::string get_answer(const std::string& k) ");
    }
    if (functionName=="push") {
      helpString.push_back("bool push(const std::string& k, const std::string& v) ");
    }
    if (functionName=="show_list") {
      helpString.push_back("bool show_list() ");
    }
    if (functionName=="clear") {
      helpString.push_back("bool clear() ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


