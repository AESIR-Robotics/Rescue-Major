/*/
// Designed this for runtype buffer builders
// Then remembered that messages are known at compile time making this unnecessary
// I do not have the heart to delete it though
// Might be useful for run time message testing

class BufferBuilder{
public:

  using info_unit = std::variant<int8_t, uint8_t,int16_t,
                              uint16_t, int32_t, uint32_t,
                              int64_t, uint64_t, float, double, char>;

  struct TypeInfo {
    char fmt;
    size_t size;
    info_unit (*make_default)();
  };

  static info_unit make_i8()  { return int8_t{}; }
  static info_unit make_u8()  { return uint8_t{}; }
  static info_unit make_i16() { return int16_t{}; }
  static info_unit make_u16() { return uint16_t{}; }
  static info_unit make_i32() { return int32_t{}; }
  static info_unit make_u32() { return uint32_t{}; }
  static info_unit make_i64() { return int64_t{}; }
  static info_unit make_u64() { return uint64_t{}; }
  static info_unit make_f32() { return float{}; }
  static info_unit make_f64() { return double{}; }
  static info_unit make_char(){ return char{}; }

  const static std::unordered_map<char, TypeInfo> type_table;

  BufferBuilder(const std::string & format): fmt{format} {
    buffer.reserve(16);
    setFormat(fmt);
  }

  bool setFormat(const std::string & new_format){
    if (!std::regex_match(new_format, regform)) {
      fmt = "";
      return false;
    }
    fmt = new_format;
    elements.clear();
    elements.reserve(fmt.size());
    for(auto x : fmt){
      elements.push_back(type_table.at(x).make_default());
    }
    buffer.clear();
    return true;
  }

  bool pack(const std::vector<info_unit> &information){
    if(elements.size() != information.size()){
      return false;
    }

    for (size_t i = 0; i < information.size(); ++i) {
      if (elements[i].index() != information[i].index()) {
        return false;
      }
    }

    elements = information;
    compileBuffer();
    return true;
  }

  const uint8_t * getBuffer() const {
    if(fmt == ""){
      return nullptr;
    }
    return buffer.data();
  }

  size_t getBufSize() const {
    return buffer.size();
  }

  std::string getFormat(){
    return fmt;
  }

private:
  std::string fmt;
  std::vector<uint8_t> buffer;
  std::vector<info_unit> elements;

  inline static std::regex regform{R"([bBhHiIqQfdc]*)"};

  bool compileBuffer(){
    buffer.clear();
    for(const auto & elem : elements){
      std::visit([this](auto&& arg){
        using T = std::decay_t<decltype(arg)>;
        const uint8_t * ptr = reinterpret_cast<const uint8_t*>(&arg);
        buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
      }, elem);
    }
    return true;
  }

};
  
const std::unordered_map<char, BufferBuilder::TypeInfo>  BufferBuilder::type_table{
      { 'b', {'b', 1, BufferBuilder::make_i8  } },
      { 'B', {'B', 1, BufferBuilder::make_u8  } },
      { 'h', {'h', 2, BufferBuilder::make_i16 } },
      { 'H', {'H', 2, BufferBuilder::make_u16 } },
      { 'i', {'i', 4, BufferBuilder::make_i32 } },
      { 'I', {'I', 4, BufferBuilder::make_u32 } },
      { 'q', {'q', 8, BufferBuilder::make_i64 } },
      { 'Q', {'Q', 8, BufferBuilder::make_u64 } },
      { 'f', {'f', 4, BufferBuilder::make_f32 } },
      { 'd', {'d', 8, BufferBuilder::make_f64 } },
      { 'c', {'c', 1, BufferBuilder::make_char} },
  };
*/
