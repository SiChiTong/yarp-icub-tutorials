# assignment_Codec.thrift

service Codec_IDL {

  /**
  * set_codec sets the codec character
  * @param c a byte
  * @return true/false on success/failure
  */
  bool set_codec(1: byte c);

  /**
  * set_mode sets the mode
  * @param mode a string  i.e. 'coder/decoder'
  * @return true/false on success/failure
  */
  bool set_mode(1: string mode);

  /**
  * get_mode gets the current mode
  * @return an string showing current mode i.e. 'coder/decoder'
  */
  string get_mode();

}
