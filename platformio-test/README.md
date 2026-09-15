# Using this code

In order for the `HUSKYLENS` library to compile correctly, please make the following change:

```c++
// In file: /.pio/libdeps/HuskyLens/HuskyLensProtocolCore.c

// Line 135
uint8_t* husky_lens_protocol_write_begin(uint8_t command){
    send_fail = false;
    send_buffer[HEADER_0_INDEX] = 0x55;
    send_buffer[HEADER_1_INDEX] = 0xAA;
    send_buffer[ADDRESS_INDEX] = 0x11;
    send_buffer[COMMAND_INDEX] = command;
    send_index = CONTENT_INDEX;
    return send_buffer; // <--- Remove &
}
```

`send_buffer` is a `uint8_t[]` and thus is already `uint8_t*`