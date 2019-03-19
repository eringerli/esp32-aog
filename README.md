# esp32-aog
Software to controll the tracor from AgOpenGPS


# compiling

- install platformio and all the required libs
- compile
- make the buffer of the I2C-queue bigger
  - click on the error and replace the following code
    ```#define I2C_BUFFER_LENGTH 128```
  with
    ```
    #ifndef I2C_BUFFER_LENGTH
      #define I2C_BUFFER_LENGTH 128
    #endif
    ```
