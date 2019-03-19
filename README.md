# esp32-aog
Software to controll the tracor from AgOpenGPS


# compiling
- init the submodules with `git submodules init` and `git submodules update`
- install platformio and all the required libs. For starting use the atom-IDE with the platformio-plugin, as it gives you a nice GUI to manage the platforms and libs. If correctly installed, you can use any IDE you want.
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
    
