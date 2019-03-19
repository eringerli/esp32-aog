# esp32-aog
Software to control the tractor from AgOpenGPS

# WARNING
Read this file through, before starting to half-ass it. It is not so hard to install a working system, just give it enough time and install it in this order.

# Install Prerequisites
1. install atom: https://atom.io/
1. inside atom:
  1. click on "install a package"
  1. search for "platformio-ide" and install it
  1. install clang as desribed on the website
  1. its taking forever, so please be patient
  1. restart as asked
  1. open the platformio home tab
     1. go to "Platforms" (left side button)
     1. choose the "Embedded" tab
     1. install the "Esspressiv 32" platform
1. install git: https://git-scm.com/downloads
   1. use the defaults in the setup, define atom as the default editor

# Downloading the repository
1. open a folder in the explorer, preferably not deep inside the drive. `C:\` or a folder under it works
1. right click on it and choose "Git Bash Here"
1. enter `git clone https://github.com/eringerli/esp32-aog.git`
1. enter `cd esp32-aog`
1. enter `git submodule init`
1. enter `git submodule update`

# Compiling
1. open the created folder above from the platformio home
1. click build, the missing dependencies should be installed automaticaly
1. it is really important to heed the warning from the compiler about `"I2C_BUFFER_LENGTH" redifined`. Go into the the file and change the code like this:
    ```#define I2C_BUFFER_LENGTH 128```
  to
    ```
    #ifndef I2C_BUFFER_LENGTH
      #define I2C_BUFFER_LENGTH 128
    #endif
    ```
    This is for the readout of the FIFO of the MMA8252, as it reads more than 192Bytes a time. If it is not changed, the code won't work and crashes!
1. if that was to fast for you (I warned you in the start!), trigger a recompile with adding a space to somewhere in `autosteer.cpp`. The warning apears again.
1. save the file and close it
1. build again
