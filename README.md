# esp32-aog
Software to control the tractor from AgOpenGPS

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
    This is for the readout of the FIFO of the MMA8252, as it reads more than 192Bytes a time. if it is not changed, the code won't work and crashes!
    
