# esp32-aog
Software to control the tractor from AgOpenGPS

# WARNING
Read this file through, before starting to half-ass it. It is not so hard to get a working system, just give it enough time and install it in this order.

# Install Prerequisites
1. install atom: https://atom.io/
1. inside atom:
   1. click on "install a package"
   1. search for "platformio-ide" and install it
   1. install clang as desribed on the website
   1. it's taking forever, so please be patient
   1. restart as asked
   1. open the platformio home tab (if not opened automaticaly, use the menu)
      1. go to "Platforms" (left side tile)
      1. choose the "Embedded" tab
      1. install the "Esspressiv 32" platform
1. install git: https://git-scm.com/downloads
   1. use the defaults in the setup, define atom as the default editor

# Downloading the repository
1. open a folder in the explorer, preferably not too deep inside the drive. `C:\` or a folder under it should work
1. right click on it and choose "Git Bash Here"
1. enter `git clone https://github.com/eringerli/esp32-aog.git`
1. enter `cd esp32-aog`
1. enter `git submodule init`
1. enter `git submodule update`

# Compiling
1. open the created folder above from the platformio home
1. click build (the tile with the tick), the missing dependencies should be installed automaticaly

# Uploading
1. click on upload (the tile with the arrow)
