# gr-plc
GNURadio blocks and library for IEEE1901/HomePlug-like communication.
## Install
    mkdir build  
    cd build  
    cmake ../  
    make  
    sudo make install  
    sudo ldconfig  
    group - rtprio 99  
To disable thread priority UHD warning, add the following to */etc/security/limits.conf*  

    @<group> - rtprio 99  

Where *&lt;group>* should be replaced with the user group
