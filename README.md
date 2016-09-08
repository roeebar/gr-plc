# gr-plc

GNU Radio blocks and library for IEEE1901/HomePlug-like communication.

See project page: http://roeebar.github.io/gr-plc/

# Installation

## Prerequisites

### GNU Radio

The project was developed for [GNU Radio 3.7.9](http://gnuradio.org/news/gnu-radio-v3-7-9-2-release/).

There are several ways to [install GNU Radio](http://gnuradio.org/redmine/projects/gnuradio/wiki/InstallingGR). The fastest route is using [GNU Radio Live](http://gnuradio.org/redmine/projects/gnuradio/wiki/GNURadioLiveDVD) on a USB drive with Persistence. A nice tutorial can be found here: http://gnuradio.org/blog/using-gnu-radio-live-sdr-environment/


### IT++

gr-plc library depends on [IT++ library](http://itpp.sourceforge.net/) for the turbo encoder/decoder. GNU Radio Live installation already includes this library. Otherwise, see [IT++ installation documentation](http://itpp.sourceforge.net/4.3.1/installation.html).


## Build and Install

For building and installing gr-plc run the following:


    git clone git://github.com/roeebar/gr-plc.git
    cd gr-plc
    mkdir build  
    cd build  
    cmake ../  
    make  
    sudo make install  
    sudo ldconfig  

### PHY Block

The physical layer is encapsulated in a hierarchical block to allow for a clearer transceiver structure in GNU Radio Companion. This hierarchical block is not included in the installation process. You have to open */examples/phy_hier.grc* with GNU Radio Companion and build it. This will install the block in *~/.grc_gnuradio/*.

### Check message port connections

Sometime the connections between the message ports (the gray ones in GNU Radio Companion) break. Therefore, please open the flow graphs and assert that everything is connected. It should be pretty obvious how the blocks are supposed to be wired. 

### Notes

- To disable thread priority UHD warning, add the following to */etc/security/limits.conf*  

        @<group> - rtprio 99  

  Where *&lt;group>* should be replaced with the user group

- For a 'debug' build, replace the cmake above with:

        cmake ../ -DCMAKE_BUILD_TYPE="Debug"
