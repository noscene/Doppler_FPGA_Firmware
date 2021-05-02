# icestorm Installation
FROM ubuntu:focal
LABEL maintainer="s.braun@mobileonly.com"

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && apt-get -y install  xxd build-essential clang bison flex libreadline-dev gawk tcl-dev libffi-dev git mercurial graphviz xdot pkg-config python python3 libftdi-dev php-cli curl gcc-arm-none-eabi   telnet 
RUN cd /root  && git clone https://github.com/cliffordwolf/icestorm.git icestorm 
RUN cd /root && git clone https://github.com/cliffordwolf/icotools.git icotools && cd icotools 
RUN cd /root && cd icestorm && make -j$(nproc) && make install 
RUN cd /root && git clone https://github.com/cseed/arachne-pnr.git arachne-pnr && cd arachne-pnr && make -j$(nproc) && make install  
RUN cd /root && git clone https://github.com/cliffordwolf/yosys.git yosys && cd yosys && make -j$(nproc) && make install 


