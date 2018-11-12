# icestorm riscv32i build chain for ice40up5k
FROM ubuntu:trusty
MAINTAINER Sven Braun <s.braun@mobileonly.com>
#EXPOSE 80 3306
RUN sudo apt-get update  && apt-get -y install  build-essential autoconf automake autotools-dev clang bison flex libreadline-dev libtool \
patchutils bc libexpat1-dev zlib1g-dev gawk tcl-dev libffi-dev git libmpc-dev libmpfr-dev libgmp-dev gawk texinfo gperf libtool mercurial \
graphviz xdot pkg-config python python3 libftdi-dev php5-cli curl gcc-arm-none-eabi  gdb-arm-none-eabi telnet && \
cd /root && git clone https://github.com/cliffordwolf/icestorm.git icestorm && cd icestorm && make -j$(nproc) && make install && \
cd /root && git clone https://github.com/cseed/arachne-pnr.git arachne-pnr && cd arachne-pnr && make -j$(nproc) && make install  && \
cd /root && git clone https://github.com/cliffordwolf/yosys.git yosys && cd yosys && make -j$(nproc) && make install && \
cd /root && git clone https://github.com/cliffordwolf/icotools.git icotools && cd icotools && cd /root && \
rm -Rf /root/icestorm && \
rm -Rf /root/yosys && \
rm -Rf /root/arachne-pnr


RUN sudo mkdir /opt/riscv32i  && cd /root  && \
git clone https://github.com/riscv/riscv-gnu-toolchain riscv-gnu-toolchain-rv32i && \
cd riscv-gnu-toolchain-rv32i && \
git checkout c3ad555 && \
git submodule update --init --recursive && \
mkdir build; cd build && \
../configure --with-arch=rv32i --prefix=/opt/riscv32i && \
make -j$(nproc) && \
cd /root && \
rm -Rf /root/riscv-gnu-toolchain-rv32i  && \
echo export PATH='/opt/riscv32i/bin:$PATH' >> ~/.bash_profile





#CMD  /etc/init.d/mysql start && apachectl -DFOREGROUND
