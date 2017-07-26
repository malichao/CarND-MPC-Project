FROM ubuntu:14.04

MAINTAINER Malcolm Ma <malichaooo@gmail.com>

# Arguments
ARG user
ARG uid
ARG gid
ARG home
ARG shell

# Basic Utilities
RUN apt-get -y update &&\
	apt-get install -y sudo ssh build-essential git software-properties-common

# Install g++ 5.x
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test &&\
	apt-get update -y && apt-get install -y gcc-5 g++-5 &&\
	update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60\
	--slave /usr/bin/g++ g++ /usr/bin/g++-5

# Install cmake 3.9
RUN wget https://cmake.org/files/v3.9/cmake-3.9.0-rc5.tar.gz && \
	tar xf cmake-3.9.0-rc5.tar.gz && \
	cd cmake-3.9.0-rc5 && ./configure && make -j && make install && \
	cd .. && rm -rf cmake*

# Install dependencies for uWebSockets
RUN apt-get install -y apt-utils libssl-dev autotools-dev automake libtool

# Build libuv1 from source as it's not available on ubuntu 14
RUN wget https://launchpad.net/ubuntu/+archive/primary/+files/libuv1_1.9.1.orig.tar.gz && \
	tar xf libuv1_1.9.1.orig.tar.gz && \
	cd libuv-v1.9.1 && ./autogen.sh && ./configure && make -j && \
	make check && make install && \
	cd .. && rm -rf libuv* && ldconfig

# Install uWebSockets
RUN git clone https://github.com/uWebSockets/uWebSockets && cd uWebSockets && \
	git checkout e94b6e1 && mkdir build && cd build && cmake .. && make -j &&\
	make install && cd ../.. && ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so &&\
	rm -r uWebSockets

# Install IPOPT, step 1
ADD install_ipopt.sh .

# Install IPOPT, step 2
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.tgz &&\
	 tar xf Ipopt-3.12.7.tgz && rm Ipopt-3.12.7.tgz

# Install IPOPT, step 3, the first line is to keep the gfortran installation silent
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections &&\
	apt-get install -y gfortran pkg-config && bash install_ipopt.sh Ipopt-3.12.7 

# Install CppAD
RUN wget http://www.coin-or.org/download/source/CppAD/cppad-20170701.gpl.tgz && \
	tar xf cppad-20170701.gpl.tgz && rm cppad-20170701.gpl.tgz && ldconfig

RUN ln -s /usr/lib/x86_64-linux-gnu/libgfortran.so.3 /usr/lib/libgfortran.so &&\
	cd cppad-20170701 && mkdir build && cd build &&\
cmake 										\
    -D CMAKE_VERBOSE_MAKEFILE=YES						\
    -G "Unix Makefiles"							   	\
     \
    -D cppad_prefix=/usr/local							\
    -D cppad_postfix=""								\
     \
    -D cmake_install_includedirs=include					\
    -D cmake_install_libdirs=lib						\
     \
    -D cmake_install_datadir=share						\
    -D cmake_install_docdir=share/doc						\
    \
    -D ipopt_prefix=/usr/local/lib						\
    \
    -D cppad_cxx_flags="-Wall -ansi -pedantic-errors -std=c++11 -Wshadow"	\
    \
    -D cppad_testvector=cppad							\
    -D cppad_max_num_threads=48							\
    -D cppad_tape_id_type=size_t						\
    -D cppad_tape_addr_type=size_t						\
    -D cppad_debug_which=debug_none						\
    -D cppad_deprecated=NO		                                   \
    \
    .. && make -j4 && make -j4 check && make -j4 install

RUN rm -rf /var/lib/apt/lists/*

# Install python development file used by matplot
RUN apt-get update -y && apt-get install -y python2.7-dev python-matplotlib

# Flask Server
EXPOSE 4567

# Install Qt creator 4.2.2 for development, this one needs OpenGL !!
# The nvidia driver must match the one on the host system
RUN add-apt-repository -y ppa:beineri/opt-qt571-trusty && apt-get -y update &&\
	apt-get install -y qt57creator nvidia-375 &&\
	ln -s /opt/qt57/bin/qtcreator /usr/local/bin/qtcreator
