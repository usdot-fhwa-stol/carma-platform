FROM ubuntu:18.04
MAINTAINER Jens Klimke (jens.klimke@rwth-aachen.de)

# installation
RUN apt-get update
RUN apt-get -y install git g++ make cmake nano libpthread-stubs0-dev

# install googletest
RUN apt-get -y install libgtest-dev
RUN cd /usr/src/googletest/googletest && cmake CMakeLists.txt && make
RUN cd /usr/src/googletest/googletest && cp *.a /usr/lib
RUN mkdir /usr/local/lib/googletest
RUN ln -s /usr/lib/libgtest.a /usr/local/lib/googletest/libgtest.a
RUN ln -s /usr/lib/libgtest_main.a /usr/local/lib/googletest/libgtest_main.a

# copy code
COPY . /app
RUN cd /app && git submodule update --init --recursive

# installation
RUN rm -rf /app/build
RUN mkdir /app/install
RUN cd /app && mkdir build && cd build && cmake -Wno-dev -G "Unix Makefiles" \
    -DBUILD_TESTS=ON \
    -DBUILD_GTEST=OFF \
    -DCMAKE_INSTALL_PREFIX=/app/install \
    -DCMAKE_BUILD_TYPE=Debug ..

# documentation, compilation, tests
RUN cd /app/build && make
RUN cd /app/build && make test
RUN cd /app/build && make install

CMD bash