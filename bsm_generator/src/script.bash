cd ~/ && \
    curl -L  https://download.libsodium.org/libsodium/releases/libsodium-1.0.18-stable.tar.gz > libsodium-1.0.18-stable.tar.gz && \
    tar -xvf libsodium-1.0.18-stable.tar.gz && \
    cd libsodium-stable && \
    ./configure && \
    make && \
    sudo make install && \
    sudo ldconfig && \
    cd ../