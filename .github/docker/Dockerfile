ARG BASE_IMAGE=gcc:13-bookworm
FROM $BASE_IMAGE

RUN set -xe \
    && apt-get update \
    && apt-get install -y cmake

RUN set -xe \
    && cd /tmp \
    && wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz \
    && tar xzf eigen-3.4.0.tar.gz \
    && cd eigen-3.4.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install \
    && cd ../.. \
    && rm -rf eigen-3.4.0.tar.gz eigen-3.4.0
