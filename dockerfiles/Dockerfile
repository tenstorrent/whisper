FROM ubuntu:18.04 as base

# Dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
      curl \
      ca-certificates \
      wget \
      python3 \
      g++-8 \
      make \
      libboost-all-dev

WORKDIR /root

RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 0

CMD ["/bin/bash"]
