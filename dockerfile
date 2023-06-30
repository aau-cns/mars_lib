FROM ubuntu:22.04

LABEL maintainer="Christian Brommer <christian.brommer@aau.at>"
LABEL description="Mars Test Environment"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    g++ \
    build-essential \
    cmake \
    libgtest-dev \
    git \
    doxygen \
    graphviz \
    && rm -rf /var/lib/apt/lists/*

COPY ./deploy/scripts/docker_application_test.sh /

CMD /docker_application_test.sh
