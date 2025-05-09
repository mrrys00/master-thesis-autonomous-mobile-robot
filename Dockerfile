FROM ubuntu:22.04

# Disable unnecessary suggestions and recommendations in APT
RUN echo 'APT::Install-Suggests "0";' >> /etc/apt/apt.conf.d/00-docker
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/00-docker

# Set Tz
ENV TZ=Europe/Warsaw \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
apt-get install tzdata

# Install necessary packages including git, make, sudo, python3, and other dependencies
RUN DEBIAN_FRONTEND=noninteractive \
  apt-get update && \
  apt-get install -y python3 sudo git make curl software-properties-common tcl && \
  rm -rf /var/lib/apt/lists/*

# Create a user and set up sudo access
RUN useradd -ms /bin/bash apprunner && \
    echo "apprunner:12345678" | chpasswd && \
    usermod -aG sudo apprunner

# Allow root login directly (optional)
RUN echo "root:12345678" | chpasswd

# Set the working directory
WORKDIR /workspace

# Copy the Makefile into the container
COPY Makefile /workspace/Makefile

# Set the sudo password environment variable for non-interactive use
ENV SUDO_PASSWORD=12345678

# Run the `prepare_pc` target during the build process
# RUN make prepare_pc

# Switch to the created user (if needed)
USER apprunner
