# Update base image and install tools, dependencies
FROM d3lta12/alpine-build-tools:latest AS base
RUN apk -U upgrade

# Install GNU Compiler Collection for arm-none-eabi targets
RUN apk add gcc-arm-none-eabi g++-arm-none-eabi binutils-arm-none-eabi github-cli

# Cleanup
WORKDIR /root
RUN apk cache clean
RUN rm -rf /tmp/* /var/tmp/*