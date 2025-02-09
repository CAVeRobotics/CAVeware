# Extract MCU packages
FROM alpine:latest AS mcu-package-extract
RUN apk -U upgrade
RUN apk add git
WORKDIR /root/STM32Cube/Repository
RUN git clone --recursive --depth 1 --branch v1.28.1 https://github.com/STMicroelectronics/STM32CubeF4.git

# Copy MCU packages to STM32CubeMX container
FROM d3lta12/stm32cubemx AS stm32cubemx
COPY --from=mcu-package-extract /root/STM32Cube/Repository/STM32CubeF4 /root/STM32Cube/Repository/STM32CubeF4