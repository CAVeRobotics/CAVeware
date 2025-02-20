# Extract MCU packages
FROM alpine:latest AS mcu-package-extract
RUN apk -U upgrade
RUN apk add git
WORKDIR /root/STM32Cube/Repository
RUN git clone --recursive --depth 1 --branch v1.28.1 https://github.com/STMicroelectronics/STM32CubeF4.git

# Initialize STM32CubeMX for building STM32F4 project
FROM d3lta12/stm32cubemx AS cube-initialize
COPY --from=mcu-package-extract /root/STM32Cube /root/STM32Cube
WORKDIR /tmp/project
RUN echo "project name project" > project.script
RUN echo "project path /tmp/project" >> project.script
RUN echo "load STM32F407ZGTx" >> project.script
RUN echo "project generate " >> project.script
RUN echo "exit" >> project.script
RUN /script /opt/STM32CubeMX/jre/bin/java -jar /opt/STM32CubeMX/STM32CubeMX -q project.script

# Copy MCU packages to STM32CubeMX container
FROM d3lta12/stm32cubemx AS stm32cubemx
COPY --from=cube-initialize /root/STM32Cube /root/STM32Cube
COPY --from=cube-initialize /root/.stm32cubemx /root/.stm32cubemx
COPY --from=cube-initialize /root/.stmcube /root/.stmcube
COPY --from=cube-initialize /root/.stmcufinder /root/.stmcufinder