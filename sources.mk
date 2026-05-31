SRC_DIR = sources
INC_DIR = includes

SRC = \
    $(SRC_DIR)/STM32_startup.c      \
    $(SRC_DIR)/system_stm32f4xx.c   \
    $(SRC_DIR)/GPIO.c               \
    $(SRC_DIR)/TIM.c                \
    $(SRC_DIR)/Timer.c              \
    $(SRC_DIR)/PWM.c                \
    $(SRC_DIR)/I2C.c                \
    $(SRC_DIR)/Drone.c              \
    $(SRC_DIR)/UART.c               \
    $(SRC_DIR)/gyro_filter.c        \
    $(SRC_DIR)/imu_mahony.c         \
    $(SRC_DIR)/pid_controller.c     \
    $(SRC_DIR)/math_impl.c     		\
	   $(SRC_DIR)/mpu6050.c     	\
    $(SRC_DIR)/main.c

INCLUDES = \
    -I$(INC_DIR)                    \
    -ICMSIS/Core/include            \
    -ICMSIS/STM32F4xx/include
