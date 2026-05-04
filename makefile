include sources.mk

BUILD_DIR = build
LINKER_DIR = linker


EXEC = $(BUILD_DIR)/app.elf

# Architectur Specific Flags
LINKER_FILE = $(LINKER_DIR)/STM32f4.ld # Linker script (ruta completa)
CPU = cortex-m4
ARCH = armv7e-m
SPECS = nosys.specs
FPU = fpv4-sp-d16

ARCHFLAGS = 		\
-mcpu=$(CPU) 		\
-mthumb 			\
-march=$(ARCH) 		\
-mfloat-abi=hard 	\
-mfpu=$(FPU) 		\
--specs=$(SPECS)


# Archivos objeto (se generarán en build/)
OBJS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRC)) # Crear lista de objetos (.o) de la lista de source (.c)

CC = arm-none-eabi-gcc

# Compiler Flags
CFLAGS = 			\
-g 					\
-O0 				\
-std=c99 			\
-Werror 			\
-Wall 				\
-DSTM32F411xE		\
$(ARCHFLAGS)	

# Linker Flags
LDFLAGS = 		\
-nostdlib 		\
-T 				\
$(LINKER_FILE)	\

# Regla para compilar .c a .o en build/
$(BUILD_DIR)/%.o : $(SRC_DIR)/%.c
	mkdir -p $(BUILD_DIR)
	$(CC) -c $< $(CFLAGS) $(INCLUDES) -o $@

# Crear directorio build si no existe
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY : build

build : $(EXEC)

$(EXEC) : $(OBJS)
	$(CC) $(OBJS) $(CFLAGS) $(INCLUDES) $(LDFLAGS) -o $@

.PHONY : flash

flash : $(EXEC)
	openocd -f board/st_nucleo_f4.cfg -c "program $(EXEC) verify reset" -c shutdown

.PHONY : clean

clean : 
	rm -f $(OBJS) $(EXEC)
	rm -rf $(BUILD_DIR)