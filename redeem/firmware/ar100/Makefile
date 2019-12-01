CC = or1k-elf-gcc

SRC=firmware_runtime.c
BIN=firmware_runtime.bin

CFLAGS=-O2 -fno-common -fno-builtin -ffreestanding
all:
	 $(CC) $(CFLAGS) -c $(SRC) -o $(BIN)