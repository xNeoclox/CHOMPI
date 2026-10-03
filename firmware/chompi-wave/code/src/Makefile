# CHOMPI Endless v0.1
TARGET = CHOMPI

DEBUG=0
OPT=-O3

APP_TYPE=BOOT_SRAM
BOOT_BIN = ../../bin/CHOMPI_Bootloader_V6_2_0.bin
LDSCRIPT = chompi_sram.lds

CPP_SOURCES = chompi_main.cpp \
	encoder.cpp

LIBDAISY_DIR = ../libs/libDaisy
DAISYSP_DIR = ../libs/DaisySP

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
