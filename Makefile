#FQBN := "attiny:avr:ATtinyX5:cpu=attiny85,clock=internal8"
FQBN := "adafruit:samd:adafruit_qtpy_m0"
PORT ?= "/dev/ttyACM0"

.DEFAULT_GOAL := all

.PHONY: all
all: compile upload

.PHONY: install-core
install-core:
	arduino-cli core install adafruit:samd
	arduino-cli core install arduino:samd

.PHONY: compile
compile:
	arduino-cli compile --fqbn $(FQBN)

.PHONY: upload
upload:
	arduino-cli upload --fqbn $(FQBN) --port $(PORT)
