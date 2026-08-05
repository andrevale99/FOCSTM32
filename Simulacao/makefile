GCC = gcc
MAIN = c/main.c

LIBS = -I./c/lib/bldc \
    -I./c/lib/inverter \
    -I./c/lib/PIcontroller \
    -I./c/lib/svpwm \
    -I./c/lib/transforms \
    -I./c/lib/VFstartup \

INCLUDE = -I./c/include

ALL_INCLUDE = $(LIBS) $(INCLUDE)

OUT = c/simulation.out
OUT_IMGS = img/

GCC_FLAGS = -Wall -lm

SRC := $(shell find c -name "*.c")

bldc_1 = motors/bldc_1.txt
bldc_1_file_csv = bldc_1_simulation.csv

bldc_2 = motors/bldc_2.txt
bldc_2_file_csv = bldc_2_simulation.csv

bldc_a2212 = motors/a2212.txt
bldc_a2221_file_csv = 2221_simulation.csv

bldc_kumar = motors/bldc_kumar.txt
bldc_kumar_file_csv = kumar_simulation.csv

clear:
	rm *.csv
	rm $(OUT_IMGS)*.pdf

bldc_1_closedloop:
	$(GCC) $(SRC) $(LIBS) -o $(OUT) $(GCC_FLAGS) $(ALL_INCLUDE)
	./$(OUT) -c $(bldc_1) -f $(bldc_1_file_csv)
	python plot.py $(bldc_1_file_csv) img

bldc_2_closedloop:
	$(GCC) $(SRC) -o $(OUT) $(GCC_FLAGS) $(ALL_INCLUDE)
	./$(OUT) -c $(bldc_2) -f $(bldc_2_file_csv)
	python plot.py $(bldc_2_file_csv) img

bldc_a2221_closedloop:
	$(GCC) $(SRC) -o $(OUT) $(GCC_FLAGS) $(ALL_INCLUDE)
	./$(OUT) -c $(bldc_a2212) -f $(bldc_a2221_file_csv)
	python plot.py $(bldc_a2221_file_csv) img

bldc_kumar_closedloop:
	$(GCC) $(SRC) $(LIBS) -o $(OUT) $(GCC_FLAGS) $(ALL_INCLUDE)
	./$(OUT) -c $(bldc_kumar) -f $(bldc_kumar_file_csv)
	python plot.py $(bldc_kumar_file_csv) img