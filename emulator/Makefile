OCSRC=$(wildcard ../oc/sources/*.c) $(wildcard ../oc/sources/mcufont/*.c) $(wildcard ../oc/sources/boy/*.c)
OCOBJ=$(addprefix bin/,$(notdir $(OCSRC:.c=.o)))
EMUSRC=emuivy.c
EMUOBJ=$(addprefix bin/,$(EMUSRC:.c=.o)) $(OCOBJ)
INCLUDES= $(wildcard ../common/*.h) $(wildcard ../oc/sources/*.h) $(wildcard ../oc/sources/mcufont/*.h) $(wildcard ../oc/sources/boy/*.h) $(wildcard *.h) $(wildcard ../common/diskio/*.h) $(wildcard ../common/fatfs/*.h)
#CC=gcc -O6 -c -DOCEMU -I$(CURDIR) -I$(CURDIR)/../oc/sources -I$(CURDIR)/../oc/sources/mcufont -I$(CURDIR)/../oc/sources/boy -I$(CURDIR)/../common -std=gnu99 -pthread $< -o $(addprefix bin/,$(notdir $@))
CC=gcc -g -c -DOCEMU -I$(CURDIR) -I$(CURDIR)/../oc/sources -I$(CURDIR)/../oc/sources/mcufont -I$(CURDIR)/../oc/sources/boy -I$(CURDIR)/../common -std=gnu99 -pthread $< -o $(addprefix bin/,$(notdir $@))

all: bin/emuivy

bin/emuivy: $(EMUOBJ)
	gcc $(EMUOBJ) -pthread -L/usr/local/lib -Lbin -lSDL2 -lSDL2main -lSDL2_ttf -lSDL2_net -lpthread -o bin/emuivy

$(EMUOBJ): $(INCLUDES)

bin/%.o: %.c
	$(CC)

bin/%.o: ../oc/sources/%.c
	$(CC)

bin/%.o: ../oc/sources/mcufont/%.c
	$(CC)

bin/%.o: ../oc/sources/boy/%.c
	$(CC)
