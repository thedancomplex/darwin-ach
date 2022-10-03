default: all

CFLAGS := -I./../../include -I/usr/local/include/dynamixel_sdk -I/usr/include/x86_64-linux-gnu/ -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g 
CC := g++

BINARIES := darwin-server
all : $(BINARIES)

LIBS := -lrt -lstdc++ -lach 

THE_ARCH := $(shell arch)

ifeq ($(THE_ARCH),i686)
	LIBS += -ldxl_x86_cpp
else
ifeq ($(THE_ARCH),x86_64)
	LIBS += -ldxl_x86_64_cpp
else
	LIBS += -ldxl_sbc_cpp
endif
endif


#LIBS := -lrt -ldxl_x86_cpp -lstdc++ -lach -ldxl_x86_64_cpp -ldxl_sbc_cpp

$(BINARIES): src/main.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

