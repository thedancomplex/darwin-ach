default: all

CFLAGS := -I./include -I/usr/local/include/dynamixel_sdk -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g --std=gnu99
CC := gcc

BINARIES := darwind
all : $(BINARIES)

LIBS := -lrt -ldxl_x86_c

$(BINARIES): src/darwind.o
	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

