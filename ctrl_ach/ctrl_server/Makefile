default: all

CFLAGS := -I./../../include -I/usr/local/include/dynamixel_sdk -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g 
CC := g++

BINARIES := main
all : $(BINARIES)

LIBS := -lrt -ldxl_x86_cpp -lstdc++ -lach

$(BINARIES): src/main.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

