default: all

CFLAGS := -I/etc/darwin-legacy/include -Wall -DLINUX -D_GNU_SOURCE -Wall -m32 -O2 -O3 -g

CC := g++

BINARIES := test
all : $(BINARIES)

LIBS := -lrt -ldxl_x86_cpp -lstdc++

$(BINARIES): src/test.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

