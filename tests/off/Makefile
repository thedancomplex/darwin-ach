default: all

CFLAGS := -I/etc/darwin-legacy/include -Wall  
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

