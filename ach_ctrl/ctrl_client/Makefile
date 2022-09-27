default: all

CFLAGS := -I./../../include  -Wall
CC := g++

BINARIES := test
all : $(BINARIES)

LIBS := -lrt -lstdc++ -lach

$(BINARIES): src/test.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

