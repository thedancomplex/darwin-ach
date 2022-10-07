default: all

CFLAGS := -I./../../include  -Wall
CC := g++

BINARIES := darwin-ach-timeout
all : $(BINARIES)

LIBS := -lrt -lstdc++ -lach

$(BINARIES): src/main.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

