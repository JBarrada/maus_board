LIBS=-lm -pthread
OPTIONS=-O2 -Wno-psabi -std=c++17

clean:
	rm -f *.o 

%.o: %.cpp
	g++ -c $< $(LIBS) $(OPTIONS) -o $@

example: clean maus_board.o fhl_ld19.o example.cpp
	g++ example.cpp maus_board.o fhl_ld19.o $(LIBS) $(OPTIONS) -o $@