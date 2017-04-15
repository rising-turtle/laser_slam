SRC  := $(wildcard *.cpp) 
OBJ  := $(SRC:%.cpp=%.o)
CFLAGS := -g -Wall
all:$(OBJ) move
$(OBJ):%.o :%.cpp
	g++ $(CFLAGS) -c $^ -o $@
move:
	mv $(OBJ) ../

.PHONY : clean
clean:
	rm *.o
