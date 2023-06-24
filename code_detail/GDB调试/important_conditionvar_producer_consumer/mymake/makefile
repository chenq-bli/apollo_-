CC=g++
CFLAG_o=-Wall 
CFlag_dbg=-g
CFLAG_ALIAS=-o

INCLUDEPATH_COMMAND=-I
LIBPATH_COMMAND=-L
LIB_NAME=-l
RM_COMMAND= rm -rf


all:main

main:libmyfun.a
	$(CC) $(CFLAG_o) $(CFlag_dbg) ../src/main.cpp $(INCLUDEPATH_COMMAND)../libs/firstlib/include $(LIBPATH_COMMAND)./lib $(LIB_NAME)myfun $(LIB_NAME)pthread $(CFLAG_ALIAS)  ./exe/mymainexe

libmyfun.a:myfun.o
	ar rs ./lib/libmyfun.a ./lib/myfun.o

myfun.o: 
	$(CC) -c ../libs/firstlib/src/myfun.cpp  $(INCLUDEPATH_COMMAND)../libs/firstlib/include $(CFLAG_ALIAS) ./lib/myfun.o

clean:
	$(RM_COMMAND) ./lib/myfun.o ./lib/libmyfun.a ./exe/mymainexe

