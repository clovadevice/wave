
###########################
# Chisyou
# make <target#>
# ex. make main or make
# ex. make clean
#
# = ; := ; ?= ; +=
#
#
############################
#CC := gcc  # for target x86
#CC := arm-linux-gnueabi-gcc  #for target arm
CFLAGS := -Wall -std=c99

INC := -I${PWD}/include \

LIB := -L/usr/lib/ # include libraries directory
DEP := -lpthread -lstdc++ -lm -ldl -lrt #: -l to include library libpthread
SRC = ${PWD}/src
.SUFFIXS : .c .cpp
######################################
CHARGEMODE_OBJ = chargemode.o util.o
######################################
# make TARGET<n>
#
#TARGET1 : dependencies1
#<TAB>command;
#TARGET2 : dependencies2
#<TAB>command;
# ......
# * all command lines start from <TAB>
##########################################
chargemode : ${CHARGEMODE_OBJ}
	${CC} ${CHARGEMODE_OBJ} ${DEP} -o $@
###############################################
# $@ means "TARGET"
# same as ${CC} main.o foo.o ${CFLAGS} -o main
#################################################
#main.o : ${SRC}/main.c
#	${CC} ${SRC}/main.c ${CFLAGS} ${INC} ${DEP} -c

#libqci_lib.a : ${SRC}/qci_lib.c
#	${CC} -static ${SRC}/qci_lib.c ${INC} ${CFLAGS} ${DEP}
################################################################
#target main.o and foo.o section can be replaced by below section.
#this is very useful if we have many target to be build.
# $< : means the dependency files
################################################################
%.o : ${SRC}/%.c
	${CC} $< ${CFLAGS} ${INC} ${DEP} -c

# $(SHARED_OBJ): $(addprefix $(SHARED_DIR)/,audio.h console.h datalocations.h)
$(SHARED_OBJ): %.o: %.c
	$(CC) -c $(CFLAGS) ${INC} ${DEP} $< -o $@

.PHONY : clean
clean :
	@rm *.a *.o chargemode
#############################################
#@ means to hide from stdout while executing
#############################################