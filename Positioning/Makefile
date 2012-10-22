CC=g++
#CC=arm-linux-g++
CFLAGS='-I./include'
CFLAGS+='-Wall'
CFLAGS+='-g'
#CFLAGS+='-DMY_DEBUG'
obj=cal_coord.o Mymath.o LSQ.o EKF.o newekf.o ReceiverFilter.o NodePosition.o
%.o: %.cpp
	$(CC) $(CFLAGS) -c $<
cal:$(obj)
	$(CC) -o $@ $(obj) 
	rm -f *.o
.PHONY clean:
	rm -f *.exe
	rm -f *.o
	rm -f *~
	rm -f ./include/*~
