CXXFLAGS=-Wall -Wextra -Werror
LDLIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui

BIN=hough-lines

.PHONY: all clean cleanall
all: $(BIN)
clean:
	$(RM) *~ *.png
cleanall: clean
	$(RM) $(BIN) *.o *.pdf


