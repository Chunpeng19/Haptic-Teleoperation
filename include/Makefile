# dependency
LDLIBS += -ldrd

# project layout
TOP_DIR = ..
include $(TOP_DIR)/Makefile.common

# local configuration
SRC_DIR   = .
OBJ_DIR   = .

# definitions
SOURCES   = $(wildcard $(SRC_DIR)/*.c)
INCLUDES  = $(wildcard $(INC_DIR)/*.h)
OBJECTS   = $(patsubst %.c, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
NAME      = $(notdir $(shell pwd)) 
TARGET    = $(BIN_DIR)/$(NAME)

# rules
all: $(TARGET)

$(OBJECTS): $(SOURCES) $(INCLUDES)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) *~
	-rm -rf $(OBJ_DIR)
	
