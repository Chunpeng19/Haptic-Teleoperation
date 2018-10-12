# project structure
TOP_DIR = .
BIN_DIR = $(TOP_DIR)/bin
include Makefile.common

# build dependencies

# build rules

all: $(TOP_DIR)

.PHONY: $(TOP_DIR)
$(TOP_DIR):
	$(MAKE) -C $@

clean:
	@for T in $(TOP_DIR); do make -C $$T $@; done

