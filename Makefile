# project structure
TOP_DIR = .
BIN_DIR = $(TOP_DIR)/bin
include Makefile.common

# build dependencies
SUBDIRS = examples

# build rules

all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done

