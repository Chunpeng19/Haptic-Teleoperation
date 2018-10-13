# import settings
TOP_DIR = ..
include $(TOP_DIR)/Makefile.common

# haptic examples
SUBDIRS += wavesimple


all: $(SUBDIRS)

.PHONY: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
