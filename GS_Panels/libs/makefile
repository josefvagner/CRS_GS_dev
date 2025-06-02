SUBDIRS := $(wildcard */.)

all: $(SUBDIRS)
$(SUBDIRS):
	@echo Building: $(name)
	$(MAKE) -C ../build/$(name) all

.PHONY: all $(SUBDIRS)
