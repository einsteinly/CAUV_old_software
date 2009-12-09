.PHONY: all
all: cmt

.PHONY: cmt
cmt:
	@echo === Building cmt ===
	@$(MAKE) -C xsens
	@cp "xsens/libcmt.a" "../$(OBJDIR)/"
	@echo === Done building cmt ===

.PHONY: clean
clean:
	@$(MAKE) -C xsens clean
