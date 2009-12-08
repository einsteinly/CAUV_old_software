.PHONY: all
all: cmt

.PHONY: cmt
cmt:
	@$(ECHO) === Building cmt ===
	@$(MAKE) -C xsens
	@cp "xsens/libcmt.a" "../$(OBJDIR)/"

.PHONY: clean
clean:
	@$(MAKE) -C xsens clean
