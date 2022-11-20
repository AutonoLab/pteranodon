.PHONY: submodule-init submodule-update submodule-build

# call the init target in third-party/Makefile
submodule-init:
	$(MAKE) -C third-party init

# call the update target in third-party/Makefile
submodule-update:
	$(MAKE) -C third-party update

# call the all target in third-party/Makefile
submodule-build:
	$(MAKE) -C third-party
