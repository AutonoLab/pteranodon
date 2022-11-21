.PHONY: submodule-init submodule-update submodule-build submodule-clean
.PHONY: help clean ci pip-deps test docs

help:
	@echo "submodule-init - initialize submodules"
	@echo "submodule-update - update submodules"
	@echo "submodule-build - build submodules"
	@echo "submodule-clean - clean submodules"
	@echo "clean - remove build artifacts"
	@echo "ci - run continuous integration"

clean:
	rm -rf build
	rm -rf pteranodon.egg-info

# call the init target in third-party/Makefile
submodule-init:
	$(MAKE) -C third-party init

# call the update target in third-party/Makefile
submodule-update:
	$(MAKE) -C third-party update

# call the all target in third-party/Makefile
submodule-build:
	$(MAKE) -C third-party

# call the clean target in third-party/Makefile
submodule-clean:
	$(MAKE) -C third-party clean

pip-deps:
	./scripts/install_pip_dependencies.sh

ci:
	python3 -m pip install -r requirements-dev.txt -q
	./scripts/run_ci.sh

test:
	python3 -m pip install -r requirements-test.txt -q
	./scripts/run_tests.sh

docs:
	python3 -m pip install -r requirements-docs.txt -q
	$(MAKE) -C docs dirhtml
