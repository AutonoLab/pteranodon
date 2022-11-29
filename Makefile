.PHONY: submodule-init submodule-update submodule-build submodule-clean submodule-clone
.PHONY: help 
.PHONY: clean clean-all 
.PHONY: install install-px4-prereqs
.PHONY: build-all 
.PHONY: ci 
.PHONY: pip-deps 
.PHONY: test test-unit test-integration test-examples 
.PHONY: docs

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  submodule-init        to initialize the submodules"
	@echo "  submodule-update      to update the submodules"
	@echo "  submodule-build       to build the submodules"
	@echo "  submodule-clean       to clean the submodules"
	@echo "  submodule-clone       to intialize, update, and clone the submodules"
	@echo "  clean                 to clean the project"
	@echo "  clean-all             to clean the project and submodules"
	@echo "  install               to install the project"
	@echo "  install-px4-prereqs   to install the px4 prerequisites"
	@echo "  build-all             to build the project and all submodules"
	@echo "  ci                    to run the CI"
	@echo "  pip-deps              to install the pip dependencies"
	@echo "  test                  to run the tests"
	@echo "  test-unit             to run the unit tests"
	@echo "  test-integration      to run the integration tests"
	@echo "  test-examples         to run the examples tests"
	@echo "  docs                  to build the documentation"

clean:
	rm -rf build
	rm -rf pteranodon.egg-info
	rm -rf .pytest_cache

clean-all: clean submodule-clean

install:
	pip3 install .

install-px4-prereqs: 
	./third-party/px4-autopilot/Tools/setup/ubuntu.sh

build-all: submodule-clone install-px4-prereqs submodule-build pip-deps install

submodule-init:
	git submodule init
	git pull --recurse-submodules

submodule-update:
	git submodule update --init --recursive

submodule-clone: submodule-init submodule-update

submodule-build:
	$(MAKE) -C third-party

submodule-clean:
	$(MAKE) -C third-party clean

pip-deps:
	./scripts/tools/install_pip_dependencies.sh

ci:
	python3 -m pip install -r requirements-dev.txt -q
	./scripts/tools/run_ci.sh

test: test-unit test-integration

test-unit:
	./scripts/tests/run_unit_tests.sh

test-integration:
	./scripts/tests/run_integration_tests.sh

test-examples:
	./scripts/tests/run_example_tests.sh

docs:
	python3 -m pip install -r requirements-docs.txt -q
	$(MAKE) -C docs dirhtml
