.PHONY: submodule-init submodule-update submodule-build submodule-clean
.PHONY: help clean install ci pip-deps test test-unit test-integration docs

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  submodule-init        to initialize the submodules"
	@echo "  submodule-update      to update the submodules"
	@echo "  submodule-build       to build the submodules"
	@echo "  submodule-clean       to clean the submodules"
	@echo "  clean                 to clean the project"
	@echo "  install               to install the project"
	@echo "  ci                    to run the CI"
	@echo "  pip-deps              to install the pip dependencies"
	@echo "  test                  to run the tests"
	@echo "  test-unit             to run the unit tests"
	@echo "  test-integration      to run the integration tests"
	@echo "  docs                  to build the documentation"

clean:
	rm -rf build
	rm -rf pteranodon.egg-info
	rm -rf .pytest_cache

install:
	pip3 install .

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

test: test-unit test-integration

test-unit:
	./scripts/tests/run_unit_tests.sh

test-integration:
	./scripts/tests/run_integration_tests.sh

docs:
	python3 -m pip install -r requirements-docs.txt -q
	$(MAKE) -C docs dirhtml
