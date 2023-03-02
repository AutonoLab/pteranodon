.PHONY: clean-pyc clean-build docs

all: update-gazebo clean install

help:
	@echo "all - Rebuilds msgs, cleans, installs"
	@echo "update-gazebo - update message definitions from Gazebo"
	@echo "install - installs package to python with pip"
	@echo "clean - runs clean-build and clean-pyc"
	@echo "clean-build - remove build artifacts"
	@echo "clean-pyc - remove Python file artifacts"
	@echo "lint - check style with flake8"
	@echo "test - run tests quickly with the default Python"
	@echo "test-all - run tests on every Python version with tox"
	@echo "coverage - check code coverage quickly with the default Python"
	@echo "docs - generate Sphinx HTML documentation, including API docs"
	@echo "release - package and upload a release"
	@echo "sdist - package"

# Locate Gazebo header installation directory.
GAZEBO_INCLUDE_DIR := \
  ${shell pkg-config gazebo --cflags 2>/dev/null | cut -d' ' -f8 | cut -d'I' -f2}

MSG_DIR := \
	pygazebo/msg/

update-gazebo:
	if [ 'z${GAZEBO_INCLUDE_DIR}' = 'z' ]; then \
    	echo "Gazebo must be installed to update message definitions"; \
    	exit 1; \
  	fi
	rm -rf ${MSG_DIR}/*_pb2.py
	for definition in $$(find ${GAZEBO_INCLUDE_DIR}/gazebo/msgs/proto -name '*.proto'); \
    do \
		protoc -I ${GAZEBO_INCLUDE_DIR}/gazebo/msgs/proto --python_out=pygazebo/msg $$definition; \
	done; \
	cd ${MSG_DIR}; \
	> __init__.py; \
	for file in $$(find . -name '*.py'); \
    do \
		protoname=`echo $$file | rev | cut -c4- | rev | cut -c3-`; \
		echo "from . import $$protoname" >> __init__.py; \
		for pyfile in $$(find . -name '*_pb2.py'); \
		do \
			LC_ALL=C sed -i "s/import $$protoname/from . import $$protoname/g" $$pyfile; \
		done \
	done

install: clean
	pip3 install .

clean: clean-build clean-pyc

clean-build:
	rm -fr build/
	rm -fr dist/
	rm -fr *.egg-info

clean-pyc:
	find . -name '*.pyc' -exec rm -f {} +
	find . -name '*.pyo' -exec rm -f {} +
	find . -name '*~' -exec rm -f {} +

lint:
	flake8 pygazebo tests --exclude msg

test:
	python3 setup.py test

test-all:
	tox

coverage:
	coverage run --source pygazebo setup.py test
	coverage report -m
	coverage html
	open htmlcov/index.html

docs:
	python3 generate_msg_docs.py > docs/pygazebo.msg.rst
	sphinx-apidoc -o docs/ pygazebo
	$(MAKE) -C docs clean
	$(MAKE) -C docs html
	xdg-open docs/_build/html/index.html

release: clean
	python3 setup.py sdist upload

sdist: clean
	python3 setup.py sdist
	ls -l dist
