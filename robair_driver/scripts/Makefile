SHELL := /bin/bash

# these files should pass pyflakes
# exclude ./env/, which may contain virtualenv packages
PYFLAKES_WHITELIST=$(shell find . -name "*.py" ! -path "./build/*" \
                    ! -path "./env/*" ! -path "./.tox/*")

# Local development management
clean:
	find -L . -name "*~" -exec rm -fr {} \;
	find -L . -name "*.pyc" -exec rm -fr {} \;

pyflakes:
	pyflakes ${PYFLAKES_WHITELIST}

pep:
	pep8 website

virtualenv: requirements.txt
	virtualenv env --python=python2.7
	./env/bin/pip install -r requirements.txt
