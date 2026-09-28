# me500emu - Mimaki ME-500 controller emulator
#   make            venv + C core + ROM check
#   make test       quick tests (skipped parts need a ROM image, see roms/README.md)
#   make test-all   includes the slow boot / job tests (a cold boot takes ~1-2 min, then cached)
#   make ui         browser UI on http://127.0.0.1:8000
#   make clean      remove the C core build and caches in the source tree (not the user cache)

PYTHON ?= python3
VENV   ?= .venv
PY      = $(VENV)/bin/python
PORT   ?= 8000
ROM    ?=
ROMARG  = $(if $(ROM),--rom $(ROM))

export PYTHONPATH := $(CURDIR)/src

.PHONY: all venv core check-rom info boot test test-all ui bridge clean distclean

all: venv core info

$(VENV)/.done: requirements.txt requirements-dev.txt
	$(PYTHON) -m venv $(VENV)
	$(VENV)/bin/pip install --upgrade pip
	$(VENV)/bin/pip install -r requirements-dev.txt
	touch $@

venv: $(VENV)/.done

core: venv
	$(PY) -c "from me500emu import fastcore; print('C core:', fastcore.build())"

info: venv
	$(PY) -m me500emu info

boot: core
	$(PY) -m me500emu boot $(ROMARG)

test: core
	$(PY) -m pytest -q -m "not slow"

test-all: core
	$(PY) -m pytest -q

ui: core
	$(PY) -m me500emu ui --port $(PORT) $(ROMARG)

bridge: core
	$(PY) -m me500emu bridge $(ROMARG)

clean:
	rm -f src/me500emu/libfastcore.so
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
	rm -rf .pytest_cache

distclean: clean
	rm -rf $(VENV)
