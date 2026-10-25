# Build math.pdf. Without a local latexmk, `make pdf` falls back to `make docker-pdf`.
TEXLIVE ?= texlive/texlive:latest-medium

pdf:
	@if command -v latexmk >/dev/null; then latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex; \
	else $(MAKE) docker-pdf; fi

docker-pdf:
	docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TEXLIVE) \
		latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex

# Redraw the README screenshots (docs/screens/*.svg) from synthetic data; test/test_screens.py fails when they are stale.
screens:
	cd .. && PYTHONPATH=.:../mhseals_hardware:$$PYTHONPATH python3 docs/screens/render.py

clean:
	latexmk -C math.tex

.PHONY: pdf docker-pdf screens clean
