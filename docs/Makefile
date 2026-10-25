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

# tuning.pdf uses pgfplots, which the medium scheme lacks: the fallback image adds it (Dockerfile).
TUNING_IMAGE ?= mhseals-docs-texlive

tuning:
	@if command -v latexmk >/dev/null; then latexmk -pdf -interaction=nonstopmode -halt-on-error tuning.tex; \
	else docker build -q -t $(TUNING_IMAGE) . >/dev/null && \
	docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TUNING_IMAGE) \
		latexmk -pdf -interaction=nonstopmode -halt-on-error tuning.tex; fi

# Regenerate the committed figure data from the frozen study rows (stdlib only, deterministic).
tuning-figures:
	python3 -I figures/tuning/make_all.py

clean:
	latexmk -C math.tex
	latexmk -C tuning.tex

.PHONY: pdf docker-pdf screens tuning tuning-figures clean
