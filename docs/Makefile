# Build math.pdf. Without a local latexmk, `make pdf` builds in the TeX Live image (Dockerfile: medium + pgfplots).
TEXIMAGE ?= mhseals-docs-texlive
LATEXMK := latexmk -pdf -interaction=nonstopmode -halt-on-error

pdf:
	@if command -v latexmk >/dev/null; then $(LATEXMK) math.tex; else $(MAKE) docker-pdf; fi

docker-pdf: docker-image
	docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TEXIMAGE) $(LATEXMK) math.tex

docker-image:
	docker build -q -t $(TEXIMAGE) . >/dev/null

# Redraw the README screenshots (docs/screens/*.svg: image tiers and the braille fallback) and the README table of them
# from synthetic data, no ROS or terminal; test/test_screens.py fails when either is stale.
screens:
	cd .. && PYTHONPATH=.:../mhseals_hardware:$$PYTHONPATH python3 docs/screens/render.py

tuning:
	@if command -v latexmk >/dev/null; then $(LATEXMK) tuning.tex; \
	else $(MAKE) docker-image && docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TEXIMAGE) \
		$(LATEXMK) tuning.tex; fi

# Regenerate the committed figure data from the frozen study rows (stdlib only, deterministic).
tuning-figures:
	python3 -I figures/tuning/make_all.py

clean:
	latexmk -C math.tex
	latexmk -C tuning.tex

.PHONY: pdf docker-pdf docker-image screens tuning tuning-figures clean
