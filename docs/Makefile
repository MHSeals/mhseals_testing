# Build math.pdf. Without a local latexmk, `make pdf` falls back to `make docker-pdf`.
TEXLIVE ?= texlive/texlive:latest-medium

pdf:
	@if command -v latexmk >/dev/null; then latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex; \
	else $(MAKE) docker-pdf; fi

docker-pdf:
	docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TEXLIVE) \
		latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex

clean:
	latexmk -C math.tex

.PHONY: pdf docker-pdf clean
