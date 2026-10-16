# Build math.pdf. `make docker-pdf` needs only Docker.
TEXLIVE ?= texlive/texlive:latest-medium

pdf:
	latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex

docker-pdf:
	docker run --rm -u "$$(id -u):$$(id -g)" -v "$(CURDIR):/docs" -w /docs $(TEXLIVE) \
		latexmk -pdf -interaction=nonstopmode -halt-on-error math.tex

clean:
	latexmk -C math.tex

.PHONY: pdf docker-pdf clean
