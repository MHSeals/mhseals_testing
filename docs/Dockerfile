# TeX Live for docs/: the medium scheme lacks pgfplots, which every part's plots use.
FROM texlive/texlive:latest-medium
RUN tlmgr update --self && tlmgr install pgfplots
