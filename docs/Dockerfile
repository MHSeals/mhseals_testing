# TeX Live for docs/: the medium scheme lacks pgfplots, which the plots in math.pdf and tuning.pdf use.
FROM texlive/texlive:latest-medium
RUN tlmgr update --self && tlmgr install pgfplots
