
# -*- mode: makefile-gmake; -*-
# $Id: Makefile 1258 2017-10-17 06:57:14Z wimalopaan $

ASCIIDOC = asciidoc
ASCIIDOCTOR = asciidoctor

ASCIIDOCTORPDF = ~/.gem/ruby/2.7.0/gems/asciidoctor-pdf-1.5.4/bin/asciidoctor-pdf

EXTENSIONSDIR = ~/asciidoctor/extensions/lib 
EXTENSIONSDIR2 = ~/asciidoctor/local

ASCIIDOCTORFLAGS += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -r view-result-docinfoprocessor -r man-inline-macro-local -r asciidoctor-diagram -r tree-block-macro 
#-r asciidoctor-mathematical # -r mathematical-treeprocessor -r mathoid-treeprocessor

ASCIIDOCTORFLAGSINC += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -r include-file-extractor -r view-result-docinfoprocessor -r man-inline-macro-local -r asciidoctor-diagram -r tree-block-macro -r asciidoctor-mathematical # -r mathematical-treeprocessor -r mathoid-treeprocessor

ASCIIDOCTORFLAGSPDF += -v -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -a allow-uri-read -r asciidoctor-diagram -r tree-block-macro -r asciidoctor-mathematical
ASCIIDOCTORFLAGSPDF += -a pdf-themesdir=themes -a pdf-theme=basic.yml -a mathematical-format=svg --trace
#ASCIIDOCTORFLAGSPDF += -a pdf-stylesdir=themes -a pdf-style=basic.yml -a mathematical-format=svg --trace

ASCIIDOCTORFLAGSPDF2 += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -a allow-uri-read -r asciidoctor-diagram -r asciidoctor-mathematical -r tree-block-macro 
ASCIIDOCTORFLAGSPDF2 += -a pdf-stylesdir=themes -a pdf-style=basic_land.yml -a mathematical-format=svg --trace

ASCIIDOCTORVARS += -a data-uri -a imagesdir=`pwd`/images -a stylesdir=css # -a stylesheet=asciidoctor-wide.css
ASCIIDOCTORVARS += -a revdate="${shell date +'%x'}"
#ASCIIDOCTORVARS += -a revnumber=0.8 -a revdate="${shell date +'%x'}"


NUP = pdfnup
NUPFLAGSL += -q --nup 1x2 --no-landscape --keepinfo --frame true
NUPFLAGSP += -q --nup 2x1 --keepinfo --frame true

# convert -filter Cubic -resize 300 pcb24.jpg pcb24_s.jpg

# all: images html pdf src # check
all: html pdf 

images:
	make -C images images

sources += rcmultiswitch.adoc rcmultiswitch_h.adoc

.PHONY: html src pdf images clean

test: test3.html 

HTML += $(patsubst %.adoc, %.html, $(sources))

html: $(HTML) 

HTMLCHECK += $(patsubst %.html, %.htmlcheck, $(HTML))

check: $(HTMLCHECK)

%.htmlcheck: %.html
	linkchecker -q $< > $@ 2>&1

PDF += $(patsubst %.adoc, %_r.pdf, $(sources))

pdf: $(PDF) # images


%.source_list: %.adoc
	grep '^include::{.*}' $< | grep -v aout | sed 's/include::.*\}\///g' | sed -E 's/(cc|S|s|h|c)\..*\[\]/\1/g' | sort | uniq >$@

%_p.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGS) $(ASCIIDOCTORVARS) -a deckjs -a split -T $(HOME)/asciidoctor/asciidoctor-backends/haml/deckjs -o $@ $<

%_r.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGS) $(ASCIIDOCTORVARS) -a revealjs -a split -b revealjs -T $(HOME)/asciidoctor/asciidoctor-reveal.js/templates/slim -o $@ $<

%.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGS) $(ASCIIDOCTORVARS) -a single -o $@ $<

%_t.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGS) $(ASCIIDOCTORVARS) -a single -a 'toc2!' -a toc -o $@ $<

%.xml: %.adoc
	$(ASCIIDOCTOR) -b docbook5 -a print -r asciidoctor-diagram -o $@ $<

%.foc: %.xml
	xsltproc --nonet $(HOME)/asciidoctor/xsl/codeblocks.xsl $< > $@

%.fo: %.foc
	java -classpath /usr/share/java/docbook-xsl-saxon/docbook-xsl-saxon.jar:/usr/share/java/saxon6/saxon.jar:/usr/local/share/java/xslthl-2.1.0.jar -Dxslthl.config=file:///usr/local/share/java/highlighters/xslthl-config.xml com.icl.saxon.StyleSheet -o $@ $< $(HOME)/asciidoctor/xsl/fo.xsl

%_draft.fo: %.foc
	java -classpath /usr/share/java/docbook-xsl-saxon/docbook-xsl-saxon.jar:/usr/share/java/saxon6/saxon.jar:/usr/local/share/java/xslthl-2.1.0.jar -Dxslthl.config=file:///usr/local/share/java/highlighters/xslthl-config.xml com.icl.saxon.StyleSheet -o $@ $< $(HOME)/asciidoctor/xsl/fo-simple.xsl

%.pdf: %.fo
	fop $< $@

%_r.pdf: %.adoc
	$(ASCIIDOCTORPDF) $(ASCIIDOCTORFLAGSPDF) $(ASCIIDOCTORVARS) -a pagenums -o $@ $<

%_r2.pdf: %_r.pdf 
	$(NUP) $(NUPFLAGSP) -o $@ $<

%_rl.pdf: %.adoc
	$(ASCIIDOCTORPDF) $(ASCIIDOCTORFLAGSPDF2) $(ASCIIDOCTORVARS) -a pagenums -o $@ $<

%_rl2.pdf: %_rl.pdf 
	$(NUP) $(NUPFLAGSL) -o $@ $<

%_w.pdf: %.html
	$(WK) $(WKFLAGS) $(WKLANDSCAPE) $< $@

%_w2.pdf: %_w.pdf 
	$(NUP) $(NUPFLAGSL) -o $@ $<

%_wp.pdf: %.html
	$(WK) $(WKFLAGS) $(WKPORTRAIT) $< $@

%_wp2.pdf: %_wp.pdf 
	$(NUP) $(NUPFLAGSP) -o $@ $<

clean:
	$(RM) *~
	$(RM) $(HTML)
	$(RM) $(PDF)
	$(RM) *.pdfmarks
	$(RM) *.htmlcheck
	$(RM) *.fo
	$(RM) *.foc
	$(RM) *.xml
	$(RM) diag*.png
	$(RM) diag*.png.cache
	$(RM) diag*.svg
	$(RM) diag*.svg.cache

