# -*- mode: makefile-gmake; -*-
# $Id: Makefile 1258 2017-10-17 06:57:14Z wimalopaan $

ASCIIDOC = asciidoc
ASCIIDOCTOR = asciidoctor

#ASCIIDOCTORPDF = ~/asciidoctor/asciidoctor-pdf/bin/asciidoctor-pdf
ASCIIDOCTORPDF = ~/.gem/ruby/2.5.0/gems/asciidoctor-pdf-1.5.0.alpha.16/bin/asciidoctor-pdf

# rendered admonitions als image
#ASCIIDOCTORPDF = ~/asciidoctor/jessedoyle-asciidoctor-pdf/bin/asciidoctor-pdf

EXTENSIONSDIR = ~/asciidoctor/extensions/lib 

ASCIIDOCTORFLAGS += -I $(EXTENSIONSDIR) -r view-result-postprocessor -r man-inline-macro -r asciidoctor-diagram -r tree-block-macro # -r mathematical-treeprocessor -r mathoid-treeprocessor
ASCIIDOCTORFLAGSPDF += -I $(EXTENSIONSDIR) -a allow-uri-read -r asciidoctor-diagram -r tree-block-macro #-r mathoid-treeprocessor
ASCIIDOCTORFLAGSPDF += -a pdf-stylesdir=themes -a pdf-style=basic.yml --trace
ASCIIDOCTORFLAGSPDF2 += -I $(EXTENSIONSDIR) -a allow-uri-read -r asciidoctor-diagram -r tree-block-macro
ASCIIDOCTORFLAGSPDF2 += -a pdf-stylesdir=themes -a pdf-style=basic_land.yml --trace
ASCIIDOCTORVARS += -a data-uri -a imagesdir=`pwd` -a stylesdir=css -a stylesheet=asciidoctor-wide.css
ASCIIDOCTORVARS += -a revnumber=1.8 -a revdate="${shell date}" -a revremark="initial"
# ASCIIDOCTORVARS += -a srcbase=undef # to identiy all included sources

#SCREENSHOT = ~/asciidoctor/asciidoctor-screenshot/buildSrc/src/main/groovy/org/asciidoctor/extension
#ASCIIDOCTORFLAGS += -I $(SCREENSHOT) -r ScreenshotExtensionRegistry.groovy

WK = ~/Software/wkhtmltopdf/wkhtmltopdf/static-build/posix-local/wkhtmltox-0.12.3/bin/wkhtmltopdf
#WK = ~/Software/wkhtmltopdf/wkhtmltox-0.12.2.1/static-build/wkhtmltox-0.12.2.1/bin/wkhtmltopdf
WKFLAGS += --footer-line --footer-left "Prof. Dr.-Ing. W. Meier" --footer-right "[page] - [topage]"  --footer-center "[date]"
WKFLAGS += --header-line --header-left "[section]" --header-center "[subsection]" --header-right "[subsubsection]" 
WKFLAGS += --header-spacing 5 --footer-spacing 5 
WKFLAGS += --header-font-size 9 --footer-font-size 9 
WKFLAGS += -T 15 -B 15 
WKFLAGS +=  --enable-toc-back-links
WKLANDSCAPE += -O landscape 
WKPORTRAIT += -O portrait

NUP = pdfnup
NUPFLAGSL += -q --nup 1x2 --no-landscape --keepinfo --frame true
NUPFLAGSP += -q --nup 2x1 --keepinfo --frame true

# all: images html pdf src # check
all: html # pdf 

images:
	make -C images images

sources += onoff.adoc 

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

