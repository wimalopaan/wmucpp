# -*- mode: makefile-gmake; -*-
# $Id: Makefile 1280 2019-07-22 07:44:49Z wimalopaan $

ASCIIDOCTOR = asciidoctor
ASCIIDOCTORPDF = ~/.gem/ruby/3.0.0/gems/asciidoctor-pdf-2.3.9/bin/asciidoctor-pdf

EXTENSIONSDIR = ~/asciidoctor/extensions/lib 
EXTENSIONSDIR2 = ~/asciidoctor/local

ASCIIDOCTORFLAGS += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -r view-result-docinfoprocessor -r man-inline-macro-local -r asciidoctor-diagram -r tree-block-macro -r asciidoctor-mathematical # -r mathematical-treeprocessor -r mathoid-treeprocessor

ASCIIDOCTORFLAGSINC += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -r include-file-extractor -r view-result-docinfoprocessor -r man-inline-macro-local -r asciidoctor-diagram -r tree-block-macro -r asciidoctor-mathematical # -r mathematical-treeprocessor -r mathoid-treeprocessor

ASCIIDOCTORFLAGSPDF += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -a allow-uri-read -r asciidoctor-diagram -r tree-block-macro -r asciidoctor-mathematical
ASCIIDOCTORFLAGSPDF += -a pdf-themesdir=themes -a pdf-theme=wm.yml -a mathematical-format=svg --trace

ASCIIDOCTORFLAGSPDF2 += -I $(EXTENSIONSDIR) -I $(EXTENSIONSDIR2) -a allow-uri-read -r asciidoctor-diagram -r asciidoctor-mathematical -r tree-block-macro 
ASCIIDOCTORFLAGSPDF2 += -a pdf-stylesdir=themes -a pdf-style=basic_land.yml -a mathematical-format=svg --trace

ASCIIDOCTORVARS += -a data-uri -a imagesdir=`pwd`/images -a stylesdir=css # -a stylesheet=asciidoctor-wide.css
ASCIIDOCTORVARS += -a revnumber=1.0 -a revdate="${shell date +'%x'}" -a revremark="pre"

all: html pdf

sources += $(wildcard bldc*.adoc)

.PHONY: html src pdf images clean

HTML += $(patsubst %.adoc, %.html, $(sources))

html: $(HTML)

PDF += $(patsubst %.adoc, %.pdf, $(sources))

pdf: $(PDF)

%.includes: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGSINC) $(ASCIIDOCTORVARS) -D /tmp -a ife_no_adoc_output_files -a ife_filename=$*.includes $<
	
%.incsrc: %.includes
	-grep '.extractor' $< | sed 's/\/.extractor//g' | sed -E "s/\.[a-zA-Z][a-zA-Z0-9]*$$//g" > $@
	-grep -E '\.cpp$$' $< >> $@
	-grep -E '\.cc$$' $< >> $@
	-grep -E '\.c$$' $< >> $@
	-grep -E '\.h$$' $< >> $@
	sort $@ | sed -E "s/^\.\.\///g" > $@.sorted
	uniq $@.sorted > $@
	rm $@.sorted

%.incsrch: %.incsrc
	more $< | awk '{print $$0 ".html"}' > $@
	
%.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTORFLAGS) $(ASCIIDOCTORVARS) -a single -o $@ $<


%.pdf: %.adoc
	$(ASCIIDOCTORPDF) $(ASCIIDOCTORFLAGSPDF) $(ASCIIDOCTORVARS) -a pagenums -o $@ $<

clean:
	$(RM) *~
	$(RM) $(HTML)
	$(RM) $(PDF)
	$(RM) *.incsrc
	$(RM) *.pdfmarks
	$(RM) *.htmlcheck
	$(RM) *.fo
	$(RM) *.foc
	$(RM) *.xml
	$(RM) diag*.png
	$(RM) diag*.png.cache
	$(RM) diag*.svg
	$(RM) diag*.svg.cache

