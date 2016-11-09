
all: doc example host test

.PHONY: test example clean host doc

test:
	make -C test all

host:
	make -C host all

example:
	make -C example all

doc:
	make -C doc all

clean:
	make -C doc clean
	make -C example clean
	make -C test clean
	make -C host clean
