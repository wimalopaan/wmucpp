
all: example host

clean: example host

.PHONY: test example clean host

test:
	make -C test all

host:
	make -C host all

example:
	make -C example all

clean:
	make -C example clean
	make -C test clean
	make -C host clean
