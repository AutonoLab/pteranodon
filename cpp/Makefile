.PHONY: all clean

all:
	mkdir -p build && cd build && cmake .. && make

clean:
	rm -rf build

install:
	cd build && sudo make install

uninstall:
	sudo rm /usr/local/lib/libpteranodon_ext.so
	sudo rm /usr/local/include/pteranodon_ext
	sudo rm -rf /usr/local/include/headers
