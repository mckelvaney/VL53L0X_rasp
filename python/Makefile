
TARGET=libvl53l0x.a

all: lib/${TARGET}

lib/${TARGET}: ../bin/${TARGET}
	cp -f $< $@

../bin/${TARGET}:
	make -C ../

