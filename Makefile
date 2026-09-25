EE_BIN = compat_layer.elf
EE_OBJS = main.o traductor.o nids_kjfs.o gsToolkit.o
EE_LIBS = -lgskit -ldmakit -lpng -lz -lm -ldraw -lgraph -ldma -lpacket

EE_INCS += -I$(PS2SDK)/ports/include -I$(GSKIT)/include
EE_LDFLAGS += -L$(PS2SDK)/ports/lib -L$(GSKIT)/lib

EE_CFLAGS += -DF_gsKit_texture_png -DF_gsKit_texture_finish -DHAVE_LIBPNG

all: $(EE_BIN)

clean:
	rm -f $(EE_BIN) $(EE_OBJS)

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal