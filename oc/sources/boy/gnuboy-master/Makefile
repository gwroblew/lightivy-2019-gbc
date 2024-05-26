
prefix = /usr/local
exec_prefix = ${prefix}
bindir = ${exec_prefix}/bin

CC = gcc
LD = $(CC)
AS = $(CC)
INSTALL = /usr/bin/install -c

CFLAGS =  -ansi -pedantic -Wall -O3 -fstrength-reduce -fthread-jumps  -fcse-follow-jumps -fcse-skip-blocks -frerun-cse-after-loop  -fexpensive-optimizations -fforce-addr -fomit-frame-pointer
LDFLAGS = $(CFLAGS)  -s
ASFLAGS = $(CFLAGS)

TARGETS =  xgnuboy fbgnuboy sdlgnuboy

ASM_OBJS = 

SYS_DEFS = -DHAVE_CONFIG_H -DIS_LITTLE_ENDIAN  -DIS_LINUX
SYS_OBJS = sys/nix/nix.o $(ASM_OBJS)
SYS_INCS = -I/usr/local/include  -I./sys/nix

FB_OBJS = sys/linux/fbdev.o sys/linux/kb.o sys/pc/keymap.o sys/linux/joy.o sys/oss/oss.o
FB_LIBS = 

SVGA_OBJS = sys/svga/svgalib.o sys/pc/keymap.o sys/linux/joy.o sys/oss/oss.o
SVGA_LIBS = -L/usr/local/lib -lvga

SDL_OBJS = sys/sdl/sdl.o sys/sdl/sdl-audio.o sys/sdl/keymap.o
SDL_LIBS = -L/usr/lib/x86_64-linux-gnu -lSDL
SDL_CFLAGS = -I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT

X11_OBJS = sys/x11/xlib.o sys/x11/keymap.o sys/linux/joy.o sys/oss/oss.o
X11_LIBS =  -lX11 -lXext

all: $(TARGETS)

include Rules

fbgnuboy: $(OBJS) $(SYS_OBJS) $(FB_OBJS)
	$(LD) $(LDFLAGS) $(OBJS) $(SYS_OBJS) $(FB_OBJS) -o $@ $(FB_LIBS)

sgnuboy: $(OBJS) $(SYS_OBJS) $(SVGA_OBJS)
	$(LD) $(LDFLAGS) $(OBJS) $(SYS_OBJS) $(SVGA_OBJS) -o $@ $(SVGA_LIBS)

sdlgnuboy: $(OBJS) $(SYS_OBJS) $(SDL_OBJS)
	$(LD) $(LDFLAGS) $(OBJS) $(SYS_OBJS) $(SDL_OBJS) -o $@ $(SDL_LIBS)

sys/sdl/sdl.o: sys/sdl/sdl.c
	$(MYCC) $(SDL_CFLAGS) -c $< -o $@

sys/sdl/keymap.o: sys/sdl/keymap.c
	$(MYCC) $(SDL_CFLAGS) -c $< -o $@

xgnuboy: $(OBJS) $(SYS_OBJS) $(X11_OBJS)
	$(LD) $(LDFLAGS) $(OBJS) $(SYS_OBJS) $(X11_OBJS) -o $@ $(X11_LIBS)

joytest: joytest.o sys/linux/joy.o
	$(LD) $(LDFLAGS) $^ -o $@

install: all
	$(INSTALL) -d $(bindir)
	$(INSTALL) -m 755 $(TARGETS) $(bindir)

clean:
	rm -f *gnuboy gmon.out *.o sys/*.o sys/*/*.o asm/*/*.o $(OBJS)

distclean: clean
	rm -f config.* sys/nix/config.h Makefile




