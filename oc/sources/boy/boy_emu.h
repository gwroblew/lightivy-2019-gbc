#ifndef __BOY_EMU_H
#define __BOY_EMU_H

#define GAME_ROM_ADDR   (0x100000)
#define GAME_ROM_SIZE   (0x100000)

void boy_Init(char *savefile);
uint32_t boy_Step();

#endif
