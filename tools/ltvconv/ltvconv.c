#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define AUDIO_BUFFER_SIZE     (32768)
#define VIDEO_BUFFER_SIZE     (65536)

uint8_t buf[AUDIO_BUFFER_SIZE];
uint8_t vb[VIDEO_BUFFER_SIZE];

int main(int argc, char *argv[]) {
  FILE *fv, *fa, *fw;

  fv = fopen(argv[1], "rb");
  fa = fopen(argv[2], "rb");
  fw = fopen(argv[3], "wb");

  uint32_t ac = (22050 * 2) / 15;
  uint32_t bs = 0;

  while(1) {
    bs += fread(&buf[bs], 1, ac, fa);
    if (bs == 0)
      break;

    uint32_t bo = bs & 0xFFFFFFFC;

    fwrite(&bo, 1, 4, fw);
    fwrite(buf, 1, bo, fw);

    bs -= bo;
    if (bs > 0) {
      memcpy(buf, &buf[bo], bs);
    }
    fread(vb, 1, 40960, fv);
    fwrite(vb, 1, 40960, fw);
  }

  fclose(fw);
  fclose(fv);
  fclose(fa);
}
