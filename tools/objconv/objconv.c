#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../../oc/sources/gfx3d.h"

#define MAX_POINTS    65536
#define MAX_FACES     65536

OBJECT3D object;
POINT3D points[MAX_POINTS];
FACE3D faces[MAX_FACES];

typedef struct {
  float x;
  float y;
  float z;
} FPOINT3D;

FPOINT3D fpoints[MAX_POINTS];

typedef struct {
  float z;
  uint32_t idx;
} FFACE3D;

FFACE3D ffaces[MAX_FACES];

FILE *fw = NULL;

#include <math.h>

/*int main(int argc, char *argv[]) {
  int j = 0;
  for (int i = 0; i < 720 + 180; i++, j++) {
    printf("%d,", (int)(32768.0f * sin(i * 3.1415926536f * 2.0f / 720.0f)));
    if (j == 16) {
      printf("\n");
      j = 0;
    }
  }
}*/

int __compar(const void *f1, const void *f2) {
  const FFACE3D *ff1 = f1;
  const FFACE3D *ff2 = f2;
  return ff1->z > ff2->z;
}

float distz(POINT3D *p) {
  return p->x * p->x + p->y * p->y + p->z * p->z;
}

void output_header() {
  int i;
  printf("const OBJECT3D RODATA testobject = {\n");
  printf("  .point_cnt = %d,\n", object.point_cnt);
  printf("  .face_cnt = %d\n", object.face_cnt);
  printf("};\n");
  printf("const POINT3D testpoints[] RODATA = {\n");
  for (i = 0; i < object.point_cnt; i++) {
    printf("  { %d, %d, %d },\n", points[i].x, points[i].y, points[i].z);
  }
  printf("  { 0, 0, 0 } };\n");
  printf("const FACE3D testfaces[] RODATA = {\n");
  for (i = 0; i < object.face_cnt; i++) {
    int j = ffaces[i].idx;
    printf("  { %d, %d, %d, %d },\n", faces[j].p1, faces[j].p2, faces[j].p3, faces[j].color);
  }
  printf("  { 0, 0, 0 } };\n");
}

int skipfaces = 0;

void output_binary() {
  int i;
  fwrite(&object, 1, sizeof(OBJECT3D), fw);
  fwrite(points, 1, object.point_cnt * sizeof(POINT3D), fw);
  if (!skipfaces) {
    for (i = 0; i < object.face_cnt; i++) {
      int j = ffaces[i].idx;
      fwrite(&faces[j], 1, sizeof(FACE3D), fw);
    }
  }
}

void output_3do() {
  int i;
  printf("const OBJECT3D RODATA testobject = {\n");
  printf("  .point_cnt = %d,\n", object.point_cnt);
  printf("  .face_cnt = %d\n", object.face_cnt);
  printf("};\n");
  printf("const POINT3D testpoints[] RODATA = {\n");
  for (i = 0; i < object.point_cnt; i++) {
    printf("  { %d, %d, %d },\n", points[i].x, points[i].y, points[i].z);
  }
  printf("  { 0, 0, 0 } };\n");
  printf("const FACE3D testfaces[] RODATA = {\n");
  for (i = 0; i < object.face_cnt; i++) {
    int j = ffaces[i].idx;
    printf("  { %d, %d, %d },\n", faces[j].p1, faces[j].p2, faces[j].p3);
  }
  printf("  { 0, 0, 0 } };\n");
}

int parseidx(char *l, int *p) {
  int cnt = 0;
  l = strtok(l, " ");
  while (l != NULL) {
    sscanf(l, "%d", &p[cnt++]);
    l = strtok(NULL, " ");
  }
  return cnt;
}

typedef struct {
  char name[256];
  float r;
  float g;
  float b;
  uint32_t colidx;
} MATERIAL;

char *palfile = "/home/greg/palette.bin";
uint16_t palette[256];
uint32_t palidx = 0;
MATERIAL mats[256];
uint32_t matcnt = 0;

void loadpal() {
  FILE *fr = fopen(palfile, "rb");
  if (fr == NULL) {
    palidx = 128;
    return;
  }
  fread(palette, 1, 512, fr);
  fread(&palidx, 1, 4, fr);
  fclose(fr);
}

void savepal() {
  FILE *fw = fopen(palfile, "wb");
  fwrite(palette, 1, 512, fw);
  fwrite(&palidx, 1, 4, fw);
  fclose(fw);
}

void parsemtl(char *mtlfile) {
  if (matcnt > 0)
    return;
  FILE *fr = fopen(mtlfile, "rt");
  char line[1000];

  while(fgets(line, 998, fr) != NULL) {
    if (strncmp(line, "newmtl", 6) == 0) {
      strcpy(mats[matcnt].name, &line[7]);
      mats[matcnt].colidx = 0;
      matcnt++;
      continue;
    }
    char *ptr = line;
    while (*ptr == 32 && *ptr != 0)
      ptr++;
    if (strncmp(ptr, "Kd", 2) == 0) {
      sscanf(&ptr[3], "%f %f %f", &mats[matcnt-1].r, &mats[matcnt-1].g, &mats[matcnt-1].b);
      uint8_t r = mats[matcnt-1].r * 255.0;
      uint8_t g = mats[matcnt-1].g * 255.0;
      uint8_t b = mats[matcnt-1].b * 255.0;
      r = r > 255 ? 255 : r;
      g = g > 255 ? 255 : g;
      b = b > 255 ? 255 : b;
      uint16_t cc = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
      palette[palidx] = (cc << 8) | (cc >> 8);
      mats[matcnt-1].colidx = palidx;
      palidx++;
    }
  }

  fclose(fr);
}

int findmaterial(char *name) {
  for (int i = 0; i < matcnt; i++) {
    if (strcmp(name, mats[i].name) == 0) {
      return mats[i].colidx;
    }
  }
  printf("Unknown material: %s\n", name);
  return 255;
}

int minmaxdone = 0;
float minx = 1000000000, maxx = -1000000000;
float miny = 1000000000, maxy = -1000000000;
float minz = 1000000000, maxz = -1000000000;
float offx;
float offy;
float offz;
float sizem;
int objcnt = 0;
uint32_t framecnt = 1;

void initobj() {
  if (fw != NULL)
    fclose(fw);
  char fn[256];
  sprintf(fn, "output%02d.3d", objcnt);
  fw = fopen(fn, "wb");
  fwrite(&framecnt, 1, 4, fw);
  objcnt++;
  minmaxdone = 0;
  minx = 1000000000, maxx = -1000000000;
  miny = 1000000000, maxy = -1000000000;
  minz = 1000000000, maxz = -1000000000;
}

void normobj() {
  int i;
  if (!minmaxdone) {
    for (i = 0; i < object.point_cnt; i++) {
      if (fpoints[i].x < minx)
        minx = fpoints[i].x;
      if (fpoints[i].x > maxx)
        maxx = fpoints[i].x;
      if (fpoints[i].y < miny)
        miny = fpoints[i].y;
      if (fpoints[i].y > maxy)
        maxy = fpoints[i].y;
      if (fpoints[i].z < minz)
        minz = fpoints[i].z;
      if (fpoints[i].z > maxz)
        maxz = fpoints[i].z;
    }
    float sizex = maxx - minx;
    float sizey = maxy - miny;
    float sizez = maxz - minz;
    offx = - (maxx + minx) / 2;
    offy = - (maxy + miny) / 2;
    offz = - (maxz + minz) / 2;

    sizem = sizex > sizey ? sizex : sizey;
    sizem = sizem > sizez ? sizem : sizez;
    minmaxdone = 1;
  }

  for (i = 0; i < object.point_cnt; i++) {
    points[i].x = (fpoints[i].x + offx) * 4096.0f / sizem;
    points[i].y = (fpoints[i].y + offy) * 4096.0f / sizem;
    points[i].z = (fpoints[i].z + offz) * 4096.0f / sizem;
  }

  for (i = 0; i < object.face_cnt; i++) {
    ffaces[i].z = distz(&points[faces[i].p1]) + distz(&points[faces[i].p2]) + distz(&points[faces[i].p3]);
    ffaces[i].idx = i;
  }
  qsort(ffaces, object.face_cnt, sizeof(FFACE3D), __compar);
}

void parseobj(char *objfile) {
  FILE *fr = fopen(objfile, "rt");
  char line[1000];
  float x, y, z;
  int i = 0, j = 0;
  int p[32];
  int pcnt;
  int colidx = 0;
  int poff = 0;

  object.point_cnt = 0;
  object.face_cnt = 0;

  while(fgets(line, 998, fr) != NULL) {
    if (strncmp(line, "o ", 2) == 0) {
      if (i != 0) {
        object.point_cnt = i;
        object.face_cnt = j;
        normobj();

        //output_header();
        output_binary();
        initobj();
      }
      poff += i;
      object.point_cnt = 0;
      object.face_cnt = 0;
      i = 0;
      j = 0;
      continue;
    }
    if (strncmp(line, "usemtl", 6) == 0) {
      colidx = findmaterial(&line[7]);
      continue;
    }
    if (line[0] == 'v' && line[1] == 32) {
      sscanf(&line[2], "%f %f %f", &x, &y, &z);
      fpoints[i].x = x;
      fpoints[i].y = y;
      fpoints[i].z = z;
      i++;
      continue;
    }
    if (line[0] == 'f' && line[1] == 32) {
      pcnt = parseidx(&line[2], p);
      for (int i = 0; i < pcnt - 2; i++) {
        faces[j].p1 = p[i] - 1 - poff;
        faces[j].p2 = p[i + 1] - 1 - poff;
        faces[j].p3 = p[pcnt - 1] - 1 - poff;
        faces[j].color = colidx;
        j++;
      }
      continue;
    }
  }
  fclose(fr);
  object.point_cnt = i;
  object.face_cnt = j;
  normobj();

  //output_header();
  output_binary();
}

void convert(char *fname) {
  printf("Converting %s...\n", fname);
  char fbuf[256];
  sprintf(fbuf, "%s.mtl", fname);
  parsemtl(fbuf);
  sprintf(fbuf, "%s.obj", fname);
  parseobj(fbuf);
}

int main(int argc, char *argv[]) {

  loadpal();

  if (argc == 2) {
    initobj();
    convert(argv[1]);
    goto _exit;
  }
  int fs = atoi(argv[2]);
  int fe = atoi(argv[3]);

  framecnt = fe - fs + 1;
  initobj();

  for (int i = fs; i <= fe; i++) {
    char fbuf[256];
    sprintf(fbuf, "%s%06d", argv[1], i);
    convert(fbuf);
    skipfaces = 1;
  }

_exit:
  fclose(fw);
  savepal();
  return 0;
}
