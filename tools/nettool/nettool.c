#include <stdio.h> 
#include <stdlib.h> 
#include <stdarg.h>
#include <errno.h> 
#include <string.h> 
#include <netdb.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/stat.h>
#include <unistd.h>
#include <arpa/inet.h>

#define FILEBUF_SIZE    (1024*1024)

char header[4096];
uint8_t filebuf[FILEBUF_SIZE];
int sockfd, servfd;

char *append(char *buf, char *str) {
  int l = strlen(str);
  memcpy(buf, str, l);
  buf[l] = 10;
  return buf + l + 1;
}

int buildheader(char *buffer, char *cmd, int cnt, ...) {
  va_list va;
  va_start(va, cnt);
  char *arg, *ptr = buffer;

  ptr = append(ptr, cmd);

  for (int i = 0; i < cnt; i++) {
    arg = va_arg(va, char *);
    ptr = append(ptr, arg);
  }
  while (((ptr - buffer) & 3) != 2)
    *ptr++ = 10;

  va_end(va);
  ptr = append(ptr, "#");
  *ptr = 0;
  return ptr - buffer;
}

uint32_t readfile(char *rname, char *fname) {
  int hs = buildheader(header, "GET", 2, "filename", rname);
  if (send(sockfd, header, hs, 0) == -1) {
    perror("send");
    exit (1);
  }
  FILE *fw = fopen(fname, "wb");
  ssize_t size, total = 0;
  while ((size = recv(sockfd, filebuf, FILEBUF_SIZE, 0)) > 0) {
    fwrite(filebuf, 1, size, fw);
    total += size;
  }
  fclose(fw);
  return total;
}

uint32_t listdir(char *rname) {
  int hs = buildheader(header, "LIST", 2, "filename", rname);
  if (send(sockfd, header, hs, 0) == -1) {
    perror("send");
    exit (1);
  }
  ssize_t size, total = 0;
  while ((size = recv(sockfd, filebuf, FILEBUF_SIZE, 0)) > 0) {
    total += size;
  }
  filebuf[total] = 0;
  return total;
}

uint32_t sendfile(char *rname, char *fname) {
  struct stat st;
  char fsize[20];

  stat(fname, &st);
  sprintf(fsize, "%ld", st.st_size);

  int hs = buildheader(header, "PUT", 4, "filename", rname, "filesize", fsize);
  if (send(sockfd, header, hs, 0) == -1) {
    perror("send");
    exit (1);
  }
  FILE *fr = fopen(fname, "rb");
  uint32_t chk = 0, cnt;
  while ((cnt = fread(filebuf, 1, FILEBUF_SIZE, fr)) != 0) {
    while (cnt & 3)
      filebuf[cnt++] = 0;
    if (send(sockfd, filebuf, cnt, 0) == -1) {
      perror("send");
      exit (1);
    }
    uint32_t *cptr = (uint32_t *)filebuf;
    for (int i = 0; i < cnt; i += 4)
      chk ^= *cptr++;
  }
  fclose(fr);
  return chk;
}

uint32_t verifyfile(char *rname) {
  int hs = buildheader(header, "VERIFY", 2, "filename", rname);
  if (send(sockfd, header, hs, 0) == -1) {
    perror("send");
    exit (1);
  }
  char resp[20];
  int i = 0, cnt;
  while ((cnt = recv(sockfd, &resp[i], 1, 0)) > 0 && resp[i] != '\n' && i < 16)
    i++;
  resp[i] = 0;
  return atoi(resp);
}

int main(int argc, char *argv[])
{
  struct hostent *he;
  struct sockaddr_in their_addr; /* connector's address information */
  struct sockaddr_in serv_addr; /* connector's address information */
  socklen_t socklen;
  char resp[16];

  if (argc < 3) {
    printf("Usage:\n");
    printf("  nettool <cmd: put, get> <remote_file> <local_file>\n");
    printf("  nettool list <remote_path>\n");
    printf("  nettool reboot <reboot_mode: 0, 1, 2, 3 - see system.h>\n");
    exit(1);
  }

  serv_addr.sin_family = AF_INET;      /* host byte order */
  serv_addr.sin_port = htons(7070);    /* short, network byte order */
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bzero(&(serv_addr.sin_zero), 8);     /* zero the rest of the struct */

  if ((servfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket");
      exit(1);
  }
  const int       optVal = 1;
  const socklen_t optLen = sizeof(optVal);
  setsockopt(servfd, SOL_SOCKET, SO_REUSEADDR, (void*) &optVal, optLen);
  int cnt = 0;
  while ((bind(servfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) != 0) { 
    cnt++;
    if (cnt == 20) {
      perror("bind");
      exit(1);
    }
  }
  listen(servfd, 5);
  if ((sockfd = accept(servfd, (struct sockaddr *)&their_addr, &socklen)) == -1) {
    perror("accept");
    exit(1);
  }

  if (strcmp(argv[1], "reboot") == 0) {
    if (argc > 3) {
      in_addr_t ip = inet_addr(argv[3]);
      their_addr.sin_addr.s_addr = ip;
    }
    printf("Rebooting...\n");
    char rb[] = "REBOOT\nmode\n0\n#\n";
    rb[12] = argv[2][0];
    if (send(sockfd, rb, 17, 0) == -1) {
      perror("send");
      exit (1);
    }
    sleep(2);
    close(sockfd);
    shutdown(servfd, SHUT_RDWR);
    close(servfd);
    exit(0);
  }

  if (strcmp(argv[1], "put") == 0) {
    printf("Sending: %s\n", argv[3]);
    uint32_t chk = sendfile(argv[2], argv[3]);
    recv(sockfd, &resp, 3, 0);
    printf("%s\n", resp);
    close(sockfd);

    printf("Verifying: %s\n", argv[2]);
    if ((sockfd = accept(servfd, (struct sockaddr *)&their_addr, &socklen)) == -1) {
      perror("accept");
      exit(1);
    }
    uint32_t chk2 = verifyfile(argv[2]);
    close(sockfd);

    if (chk != chk2) {
      printf("Checksum mismatch [local] [remote]: %d %d\n", chk, chk2);
      return 1;
    }
    shutdown(servfd, SHUT_RDWR);
    close(servfd);
    return 0;
  }

  if (strcmp(argv[1], "get") == 0) {
    printf("Reading: %s\n", argv[2]);
    printf("Done. File size: %d\n", readfile(argv[2], argv[3]));
    shutdown(servfd, SHUT_RDWR);
    close(servfd);
    return 0;
  }

  if (strcmp(argv[1], "list") == 0) {
    printf("Directory: %s\n", argv[2]);
    listdir(argv[2]);
    printf("%s\n", filebuf);
    shutdown(servfd, SHUT_RDWR);
    close(servfd);
    return 0;
  }
  shutdown(servfd, SHUT_RDWR);
  close(servfd);
  return 0;
}
