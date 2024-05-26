#ifndef __CONFIG_H
#define __CONFIG_H

void config_init(char *buf);
const char *config_get_string(const char *name, const char *def);
int config_get_number(const char *name, int def);
int config_set(char *buf, char *name, char *value);

#endif
