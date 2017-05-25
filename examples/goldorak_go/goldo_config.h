#ifndef __GOLDO_CONFIG_H__
#define __GOLDO_CONFIG_H__
#define OK 0
#define ERROR 1

#define goldo_trace(message) printf("%s",__FILE__)
#define goldo_log(importance,...) printf(__VA_ARGS__)

#endif /* __GOLDO_CONFIG_H__ */