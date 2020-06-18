/** 
 * \file log.h
 * \brief Logger (multiple level) for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_AD_LOG_H
#define HWATCH_AD_LOG_H

#include <stdio.h>
#include <string.h>

extern unsigned int ad_log_level;
extern unsigned int ad_log_id;

#define HWATCH_LOG_LEVEL_OFF     (0)
#define HWATCH_LOG_LEVEL_ERR     (1)
#define HWATCH_LOG_LEVEL_WARN    (2)
#define HWATCH_LOG_LEVEL_INFO    (3)
#define HWATCH_LOG_LEVEL_DEBUG   (4)

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define OLD_LOG_HELPER(log_level, ...) do {\
                                                  if (log_level <= ad_log_level)\
                                                  {\
                                                    switch(log_level)\
                                                    {\
                                                    case HWATCH_LOG_LEVEL_OFF:    break;\
                                                    case HWATCH_LOG_LEVEL_ERR:    printf("ERROR>  : "); break;\
                                                    case HWATCH_LOG_LEVEL_WARN:   printf("WARNING>: "); break;\
                                                    case HWATCH_LOG_LEVEL_INFO:   printf("INFO>   : "); break;\
                                                    case HWATCH_LOG_LEVEL_DEBUG:  printf("DEBUG>  : "); break;\
                                                    default:                    printf("UNKNOWN>: "); break;\
                                                    }\
                                                    if(HWATCH_LOG_LEVEL_OFF != log_level)\
                                                    {\
                                                      printf("%s:%d:%s ", __FILENAME__ , __LINE__, __func__);\
                                                      printf(__VA_ARGS__);\
                                                      printf("\n"); \
                                                    }\
                                                  }\
                                                } while (0)
                          
#define OLD_LOG(...) OLD_LOG_HELPER(__VA_ARGS__)

#define HWATCH_LOG(log_level, ...) do {\
                            if (log_level <= ad_log_level) {\
                                HWATCH_Print_Log_Level_Str(log_level);\
                                printf("%s:%d:%s ", __FILENAME__ , __LINE__, __func__);\
                                printf(__VA_ARGS__);\
                                printf("\n");\
                            }\
                        } while (0)           

#define HWATCH_LOG_ERR(...)  HWATCH_LOG(HWATCH_LOG_LEVEL_ERR, __VA_ARGS__)
#define HWATCH_LOG_WARN(...)  HWATCH_LOG(HWATCH_LOG_LEVEL_WARN, __VA_ARGS__)
#define HWATCH_LOG_INFO(...)  HWATCH_LOG(HWATCH_LOG_LEVEL_INFO, __VA_ARGS__)
#define HWATCH_LOG_DEBUG(...)  HWATCH_LOG(HWATCH_LOG_LEVEL_DEBUG, __VA_ARGS__)
                          
void HWATCH_Print_Log_Level_Str(int const log_level);
char* HWATCH_Log_Level_To_Str(int const log_level);
#endif  /* HWATCH_AD_LOG_H */