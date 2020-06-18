/** 
 * \file log.c
 * \brief Logging(multi-level) functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#include "log.h"

void HWATCH_Print_Log_Level_Str(int const log_level)
{
  switch(log_level)
  {
    case HWATCH_LOG_LEVEL_OFF:    break;
    case HWATCH_LOG_LEVEL_ERR:    printf("%5u|%7s> ", ++ad_log_id, HWATCH_Log_Level_To_Str(HWATCH_LOG_LEVEL_ERR)); break;
    case HWATCH_LOG_LEVEL_WARN:   printf("%5u|%7s> ", ++ad_log_id, HWATCH_Log_Level_To_Str(HWATCH_LOG_LEVEL_WARN)); break;
    case HWATCH_LOG_LEVEL_INFO:   printf("%5u|%7s> ", ++ad_log_id, HWATCH_Log_Level_To_Str(HWATCH_LOG_LEVEL_INFO)); break;
    case HWATCH_LOG_LEVEL_DEBUG:  printf("%5u|%7s> ", ++ad_log_id, HWATCH_Log_Level_To_Str(HWATCH_LOG_LEVEL_DEBUG)); break;
    default:                    printf("%5u|%7s> ", ++ad_log_id, HWATCH_Log_Level_To_Str(10)); break;
  }
}

char* HWATCH_Log_Level_To_Str(int const log_level)
{
  static char logStr[8];
  switch(log_level)
  {
    case HWATCH_LOG_LEVEL_OFF:    break;
    case HWATCH_LOG_LEVEL_ERR:    strncpy(logStr, "ERROR", 7); break;
    case HWATCH_LOG_LEVEL_WARN:   strncpy(logStr, "WARNING", 7); break;
    case HWATCH_LOG_LEVEL_INFO:   strncpy(logStr, "INFO", 7); break;
    case HWATCH_LOG_LEVEL_DEBUG:  strncpy(logStr, "DEBUG", 7); break;
    default:                    strncpy(logStr, "UNKNOWN", 7); break;
  }
  
  return logStr;
}
