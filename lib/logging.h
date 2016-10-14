/*
 * Gr-plc - IEEE 1901 module for GNU Radio
 * Copyright (C) 2016 Roee Bar <roeeb@ece.ubc.ca>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LOG_H
#define LOG_H

#define INIT_GR_LOG    if (d_log_level == 0) {                  \
                         GR_LOG_SET_LEVEL(d_logger, "NOTICE");  \
                       } else if (d_log_level == 1) {           \
                         GR_LOG_SET_LEVEL(d_logger, "INFO");    \
                       } else if (d_log_level >= 2) {           \
                         GR_LOG_SET_LEVEL(d_logger, "DEBUG");   \
                       }                                        \
                       GR_LOG_SET_CONSOLE_APPENDER(d_logger, "stdout", "gr::log :%p: " + alias() +" - %m%n");
#define PRINT_INFO_VECTOR(x,name)   if (d_log_level >= 1)  {                                \
                                    std::stringstream log_ss;                               \
                                    log_ss << name  << " = [";                              \
                                    for (auto iter=x.begin(); iter != x.end(); iter++)      \
                                        log_ss << *iter << ",";                             \
                                    log_ss << "]";                                          \
                                    GR_LOG_INFO(d_logger, log_ss.str());}
#define PRINT_INFO_VAR(x,name) if (d_log_level >= 1) {std::stringstream log_ss; log_ss << (name) << " = " << (x); GR_LOG_INFO(d_logger, log_ss.str());}
#define PRINT_INFO(msg) if (d_log_level >= 1) {GR_LOG_INFO(d_logger, msg);}
#define PRINT_DEBUG(msg) if (d_log_level >= 2) {GR_LOG_DEBUG(d_logger, msg);}
#define PRINT_NOTICE(msg) GR_LOG_NOTICE(d_logger, msg);

#endif /* LOG_H */
