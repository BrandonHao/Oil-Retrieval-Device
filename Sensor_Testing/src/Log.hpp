#ifndef __LOG_HPP__
#define __LOG_HPP__

#ifdef DEBUG
#define LOG(x) Serial.println(x)
#else
#define LOG(x) while(false)
#endif

#endif
