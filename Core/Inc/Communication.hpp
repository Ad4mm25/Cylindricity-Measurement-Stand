/*
 * Communication.hpp
 * Plik naglowkowy biblioteki obsługującej komunikację STM32 - Raspberry Pi
 *
 * autor: Adam Masalski
 */

#ifdef __cplusplus
extern "C" {
#endif
void SendSentence(char *a);
void SendSingleChar(char d);
void MessageHandler();
void SendCurrentPoint(unsigned int _currentPoint);
void SendCurrentPosition(double x, double z, double c);

#ifdef __cplusplus
}
#endif
