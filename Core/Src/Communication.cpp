/*
 * Biblioteka MyFunctions.c
 * Biblioteka C++ obsługująca komunikację między STM32 a Raspberry Pi
 * Zawiera funkcje, które:
 * - odbierają dane z komputera,
 * - interpretują odebrane dane i na ich podstawie wykonują określone zadania,
 * - wysyłają dane z mikrokontrolera.
 *
 *
 * autor: Adam Masalski
 */


#include <Communication.hpp>
#include "stm32f4xx_hal.h"
#include <string>
#include "math.h"
#include "usart.h"
#include "adc.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "MyFunctions.h"
#include "UartRingbuffer.h"

#ifdef __cplusplus
}
#endif





/**
 * Zmienne globalne
 * - separator
 * - struktura UART
 * - licznik do wysyłania wiadomości
 */
std::string delimeter = "|";
UART_HandleTypeDef uart;
uint16_t counterSend = 0;


/**
 * Zmienne zewnętrzne
 */
extern ring_buffer rx_buffer;
extern Cbool wasInitialized;

extern struct Stepper xStepper;
extern struct Stepper zStepper;
extern struct Stepper cStepper;
extern struct StepperUV uStepper;
extern struct StepperUV vStepper;

extern struct Measurement currentMeasurement;

extern struct Calibration axisCalibration;

extern Mode workMode;

extern uint32_t adcData;




/**
  * @brief  Funkcja wysyła pojedynczy znak przez UART
  *
  * @param  d - znak do wysłania
  *
  * @retval Brak
  */
void SendSingleChar(char d)
{
	Uart_write(d);
}



/**
  * @brief  Funkcja wysyła string przez UART
  *
  * @param  d - znak do wysłania
  *
  * @retval Brak
  */
void SendSentenceString(std::string a)
{
	for(unsigned int d = 0; d < a.length(); d++)
	{
		SendSingleChar(a[d]);
	}
}



/**
  * @brief  Funkcja odczytuje wszystkie odebrane dane z buforu RX
  *
  * @param  Brak
  *
  * @retval s - string złożony z odebranych znaków
  */
std::string ReadMessage()
{
	std::string s;
	while(IsDataAvailable())
	{
		s += Uart_read();
	}
	return s;
}



/**
  * @brief  Funkcja wysyła informację zwrotnę do Raspberry Pi o otrzymaniu informacji
  * 		o starcie programu
  *
  * @param  Brak
  * @retval Brak
  */
void StartProgram()
{
	SendSentenceString("OK\n");
}



/**
  * @brief  Funkcja odbiera aktualne pozycje osi z Raspberry Pi, uruchamia
  * 		odczyt danych z ADC i zmienia stan mikrokontrolera na "zainicjalizowany"
  *
  * @param  s - wiadomość zawierająca pozycje osi
  * @retval Brak
  */
void GetCurrentPositions(std::string s)
{
	size_t pos = 0;
	s.erase(s.begin(), s.begin()+2);
	pos = s.find(delimeter);
	xStepper.currentPosition = std::stof(s.substr(0, pos));
	xStepper.Steps = (int32_t)(xStepper.currentPosition * 4000);
	s.erase(0, pos+1);

	pos = s.find(delimeter);

	zStepper.currentPosition = std::stof(s.substr(0, pos));
	zStepper.Steps = (int32_t)(zStepper.currentPosition * 2560);
	cStepper.currentPosition = std::stof(s.substr(pos+1, std::string::npos));
	cStepper.Steps = (int32_t)(cStepper.currentPosition / 0.018);

	HAL_ADC_Start_DMA(&hadc1, &adcData, 1);
	wasInitialized = TRUE;
}



/**
  * @brief  Funkcja po odebraniu informacji z Raspberry Pi o zatrzymaniu programu
  * 		wyłącza ADC i zmienia stan mikrokontrolera na "niezainicjalizowany"
  *
  * @param  Brak
  * @retval Brak
  */
void StopProgram()
{
	HAL_ADC_Stop_DMA(&hadc1);
	wasInitialized = FALSE;
}




/**
  * @brief  Funkcja odbiera informacje o parametrach pomiaru,
  * 		dekoduję tę informację i ustawia odpowiednie wartości parametrów
  * 		takie jak strategia badania, prędkosci osi, skok, rodzaj badanej
  * 		części
  *
  * @param  s - wiadomość zawierająca informacje o parametrach
  * @retval Brak
  */
void GetMeasurementInfo(std::string s)
{
	currentMeasurement.measType = (MeasurementType)(s[2]-48);
	switch(currentMeasurement.measType)
	{
	case LP:		//2|Badanie,xSpeed,Typ,zSpeed,cSpeed,zStep
		currentMeasurement.xSpeed = (MeasurementSpeed)(s[3]-48);
		currentMeasurement.measuredPart = (PartType)(s[4]-48);
		currentMeasurement.zSpeed = (MeasurementSpeed)(s[5]-48);
		currentMeasurement.cSpeed = (MeasurementSpeed)(s[6]-48);
		SetStep(int(s[7]-48));
		break;
	case GD:		//2|Badanie,xSpeed,Typ,zSpeed,cSpeed,cStep
		currentMeasurement.xSpeed = (MeasurementSpeed)(s[3]-48);
		currentMeasurement.measuredPart = (PartType)(s[4]-48);
		currentMeasurement.zSpeed = (MeasurementSpeed)(s[5]-48);
		currentMeasurement.cSpeed = (MeasurementSpeed)(s[6]-48);
		SetStep(int(s[7]-48));
		break;
	case XY:		//2|Badanie,zSpeed,xSpeed,cSpeed,xStep
		currentMeasurement.zSpeed = (MeasurementSpeed)(s[3]-48);
		currentMeasurement.xSpeed = (MeasurementSpeed)(s[4]-48);
		currentMeasurement.cSpeed = (MeasurementSpeed)(s[5]-48);
		SetStep(int(s[6]-48));
		break;
	case Spiral:	//2|Badanie,xSpeed, Typ, cSpeed,zStep, Direction
		currentMeasurement.xSpeed = (MeasurementSpeed)(s[3]-48);
		currentMeasurement.measuredPart = (PartType)(s[4]-48);
		currentMeasurement.cSpeed = (MeasurementSpeed)(s[5]-48);
		SetStep(int(s[6]-48));
		currentMeasurement.direction = uint16_t(s[7])-48;
		break;
	case Cage:		//2|Badanie,xSpeed,Typ,zSpeed,cSpeed,zStep,cStep
		currentMeasurement.xSpeed = (MeasurementSpeed)(s[3]-48);
		currentMeasurement.measuredPart = (PartType)(s[4]-48);
		currentMeasurement.zSpeed = (MeasurementSpeed)(s[5]-48);
		currentMeasurement.cSpeed = (MeasurementSpeed)(s[6]-48);
		SetStep(int(s[7]-48));
		SetStep(int(s[8]-48));
		break;
	}
}



/**
  * @brief  Funkcja odbiera informacje o zadanej pozycji osi
  * 		i wysyła pozycję, do której ma przemieścić się dany silnik
  *
  * @param  _stepper - wskaźnik do struktury silnika Stepper
  * @param  s - string zawierający oś silnika i zadaną pozycję
  * @retval Brak
  */
void GetDestination(struct Stepper* _stepper, std::string s)			//"x|double"
{
	s.erase(s.begin(), s.begin()+2);
	MoveStepperToPosition(_stepper, std::stof(s));
}



/**
  * @brief  Funkcja blokuje możliwość ręcznego ruchu silnika
  *
  * @param  s - string informujący o blokadzie i rodzaju osi
  * @retval Brak
  */
void AxisLock(std::string s)
{
	switch (s[2])
	{
	case 'x':
		StepperLock(&xStepper, TRUE);
		break;
	case 'z':
		StepperLock(&zStepper, TRUE);
		break;
	case 'c':
		StepperLock(&cStepper, TRUE);
		break;
	default:
		break;
	}
}



/**
  * @brief  Funkcja odblokowuje możliwość ręcznego ruchu silnika
  *
  * @param  s - string informujący o odblokowaniu i rodzaju osi
  * @retval Brak
  */
void AxisUnlock(std::string s)
{
	switch (s[2])
	{
	case 'x':
		StepperLock(&xStepper, FALSE);
		break;
	case 'z':
		StepperLock(&zStepper, FALSE);
		break;
	case 'c':
		StepperLock(&cStepper, FALSE);
		break;
	default:
		break;
	}
}



/**
  * @brief  Funkcja podobna do GetDestination, ale odbiera informacje
  * 		o zadanej pozycji w dwóch osiach i wysyła pozycje,
  * 		do których mają przemieścić się dane silniki
  *
  * @param  _stepper1 - wskaźnik do struktury pierwszego silnika krokowego
  * @param  _stepper2 - wskaźnik do struktury drugiego silnika krokowego
  * @param  s - string zawierający osie silników i zadane pozycje
  * @retval Brak
  */
void GetDoubleDestination(struct Stepper* _stepper1, struct Stepper* _stepper2, std::string s)
{
	size_t pos = 0;
	s.erase(s.begin(), s.begin()+2);

	pos = s.find(delimeter);
	MoveStepperToPosition(_stepper1, std::stof(s.substr(0, pos)));
	MoveStepperToPosition(_stepper2, std::stof(s.substr(pos+1, std::string::npos)));
}



/**
  * @brief  Funkcja odbiera informacje o pozycji końcowej lub początkowej
  * 		pomiaru i przypisuje te dane do struktury currentMeasurement,
  * 		a następnie rozpoczyna tryb przygotowania (tryb "preapring")
  *

  * @param  s - string pozycję końca lub początku
  * @retval Brak
  */
void GetStartEnd(std::string s)
{
	size_t pos = 0;
	switch(s[2])
	{
	case 'k':
		s.erase(s.begin(), s.begin()+4);
		pos = s.find(delimeter);
		currentMeasurement.zEnd = std::stof(s.substr(0, pos));
		currentMeasurement.cEnd = std::stof(s.substr(pos+1, std::string::npos));
		break;
	case 'p':
		s.erase(s.begin(), s.begin()+4);
		pos = s.find(delimeter);
		currentMeasurement.zStart = std::stof(s.substr(0, pos));
		currentMeasurement.cStart = std::stof(s.substr(pos+1, std::string::npos));
		StartPreparing();
		break;
	}
}



/**
  * @brief  Funkcja zeruje pozycję osi
  *
  * @param  s - string zawierający informację o wyzerowaniu osi
  * @retval Brak
  */
void ZeroAxis(std::string s)
{
	switch(s[2])
	{
	case 'X':
		xStepper.Steps = 0;
		xStepper.currentPosition = 0.0f;
		xStepper.StepperTimer->CNT = 0;
		break;
	case 'Z':
		zStepper.Steps = 0;
		zStepper.currentPosition = 0.0f;
		zStepper.StepperTimer->CNT = 0;
		break;
	case 'C':
		cStepper.Steps = 0;
		cStepper.currentPosition = 0.0f;
		cStepper.StepperTimer->CNT = 0;
		break;
	}
}



/**
  * @brief  Funkcja rozpoczyna kalibrację osi części
  * 		względem osi stołu
  *
  * @param  s - string zawierający informację o kalibracji
  * @retval Brak
  */
void StartCalibration(std::string s)
{
	workMode = calibration;
	size_t pos = 0;
	switch(s[2])
	{
	case '0':
		axisCalibration.partType = tuleja;
		break;
	case '1':
		axisCalibration.partType = walek;
		break;
	}

	s.erase(s.begin(), s.begin()+4);
	pos = s.find(delimeter);
	axisCalibration.stage = 1;
	axisCalibration.height = std::stof(s.substr(0, pos));
	axisCalibration.diameter = std::stof(s.substr(pos+1, std::string::npos));
}



/**
  * @brief  Funkcja rozpoczyna kalibrację osi części
  * 		względem osi stołu
  *
  * @param  s - string zawierający informację o kalibracji
  * @retval Brak
  */
void SendCurrentPosition(double x, double z, double c)
{

	std::string s = "1|";
	s += std::to_string((int32_t)roundf(x * 100000));
	s += "|";
	s += std::to_string((int32_t)roundf(z * 100000));
	s+= "|";
	s += std::to_string((int32_t)roundf(c * 100000));
	s += "|\n";
	//ChangePinState(TX_FLAG_GPIO_Port, TX_FLAG_Pin, 1);
	SendSentenceString(s);

}



/**
  * @brief  Funkcja wysyła do Raspberry Pi informację
  * 		o aktualnym punkcie ścieżki pomiaru
  *
  * @param  _currentPoint aktualny punkt ścieżki
  * @retval Brak
  */
void SendCurrentPoint(unsigned int _currentPoint)
{
	std::string s = "2|";
	s += std::to_string(_currentPoint);
	s += "|\n";

	SendSentenceString(s);

}




/**
  * @brief  Funkcja cyklicznie wysyła informacje o aktualnym położeniu głowicy
  * 		w trybie ręcznym
  *
  * @param  Brak
  * @retval Brak
  */
void SendingHandler()
{
	if (wasInitialized == TRUE && workMode == manual)
	{
		counterSend++;
		if (counterSend > 5)
		{
			counterSend = 0;
			if (xStepper.lastSentPosition != xStepper.currentPosition ||
					zStepper.lastSentPosition != zStepper.currentPosition ||
					cStepper.lastSentPosition != cStepper.currentPosition)
			{
				xStepper.lastSentPosition = xStepper.currentPosition;
				zStepper.lastSentPosition = zStepper.currentPosition;
				cStepper.lastSentPosition = cStepper.currentPosition;
				SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
			}

		}
	}


}



/**
  * @brief  Główna funkcja obsługująca wysyłanie danych
  * 		oraz odbieranie wiadomości z Raspberry Pi.
  * 		Odebrane dane są interpretowane i na ich podstawie wybierana
  * 		jest odpowiednia funkcja
  *
  * @param  Brak
  * @retval Brak
  */
void MessageHandler()
{
	if(rx_buffer.head == rx_buffer.tail)		//jeżeli brak wiadomosci do odebrania
	{
		SendingHandler();
		return;
	}

	std::string s;
	s = ReadMessage();

	if (s.size() != 0)
	{
		switch (s[0])
		{
		case '0':
			StartProgram(); 		break;
		case '1':
			StopMeasuring();		break;
		case '2':
			GetMeasurementInfo(s);	break;
		case '3':
			AxisLock(s);			break;
		case '4':
			AxisUnlock(s);			break;
		case '5':
			GetCurrentPositions(s);	break;
		case '6':
			StartCalibration(s);	break;
		case 'S':
			StopProgram();			break;
		case 'x':
			GetDestination(&xStepper, s);					break;
		case 'z':
			GetDestination(&zStepper, s);					break;
		case 'd':
			GetDoubleDestination(&zStepper, &cStepper, s);	break;
		case '9':
			ZeroAxis(s);			break;
		case 'P':
			GetStartEnd(s);			break;
		default:
			break;
		}
	}

}

