/*
 * Biblioteka MyFunctions.c
 * Glowny plik programu, zawiera funkcje obslugujace dzialanie stanowiska
 *
 * autor: Adam Masalski
 */

#include <MyFunctions.h>
#include "Communication.hpp"
#include <math.h>


/* Deklaracja struktur silnikow stanowiska */
struct Stepper xStepper;
struct Stepper zStepper;
struct Stepper cStepper;
struct StepperUV uStepper;
struct StepperUV vStepper;

/* Deklaracja struktury aktualnego pomiaru */
struct Measurement currentMeasurement;

struct Calibration axisCalibration;

/* Zmienne globalne */
/* Wartosci skoku osi 2 w pomiarach */
double xSteps[9] =  {0.0025, 0.005, 0.01,  0.02, 0.05, 0.1,  0.2,  0.5, 1.0 };
double zSteps[9] =  {0.0025, 0.005, 0.01,  0.02, 0.05, 0.1,  0.2,  0.5, 1.0 };
double cSteps[10] = {0.036,  0.072, 0.144, 0.36, 0.72, 1.08, 1.44, 1.8, 3.6, 7.2};

/* Roznica miedzy pozycja zadana a aktualna */
double delta = 0.0;
double delta2 = 0.0;

uint16_t counterMsg = 0;
Cbool wasInitialized = FALSE;
uint32_t adcData = 0;

/* Tryb pracy stanowiska */
Mode workMode = manual;



/**
  * @brief  Funkcja zmienia stan pinu mikrokontrolera za pomoca 1 lub 0
  *
  * @param  GPIOx - port
  * @param  GPIO_Pin - wartosc pinu (0..15)
  * @param  Stan pinu, jaki ma byc zapisany
  * 			@arg 1: ustawienie stanu wysokiego
  * 			@arg 0: ustawienie stanu niskiego
  *
  * @retval Brak
  */
void ChangePinState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int _state)
{
	assert_param(IS_GPIO_PIN(GPIO_Pin));
	assert_param(IS_GPIO_PIN_ACTION(PinState));

	if(_state == 0)
	{
		GPIOx->BSRR = GPIO_Pin;
	}
	else
	{
		GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
	}
}



/**
 * @brief  Funkcja inicjalizcuje podzespoly i uruchamia glowny licznik
 *
 * @param  Brak
 * @retval Brak
 */
void InitializeComponents()
{
	HAL_ADC_Stop(&hadc1);

	InitializeSteppers();

	HAL_TIM_Encoder_Start_IT(&X_ENCODER_TIMER_HANDLE, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&Z_ENCODER_TIMER_HANDLE, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&C_ENCODER_TIMER_HANDLE, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&MAIN_TIMER_HANDLE);

}



/**
 * @brief  Funkcja ustala wartosc kroku pomiaru
 *
 * @note   Funkcja ustawia wartosc w osi 2 w zaleznosci
 * 			od wybranego trybu pomiaru
 *
 * @param  _step indeks tablic ..Steps[]
 *
 * @retval Brak
 */
void SetStep(int _step)
{
	static uint8_t _count = 0;
	switch (currentMeasurement.measType)
	{
	case LP:
		currentMeasurement.step = zSteps[_step];
		break;
	case GD:
		currentMeasurement.step = cSteps[_step];
		break;
	case XY:
		currentMeasurement.step = xSteps[_step];
		break;
	case Spiral:
		currentMeasurement.step = zSteps[_step];
		break;
	case Cage:
		if (_count == 0)
		{
			currentMeasurement.step = zSteps[_step];
			_count++;
		}
		else
		{
			currentMeasurement.step2 = cSteps[_step];
			_count = 0;
		}
		break;
	}
}



/**
 * @brief  Funkcja ustala kierunek skoku pomiaru
 * 			w zaleznosci od stosunku miedzy punktami startu i konca
 *
 * @param  Brak
 * @retval Brak
 */
void SetStepDirection()
{
	switch (currentMeasurement.measType)
	{
	case LP:
		if (currentMeasurement.zStart > currentMeasurement.zEnd)
			currentMeasurement.step = -currentMeasurement.step;
		break;
	case GD:
		if (currentMeasurement.cStart > currentMeasurement.cEnd)
			currentMeasurement.step = -currentMeasurement.step;
		break;
	case XY:
		if (currentMeasurement.xStart > currentMeasurement.xEnd)
			currentMeasurement.step = -currentMeasurement.step;
		break;
	case Cage:
		if (currentMeasurement.zStart > currentMeasurement.zEnd)
			currentMeasurement.step = -currentMeasurement.step;
		if (currentMeasurement.cStart <= currentMeasurement.cEnd)
			currentMeasurement.step2 = -currentMeasurement.step2;
		break;
	case Spiral:
		if (currentMeasurement.zStart > currentMeasurement.zEnd)
			currentMeasurement.step = -currentMeasurement.step;
		break;
	}
}



/**
 * @brief  Funkcja odczytująca ostatnią wartość na przetworniku A/C
 *
 * @param  Brak
 * @retval Brak
 */
void ADCHandler()
{
	if (workMode == automatic && adcData >= 4094)
	{
		HardStop(&xStepper);
		HardStop(&zStepper);
		HardStop(&cStepper);
		workMode = error;
	}
}



/**
 * @brief  Funkcja inicjalizuje strukture silnika krokowego osi glownej
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika
 * @param  StepperTimer licznik silnika
 * @param  TimerHandle
 * @param  PinStep wyjscie sygnalu STEP
 * @param  PortDir port pinu DIR
 * @param  PinDir wyjscie sygnalu DIR
 * @param  Axis os silnika
 * @param  Encoder - enkoder osi
 *
 * @retval Brak
 */
void StepperInit(struct Stepper* _stepper,
		TIM_TypeDef* StepperTimer,
		TIM_HandleTypeDef TimerHandle,
		uint16_t PinStep,
		GPIO_TypeDef* PortDir,
		uint16_t PinDir,
		StepperAxis Axis,
		TIM_TypeDef* Encoder)
{
	_stepper->StepperTimer = StepperTimer;
	_stepper->TimerHandle = TimerHandle;

	_stepper->PinStep = PinStep;
	_stepper->PortDir = PortDir;
	_stepper->PinDir = PinDir;

	_stepper->Axis = Axis;

	_stepper->State = stopped;
	_stepper->LastState = stopped;

	//_stepper->StepperTimer->CR1 = 0;
	//HAL_TIM_PWM_Start_IT(&TimerHandle, TIM_CHANNEL_1);

	ChangeStepperSpeed(_stepper, manualSpeed);

	_stepper->Steps = 0;

	_stepper->Encoder = Encoder;

	_stepper->currentPosition = 0.0;
	_stepper->targetPosition = 0.0;
	_stepper->lastSentPosition = _stepper->currentPosition;

	_stepper->changePosition = FALSE;
	_stepper->inPosition = FALSE;

	_stepper->isLocked = FALSE;

}



/**
 * @brief  Funkcja inicjalizuje strukture silnika krokowego osi pomocniczej
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika
 * @param  PortStep port pinu STEP
 * @param  PinStep wyjscie sygnalu STEP
 * @param  PortDir port pinu DIR
 * @param  PinDir wyjscie sygnalu DIR
 * @param  Axis os silnika
 *
 * @retval Brak
 */
void StepperUVInit(struct StepperUV *_stepper,
		GPIO_TypeDef* PortStep,
		uint16_t PinStep,
		GPIO_TypeDef* PortDir,
		uint16_t PinDir,
		StepperAxis Axis)
{
	_stepper->PortStep = PortStep;
	_stepper->PinStep = PinStep;

	_stepper->PortDir = PortDir;
	_stepper->PinDir = PinDir;

	_stepper->Axis = Axis;

	_stepper->State = stopped;
	_stepper->LastState = stopped;

	_stepper->Speed = speed1;
	_stepper->Tick = 0;

	_stepper->Steps = 0;
}



/**
 * @brief  Funkcja inicjalizuje 3 silniki główne i 2 silniki pomocnicze
 *
 * @param  Brak
 * @retval Brak
 */
void InitializeSteppers()
{

	StepperInit(&xStepper, X_TIMER, X_TIMER_HANDLE, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin, X, X_ENCODER_TIMER);
	StepperInit(&zStepper, Z_TIMER, Z_TIMER_HANDLE, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin, Z, Z_ENCODER_TIMER);
	StepperInit(&cStepper, C_TIMER, C_TIMER_HANDLE, C_STEP_Pin, C_DIR_GPIO_Port, C_DIR_Pin, C, C_ENCODER_TIMER);
	//cStepper.Steps = 10000;

	StepperUVInit(&uStepper, U_STEP_GPIO_Port, U_STEP_Pin, U_DIR_GPIO_Port, U_DIR_Pin, U);
	StepperUVInit(&vStepper, V_STEP_GPIO_Port, V_STEP_Pin, V_DIR_GPIO_Port, V_DIR_Pin, V);
}



/**
 * @brief  Funkcja zmienia stan silnika głównego na:
 * 		   - Zatrzymany (stopped),
 * 		   - Jadący w plus (movingPlus),
 * 		   - Jadący w minus (mobingMinus).
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @param  _state zadany stan silnika
 *
 * @retval Brak
 */
void ChangeStepperState(struct Stepper *_stepper, StepperState _state)
{
	_stepper->State = _state;

	if (_state != stopped)
		_stepper->LastState = _state;

	switch(_state)
	{
		case movingPlus:
			ChangePinState(_stepper->PortDir, _stepper->PinDir, 1);
			HAL_TIM_PWM_Start_IT(&_stepper->TimerHandle, TIM_CHANNEL_1);
			break;

		case movingMinus:
			ChangePinState(_stepper->PortDir, _stepper->PinDir, 0);
			HAL_TIM_PWM_Start_IT(&_stepper->TimerHandle, TIM_CHANNEL_1);
			break;

		default:
			HAL_TIM_PWM_Stop_IT(&_stepper->TimerHandle, TIM_CHANNEL_1);
			break;
	}
}



/**
 * @brief  Funkcja zmienia stan silnika pomocniczego na:
 * 		   - Zatrzymany (stopped),
 * 		   - Jadący w plus (movingPlus),
 * 		   - Jadący w minus (mobingMinus).
 *
 * @param  _stepperUV wskaznik do zadeklarowanej struktury silnika pomocniczego
 * @param  _state zadany stan silnika
 *
 * @retval Brak
 */
void ChangeStepperUVState(struct StepperUV *_stepper, StepperState _state)
{
	_stepper->State = _state;

	if (_state != stopped)
		_stepper->LastState = _state;

	switch(_state)
	{
		case movingPlus:
			ChangePinState(_stepper->PortDir, _stepper->PinDir, 1);
			HAL_TIM_Base_Start_IT(&UV_TIMER_HANDLE);
			break;

		case movingMinus:
			ChangePinState(_stepper->PortDir, _stepper->PinDir, 0);
			HAL_TIM_Base_Start_IT(&UV_TIMER_HANDLE);
			break;

		default:
			HAL_TIM_Base_Stop_IT(&UV_TIMER_HANDLE);
			break;
	}
}



/**
 * @brief  Funkcja umożliwia ruch pomocniczego silnika krokowego do zadanej pozycji
 *
 * @param  _stepperUV wskaznik do zadeklarowanej struktury silnika pomocniczego
 * @param  _target zadana pozycja
 *
 * @retval Brak
 */
void MoveUVStepper(struct StepperUV *_stepper, double _target)
{
	_stepper->Steps = 0;
	_stepper->StepsToGo = (int32_t)(_target * 1000.0);
	if (_stepper->StepsToGo < _stepper->Steps)
		ChangeStepperUVState(_stepper, movingMinus);
	else if (_stepper->StepsToGo > _stepper->Steps)
		ChangeStepperUVState(_stepper, movingPlus);
	else
		ChangeStepperUVState(_stepper, stopped);
}



/**
 * @brief  Funkcja zatrzymuje silnik
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void StopStepper(struct Stepper *_stepper)
{
	ChangeStepperState(_stepper, stopped);
}



/**
 * @brief  Funkcja powoduje jazdę silnika w kierunku dodatnim
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void GoPlus(struct Stepper* _stepper)
{
	ChangeStepperState(_stepper, movingPlus);
}



/**
 * @brief  Funkcja powoduje jazdę silnika w kierunku ujemnym
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void GoMinus(struct Stepper* _stepper)
{
	ChangeStepperState(_stepper, movingMinus);
}



/**
 * @brief  Funkcja zmienia stan parametru isLocked struktury silnika,
 * 		   pozwala na zablokowanie lub odblokowanie możliwości wykonania ręcznego ruchu silnikiem
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @param  _state	zadany stan parametru isLocked - TRUE - zablokowany, FALSE - odblokowany
 *
 * @retval Brak
 */
void StepperLock(struct Stepper* _stepper, Cbool _state)
{
	_stepper->isLocked = _state;
}



/**
 * @brief  Funkcja zatrzymuje silnik i wyłącza jazdę do zadanej pozycji
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void HardStop(struct Stepper* _stepper)
{
	_stepper->changePosition = FALSE;
	StopStepper(_stepper);
}



/**
 * @brief  Funkcja zmienia prędkość silnika (przez zmianę okresu sygnału STEP) w zależności
 * 		   od prędkości pomiaru określonej przez użytkownika
 *
 * @param  _stepper wskaznik do zadeklarowanej struktury silnika głównego
 * @param  _speed	wybrana prędkość pomiaru
 *
 * @retval Brak
 */
void ChangeStepperSpeed(struct Stepper *_stepper, MeasurementSpeed _speed)
{
	switch (_stepper->Axis)
	{
	case X:
		switch (_speed)
		{
		case verySlow:
			_stepper->Speed = 1000;	break;
		case slow:
			_stepper->Speed = 500;	break;
		case normal:
			_stepper->Speed = 200;	break;
		case fast:
			_stepper->Speed = 100;	break;
		case manualSpeed:
			_stepper->Speed = 100;	break;
		}
		break;
	case Z:
		switch (_speed)
		{
		case verySlow:
			_stepper->Speed = 2000;	break;
		case slow:
			_stepper->Speed = 1000;	break;
		case normal:
			_stepper->Speed = 130;	break;
		case fast:
			//_stepper->StepperTimer->PSC = 44;
			_stepper->StepperTimer->CCR1 = 15;
			_stepper->Speed = 32;	break;
		case manualSpeed:
			//_stepper->StepperTimer->PSC = 0;
			_stepper->Speed = 100;
			_stepper->StepperTimer->CCR1 = 50;
			break;
		}
		break;
	case C:
		switch (_speed)
		{
		case verySlow:
			_stepper->Speed = 3000;	break;
		case slow:
			_stepper->Speed = 1000;	break;
		case normal:
			_stepper->Speed = 500;	break;
		case fast:
			_stepper->Speed = 250;	break;
		case manualSpeed:
			_stepper->Speed = 200;	break;
			break;
		}
		break;
	default:
		break;
	}
	_stepper->StepperTimer->ARR = _stepper->Speed - 1;

}



/**
 * @brief  Funkcja określa zadaną pozycję, do której ma pojechać głowica względem stołu
 * 		   w osi określonej przez _stepper. Dodatkowo parametr o zmianie pozycji silnika
 * 		   ustawiany jest na TRUE.
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika głównego
 * @param  _position	zadana pozycja, do której ma udać się głowica lub obrócić stół obrotowy
 *
 * @retval Brak
 */
void MoveStepperToPosition(struct Stepper* _stepper, double _position)
{
	_stepper->lastPosition = _stepper->currentPosition;
	_stepper->targetPosition = _position;
	_stepper->changePosition = TRUE;
}



/**
 * @brief  Funkcja odczytuje pozycję głowicy w osiach X i Z oraz kąt obrotu stołu
 * 		   na podstawie danych z enkdoerów.
 *
 * @param  Brak
 * @retval Brak
 */
void ReadCurrentPositions()
{
	//ACTUAL
	/*
	xStepper.currentPosition = (double)(X_ENCODER_TIMER->CNT) / 2000.0f;	 	// [mm]
	zStepper.currentPosition = (double)(Z_ENCODER_TIMER->CNT) / 2000.0f;		// [mm]
	cStepper.currentPosition = (double)(C_ENCODER_TIMER->CNT) * 0.018f;		// [stopnie]
	*/

	//SIMULATED
	xStepper.currentPosition = (double)(xStepper.Steps) * 0.00025f;			//mm
	zStepper.currentPosition = (double)(zStepper.Steps) * 0.000390625f;		//mm
	cStepper.currentPosition = (double)(cStepper.Steps) * 0.018f;			//STOPNIE
}



/**
 * @brief  Funkcja obsługująca ruch silników głównych.
 * 		   Porównuje pozycję docelową i rzeczywistą i na podstawie tej różnicy
 * 		   decyduje o ruchu silnika lub jego zatrzymaniu. Przy zbliżaniu się
 * 		   do celu dodatkowo funkcja zwalnia ruch silnika
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void MovementHandler(struct Stepper *_stepper)
{
	switch (_stepper->Axis)
	{
	case X:
		if (_stepper->changePosition == TRUE)
		{
			delta = _stepper->currentPosition - _stepper->targetPosition;

			//double a = (_stepper->Speed - 1000) / 0.99;
			//double b = (99901 - _stepper->Speed) / 99;

			if (fabs(delta) < 0.01)
				_stepper->StepperTimer->ARR = 999;
			/*
			else if (fabs(delta) < 1.0)
			{
				_stepper->StepperTimer->ARR = (uint16_t)(-808.0808 * delta + 1007.0808);
			}
			*/

			if (delta > 0.00015) 			//jeżeli docelowa pozycja jest w minus
				GoMinus(_stepper);
			else if (delta < -0.00015)
				GoPlus(_stepper);
			else
			{
				StopStepper(_stepper);
				_stepper->StepperTimer->ARR = _stepper->Speed - 1;
				_stepper->changePosition = FALSE;
			}
		}
		break;
	case Z:
		if (_stepper->changePosition == TRUE)
		{
			delta = _stepper->currentPosition - _stepper->targetPosition;
			//delta2 = _stepper->currentPosition - _stepper->lastPosition;
			//double a = (_stepper->Speed - 2000) / 0.92; //-2065
			//double b = (199901 - _stepper->Speed) / 92; //2171,5

			if (fabs(delta) < 0.08  && (currentMeasurement.measType != Spiral && workMode == automatic))
				_stepper->StepperTimer->ARR = 1999;
			//else if (fabs(delta2) < 1.0)
			//{
			//	_stepper->StepperTimer->ARR = (uint16_t)(a * delta2 + b);
			//}
			/*
			else if (fabs(delta) < 1.0)
			{
				_stepper->StepperTimer->ARR = (uint16_t)(-808.0808 * delta + 1007.0808);
			}
			*/

			if (delta > 0.0002) 			//jeżeli docelowa pozycja jest w minus
				GoMinus(_stepper);
			else if (delta < -0.0002)
				GoPlus(_stepper);
			else
			{
				StopStepper(_stepper);
				_stepper->StepperTimer->ARR = _stepper->Speed - 1;
				_stepper->changePosition = FALSE;
			}
		}
		break;
	case C:
		if (_stepper->changePosition == TRUE)
		{
			delta = _stepper->currentPosition - _stepper->targetPosition;
			if (fabs(delta) < 0.18)
				_stepper->StepperTimer->ARR = 2999;
			if (delta > 0.01) 			//jeżeli docelowa pozycja jest w minus
				GoMinus(_stepper);
			else if (delta < -0.01)
				GoPlus(_stepper);
			else
			{
				StopStepper(_stepper);
				_stepper->StepperTimer->ARR = _stepper->Speed - 1;
				_stepper->changePosition = FALSE;
			}
		}
		break;
	default:
		break;
	}
}




/**
 * @brief  Funkcja porównuje zadaną i aktualną pozycję głowicy
 * 		   dla danej osi i jeżeli wartości te pokrywają się zwraca
 * 		   prawdę.
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika głównego
 * @param  _position	zadana pozycja porównywana z aktualną
 *
 * @retval TRUE jeżeli pozycje zadana i rzeczywista pokrywają się, FALSE, gdy nie pokrywają się
 */
Cbool CheckPosition(struct Stepper* _stepper, double _targetPosition)
{
	delta = _stepper->currentPosition - _targetPosition;
	switch (_stepper->Axis)
	{
	case X:
		if (fabs(delta) < 0.00015)
		{
			_stepper->inPosition = TRUE;
			return TRUE;
		}
		else
		{
			_stepper->inPosition = FALSE;
			return FALSE;
		}
		break;
	case Z:
		if (fabs(delta) < 0.0002)
		{
			_stepper->inPosition = TRUE;
			return TRUE;
		}
		else
		{
			_stepper->inPosition = FALSE;
			return FALSE;
		}
		break;
	case C:
		if (fabs(delta) < 0.01)
		{
			_stepper->inPosition = TRUE;
			return TRUE;
		}
		else
		{
			_stepper->inPosition = FALSE;
			return FALSE;
		}
		break;
	default:
		break;
	}
	return FALSE;
}



/**
 * @brief  Funkcja zliczająca kroki silnika, wywoływana przy wykryciu zbocza narastającego sygnału STEP
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void StepperHandler(struct Stepper* _stepper)
{
	if (_stepper->State == movingPlus)
		_stepper->Steps++;
	else
		_stepper->Steps--;
}



/**
 * @brief  Funkcja zliczająca kroki silnika pomocniczego
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika pomocniczego
 * @retval Brak
 */
void UVCheckSteps(struct StepperUV *_stepper)
{
	_stepper->Steps++;
	if (_stepper->Steps == _stepper->StepsToGo)
		ChangeStepperUVState(_stepper, stopped);
}


/**
 * @brief  Funkcja generująca sygnał STEP silnika pomocniczego
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika pomocniczego
 * @retval Brak
 */
void UVStepperHandler(struct StepperUV *_stepper)
{
	if (_stepper->State != stopped)
	{
		_stepper->Tick++;
		if (_stepper->Tick >= _stepper->Speed)
		{
			ChangePinState(_stepper->PortStep, _stepper->PinStep, 1);
			_stepper->Tick = 0;
			UVCheckSteps(_stepper);
			return;
		}

		if (_stepper->Tick == 1)
		{
			ChangePinState(_stepper->PortStep, _stepper->PinStep, 0);
		}
	}
}









/**
 * @brief  Funkcja zmienia stan programu na "Preparing"
 *
 * @param  Brak
 * @retval Brak
 */
void StartPreparing()
{
	currentMeasurement.stage = 0;
	workMode = preparing;
}



/**
 * @brief  Funkcja obsługująca działanie programu w trybie przygotowania do pomiaru.
 * 		   Ustawia głowicę do pozycji początkowej.
 *
 * @param  Brak
 * @retval Brak
 */
void PreparingHandler()
{
	switch (currentMeasurement.measType)
	{
	case LP:
	case GD:
	case Cage:
	case Spiral:
		switch (currentMeasurement.stage)
		{
		case 0:		// Glowica stoi, sprawdz gdzie się znajduje
			SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
			if (CheckPosition(&cStepper, currentMeasurement.cStart) && CheckPosition(&zStepper, currentMeasurement.zStart))
			{
				GeneratePath();
			}
			else
			{
				currentMeasurement.stage++;
			}
			break;
		case 1:		// Glowica stoi, odjedz od czesci w X
			switch (currentMeasurement.measuredPart)
			{
			case tuleja:
				MoveStepperToPosition(&xStepper, xStepper.currentPosition - hop);
				break;
			case walek:
				MoveStepperToPosition(&xStepper, xStepper.currentPosition + hop);
				break;
			default:
				break;
			}
			currentMeasurement.stage++;
			break;
		case 2:		// Glowica jedzie w X, sprawdz czy osiagnela wymagana pozycje
			MovementHandler(&xStepper);
			if (xStepper.changePosition == FALSE)
			{
				SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
				currentMeasurement.stage++;
			}
			break;
		case 3:		// Glowica stoi, przesun w Z i C
			MoveStepperToPosition(&zStepper, currentMeasurement.zStart);
			MoveStepperToPosition(&cStepper, currentMeasurement.cStart);
			currentMeasurement.stage++;
			break;
		case 4:		// Glowica jedzie w Z i C, sprawdz czy osiagnela wymagana pozycje
			MovementHandler(&zStepper);
			MovementHandler(&cStepper);
			if (zStepper.changePosition == FALSE && cStepper.changePosition == FALSE)
			{
				SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
				currentMeasurement.stage++;
			}
			break;
		case 5:		// Glowica stoi, dojedz do czesci w X
			switch (currentMeasurement.measuredPart)
			{
			case tuleja:
				MoveStepperToPosition(&xStepper, xStepper.currentPosition + hop);
				break;
			case walek:
				MoveStepperToPosition(&xStepper, xStepper.currentPosition - hop);
				break;
			default:
				break;
			}
			currentMeasurement.stage++;
			break;
		case 6:		// Glowica jedzie w X, sprawdz czy osiagnela wymagana pozycje
			MovementHandler(&xStepper);
			if (xStepper.changePosition == FALSE)
			{
				SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
				GeneratePath();
			}
			break;
		}

		break;


	case XY:
		switch (currentMeasurement.stage)
		{
		case 0:		// Glowica stoi, sprawdz gdzie się znajduje
			if (CheckPosition(&cStepper, currentMeasurement.cStart) && CheckPosition(&xStepper, currentMeasurement.xStart))
			{
				GeneratePath();
			}
			else
			{
				currentMeasurement.stage++;
			}
			break;
		case 1:		// Glowica stoi, odjedz od powierzchni w Z
			MoveStepperToPosition(&zStepper, zStepper.currentPosition + hop);
			currentMeasurement.stage++;
			break;
		case 2:		// Glowica jedzie w Z, sprawdz czy osiagnela wymagana pozycje
			MovementHandler(&zStepper);
			if (zStepper.changePosition == FALSE)
			{
				currentMeasurement.stage++;
			}
			break;
		case 3:		// Glowica stoi, przesun sie w osiach X i C
			MoveStepperToPosition(&xStepper, currentMeasurement.xStart);
			MoveStepperToPosition(&cStepper, currentMeasurement.cStart);
			currentMeasurement.stage++;
			break;
		case 4:		// Glowica jedzie, sprawdz czy osiagnela wymagana pozycje
			MovementHandler(&xStepper);
			MovementHandler(&cStepper);
			if (zStepper.changePosition == FALSE && cStepper.changePosition == FALSE)
			{
				currentMeasurement.stage++;
			}
			break;
		case 5:		// Glowica stoi, dojedz w Z do plaszczyzny
			MoveStepperToPosition(&zStepper, zStepper.currentPosition - hop);
			currentMeasurement.stage++;
			break;
		case 6:		// Glowica jedzie w Z, sprawdz czy dojechala do pozycji
			MovementHandler(&zStepper);
			if (xStepper.changePosition == FALSE)
			{
				GeneratePath();
			}
			break;
		}

		break;
	}
}



/**
 * @brief  Funkcja oblicza liczbę punktów pomiarowych na podsawie pozycji końcowej i początkowej.
 * 		   Ustala kierunek ruchu, prędkości i rozpoczyna pomiar.
 *
 * @param  Brak
 * @retval Brak
 */
void GeneratePath()
{
	switch (currentMeasurement.measType)
	{
	case LP:
		currentMeasurement.pathPoints = ((uint32_t)(fabs(currentMeasurement.zEnd - currentMeasurement.zStart) / currentMeasurement.step) + 1) * 2;
		break;
	case GD:
		currentMeasurement.pathPoints = ((uint32_t)(fabs(currentMeasurement.cEnd - currentMeasurement.cStart) / currentMeasurement.step) + 1) * 2;
		break;
	case XY:
		currentMeasurement.pathPoints = ((uint32_t)(fabs(currentMeasurement.xEnd - currentMeasurement.xStart) / currentMeasurement.step) + 1) * 2;
		break;
	case Cage:
		currentMeasurement.pathPoints = ((uint32_t)(fabs(currentMeasurement.zEnd - currentMeasurement.zStart) / currentMeasurement.step) + 1) * 2;
		currentMeasurement.pathPoints2 = ((uint32_t)(fabs(currentMeasurement.cEnd - currentMeasurement.cStart) / currentMeasurement.step2) + 1) * 2;
		break;
	case Spiral:
		// nie dotyczy
		break;
	}
	currentMeasurement.stage = 7;
	currentMeasurement.currentPoint = 0;
	SetStepDirection();
	SetStepperSpeeds();
	StartMeasuring();
}



/**
 * @brief  Funkcja ustawia prędkości silników na czas trwania pomiaru.
 *
 * @param  Brak
 * @retval Brak
 */
void SetStepperSpeeds()
{
	// Tryb linii śrubowej - obliczenie prędkości Z w zależności od skoku i prędkości C
	if (currentMeasurement.measType == Spiral)
	{
		ChangeStepperSpeed(&xStepper, currentMeasurement.xSpeed);
		ChangeStepperSpeed(&cStepper, currentMeasurement.cSpeed);
		//uint16_t zSpiralSpeed = (uint16_t)(((double)cStepper.Speed*7.8125)/(currentMeasurement.step));
		double zSpiralSpeed = ((double)cStepper.Speed*7.8125)/(currentMeasurement.step);

		if (((cStepper.Speed == 250) && fabs(currentMeasurement.step - 0.2f) < 0.001)
				|| ((cStepper.Speed == 250) && fabs(currentMeasurement.step - 1.0f) < 0.001))
		{
			zStepper.StepperTimer->PSC = 179;
			zStepper.Speed = (uint16_t)(zSpiralSpeed * 2);
			zStepper.StepperTimer->ARR = zStepper.Speed - 1;
		}
		else
		{
			zStepper.StepperTimer->PSC = 56249;
			zStepper.Speed = (uint16_t)(zSpiralSpeed / 156.25);
			zStepper.StepperTimer->ARR = zStepper.Speed - 1;
		}

		return;
	}

	ChangeStepperSpeed(&xStepper, currentMeasurement.xSpeed);
	ChangeStepperSpeed(&zStepper, currentMeasurement.zSpeed);
	ChangeStepperSpeed(&cStepper, currentMeasurement.cSpeed);
}



/**
 * @brief  Funkcja zmienia stan programu na "Automatic"
 *
 * @param  Brak
 * @retval Brak
 */
void StartMeasuring()
{
	workMode = automatic;
	SendCurrentPoint(1);
}




// #############################################################################################
// ###############################  STRATEGIE POMIAROWE  #######################################
// #############################################################################################



/**
 * @brief  Funkcja obsługująca działanie programu w trybie automatycznym,
 * 		   wykonuje badanie według strategii przekrojów poprzecznych
 *
 * @param  Brak
 * @retval Brak
 */
void AutomaticHandlerLP()
{
	switch (currentMeasurement.stage)
	{
	// Glowica w punkcie poczatkowym Z i C
	case 7:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w cStart, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&cStepper, currentMeasurement.cEnd);
			currentMeasurement.stage++;
		}
		break;

	case 8:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w cStart lub cEnd, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			if (currentMeasurement.currentPoint >= currentMeasurement.pathPoints)
			{
				StopMeasuring();
			}
			else
			{
				SendCurrentPoint(currentMeasurement.currentPoint);
				MoveStepperToPosition(&zStepper, zStepper.currentPosition + currentMeasurement.step);
				if (currentMeasurement.currentPoint % 4 == 0)
					currentMeasurement.stage--;
				else
					currentMeasurement.stage++;
			}

		}
		break;
	case 9:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w cEnd, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&cStepper, currentMeasurement.cStart);
			currentMeasurement.stage--;
		}
		break;
	}
}




/**
 * @brief  Funkcja obsługująca działanie programu w trybie automatycznym,
 * 		   wykonuje badanie według strategii tworzących
 *
 * @param  Brak
 * @retval Brak
 */
void AutomaticHandlerGD()
{
	switch (currentMeasurement.stage)
	{
	// Glowica w punkcie poczatkowym Z i C
	case 7:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w zStart, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&zStepper, currentMeasurement.zEnd);
			currentMeasurement.stage++;
		}
		break;

	case 8:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w zStart lub zEnd, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			if (currentMeasurement.currentPoint >= currentMeasurement.pathPoints)
			{
				StopMeasuring();
			}
			else
			{
				SendCurrentPoint(currentMeasurement.currentPoint);
				MoveStepperToPosition(&cStepper, cStepper.currentPosition + currentMeasurement.step);
				if (currentMeasurement.currentPoint % 4 == 0)
					currentMeasurement.stage--;
				else
					currentMeasurement.stage++;
			}

		}
		break;
	case 9:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w zEnd, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&zStepper, currentMeasurement.zStart);
			currentMeasurement.stage--;
		}
		break;
	}
}



/**
 * @brief  Funkcja obsługująca działanie programu w trybie automatycznym,
 * 		   wykonuje badanie według strategii linii śrubowej
 *
 * @param  Brak
 * @retval Brak
 */
void AutomaticHandlerSpiral()
{
	switch (currentMeasurement.stage)
	{
	// Glowica w punkcie poczatkowym Z i C
	case 7:
		if (currentMeasurement.direction == 1)
			GoPlus(&cStepper);
		else
			GoMinus(&cStepper);

		MoveStepperToPosition(&zStepper, currentMeasurement.zEnd);
		currentMeasurement.stage++;
		break;

	case 8:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)
		{
			StopMeasuring();
		}
		break;
	}
}



/**
 * @brief  Funkcja obsługująca działanie programu w trybie automatycznym,
 * 		   wykonuje badanie według strategii klatki
 *
 * @param  Brak
 * @retval Brak
 */
void AutomaticHandlerCage()
{
	switch (currentMeasurement.stage)
	{
	// najpierw LP
	// Glowica w punkcie poczatkowym Z i C
	case 7:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w cStart, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&cStepper, currentMeasurement.cEnd);
			currentMeasurement.stage++;
		}
		break;

	case 8:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w cStart lub cEnd, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			if (currentMeasurement.currentPoint >= currentMeasurement.pathPoints)
			{
				currentMeasurement.currentPoint = 0;
				if (fabs(cStepper.currentPosition - currentMeasurement.cEnd) < 0.001)		//głowica kończy w cEnd
					currentMeasurement.stage = 17;
				else																		//głowica kończy w cStart
				{
					currentMeasurement.step2 = -currentMeasurement.step2;
					currentMeasurement.stage = 19;
				}
			}
			else
			{
				SendCurrentPoint(currentMeasurement.currentPoint);
				MoveStepperToPosition(&zStepper, zStepper.currentPosition + currentMeasurement.step);
				if (currentMeasurement.currentPoint % 4 == 0)
					currentMeasurement.stage--;
				else
					currentMeasurement.stage++;
			}

		}
		break;
	case 9:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w cEnd, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&cStepper, currentMeasurement.cStart);
			currentMeasurement.stage--;
		}
		break;

	// Po zakonczeniu LP - GD od konca
	case 17:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w zEnd, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&zStepper, currentMeasurement.zStart);
			currentMeasurement.stage++;
		}
		break;

	case 18:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)		//Glowica w zStart lub zEnd, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			if (currentMeasurement.currentPoint >= currentMeasurement.pathPoints2)
			{
				StopMeasuring();
			}
			else
			{
				SendCurrentPoint(currentMeasurement.currentPoint + currentMeasurement.pathPoints);
				MoveStepperToPosition(&cStepper, cStepper.currentPosition + currentMeasurement.step2);
				if (currentMeasurement.currentPoint % 4 == 0)
					currentMeasurement.stage--;
				else
					currentMeasurement.stage++;
			}

		}
		break;
	case 19:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w zStart, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&zStepper, currentMeasurement.zEnd);
			currentMeasurement.stage--;
		}
		break;
	}
}




/**
 * @brief  Funkcja obsługująca działanie programu w trybie automatycznym,
 * 		   wykonuje badanie płaszczyzny
 *
 * @param  Brak
 * @retval Brak
 */
void AutomaticHandlerXY()
{
	switch (currentMeasurement.stage)
	{
	// Glowica w punkcie poczatkowym X i C
	case 7:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w xStart, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&xStepper, currentMeasurement.xEnd);
			currentMeasurement.stage++;
		}
		break;

	case 8:
		MovementHandler(&xStepper);
		if (xStepper.changePosition == FALSE)		//Glowica w xStart lub xEnd, skonczyla jazde w Z
		{
			currentMeasurement.currentPoint++;
			if (currentMeasurement.currentPoint >= currentMeasurement.pathPoints)
			{
				StopMeasuring();
			}
			else
			{
				SendCurrentPoint(currentMeasurement.currentPoint);
				MoveStepperToPosition(&cStepper, cStepper.currentPosition + currentMeasurement.step);
				if (currentMeasurement.currentPoint % 4 == 0)
					currentMeasurement.stage--;
				else
					currentMeasurement.stage++;
			}

		}
		break;
	case 9:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)		//Glowica w xEnd, skonczyla jazde w C
		{
			currentMeasurement.currentPoint++;
			MoveStepperToPosition(&xStepper, currentMeasurement.xStart);
			currentMeasurement.stage--;
		}
		break;

	}
}





// ##################################################################################################







/**
 * @brief  Funkcja resetuje pozycję docelową głowicy w osi określonej przez _stepper
 *
 * @param  _stepper 	wskaznik do zadeklarowanej struktury silnika głównego
 * @retval Brak
 */
void ResetTargetPositions(struct Stepper* _stepper)
{
	_stepper->changePosition = FALSE;
	_stepper->targetPosition = _stepper->currentPosition;
}



/**
 * @brief  Funkcja wykonuje procedurę zakończenia pomiaru.
 * 		   Zatrzymuje wszystkie silniki, resetuje ich parametru i przywraca prędkości z trybu ręcznego
 *
 * @param  Brak
 * @retval Brak
 */
void StopMeasuring()
{

	ChangeStepperState(&xStepper, stopped);
	ChangeStepperState(&zStepper, stopped);
	ChangeStepperState(&cStepper, stopped);

	SendCurrentPoint(currentMeasurement.currentPoint);

	ResetTargetPositions(&xStepper);
	ResetTargetPositions(&zStepper);
	ResetTargetPositions(&cStepper);

	ChangeStepperUVState(&uStepper, stopped);
	ChangeStepperUVState(&vStepper, stopped);

	ChangeStepperSpeed(&xStepper, manualSpeed);
	ChangeStepperSpeed(&zStepper, manualSpeed);
	ChangeStepperSpeed(&cStepper, manualSpeed);


	currentMeasurement.pathPoints = 0;
	currentMeasurement.pathPoints2 = 0;
	currentMeasurement.currentPoint = 0;

	xStepper.isLocked = FALSE;
	zStepper.StepperTimer->PSC = 179;


	workMode = manual;
	//SendCurrentPosition(xStepper.currentPosition, zStepper.currentPosition, cStepper.currentPosition);
}




/**
 * @brief  Funkcja oblicza pozycję osi przedmiotu walcowego względem stołu obrotowego
 * 		   i na podstawie wyniku wykonuje ruch za pomocą silników pomocniczych U i V
 *
 * @param  _cal		wskaźnik do struktury Calibration zawierającej dane zebrane podczas pomiaru
 * @retval Brak
 */
void AlignAxis(struct Calibration *_cal)
{
	double u[3];
	double v[3];
	for (int i = 0; i < 3; i++)
	{
		u[i] = (_cal->xData[i] + ((double)_cal->adcRead[i]/4096.0) - 1.0) * cos(_cal->cData[i]*PI/180.0);
		v[i] = (_cal->xData[i] + ((double)_cal->adcRead[i]/4096.0) - 1.0) * sin(_cal->cData[i]*PI/180.0);
	}
	double u01 = u[0] - u[1];
	double u02 = u[0] - u[2];
	double v01 = v[0] - v[1];
	double v02 = v[0] - v[2];
	double su02 = u[0]*u[0] - u[2]*u[2];
	double sv02 = v[0]*v[0] - v[2]*v[2];
	double su10 = u[1]*u[1] - u[0]*u[0];
	double sv10 = v[1]*u[1] - v[0]*u[0];

	_cal->offsetU = -(su02*u01 + sv02*u01 + su10*u02 + sv10*u02) / (2* (-v02)*u01 - (-v01)*u02);
	_cal->offsetV = -(su02*v01 + sv02*v01 + su10*v02 + sv10*v02) / (2* (-u02)*v01 - (-u01)*v02);

	MoveUVStepper(&uStepper, _cal->offsetU);
	MoveUVStepper(&vStepper, _cal->offsetV);
}



/**
 * @brief  Funkcja obsługująca działanie stanowiska w trybie ustawiania osi,
 * 		   pozwala na zebranie 3 punktów pomiarowych na przedmiocie walcowym i po wykonaniu pomiaru
 * 		   wykonywane są oblioczenia przez wywołanie funkcji AlignAxis()
 *
 * @param  _cal		wskaźnik do struktury Calibration zawierającej dane zebrane podczas pomiaru
 * @retval Brak
 */
void AxisPositionCalibrationHandler()
{
	static uint16_t calPoint = 0;
	switch (axisCalibration.stage)
	{
	case 1:
		MoveStepperToPosition(&zStepper, axisCalibration.height);
		axisCalibration.stage++;
		break;
	case 2:
		MovementHandler(&zStepper);
		if (zStepper.changePosition == FALSE)
		{
			MoveStepperToPosition(&xStepper, axisCalibration.diameter);
			axisCalibration.stage++;
		}
		break;
	case 3:
		MovementHandler(&xStepper);
		if (xStepper.changePosition == FALSE)
		{
			axisCalibration.xData[calPoint] = xStepper.currentPosition;
			axisCalibration.zData[calPoint] = zStepper.currentPosition;
			axisCalibration.cData[calPoint] = cStepper.currentPosition;
			axisCalibration.adcRead[calPoint] = adcData;
			calPoint++;
			if (calPoint >= 3)
			{
				switch (axisCalibration.partType)
				{
				case tuleja:
					MoveStepperToPosition(&xStepper, xStepper.currentPosition - 4*hop);
					break;
				case walek:
					MoveStepperToPosition(&xStepper, xStepper.currentPosition + 4*hop);
					break;
				default:
					break;
				}
				axisCalibration.stage = 6;
			}
			else
			{
				switch (axisCalibration.partType)
				{
				case tuleja:
					MoveStepperToPosition(&xStepper, xStepper.currentPosition - 2*hop);
					break;
				case walek:
					MoveStepperToPosition(&xStepper, xStepper.currentPosition + 2*hop);
					break;
				default:
					break;
				}
				axisCalibration.stage++;
			}

		}
		break;
	case 4:
		MovementHandler(&xStepper);
		if (xStepper.changePosition == FALSE)
		{
			MoveStepperToPosition(&cStepper, cStepper.currentPosition + 120.0);
			axisCalibration.stage++;
		}
		break;
	case 5:
		MovementHandler(&cStepper);
		if (cStepper.changePosition == FALSE)
		{
			MoveStepperToPosition(&xStepper, axisCalibration.diameter);
			axisCalibration.stage = 3;
		}
		break;
	case 6:
		AlignAxis(&axisCalibration);
		axisCalibration.stage++;
		break;
	case 7:

		break;
	}
}










/**
 * @brief  Główna funkcja obsługjąca działanie programu, steruje wiadomościami, ADC, enkdoerami,
 * 		   silnikami w zależnosci od trybu pracy
 *
 * @param  Brak
 * @retval Brak
 */
void MainFunction()
{
	counterMsg++;
	if (counterMsg >= 100)
	{
		ChangePinState(TX_FLAG_GPIO_Port, TX_FLAG_Pin, 1);
		MessageHandler();
		counterMsg = 0;
	}
	if (counterMsg == 50)
		ChangePinState(TX_FLAG_GPIO_Port, TX_FLAG_Pin, 0);

	if (wasInitialized == TRUE)
	{
		ReadCurrentPositions();
		ADCHandler();

		switch (workMode)
		{
		case manual:
			MovementHandler(&xStepper);
			MovementHandler(&zStepper);
			MovementHandler(&cStepper);
			break;

		case preparing:
			PreparingHandler();
			break;

		case automatic:
			switch (currentMeasurement.measType)
			{
			case LP:
				AutomaticHandlerLP();
				break;
			case GD:
				AutomaticHandlerGD();
				break;
			case XY:
				AutomaticHandlerXY();
				break;
			case Spiral:
				AutomaticHandlerSpiral();
				break;
			case Cage:
				AutomaticHandlerCage();
				break;
			}
			break;

		case calibration:
			AxisPositionCalibrationHandler();
			break;

		default:
			break;
		}
	}


}
