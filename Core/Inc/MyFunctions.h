/*
 * MyFunctions.h
 * Plik naglowkowy
 *
 * autor: Adam Masalski
 */

#include "tim.h"
#include "gpio.h"
#include "adc.h"

/**
 * Definicje licznikow
 */
#define MAIN_TIMER			TIM6
#define MAIN_TIMER_HANDLE	htim6

#define UV_TIMER				TIM7
#define UV_TIMER_HANDLE			htim7

#define X_TIMER					TIM10
#define X_TIMER_HANDLE			htim10
#define X_ENCODER_TIMER			TIM1
#define X_ENCODER_TIMER_HANDLE	htim1

#define Z_TIMER					TIM11
#define Z_TIMER_HANDLE			htim11
#define Z_ENCODER_TIMER			TIM2
#define Z_ENCODER_TIMER_HANDLE	htim2

#define C_TIMER					TIM13
#define C_TIMER_HANDLE			htim13
#define C_ENCODER_TIMER			TIM3
#define C_ENCODER_TIMER_HANDLE	htim3

#define NUMBER_OF_STEPPERS		5

#define hop						5.0

#define PI						3.14159265

/**
 * Utworzenie zmiennej Bool w jezyku C
 */
typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
} Cbool;


/**
 * Enumerator trybu pracy stanowiska
 * manual 		- 	tryb reczny
 * automatic 	- 	tryb automatyczny
 * preparing 	- 	tryb przygotowania do badania
 */
typedef enum
{
	manual, automatic, preparing, calibration, error
} Mode;


/**
 * Enumerator typu badania
 * LP 		- 	badanie walcowosci: metoda przekrojow poprzecznych
 * GD 		- 	badanie walcowosci: metoda tworzacych
 * Spiral 	- 	badanie walcowosci: metoda linii srubowej
 * Cage 	- 	badanie walcowosci: metoda klatki
 * XY 		- 	badanie plaszczyzn
 */
typedef enum
{
	LP, GD, Spiral, Cage, XY
} MeasurementType;


/**
 * Enumerator szybkosci pomiaru
 */
typedef enum
{
	verySlow, slow, normal, fast, manualSpeed
} MeasurementSpeed;


/**
 * Enumerator rodzaju badanej czesci
 */
typedef enum
{
	tuleja, walek, plaszczyzna
} PartType;


/**
 * Enumerator osi stanowiska
 */
typedef enum
{
	X, Z, C, U, V
} StepperAxis;


/**
 * Enumerator stanu silnika
 * stopped 		- silnik zatrzymany
 * movingPlus 	- silnik w ruchu, do przodu
 * movingMinus 	- silnik w ruchu, do tylu
 */
typedef enum
{
	stopped, movingPlus, movingMinus
} StepperState;



//
// speed1 - wolno
// speed2 - srednio
// speed3 - szybko
/**
 * Enumerator szybkosci silnikow krokowych
 * Wyrazony w mikrosekundach pomiedzy kolejnymi krokami
 */
typedef enum
{
	speed1 = 1000, speed2 = 600, speed3 = 300, speed4 = 100
} StepperSpeed;


/**
 * Struktura silnika krokowego
 * Zawiera wszystkie informacje na temat danego silnika i jego stanu
 */
struct Stepper
{
	TIM_TypeDef* StepperTimer;		// licznik silnika
	TIM_HandleTypeDef TimerHandle;	//

	uint16_t PinStep;				// pin STEP
	GPIO_TypeDef* PortDir;			// port DIR
	uint16_t PinDir;				// pin DIR

	StepperAxis Axis;				// os silnika
	StepperState State;				// obecny stan silnika
	StepperState LastState;			// poprzedni stan silnika

	uint16_t Speed;					// predkosc silnika
	int32_t Steps;					// liczba wykonanych krokow

	TIM_TypeDef* Encoder;			// licznik enkodera

	double currentPosition;			// obecna pozycja
	double lastPosition;			// ostatnia pozycja
	double targetPosition;			// zadana pozycja
	double lastSentPosition;		// ostatnia zadana pozycja


	Cbool changePosition;			// czy zmienic pozycje
	Cbool inPosition;				// czy silnik osiagnal pozycje

	Cbool isLocked;					// czy silnik jest zablokowany
};


/**
 * Struktura silnika pomocniczego U, V
 */
struct StepperUV
{
	GPIO_TypeDef* PortStep;
	uint16_t PinStep;

	GPIO_TypeDef* PortDir;
	uint16_t PinDir;

	StepperAxis Axis;
	StepperState State;
	StepperState LastState;

	StepperSpeed Speed;
	uint16_t Tick;

	int32_t Steps;
	int32_t StepsToGo;
};


/**
 * Struktura pomiaru
 * Zawiera dane na temat szczegolow aktualnego badania
 */
struct Measurement
{
	MeasurementType measType;	// typ badania
	MeasurementSpeed xSpeed;	// predkosc w osi X
	MeasurementSpeed zSpeed;	// predkosc w osi Z
	MeasurementSpeed cSpeed;	// predkosc w osi C
	PartType measuredPart;		// typ badanej czesci
	double step;					// skok pomiaru
	double step2;				//
	double xStart;				// start w X
	double zStart;				// start w Z
	double cStart;				// start w C
	double xEnd;					// koniec w X
	double zEnd;					// koniec w Z
	double cEnd;					// koniec w C

	uint16_t stage;				// aktualny etap badania
	uint32_t pathPoints;		// punkty pomiaru
	uint32_t pathPoints2;		//
	unsigned int currentPoint;		// aktualny punkt pomiaru

	uint16_t direction;			// kierunek ruchu
};

/**
 * Struktura ustawiania osi
 * Zawiera dane na temat pomiaru pozycji osi części względem stołu
 */
struct Calibration
{
	double xData[3];
	double zData[3];
	double cData[3];
	uint32_t adcRead[3];

	uint16_t stage;
	PartType partType;
	double diameter;
	double height;

	double offsetU;
	double offsetV;
};


/* Podstawowe funkcje */
void ChangePinState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int _state);
void InitializeComponents();
void SetStep(int _step);
void SetStepDirection();
void ADCHandler();



/* Funkcje silnikow krokowych */
void StepperInit(struct Stepper* _stepper, TIM_TypeDef* StepperTimer, TIM_HandleTypeDef TimerHandle, uint16_t PinStep, GPIO_TypeDef* PortDir, uint16_t PinDir, StepperAxis Axis, TIM_TypeDef* Encoder);
void StepperUVInit(struct StepperUV* _stepper, GPIO_TypeDef* PortStep, uint16_t PinStep, GPIO_TypeDef* PortDir, uint16_t PinDir, StepperAxis Axis);
void InitializeSteppers();

void ChangeStepperState(struct Stepper *_stepper, StepperState _state);
void ChangeStepperUVState(struct StepperUV *_stepper, StepperState _state);
void MoveUVStepper(struct StepperUV *_stepper, double _target);

void StopStepper(struct Stepper *_stepper);
void GoPlus(struct Stepper* _stepper);
void GoMinus(struct Stepper* _stepper);
void StepperLock(struct Stepper* _stepper, Cbool _state);
void HardStop(struct Stepper* _stepper);

void ChangeStepperSpeed(struct Stepper *_stepper, MeasurementSpeed _speed);
void MoveStepperToPosition(struct Stepper* _stepper, double _position);
void ReadCurrentPositions();
void MovementHandler(struct Stepper *_stepper);

void StepperHandler(struct Stepper* _stepper);
void UVCheckSteps(struct StepperUV *_stepper);
void UVStepperHandler(struct StepperUV *_stepper);


/* Pomiary */
Cbool CheckPosition(struct Stepper* _stepper, double _targetPosition);
void StartPreparing();
void PreparingHandler();
void GeneratePath();
void SetStepperSpeeds();

void StartMeasuring();
void AutomaticHandlerLP();
void AutomaticHandlerGD();
void AutomaticHandlerSpiral();
void AutomaticHandlerCage();
void AutomaticHandlerXY();
void ResetTargetPositions(struct Stepper* _stepper);
void StopMeasuring();

void AlignAxis(struct Calibration *_cal);
void AxisPositionCalibrationHandler();


void MainFunction();











