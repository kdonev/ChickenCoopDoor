#include <CircularBuffer.h>

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include <RTClib.h>
#include <math.h>

RTC_DS1307 rtc;

//STANDARD CONSTANTS
double pi = 3.1415926535;   // Pi
double solarConst = 1367;           // solar constant W.m-2

// sofia
double rlat = 42.671 * pi/180;
double rlong = 23.233 * pi/180;
double g_localTimeAdjust = 3;

char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

const int motorDirpin = 4;
const int motorPwmpin = 5;
const int motorSpeed = 96;

const int buttonUpPin = 9;
const int buttonDownPin = 10;

const int btnDownStopPin = 11;
const int btnUpStopPin = 12;

typedef long DelayMs;

const DelayMs maxDelayForState = 5000; // in miliseconds

const DelayMs maxDoorCloseMoveTime = 15000L;
const DelayMs maxDoorOpenMoveTime = 80000L;

// both delays are in minutes and can be negative too.
const int closeDelayAfterSunset = 15;
const int openDelayAfterSunrise = 0;

enum State
{
	Unknown,
	Down,
	Up,
	MoveUp,
	MoveDown,
    Error
};

State currentState = Unknown;
bool currentStateIsFromButton = false;

const int buttonsMode = INPUT_PULLUP;

// both are in minutes from 0:00
int nextCloseTime;
int nextOpenTime;

long maxDoorMoveTime = maxDoorOpenMoveTime;
long doorMovedTime;

struct LogEvent
{
    DateTime time;
    const char* event;
};

CircularBuffer<LogEvent, 50> events;

void logEvent(const char* event)
{
    Serial.println(event);
    events.push({rtc.now(), event});
}

void printEventLog()
{
    using index_t = decltype(events)::index_t;
    for (index_t i = 0; i < events.size(); i++) 
    {
        printTime("", events[i].time, events[i].event);
    }
}

void setup() {
  while (!Serial); // for Leonardo/Micro/Zero

  Serial.begin(9600);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    
  // put your setup code here, to run once:
  pinMode(motorDirpin, OUTPUT);

  pinMode(buttonUpPin, buttonsMode);
  pinMode(buttonDownPin, buttonsMode);
  pinMode(btnDownStopPin, buttonsMode);
  pinMode(btnUpStopPin, buttonsMode);
  
  logEvent("Started");

  stopMotor();
  calculateTimesForTomorrow();
}

void calculateTimesForTomorrow()
{
    logEvent("Calculating times for tommorow.");
  Serial.println("calculateTimesForTomorrow");
 	DateTime now = rtc.now();
  printTime("Now is ", now);
	DateTime tomorrow (now + TimeSpan(0, 24, 0, 0));
  printTime("Tomorrow is ", tomorrow);

	double sunriseLocalTime = CalclulateSunriseLocalTime(tomorrow.day(), tomorrow.month(), tomorrow.year(), rlong, rlat);
	nextOpenTime = int(sunriseLocalTime * 60) + openDelayAfterSunrise;
  Serial.print("Next open time ");
  Serial.println(nextOpenTime);
  
	double sunsetLocalTime = CalculateSunsetLocalTime(tomorrow.day(), tomorrow.month(), tomorrow.year(), rlong, rlat);
	nextCloseTime = int(sunsetLocalTime * 60) + closeDelayAfterSunset;
   Serial.print("Next close time ");
  Serial.println(nextCloseTime);
}

int nowInMinutes()
{
	DateTime now = rtc.now();
	return now.hour() * 60 + now.minute();
}

long nowInSeconds()
{
	DateTime now = rtc.now();
	return now.secondstime();
}

bool isPressed(int buttonPin)
{
	int state = digitalRead(buttonPin);
	return state == LOW;
}

void stopMotor()
{
    analogWrite(motorPwmpin, 0);

    logEvent("Stoped the motor");
}

void startMoveUp()
{
    bool pressed = isPressed(btnUpStopPin);
    if (!pressed)
    {
        digitalWrite(motorDirpin, HIGH);
        analogWrite(motorPwmpin, motorSpeed);
        doorMovedTime = 0;
        maxDoorMoveTime = maxDoorOpenMoveTime;

        logEvent("Started opening");
    }
    else
    {
        logEvent("Already opened");
    }
    
}

void startMoveDown()
{
    bool pressed = isPressed(btnDownStopPin);
    if (!pressed)
    {
        digitalWrite(motorDirpin, LOW);
        analogWrite(motorPwmpin, motorSpeed);
        doorMovedTime = 0;
        maxDoorMoveTime = maxDoorCloseMoveTime;
        logEvent("Started closing");
    }
    else
    {
        logEvent("Already closed");
    }    
}

void checkMoveTimeout()
{
    if(doorMovedTime > maxDoorMoveTime)
    {
        stopMotor();
        logEvent("Motor stoped by timeout");
        currentState = Error;
    }
}

void checkDown()
{
    // logEvent("checkDown");
	bool pressed = isPressed(btnDownStopPin);
	if(pressed)
	{
        logEvent("Door is down.");
		// keep moving so the wheel can go little more
		delay(1000);
		stopMotor();
		currentState = Down;
	}
    else
    {
        checkMoveTimeout();
    }
    
}

void checkUp()
{
	bool pressed = isPressed(btnUpStopPin);
	if( pressed )
	{
        stopMotor();
        logEvent("Door is up.");
		currentState = Up;
	}
    else
    {
        checkMoveTimeout();
    }
    
}

void openTheDoor()
{
	startMoveUp();
	currentState = MoveUp;
}

void closeTheDoor()
{
	startMoveDown();
	currentState = MoveDown;
	
	calculateTimesForTomorrow();
}

DelayMs doUnknownState()
{
    // printDebugInfo();

	checkDown();
	checkUp();
	
	if(currentState == Unknown)
		openTheDoor();

    return 0;
}

int minutesDiff(int from, int to)
{
    const int oneDayInMinutes = 24*60;
    while (to < from) to += oneDayInMinutes;
    return to - from;
}

DelayMs toMs(int minutes)
{
    return DelayMs(minutes) * 60 * 1000;
}

DelayMs timeToOpen()
{
	int now = nowInMinutes();
	if ( nextOpenTime < now && now < nextCloseTime )
        return 0;
    else
        return toMs( minutesDiff(now, nextOpenTime) );
}

DelayMs timeToClose()
{
	int now = nowInMinutes();
	if ( now < nextOpenTime || nextCloseTime < now )
        return 0;
    else
        return toMs( minutesDiff(now, nextCloseTime) );
}

DelayMs doDown()
{
    // logEvent("doDown");
    DelayMs tto;
    if (currentStateIsFromButton)
    {
        tto = timeToClose();
        if (tto == 0)
            currentStateIsFromButton = false;
    }
    else
    {
        tto = timeToOpen();
        if (tto == 0)
            openTheDoor();
    }

    return tto;
}

DelayMs doUp()
{
    // logEvent("doUp");
    DelayMs ttc;
    if (currentStateIsFromButton)
    {
        ttc = timeToOpen();
        if (ttc == 0)
            currentStateIsFromButton = false;
    }
    else
    {
        ttc = timeToClose();
        if (ttc == 0)
            closeTheDoor();
    }

    return ttc;
}

DelayMs doMoveUp()
{
	checkUp();
    return 10;
}

DelayMs doMoveDown()
{
	checkDown();
    return 10;
}

DelayMs doState()
{
    DelayMs delay = 100;
	switch(currentState)
	{
		case Unknown:
			delay = doUnknownState();
		break;
		
		case Down:
			delay = doDown();
		break;
		
		case Up:
			delay = doUp();
		break;
		
		case MoveUp:
			delay = doMoveUp();
		break;
		
		case MoveDown:
			delay = doMoveDown();
		break;
        
        case Error:
            delay = maxDelayForState;
        break;
	}

    delay = min(delay, maxDelayForState);
    return delay;
}

void printTime(const char* msg, DateTime t)
{
  printTime(msg, t, nullptr);
}

void printTime(const char* msg, DateTime t, const char* msgAfter)
{
    Serial.print(msg);
    Serial.print(t.year(), DEC);
    Serial.print('/');
    Serial.print(t.month(), DEC);
    Serial.print('/');
    Serial.print(t.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[t.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(t.hour(), DEC);
    Serial.print(':');
    Serial.print(t.minute(), DEC);
    Serial.print(':');
    Serial.print(t.second(), DEC);
    if (msgAfter)
    {
        Serial.print(" ");
        Serial.print(msgAfter);
    }
    Serial.println();
}

void printCurrentState()
{
    Serial.print("Current state is ");
    switch(currentState)
	{
		case Unknown:
			Serial.println("Unknown");
		break;
		
		case Down:
			Serial.println("closed");
		break;
		
		case Up:
			Serial.println("opened");
		break;
		
		case MoveUp:
			Serial.println("opening");
		break;
		
		case MoveDown:
			Serial.println("closing");
		break;

        case Error:
			Serial.println("ERROR");
		break;

        default:
            Serial.println("!!!BROKEN!!!");
		break;
	}

    if(currentStateIsFromButton)
        Serial.println("Current state is from button");
}

void printMsDelay(const char* msg, DelayMs d)
{
    Serial.print(msg);
    Serial.print(d);
    Serial.print(" ");

    TimeSpan ts(d / 1000);

    Serial.print(ts.days(), DEC);
    Serial.print("d ");
    Serial.print(ts.hours(), DEC);
    Serial.print("h ");
    Serial.print(ts.minutes(), DEC);
    Serial.print("m ");
    Serial.print(ts.seconds(), DEC);
    Serial.print("s ");
    Serial.print(d % 1000, DEC);
    Serial.print("ms ");

    Serial.println();
}

void printButtonState(const char* btnName, int btnPin)
{
  Serial.print(btnName);
  Serial.print(" is ");
  Serial.println(isPressed(btnPin)?"pressed":"released");
}

void printButtonsStates()
{
  printButtonState("Close button", buttonDownPin);
  printButtonState("Open button", buttonUpPin);
  printButtonState("Door up sensor", btnUpStopPin);
  printButtonState("Door down sensor", btnDownStopPin);
}

void printDebugInfo()
{
    Serial.println();
    Serial.println();
    Serial.println("---------------- Events Log -----------------------------------");
    printEventLog();

    Serial.println("---------------- States info ----------------------------------");

    DateTime now = rtc.now();
    printTime("Now is ", now);
    DateTime midnight = now - TimeSpan(0, now.hour(), now.minute(), now.second());
    printTime("Midnight is ", midnight);

    printCurrentState();

    printButtonsStates();

    DateTime openTime (midnight + TimeSpan(0, nextOpenTime/60, nextOpenTime%60, 0));
    printTime("Next open time is ", openTime);
    
    DateTime closeTime (midnight + TimeSpan(0, nextCloseTime/60, nextCloseTime%60, 0));
    printTime("Next close time is ", closeTime);

    printMsDelay("Delay to next open ", timeToOpen());
    printMsDelay("Delay to next close ", timeToClose());
}

void checkForDebugPrint()
{
    if (isPressed(buttonDownPin) && isPressed(buttonUpPin))
    {
        printDebugInfo();
    }
}

void checkButtons()
{
    if (isPressed(buttonDownPin) && !isPressed(buttonUpPin) && currentState != MoveDown)
    {
        logEvent("Handle close button");
        closeTheDoor();
        currentStateIsFromButton = true;
    }
    else if (!isPressed(buttonDownPin) && isPressed(buttonUpPin) && currentState != MoveUp)
    {
        logEvent("Handle open button");
        openTheDoor();
        currentStateIsFromButton = true;
    }
}

void loop() {
    // printDebugInfo();

    checkForDebugPrint();

    checkButtons();

    DelayMs stateDelay = doState();
    delay(stateDelay);
    doorMovedTime += stateDelay;
}

// Function to convert radian to hours
double RadToHours (double tmp)
{
    //double pi = 3.1415926535; // Pi
    return (tmp * 12 / pi);
}
// Function to convert hours to radians
double HoursToRads (double tmp)
{
    //double pi = 3.1415926535; // Pi
    return (tmp * pi / 12);
}




// Function to calculate the angle of the day
double AngleOfDay (int day,     // number of the day 
                   int month,   // number of the month
                   int year // year 
                 )

{   // local vars
    int i, leap;
    int numOfDays = 0;                                              // number of Day 13 Nov=317
    int numOfDaysofMonths[12] = {0,31,28,31,30,31,30,31,31,30,31,30};   // Number of days per month
    int AllYearDays;                                                // Total number of days in a year 365 or 366
    double DayAngle;                                                // angle of the day (radian)
    //double pi = 3.1415926535; // Pi

    // leap year ??
    leap = 0;
    if ((year % 400)==0) 
    {   AllYearDays = 366;
        leap = 1;
    }
    else if ((year % 100)==0) AllYearDays = 365;
         else if ((year % 4)==0)
            {   AllYearDays = 366;
                leap = 1;
            }
             else AllYearDays = 365;

    // calculate number of day
    for (i=0;i<month;i++) numOfDays += numOfDaysofMonths[i];
    if ( (month > 2) && leap) numOfDays++;
    numOfDays += day;

    // calculate angle of day
    DayAngle = (2*pi*(numOfDays-1)) / AllYearDays;
    return DayAngle;

}


// Function to calculate declination - in radian
double Declination (double DayAngle     // angle day in radian
                    )
{
    double SolarDeclination;
    // Solar declination (radian)
    SolarDeclination = 0.006918 
            - 0.399912 * cos (DayAngle)
            + 0.070257 * sin (DayAngle)
            - 0.006758 * cos (2*DayAngle)
            + 0.000907 * sin (2*DayAngle)
            - 0.002697 * cos (3*DayAngle)
            + 0.00148 * sin (3*DayAngle);
    return SolarDeclination;
}



// Function to calculate Equation of time ( et = TSV - TU )
double EqOfTime (double DayAngle        // angle day (radian)
                      )
{
    double et;
    // Equation of time (radian)
    et = 0.000075
         + 0.001868 * cos (DayAngle)
         - 0.032077 * sin (DayAngle)
         - 0.014615 * cos (2*DayAngle)
         - 0.04089 * sin (2*DayAngle);
    // Equation of time in hours
    et = RadToHours(et);

    return et;
}

// Calculation of the duration of the day in radian
double DayDurationRadian (double _declination,      // _declination in radian
                          double lat                // latitude in radian
                         )
{
    double dayDurationj;

    dayDurationj = 2 * acos( -tan(lat) * tan(_declination) );
    return dayDurationj;
}

// Function to calculate Day duration in Hours
double DayDuratInHours (double _declination     // _declination in radian
                  , double lat              // latitude in radian
                  )
{
    double dayDurationj;

    dayDurationj = DayDurationRadian(_declination, lat);
    dayDurationj = RadToHours(dayDurationj);
    return dayDurationj;
}


// Function to calculate the times TSV-UTC
double Tsv_Tu (double rlong             // longitude en radian positive a l est. 
               ,double eqOfTime         // Equation of times en heure
              )
{
    double diffUTC_TSV; double pi = 3.1415926535;   // Pi

    // diffUTC_TSV Solar time as a function of longitude and the eqation of time
    diffUTC_TSV = rlong * (12 / pi) + eqOfTime;

    // difference with local time
    return diffUTC_TSV;
}


// Calculations of the orbital excentricity
double Excentricity(int day,
                    int month,
                    int year)
{

    double dayAngleRad, E0;

    // calculate the angle of day in radian
    dayAngleRad = AngleOfDay(day, month, year);

    // calculate the excentricity
    E0 = 1.000110 + 0.034221 * cos(dayAngleRad) 
            + 0.001280 * sin(dayAngleRad)
            +0.000719 * cos(2*dayAngleRad)
            +0.000077 * sin(2*dayAngleRad);

    return E0;
}

// Calculate the theoretical energy flux for the day radiation
double TheoreticRadiation(int day, int month, int year, 
                            double lat          // Latitude in radian !
                            )
{
    double RGth;        // Theoretical radiation
    double decli;       // Declination
    double E0;
    double sunriseHourAngle;            // Hour angle of sunset



    // Calculation of the declination in radian
    decli = Declination (AngleOfDay(day, month, year));

    // Calcuate excentricity
    E0 = Excentricity(day, month, year);

    // Calculate hour angle in radian
    sunriseHourAngle = DayDurationRadian(decli, lat) / 2;

    // Calculate Theoretical radiation en W.m-2
    RGth = solarConst * E0 * (cos(decli)*cos(lat)*sin(sunriseHourAngle)/sunriseHourAngle + sin(decli)*sin(lat));

    return RGth;

}

// Function to calculate decimal hour of sunrise: result in local hour
double CalclulateSunriseLocalTime(int day,
                        int month,
                        int year,
                        double rlong,
                        double rlat)

{   
    double result;
    // Calculate the angle of the day
    double DayAngle = AngleOfDay(day, month, year);
    // Declination
    double SolarDeclination = Declination(DayAngle);
    // Equation of times
    double eth = EqOfTime(DayAngle);
    // True solar time
    double diffUTC_TSV = Tsv_Tu(rlong,eth);
    // Day duration
    double dayDurationj = DayDuratInHours(SolarDeclination,rlat);

    // final result
    result = 12 - fabs(dayDurationj / 2) - diffUTC_TSV + g_localTimeAdjust;

    return result;

}

// Function to calculate decimal hour of sunset: result in local hour
double CalculateSunsetLocalTime(int day,
                          int month,
                          int year,
                          double rlong,
                          double rlat)

{   
    double result;

    // Calculate the angle of the day
    double DayAngle = AngleOfDay(day, month, year);
    // Declination
    double SolarDeclination = Declination(DayAngle);
    // Equation of times
    double eth = EqOfTime(DayAngle);
    // True solar time
    double diffUTC_TSV = Tsv_Tu(rlong,eth);
    // Day duration
    double dayDurationj = DayDuratInHours(SolarDeclination,rlat);

    // resultat
    result = 12 + fabs(dayDurationj / 2) - diffUTC_TSV + g_localTimeAdjust;

    return result;

}



// Function to calculate decimal hour of sunrise: result universal time
double CalculateSunriseUniversalTime(int day,
                        int month,
                        int year,
                        double rlong,
                        double rlat)

{   
    double result;
    // Calculate the angle of the day
    double DayAngle = AngleOfDay(day, month, year);
    // Declination
    double SolarDeclination = Declination(DayAngle);
    // Equation of times
    double eth = EqOfTime(DayAngle);
    // True solar time
    double diffUTC_TSV = Tsv_Tu(rlong,eth);
    // Day duration
    double dayDurationj = DayDuratInHours(SolarDeclination,rlat);
    // resultat
    result = 12 - fabs(dayDurationj / 2) - diffUTC_TSV;

    return result;

}

// Function to calculate decimal hour of sunset: result in universal time
double CalculateSunsetUniversalTime(int day,
                          int month,
                          int year,
                          double rlong,
                          double rlat)

{   
    double result;

    // Calculate the angle of the day
    double DayAngle = AngleOfDay(day, month, year);
    // Declination
    double SolarDeclination = Declination(DayAngle);
    // Equation of times
    double eth = EqOfTime(DayAngle);
    // True solar time
    double diffUTC_TSV = Tsv_Tu(rlong,eth);
    // Day duration
    double dayDurationj = DayDuratInHours(SolarDeclination,rlat);
    // resultat
    result = 12 + fabs(dayDurationj / 2) - diffUTC_TSV;

    return result;

}


// Function to calculate the height of the sun in radians the day to day j and hour TU
double SolarHeight (int tu,     // universal times (0,1,2,.....,23)
                      int day,
                      int month,
                      int year,
                      double lat,   // latitude in radian
                      double rlong  // longitude in radian
                     )
{
    // local variables
    double pi = 3.1415926535;   // Pi
    double result, tsvh;

    // angle of the day
    double DayAngle = AngleOfDay(day, month, year);
    // _declination
    double decli = Declination(DayAngle);
    // eq of time
    double eq = EqOfTime(DayAngle);
    // calculate the tsvh with rlong positiv for the east and negative for the west
    tsvh = tu + rlong*180/(15*pi) + eq;
    // hour angle per hour
    double ah = acos( -cos((pi/12)*tsvh) );
    // final result
    result = asin( sin(lat)*sin(decli) + cos(lat)*cos(decli)*cos(ah) );


    return result;
}

///////////EXTRA FUNCTIONS/////////////////////////////

//Julian day conversion for days calculations
//Explanation for this sick formula...for the curious guys...
//http://www.cs.utsa.edu/~cs1063/projects/Spring2011/Project1/jdn-explanation.html

int julian(int year, int month, int day) {
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;
  if (year > 1582 || (year == 1582 && month > 10) || (year == 1582 && month == 10 && day >= 15))
    return day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
  else
    return day + (153 * m + 2) / 5 + 365 * y + y / 4 - 32083;
}
