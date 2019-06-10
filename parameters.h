#define APWM 9
#define AIN1 18 // A0
#define AIN2 19 // A1
#define BPWM 6
#define BIN1 5  //change
#define BIN2 4
#define ENA_A 0
#define ENA_B 1
#define ENB_A 2
#define ENB_B 3
#define INT_PIN 7
#define LED_PIN 18
#define MPU_CS 20
#define ANGLE_LIMIT  45
#define DT 0.01f
#define fc 1.0f
#define Tc (1./(2.*PI*fc))
#define ALPHA (Tc/(Tc+DT))
#define BETA (1-ALPHA)
