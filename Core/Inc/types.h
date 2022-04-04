#ifndef __TYPES_H__
#define __TYPES_H__

typedef enum ir_mode {
	CYCLING = 0, LEARNING = 1
} IRMode;

typedef enum {
	DAY, NIGHT, TWILIGHT
} TimeOfDay;

typedef struct {
	IRMode mode;
	TimeOfDay timeOfDay;
	NEC_Frame dayFrame;
	NEC_Frame nightFrame;
	NEC_Frame twilightFrame;
} AppState;


#endif
