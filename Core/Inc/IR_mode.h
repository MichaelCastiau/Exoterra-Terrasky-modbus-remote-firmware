typedef enum ir_mode {
	CYCLING = 0, LEARNING = 1
} IRMode;

typedef enum time_of_day {
	DAY, NIGHT, TWILIGHT
} TimeOfDay;

typedef struct app_state {
	IRMode mode;
	TimeOfDay timeOfDay;
} AppState;
