
typedef enum IS31FL3195_Pattern_Gamma {
	IS31FL3195_GAMMA_24 = 0b00,
	IS31FL3195_GAMMA_35 = 0b01,
	IS31FL3195_GAMMA_LINEAR = 0b10,
} IS31FL3195_Pattern_Gamma;

typedef enum IS31FL3195_Pattern_Instruction {
	IS31FL3195_INST_HALT = 0b00,

	// 1 => 2, 2 => 1, 3 => 1, 4 => HALT
	IS31FL3195_INST_GOTO_A = 0b01,
	// 1 => HALT, 2 => 3, 3 => 2, 4 => HALT
	IS31FL3195_INST_GOTO_B = 0b10,
} IS31FL3195_Pattern_NXT;

typedef struct __attribute__((packed)) IS31FL3195_Pattern {
	IS31FL3195_Timing delay: 4;

	IS31FL3195_Pattern_Instruction next: 2;
} IS31FL3195_Pattern;
