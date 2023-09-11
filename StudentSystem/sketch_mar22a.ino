#include <Servo.h>
#include <Wire.h>

#define I2C_SDA 2
#define I2C_SCL 3

#define DRIVE_L 19
#define DRIVE_R 21
#define ARM_LIFT 14

struct CState {
	byte Axis0;
	byte Axis1;
	byte Axis2;
	byte Axis3;
	byte Axis4;
	byte Axis5;
	byte Axis6;
	byte Axis7;

	byte Button0;
	byte Button1;
	byte Button2;
	byte Button3;
	byte Button4;
	byte Button5;
	byte Button6;
	byte Button7;
	byte Button8;
	byte Button9;
};

byte StateClean;

CState state;

Servo DriveL, DriveR, ArmLift;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

	Serial1.setTX(0);
	Serial1.setRX(1);
	Serial1.begin(9600);

	Serial.begin(9600);

	Serial.println();
	Serial.println();
	Serial.println("Connecting Serial");

	// put your setup code here, to run once:
	pinMode(DRIVE_L, OUTPUT);
	pinMode(DRIVE_R, OUTPUT);
	pinMode(ARM_LIFT, OUTPUT);

	DriveL.attach(DRIVE_L);
	DriveR.attach(DRIVE_R);
	ArmLift.attach(ARM_LIFT);
	Serial.println("setup complete");

	StateClean = 1;
}

void setup1() {
	Wire1.setSDA(I2C_SDA);
	Wire1.setSCL(I2C_SCL);
	Wire1.setClock(400000);
	Wire1.begin();
}

unsigned long doStuffNow;

void loop() {
	if (millis() > doStuffNow) {
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		doStuffNow = millis() + 50;
	} else {
		delay(10);
	}
}

void loop1() {
	updateStateFromI2C();
	delay(50);
}

void updateStateFromI2C() {
	// Request 18 bytes from Device 8
	size_t amountRead = Wire1.requestFrom(8, 18);

	if (Wire1.available() >= 18) {
		// Read the data and populate the struct
		Wire1.readBytes(reinterpret_cast<uint8_t*>(&state), sizeof(CState));
	} else {
		Serial.println("Error: Not enough data received.");
	}
	Serial.println(String("Asked from 8, 18 bytes, got ") + amountRead + "\n");

}

void old_updateStateFromI2C() {
	Serial.println("In I2C");
	StateClean = 0;
	if (!Wire1.available()) { return; }
	state.Axis0 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Axis1 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Axis2 = Wire1.read();

	if (!Wire1.available()) { return; }
	state.Axis3 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Axis4 = Wire1.read();

	if (!Wire1.available()) { return; }
	state.Axis5 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Axis6 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Axis7 = Wire1.read();

	if (!Wire1.available()) { return; }
	state.Button0 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button1 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button2 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button3 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button4 = Wire1.read();


	if (!Wire1.available()) { return; }
	state.Button5 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button6 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button7 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button8 = Wire1.read();
	if (!Wire1.available()) { return; }
	state.Button9 = Wire1.read();

	StateClean = 1;
}
