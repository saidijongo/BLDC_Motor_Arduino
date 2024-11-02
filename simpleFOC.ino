/*
  SimpleFOC_STM32F405RGT6
  odrive 3.6 - odesk 4.2
  Hall sensor example code

  This is a code intended to test the hall sensors connections and to demonstrate the hall sensor setup.

  odrive_example_encoder.ino
  https://github.com/simplefoc/Arduino-FOC/blob/dev/examples/hardware_specific_examples/Odrive_examples/odrive_example_encoder/odrive_example_encoder.ino

  https://community.simplefoc.com/t/problems-with-custom-firmware-for-odrive-v3-5-using-simplefoc/2743
  #include <Arduino.h>
  #include <Wire.h>
  https://github.com/simplefoc/Arduino-FOC/tree/master/examples/hardware_specific_examples/Odrive_examples

*/

#include <SimpleFOC.h>

//M0
#define A_HALL1 PB4
#define A_HALL2 PB5
#define A_HALL3 PC9

//M0
//#define M0_IA // _NC ????????????
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

// M1 & M2 common enable pin
#define EN_GATE PB12

//Pole pair
#define PP 10

//Temp
#define M0_TEMP PC5

// Motor instance
BLDCMotor motor = BLDCMotor(PP);
//BLDCDriver6PWM driver = BLDCDriver6PWM(PC6, PA7, PC7, PB0, PC8, PB1, PB12);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

// low side current sensing define
// 0.0005 Ohm resistor
// gain of 10x
// current sensing on B and C phases, phase A not connected
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, _NC, M0_IB, M0_IC);

// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, PP); // PP = xx aimants / 2 = pole pair

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
void doC() {
  sensor.handleC();
}

//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬SoftwareSerial▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
#include <SoftwareSerial.h>
// SPI pinout mosi/miso ==> Utilisé pour serial RX/TX
#define SERIAL_RX PA3 //PC12
#define SERIAL_TX PA2 //PC11
// Set up a new SoftwareSerial object
SoftwareSerial mySerial =  SoftwareSerial(SERIAL_RX, SERIAL_TX);

// https://docs.simplefoc.com/communication
// instantiate the commander
Commander command = Commander(mySerial);

// motor SimpleFOCStudio ==> M
void doMotor(char* cmd) {
  //command.motor(&motor, cmd);
  command.target(&motor, cmd);
  //command.motion(&motor,cmd);
}


//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬setup▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
void setup() {
  //▬▬▬▬▬▬▬▬▬▬▬▬GPIO1/GPIO2/GPIO3/GPIO4▬▬▬▬▬▬▬▬▬▬▬▬
  /*
    pinMode(PA0, OUTPUT); // GPIO1
    digitalWrite(PA0, LOW); // HIGH/LOW

    pinMode(PA1, OUTPUT); // GPIO2
    digitalWrite(PA1, LOW); // HIGH/LOW

    pinMode(PA2, OUTPUT); // GPIO3
    digitalWrite(PA2, LOW); // HIGH/LOW

    pinMode(PA3, OUTPUT); // GPIO4
    digitalWrite(PA3, LOW); // HIGH/LOW
  */

  //▬▬▬▬▬▬▬▬▬▬▬▬Temp▬▬▬▬▬▬▬▬▬▬▬▬
  pinMode(M0_TEMP, INPUT); // M0_TEMP PC5

  //▬▬▬▬▬▬▬▬▬▬▬▬SoftwareSerial▬▬▬▬▬▬▬▬▬▬▬▬
  // Define pin modes for TX and RX
  pinMode(SERIAL_RX, INPUT_PULLUP); // sur GPIO PA3
  pinMode(SERIAL_TX, OUTPUT); // sur GPIO PA2
  mySerial.begin(115200); // 115200 Set the baud rate for the SoftwareSerial object
  _delay(1000);

  //▬▬▬▬▬▬▬▬▬▬▬▬driver▬▬▬▬▬▬▬▬▬▬▬▬
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000; // 20000 max SRM32
  driver.voltage_power_supply = 24.0f; // power supply voltage [V]
  driver.voltage_limit = 24.0f;  // Max DC voltage allowed - default voltage_power_supply
  driver.init();// driver init
  motor.linkDriver(&driver); // link the motor and the driver

  //▬▬▬▬▬▬▬▬▬▬▬▬Current▬▬▬▬▬▬▬▬▬▬▬▬
  // ici passe pas ?

  //▬▬▬▬▬▬▬▬▬▬▬▬sensor▬▬▬▬▬▬▬▬▬▬▬▬
  sensor.pullup = Pullup::USE_EXTERN; // check if you need internal pullups
  sensor.init(); // initialise encoder hardware
  sensor.enableInterrupts(doA, doB, doC); // hardware interrupt enable
  motor.linkSensor(&sensor);
  _delay(1000);

  //▬▬▬▬▬▬▬▬▬▬▬▬mode▬▬▬▬▬▬▬▬▬▬▬▬
  // set torque mode
  // https://docs.simplefoc.com/foc_current_torque_mode
  motor.torque_controller = TorqueControlType::foc_current; // foc_current || dc_current || voltage
  motor.controller = MotionControlType::velocity; // angle velocity torque  // Control loop type
  // choose FOC modulation
  // FOCModulationType::SinePWM; (default)
  // FOCModulationType::SpaceVectorPWM;
  // FOCModulationType::Trapezoid_120;
  // FOCModulationType::Trapezoid_150;
  motor.foc_modulation = FOCModulationType::SinePWM;  // pwm modulation settings
  motor.modulation_centered = 1; // 1

  //▬▬▬▬▬▬▬▬▬▬▬▬ALL_PID▬▬▬▬▬▬▬▬▬▬▬▬
  // velocity PID
  motor.PID_velocity.P = 0.1f; // 0.2
  motor.PID_velocity.I = 1.0f; // 2.0
  motor.PID_velocity.D = 0.001; // 0.001
  //motor.PID_velocity.output_ramp = 1000.0;
  //motor.PID_velocity.limit = 1.0;  // ? current_limit [Amps] // https://community.simplefoc.com/t/setting-current-limits-in-main-loop/1972
  motor.LPF_velocity.Tf = 0.01; //0.01  // Low pass filtering time constant

  // angle PID
  motor.P_angle.P = 30.0; // 14.0
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 10000.0; // 10000.0
  //motor.P_angle.limit = 50.0; // ? velocity_limit [rad/s]
  motor.LPF_angle.Tf = 0.001;  // Low pass filtering time constant

  // current q PID
  motor.PID_current_q.P = 1.0; // 3
  motor.PID_current_q.I = 100.0; // 300
  //motor.PID_current_q.D = 0.0;
  //motor.PID_current_q.output_ramp = 1000.0f;
  //motor.PID_current_q.limit = 3.0; // https://community.simplefoc.com/t/setting-current-limits-in-main-loop/1972
  //motor.LPF_current_q.Tf = 0.005;  // Low pass filtering time constant

  // current d PID
  motor.PID_current_d.P = 1.0; // 3
  motor.PID_current_d.I = 100.0; // 300
  //motor.PID_current_d.D = 0.0;
  //motor.PID_current_d.output_ramp = 1000.0f;
  //motor.PID_current_d.limit = 3.0; // https://community.simplefoc.com/t/setting-current-limits-in-main-loop/1972
  //motor.LPF_current_d.Tf = 0.005;  // Low pass filtering time constant

  //▬▬▬▬▬▬▬▬▬▬▬▬Limits▬▬▬▬▬▬▬▬▬▬▬▬
  motor.velocity_limit = 3.0f; // [rad/s] (120[rad/s] ==> ~1200[RPM])
  motor.voltage_limit = 0.5 * driver.voltage_limit; // [Volts] // Calcul ==> 5.57[Ohms]*1.0[Amps]=5,57[Volts] // [V] - if phase resistance not defined
  //motor.voltage_limit = 24.0f; // [Volts] // Calcul ==> 5.57[Ohms]*1.0[Amps]=5,57[Volts] // [V] - if phase resistance not defined
  motor.current_limit = 5.0f; // Current limit [Amps] - if phase resistance defined
  motor.phase_resistance = 0.5f; // [Ohms]  // motor phase resistance // I_max = V_dc/R
  motor.KV_rating = 100; // [rpm/Volt] - default not set // motor KV rating [rpm/V]
  // commenter les 2 ci-dessous pour avoir le test au demarrage
  motor.zero_electric_angle = 2.09f; // 4.19  // zero_electric_angle
  motor.sensor_direction = Direction::CCW; // Cw/CCW // direction

  //▬▬▬▬▬▬▬▬▬▬▬▬SimpleFOCDebug▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/debugging
  //SimpleFOCDebug::enable(NULL);
  //SimpleFOCDebug::enable(&mySerial);

  //▬▬▬▬▬▬▬▬▬▬▬▬motion_downsample▬▬▬▬▬▬▬▬▬▬▬▬
  /*
    Pour de nombreuses applications de contrôle de mouvement, il est judicieux d'exécuter
    plusieurs boucles de contrôle de couple pour chaque boucle de contrôle de mouvement.
    Cela peut avoir un impact important sur la fluidité et peut fournir de meilleures performances à grande vitesse.
    C'est pourquoi cette bibliothèque permet une stratégie de sous-échantillonnage très simple pour la fonction move()
    qui est définie à l'aide du paramètre

    La stratégie de downsampling fonctionne de manière très simple, même si la fonction motor.move()
    est appelée dans chaque boucle arduino, elle ne sera exécutée qu'à chaque fois que motor.motion_downsample sera appelé.
    Ce paramètre est optionnel et peut être configuré en temps réel.
  */
  motor.motion_downsample = 0; // 0 https://docs.simplefoc.com/bldcmotor

  //▬▬▬▬▬▬▬▬▬▬▬▬init▬▬▬▬▬▬▬▬▬▬▬▬
  motor.init(); // initialise motor

  //▬▬▬▬▬▬▬▬▬▬▬▬linkCurrentSense▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/inline_current_sense
  currentSense.linkDriver(&driver); // link the driver
  currentSense.init(); // init the current sense
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
  //▬▬▬▬▬▬▬▬▬▬▬▬initFOC▬▬▬▬▬▬▬▬▬▬▬▬
  motor.initFOC(); // init FOC

  //▬▬▬▬▬▬▬▬▬▬▬▬command▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/commander_interface
  // add the motor to the commander interface
  command.decimal_places = 4; // default 3
  command.add('M', doMotor, "motor exemple ==> M10"); // The letter (here 'M') you will provide to the SimpleFOCStudio

  //▬▬▬▬▬▬▬▬▬▬▬▬target▬▬▬▬▬▬▬▬▬▬▬▬
  motor.target = 0;

  _delay(1000);
} // End setup
//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬End_setup▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬



//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬loop▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬

// current https://docs.simplefoc.com/inline_current_sense
PhaseCurrent_s current;// getPhaseCurrents

float actual_rpm;
long loop_count = 0;// loop down-sampling counter

void loop() {
  current = currentSense.getPhaseCurrents();
  float current_magnitude = currentSense.getDCCurrent(); // https://docs.simplefoc.com/inline_current_sense#example-code

  /*
    //▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬mySerial_send▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    // On envoie au M5StickC-plus la vitesse en RPM
    if (loop_count++ > 1000) { // control loop each ~xx ms
      //actual_rpm = (sensor.getVelocity() * 9.5492968) * -1;
      actual_rpm = sensor.getVelocity();
      //mySerial.println(actual_rpm);
      String toSEND = (String(actual_rpm) + "\n");
      //mySerial.write((char*)toSEND.c_str());
      loop_count = 0; // restart the counter
    }
  */

  motor.loopFOC(); // main FOC algorithm function
  motor.move();
  command.run();

} // End loop
//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬End_loop▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
