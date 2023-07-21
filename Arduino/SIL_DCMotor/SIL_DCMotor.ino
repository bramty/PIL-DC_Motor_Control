#define BUFFER_SIZE 9
#define T_REF_UPDATE 5000
uint8_t buff[BUFFER_SIZE];

#define ERR_LIM_MIN -5.0f
#define ERR_LIM_MAX  5.0f

//Union for sending/receiving float to/from Simulink
typedef union {
  float fval;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t yk1, yk2, rk, uk;

//Struct for PID Controller Component
struct PID{
  float kp;
  float ki;
  float kd;
  float rk;
  float yk;
  float uk;
  float uk_1;
  float e;
  float e_sum;
  float e_1;
};

struct PID n; // PID for motor speed control
struct PID i; // PID for armature current control

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int potVal;
float ref;

long prev_time;


/*----------- PID Parameters -----------*/
float sample_time = 1; // in seconds
//PID Controller for Motor Speed 
const float kr_n = 2.5e-3;
const float ti_n = 1.3991;
const float td_n = 0.083946;
//PID Controller for Armature Current 
const float kr_i = 0.446;
const float ti_i = 3.8455;
const float td_i = 0;

//Command Limit Parameter
const float I_MAX = 6;
const float I_MIN = 0;
const float V_MAX = 12;
const float V_MIN = 0;
const float N_MAX = 100;

float n_test = 100.0;
float i_test = 0.0;

void setup() {
  
  // Set up serial communication
  Serial.begin(115200);
  
  // Initialize motor speed PID loop component
  initialize_pid_param( &n, kr_n, ti_n, td_n, sample_time );

  // Initialize armature current PID loop component
  initialize_pid_param( &i, kr_i, ti_i, td_i, sample_time );
  
  // Set motor speed reference to 0 initially
  n.rk = 0.0;

  // Capture start timing for reference reading with potentiometer
  prev_time = millis();

}

void loop() {
 
  // Read potentiometer voltage every T_REF_UPDATE
  if ( millis() - prev_time > T_REF_UPDATE ) {
    potVal = analogRead(analogInPin);
    n.rk = map(potVal, 0, 1023, 0, N_MAX);
    prev_time = millis();
  }

  readFromMatlab(&yk1, &yk2);

  // Assigned the received data
  n.yk = yk1.fval;
  i.yk = yk2.fval;

  //---------------------------------------------------------//
  // PID Control for DC Motor Speed Loop
  //---------------------------------------------------------//
  PID_control( &n, n.rk, n.yk, I_MAX, I_MIN, sample_time );

  //---------------------------------------------------------//
  // PID Control for DC Motor Armature Current Loop
  //---------------------------------------------------------//
  PID_control( &i, n.uk, i.yk, V_MAX, V_MIN, sample_time );

  // assign calculated armature voltage to uk and reference speed to rk
  uk.fval = (float) i.uk;
  rk.fval = (float) n.rk;

  // Transmit uk and rk via serial to SIL
  writeToMatlab(uk, rk);

  // Delay for sampling time
  delay((int)sample_time*1000);

}

void readFromMatlab( FLOATUNION_t* f1, FLOATUNION_t* f2 ){
  int count = 0;
  FLOATUNION_t f;
  bool allReceived = false;
  uint8_t buffer;
  int idx = 0;

  // read the incoming bytes:
  int rlen = Serial.readBytesUntil('\n', buff, BUFFER_SIZE);

  for( int i=0;i<4;i++){
    f1->bytes[i]=buff[i];
  }
  for( int i=4;i<8;i++){
    f2->bytes[i-4]=buff[i];
  }
}

void writeToMatlab( FLOATUNION_t fnumber, FLOATUNION_t fnumber2 ) {
  
  // Print header: Important to avoid sync errors!
  Serial.write('A');

  // Send each byte of uk and rk respectively 
  for (int i=0; i<4; i++){
	  Serial.write(fnumber.bytes[i]); 
  }
  for (int i=0; i<4; i++){
	  Serial.write(fnumber2.bytes[i]); 
  }

  // Delimiter
  Serial.print('\n');

}

void initialize_pid_param( struct PID* pid, float p, float i, float d, float T ){
  
  // Initialize all PID component parameters
  // Set Kp = Kr
  pid->kp     = p;

  // Zero-value Ti protection
  // Set Ki = Kp*T/Ti if Ti is not 0
  if (i != 0.0)  
    pid->ki   = p*T/i;
  else
    pid->ki   = 0;

  // Set Kd = Kp*Td/T
  pid->kd     = p*d/T;

  // Set the remaining PID parameters to 0.0
  pid->rk     = 0.0;
  pid->yk     = 0.0;
  pid->uk     = 0.0;
  pid->uk_1   = 0.0;
  pid->e_sum  = 0.0;
  pid->e_1    = 0.0;

}

void PID_control( struct PID* pid, float ref, float out, float lim_top, float lim_btm, float dt ){
  
  // Assign the reference value and output value from the model
  pid->rk = ref;
  pid->yk = out;
  
  // Calculate the error
  float error = ref - out;
  pid->e = error;

  // Calculate the proportional term
  float proportional = pid->kp * error;

  // Calculate the integral term
  pid->e_sum += error;
  float integral = pid->ki * pid->e_sum;

  // Calculate the derivative term
  float derivative = pid->kd * (error - pid->e_1);
  
  // Calculate the command output
  pid->uk = proportional + integral + derivative;
  
  // Command Limit
  if(pid->uk >= lim_top)
    pid->uk = lim_top;
  if(pid->uk <= lim_btm)
    pid->uk = lim_btm;
  
  // PID memory update
  pid->uk_1 = pid->uk;
  pid->e_1 = error;

}

