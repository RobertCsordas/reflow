#include <Arduino.h>
#include <max6675.h>
#include <cmath>

bool tuning = false;

static const float L = 9.244475; // Dead time, seconds
static const float M = 3.055077; // Slope, °C/seconds

static const float Tcool = 10*60.0;
// PID parameters
static const float observerFactor = 0.2; // Observer factor for the PID controller

static const float LAMBDA_FACTOR = 0.2;
const float Kd = 0.0;
float Kp = 0.0; 
float Ki = 0.0; 


const uint8_t thermoCLK = 18; // SCK pin
const uint8_t thermoCS = 5;  // CS pin
const uint8_t thermoDO = 19; // MISO pin

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

const int SAMPLE_TIME = 500; // PID sample time in milliseconds
unsigned long last_sample_time = 0;
const float SAMPLE_PERIOD = SAMPLE_TIME / 1000.0; // Convert to seconds


struct ThermalStep {
  float targetTemp;
  float duration;
};


const ThermalStep thermalProfile[] = {
  {100.0, 30.0},
  {150.0, 90.0},
  {183.0, 30.0},
  {235.0, 60.0}
};

constexpr int PWM_PIN = 21;
constexpr int LEDC_CHANNEL = 0;
constexpr int RES_BITS = 12;  // e.g., 12-bit resolution ⇒ duty 0–4095


void heaterOn(float duty){
  ledcWrite(LEDC_CHANNEL, int(4095 * duty));
}


void heaterOff(){
  heaterOn(0);
}


float tAmbient = 0.0;
float yCurrentEstimate = 0.0;
float integral = 0.0;
float lastError = 0.0;

volatile int currStep = -1;
volatile float maxTemp = 0.0;
volatile long startTime = 0;
volatile float slope = 0.0;
volatile float startTemp = 0.0;


void setProfileStep(int index, float currentTemp) {
  maxTemp = currentTemp;
  startTemp = currentTemp;
  currStep = index;
  startTime = millis();
  if (index < 0 || index >= sizeof(thermalProfile) / sizeof(ThermalStep)) {
    slope = 0;
    return;
  }
  slope = ((thermalProfile[index].targetTemp - currentTemp) / thermalProfile[index].duration) / 1000.0; // °C/ms
}





void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  float lambd = L * LAMBDA_FACTOR;
  Kp = 1.0 / (M * lambd);
  Ki = Kp / (4.0 * lambd);

  Serial.println("PID params: Kp: "+String(Kp, 6)+" Ki: "+String(Ki, 6));
  
  ledcSetup(LEDC_CHANNEL, 1, RES_BITS);
  heaterOff();
  ledcAttachPin(PWM_PIN, LEDC_CHANNEL);

  yCurrentEstimate = 0.0;

  delay(200);
  tAmbient = thermocouple.readCelsius();

  Serial.println("Ambient temperature: " + String(tAmbient,2) + " °C");
}


float profileStep(float currentTemp, long time) {
  float targetTemp = 0.0;

  if (currStep < 0 || currStep >= sizeof(thermalProfile) / sizeof(ThermalStep)) {
    return 0.0;
  }

  bool stepTimeEnded = (time - startTime) >= thermalProfile[currStep].duration * 1000;
  unsigned long elapsed = time - startTime;

  if (stepTimeEnded) {
    targetTemp = thermalProfile[currStep].targetTemp;
  } else {
    targetTemp = startTemp + slope * elapsed;
  }

  if (currentTemp > maxTemp) {
    maxTemp = currentTemp;
  }

  if (stepTimeEnded && (maxTemp >= thermalProfile[currStep].targetTemp)){
    setProfileStep(currStep + 1, currentTemp);
  }

  return targetTemp;
}

static const float TUNE_LENGTH_MIN = 5; // How long the tuning should take.
static const float TUNE_POWER = 0.15; // How much power to apply during tuning (0.0 to 1.0).
static const int TUNE_STEPS = TUNE_LENGTH_MIN * 60.0 / SAMPLE_PERIOD;
int tune_state = 0;
float hist[TUNE_STEPS] = {0.0};
float temp0 = 0.0;

void tune(unsigned long now, float temp) {
  if (tune_state == 0){
    temp0 = temp;
    Serial.println("Tuning started at " + String(temp0) + " °C");
    heaterOn(TUNE_POWER);
    tune_state++;
  } else if (tune_state <= TUNE_STEPS) {
    hist[tune_state-1] = temp - temp0;
    Serial.println("Tuning progress "+String(((float)tune_state)/TUNE_STEPS*100.0,1)+"%: temp: " + String(temp) + " °C");
    tune_state++;
  } else {
    heaterOff();
    float threshold = hist[TUNE_STEPS - 1] * 0.02;
    int i0 = 0;
    while ((i0 < (TUNE_STEPS-10)) && (hist[i0] < threshold)) {
      i0++;
    }

    float L_est_tresh = i0 * SAMPLE_PERIOD;


    const int xmid = TUNE_STEPS / 2;
    float m = (hist[TUNE_STEPS-1] - hist[xmid]) / ((TUNE_STEPS - 1.0f - xmid) * SAMPLE_PERIOD) / TUNE_POWER;

    const int mid = (TUNE_STEPS-1 + xmid) / 2;
    const float tmid = mid * SAMPLE_PERIOD;

    float L_est_intersect = tmid - (hist[mid] / (m*TUNE_POWER));

    float L_intersect_factor = 0.5;
    float L_est = L_est_intersect * L_intersect_factor + L_est_tresh * (1-L_intersect_factor);

    int istart = int(L_est / SAMPLE_TIME);
    m = (hist[TUNE_STEPS] - hist[istart]) / ((TUNE_STEPS - 1 - istart) * SAMPLE_PERIOD) / TUNE_POWER;

    Serial.println("System identification finished. M = "+String(m, 6) + " °C/s, L = " + String(L_est, 6) + " s");
   
    while (true){
      delay(1000); // Stop here
    }
  }
}

unsigned long hist_index = 0;

float controller(unsigned long now, float temp, float target, float &futureTemp) {
  float uDelayed = hist[hist_index];
  float dt = SAMPLE_TIME / 1000.0; // Convert to seconds

  const int response_delay = min(int(std::round(L / dt)), TUNE_STEPS);

  yCurrentEstimate += dt * M * uDelayed - (dt / Tcool) * yCurrentEstimate;
  yCurrentEstimate += observerFactor * ((temp - tAmbient) - yCurrentEstimate);

  float yFuture = yCurrentEstimate;
  for (int i = 1; i < response_delay; i++) {
    int j = (hist_index + i) % response_delay;
    yFuture += dt * (M * hist[j] - 1.0 / Tcool * yFuture);
  }

  futureTemp = tAmbient + yFuture;
  float error = target - futureTemp;
  integral = constrain(integral + error * dt, -0.5/Ki, 0.5/Ki); // Constrain integral to avoid windup
  float derivative = (error - lastError) / dt;

  float u = Kp * error
          + Ki * integral
          + Kd * derivative;

  u = constrain(u, 0.0, 1.0);
  hist[hist_index] = u;

  hist_index += 1;
  if (hist_index >= response_delay) {
    hist_index = 0; // Reset index after dead time
  }
  return u;
}


float targetTemp = 0.0;

void tick(){
  float temp = thermocouple.readCelsius();
  unsigned long ms = millis();

  if (tuning) {
    tune(ms, temp);
    return;
  }

  if (currStep == -1){
    setProfileStep(0, temp);
    targetTemp = profileStep(temp, ms);
  }

  float futureTemp = 0.0;
  float dutyCycle = controller(ms, temp, targetTemp, futureTemp);
  targetTemp = profileStep(futureTemp, ms);
  heaterOn(dutyCycle);

  Serial.println("Step: " + String(currStep) + ", Curr: "+ String(temp) + " °C, Target: "+String(targetTemp) + " °C" +
                 ", Max: " + String(maxTemp) + " °C, " + " Durty cycle: " + String(dutyCycle,2) + ", Time: " + String(ms) + " ms");
 
}

void loop(){
  unsigned long now =  millis();
  if ((now - last_sample_time) >= SAMPLE_TIME) {
    if (last_sample_time == 0) {
      last_sample_time = now; // Initialize the first sample time
    } else {
      last_sample_time += SAMPLE_TIME;
    }
  } else {
    return; // Skip if not time for the next sample
  }

  tick();
  delay(1);
}