/*
  MPU6050_ConfigSweep.ino

  Sweeps every combination of AccelScale, GyroScale and DLPF, applies
  it to the MPU6050 and prints one reading per combination as CSV over
  Serial. Useful for characterizing how each setting affects the raw
  output (e.g. importing the log into a spreadsheet or a script for
  analysis).

  CSV columns:
  accelScale,gyroScale,dlpf,smplrtDiv,accX,accY,accZ,gyrX,gyrY,gyrZ,tempC,
  gyroBW_Hz,outputRate_Hz,nyquist_Hz,dlpf_below_nyquist
*/

#include <DevLab_MPU6050.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define MPU6050_ADDR 0x68

DevLab_MPU6050 imu(Wire, MPU6050_ADDR);
DevLab_MPU6050::SensorData data;

const uint8_t ACCEL_SCALES[]  = {0, 1, 2, 3};
const uint8_t GYRO_SCALES[]   = {0, 1, 2, 3};
const uint8_t DLPF_SETTINGS[] = {0, 1, 2, 3, 4, 5, 6};

// Element counts computed explicitly -- sizeof(array) on a uint8_t[]
// happens to equal the element count only because uint8_t is 1 byte.
const uint8_t N_ACCEL = sizeof(ACCEL_SCALES)  / sizeof(ACCEL_SCALES[0]);
const uint8_t N_GYRO  = sizeof(GYRO_SCALES)   / sizeof(GYRO_SCALES[0]);
const uint8_t N_DLPF  = sizeof(DLPF_SETTINGS) / sizeof(DLPF_SETTINGS[0]);

const uint8_t CLK_SOURCE_PLL_GYRO_X = 1; // stable reference, recommended by Invensense

// DLPF_CFG bandwidths in Hz (MPU-6000/6050 register map, DLPF_CFG table).
const float ACCEL_BW_HZ[] = {260.0f, 184.0f, 94.0f, 44.0f, 21.0f, 10.0f, 5.0f};
const float GYRO_BW_HZ[]  = {256.0f, 188.0f, 98.0f, 42.0f, 20.0f, 10.0f, 5.0f};

// Nyquist must be at least this many times the DLPF corner frequency.
const float NYQUIST_MARGIN = 1.2f;

// Number of sample periods to wait after an AFS_SEL/FS_SEL change before
// trusting readData() -- absorbs I2C transaction latency and ADC
// conversion jitter so we don't sample mid-transition.
const uint8_t SCALE_SETTLE_PERIODS = 5;

// How many full sweeps to run back-to-back (set from setup(), or change
// here and call sweepAndPrint() once per iteration from setup()/loop()).
const uint16_t N_REPEATS = 100;

// Internal sampling rate feeding SMPLRT_DIV: 8kHz when the DLPF is
// bypassed (DLPF_CFG=0), 1kHz for any other DLPF setting.
float internalFsHz(uint8_t dlpf) {
  return (dlpf == 0) ? 8000.0f : 1000.0f;
}

// SMPLRT_DIV per DLPF_CFG, computed once at startup. Chosen as the
// LARGEST divisor (lowest output rate, gentlest on I2C traffic) that
// still keeps Nyquist >= NYQUIST_MARGIN * corner.
uint8_t g_smplrtDivByDlpf[7];

void computeSmplrtDivTable() {
  for (uint8_t d = 0; d < N_DLPF; d++) {
    uint8_t dlpf = DLPF_SETTINGS[d];
    float fsBase = internalFsHz(dlpf);
    float cornerHz = min(ACCEL_BW_HZ[d], GYRO_BW_HZ[d]);

    float maxRatio = fsBase / (2.0f * NYQUIST_MARGIN * cornerHz);
    long divCandidate = (long)floor(maxRatio - 1.0f);

    if (divCandidate < 0)   divCandidate = 0;
    if (divCandidate > 255) divCandidate = 255;

    g_smplrtDivByDlpf[d] = (uint8_t)divCandidate;
  }
}

float samplePeriodMs(uint8_t dIndex) {
  uint8_t dlpf = DLPF_SETTINGS[dIndex];
  float fsBase = internalFsHz(dlpf);
  uint8_t div = g_smplrtDivByDlpf[dIndex];
  return 1000.0f * (1 + div) / fsBase;
}

// Time for the DLPF's single-pole low-pass to settle (~5 time constants,
// tau = 1/(2*pi*BW)), using the tighter of the accel/gyro corner
// frequencies, plus one sample period so the settled value has actually
// been latched into the output registers.
uint16_t dlpfSettleMs(uint8_t dIndex) {
  float bwHz = min(ACCEL_BW_HZ[dIndex], GYRO_BW_HZ[dIndex]);
  float tauMs = 1000.0f / (2.0f * PI * bwHz);
  float settleMs = 5.0f * tauMs + samplePeriodMs(dIndex);
  return (uint16_t)ceil(settleMs);
}

// Time to wait after an AFS_SEL/FS_SEL change: several sample periods.
uint16_t scaleSettleMs(uint8_t dIndex) {
  float settleMs = SCALE_SETTLE_PERIODS * samplePeriodMs(dIndex);
  return (uint16_t)ceil(settleMs);
}

void sweepAndPrint() {
  // --- Repeat-call fix -------------------------------------------------
  // Explicitly (re)apply and settle the FIRST combination of the sweep
  // (AFS_SEL=0, FS_SEL=0, DLPF_CFG=0) before entering the nested loops.
  // Without this, if sweepAndPrint() is called back-to-back in a loop,
  // this first reading follows immediately after the LAST combination
  // of the previous call (AFS_SEL=3, DLPF_CFG=6) with no settle at all,
  // and can carry over the previous call's scale (see header note).
  // dlpfSettleMs() is used here (not scaleSettleMs()) because it's the
  // longer of the two and covers both the DLPF and the scale settling.
  imu.setDLPF(DLPF_SETTINGS[0]);
  imu.sampleRateDiv(g_smplrtDivByDlpf[0]);
  imu.setAccelScale(ACCEL_SCALES[0]);
  imu.setGyroScale(GYRO_SCALES[0]);
  delay(dlpfSettleMs(0));
  // ----------------------------------------------------------------------

  for (uint8_t d = 0; d < N_DLPF; d++) {
    uint8_t dlpf = DLPF_SETTINGS[d];
    uint8_t div = g_smplrtDivByDlpf[d];

    imu.setDLPF(dlpf);
    imu.sampleRateDiv(div);
    delay(dlpfSettleMs(d));

    float fsBase = internalFsHz(dlpf);
    float outputRateHz = fsBase / (1 + div);
    float nyquistHz = outputRateHz / 2.0f;
    float gyroBw = GYRO_BW_HZ[d];
    float cornerHz = min(ACCEL_BW_HZ[d], GYRO_BW_HZ[d]);
    bool belowNyquist = (cornerHz <= nyquistHz);

    uint16_t scaleDelayMs = scaleSettleMs(d);

    for (uint8_t a = 0; a < N_ACCEL; a++) {
      imu.setAccelScale(ACCEL_SCALES[a]);

      for (uint8_t g = 0; g < N_GYRO; g++) {
        imu.setGyroScale(GYRO_SCALES[g]);

        delay(scaleDelayMs);

        Serial.print(ACCEL_SCALES[a]); Serial.print(',');
        Serial.print(GYRO_SCALES[g]);  Serial.print(',');
        Serial.print(dlpf);            Serial.print(',');
        Serial.print(div);             Serial.print(',');

        if (imu.readData(data)) {
          Serial.print(data.accX); Serial.print(',');
          Serial.print(data.accY); Serial.print(',');
          Serial.print(data.accZ); Serial.print(',');
          Serial.print(data.gyrX); Serial.print(',');
          Serial.print(data.gyrY); Serial.print(',');
          Serial.print(data.gyrZ); Serial.print(',');
          Serial.print(data.temperatureC, 2); Serial.print(',');

          Serial.print(gyroBw, 1);          Serial.print(',');
          Serial.print(outputRateHz, 2);    Serial.print(',');
          Serial.print(nyquistHz, 2);       Serial.print(',');
          Serial.println(belowNyquist ? "OK" : "ALIAS_RISK");
        } else {
          Serial.println("READ_ERROR");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando MPU6050...");

  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("ERROR: MPU6050 no encontrado.");
    while (1) {
      delay(1000);
    }
  }

  imu.setClkSource(CLK_SOURCE_PLL_GYRO_X);
  computeSmplrtDivTable();

  Serial.println("MPU6050 inicializado correctamente.");
  Serial.println("Tabla SMPLRT_DIV calculada por DLPF_CFG (Nyquist >= 1.2x corner):");
  for (uint8_t d = 0; d < N_DLPF; d++) {
    Serial.print("#  DLPF_CFG=");
    Serial.print(DLPF_SETTINGS[d]);
    Serial.print(" -> SMPLRT_DIV=");
    Serial.println(g_smplrtDivByDlpf[d]);
  }

  Serial.print("Repeticiones del sweep completo: ");
  Serial.println(N_REPEATS);

  Serial.println("Iniciando sweep de configuraciones...");
  Serial.println("accelScale,gyroScale,dlpf,smplrtDiv,accX,accY,accZ,gyrX,gyrY,gyrZ,tempC,gyroBW_Hz,outputRate_Hz,nyquist_Hz,dlpf_below_nyquist");

  for (uint16_t i = 0; i < N_REPEATS; i++) {
    sweepAndPrint();
  }

  Serial.println("Sweep completo.");
}

void loop() {
  // Nothing to do here; the sweep runs once in setup().
}
