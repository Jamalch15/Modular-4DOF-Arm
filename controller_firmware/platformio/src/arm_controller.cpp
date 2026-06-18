#include <Arduino.h>
#include <math.h>

#if defined(ARM_CONTROLLER_ENABLE_AS5048A)
#include <SPI.h>
#endif

// Full-arm ESP32-S3 controller for the PC dashboard protocol.
// Current assumption: the PC plans trajectories and uploads timed joint points.
// The ESP follows the queued path open-loop, drives only configured hardware
// axes, and simulates disabled axes in its reported pose.

#if defined(ARM_CONTROLLER_NATIVE_USB) || defined(ARM_PROTOCOL_NATIVE_USB)
#define ARM_SERIAL Serial
#else
#define ARM_SERIAL Serial0
#endif

#ifndef ESP_RGB_LED_PIN
#define ESP_RGB_LED_PIN 48
#endif

namespace {
constexpr unsigned long kSerialWaitMs = 3000;
constexpr uint32_t kStatusIntervalMs = 1000;
constexpr int kJointCount = 4;
constexpr int kMaxLineLength = 320;
constexpr int kMaxTrajectoryPoints = 240;
constexpr uint32_t kJogWatchdogMs = 350;
constexpr int kServoPwmResolutionBits = 14;
constexpr int kServoPwmChannelBase = 0;
constexpr uint32_t kServoPwmMaxDuty = (1UL << kServoPwmResolutionBits) - 1UL;
constexpr float kDefaultHome[kJointCount] = {0.0f, 20.0f, 20.0f, 0.0f};
constexpr float kDefaultMin[kJointCount] = {-160.0f, -30.0f, -120.0f, -120.0f};
constexpr float kDefaultMax[kJointCount] = {160.0f, 115.0f, 120.0f, 120.0f};
const char* kDefaultNames[kJointCount] = {"base", "shoulder", "elbow", "wrist"};

#if defined(ARM_CONTROLLER_ENABLE_AS5048A)
#ifndef ARM_ENCODER_SCK_PIN
#define ARM_ENCODER_SCK_PIN 18
#endif
#ifndef ARM_ENCODER_MISO_PIN
#define ARM_ENCODER_MISO_PIN 19
#endif
#ifndef ARM_ENCODER_MOSI_PIN
#define ARM_ENCODER_MOSI_PIN 23
#endif
#ifndef ARM_ENCODER_BASE_CS_PIN
#define ARM_ENCODER_BASE_CS_PIN 5
#endif
#ifndef ARM_ENCODER_SHOULDER_CS_PIN
#define ARM_ENCODER_SHOULDER_CS_PIN 7
#endif
constexpr uint32_t kEncoderSpiClockHz = 1000000;
constexpr uint16_t kAs5048ReadAngleCommand = 0xFFFF;
constexpr uint16_t kAs5048ClearErrorCommand = 0x4001;
constexpr uint16_t kAs5048AngleMask = 0x3FFF;
constexpr uint16_t kAs5048ErrorFlag = 0x4000;
constexpr int kEncoderCsPins[kJointCount] = {ARM_ENCODER_BASE_CS_PIN, ARM_ENCODER_SHOULDER_CS_PIN, -1, -1};
SPISettings encoderSpiSettings(kEncoderSpiClockHz, MSBFIRST, SPI_MODE1);
#endif

enum class ControllerState {
  Idle,
  Moving,
  Stopped,
  Estop,
  Fault,
};

enum class ActuatorType {
  Stepper,
  Servo,
  Unknown,
};

enum class AxisState {
  Simulated,
  Hardware,
  Invalid,
};

enum class ToolType {
  Generic,
  ServoGripper,
  Electromagnet,
};

struct StepperConfig {
  int stepPin = -1;
  int dirPin = -1;
  int enablePin = -1;
  bool enableActiveLow = true;
  int fullStepsPerRev = 200;
  int microsteps = 16;
  float gearRatio = 1.0f;
};

struct ServoConfig {
  int pwmPin = -1;
  int pulseMinUs = 500;
  int pulseMaxUs = 2500;
  int pwmFrequencyHz = 50;
  float rangeDeg = 270.0f;
  float neutralDeg = 135.0f;
  float gearRatio = 1.0f;
};

struct JointConfig {
  char name[20] = "";
  ActuatorType actuator = ActuatorType::Unknown;
  bool enabled = false;
  bool received = false;
  float zeroOffsetDeg = 0.0f;
  int directionSign = 1;
  float minDeg = -180.0f;
  float maxDeg = 180.0f;
  float homeDeg = 0.0f;
  float maxSpeedDegS = 45.0f;
  float maxAccelDegS2 = 120.0f;
  StepperConfig stepper;
  ServoConfig servo;
  AxisState axisState = AxisState::Simulated;
};

struct ToolConfig {
  char name[20] = "generic";
  ToolType type = ToolType::Generic;
  bool received = false;
  int pwmPin = -1;
  int pulseMinUs = 500;
  int pulseMaxUs = 2500;
  int pwmFrequencyHz = 50;
  float openValue = 0.0f;
  float closedValue = 1.0f;
  int gpioPin = -1;
  bool activeHigh = true;
};

struct StepperRuntime {
  long currentSteps = 0;
  long targetSteps = 0;
  unsigned long lastStepUs = 0;
};

struct ServoRuntime {
  int pulseUs = 1500;
  int channel = -1;
  bool attached = false;
  float velocityDegS = 0.0f;
  unsigned long lastUpdateUs = 0;
};

struct ToolRuntime {
  int channel = 6;
  bool attached = false;
  int attachedPwmPin = -1;
  int gpioPin = -1;
  bool gpioActiveHigh = true;
};

struct TrajectoryPoint {
  float timeS = 0.0f;
  float jointsDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct TrajectoryRuntime {
  TrajectoryPoint points[kMaxTrajectoryPoints];
  int expectedCount = 0;
  int count = 0;
  bool receiving = false;
  bool ready = false;
  bool active = false;
  unsigned long startUs = 0;
  float durationS = 0.0f;
};

ControllerState controllerState = ControllerState::Idle;
JointConfig joints[kJointCount];
JointConfig draftJoints[kJointCount];
ToolConfig activeTool;
ToolConfig draftTool;
StepperRuntime stepperRuntime[kJointCount];
ServoRuntime servoRuntime[kJointCount];
ToolRuntime toolRuntime;
TrajectoryRuntime trajectoryRuntime;
float currentJointsDeg[kJointCount] = {kDefaultHome[0], kDefaultHome[1], kDefaultHome[2], kDefaultHome[3]};
float targetJointsDeg[kJointCount] = {kDefaultHome[0], kDefaultHome[1], kDefaultHome[2], kDefaultHome[3]};
float lastSpeedDegS = 25.0f;
float lastAccelDegS2 = 120.0f;
bool armed = false;
bool homed = false;
bool configInProgress = false;
bool jogActive = false;
bool jogVelocityMode = false;
bool jogStopRequested = false;
uint32_t lastJogMs = 0;
unsigned long jogLastUpdateUs = 0;
float jogTargetVelocityDegS[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float jogCurrentVelocityDegS[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float jogStepperRemainderSteps[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
char faultText[40] = "OK";
char toolState[12] = "unknown";
char poseSourceText[12] = "unknown";
float toolValue = 0.0f;
bool encoderAvailable[kJointCount] = {false, false, false, false};
float encoderAnglesDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
String commandLine;
uint32_t lastStatusMs = 0;

float clampFloat(float value, float minValue, float maxValue) {
  return min(max(value, minValue), maxValue);
}

bool validPinOrUnused(int pin) {
  return pin == -1 || (pin >= 0 && pin <= 48);
}

ActuatorType parseActuator(const String& value) {
  if (value.equalsIgnoreCase("stepper")) {
    return ActuatorType::Stepper;
  }
  if (value.equalsIgnoreCase("servo")) {
    return ActuatorType::Servo;
  }
  return ActuatorType::Unknown;
}

const char* actuatorName(ActuatorType actuator) {
  switch (actuator) {
    case ActuatorType::Stepper:
      return "stepper";
    case ActuatorType::Servo:
      return "servo";
    case ActuatorType::Unknown:
      return "unknown";
  }
  return "unknown";
}

ToolType parseToolType(const String& value) {
  if (value.equalsIgnoreCase("servo_gripper")) {
    return ToolType::ServoGripper;
  }
  if (value.equalsIgnoreCase("electromagnet")) {
    return ToolType::Electromagnet;
  }
  return ToolType::Generic;
}

const char* toolTypeName(ToolType type) {
  switch (type) {
    case ToolType::ServoGripper:
      return "servo_gripper";
    case ToolType::Electromagnet:
      return "electromagnet";
    case ToolType::Generic:
      return "generic";
  }
  return "generic";
}

const char* stateName() {
  switch (controllerState) {
    case ControllerState::Idle:
      return "idle";
    case ControllerState::Moving:
      return "moving";
    case ControllerState::Stopped:
      return "stopped";
    case ControllerState::Estop:
      return "estop";
    case ControllerState::Fault:
      return "fault";
  }
  return "fault";
}

JointConfig defaultJoint(int index) {
  JointConfig joint;
  snprintf(joint.name, sizeof(joint.name), "%s", kDefaultNames[index]);
  joint.actuator = index < 2 ? ActuatorType::Stepper : ActuatorType::Servo;
  joint.enabled = false;
  joint.received = false;
  joint.zeroOffsetDeg = 0.0f;
  joint.directionSign = 1;
  joint.minDeg = kDefaultMin[index];
  joint.maxDeg = kDefaultMax[index];
  joint.homeDeg = kDefaultHome[index];
  joint.maxSpeedDegS = index < 2 ? 45.0f : 60.0f;
  joint.maxAccelDegS2 = index < 2 ? 120.0f : 180.0f;
  joint.axisState = AxisState::Simulated;
  return joint;
}

ToolConfig defaultTool() {
  ToolConfig tool;
  snprintf(tool.name, sizeof(tool.name), "generic");
  tool.type = ToolType::Generic;
  tool.received = false;
  return tool;
}

void resetDraftConfig() {
  for (int i = 0; i < kJointCount; i++) {
    draftJoints[i] = defaultJoint(i);
  }
  draftTool = defaultTool();
}

void clearFaultText() {
  strlcpy(faultText, "OK", sizeof(faultText));
}

void setFault(const char* message) {
  controllerState = ControllerState::Fault;
  strlcpy(faultText, message, sizeof(faultText));
}

String tokenValue(const String& line, const char* key, const String& fallback) {
  const String prefix = String(key) + "=";
  int start = line.indexOf(prefix);
  if (start < 0) {
    return fallback;
  }
  start += prefix.length();
  int end = line.indexOf(' ', start);
  if (end < 0) {
    end = line.length();
  }
  return line.substring(start, end);
}

int tokenInt(const String& line, const char* key, int fallback) {
  String value = tokenValue(line, key, "");
  if (value.length() == 0) {
    return fallback;
  }
  return value.toInt();
}

float tokenFloat(const String& line, const char* key, float fallback) {
  String value = tokenValue(line, key, "");
  if (value.length() == 0) {
    return fallback;
  }
  return value.toFloat();
}

String tokenString(const String& line, const char* key, const String& fallback) {
  return tokenValue(line, key, fallback);
}

void clearTrajectory() {
  trajectoryRuntime.expectedCount = 0;
  trajectoryRuntime.count = 0;
  trajectoryRuntime.receiving = false;
  trajectoryRuntime.ready = false;
  trajectoryRuntime.active = false;
  trajectoryRuntime.startUs = 0;
  trajectoryRuntime.durationS = 0.0f;
}

void clearJogMotion(bool freezeTarget = false) {
  jogActive = false;
  jogVelocityMode = false;
  jogStopRequested = false;
  lastJogMs = 0;
  jogLastUpdateUs = 0;
  for (int i = 0; i < kJointCount; i++) {
    jogTargetVelocityDegS[i] = 0.0f;
    jogCurrentVelocityDegS[i] = 0.0f;
    jogStepperRemainderSteps[i] = 0.0f;
  }
  if (!freezeTarget) {
    return;
  }
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  if (controllerState == ControllerState::Moving) {
    controllerState = ControllerState::Idle;
  }
}

String validationErrorForJoint(const JointConfig& joint, int index) {
  if (!joint.enabled) {
    return "";
  }
  if (joint.actuator == ActuatorType::Stepper) {
    if (joint.stepper.stepPin < 0 || !validPinOrUnused(joint.stepper.stepPin)) {
      return "joint_" + String(index + 1) + "_missing_step_pin";
    }
    if (joint.stepper.dirPin < 0 || !validPinOrUnused(joint.stepper.dirPin)) {
      return "joint_" + String(index + 1) + "_missing_dir_pin";
    }
    if (!validPinOrUnused(joint.stepper.enablePin)) {
      return "joint_" + String(index + 1) + "_invalid_optional_pin";
    }
    if (joint.stepper.fullStepsPerRev <= 0) {
      return "joint_" + String(index + 1) + "_invalid_full_steps";
    }
    if (joint.stepper.microsteps <= 0) {
      return "joint_" + String(index + 1) + "_invalid_microsteps";
    }
    if (joint.stepper.gearRatio <= 0.0f) {
      return "joint_" + String(index + 1) + "_invalid_gear";
    }
    return "";
  }
  if (joint.actuator == ActuatorType::Servo) {
    if (joint.servo.pwmPin < 0 || !validPinOrUnused(joint.servo.pwmPin)) {
      return "joint_" + String(index + 1) + "_missing_pwm_pin";
    }
    if (joint.servo.pulseMinUs <= 0 || joint.servo.pulseMaxUs <= joint.servo.pulseMinUs) {
      return "joint_" + String(index + 1) + "_invalid_pulse_range";
    }
    if (joint.servo.pwmFrequencyHz <= 0 || joint.servo.rangeDeg <= 0.0f || joint.servo.gearRatio <= 0.0f) {
      return "joint_" + String(index + 1) + "_invalid_servo_mapping";
    }
    return "";
  }
  return "joint_" + String(index + 1) + "_unsupported_actuator";
}

String validationErrorForTool(const ToolConfig& tool) {
  if (tool.type == ToolType::Generic) {
    return "";
  }
  if (tool.type == ToolType::ServoGripper) {
    if (tool.pwmPin < 0 || !validPinOrUnused(tool.pwmPin)) {
      return "active_tool_missing_pwm_pin";
    }
    if (tool.pulseMinUs <= 0 || tool.pulseMaxUs <= tool.pulseMinUs) {
      return "active_tool_invalid_pulse_range";
    }
    if (tool.pwmFrequencyHz <= 0) {
      return "active_tool_invalid_pwm_frequency";
    }
    if (tool.openValue < 0.0f || tool.openValue > 1.0f || tool.closedValue < 0.0f || tool.closedValue > 1.0f) {
      return "active_tool_open_close_values_out_of_range";
    }
    return "";
  }
  if (tool.type == ToolType::Electromagnet) {
    if (tool.gpioPin < 0 || !validPinOrUnused(tool.gpioPin)) {
      return "active_tool_missing_gpio_pin";
    }
    return "";
  }
  return "";
}

String classifyConfig(JointConfig config[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    config[i].axisState = AxisState::Simulated;
    const String error = validationErrorForJoint(config[i], i);
    if (error.length() > 0) {
      config[i].axisState = AxisState::Invalid;
      return error;
    }
    if (config[i].enabled) {
      config[i].axisState = AxisState::Hardware;
    }
  }
  return "";
}

String hardwareMode() {
  int hardwareCount = 0;
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Invalid) {
      return "invalid";
    }
    if (joints[i].axisState == AxisState::Hardware) {
      hardwareCount++;
    }
  }
  if (hardwareCount == kJointCount) {
    return "hardware";
  }
  if (hardwareCount > 0) {
    return "mixed";
  }
  return "simulated";
}

String enabledBits() {
  String bits;
  for (int i = 0; i < kJointCount; i++) {
    bits += joints[i].axisState == AxisState::Hardware ? "1" : "0";
  }
  return bits;
}

String encoderBits() {
  String bits;
  for (int i = 0; i < kJointCount; i++) {
    bits += encoderAvailable[i] ? "1" : "0";
  }
  return bits;
}

const char* poseSourceName() {
  const bool encoderPose = encoderAvailable[0] || encoderAvailable[1];
  if (encoderPose && homed) {
    return "mixed";
  }
  if (encoderPose) {
    return "encoder";
  }
  return poseSourceText;
}

const char* closedLoopModeName() {
#if defined(ARM_CONTROLLER_ENABLE_AS5048A)
  return "readback";
#else
  return "off";
#endif
}

bool configHasInvalidAxis() {
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Invalid) {
      return true;
    }
  }
  return false;
}

float stepperStepsPerDegree(int index) {
  const StepperConfig& stepper = joints[index].stepper;
  return (static_cast<float>(stepper.fullStepsPerRev) * static_cast<float>(stepper.microsteps) * stepper.gearRatio) /
         360.0f;
}

long jointDegToSteps(int index, float jointDeg) {
  const float signedDeg = (jointDeg + joints[index].zeroOffsetDeg) * static_cast<float>(joints[index].directionSign);
  return lroundf(signedDeg * stepperStepsPerDegree(index));
}

float stepsToJointDeg(int index, long steps) {
  const float scale = stepperStepsPerDegree(index);
  if (scale <= 0.0f) {
    return currentJointsDeg[index];
  }
  const float signedDeg = static_cast<float>(steps) / scale;
  return signedDeg / static_cast<float>(joints[index].directionSign) - joints[index].zeroOffsetDeg;
}

int servoPulseForJoint(int index, float jointDeg) {
  const ServoConfig& servo = joints[index].servo;
  const float servoDeg =
      servo.neutralDeg + static_cast<float>(joints[index].directionSign) * (jointDeg + joints[index].zeroOffsetDeg) *
                             servo.gearRatio;
  const float clampedDeg = clampFloat(servoDeg, 0.0f, servo.rangeDeg);
  const float fraction = clampedDeg / servo.rangeDeg;
  return static_cast<int>(roundf(static_cast<float>(servo.pulseMinUs) +
                                 fraction * static_cast<float>(servo.pulseMaxUs - servo.pulseMinUs)));
}

uint32_t servoDutyForPulse(int index, int pulseUs) {
  const uint32_t frequency = static_cast<uint32_t>(max(1, joints[index].servo.pwmFrequencyHz));
  const float periodUs = 1000000.0f / static_cast<float>(frequency);
  const float duty = clampFloat(static_cast<float>(pulseUs) / periodUs, 0.0f, 1.0f);
  return static_cast<uint32_t>(roundf(duty * static_cast<float>(kServoPwmMaxDuty)));
}

void writeServoPwm(int index, bool enabled) {
  ServoRuntime& runtime = servoRuntime[index];
  if (!runtime.attached || runtime.channel < 0) {
    return;
  }
  ledcWrite(runtime.channel, enabled ? servoDutyForPulse(index, runtime.pulseUs) : 0);
}

void writeStepperEnable(int index, bool enabled) {
  const StepperConfig& stepper = joints[index].stepper;
  if (stepper.enablePin < 0) {
    return;
  }
  const bool activeLevel = stepper.enableActiveLow ? LOW : HIGH;
  digitalWrite(stepper.enablePin, enabled ? activeLevel : !activeLevel);
}

int toolPulseForValue(float value) {
  const float clamped = clampFloat(value, 0.0f, 1.0f);
  return static_cast<int>(roundf(static_cast<float>(activeTool.pulseMinUs) +
                                 clamped * static_cast<float>(activeTool.pulseMaxUs - activeTool.pulseMinUs)));
}

uint32_t toolDutyForPulse(int pulseUs) {
  const uint32_t frequency = static_cast<uint32_t>(max(1, activeTool.pwmFrequencyHz));
  const float periodUs = 1000000.0f / static_cast<float>(frequency);
  const float duty = clampFloat(static_cast<float>(pulseUs) / periodUs, 0.0f, 1.0f);
  return static_cast<uint32_t>(roundf(duty * static_cast<float>(kServoPwmMaxDuty)));
}

void writeToolOutput() {
  if (activeTool.type == ToolType::ServoGripper && toolRuntime.attached) {
    ledcWrite(toolRuntime.channel, toolDutyForPulse(toolPulseForValue(toolValue)));
  } else if (activeTool.type == ToolType::Electromagnet && activeTool.gpioPin >= 0) {
    const bool on = toolValue >= 0.5f;
    const int activeLevel = activeTool.activeHigh ? HIGH : LOW;
    digitalWrite(activeTool.gpioPin, on ? activeLevel : !activeLevel);
  }
}

void setToolSafe() {
  if (activeTool.type == ToolType::ServoGripper) {
    toolValue = clampFloat(activeTool.openValue, 0.0f, 1.0f);
    strlcpy(toolState, "open", sizeof(toolState));
    writeToolOutput();
  } else if (activeTool.type == ToolType::Electromagnet) {
    toolValue = 0.0f;
    strlcpy(toolState, "off", sizeof(toolState));
    writeToolOutput();
  } else {
    toolValue = 0.0f;
    strlcpy(toolState, "unknown", sizeof(toolState));
  }
}

void releaseToolOutputs() {
  if (toolRuntime.attached && toolRuntime.attachedPwmPin >= 0) {
    ledcWrite(toolRuntime.channel, 0);
    ledcDetachPin(toolRuntime.attachedPwmPin);
  }
  if (toolRuntime.gpioPin >= 0) {
    const int activeLevel = toolRuntime.gpioActiveHigh ? HIGH : LOW;
    digitalWrite(toolRuntime.gpioPin, !activeLevel);
  }
  toolRuntime.attached = false;
  toolRuntime.attachedPwmPin = -1;
  toolRuntime.gpioPin = -1;
  toolRuntime.gpioActiveHigh = true;
}

void configureToolPins() {
  releaseToolOutputs();
  if (activeTool.type == ToolType::ServoGripper && activeTool.pwmPin >= 0) {
    toolRuntime.channel = kServoPwmChannelBase + kJointCount + 1;
    ledcSetup(toolRuntime.channel, activeTool.pwmFrequencyHz, kServoPwmResolutionBits);
    ledcAttachPin(activeTool.pwmPin, toolRuntime.channel);
    toolRuntime.attached = true;
    toolRuntime.attachedPwmPin = activeTool.pwmPin;
  } else if (activeTool.type == ToolType::Electromagnet && activeTool.gpioPin >= 0) {
    pinMode(activeTool.gpioPin, OUTPUT);
    toolRuntime.gpioPin = activeTool.gpioPin;
    toolRuntime.gpioActiveHigh = activeTool.activeHigh;
  }
  setToolSafe();
}

void disableHardwareOutputs() {
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      writeStepperEnable(i, false);
    }
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
        joints[i].servo.pwmPin >= 0) {
      writeServoPwm(i, false);
    }
  }
  setToolSafe();
}

void configurePins() {
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware) {
      continue;
    }
    if (joints[i].actuator == ActuatorType::Stepper) {
      const StepperConfig& stepper = joints[i].stepper;
      pinMode(stepper.stepPin, OUTPUT);
      pinMode(stepper.dirPin, OUTPUT);
      digitalWrite(stepper.stepPin, LOW);
      digitalWrite(stepper.dirPin, LOW);
      if (stepper.enablePin >= 0) {
        pinMode(stepper.enablePin, OUTPUT);
        writeStepperEnable(i, false);
      }
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].channel = kServoPwmChannelBase + i;
      ledcSetup(servoRuntime[i].channel, joints[i].servo.pwmFrequencyHz, kServoPwmResolutionBits);
      ledcAttachPin(joints[i].servo.pwmPin, servoRuntime[i].channel);
      servoRuntime[i].attached = true;
      writeServoPwm(i, false);
    }
  }
}

void syncRuntimeFromCurrentPose() {
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].currentSteps = jointDegToSteps(i, currentJointsDeg[i]);
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
      servoRuntime[i].pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
      writeServoPwm(i, armed && joints[i].axisState == AxisState::Hardware);
    }
  }
}

#if defined(ARM_CONTROLLER_ENABLE_AS5048A)
uint16_t encoderTransfer16(int csPin, uint16_t value) {
  SPI.beginTransaction(encoderSpiSettings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  const uint16_t response = SPI.transfer16(value);
  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);
  return response;
}

bool readAs5048AngleDeg(int csPin, float& angleDeg) {
  encoderTransfer16(csPin, kAs5048ReadAngleCommand);
  const uint16_t response = encoderTransfer16(csPin, 0x0000);
  const bool error = (response & kAs5048ErrorFlag) != 0;
  if (error) {
    encoderTransfer16(csPin, kAs5048ClearErrorCommand);
    encoderTransfer16(csPin, 0x0000);
    return false;
  }
  const uint16_t raw = response & kAs5048AngleMask;
  angleDeg = static_cast<float>(raw) * 360.0f / 16384.0f;
  return true;
}

void configureEncoderPins() {
  SPI.begin(ARM_ENCODER_SCK_PIN, ARM_ENCODER_MISO_PIN, ARM_ENCODER_MOSI_PIN);
  for (int i = 0; i < kJointCount; i++) {
    if (kEncoderCsPins[i] >= 0) {
      pinMode(kEncoderCsPins[i], OUTPUT);
      digitalWrite(kEncoderCsPins[i], HIGH);
    }
  }
}

void updateEncoderReadback() {
  for (int i = 0; i < kJointCount; i++) {
    if (kEncoderCsPins[i] < 0) {
      encoderAvailable[i] = false;
      continue;
    }
    float angleDeg = 0.0f;
    encoderAvailable[i] = readAs5048AngleDeg(kEncoderCsPins[i], angleDeg);
    if (encoderAvailable[i]) {
      encoderAnglesDeg[i] = angleDeg;
    }
  }
}
#else
void configureEncoderPins() {}

void updateEncoderReadback() {}
#endif

bool validateJointLimits(const float requested[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    if (requested[i] < joints[i].minDeg || requested[i] > joints[i].maxDeg) {
      ARM_SERIAL.printf("ERR code=LIMIT message=j%d_out_of_range\r\n", i + 1);
      return false;
    }
  }
  return true;
}

bool hasHardwareMotionPending() {
  if (trajectoryRuntime.active) {
    return true;
  }
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper &&
        stepperRuntime[i].currentSteps != stepperRuntime[i].targetSteps) {
      return true;
    }
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
        (fabsf(targetJointsDeg[i] - currentJointsDeg[i]) > 0.05f || fabsf(servoRuntime[i].velocityDegS) > 0.05f)) {
      return true;
    }
  }
  return false;
}

void setTargetPose(const float requested[kJointCount]) {
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = requested[i];
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = jointDegToSteps(i, requested[i]);
    } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].lastUpdateUs = nowUs;
    } else {
      currentJointsDeg[i] = requested[i];
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, requested[i]);
        writeServoPwm(i, armed && joints[i].axisState == AxisState::Hardware);
      }
    }
  }
  controllerState = hasHardwareMotionPending() ? ControllerState::Moving : ControllerState::Idle;
  clearFaultText();
}

void setTrajectoryTargetPose(const float requested[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = requested[i];
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = jointDegToSteps(i, requested[i]);
    } else if (joints[i].axisState != AxisState::Hardware) {
      currentJointsDeg[i] = requested[i];
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, requested[i]);
      }
    }
  }
  controllerState = ControllerState::Moving;
  clearFaultText();
}

float trajectoryDerivative(int pointIndex, int jointIndex) {
  const int count = trajectoryRuntime.count;
  if (count < 2) {
    return 0.0f;
  }
  if (pointIndex <= 0) {
    const float dt = max(0.001f, trajectoryRuntime.points[1].timeS - trajectoryRuntime.points[0].timeS);
    return (trajectoryRuntime.points[1].jointsDeg[jointIndex] - trajectoryRuntime.points[0].jointsDeg[jointIndex]) / dt;
  }
  if (pointIndex >= count - 1) {
    const float dt = max(0.001f, trajectoryRuntime.points[count - 1].timeS - trajectoryRuntime.points[count - 2].timeS);
    return (trajectoryRuntime.points[count - 1].jointsDeg[jointIndex] -
            trajectoryRuntime.points[count - 2].jointsDeg[jointIndex]) /
           dt;
  }
  const float dt =
      max(0.001f, trajectoryRuntime.points[pointIndex + 1].timeS - trajectoryRuntime.points[pointIndex - 1].timeS);
  return (trajectoryRuntime.points[pointIndex + 1].jointsDeg[jointIndex] -
          trajectoryRuntime.points[pointIndex - 1].jointsDeg[jointIndex]) /
         dt;
}

void interpolateTrajectory(float elapsedS, float out[kJointCount]) {
  const int count = trajectoryRuntime.count;
  if (count <= 0) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = currentJointsDeg[i];
    }
    return;
  }
  if (elapsedS <= trajectoryRuntime.points[0].timeS || count == 1) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = trajectoryRuntime.points[0].jointsDeg[i];
    }
    return;
  }
  if (elapsedS >= trajectoryRuntime.points[count - 1].timeS) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = trajectoryRuntime.points[count - 1].jointsDeg[i];
    }
    return;
  }

  int segment = 0;
  while (segment < count - 2 && elapsedS > trajectoryRuntime.points[segment + 1].timeS) {
    segment++;
  }
  const TrajectoryPoint& p0 = trajectoryRuntime.points[segment];
  const TrajectoryPoint& p1 = trajectoryRuntime.points[segment + 1];
  const float dt = max(0.001f, p1.timeS - p0.timeS);
  const float u = clampFloat((elapsedS - p0.timeS) / dt, 0.0f, 1.0f);
  const float u2 = u * u;
  const float u3 = u2 * u;
  const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
  const float h10 = u3 - 2.0f * u2 + u;
  const float h01 = -2.0f * u3 + 3.0f * u2;
  const float h11 = u3 - u2;

  for (int joint = 0; joint < kJointCount; joint++) {
    const float m0 = trajectoryDerivative(segment, joint);
    const float m1 = trajectoryDerivative(segment + 1, joint);
    const float raw = h00 * p0.jointsDeg[joint] + h10 * dt * m0 + h01 * p1.jointsDeg[joint] + h11 * dt * m1;
    const float low = min(p0.jointsDeg[joint], p1.jointsDeg[joint]);
    const float high = max(p0.jointsDeg[joint], p1.jointsDeg[joint]);
    out[joint] = clampFloat(raw, low, high);
  }
}

void updateTrajectoryFollower(unsigned long nowUs) {
  if (!trajectoryRuntime.active) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    trajectoryRuntime.active = false;
    trajectoryRuntime.ready = false;
    return;
  }

  const float elapsedS = static_cast<float>(nowUs - trajectoryRuntime.startUs) / 1000000.0f;
  float requested[kJointCount] = {};
  interpolateTrajectory(elapsedS, requested);
  setTrajectoryTargetPose(requested);

  if (elapsedS >= trajectoryRuntime.durationS) {
    for (int i = 0; i < kJointCount; i++) {
      requested[i] = trajectoryRuntime.points[trajectoryRuntime.count - 1].jointsDeg[i];
    }
    trajectoryRuntime.active = false;
    trajectoryRuntime.ready = false;
    setTargetPose(requested);
  }
}

bool anyJogVelocityPending() {
  for (int i = 0; i < kJointCount; i++) {
    if (fabsf(jogTargetVelocityDegS[i]) > 0.001f || fabsf(jogCurrentVelocityDegS[i]) > 0.001f ||
        fabsf(jogStepperRemainderSteps[i]) >= 1.0f) {
      return true;
    }
  }
  return false;
}

void requestJogVelocityStop() {
  if (!jogVelocityMode) {
    clearJogMotion(true);
    return;
  }
  for (int i = 0; i < kJointCount; i++) {
    jogTargetVelocityDegS[i] = 0.0f;
  }
  jogStopRequested = true;
  jogActive = true;
  lastJogMs = millis();
  if (controllerState == ControllerState::Moving || anyJogVelocityPending()) {
    controllerState = ControllerState::Moving;
  }
}

void emitJogStepperSteps(int index, long stepsToEmit) {
  if (stepsToEmit == 0) {
    return;
  }
  StepperRuntime& runtime = stepperRuntime[index];
  const int direction = stepsToEmit > 0 ? 1 : -1;
  long remaining = stepsToEmit > 0 ? stepsToEmit : -stepsToEmit;
  digitalWrite(joints[index].stepper.dirPin, direction > 0 ? HIGH : LOW);
  while (remaining > 0) {
    digitalWrite(joints[index].stepper.stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(joints[index].stepper.stepPin, LOW);
    runtime.currentSteps += direction;
    remaining--;
  }
  currentJointsDeg[index] = stepsToJointDeg(index, runtime.currentSteps);
  targetJointsDeg[index] = currentJointsDeg[index];
}

void updateJogVelocity(unsigned long nowUs) {
  if (!jogActive || !jogVelocityMode) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    clearJogMotion(true);
    return;
  }
  if (jogLastUpdateUs == 0) {
    jogLastUpdateUs = nowUs;
    return;
  }

  const unsigned long elapsedUs = nowUs - jogLastUpdateUs;
  if (elapsedUs < 1000UL) {
    return;
  }
  jogLastUpdateUs = nowUs;
  const float dtS = clampFloat(static_cast<float>(elapsedUs) / 1000000.0f, 0.001f, 0.05f);
  const float accelLimit = max(0.1f, lastAccelDegS2);
  bool moving = false;

  for (int i = 0; i < kJointCount; i++) {
    const float axisSpeedLimit = max(0.1f, joints[i].maxSpeedDegS);
    jogTargetVelocityDegS[i] = clampFloat(jogTargetVelocityDegS[i], -axisSpeedLimit, axisSpeedLimit);
    const float maxVelocityStep = accelLimit * dtS;
    const float velocityDelta = jogTargetVelocityDegS[i] - jogCurrentVelocityDegS[i];
    if (velocityDelta > maxVelocityStep) {
      jogCurrentVelocityDegS[i] += maxVelocityStep;
    } else if (velocityDelta < -maxVelocityStep) {
      jogCurrentVelocityDegS[i] -= maxVelocityStep;
    } else {
      jogCurrentVelocityDegS[i] = jogTargetVelocityDegS[i];
    }

    if (fabsf(jogCurrentVelocityDegS[i]) <= 0.001f && fabsf(jogTargetVelocityDegS[i]) <= 0.001f) {
      jogCurrentVelocityDegS[i] = 0.0f;
      targetJointsDeg[i] = currentJointsDeg[i];
      continue;
    }

    float nextDeg = currentJointsDeg[i] + jogCurrentVelocityDegS[i] * dtS;
    if (nextDeg > joints[i].maxDeg) {
      nextDeg = joints[i].maxDeg;
      jogCurrentVelocityDegS[i] = 0.0f;
      jogTargetVelocityDegS[i] = 0.0f;
    } else if (nextDeg < joints[i].minDeg) {
      nextDeg = joints[i].minDeg;
      jogCurrentVelocityDegS[i] = 0.0f;
      jogTargetVelocityDegS[i] = 0.0f;
    }

    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      const float stepDelta = (nextDeg - currentJointsDeg[i]) * static_cast<float>(joints[i].directionSign) *
                              stepperStepsPerDegree(i);
      const float accumulatedSteps = jogStepperRemainderSteps[i] + stepDelta;
      long stepsToEmit = static_cast<long>(accumulatedSteps);
      stepsToEmit = max(-80L, min(80L, stepsToEmit));
      jogStepperRemainderSteps[i] = accumulatedSteps - static_cast<float>(stepsToEmit);
      emitJogStepperSteps(i, stepsToEmit);
    } else {
      currentJointsDeg[i] = nextDeg;
      targetJointsDeg[i] = nextDeg;
      if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
          servoRuntime[i].attached && nowUs - servoRuntime[i].lastUpdateUs >= 20000UL) {
        servoRuntime[i].lastUpdateUs = nowUs;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
        writeServoPwm(i, true);
      }
    }

    if (fabsf(jogTargetVelocityDegS[i]) > 0.001f || fabsf(jogCurrentVelocityDegS[i]) > 0.001f ||
        fabsf(jogStepperRemainderSteps[i]) >= 1.0f) {
      moving = true;
    }
  }

  if (moving) {
    controllerState = ControllerState::Moving;
    clearFaultText();
  } else if (jogStopRequested) {
    clearJogMotion(true);
  } else {
    controllerState = ControllerState::Idle;
  }
}

void updateJogWatchdog(uint32_t nowMs) {
  if (!jogActive) {
    return;
  }
  if (nowMs - lastJogMs > kJogWatchdogMs) {
    requestJogVelocityStop();
  }
}

void printHello() {
  ARM_SERIAL.println("HELLO name=esp32s3-arm firmware=arm_controller protocol=3 config=1");
}

void printStatus() {
  updateEncoderReadback();
  const bool knownPose = homed || encoderAvailable[0] || encoderAvailable[1];
  ARM_SERIAL.printf(
      "STATUS state=%s homed=%d known=%d pose_source=%s armed=%d hw=%s enabled=%s enc=%s e1=%.2f e2=%.2f "
      "j1=%.2f j2=%.2f j3=%.2f j4=%.2f closed_loop=%s tool_type=%s tool=%s tool_value=%.3f fault=%s\r\n",
      stateName(), homed ? 1 : 0, knownPose ? 1 : 0, poseSourceName(), armed ? 1 : 0, hardwareMode().c_str(),
      enabledBits().c_str(), encoderBits().c_str(), encoderAnglesDeg[0], encoderAnglesDeg[1], currentJointsDeg[0],
      currentJointsDeg[1], currentJointsDeg[2], currentJointsDeg[3], closedLoopModeName(), toolTypeName(activeTool.type),
      toolState, toolValue, faultText);
}

void printError(const char* code, const String& message) {
  ARM_SERIAL.printf("ERR code=%s message=%s\r\n", code, message.c_str());
}

void handleConfig(const String& rawCommand, const String& upperCommand) {
  if (upperCommand.startsWith("CONFIG BEGIN")) {
    if (controllerState == ControllerState::Moving) {
      printError("CONFIG", "stop_motion_before_config");
      return;
    }
    clearTrajectory();
    clearJogMotion(true);
    const int axes = tokenInt(rawCommand, "axes", 0);
    if (axes != kJointCount) {
      printError("CONFIG", "axes_must_be_4");
      return;
    }
    resetDraftConfig();
    configInProgress = true;
    return;
  }

  if (upperCommand.startsWith("CONFIG JOINT")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    const int index = tokenInt(rawCommand, "index", 0) - 1;
    if (index < 0 || index >= kJointCount) {
      printError("CONFIG", "invalid_joint_index");
      return;
    }
    JointConfig joint = defaultJoint(index);
    const String name = tokenString(rawCommand, "name", kDefaultNames[index]);
    snprintf(joint.name, sizeof(joint.name), "%s", name.c_str());
    joint.actuator = parseActuator(tokenString(rawCommand, "actuator", actuatorName(joint.actuator)));
    joint.enabled = tokenInt(rawCommand, "enabled", 0) != 0;
    joint.zeroOffsetDeg = tokenFloat(rawCommand, "zero", 0.0f);
    joint.directionSign = tokenInt(rawCommand, "sign", 1) < 0 ? -1 : 1;
    joint.minDeg = tokenFloat(rawCommand, "min", joint.minDeg);
    joint.maxDeg = tokenFloat(rawCommand, "max", joint.maxDeg);
    joint.homeDeg = tokenFloat(rawCommand, "home", joint.homeDeg);
    joint.maxSpeedDegS = tokenFloat(rawCommand, "max_speed", joint.maxSpeedDegS);
    joint.maxAccelDegS2 = tokenFloat(rawCommand, "max_accel", joint.maxAccelDegS2);
    if (joint.actuator == ActuatorType::Stepper) {
      joint.stepper.stepPin = tokenInt(rawCommand, "step", -1);
      joint.stepper.dirPin = tokenInt(rawCommand, "dir", -1);
      joint.stepper.enablePin = tokenInt(rawCommand, "enable", -1);
      joint.stepper.enableActiveLow = tokenInt(rawCommand, "enable_low", 1) != 0;
      joint.stepper.fullStepsPerRev = tokenInt(rawCommand, "full_steps", 200);
      joint.stepper.microsteps = tokenInt(rawCommand, "microsteps", 16);
      joint.stepper.gearRatio = tokenFloat(rawCommand, "gear", 1.0f);
    } else if (joint.actuator == ActuatorType::Servo) {
      joint.servo.pwmPin = tokenInt(rawCommand, "pwm", -1);
      joint.servo.pulseMinUs = tokenInt(rawCommand, "min_us", 500);
      joint.servo.pulseMaxUs = tokenInt(rawCommand, "max_us", 2500);
      joint.servo.pwmFrequencyHz = tokenInt(rawCommand, "freq", 50);
      joint.servo.rangeDeg = tokenFloat(rawCommand, "servo_range", 270.0f);
      joint.servo.neutralDeg = tokenFloat(rawCommand, "neutral", 135.0f);
      joint.servo.gearRatio = tokenFloat(rawCommand, "gear", 1.0f);
    }
    joint.received = true;
    draftJoints[index] = joint;
    return;
  }

  if (upperCommand.startsWith("CONFIG TOOL")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    const bool active = tokenInt(rawCommand, "active", 0) != 0;
    if (!active) {
      return;
    }
    ToolConfig tool = defaultTool();
    const String name = tokenString(rawCommand, "name", "tool");
    snprintf(tool.name, sizeof(tool.name), "%s", name.c_str());
    tool.type = parseToolType(tokenString(rawCommand, "type", "generic"));
    tool.pwmPin = tokenInt(rawCommand, "pwm", -1);
    tool.pulseMinUs = tokenInt(rawCommand, "min_us", 500);
    tool.pulseMaxUs = tokenInt(rawCommand, "max_us", 2500);
    tool.pwmFrequencyHz = tokenInt(rawCommand, "freq", 50);
    tool.openValue = clampFloat(tokenFloat(rawCommand, "open", 0.0f), 0.0f, 1.0f);
    tool.closedValue = clampFloat(tokenFloat(rawCommand, "close", 1.0f), 0.0f, 1.0f);
    tool.gpioPin = tokenInt(rawCommand, "pin", -1);
    tool.activeHigh = tokenInt(rawCommand, "active_high", 1) != 0;
    tool.received = true;
    draftTool = tool;
    return;
  }

  if (upperCommand.startsWith("CONFIG END")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    configInProgress = false;
    for (int i = 0; i < kJointCount; i++) {
      if (!draftJoints[i].received) {
        printError("CONFIG", "missing_joint_" + String(i + 1));
        return;
      }
      if (draftJoints[i].minDeg >= draftJoints[i].maxDeg) {
        printError("CONFIG", "joint_" + String(i + 1) + "_invalid_limits");
        return;
      }
      if (draftJoints[i].homeDeg < draftJoints[i].minDeg || draftJoints[i].homeDeg > draftJoints[i].maxDeg) {
        printError("CONFIG", "joint_" + String(i + 1) + "_home_out_of_range");
        return;
      }
    }
    const String error = classifyConfig(draftJoints);
    if (error.length() > 0) {
      printError("CONFIG", error);
      return;
    }
    const String toolError = validationErrorForTool(draftTool);
    if (toolError.length() > 0) {
      printError("CONFIG", toolError);
      return;
    }
    disableHardwareOutputs();
    for (int i = 0; i < kJointCount; i++) {
      joints[i] = draftJoints[i];
    }
    activeTool = draftTool;
    configurePins();
    configureToolPins();
    syncRuntimeFromCurrentPose();
    if (armed) {
      for (int i = 0; i < kJointCount; i++) {
        if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
          writeStepperEnable(i, true);
        } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
          writeServoPwm(i, true);
        }
      }
    }
    clearFaultText();
    ARM_SERIAL.printf("OK command=CONFIG axes=4 hw=%s enabled=%s\r\n", hardwareMode().c_str(), enabledBits().c_str());
    printStatus();
    return;
  }

  printError("CONFIG", "unknown_config_command");
}

void handleArm(const String& rawCommand) {
  const int requested = rawCommand.substring(rawCommand.indexOf(' ') + 1).toInt();
  if (requested != 0) {
    if (controllerState == ControllerState::Estop) {
      printError("ESTOP", "reset_required");
      return;
    }
    if (configHasInvalidAxis()) {
      printError("CONFIG", "hardware_config_invalid");
      return;
    }
    armed = true;
    for (int i = 0; i < kJointCount; i++) {
      if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
        writeStepperEnable(i, true);
      } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
        writeServoPwm(i, true);
      }
    }
    clearFaultText();
  } else {
    armed = false;
    clearTrajectory();
    clearJogMotion(true);
    const unsigned long nowUs = micros();
    for (int i = 0; i < kJointCount; i++) {
      targetJointsDeg[i] = currentJointsDeg[i];
      if (joints[i].actuator == ActuatorType::Stepper) {
        stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
      } else if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
      }
    }
    disableHardwareOutputs();
    if (controllerState != ControllerState::Estop) {
      controllerState = ControllerState::Stopped;
      clearFaultText();
    }
  }
  ARM_SERIAL.printf("OK command=ARM armed=%d\r\n", armed ? 1 : 0);
  printStatus();
}

void handleMoveJ(const char* buffer) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }

  float requested[kJointCount] = {};
  float speed = 0.0f;
  float accel = 0.0f;
  const int parsed =
      sscanf(buffer, "%*s %f %f %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3], &speed, &accel);
  if (parsed != 6) {
    printError("USAGE", "MOVEJ_requires_j1_j2_j3_j4_speed_accel");
    return;
  }
  if (speed <= 0.0f || accel <= 0.0f) {
    printError("LIMIT", "speed_and_accel_must_be_positive");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }

  lastSpeedDegS = speed;
  lastAccelDegS2 = accel;
  clearTrajectory();
  clearJogMotion(false);
  setTargetPose(requested);
  ARM_SERIAL.printf("OK command=MOVEJ hw=%s\r\n", hardwareMode().c_str());
  printStatus();
}

void handleJog(const String& rawCommand, const String& upperCommand, const char* buffer) {
  (void)rawCommand;
  if (upperCommand.startsWith("JOG STOP")) {
    requestJogVelocityStop();
    ARM_SERIAL.println("OK command=JOG_STOP");
    printStatus();
    return;
  }
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }
  if (trajectoryRuntime.active || trajectoryRuntime.receiving) {
    printError("STATE", "trajectory_active");
    return;
  }

  if (upperCommand.startsWith("JOGV")) {
    float requestedVelocity[kJointCount] = {};
    float accel = 0.0f;
    const int parsed =
        sscanf(buffer, "%*s %f %f %f %f %f", &requestedVelocity[0], &requestedVelocity[1], &requestedVelocity[2],
               &requestedVelocity[3], &accel);
    if (parsed != 5) {
      printError("USAGE", "JOGV_requires_v1_v2_v3_v4_accel");
      return;
    }
    if (accel <= 0.0f) {
      printError("LIMIT", "accel_must_be_positive");
      return;
    }

    clearTrajectory();
    lastAccelDegS2 = accel;
    lastSpeedDegS = 0.0f;
    bool moving = false;
    for (int i = 0; i < kJointCount; i++) {
      const float speedLimit = max(0.1f, joints[i].maxSpeedDegS);
      float velocity = clampFloat(requestedVelocity[i], -speedLimit, speedLimit);
      if ((currentJointsDeg[i] >= joints[i].maxDeg && velocity > 0.0f) ||
          (currentJointsDeg[i] <= joints[i].minDeg && velocity < 0.0f)) {
        velocity = 0.0f;
      }
      jogTargetVelocityDegS[i] = velocity;
      lastSpeedDegS = max(lastSpeedDegS, fabsf(velocity));
      if (fabsf(velocity) > 0.001f) {
        moving = true;
      }
    }
    jogActive = true;
    jogVelocityMode = true;
    jogStopRequested = false;
    lastJogMs = millis();
    jogLastUpdateUs = micros();
    controllerState = moving || anyJogVelocityPending() ? ControllerState::Moving : ControllerState::Idle;
    clearFaultText();
    ARM_SERIAL.printf("OK command=JOGV hw=%s\r\n", hardwareMode().c_str());
    return;
  }

  float requested[kJointCount] = {};
  float speed = 0.0f;
  float accel = 0.0f;
  const int parsed =
      sscanf(buffer, "%*s %f %f %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3], &speed, &accel);
  if (parsed != 6) {
    printError("USAGE", "JOGJ_requires_j1_j2_j3_j4_speed_accel");
    return;
  }
  if (speed <= 0.0f || accel <= 0.0f) {
    printError("LIMIT", "speed_and_accel_must_be_positive");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }

  clearTrajectory();
  clearJogMotion(false);
  lastSpeedDegS = speed;
  lastAccelDegS2 = accel;
  jogActive = true;
  jogVelocityMode = false;
  lastJogMs = millis();
  setTargetPose(requested);
  ARM_SERIAL.printf("OK command=JOGJ hw=%s\r\n", hardwareMode().c_str());
}

void handleTrajectory(const String& rawCommand, const String& upperCommand) {
  if (upperCommand.startsWith("TRAJ CLEAR")) {
    clearTrajectory();
    clearJogMotion(false);
    ARM_SERIAL.println("OK command=TRAJ_CLEAR");
    return;
  }
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }

  if (upperCommand.startsWith("TRAJ BEGIN")) {
    if (controllerState == ControllerState::Moving || trajectoryRuntime.active) {
      printError("STATE", "trajectory_requires_idle");
      return;
    }
    const int count = tokenInt(rawCommand, "count", 0);
    const float durationS = tokenFloat(rawCommand, "duration", 0.0f);
    const float speed = tokenFloat(rawCommand, "speed", 0.0f);
    const float accel = tokenFloat(rawCommand, "accel", 0.0f);
    if (count < 2 || count > kMaxTrajectoryPoints) {
      printError("LIMIT", "trajectory_count_out_of_range");
      return;
    }
    if (durationS <= 0.0f || speed <= 0.0f || accel <= 0.0f) {
      printError("LIMIT", "trajectory_duration_speed_accel_must_be_positive");
      return;
    }
    clearTrajectory();
    clearJogMotion(false);
    trajectoryRuntime.expectedCount = count;
    trajectoryRuntime.durationS = durationS;
    trajectoryRuntime.receiving = true;
    lastSpeedDegS = speed;
    lastAccelDegS2 = accel;
    ARM_SERIAL.printf("OK command=TRAJ_BEGIN count=%d duration=%.3f\r\n", count, durationS);
    return;
  }

  if (upperCommand.startsWith("TRAJ POINT")) {
    if (!trajectoryRuntime.receiving || trajectoryRuntime.active) {
      printError("STATE", "trajectory_begin_required");
      return;
    }
    const int index = tokenInt(rawCommand, "index", -1);
    if (index != trajectoryRuntime.count || index < 0 || index >= trajectoryRuntime.expectedCount) {
      printError("USAGE", "trajectory_point_index_must_be_sequential");
      return;
    }
    TrajectoryPoint point;
    point.timeS = tokenFloat(rawCommand, "t", -1.0f);
    for (int i = 0; i < kJointCount; i++) {
      const String key = String("j") + String(i + 1);
      point.jointsDeg[i] = tokenFloat(rawCommand, key.c_str(), NAN);
    }
    if (!isfinite(point.timeS)) {
      printError("USAGE", "trajectory_point_requires_time");
      return;
    }
    if (index == 0 && fabsf(point.timeS) > 0.001f) {
      printError("USAGE", "trajectory_first_point_time_must_be_zero");
      return;
    }
    if (index > 0 && point.timeS <= trajectoryRuntime.points[index - 1].timeS) {
      printError("USAGE", "trajectory_point_times_must_increase");
      return;
    }
    if (index == trajectoryRuntime.expectedCount - 1 && fabsf(point.timeS - trajectoryRuntime.durationS) > 0.05f) {
      printError("USAGE", "trajectory_last_point_time_must_match_duration");
      return;
    }
    for (int i = 0; i < kJointCount; i++) {
      if (!isfinite(point.jointsDeg[i])) {
        printError("USAGE", "trajectory_point_requires_j1_j2_j3_j4");
        return;
      }
    }
    if (!validateJointLimits(point.jointsDeg)) {
      return;
    }
    trajectoryRuntime.points[index] = point;
    trajectoryRuntime.count++;
    trajectoryRuntime.ready = trajectoryRuntime.count == trajectoryRuntime.expectedCount;
    ARM_SERIAL.printf("OK command=TRAJ_POINT index=%d\r\n", index);
    return;
  }

  if (upperCommand.startsWith("TRAJ START")) {
    if (!trajectoryRuntime.ready || trajectoryRuntime.count != trajectoryRuntime.expectedCount) {
      printError("STATE", "trajectory_not_ready");
      return;
    }
    trajectoryRuntime.receiving = false;
    trajectoryRuntime.active = true;
    trajectoryRuntime.startUs = micros();
    controllerState = ControllerState::Moving;
    clearFaultText();
    ARM_SERIAL.printf(
        "OK command=TRAJ_START count=%d duration=%.3f\r\n", trajectoryRuntime.count, trajectoryRuntime.durationS);
    return;
  }

  printError("USAGE", "TRAJ_requires_BEGIN_POINT_START_or_CLEAR");
}

void handleHome() {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  float requested[kJointCount] = {};
  for (int i = 0; i < kJointCount; i++) {
    requested[i] = joints[i].homeDeg;
  }
  clearTrajectory();
  clearJogMotion(false);
  setTargetPose(requested);
  homed = true;
  strlcpy(poseSourceText, "home", sizeof(poseSourceText));
  ARM_SERIAL.println("OK command=HOME");
  printStatus();
}

void handleSetPose(const char* buffer) {
  if (armed) {
    printError("ARM", "setpose_requires_disarmed");
    return;
  }
  if (controllerState == ControllerState::Moving) {
    printError("STATE", "setpose_requires_stopped");
    return;
  }
  float requested[kJointCount] = {};
  const int parsed = sscanf(buffer, "%*s %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3]);
  if (parsed != 4) {
    printError("USAGE", "SETPOSE_requires_j1_j2_j3_j4");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }
  for (int i = 0; i < kJointCount; i++) {
    currentJointsDeg[i] = requested[i];
    targetJointsDeg[i] = requested[i];
  }
  clearTrajectory();
  clearJogMotion(false);
  syncRuntimeFromCurrentPose();
  homed = true;
  strlcpy(poseSourceText, "setpose", sizeof(poseSourceText));
  controllerState = ControllerState::Stopped;
  clearFaultText();
  ARM_SERIAL.println("OK command=SETPOSE");
  printStatus();
}

void handleStop() {
  clearTrajectory();
  clearJogMotion(true);
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  if (controllerState != ControllerState::Estop) {
    controllerState = ControllerState::Stopped;
    clearFaultText();
  }
  setToolSafe();
  ARM_SERIAL.println("OK command=STOP");
  printStatus();
}

void handleEstop() {
  armed = false;
  clearTrajectory();
  clearJogMotion(true);
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  disableHardwareOutputs();
  controllerState = ControllerState::Estop;
  strlcpy(faultText, "ESTOP", sizeof(faultText));
  ARM_SERIAL.println("OK command=ESTOP");
  printStatus();
}

void handleTool(const String& rawCommand, const String& upperCommand) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL OPEN")) {
    strlcpy(toolState, "open", sizeof(toolState));
    toolValue = clampFloat(activeTool.openValue, 0.0f, 1.0f);
  } else if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL CLOSE")) {
    strlcpy(toolState, "closed", sizeof(toolState));
    toolValue = clampFloat(activeTool.closedValue, 0.0f, 1.0f);
  } else if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL SET")) {
    toolValue = clampFloat(tokenFloat(rawCommand, "value", toolValue), 0.0f, 1.0f);
    strlcpy(toolState, "set", sizeof(toolState));
  } else if (activeTool.type == ToolType::Electromagnet && upperCommand.startsWith("TOOL ON")) {
    strlcpy(toolState, "on", sizeof(toolState));
    toolValue = 1.0f;
  } else if (activeTool.type == ToolType::Electromagnet && upperCommand.startsWith("TOOL OFF")) {
    strlcpy(toolState, "off", sizeof(toolState));
    toolValue = 0.0f;
  } else {
    printError("USAGE", "tool_command_not_supported_by_active_tool");
    return;
  }
  writeToolOutput();
  ARM_SERIAL.printf("OK command=TOOL state=%s value=%.3f\r\n", toolState, toolValue);
  printStatus();
}

unsigned long stepIntervalUs(int index) {
  const float speed = max(1.0f, min(lastSpeedDegS, joints[index].maxSpeedDegS));
  const float stepRate = max(1.0f, speed * stepperStepsPerDegree(index));
  return static_cast<unsigned long>(1000000.0f / stepRate);
}

void updateSteppers(unsigned long nowUs) {
  if (jogActive && jogVelocityMode) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped) {
    return;
  }
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware || joints[i].actuator != ActuatorType::Stepper) {
      continue;
    }
    StepperRuntime& runtime = stepperRuntime[i];
    const long delta = runtime.targetSteps - runtime.currentSteps;
    if (delta == 0) {
      continue;
    }
    const unsigned long interval = stepIntervalUs(i);
    if (nowUs - runtime.lastStepUs < interval) {
      continue;
    }
    runtime.lastStepUs = nowUs;
    digitalWrite(joints[i].stepper.dirPin, delta > 0 ? HIGH : LOW);
    digitalWrite(joints[i].stepper.stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(joints[i].stepper.stepPin, LOW);
    runtime.currentSteps += delta > 0 ? 1 : -1;
    currentJointsDeg[i] = stepsToJointDeg(i, runtime.currentSteps);
  }

  if (controllerState == ControllerState::Moving && !hasHardwareMotionPending()) {
    controllerState = ControllerState::Idle;
    clearFaultText();
  }
}

void updateServoPwm(unsigned long nowUs) {
  if (jogActive && jogVelocityMode) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    for (int i = 0; i < kJointCount; i++) {
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
      }
    }
    return;
  }

  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware || joints[i].actuator != ActuatorType::Servo) {
      continue;
    }
    ServoRuntime& runtime = servoRuntime[i];
    if (!runtime.attached) {
      continue;
    }
    if (runtime.lastUpdateUs == 0) {
      runtime.lastUpdateUs = nowUs;
      continue;
    }

    const unsigned long elapsedUs = nowUs - runtime.lastUpdateUs;
    if (elapsedUs < 20000UL) {
      continue;
    }
    runtime.lastUpdateUs = nowUs;

    const float dtS = static_cast<float>(elapsedUs) / 1000000.0f;
    const float delta = targetJointsDeg[i] - currentJointsDeg[i];
    if (fabsf(delta) <= 0.05f) {
      currentJointsDeg[i] = targetJointsDeg[i];
      runtime.velocityDegS = 0.0f;
      runtime.pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
      writeServoPwm(i, true);
      continue;
    }

    const float direction = delta > 0.0f ? 1.0f : -1.0f;
    const float speedLimit = max(0.1f, min(lastSpeedDegS, joints[i].maxSpeedDegS));
    const float accelLimit = max(0.1f, min(lastAccelDegS2, joints[i].maxAccelDegS2));
    const float stoppingDistance = (runtime.velocityDegS * runtime.velocityDegS) / (2.0f * accelLimit);
    float desiredVelocity = direction * speedLimit;
    if (runtime.velocityDegS * direction > 0.0f && stoppingDistance >= fabsf(delta)) {
      desiredVelocity = 0.0f;
    }

    const float maxVelocityStep = accelLimit * dtS;
    const float velocityDelta = desiredVelocity - runtime.velocityDegS;
    if (velocityDelta > maxVelocityStep) {
      runtime.velocityDegS += maxVelocityStep;
    } else if (velocityDelta < -maxVelocityStep) {
      runtime.velocityDegS -= maxVelocityStep;
    } else {
      runtime.velocityDegS = desiredVelocity;
    }

    const float step = runtime.velocityDegS * dtS;
    if (fabsf(step) >= fabsf(delta)) {
      currentJointsDeg[i] = targetJointsDeg[i];
      runtime.velocityDegS = 0.0f;
    } else {
      currentJointsDeg[i] += step;
    }
    runtime.pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
    writeServoPwm(i, true);
  }

  if (controllerState == ControllerState::Moving && !hasHardwareMotionPending()) {
    controllerState = ControllerState::Idle;
    clearFaultText();
  }
}

void handleCommand(String rawCommand) {
  rawCommand.trim();
  if (rawCommand.length() == 0) {
    return;
  }

  String upperCommand = rawCommand;
  upperCommand.toUpperCase();

  char buffer[kMaxLineLength] = {};
  rawCommand.toCharArray(buffer, sizeof(buffer));
  char command[24] = {};
  sscanf(buffer, "%23s", command);

  if (strcasecmp(command, "HELLO") == 0) {
    printHello();
  } else if (strcasecmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcasecmp(command, "CONFIG") == 0) {
    handleConfig(rawCommand, upperCommand);
  } else if (strcasecmp(command, "ARM") == 0) {
    handleArm(rawCommand);
  } else if (strcasecmp(command, "SETPOSE") == 0) {
    handleSetPose(buffer);
  } else if (strcasecmp(command, "MOVEJ") == 0) {
    handleMoveJ(buffer);
  } else if (strcasecmp(command, "JOGJ") == 0 || strcasecmp(command, "JOGV") == 0 || strcasecmp(command, "JOG") == 0) {
    handleJog(rawCommand, upperCommand, buffer);
  } else if (strcasecmp(command, "TRAJ") == 0) {
    handleTrajectory(rawCommand, upperCommand);
  } else if (strcasecmp(command, "STOP") == 0) {
    handleStop();
  } else if (strcasecmp(command, "ESTOP") == 0) {
    handleEstop();
  } else if (strcasecmp(command, "HOME") == 0) {
    handleHome();
  } else if (strcasecmp(command, "TOOL") == 0) {
    handleTool(rawCommand, upperCommand);
  } else {
    printError("UNKNOWN", command);
  }
}

void readSerialCommands() {
  while (ARM_SERIAL.available() > 0) {
    const char incoming = static_cast<char>(ARM_SERIAL.read());
    if (incoming == '\n' || incoming == '\r') {
      handleCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < kMaxLineLength - 1) {
      commandLine += incoming;
    }
  }
}

void turnOffOnboardRgbLed() {
#if defined(ESP32) && defined(ARM_CONTROLLER_ENABLE_ONBOARD_RGB_OFF)
  pinMode(ESP_RGB_LED_PIN, OUTPUT);
  digitalWrite(ESP_RGB_LED_PIN, LOW);
  neopixelWrite(ESP_RGB_LED_PIN, 0, 0, 0);
#endif
}
}  // namespace

void setup() {
  turnOffOnboardRgbLed();

  for (int i = 0; i < kJointCount; i++) {
    joints[i] = defaultJoint(i);
    currentJointsDeg[i] = kDefaultHome[i];
    targetJointsDeg[i] = kDefaultHome[i];
  }
  activeTool = defaultTool();
  resetDraftConfig();
  clearTrajectory();
  clearJogMotion(false);
  classifyConfig(joints);
  syncRuntimeFromCurrentPose();
  configureEncoderPins();
  clearFaultText();

  ARM_SERIAL.begin(115200);
  const unsigned long startMs = millis();
  while (!ARM_SERIAL && millis() - startMs < kSerialWaitMs) {
    delay(10);
  }

  lastStatusMs = millis();
  printHello();
  printStatus();
}

void loop() {
  readSerialCommands();
  const unsigned long nowUs = micros();
  const uint32_t nowMs = millis();
  updateJogWatchdog(nowMs);
  updateJogVelocity(nowUs);
  updateTrajectoryFollower(nowUs);
  updateSteppers(nowUs);
  updateServoPwm(nowUs);

  if (nowMs - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
