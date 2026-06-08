#include <Arduino.h>
#include <math.h>
#include <AccelStepper.h>

#define M1_STEP_PIN  19
#define M1_DIR_PIN   18
#define M2_STEP_PIN  17
#define M2_DIR_PIN   16

AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

const int   MICROSTEPS    = 8;
const int   STEPS_PER_REV = 200 * MICROSTEPS;
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0f;
const float DEG_PER_STEP  = 360.0f / STEPS_PER_REV;

const float LINK1_MM      = 202.5f;
const float LINK2_MM      = 177.5f;

const float TARGET_CART_SPEED_MM_S = 101.6f;
const float ACCEL         = 4000.0f;
const float HOME_X        = LINK1_MM + LINK2_MM;
const float LINE_STEP_MM  = 2.0f;

// ── Struct must be at file scope, before any function that uses it ──
struct JointSpeeds { float s1; float s2; };

// ============================================================
// PASTE YOUR G-CODE HERE
// ============================================================
const char* GCODE[] = {
    "G28 ; Home",
    "M5  ; Pen up",
    "M5",
    "G0 X178.01 Y-4.08",
    "M3",
    "G1 X176.92 Y-12.35 F600",
    "G1 X173.73 Y-20.06 F600",
    "G1 X168.65 Y-26.68 F600",
    "G1 X162.03 Y-31.76 F600",
    "G1 X154.32 Y-34.95 F600",
    "G1 X146.05 Y-36.04 F600",
    "G1 X137.78 Y-34.95 F600",
    "G1 X130.07 Y-31.76 F600",
    "G1 X123.45 Y-26.68 F600",
    "G1 X118.37 Y-20.06 F600",
    "G1 X115.18 Y-12.35 F600",
    "G1 X114.09 Y-4.08 F600",
    "G1 X115.18 Y4.19 F600",
    "G1 X118.37 Y11.90 F600",
    "G1 X123.45 Y18.51 F600",
    "G1 X130.07 Y23.59 F600",
    "G1 X137.78 Y26.79 F600",
    "G1 X146.05 Y27.87 F600",
    "G1 X154.32 Y26.79 F600",
    "G1 X162.03 Y23.59 F600",
    "G1 X168.65 Y18.51 F600",
    "G1 X173.73 Y11.90 F600",
    "G1 X176.92 Y4.19 F600",
    "G1 X178.01 Y-4.08 F600",
    "M5 ; Pen up",
    "G28 ; Home",
    "M0  ; Motors off",
    nullptr
};

const int MAX_WAYPOINTS = 128;
float WAYPOINTS[MAX_WAYPOINTS][2];
int   NUM_WAYPOINTS = 0;

float currentX  = HOME_X;
float currentY  = 0.0f;
float currentT1 = 0.0f;
float currentT2 = 0.0f;

volatile bool emergencyStop = false;

// ============================================================
// G-CODE PARSER
// ============================================================

float gcodeParam(const char* line, char param, float def) {
    const char* p = line;
    while (*p) {
        if (toupper(*p) == toupper(param)) {
            p++;
            while (*p == ' ') p++;
            char* end;
            float val = strtof(p, &end);
            if (end != p) return val;
        }
        p++;
    }
    return def;
}

void stripComment(const char* input, char* output, int maxLen) {
    int i = 0;
    while (input[i] && input[i] != ';' && i < maxLen - 1) {
        output[i] = toupper(input[i]);
        i++;
    }
    while (i > 0 && output[i-1] == ' ') i--;
    output[i] = '\0';
}

void addWaypoint(float x, float y) {
    if (NUM_WAYPOINTS >= MAX_WAYPOINTS) {
        Serial.println("WARNING: MAX_WAYPOINTS exceeded");
        return;
    }
    WAYPOINTS[NUM_WAYPOINTS][0] = x;
    WAYPOINTS[NUM_WAYPOINTS][1] = y;
    NUM_WAYPOINTS++;
}

void parseGcode() {
    NUM_WAYPOINTS = 0;
    float curX = HOME_X;
    float curY = 0.0f;

    Serial.println("=== Parsing G-code ===");

    for (int i = 0; GCODE[i] != nullptr; i++) {
        char line[64];
        stripComment(GCODE[i], line, sizeof(line));
        if (strlen(line) == 0) continue;

        Serial.printf("  [%d] %s\n", i, line);

        if (strncmp(line, "G28", 3) == 0) {
            addWaypoint(HOME_X, 0.0f);
            curX = HOME_X; curY = 0.0f;
            Serial.printf("      -> HOME (%.2f, 0.00)\n", HOME_X);
        }
        else if (strncmp(line, "G0", 2) == 0 || strncmp(line, "G1", 2) == 0) {
            float x = gcodeParam(line, 'X', curX);
            float y = gcodeParam(line, 'Y', curY);
            addWaypoint(x, y);
            curX = x; curY = y;
            Serial.printf("      -> (%.2f, %.2f)\n", x, y);
        }
        else if (strncmp(line, "M3", 2) == 0) Serial.println("      -> pen down");
        else if (strncmp(line, "M5", 2) == 0) Serial.println("      -> pen up");
        else if (strncmp(line, "M0", 2) == 0) Serial.println("      -> stop (ignored)");
    }

    Serial.printf("=== %d waypoints parsed ===\n", NUM_WAYPOINTS);
    for (int i = 0; i < NUM_WAYPOINTS; i++)
        Serial.printf("  [%d] (%.2f, %.2f)\n", i, WAYPOINTS[i][0], WAYPOINTS[i][1]);
}

// ============================================================
// ESTOP
// ============================================================

void checkSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == ' ') {
            emergencyStop = !emergencyStop;
            while (Serial.available()) Serial.read();
            Serial.println(emergencyStop ? "ESTOP" : "Resumed");
        }
    }
}

void waitForEstopClear() {
    motor1.stop(); motor2.stop();
    for (int i = 0; i < 500; i++) { motor1.run(); motor2.run(); }
    motor1.disableOutputs(); motor2.disableOutputs();
    Serial.println("!!! STOPPED — spacebar to resume");
    while (emergencyStop) { checkSerial(); vTaskDelay(10 / portTICK_PERIOD_MS); }
    motor1.enableOutputs(); motor2.enableOutputs();
    Serial.println("Resuming.");
}

// ============================================================
// KINEMATICS
// ============================================================

void fk(float t1_deg, float t2_deg, float &x, float &y) {
    float t1 = t1_deg * DEG_TO_RAD;
    float t2 = t2_deg * DEG_TO_RAD;
    x = LINK1_MM * cos(t1) + LINK2_MM * cos(t1 + t2);
    y = LINK1_MM * sin(t1) + LINK2_MM * sin(t1 + t2);
}

bool ik(float x, float y, float &t1_deg, float &t2_deg) {
    float d = sqrtf(x * x + y * y);
    if (d > LINK1_MM + LINK2_MM || d < fabsf(LINK1_MM - LINK2_MM)) {
        Serial.printf("  IK out of reach: (%.2f, %.2f) d=%.2f\n", x, y, d);
        return false;
    }
    float cosT2 = (d * d - LINK1_MM * LINK1_MM - LINK2_MM * LINK2_MM)
                  / (2.0f * LINK1_MM * LINK2_MM);
    cosT2 = constrain(cosT2, -1.0f, 1.0f);
    float t2a = acosf(cosT2);
    float t2b = -t2a;
    float t2 = (fabsf(t2a * RAD_TO_DEG - currentT2) <= fabsf(t2b * RAD_TO_DEG - currentT2))
               ? t2a : t2b;
    float t1 = atan2f(y, x) - atan2f(LINK2_MM * sinf(t2),
                                      LINK1_MM + LINK2_MM * cosf(t2));
    t1_deg = t1 * RAD_TO_DEG;
    t2_deg = t2 * RAD_TO_DEG;
    return true;
}

// ============================================================
// SPEED COMPUTATION
// ============================================================

JointSpeeds cartSpeedToJointSteps(float t1_deg, float t2_deg,
                                   float dx, float dy,
                                   float cart_speed_mm_s) {
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-6f) return {1.0f, 1.0f};

    float vx = (dx / len) * cart_speed_mm_s;
    float vy = (dy / len) * cart_speed_mm_s;

    float t1  = t1_deg * DEG_TO_RAD;
    float t2  = t2_deg * DEG_TO_RAD;
    float t12 = t1 + t2;

    float J11 = -LINK1_MM * sinf(t1) - LINK2_MM * sinf(t12);
    float J12 = -LINK2_MM * sinf(t12);
    float J21 =  LINK1_MM * cosf(t1) + LINK2_MM * cosf(t12);
    float J22 =  LINK2_MM * cosf(t12);

    float det = J11 * J22 - J12 * J21;

    if (fabsf(det) < 1.0f) return {20.0f, 20.0f};

    float w1_rad = ( J22 * vx - J12 * vy) / det;
    float w2_rad = (-J21 * vx + J11 * vy) / det;

    float s1 = fabsf(w1_rad * RAD_TO_DEG * STEPS_PER_DEG);
    float s2 = fabsf(w2_rad * RAD_TO_DEG * STEPS_PER_DEG);

    s1 = constrain(s1, 1.0f, 4000.0f);
    s2 = constrain(s2, 1.0f, 4000.0f);
    return {s1, s2};
}

// ============================================================
// MOTION
// ============================================================

bool setTargetXY(float x, float y) {
    float t1, t2;
    if (!ik(x, y, t1, t2)) return false;
    motor1.moveTo(lroundf(t1 * STEPS_PER_DEG));
    motor2.moveTo(lroundf(t2 * STEPS_PER_DEG));
    currentT1 = t1;
    currentT2 = t2;
    return true;
}

void drawLineTo(float x1, float y1, const char* label) {
    float x0 = currentX, y0 = currentY;
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);

    Serial.printf("\n-> line %s (%.2f,%.2f)->(%.2f,%.2f) len=%.1fmm\n",
                  label, x0, y0, x1, y1, len);

    if (len < 0.3f) {
        Serial.println("   (skip — already there)");
        setTargetXY(x1, y1);
        currentX = x1; currentY = y1;
        return;
    }

    int nSeg = max(1, (int)ceilf(len / LINE_STEP_MM));

    float mx = x0 + dx * 0.5f, my = y0 + dy * 0.5f;
    float mt1, mt2;
    if (!ik(mx, my, mt1, mt2)) { mt1 = currentT1; mt2 = currentT2; }
    JointSpeeds spd = cartSpeedToJointSteps(mt1, mt2, dx, dy, TARGET_CART_SPEED_MM_S);

    motor1.setMaxSpeed(spd.s1); motor1.setAcceleration(ACCEL);
    motor2.setMaxSpeed(spd.s2); motor2.setAcceleration(ACCEL);

    Serial.printf("   nSeg=%d  s1=%.0f s2=%.0f steps/s\n", nSeg, spd.s1, spd.s2);

    const long SEGMENT_TOLERANCE = 80;

    int seg = 1;
    while (seg <= nSeg) {
        checkSerial();
        if (emergencyStop) waitForEstopClear();

        float frac = (float)seg / (float)nSeg;
        setTargetXY(x0 + frac * dx, y0 + frac * dy);

        while (true) {
            checkSerial();
            if (emergencyStop) waitForEstopClear();
            motor1.run();
            motor2.run();

            bool close  = labs(motor1.distanceToGo()) <= SEGMENT_TOLERANCE
                       && labs(motor2.distanceToGo()) <= SEGMENT_TOLERANCE;
            bool atLast = (seg == nSeg);

            if (!atLast && close) break;
            if (atLast && motor1.distanceToGo() == 0
                       && motor2.distanceToGo() == 0) break;
        }
        seg++;
    }

    currentX = x1;
    currentY = y1;

    Serial.printf("   done -> (%.2f, %.2f)\n", currentX, currentY);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    motor1.setMaxSpeed(500.0f); motor1.setAcceleration(ACCEL);
    motor2.setMaxSpeed(500.0f); motor2.setAcceleration(ACCEL);

    parseGcode();

    Serial.printf("Home=%.1f | interp %.0fmm | speed %.0f mm/s | %d waypoints\n",
                  HOME_X, LINE_STEP_MM, TARGET_CART_SPEED_MM_S, NUM_WAYPOINTS);
    Serial.println("Arm FULLY EXTENDED. Spacebar = estop. Starting in 3s...");
    delay(3000);

    unsigned long startTime = millis();

    for (int i = 1; i < NUM_WAYPOINTS; i++) {
        char label[32];
        snprintf(label, sizeof(label), "pt %d/%d", i + 1, NUM_WAYPOINTS);
        drawLineTo(WAYPOINTS[i][0], WAYPOINTS[i][1], label);
    }

    Serial.printf("\n=== Done in %.1fs ===\n", (millis() - startTime) / 1000.0f);
    motor1.disableOutputs();
    motor2.disableOutputs();
    Serial.println("Motors off.");

    while (true) vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void loop() {}