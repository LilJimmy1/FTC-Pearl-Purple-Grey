package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {

    // ══════════════════════════════════════════════════════════════
    //                    TUNING PARAMETERS
    // ══════════════════════════════════════════════════════════════

    // Base P gain - used for small errors
    public static double KP = 0.024;

    // Boost gain for large errors (more aggressive when far off target)
    public static double KP_FAR = 0.04;           // Higher gain when error > FAR_THRESHOLD
    public static double FAR_THRESHOLD = 8.0;     // Degrees - use KP_FAR above this

    // Prediction - compensates for lag
    public static double PREDICT_TIME = 0.06;     // Increased from 0.04

    // Feedforward for robot rotation
    public static double YAW_FF = 0.005;

    // Deadband and minimum power
    public static double DEADBAND_DEG = 0.5;
    public static double MIN_POWER = 0.18;        // Increased from 0.15

    // Direction and limits
    public static double DIRECTION = -1.0;
    public static double MIN_ANGLE = -360.0;
    public static double MAX_ANGLE = 360.0;
    public static double TICKS_PER_DEG = 4.0;

    // On-target threshold
    public static double ON_TARGET_THRESHOLD = 2.5;  // Slightly more forgiving

    // Search behavior when target lost
    public static double SEARCH_POWER = 0.2;      // Power to use when searching
    public static double SEARCH_TIMEOUT = 0.5;    // Seconds before starting search

    // ══════════════════════════════════════════════════════════════

    private final DcMotorEx motor;
    private final Limelight limelight;
    private final Telemetry telemetry;

    private double lastError = 999.0;
    private double actualError = 999.0;
    private double lastPower = 0;
    private double errorRate = 0;
    private double yawRate = 0;
    private double lastYaw = 0;
    private boolean hasValidTarget = false;
    private double lastKnownDirection = 0;  // Remember which way target was
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime targetLostTimer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Limelight limelight, Object poseProvider, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotorEx.class, "turret");
        this.limelight = limelight;
        this.telemetry = telemetry;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.reset();
        targetLostTimer.reset();
    }

    /**
     * CALL THIS EVERY LOOP - Main auto-aim function
     */
    public void autoAim() {
        double currentAngle = getTurretAngleDeg();

        // Soft limit protection
        if (currentAngle < MIN_ANGLE || currentAngle > MAX_ANGLE) {
            double homeError = -currentAngle;
            motor.setPower(Range.clip(KP * homeError, -0.3, 0.3));
            hasValidTarget = false;
            actualError = 999.0;
            lastError = 999.0;
            return;
        }

        if (!limelight.hasTarget()) {
            hasValidTarget = false;
            actualError = 999.0;
            lastError = 999.0;

            // Search behavior - continue moving in last known direction
            if (targetLostTimer.seconds() > SEARCH_TIMEOUT && lastKnownDirection != 0) {
                // Slowly search in the direction we last saw the target
                double searchPower = SEARCH_POWER * Math.signum(lastKnownDirection) * DIRECTION;
                motor.setPower(searchPower);
                lastPower = searchPower;
            } else {
                // Just lost target - hold position briefly
                motor.setPower(0);
                lastPower = 0;
            }
            return;
        }

        // We have a target!
        hasValidTarget = true;
        targetLostTimer.reset();  // Reset lost timer

        // Get timing
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // Get current error from Limelight
        double rawError = limelight.getFilteredTx();
        actualError = rawError;

        // Remember direction for search behavior
        if (Math.abs(rawError) > 1.0) {
            lastKnownDirection = rawError;
        }

        // Calculate error rate (how fast error is changing)
        if (Math.abs(lastError) < 500) {  // Only if we had a valid previous error
            errorRate = (rawError - lastError) / dt;
        } else {
            errorRate = 0;
        }

        // Predict where target WILL BE
        double predictedError = rawError + (errorRate * PREDICT_TIME);

        // Choose gain based on error magnitude (more aggressive when far off)
        double gain;
        if (Math.abs(predictedError) > FAR_THRESHOLD) {
            gain = KP_FAR;  // High gain for large errors
        } else {
            // Smooth transition between KP_FAR and KP
            double t = Math.abs(predictedError) / FAR_THRESHOLD;
            gain = KP + (KP_FAR - KP) * t;
        }

        // Apply deadband for motor control
        double controlError = predictedError;
        if (Math.abs(controlError) < DEADBAND_DEG) {
            controlError = 0;
        }

        // P control with variable gain
        double power = gain * controlError;

        // Add feedforward for robot rotation
        power += yawRate * YAW_FF;

        // Apply direction
        power *= DIRECTION;

        // Apply minimum power to overcome friction (only if we need to move)
        if (controlError != 0 && Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }

        power = Range.clip(power, -1.0, 1.0);
        motor.setPower(power);

        lastError = rawError;
        lastPower = power;
    }

    /**
     * Call every loop with IMU yaw (radians) for feedforward
     */
    public void updateRobotYaw(double yawRad) {
        double dt = timer.seconds();
        if (dt > 0 && dt < 0.5) {
            yawRate = Math.toDegrees(yawRad - lastYaw) / dt;
        }
        lastYaw = yawRad;
    }

    public void manualControl(double stick) {
        double angle = getTurretAngleDeg();
        if ((angle <= MIN_ANGLE && stick < 0) || (angle >= MAX_ANGLE && stick > 0)) {
            motor.setPower(0);
            return;
        }
        motor.setPower(Range.clip(stick, -1.0, 1.0));
    }

    public void homeStep() {
        double error = -getTurretAngleDeg();
        double power = Range.clip(KP * error, -0.5, 0.5);
        if (Math.abs(error) > 1.0 && Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }
        motor.setPower(power);
        lastError = error;
    }

    public void stop() {
        motor.setPower(0);
        lastPower = 0;
    }

    public double getTurretAngleDeg() {
        return motor.getCurrentPosition() / TICKS_PER_DEG;
    }

    /**
     * Returns true ONLY if:
     * 1. We have a valid target
     * 2. Actual error is below threshold
     */
    public boolean isOnTarget() {
        return hasValidTarget && Math.abs(actualError) < ON_TARGET_THRESHOLD;
    }

    public boolean hasTarget() {
        return hasValidTarget;
    }

    public double getActualError() {
        return actualError;
    }

    public int getEncoderTicks() {
        return motor.getCurrentPosition();
    }

    public double getLastError() {
        return lastError;
    }

    public double getErrorRate() {
        return errorRate;
    }

    public double getLastMotorPower() {
        return lastPower;
    }

    public double getYawRateDegPerSec() {
        return yawRate;
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}