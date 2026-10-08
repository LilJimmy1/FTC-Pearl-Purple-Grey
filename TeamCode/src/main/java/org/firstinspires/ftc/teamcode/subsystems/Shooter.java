package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Dual-motor shooter with distance-based RPM interpolation.
 * - shooter: HAS encoder (reads RPM)
 * - shooter2: follower (same power)
 *
 * Interpolation allows smooth, accurate RPM targeting based on Limelight distance.
 */
@Config
public class Shooter extends SubsystemBase {

    private final DcMotorEx shooter;   // HAS encoder
    private final DcMotorEx shooter2;  // follower
    private final VoltageSensor voltageSensor;

    public static final double TICKS_PER_REV = 28.0;  // REV HD Hex

    // ====== MANUAL OVERRIDES (Bumper Controls) ======
    public static double CLOSE_RPM = 3250.0;  // Left Bumper
    public static double FAR_RPM   = 4060.0;  // Right Bumper

    // ====== INTERPOLATION TABLE ======
    // Format: { distance (inches), RPM }
    // TUNE THESE VALUES BY TESTING AT EACH DISTANCE!
    // Add/remove rows as needed. Keep distances in ascending order.
    public static double[][] SHOOTER_TABLE = {
            { 24.0, 2800.0 },   // 2 feet - close shot
            { 36.0, 3100.0 },   // 3 feet
            { 48.0, 3400.0 },   // 4 feet
            { 60.0, 3650.0 },   // 5 feet
            { 72.0, 3850.0 },   // 6 feet
            { 84.0, 4000.0 },   // 7 feet
            { 96.0, 4100.0 },   // 8 feet - far shot
    };

    // ====== PID TUNING ======
    public static double kP = 0.0010;   // Proportional gain
    public static double kI = 0.0;    // Integral gain (optional, start at 0)
    public static double kD = 0.00015;    // Derivative gain (optional, start at 0)
    public static double kF = 0.00022; // Feedforward - base power boost

    // ====== VOLTAGE COMPENSATION ======
    public static double NOMINAL_VOLTAGE = 12.0;
    public static boolean USE_VOLTAGE_COMP = true;

    // ====== SPIN-UP TOLERANCE ======
    public static double DEFAULT_TOLERANCE_RPM = 100.0;

    // Internal state
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double motorPower = 0.0;
    private double rpmError = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastDistance = 0.0;  // Track last distance for telemetry

    public Shooter(HardwareMap hardwareMap) {
        shooter  = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Use RUN_WITHOUT_ENCODER mode but still read velocity
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Get voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        // Auto-updates - not needed for LinearOpMode
    }

    // ==================== INTERPOLATION ====================

    /**
     * Linear interpolation to find RPM for a given distance.
     * Uses the SHOOTER_TABLE lookup table.
     *
     * @param distance Distance to target in inches
     * @return Interpolated RPM value
     */
    public double interpolateRPM(double distance) {
        // Handle edge cases - clamp to table bounds
        if (distance <= SHOOTER_TABLE[0][0]) {
            return SHOOTER_TABLE[0][1];
        }
        if (distance >= SHOOTER_TABLE[SHOOTER_TABLE.length - 1][0]) {
            return SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1];
        }

        // Find the two points to interpolate between
        for (int i = 0; i < SHOOTER_TABLE.length - 1; i++) {
            double x1 = SHOOTER_TABLE[i][0];     // distance 1
            double y1 = SHOOTER_TABLE[i][1];     // RPM 1
            double x2 = SHOOTER_TABLE[i + 1][0]; // distance 2
            double y2 = SHOOTER_TABLE[i + 1][1]; // RPM 2

            if (distance >= x1 && distance <= x2) {
                // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                double t = (distance - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }

        // Fallback (shouldn't reach here)
        return SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1];
    }

    // ==================== DISTANCE-BASED CONTROL ====================

    /**
     * Spin shooter at interpolated RPM based on distance.
     * Call this every loop while auto-aiming.
     *
     * @param distance Distance to target in inches (from Limelight)
     */
    public void spinAtDistance(double distance) {
        lastDistance = distance;
        targetRPM = interpolateRPM(distance);
        runPID();
    }

    /**
     * Spin shooter at interpolated RPM, only if target is valid.
     * Returns false if no valid target (shooter won't spin).
     *
     * @param distance Distance to target in inches
     * @param hasTarget Whether Limelight has a valid target
     * @return true if spinning, false if stopped due to no target
     */
    public boolean spinAtDistanceIfValid(double distance, boolean hasTarget) {
        if (!hasTarget || distance <= 0) {
            stop();
            return false;
        }
        spinAtDistance(distance);
        return true;
    }

    // ==================== SIMPLE CONTROL (MANUAL OVERRIDES) ====================

    /**
     * Spin at CLOSE_RPM (Left Bumper)
     * Call this every loop while button is held
     */
    public void spinClose() {
        targetRPM = CLOSE_RPM;
        lastDistance = 0;
        runPID();
    }

    /**
     * Spin at FAR_RPM (Right Bumper)
     * Call this every loop while button is held
     */
    public void spinFar() {
        targetRPM = FAR_RPM;
        lastDistance = 0;
        runPID();
    }

    /**
     * Spin at a specific RPM (for testing or custom use)
     * Call this every loop while active
     */
    public void spinAtRPM(double rpm) {
        targetRPM = rpm;
        lastDistance = 0;
        runPID();
    }

    /**
     * Stop both motors
     */
    public void stop() {
        targetRPM = 0.0;
        motorPower = 0.0;
        rpmError = 0.0;
        integralSum = 0.0;
        lastError = 0.0;

        shooter.setPower(0.0);
        shooter2.setPower(0.0);
    }

    // ==================== PID CONTROL ====================

    private void runPID() {
        if (targetRPM <= 0) {
            stop();
            return;
        }

        // Read current RPM from motor's built-in velocity
        double ticksPerSec = shooter.getVelocity();
        currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV;

        // Calculate error
        rpmError = targetRPM - currentRPM;

        // PID calculations
        double pOutput = kP * rpmError;

        // Integral (with anti-windup)
        if (Math.abs(rpmError) < 500) {  // Only accumulate when close
            integralSum += rpmError;
            integralSum = Range.clip(integralSum, -10000, 10000);  // Clamp
        }
        double iOutput = kI * integralSum;

        // Derivative
        double derivative = rpmError - lastError;
        double dOutput = kD * derivative;
        lastError = rpmError;

        // Feedforward
        double feedforward = kF * targetRPM;

        // Sum all components
        double power = pOutput + iOutput + dOutput + feedforward;

        // Voltage compensation
        if (USE_VOLTAGE_COMP) {
            double voltage = voltageSensor.getVoltage();
            if (voltage > 0) {
                power *= (NOMINAL_VOLTAGE / voltage);
            }
        }

        // Clamp and apply
        power = Range.clip(power, 0.0, 1.0);
        motorPower = power;

        shooter.setPower(power);
        shooter2.setPower(power);
    }

    // ==================== GETTERS FOR TELEMETRY ====================

    public double getCurrentRPM() {
        double ticksPerSec = shooter.getVelocity();
        currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV;
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPMError() {
        return rpmError;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public double getLastDistance() {
        return lastDistance;
    }

    public boolean atTarget(double toleranceRPM) {
        return Math.abs(rpmError) < toleranceRPM;
    }

    public boolean atTarget() {
        return atTarget(DEFAULT_TOLERANCE_RPM);
    }

    /**
     * Check if shooter is ready to fire (at target RPM)
     */
    public boolean isReadyToFire() {
        return targetRPM > 0 && atTarget(DEFAULT_TOLERANCE_RPM);
    }

    /**
     * Get interpolation table info as string (for telemetry)
     */
    public String getTableInfo() {
        return String.format("Table: %.0f-%.0f in → %.0f-%.0f RPM",
                SHOOTER_TABLE[0][0],
                SHOOTER_TABLE[SHOOTER_TABLE.length - 1][0],
                SHOOTER_TABLE[0][1],
                SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1]);
    }
}