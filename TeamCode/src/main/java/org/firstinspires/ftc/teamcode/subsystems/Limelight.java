package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Limelight extends SubsystemBase {

    // Tuning
    public static double FILTER_ALPHA = 0.4;  // 0.2=smooth, 0.6=responsive

    // Distance calculation - MEASURE THESE ON THE ROBOT!
    public static double CAMERA_ANGLE_DEG = 15.0;    // Camera tilt up from horizontal
    public static double CAMERA_HEIGHT_IN = 8.0;     // Camera height from floor
    public static double TARGET_HEIGHT_IN = 14.0;    // Target height from floor

    // Hardware
    private final Limelight3A limelight;

    // State
    private double rawTx, rawTy, rawTa;
    private double filteredTx, filteredTy;
    private boolean hasTarget;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // Constructor with telemetry (for compatibility)
    public Limelight(HardwareMap hardwareMap, Object telemetry) {
        this(hardwareMap);
    }

    /**
     * Call every loop to update readings
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;
            rawTx = result.getTx();
            rawTy = result.getTy();
            rawTa = result.getTa();

            // Simple exponential filter
            filteredTx = FILTER_ALPHA * rawTx + (1 - FILTER_ALPHA) * filteredTx;
            filteredTy = FILTER_ALPHA * rawTy + (1 - FILTER_ALPHA) * filteredTy;
        } else {
            hasTarget = false;
        }
    }

    // Backwards compatibility
    public void periodic() {
        update();
    }

    // Getters

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTx() {
        return rawTx;
    }

    public double getTy() {
        return rawTy;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getFilteredTy() {
        return filteredTy;
    }

    public double getTargetArea() {
        return rawTa;
    }

    /**
     * Get distance to target in inches using ty angle
     */
    public double getDistanceInches() {
        if (!hasTarget) return -1;

        double angleRad = Math.toRadians(CAMERA_ANGLE_DEG + filteredTy);
        if (Math.abs(angleRad) < 0.01) return -1;

        return (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleRad);
    }

    // Alias for compatibility
    public double getFilteredDistance() {
        return getDistanceInches();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }
}