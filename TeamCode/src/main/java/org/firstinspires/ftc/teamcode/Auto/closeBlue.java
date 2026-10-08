package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Config
@Autonomous(name = "ClosePositionBlue", group = "BLUE")
public class closeBlue extends OpMode {

    // ==================== TUNABLE PARAMETERS ====================

    // --- LIMELIGHT ---
    public static int LIMELIGHT_PIPELINE = 0;

    // --- SPEEDS ---
    public static double SHOOTER_RPM = 3300.0;  // Slightly increased for margin
    public static double INTAKE_SPEED = 1.0;
    public static double SHOOTER_REVERSE_RPM = -500.0;

    // --- PATH SPEEDS ---
    public static double SHOOTING_PATH_SPEED = 0.8;   // Slower for better tracking
    public static double INTAKE_PATH_SPEED = 0.6;
    public static double PARK_PATH_SPEED = 0.8;

    // --- TIMING (seconds) ---
    public static double PAUSE_TIME = 0.5;
    public static double PAUSE_BEFORE_INTAKE = 0.3;
    public static double SHOOT_TIME_FIRST = 2.5;
    public static double SHOOT_TIME_OTHER = 2.0;
    public static double SPIN_UP_TIMEOUT = 2.0;

    // --- SHOOTER TOLERANCE ---
    public static double SHOOTER_RPM_TOLERANCE = 150.0;

    // --- TURRET ---
    public static boolean WAIT_FOR_TURRET_ALIGN = true;
    public static double TURRET_ALIGN_TIMEOUT = 2.5;  // Increased for far shots

    // --- PRE-SPIN ---
    public static boolean PRE_SPIN_ON_RETURN = true;

    // ==================== END TUNABLE PARAMETERS ====================

    private Follower follower;
    private Timer timer;
    private Timer turretTimer;
    private Timer spinUpTimer;
    private Timer matchTimer;

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private PathChain goShoot1, intake1, goShoot2, intake2, goShoot3, park;

    private final Pose startPose = new Pose(25.481, 122.542, Math.toRadians(-180));

    private enum State {
        GO_SHOOT_1, SHOOT_1,
        INTAKE_1,
        GO_SHOOT_2, SHOOT_2,
        INTAKE_2,
        GO_SHOOT_3, SHOOT_3,
        PARK,
        DONE
    }

    private State currentState = State.GO_SHOOT_1;
    private boolean waitingForPath = false;
    private boolean isPausing = false;
    private boolean isSpinningUp = false;
    private boolean isAligningTurret = false;
    private boolean isShooting = false;
    private int currentPipeline = -1;

    private String shootStatus = "N/A";
    private int shotsCompleted = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        timer = new Timer();
        turretTimer = new Timer();
        spinUpTimer = new Timer();
        matchTimer = new Timer();

        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, null, telemetry);

        limelight.setPipeline(LIMELIGHT_PIPELINE);
        currentPipeline = LIMELIGHT_PIPELINE;

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized - Close Blue");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (LIMELIGHT_PIPELINE != currentPipeline) {
            limelight.setPipeline(LIMELIGHT_PIPELINE);
            currentPipeline = LIMELIGHT_PIPELINE;
        }

        limelight.periodic();

        telemetry.addData("=== CLOSE BLUE AUTO ===", "");
        telemetry.addData("Status", "Ready");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Has Target", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx", "%.2f", limelight.getFilteredTx());
        }
        telemetry.addData("", "");
        telemetry.addData("CHECKLIST:", "");
        telemetry.addData("  1. Robot at start position?", "");
        telemetry.addData("  2. Limelight sees target?", limelight.hasTarget() ? "YES" : "NO!");
        telemetry.addData("  3. Balls loaded?", "");
        telemetry.update();
    }

    private void buildPaths() {
        goShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(25.481, 122.542),
                        new Pose(39.059, 96.224),
                        new Pose(62.193, 80.298)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62.193, 80.298),
                        new Pose(47.106, 77.113),
                        new Pose(13.914, 85.327)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        goShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.914, 85.327),
                        new Pose(62.193, 80.633)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62.193, 80.633),
                        new Pose(63.870, 69.905),
                        new Pose(41.742, 56.494),
                        new Pose(11.902, 57.667)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        goShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(11.902, 57.667),
                        new Pose(28.331, 61.858),
                        new Pose(50.962, 69.234),
                        new Pose(62.193, 80.801)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(62.193, 80.801),
                        new Pose(21.793, 71.749)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();
    }

    @Override
    public void start() {
        limelight.setPipeline(LIMELIGHT_PIPELINE);

        currentState = State.GO_SHOOT_1;
        waitingForPath = false;
        isPausing = false;
        isSpinningUp = false;
        isAligningTurret = false;
        isShooting = false;
        shootStatus = "N/A";
        shotsCompleted = 0;
        timer.resetTimer();
        turretTimer.resetTimer();
        spinUpTimer.resetTimer();
        matchTimer.resetTimer();

        intake.stop();
        shooter.stop();
    }

    @Override
    public void loop() {
        // CRITICAL: Always update every loop
        follower.update();
        limelight.periodic();
        turret.autoAim();

        switch (currentState) {

            case GO_SHOOT_1:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(goShoot1);
                    waitingForPath = true;
                    intake.stop();
                    // Start spinning shooter early
                    shooter.spinAtRPM(SHOOTER_RPM);
                }

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_1;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    isShooting = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_1:
                shootSequence(SHOOT_TIME_FIRST, State.INTAKE_1);
                break;

            case INTAKE_1:
                if (isPausing) {
                    intake.stop();
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(intake1);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        intake.stop();
                        currentState = State.GO_SHOOT_2;
                        waitingForPath = false;
                    }
                }
                break;

            case GO_SHOOT_2:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(goShoot2);
                    waitingForPath = true;
                }

                // PRE-SPIN shooter while returning
                intake.stop();
                if (PRE_SPIN_ON_RETURN) {
                    shooter.spinAtRPM(SHOOTER_RPM);
                } else {
                    shooter.stop();
                }

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_2;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    isShooting = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_2:
                shootSequence(SHOOT_TIME_OTHER, State.INTAKE_2);
                break;

            case INTAKE_2:
                if (isPausing) {
                    intake.stop();
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(intake2);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        intake.stop();
                        currentState = State.GO_SHOOT_3;
                        waitingForPath = false;
                    }
                }
                break;

            case GO_SHOOT_3:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(goShoot3);
                    waitingForPath = true;
                }

                // PRE-SPIN shooter while returning
                intake.stop();
                if (PRE_SPIN_ON_RETURN) {
                    shooter.spinAtRPM(SHOOTER_RPM);
                } else {
                    shooter.stop();
                }

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_3;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    isShooting = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_3:
                shootSequence(SHOOT_TIME_OTHER, State.PARK);
                break;

            case PARK:
                if (isPausing) {
                    intake.stop();
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(PARK_PATH_SPEED);
                        follower.followPath(park);
                        waitingForPath = true;
                    }

                    intake.stop();
                    shooter.stop();

                    if (!follower.isBusy()) {
                        currentState = State.DONE;
                        waitingForPath = false;
                    }
                }
                break;

            case DONE:
                intake.stop();
                shooter.stop();
                turret.stop();
                break;
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("=== CLOSE BLUE ===", "");
        telemetry.addData("Match Time", "%.1f s", matchTimer.getElapsedTimeSeconds());
        telemetry.addData("State", currentState);
        telemetry.addData("Phase", getPhaseString());
        telemetry.addData("Shots Completed", shotsCompleted + " / 3");

        telemetry.addData("--- LIMELIGHT ---", "");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Has Target", limelight.hasTarget());

        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target", "%.0f RPM", SHOOTER_RPM);
        telemetry.addData("Current", "%.0f RPM", shooter.getCurrentRPM());
        telemetry.addData("At Target", shooter.atTarget(SHOOTER_RPM_TOLERANCE));

        if (isSpinningUp) {
            telemetry.addData("Spin-Up", "%.1f / %.1f s",
                    spinUpTimer.getElapsedTimeSeconds(), SPIN_UP_TIMEOUT);
        }

        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Has Target", turret.hasTarget());
        telemetry.addData("Error", "%.1f deg", turret.getActualError());
        telemetry.addData("On Target", turret.isOnTarget());
        telemetry.addData("Power", "%.2f", turret.getLastMotorPower());

        if (isAligningTurret) {
            telemetry.addData("Align", "%.1f / %.1f s",
                    turretTimer.getElapsedTimeSeconds(), TURRET_ALIGN_TIMEOUT);
        }

        telemetry.addData("--- STATUS ---", "");
        telemetry.addData("Result", shootStatus);

        telemetry.update();
    }

    private String getPhaseString() {
        if (isPausing) return "PAUSING";
        if (isSpinningUp) return "SPIN-UP";
        if (isAligningTurret) return "ALIGNING";
        if (isShooting) return "SHOOTING";
        if (currentState == State.DONE) return "COMPLETE";
        return "RUNNING";
    }

    private void shootSequence(double shootTime, State nextState) {
        // Phase 1: Pause
        if (isPausing) {
            // Always keep shooter spinning during pause
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();

            if (timer.getElapsedTimeSeconds() >= PAUSE_TIME) {
                isPausing = false;
                isSpinningUp = true;
                spinUpTimer.resetTimer();
            }
            return;
        }

        // Phase 2: Spin-up (with timeout)
        if (isSpinningUp) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();

            boolean atTarget = shooter.atTarget(SHOOTER_RPM_TOLERANCE);
            boolean timeout = spinUpTimer.getElapsedTimeSeconds() >= SPIN_UP_TIMEOUT;

            if (atTarget) {
                isSpinningUp = false;
                shootStatus = "RPM OK";

                if (WAIT_FOR_TURRET_ALIGN) {
                    isAligningTurret = true;
                    turretTimer.resetTimer();
                } else {
                    isShooting = true;
                    timer.resetTimer();
                }
            } else if (timeout) {
                isSpinningUp = false;
                shootStatus = "RPM TIMEOUT";

                // Still try to align turret
                if (WAIT_FOR_TURRET_ALIGN) {
                    isAligningTurret = true;
                    turretTimer.resetTimer();
                } else {
                    isShooting = true;
                    timer.resetTimer();
                }
            }
            return;
        }

        // Phase 3: Align turret
        if (isAligningTurret) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();

            boolean turretReady = turret.isOnTarget();
            boolean timeout = turretTimer.getElapsedTimeSeconds() >= TURRET_ALIGN_TIMEOUT;

            if (turretReady) {
                isAligningTurret = false;
                isShooting = true;
                timer.resetTimer();
                shootStatus = "ALIGNED";
            } else if (timeout) {
                isAligningTurret = false;
                isShooting = true;
                timer.resetTimer();
                shootStatus = "ALIGN TIMEOUT";
            }
            return;
        }

        // Phase 4: Shoot
        if (isShooting) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.run(INTAKE_SPEED);

            if (timer.getElapsedTimeSeconds() >= shootTime) {
                shotsCompleted++;
                currentState = nextState;
                waitingForPath = false;
                isPausing = true;
                isSpinningUp = false;
                isAligningTurret = false;
                isShooting = false;
                timer.resetTimer();
            }
        }
    }
}