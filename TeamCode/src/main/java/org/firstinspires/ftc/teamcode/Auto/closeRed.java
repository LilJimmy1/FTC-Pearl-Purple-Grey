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
@Autonomous(name = "ClosePositionRed", group = "RED")
public class closeRed extends OpMode {

    // ==================== TUNABLE PARAMETERS ====================

    // --- LIMELIGHT ---
    public static int LIMELIGHT_PIPELINE = 1;

    // --- SPEEDS ---
    public static double SHOOTER_RPM = 3300.0;
    public static double INTAKE_SPEED = 1.0;
    public static double SHOOTER_REVERSE_RPM = -500.0;

    // --- PATH SPEEDS ---
    public static double SHOOTING_PATH_SPEED = 0.8;
    public static double INTAKE_PATH_SPEED = 0.8;   // Increased from 0.6
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
    public static double TURRET_ALIGN_TIMEOUT = 2.5;

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

    private final Pose startPose = new Pose(117.681, 122.878, Math.toRadians(0));

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
    private boolean intakeRunning = false;
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

        telemetry.addData("Status", "Initialized - Close Red");
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

        telemetry.addData("=== CLOSE RED AUTO ===", "");
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
        // Path 1 - Start to first shooting position (curve)
        goShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(117.681, 122.878),
                        new Pose(101.923, 104.605),
                        new Pose(83.986, 82.980)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2 - First intake sweep (curve) - UPDATED
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.986, 82.980),
                        new Pose(86.836, 72.084),
                        new Pose(129.248, 81.471)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3 - Return to shooting position (curve, PRE-SPIN) - UPDATED
        goShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(129.248, 81.471),
                        new Pose(104.605, 88.009),
                        new Pose(83.986, 83.148)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4 - Second intake sweep (curve) - UPDATED
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.986, 83.148),
                        new Pose(62.193, 61.858),
                        new Pose(130.254, 57.332)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 5 - Return to final shooting position (curve, PRE-SPIN) - UPDATED
        goShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130.254, 57.332),
                        new Pose(80.969, 66.217),
                        new Pose(84.154, 83.148)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 6 - Park
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(84.154, 83.148),
                        new Pose(113.825, 56.326)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
        intakeRunning = false;
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
                    intakeRunning = false;
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
                    intakeRunning = false;
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                        waitingForPath = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(intake1);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    intakeRunning = true;
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
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

                intake.stop();
                intakeRunning = false;
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
                    intakeRunning = false;
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                        waitingForPath = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(intake2);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    intakeRunning = true;
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
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

                intake.stop();
                intakeRunning = false;
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
                    intakeRunning = false;
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                        waitingForPath = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(PARK_PATH_SPEED);
                        follower.followPath(park);
                        waitingForPath = true;
                    }

                    intake.stop();
                    intakeRunning = false;
                    shooter.stop();

                    if (!follower.isBusy()) {
                        currentState = State.DONE;
                        waitingForPath = false;
                    }
                }
                break;

            case DONE:
                intake.stop();
                intakeRunning = false;
                shooter.stop();
                turret.stop();
                break;
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("=== CLOSE RED ===", "");
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

        telemetry.addData("--- INTAKE ---", "");
        telemetry.addData("Running", intakeRunning ? "YES" : "no");

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
        if (intakeRunning) return "INTAKING";
        if (currentState == State.DONE) return "COMPLETE";
        return "RUNNING";
    }

    private void shootSequence(double shootTime, State nextState) {
        if (isPausing) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();
            intakeRunning = false;

            if (timer.getElapsedTimeSeconds() >= PAUSE_TIME) {
                isPausing = false;
                isSpinningUp = true;
                spinUpTimer.resetTimer();
            }
            return;
        }

        if (isSpinningUp) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();
            intakeRunning = false;

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

        if (isAligningTurret) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();
            intakeRunning = false;

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

        if (isShooting) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.run(INTAKE_SPEED);
            intakeRunning = true;

            if (timer.getElapsedTimeSeconds() >= shootTime) {
                shotsCompleted++;
                currentState = nextState;
                waitingForPath = false;
                isPausing = true;
                isSpinningUp = false;
                isAligningTurret = false;
                isShooting = false;
                intakeRunning = false;
                timer.resetTimer();
            }
        }
    }
}