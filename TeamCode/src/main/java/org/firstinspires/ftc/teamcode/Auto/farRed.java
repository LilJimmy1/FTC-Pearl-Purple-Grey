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
@Autonomous(name = "FarPositionRed", group = "RED")
public class farRed extends OpMode {

    // ==================== TUNABLE PARAMETERS ====================

    // --- LIMELIGHT ---
    public static int LIMELIGHT_PIPELINE = 1;

    // --- SPEEDS ---
    public static double SHOOTER_RPM = 4150.0;
    public static double INTAKE_SPEED = 1.0;
    public static double SHOOTER_REVERSE_RPM = -500.0;

    // --- PATH SPEEDS ---
    public static double SHOOTING_PATH_SPEED = 1.0;
    public static double INTAKE_PATH_SPEED = 0.6;

    // --- TIMING (seconds) ---
    public static double PAUSE_TIME = 0.5;
    public static double PAUSE_BEFORE_INTAKE = 0.3;
    public static double SHOOT_TIME_FIRST = 2.5;
    public static double SHOOT_TIME_OTHER = 2.0;
    public static double SPIN_UP_TIMEOUT = 2.0;

    // --- INTAKE PULSING ---
    public static double INTAKE_PULSE_ON = 0.3;
    public static double INTAKE_PULSE_OFF = 0.4;
    public static boolean USE_PULSED_INTAKE = true;

    // --- SHOOTER TOLERANCE ---
    public static double SHOOTER_RPM_TOLERANCE = 200.0;

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
    private Timer pulseTimer;

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private PathChain path1, path2, path3, path4, path5;

    private final Pose startPose = new Pose(80.298, 7.879, Math.toRadians(0));

    private enum State {
        PATH_1, SHOOT_1,
        PATH_2,
        PATH_3, SHOOT_2,
        PATH_4,
        PATH_5, SHOOT_3,
        DONE
    }

    private State currentState = State.PATH_1;
    private boolean waitingForPath = false;
    private boolean isPausing = false;
    private boolean isSpinningUp = false;
    private boolean isAligningTurret = false;
    private boolean isShooting = false;
    private boolean intakePulseOn = true;
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
        pulseTimer = new Timer();

        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, null, telemetry);

        limelight.setPipeline(LIMELIGHT_PIPELINE);
        currentPipeline = LIMELIGHT_PIPELINE;

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized - Far Red");
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

        telemetry.addData("=== FAR RED AUTO ===", "");
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
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(80.298, 7.879),
                        new Pose(83.818, 16.931)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.818, 16.931),
                        new Pose(89.853, 45.262),
                        new Pose(134.445, 33.024)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(134.445, 33.024),
                        new Pose(83.651, 16.931)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.651, 16.931),
                        new Pose(86.836, 74.095),
                        new Pose(133.104, 55.823)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(133.104, 55.823),
                        new Pose(83.483, 17.099)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void start() {
        limelight.setPipeline(LIMELIGHT_PIPELINE);

        currentState = State.PATH_1;
        waitingForPath = false;
        isPausing = false;
        isSpinningUp = false;
        isAligningTurret = false;
        isShooting = false;
        intakePulseOn = true;
        intakeRunning = false;
        shootStatus = "N/A";
        shotsCompleted = 0;
        timer.resetTimer();
        turretTimer.resetTimer();
        spinUpTimer.resetTimer();
        matchTimer.resetTimer();
        pulseTimer.resetTimer();

        intake.stop();
        shooter.stop();
    }

    @Override
    public void loop() {
        follower.update();
        limelight.periodic();
        turret.autoAim();

        switch (currentState) {

            case PATH_1:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(path1);
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
                shootSequence(SHOOT_TIME_FIRST, State.PATH_2);
                break;

            case PATH_2:
                if (isPausing) {
                    intake.stop();
                    intakeRunning = false;
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                        waitingForPath = false;  // CRITICAL: Reset so path starts
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(path2);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    intakeRunning = true;
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        currentState = State.PATH_3;
                        waitingForPath = false;
                    }
                }
                break;

            case PATH_3:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(path3);
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
                shootSequence(SHOOT_TIME_OTHER, State.PATH_4);
                break;

            case PATH_4:
                if (isPausing) {
                    intake.stop();
                    intakeRunning = false;
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_BEFORE_INTAKE) {
                        isPausing = false;
                        waitingForPath = false;  // CRITICAL: Reset so path starts
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(path4);
                        waitingForPath = true;
                    }

                    intake.run(INTAKE_SPEED);
                    intakeRunning = true;
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        currentState = State.PATH_5;
                        waitingForPath = false;
                    }
                }
                break;

            case PATH_5:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(path5);
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
                shootSequence(SHOOT_TIME_OTHER, State.DONE);
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
        telemetry.addData("=== FAR RED ===", "");
        telemetry.addData("Match Time", "%.1f s", matchTimer.getElapsedTimeSeconds());
        telemetry.addData("State", currentState);
        telemetry.addData("Phase", getPhaseString());
        telemetry.addData("Shots Completed", shotsCompleted + " / 3");

        telemetry.addData("--- LIMELIGHT ---", "");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("LL Has Target", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("LL tx", "%.2f", limelight.getFilteredTx());
        }

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
        telemetry.addData("On Target", turret.isOnTarget() ? "YES!" : "no");
        telemetry.addData("Power", "%.2f", turret.getLastMotorPower());

        if (isAligningTurret) {
            telemetry.addData("ALIGN TIMER", "%.1f / %.1f s",
                    turretTimer.getElapsedTimeSeconds(), TURRET_ALIGN_TIMEOUT);
        }

        telemetry.addData("--- INTAKE ---", "");
        telemetry.addData("Running", intakeRunning ? "YES" : "no");
        if (isShooting && USE_PULSED_INTAKE) {
            telemetry.addData("Pulse", intakePulseOn ? "FEEDING" : "RPM RECOVERY");
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
        // Phase 1: Pause (keep shooter spinning)
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

        // Phase 2: Spin-up (with timeout)
        if (isSpinningUp) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();
            intakeRunning = false;

            boolean atTarget = shooter.atTarget(SHOOTER_RPM_TOLERANCE);
            boolean timeout = spinUpTimer.getElapsedTimeSeconds() >= SPIN_UP_TIMEOUT;

            if (atTarget) {
                isSpinningUp = false;
                shootStatus = "RPM OK";

                // ALWAYS go to alignment phase
                if (WAIT_FOR_TURRET_ALIGN) {
                    isAligningTurret = true;
                    turretTimer.resetTimer();
                } else {
                    isShooting = true;
                    intakePulseOn = true;
                    pulseTimer.resetTimer();
                    timer.resetTimer();
                }
            } else if (timeout) {
                isSpinningUp = false;
                shootStatus = "RPM TIMEOUT";

                // Still go to alignment phase
                if (WAIT_FOR_TURRET_ALIGN) {
                    isAligningTurret = true;
                    turretTimer.resetTimer();
                } else {
                    isShooting = true;
                    intakePulseOn = true;
                    pulseTimer.resetTimer();
                    timer.resetTimer();
                }
            }
            return;
        }

        // Phase 3: Align turret (CRITICAL)
        if (isAligningTurret) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();
            intakeRunning = false;

            boolean turretReady = turret.isOnTarget();
            boolean timeout = turretTimer.getElapsedTimeSeconds() >= TURRET_ALIGN_TIMEOUT;

            if (turretReady) {
                isAligningTurret = false;
                isShooting = true;
                intakePulseOn = true;
                pulseTimer.resetTimer();
                timer.resetTimer();
                shootStatus = "ALIGNED";
            } else if (timeout) {
                isAligningTurret = false;
                isShooting = true;
                intakePulseOn = true;
                pulseTimer.resetTimer();
                timer.resetTimer();
                shootStatus = "ALIGN TIMEOUT";
            }
            return;
        }

        // Phase 4: Shoot with pulsed intake
        if (isShooting) {
            shooter.spinAtRPM(SHOOTER_RPM);

            if (USE_PULSED_INTAKE) {
                double pulseTime = pulseTimer.getElapsedTimeSeconds();
                double cycleTime = INTAKE_PULSE_ON + INTAKE_PULSE_OFF;
                double cyclePosition = pulseTime % cycleTime;

                if (cyclePosition < INTAKE_PULSE_ON) {
                    intake.run(INTAKE_SPEED);
                    intakePulseOn = true;
                    intakeRunning = true;
                } else {
                    intake.stop();
                    intakePulseOn = false;
                    intakeRunning = false;
                }
            } else {
                intake.run(INTAKE_SPEED);
                intakeRunning = true;
            }

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