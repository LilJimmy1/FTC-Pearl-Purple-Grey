package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
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
@Autonomous(name = "FarPositionBlue", group = "BLUE")
public class farBlue extends OpMode {

    // ==================== TUNABLE PARAMETERS ====================

    // --- LIMELIGHT ---
    public static int LIMELIGHT_PIPELINE = 0;

    // --- SPEEDS ---
    public static double SHOOTER_RPM = 4060.0;
    public static double INTAKE_SPEED = 1.0;
    public static double SHOOTER_REVERSE_RPM = -500.0;

    // --- PATH SPEEDS ---
    public static double SHOOTING_PATH_SPEED = 1.0;
    public static double INTAKE_PATH_SPEED = 0.85;

    // --- TIMING (seconds) ---
    public static double PAUSE_TIME = 1.0;
    public static double SHOOT_TIME_FIRST = 3.5;
    public static double SHOOT_TIME_OTHER = 4.0;

    // --- SHOOTER TOLERANCE ---
    public static double SHOOTER_RPM_TOLERANCE = 100.0;

    // --- TURRET ---
    public static boolean WAIT_FOR_TURRET_ALIGN = true;
    public static double TURRET_ALIGN_TIMEOUT = 3.5;

    // ==================== END TUNABLE PARAMETERS ====================

    private Follower follower;
    private Timer timer;
    private Timer turretTimer;

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private PathChain path1, path2, path3, path4, path5, path6, path7;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(-180));

    private enum State {
        PATH_1, SHOOT_1,
        PATH_2, PATH_3,
        PATH_4, SHOOT_2,
        PATH_5, PATH_6,
        PATH_7, SHOOT_3,
        DONE
    }

    private State currentState = State.PATH_1;
    private boolean waitingForPath = false;
    private boolean isPausing = false;
    private boolean isSpinningUp = false;
    private boolean isAligningTurret = false;
    private int currentPipeline = -1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        timer = new Timer();
        turretTimer = new Timer();

        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, null, telemetry);

        limelight.setPipeline(LIMELIGHT_PIPELINE);
        currentPipeline = LIMELIGHT_PIPELINE;

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Initialized - Far Blue");
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

        telemetry.addData("Status", "Ready - Far Blue");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Has Target", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx", "%.2f", limelight.getFilteredTx());
        }
        telemetry.update();
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(63.031, 14.584)))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(63.031, 14.584), new Pose(41.239, 35.204)))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.239, 35.204), new Pose(9.388, 35.371)))
                .setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.388, 35.371), new Pose(62.864, 14.584)))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(62.864, 14.584), new Pose(14.920, 9.220)))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14.920, 9.220), new Pose(8.382, 9.052)))
                .setTangentHeadingInterpolation()
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8.382, 9.052), new Pose(63.199, 14.584)))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
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
        timer.resetTimer();
        turretTimer.resetTimer();

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
                    shooter.stop();
                }

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_1;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_1:
                shootSequence(SHOOT_TIME_FIRST, State.PATH_2);
                break;

            case PATH_2:
                if (isPausing) {
                    intake.stop();
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_TIME) {
                        isPausing = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(path2);
                        waitingForPath = true;
                    }
                    intake.run(INTAKE_SPEED);
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        currentState = State.PATH_3;
                        waitingForPath = false;
                    }
                }
                break;

            case PATH_3:
                if (!waitingForPath) {
                    follower.setMaxPower(INTAKE_PATH_SPEED);
                    follower.followPath(path3);
                    waitingForPath = true;
                }
                intake.run(INTAKE_SPEED);
                shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                if (!follower.isBusy()) {
                    currentState = State.PATH_4;
                    waitingForPath = false;
                }
                break;

            case PATH_4:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(path4);
                    waitingForPath = true;
                }
                intake.stop();
                shooter.spinAtRPM(SHOOTER_RPM);

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_2;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_2:
                shootSequence(SHOOT_TIME_OTHER, State.PATH_5);
                break;

            case PATH_5:
                if (isPausing) {
                    intake.stop();
                    shooter.stop();
                    if (timer.getElapsedTimeSeconds() >= PAUSE_TIME) {
                        isPausing = false;
                    }
                } else {
                    if (!waitingForPath) {
                        follower.setMaxPower(INTAKE_PATH_SPEED);
                        follower.followPath(path5);
                        waitingForPath = true;
                    }
                    intake.run(INTAKE_SPEED);
                    shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                    if (!follower.isBusy()) {
                        currentState = State.PATH_6;
                        waitingForPath = false;
                    }
                }
                break;

            case PATH_6:
                if (!waitingForPath) {
                    follower.setMaxPower(INTAKE_PATH_SPEED);
                    follower.followPath(path6);
                    waitingForPath = true;
                }
                intake.run(INTAKE_SPEED);
                shooter.spinAtRPM(SHOOTER_REVERSE_RPM);

                if (!follower.isBusy()) {
                    currentState = State.PATH_7;
                    waitingForPath = false;
                }
                break;

            case PATH_7:
                if (!waitingForPath) {
                    follower.setMaxPower(SHOOTING_PATH_SPEED);
                    follower.followPath(path7);
                    waitingForPath = true;
                }
                intake.stop();
                shooter.spinAtRPM(SHOOTER_RPM);

                if (!follower.isBusy()) {
                    currentState = State.SHOOT_3;
                    waitingForPath = false;
                    isPausing = true;
                    isSpinningUp = false;
                    isAligningTurret = false;
                    timer.resetTimer();
                }
                break;

            case SHOOT_3:
                shootSequence(SHOOT_TIME_OTHER, State.DONE);
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
        telemetry.addData("State", currentState);
        telemetry.addData("Phase", getPhaseString());
        telemetry.addData("Timer", "%.2f", timer.getElapsedTimeSeconds());

        telemetry.addData("--- LIMELIGHT ---", "");
        telemetry.addData("Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Has Target", limelight.hasTarget());

        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target RPM", SHOOTER_RPM);
        telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Ready", shooter.atTarget(SHOOTER_RPM_TOLERANCE));

        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Error", "%.2f deg", turret.getLastError());
        telemetry.addData("On Target", turret.isOnTarget());

        telemetry.update();
    }

    private String getPhaseString() {
        if (isPausing) return "PAUSING";
        if (isSpinningUp) return "SPIN-UP";
        if (isAligningTurret) return "ALIGNING";
        if (currentState == State.DONE) return "COMPLETE";
        return "RUNNING";
    }

    private void shootSequence(double shootTime, State nextState) {
        if (isPausing) {
            if (shooter.getCurrentRPM() > SHOOTER_RPM * 0.5) {
                shooter.spinAtRPM(SHOOTER_RPM);
            } else {
                shooter.stop();
            }
            intake.stop();

            if (timer.getElapsedTimeSeconds() >= PAUSE_TIME) {
                isPausing = false;
                isSpinningUp = true;
            }
            return;
        }

        if (isSpinningUp) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();

            if (shooter.atTarget(SHOOTER_RPM_TOLERANCE)) {
                isSpinningUp = false;
                isAligningTurret = WAIT_FOR_TURRET_ALIGN;
                turretTimer.resetTimer();

                if (!WAIT_FOR_TURRET_ALIGN) {
                    timer.resetTimer();
                }
            }
            return;
        }

        if (isAligningTurret) {
            shooter.spinAtRPM(SHOOTER_RPM);
            intake.stop();

            boolean turretReady = turret.isOnTarget();
            boolean timeout = turretTimer.getElapsedTimeSeconds() >= TURRET_ALIGN_TIMEOUT;

            if (turretReady || timeout) {
                isAligningTurret = false;
                timer.resetTimer();
            }
            return;
        }

        shooter.spinAtRPM(SHOOTER_RPM);
        intake.run(INTAKE_SPEED);

        if (timer.getElapsedTimeSeconds() >= shootTime) {
            currentState = nextState;
            waitingForPath = false;
            isPausing = true;
            isSpinningUp = false;
            isAligningTurret = false;
            timer.resetTimer();
        }
    }
}