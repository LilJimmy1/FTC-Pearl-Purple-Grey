package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name = "RED ALLIANCE TeleOp", group = "Linear Opmode")
public class TeleOP extends LinearOpMode {

    private static final double SLOW_DRIVE_SPEED = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive motors
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor br = hardwareMap.dcMotor.get("br");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        // Subsystems
        Limelight limelight = new Limelight(hardwareMap);
        limelight.setPipeline(1);  // RED = pipeline 1

        Shooter shooter = new Shooter(hardwareMap);
        Turret turret = new Turret(hardwareMap, limelight, null, telem);
        Intake intake = new Intake(hardwareMap);

        CommandScheduler.getInstance().schedule(new IntakeCommand(intake,
                () -> gamepad2.right_trigger - gamepad2.left_trigger));

        // Track shooter ready state for rumble feedback
        boolean wasShooterReady = false;

        telem.addLine("Ready - RED ALLIANCE");
        telem.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // === UPDATE SENSORS ===
            limelight.update();
            turret.updateRobotYaw(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            //turret.autoAim();

            // === SHOOTER (Gamepad 2) ===
            if (gamepad2.left_bumper) {
                shooter.spinClose();
            } else if (gamepad2.right_bumper) {
                shooter.spinFar();
            } else if (gamepad2.y && limelight.hasTarget()) {
                shooter.spinAtDistance(limelight.getDistanceInches());
            } else {
                shooter.stop();
            }

            // === SHOOTER RUMBLE FEEDBACK ===
            boolean isShooterReady = shooter.isReadyToFire();
            if (isShooterReady && !wasShooterReady) {
                // Rumble both motors at full intensity for 250ms when shooter reaches target RPM
                gamepad2.rumble(1.0, 1.0, 250);
            }
            wasShooterReady = isShooterReady;

            // === TURRET (Gamepad 2) ===
            if (gamepad2.a) {
                turret.homeStep();
            } else if (gamepad2.dpad_left) {
                turret.manualControl(-0.6);
            } else if (gamepad2.dpad_right) {
                turret.manualControl(0.6);
            } else {
                turret.autoAim();
            }

            // === DRIVE (Gamepad 1) ===
            double y, x, rx;

            // Dpad slow drive (takes priority over joysticks)
            if (gamepad1.dpad_up) {
                y = SLOW_DRIVE_SPEED;
                x = 0;
                rx = 0;
            } else if (gamepad1.dpad_down) {
                y = -SLOW_DRIVE_SPEED;
                x = 0;
                rx = 0;
            } else if (gamepad1.dpad_left) {
                y = 0;
                x = -SLOW_DRIVE_SPEED;
                rx = 0;
            } else if (gamepad1.dpad_right) {
                y = 0;
                x = SLOW_DRIVE_SPEED;
                rx = 0;
            } else {
                // Normal joystick drive
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x * 1.1;
                rx = gamepad1.right_stick_x;
            }

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            fl.setPower((y + x + rx) / denom);
            bl.setPower((y - x + rx) / denom);
            fr.setPower((y - x - rx) / denom);
            br.setPower((y + x - rx) / denom);

            CommandScheduler.getInstance().run();

            // === TELEMETRY ===
            telem.addData("=== SHOOTER ===", "");
            telem.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
            telem.addData("Ready", shooter.isReadyToFire() ? "YES" : "NO");

            telem.addData("=== TURRET ===", "");
            telem.addData("Error", "%.2f°", turret.getLastError());
            telem.addData("Locked", turret.isOnTarget() ? "YES" : "NO");
            telemetry.addData("Error Rate", turret.getErrorRate());

            telem.addData("=== LIMELIGHT ===", "");
            telem.addData("Target", limelight.hasTarget() ? "YES" : "NO");
            telem.addData("tx", "%.2f°", limelight.getFilteredTx());
            telem.addData("Distance", "%.1f in", limelight.getDistanceInches());

            telem.addData("Raw Error", "%.2f°", turret.getLastError());
            telem.addData("Has Target", limelight.hasTarget());
            telem.addData("Error < 1?", Math.abs(turret.getLastError()) < 1.0);

            telem.update();
        }

        CommandScheduler.getInstance().reset();
    }
}