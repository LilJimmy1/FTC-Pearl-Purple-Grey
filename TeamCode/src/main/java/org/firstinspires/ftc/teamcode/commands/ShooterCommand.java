//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//import java.util.function.DoubleSupplier;
//
//public class ShooterCommand extends CommandBase {
//
//    private final Shooter shooter;
//    private final DoubleSupplier inputSupplier;
//
//    /**
//     * @param shooter        Shooter subsystem
//     * @param inputSupplier  e.g. () -> gamepad2.right_trigger
//     */
//    public ShooterCommand(Shooter shooter, DoubleSupplier inputSupplier) {
//        this.shooter = shooter;
//        this.inputSupplier = inputSupplier;
//        addRequirements(shooter);
//    }
//
//    @Override
//    public void initialize() {
//        // No mode changes here; Shooter keeps whatever manual target RPM
//        // and mode you configured (e.g. via Dashboard).
//    }
//
//    @Override
//    public void execute() {
//        double cmd = inputSupplier.getAsDouble();   // e.g. gamepad2.right_trigger
//
//        // Fire when trigger pressed > 0.5
//        if (cmd > 0.5) {
//            shooter.runAtManualRPM();
//        } else {
//            shooter.stop();
//        }
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        shooter.stop();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;   // runs until TeleOp ends
//    }
//}
