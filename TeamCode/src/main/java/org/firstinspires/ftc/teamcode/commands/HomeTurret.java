//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//
//public class HomeTurret extends CommandBase {
//
//    private final Turret turret;
//    private final ElapsedTime timer = new ElapsedTime();
//    private final double timeoutSeconds;
//
//    /**
//     * @param turret         your Turret subsystem
//     * @param timeoutSeconds safety timeout (e.g. 2.0)
//     */
//    public HomeTurret(Turret turret, double timeoutSeconds) {
//        this.turret = turret;
//        this.timeoutSeconds = timeoutSeconds;
//        addRequirements(turret);
//    }
//
//    @Override
//    public void initialize() {
//        timer.reset();
//    }
//
//    @Override
//    public void execute() {
//        turret.homeStep();
//    }
//
//    @Override
//    public boolean isFinished() {
//        // stop either when we're home, or if timeout hits
//        return turret.isAtHome() || timer.seconds() > timeoutSeconds;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        turret.stop();
//        if (turret.isAtHome()) {
//            // encoder re-zero when homed
//            turret.resetEncoderAtHome();
//        }
//    }
//}
