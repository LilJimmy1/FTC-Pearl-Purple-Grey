package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretAlignment extends CommandBase {

    private final Turret turret;

    public TurretAlignment(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Use the auto-aim wrapper (which internally calls updateAutoAimAndGetTargetRpm)
        turret.autoAim();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        // continuous command; ends only when interrupted
        return false;
    }
}
