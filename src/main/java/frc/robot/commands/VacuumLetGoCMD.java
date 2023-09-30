package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vacuum;

public class VacuumLetGoCMD extends CommandBase {





    public VacuumLetGoCMD(Vacuum vacuum) {
        addRequirements(vacuum);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Vacuum.solenoid.set(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Vacuum.solenoid.set(false);
    }
    
}
