package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vacuum;

public class VacuumSuckCMD  extends CommandBase{

    private double speed;
    

    public VacuumSuckCMD(Vacuum vacuum, double speed) {
        this.speed = speed;
        addRequirements(vacuum);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Vacuum.vacuum.set(VictorSPXControlMode.PercentOutput, speed);
       Vacuum.solenoid.set(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Vacuum.vacuum.set(VictorSPXControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
