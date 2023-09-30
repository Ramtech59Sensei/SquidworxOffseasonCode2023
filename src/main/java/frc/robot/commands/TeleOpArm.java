package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class TeleOpArm extends CommandBase {
    private static Arm myArm;
    public TeleOpArm(Arm arm) {
        myArm = arm;
        addRequirements(myArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((Robot.m_driverController.getBButton() || Robot.m_CoController.getBButton())){
            myArm.moveHA(0.5);
        }else if (Robot.m_driverController.getAButton() || Robot.m_CoController.getAButton()){
            myArm.moveHA(-0.5);
        }else {
            myArm.moveHA(0.0);
        } 
        if ((Robot.m_driverController.getLeftBumper() || Robot.m_CoController.getLeftBumper())){
            myArm.moveVA(0.5);
        }else if (Robot.m_driverController.getRightBumper() || Robot.m_CoController.getRightBumper()){
            myArm.moveVA(0.5);
        }else  myArm.moveVA(0.0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        myArm.moveHA(0.0);
        myArm.moveVA(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
