package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmPreset extends CommandBase{

    private double xPosition;
    private double yPosition;
    private final Arm myArm ;
    private int i;

    public MoveArmPreset(Arm Arm, double xPos, double yPos) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.xPosition = xPos;
        this.yPosition = yPos;
        this.myArm = Arm;
        addRequirements(Arm);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
         // calculate motion PAth
         int i= 0;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // followw path 
        //   pose = readpathposition(i);  
        //   myArm.moveHASetPoint(pose.hposition);
        //   myArm.moveVASetPoint(pose.vPosition);
        //   i++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        }
    }    

