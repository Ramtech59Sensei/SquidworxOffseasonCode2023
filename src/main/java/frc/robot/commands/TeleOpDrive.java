package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;

public class TeleOpDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_subsystem;
    private double joystick_Y =0;
    private double joystick_X =0;
    private double joystick_Z =0;  
    private double rotMultiplier = 3;  
    // private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Drive.kXLimiterRate);
    // private SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Drive.kYLimiterRate);
    // private SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Drive.kZLimiterRate);
    //public double maxSpeed = PhysicsConstants.kMaxSpeedMtsxSec;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    // Use addRequirements() here to declare subsystem dependencies.
    public TeleOpDrive(DriveSubsystem subsystem) {
      m_subsystem = subsystem;
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.drive( -MathUtil.applyDeadband(Robot.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(Robot.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(Robot.m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true, false);
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