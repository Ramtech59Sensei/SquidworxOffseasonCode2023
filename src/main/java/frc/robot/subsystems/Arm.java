package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
  private static final double VerticalMinLimit = .05;
  private static final double VerticalMaxLimit = 2.2;
  private static final double HorizontalMinLimit = .4;
  private static final double HorizontalMaxLimit = .96;
  private static final CANSparkMax ArmVerticalExtMotor=new CANSparkMax(50, MotorType.kBrushed);
  private static final CANSparkMax ArmHorizontalExtMotor=new CANSparkMax(51, MotorType.kBrushed);
  private static SparkMaxAnalogSensor potentiometerH = ArmHorizontalExtMotor.getAnalog(Mode.kAbsolute);
  private static SparkMaxAnalogSensor potentiometerV = ArmVerticalExtMotor.getAnalog(Mode.kAbsolute);
  private static SparkMaxPIDController  armHorizontalExt_pidController = ArmHorizontalExtMotor.getPIDController();
  private static SparkMaxPIDController  armVerticalExt_pidController = ArmVerticalExtMotor.getPIDController();



    
  public Arm() {
    // double kMaxOutput = 1; 
    // double kMinOutput = -1;
    // double kMaxOutputH = 1; 
    // double kMinOutputH = -1;
    // armHorizontalExt_pidController.setP(kP0, 0);
    // armHorizontalExt_pidController.setI(kI0, 0);
    // armHorizontalExt_pidController.setD(kD0, 0);
    // armHorizontalExt_pidController.setOutputRange(kMinOutput0, kMaxOutput0,0);
    //ArmHorizontalExtMotor.setClosedLoopRampRate(0.1);

    // armVerticalExt_pidController.setP(kP0, 0);
    // armVerticalExt_pidController.setI(kI0, 0);
    // armVerticalExt_pidController.setD(kD0, 0);
    // armVerticalExt_pidController.setOutputRange(kMinOutput0, kMaxOutput0,0);
    //ArmVerticalExtMotor.setClosedLoopRampRate(0.1);
      
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vertical", potentiometerV.getPosition());
    SmartDashboard.putNumber("Horizontal", potentiometerH.getPosition());
  }

  public void moveVA(double power){
    if (power > 0 ){
      if (potentiometerV.getPosition() < VerticalMaxLimit){
        ArmVerticalExtMotor.set(power);
      } else {
        ArmVerticalExtMotor.set(0);
      }
    }else if(power < 0){  
      if (potentiometerV.getPosition() > VerticalMinLimit){
        ArmVerticalExtMotor.set(power);
      }else {
        ArmVerticalExtMotor.set(0.0);
      } 
    }else ArmHorizontalExtMotor.set(0.0);
  }

  public void moveHA(double power){
    if (power > 0 ){
            if (potentiometerH.getPosition() < HorizontalMaxLimit){
                ArmHorizontalExtMotor.set(power);
            }else {
                ArmHorizontalExtMotor.set(0.0);
            } 
    }else if(power < 0){
            if (potentiometerH.getPosition() > HorizontalMinLimit){
                ArmHorizontalExtMotor.set(power);
            }else {
                ArmHorizontalExtMotor.set(0.0);
            } 
    }else ArmHorizontalExtMotor.set(0.0);
  }
  public void moveHASetPoint(double position){
    armHorizontalExt_pidController.setReference(position, CANSparkMax.ControlType.kPosition,0);
  }
  public void moveVASetPoint(double position){
    armVerticalExt_pidController.setReference(position, CANSparkMax.ControlType.kPosition,0);
  }
}