package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vacuum  extends SubsystemBase{

    public static VictorSPX vacuum = new VictorSPX(60);

    public static Solenoid solenoid = new Solenoid(15, PneumaticsModuleType.CTREPCM, 3);

    public static Compressor pcm = new Compressor(15,PneumaticsModuleType.CTREPCM);
    

   

    public Vacuum(){

        SmartDashboard.putBoolean("Vacuum", solenoid.get());

    solenoid.set(false);

    }

    @Override
    public void periodic() {
        
    }
}
