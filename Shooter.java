package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterpivot = new TalonFX(5);
    private final TalonFX shooterwheels = new TalonFX(6);
    private final CANcoder encoder = new CANcoder(2);
    //Need to confirm ID

    public Shooter(){
        var shooterConfig = new TalonFXConfiguration();

        shooterConfig.CurrentLimits.StatorCurrentLimit = 50; 
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.MotorOutput.Inverted = (InvertedValue.CounterClockwise_Positive);

        shooterpivot.getConfigurator().apply(shooterConfig);


        var shooterwheelsConfig = new TalonFXConfiguration();
        
        shooterwheelsConfig.CurrentLimits.StatorCurrentLimit = 50; 
        shooterwheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterwheelsConfig.MotorOutput.Inverted = (InvertedValue.CounterClockwise_Positive);

        shooterwheels.getConfigurator().apply(shooterConfig);
    }
    
    public Command shooterMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
    }

    public boolean shooterCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
      }


    public double getVelocity(){
       return shooterpivot.getVelocity().getValue();
    }

    public double getPosition(){
        return shooterpivot.getPosition().getValue();
    }

    public void setVelocity(double velocity){
        shooterpivot.set(velocity);
      
    }

    public void stop(){
        shooterpivot.setVoltage(0);
        }

    
    public void resetEncoder(){
        encoder.setPosition(0);
    }

    public void setVelocityWheels(double velocity){
        shooterwheels.set(velocity);
    }

    public void getVelocityWheels(){
        shooterwheels.getVelocity().getValue();
    }





}
