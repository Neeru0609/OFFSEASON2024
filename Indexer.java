package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private final TalonFX indexermotor = new TalonFX(7);
    private final TalonFX wheelsmotor = new TalonFX(8);

    public Indexer() {
    var indexerConfig = new TalonFXConfiguration();

    indexerConfig.CurrentLimits.StatorCurrentLimit = 35;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.MotorOutput.Inverted = (InvertedValue.Clockwise_Positive);
    
    indexermotor.getConfigurator().apply(indexerConfig);
    }
    public void Methodspin1(){
        wheelsmotor.set(1000);

        while (true){
            double currentDraw = Math.abs(indexermotor.getSupplyCurrent().getValueAsDouble());

            if(currentDraw > 25){
                wheelsmotor.set(0);
                break;
            }
        }
        
    }

    public void Methodspin2(){
        wheelsmotor.set(25);

        while(true){
            double currentDraw = Math.abs(indexermotor.getSupplyCurrent().getValueAsDouble());

            if(currentDraw < 5){
                wheelsmotor.set(0);
                break;
            }
        }
    }

    public void stop(){
        indexermotor.setVoltage(0);
    }

    public Command indexerMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
    }

    public boolean indexerCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
      }
}
