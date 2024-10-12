// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX intakemotor = new TalonFX(3);
  private final TalonFX pivotmotor = new TalonFX(4);
  private final double pivotTarget = 50.0;  
  private final double tolerance = 2.0;   
  private static final double kP = 0.5; 
  private static final double kI = 0.1; 
  private static final double kD = 0.5;  
  private final PIDController pidController = new PIDController(kP, kI, kD);


  private StatusSignal<Double> pivotPos; 

  public Intake() {
    var intakeConfig = new TalonFXConfiguration();

    intakeConfig.CurrentLimits.StatorCurrentLimit = 35;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.MotorOutput.Inverted = (InvertedValue.Clockwise_Positive);
    
    intakemotor.getConfigurator().apply(intakeConfig);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.StatorCurrentLimit = 35; 
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.MotorOutput.Inverted = (InvertedValue.Clockwise_Positive);

    pivotmotor.getConfigurator().apply(pivotConfig);

    pidController.setTolerance(tolerance);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
   public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  } 

    public Command intakeMethCommand(){
      return runOnce(
        () -> {
          /* one-time action goes here */
        });
    }
    public boolean intakeCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void setVelocity(double vel){
    intakemotor.set(vel);
  }

  public void setVelocity(){
    intakemotor.set(0);
  }

  public void stop(){
    intakemotor.setVoltage(0);
  }

  public double getPivotPosition(){
    return pivotmotor.getPosition().getValue();
  }

  public void setPivotVelocity(double vel){
    pivotmotor.set(vel);
  }
  public double getPivotAngle(){
    return getPivotPosition()*(360.00/4096) ; 
  }

  public void moveUp(){

    double targetpos = getPivotAngle() + 50;
    pidController.setSetpoint(targetpos);
    
    while(!pidController.atSetpoint()){
      double output = pidController.calculate(getPivotAngle());
      setPivotVelocity(output);
    }

    stop();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
