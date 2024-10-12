package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class GotoPositionCommand extends Command {
    private final Shooter shooter;
    private final double targetPos; 
    private final double tolerance = 2.0;
    private final PIDController pidController;

    private static double kP = 0.05;
    private static double kI = 0.0;
    private static double kD = 0.05;
     
    public GotoPositionCommand(Shooter shooter, double targetPos) {
        this.shooter = shooter;
        this.targetPos = targetPos;
    
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(tolerance);
        addRequirements(shooter);
}
@Override
  public void initialize() {
    pidController.reset();
    pidController.setSetpoint(targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = pidController.calculate(shooter.getPosition());

    shooter.setVelocity(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

}
