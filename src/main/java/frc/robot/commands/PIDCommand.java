/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SingleMotorSubsystem;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PIDCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SingleMotorSubsystem m_subsystem;
  //private static final double kPositionSetPoint = 2;
  private final double kP = 0.25;
  private final double kD = 0.33;
  private final AnalogPotentiometer mPotentiometer;

  private double lastTime;
  private double lastError;

  /**
   * Creates a new PIDCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PIDCommand (SingleMotorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    mPotentiometer = new AnalogPotentiometer(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = mPotentiometer.get() - m_subsystem.getPosition();
    double errorDifference = error - lastError;
    lastError = error;

    double currentTime = Timer.getFPGATimestamp();
    double timeDifference = currentTime - lastTime;
    lastTime = currentTime;

    double derivative = errorDifference/timeDifference;

    double output = error*kP+derivative*kD;

    m_subsystem.setPercentOutput(output);

    SmartDashboard.putNumber("DriveForward/LeftSpeed", output);
    SmartDashboard.putNumber("Derivative", derivative);
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