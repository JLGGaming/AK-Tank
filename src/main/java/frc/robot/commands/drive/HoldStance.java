// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class HoldStance extends Command {
  /** Creates a new HoldStance. */
  PIDController leftPID = new PIDController(40, 0, 0);

  PIDController rightPID = new PIDController(40, 0, 0);

  public HoldStance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double leftSetpoint = RobotContainer.drive.getLeftPositionMeters();
    double rightSetpoint = RobotContainer.drive.getRightPositionMeters();

    leftPID.setSetpoint(leftSetpoint);
    rightPID.setSetpoint(rightSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.driveVolts(
        leftPID.calculate(RobotContainer.drive.getLeftPositionMeters()),
        rightPID.calculate(RobotContainer.drive.getRightPositionMeters()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
