/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class MoveToFloor extends Command {
  private Elevator elevator;

  public MoveToFloor() {
    this.elevator = Robot.elevator;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.commandFlag = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!elevator.getElevatorMicFloor())
      elevator.setElevatorSpeed(-0.4);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.getElevatorMicFloor();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setElevatorSpeed(0);
    Robot.commandFlag = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
