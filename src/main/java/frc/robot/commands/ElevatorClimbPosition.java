/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorClimbPosition extends Command {
  Elevator elevator;
  public ElevatorClimbPosition() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    elevator = Robot.elevator;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(4);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    elevator.setElevatorSpeed(-0.25);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotMap.elevatorTalonR.getSelectedSensorPosition() > 320 || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setElevatorSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elevator.setElevatorSpeed(0);
  }
}
