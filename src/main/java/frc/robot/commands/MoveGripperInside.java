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
import frc.robot.subsystems.Gripper;

public class MoveGripperInside extends Command {
  Gripper gripper;
  double opticAmount;
  public MoveGripperInside() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    gripper = Robot.gripper;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    gripper.resetOpticalEncoder();
    setTimeout(3);
    opticAmount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    gripper.setGripperSpeed(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || RobotMap.gripperSwitch.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Gripper.gripperStatus = Gripper.GripperStatus.Top;
    this.gripper.setGripperSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Gripper.gripperStatus = Gripper.GripperStatus.Top;
    this.gripper.setGripperSpeed(0);
  }
}
