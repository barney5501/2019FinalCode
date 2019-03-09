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
import frc.robot.subsystems.Gripper.GripperStatus;

public class MoveGripperToFloor extends Command {
  Gripper gripper;
  double opticAmount;
  boolean flag;
  public MoveGripperToFloor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    gripper = Robot.gripper;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    gripper.resetOpticalEncoder();
    opticAmount = 0;
    flag = false;
    setTimeout(2.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    switch (Gripper.gripperStatus)
    {
      case Top:
      {
        System.out.println("TOP");
        opticAmount = 6;
        if (RobotMap.opticalEncoder.get() == 5)
        {
          gripper.setGripperSpeed(-1);
        }
        else{
          gripper.setGripperSpeed(-1);
        }
        break;
      }
      case Mid: 
      {
        System.out.println("BOTTOM");
        opticAmount = 3;
        if (RobotMap.opticalEncoder.get() == 5)
        {
          gripper.setGripperSpeed(-1);
        }
        else{
          gripper.setGripperSpeed(-1);
        }
        break;
      }
    }
    if (RobotMap.opticalEncoder.get() > opticAmount - 1)
    {
      flag = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || flag || Gripper.gripperStatus == Gripper.GripperStatus.Bottom;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Gripper.gripperStatus = GripperStatus.Bottom;
    this.gripper.setRollerGripperSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Gripper.gripperStatus = GripperStatus.Bottom;
    this.gripper.setRollerGripperSpeed(0);
  }
}
