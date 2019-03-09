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

public class MoveGripperToMid extends Command {
  Gripper gripper;
  boolean flag;
  double opticAmount;
  public MoveGripperToMid() {
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
    flag = false;
  }

  // Called repeatedly when this Command is scheduled to run
  //TODO: fix this
  @Override
  protected void execute() {
    switch (Gripper.gripperStatus)
    {
      case Top:
      {
        opticAmount = 3;
        if (RobotMap.opticalEncoder.get() == 2)
        {
          gripper.setGripperSpeed(-0.9);
        }
        else{
          gripper.setGripperSpeed(-1);
        }
        break;
      }
      case Bottom: 
      {
        opticAmount = 8;
        gripper.setGripperSpeed(1);
        break;
      }
    }

      if (RobotMap.opticalEncoder.get() >= opticAmount - 1)
      {
        flag = true;
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return flag || Gripper.gripperStatus == Gripper.GripperStatus.Mid || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Done");
    Gripper.gripperStatus = Gripper.GripperStatus.Mid;
    this.gripper.setGripperSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Gripper.gripperStatus = Gripper.GripperStatus.Mid;
    this.gripper.setGripperSpeed(0);
  }
}
