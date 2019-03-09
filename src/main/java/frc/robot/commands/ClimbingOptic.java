/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ClimbingOptic extends Command {
  boolean flag;
  public ClimbingOptic() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    flag = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.climbSolFlag)
    {
      if (RobotMap.checkIfNeedToCloseLeft.get() && flag == false)
      {
        Robot.m_oi.joystickDriver.setRumble(RumbleType.kRightRumble, 1);
        Robot.m_oi.joystickDriver.setRumble(RumbleType.kLeftRumble, 1);
      }
      flag = RobotMap.checkIfNeedToCloseLeft.get();
      if (!RobotMap.solenoidBackRight.get())
      {
        Robot.m_oi.joystickDriver.setRumble(RumbleType.kRightRumble, 0);
        Robot.m_oi.joystickDriver.setRumble(RumbleType.kLeftRumble, 0);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
