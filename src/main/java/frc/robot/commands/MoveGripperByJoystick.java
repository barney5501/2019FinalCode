/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Gripper;

public class MoveGripperByJoystick extends Command {

  private double speed;
  private Gripper gripper;
  private OI oi;
  public MoveGripperByJoystick() {
    this.oi = Robot.m_oi;
    this.gripper = Robot.gripper;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //TODO 
    this.speed = -this.oi.consoleOperator.getRawAxis(1) * SmartDashboard.getNumber("MaxGripperSpeed", 1);
    //if (!gripper.getGripperSwitch())
    this.gripper.setGripperSpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //gripper.getGripperSwitch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.gripper.setGripperSpeed(0);


  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("error");
    this.gripper.setGripperSpeed(0);
  }
}
