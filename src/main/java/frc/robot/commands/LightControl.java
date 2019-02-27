/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LightControl extends Command {

  byte[] WriteData;
  boolean platte;
  int percent;
  I2C com;
  OI oi;
  double ting;
  int last[]=new int[10];
  public LightControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    oi = Robot.m_oi;
    com = Robot.i2c;
    this.WriteData = new byte[2];
    platte = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // System.out.println("before");
      ting = RobotMap.elevatorTalonR.getSelectedSensorPosition() / 30;
      this.WriteData = (ting+"").getBytes();
      this.com.transaction(this.WriteData, this.WriteData.length, this.WriteData, 1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
   // System.out.println("end");
    
   // System.out.println("end after");
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("interupted");
  }
}
