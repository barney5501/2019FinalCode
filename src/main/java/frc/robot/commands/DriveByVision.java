/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Driver;

public class DriveByVision extends Command {
  Driver driver;
  double angle;
  AnalogGyro gyro;
  public DriveByVision() {
    // Use requires() here to declare subsystem dependencies
    gyro = RobotMap.gyro;
    driver = Robot.driver;
  }

  // Called just before this Command runs the first time
  boolean startStraightLine;
  double visionAngle;
  @Override
  protected void initialize() {
    visionAngle = Robot.table.getEntry("angle").getDouble(0)*0.75;
    if ((Robot.table.getEntry("angle").getDouble(0) != -999))
    {
    setTimeout(2);
    RobotMap.gyro.reset();
      driver.setAbsoluteTolerance(1);
      driver.setSetpoint(visionAngle);
      SmartDashboard.putNumber("Vision Angle * 0.75", Robot.table.getEntry("angle").getDouble(0)*0.75);
      driver.setOutputRange(-0.8,0.8);
      driver.getPIDController().setPID(SmartDashboard.getNumber("Pgyro",0),SmartDashboard.getNumber("Igyro",0),SmartDashboard.getNumber("Dgyro",0));
      driver.enable();
      Robot.visionFlag = true;
      startStraightLine = true;
      Driver.drivingStraightSpeed = 0;
    }
  }
  boolean pidFlag;
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ((Robot.table.getEntry("angle").getDouble(0) != -999))
    {
      pidFlag = driver.onTarget();
      if (pidFlag && startStraightLine)
      {
        driver.disable();
        driver.setSetpoint(visionAngle);
        driver.setOutputRange(-0.5,0.5);
        driver.drivingStraightSpeed = 0.6;
        driver.enable();
        startStraightLine = false;
      }
    }
    else
    {
      Robot.m_oi.joystickDriver.setRumble(RumbleType.kRightRumble, 1);
      Robot.m_oi.joystickDriver.setRumble(RumbleType.kLeftRumble, 1);
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
    Robot.visionFlag = false;
    driver.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.visionFlag = false;
    driver.disable();
    Driver.drivingStraightSpeed = 0;
    startStraightLine = true;
    Robot.m_oi.joystickDriver.setRumble(RumbleType.kRightRumble, 0);
    Robot.m_oi.joystickDriver.setRumble(RumbleType.kLeftRumble, 0);
  }
}
