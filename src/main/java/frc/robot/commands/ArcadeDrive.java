
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climb;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Driver;

public class ArcadeDrive extends Command {
  private Driver driver; 
  private OI oi;
  private double speed;
  private double rotation;
  private Climb climb;
  private double k; //speed from Smartdashboard

  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.climb = Robot.climb;
    this.driver = Robot.driver;
    this.oi = Robot.m_oi;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //TODO: gyro
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    this.speed = (this.oi.joystickDriver.getRawAxis(1) * SmartDashboard.getNumber("MaxDrivingSpeed", 0.7));
    this.rotation = (this.oi.joystickDriver.getRawAxis(4) * SmartDashboard.getNumber("MaxRotationSpeed", 0.7));
    k = 1 - SmartDashboard.getNumber("MaxDrivingSpeed", 0.7);

    if (speed < 0)
    {
      this.speed += -(oi.joystickDriver.getRawAxis(3) * k);
    }
    else
    {
      this.speed += (oi.joystickDriver.getRawAxis(3) * k);
    }
    
    if (Math.abs(speed) <= 0.2) {
      speed = 0;
    }
    if (Math.abs(rotation) <= 0.2) {
      rotation = 0;
    }

    if (Math.abs(oi.joystickDriver.getRawAxis(2)) > 0.3)
    {
      this.speed *= 0.55/SmartDashboard.getNumber("MaxDrivingSpeed", 0.7);
      this.rotation *= 0.65/SmartDashboard.getNumber("MaxRotationSpeed", 0.7);
    }

    if (Math.abs(RobotMap.elevatorTalonR.getSelectedSensorPosition()) > 2000)
    {
        this.speed *= 0.6;
    }
    
    if (Robot.climbSolFlag)
    {
      this.speed *= 0.5/SmartDashboard.getNumber("MaxDrivingSpeed", 0.7);
      this.driver.arcadeDrive(speed, 0);
      RobotMap.moveClimbWheel.set(speed);
      if (Math.abs(this.speed) > 0.2)
      {
        Robot.climbingFlag = true;
      }
    }
         
    if (!Robot.visionFlag)
      this.driver.arcadeDrive(speed, rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.driver.stopAllWheels();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.driver.stopAllWheels();
  }
}
