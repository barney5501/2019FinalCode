/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
     
import javax.crypto.spec.SecretKeySpec;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;

public class LiftByJoystick extends Command {
  private Elevator elevator;
  private boolean climbStart;
  private double elevatorSpeed;
  private OI oi;
  private boolean flag;
  private Timer time;

  public LiftByJoystick() {
    this.elevator = Robot.elevator;
    this.oi = Robot.m_oi;
    flag = false;
    time = new Timer();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climbStart = false;
  }

  // Called repeatedly when this Command is scheduled to run
  // TODO: check
  @Override
  protected void execute() {
    this.elevatorSpeed = (this.oi.joystickOperator.getRawAxis(1) * SmartDashboard.getNumber("MaxElevatorSpeed", 0.9));
    if (!Robot.commandFlag && Math.abs(this.elevatorSpeed) < 0.1) {
      if (flag == false) {
        elevator.setElevatorSpeed(0.056);
        flag = true;
      }
    } else if (!Robot.commandFlag) {
      this.elevator.setElevatorSpeed(elevatorSpeed);
      flag = false;
    }

    //climb sequence
    if (Robot.climbingFlag && !climbStart) 
    {
      time.start();
      climbStart = true;
    }
    if (climbStart && time.get() < 1)
    {
      this.elevator.setElevatorSpeed(0.2);
    }

    
    if (RobotMap.elevatorMicFloor.get())
    {
      this.elevator.setElevatorSpeed(0);
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
    this.elevator.setElevatorSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.elevator.setElevatorSpeed(0.0);
  }
}
