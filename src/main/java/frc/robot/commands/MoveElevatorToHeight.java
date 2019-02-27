/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToHeight extends Command {

  private Elevator elevatorPID;
  private double height;

  public MoveElevatorToHeight(double height) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.elevatorPID = Robot.elevator;
    this.height = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.commandFlag = true;
    setTimeout(2);
    // both begin true in order to ignore current sensor
    elevatorPID.disable();
    elevatorPID.setAbsoluteTolerance(20);
    double p = SmartDashboard.getNumber("P", 0);
    double i = SmartDashboard.getNumber("I", 0);
    double d = SmartDashboard.getNumber("D", 0);
    elevatorPID.setSetpoint(this.height);
    elevatorPID.setOutputRange(-0.5, 0.5);
    elevatorPID.getPIDController().setPID(p, i, d);
    elevatorPID.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(elevatorPID.getPIDController().get());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || elevatorPID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevatorPID.disable();
    Robot.elevator.setElevatorSpeed(0.05);
    Robot.commandFlag = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
