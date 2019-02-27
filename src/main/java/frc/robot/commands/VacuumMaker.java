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

public class VacuumMaker extends Command {

  private Gripper gripper;
  public VacuumMaker() {

    gripper = Robot.gripper;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  int i;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.vacuumFlag = true;
    i = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (i == 0)
      this.gripper.switchVac();
    i++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !RobotMap.vacumSwitch.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.vacuumFlag = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
