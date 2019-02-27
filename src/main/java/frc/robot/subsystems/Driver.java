/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Add your docs here.
 */
public class Driver extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX driverFrontRight;
  private WPI_TalonSRX driverFrontLeft;
  private WPI_VictorSPX driverFollowerRight;
  private WPI_VictorSPX driverFollowerLeft;
  private AnalogGyro gyro;
  // TODO: add GyroNAVX
  private SpeedControllerGroup rightController;
  private SpeedControllerGroup leftController;
  private DifferentialDrive diffDrive;

public Driver(WPI_TalonSRX _driverFrontRight ,WPI_TalonSRX _driverFrontLeft,WPI_VictorSPX _driverFollowerRight , WPI_VictorSPX _driverFollowerLeft, AnalogGyro _gyro){
  super(0.12,0,0);
  this.gyro = _gyro;
  this.driverFrontRight=_driverFrontRight;
  this.driverFrontLeft=_driverFrontLeft;
  this.driverFollowerRight=_driverFollowerRight;
  this.driverFollowerLeft=_driverFollowerLeft;
  this.rightController = new SpeedControllerGroup(this.driverFrontRight);
  this.leftController = new SpeedControllerGroup(this.driverFrontLeft);
  this.diffDrive = new DifferentialDrive(this.leftController, this.rightController);
}

public double getSpeedRight(){
  return this.driverFrontRight.get();
}

public double getSpeedLeft(){
  return this.driverFrontLeft.get();
}


public void setSpeedRight(double speed)
{
  this.driverFrontRight.set(speed);
}

public void setSpeedLeft(double speed)
{
  this.driverFrontLeft.set(speed);
}

public void stopAllWheels()
{
  this.driverFrontLeft.set(0);
  this.driverFrontRight.set(0);
}

public void arcadeDrive(double speed, double rotation)
{
  this.diffDrive.arcadeDrive(-speed, rotation);
}

@Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return gyro.getAngle();
  }
  public static double drivingStraightSpeed;
  @Override
  protected void usePIDOutput(double output) {
    this.diffDrive.arcadeDrive(drivingStraightSpeed, output);
  }
  @Override
  public void initDefaultCommand() {
  }
}
