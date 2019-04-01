/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Elevator extends PIDSubsystem {


  private WPI_TalonSRX elevatorTalonR;
  private WPI_VictorSPX elevatorFollowerVictorR;
  private WPI_TalonSRX elevatorTalonL;
  private WPI_VictorSPX elevatorFollowerVictorL;
  private Compressor compressor;
  private DigitalInput elevatorMicFloor;
  public DigitalInput elevatorMDigitalInput;
  public double kP, kD, kI;
  public double heightFromFloor;

  // Constractor
  public Elevator(WPI_TalonSRX elevTalR, WPI_VictorSPX elevVicR, WPI_TalonSRX elevTalL, WPI_VictorSPX elevVicL,
      Compressor comp, DigitalInput elevMicFl,DigitalInput elevatorMDigitalInput) {
    super(0.12,0,0);
    this.elevatorTalonL = elevTalL;
    this.elevatorTalonR = elevTalR;
    this.elevatorFollowerVictorL = elevVicL;
    this.elevatorFollowerVictorR = elevVicR;
    this.compressor = comp;
    this.elevatorMicFloor = elevMicFl;
    this.elevatorMDigitalInput = elevatorMDigitalInput;

    this.kP = 0.08;
    this.kI = 0;
    this.kD = 0.04;

  }

  public double getEncoderPosition() {
    return this.elevatorTalonR.getSelectedSensorPosition(0); // * CONVERT_TICKS_TO_METER;
  }

  public double getEncoderVelocity() {
    return this.elevatorTalonR.getSelectedSensorVelocity(0);
  }

  public void resetEncoder() {
    this.elevatorTalonR.setSelectedSensorPosition(0, 0, 10);
  }

  // Set elevator speed
  public void setElevatorSpeed(double speed) {
    this.elevatorTalonR.set(speed);
    this.elevatorTalonL.set(speed);
    this.elevatorFollowerVictorL.set(speed);
    this.elevatorFollowerVictorR.set(speed);
  }

  // Get elevator speed
  public double getElevatorSpeed() {
    return this.elevatorTalonR.get();
  }

  // get sensors

  public boolean getElevatorMicFloor() {
    return this.elevatorMicFloor.get();
  }

  public boolean getCompressorStatus() {
    return this.compressor.enabled();
  }

  public void stopElevatorMotors() {
    this.elevatorTalonR.set(0);
    this.elevatorTalonL.set(0);
  }
  // TODO: add set func to the compressor
  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return elevatorTalonR.getSelectedSensorPosition(0);
  }

  @Override
  protected void usePIDOutput(double output) {
    this.elevatorTalonL.set(-output);
    this.elevatorTalonR.set(-output);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
