/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climb extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid solenoidFrontRight;
  private Solenoid solenoidFrontLeft;
  private Solenoid solenoidBackRight;
  private Solenoid solenoidBackLeft;
  private WPI_VictorSPX moveClimbWheel;
  private DigitalInput checkIfNeedToCloseLeft;
  private DigitalInput checkIfNeedToCloseRight;
  private DigitalInput checkIfNeedToCloseBackWheel;
  //////////////////////////////////////////////////////////////////
  public Climb (Solenoid solenoidFrontRight , Solenoid solenoidFrontLeft ,Solenoid solenoidBackRight
  ,  Solenoid solenoidBackLeft ,WPI_VictorSPX moveClimbWheel ,DigitalInput _checkIfNeedToCloseLeft
  ,DigitalInput _checkIfNeedToCloseRight , DigitalInput _checkIfNeedToCloseBackWheel ){
    this.solenoidFrontRight = solenoidFrontRight;
    this.solenoidFrontLeft = solenoidFrontLeft;
    this.solenoidBackRight = solenoidBackRight;
    this.solenoidBackLeft =solenoidBackLeft;
    this.moveClimbWheel = moveClimbWheel;
    this.checkIfNeedToCloseLeft = _checkIfNeedToCloseLeft ;
    this.checkIfNeedToCloseRight = _checkIfNeedToCloseRight ;
    this.checkIfNeedToCloseBackWheel=_checkIfNeedToCloseBackWheel;
  }
  ////////////////////////////////////////////////////////////////////
  //check if solenoids are open and if not return false
  public boolean isFrontSolenoidsOpen(){
    return this.solenoidFrontLeft.get();
  }
  public boolean isBackSolenoidsOpen(){
    return this.solenoidBackLeft.get();
  }

  ////////////////////////////////////////////////////////////////////
  //set solenoid 

  public void SwitchAllSolenoids() 
  {
    //this.solenoidFrontLeft.set(!this.solenoidFrontLeft.get()); //!this.solenoidFrontLeft.get()
    //this.solenoidFrontRight.set(!this.solenoidFrontRight.get());
    this.solenoidBackLeft.set(!this.solenoidBackLeft.get());
    this.solenoidBackRight.set(!this.solenoidBackRight.get());
  }

  public void SwitchFrontSolenoids()
  {
    //this.solenoidFrontLeft.set(!this.solenoidFrontLeft.get());
    //this.solenoidFrontRight.set(!this.solenoidFrontLeft.get());
    this.solenoidBackRight.set(!this.solenoidBackRight.get());
  }

  public void SwitchBackSolenoids()
  {
    //this.solenoidBackLeft.set(!this.solenoidBackLeft.get());
    //this.solenoidBackRight.set(!this.solenoidBackRight.get());
    this.solenoidBackLeft.set(!this.solenoidBackLeft.get());
  }


  

  /////////////////////////////////////////////////////////////////////
  
  //get and set ClimbWheel speed
  public void setClimbWheelSpeed(double speed){
    this.moveClimbWheel.set(speed);
  }
  public double getClimbWheelSpeed(){
    return this.moveClimbWheel.get();
  }

  /////////////////////////////////////////////////////////////////////

  //check if need to close the front solenoids
  public boolean canCloseFrontSolenoids(){
    return (this.checkIfNeedToCloseLeft.get()&&this.checkIfNeedToCloseRight.get());
  }

  //check if need to close the back solenoid/wheel
  public boolean canCloseBackSolenoid(){
    return this.checkIfNeedToCloseBackWheel.get();
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /////////////////////////////////////////////////////////////////////

}
