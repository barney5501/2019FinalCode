/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class Gripper extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum GripperStatus{
    Top,
    Mid,
    Bottom;
    }


  private Spark moveGripper;
  private Spark rollerGripper;
  private Counter opticalEncoder;
  private DigitalInput gripperSwitch;
  private Solenoid panelArm;
  private Solenoid vacSol;
  public static GripperStatus gripperStatus;
  
  //TODO: add optic encoder

  //constructor function
  public Gripper(Spark _moveGripper, Spark _rollerGripper, Counter _opticalEncoder, 
  Solenoid _panelArm, Solenoid _vacSol, DigitalInput _gripperSwitch)
  {
    this.moveGripper = _moveGripper;
    this.rollerGripper = _rollerGripper;
    this.opticalEncoder = _opticalEncoder;
    this.gripperSwitch = _gripperSwitch;
    this.panelArm = _panelArm;
    this.vacSol = _vacSol;
    gripperStatus = GripperStatus.Top; //The Gripper starts inside the robot.

    ////////////////////////////////////////
    //this.opticalEncoder.setDistancePerPulse(1); //1 stands for everytime the optic is true
    //this.opticalEncoder.reset();
  }
  //get OpticalEncoder Value
  public double getOpticalEncoderDistance(){
    return this.opticalEncoder.getDistance();
  }
  //reset OpticalEncoder 
  public void resetOpticalEncoder(){
   this.opticalEncoder.reset();
  }

  //gets the speed of the right and left gripper motors (spark)
  public double getGripperSpeed()
  {
    return this.moveGripper.get();
    //return this.moveGripperLeft.get();
  }

  //sets the speed of the right and left gripper motors (spark)
  public void setGripperSpeed(double speed)
  {
    this.moveGripper.set(speed);
  }

  //gets the speed of the roller gripper speed
  public double getRollerGripperSpeed()
  {
    return this.rollerGripper.get();
  }

  //sets the speed of the roller gripper's speed
  public void setRollerGripperSpeed(double speed)
  {
    this.rollerGripper.set(speed);
  }

  public boolean getVacSol(){
    return this.vacSol.get();
  }

  public boolean getGripperSwitch(){
    return this.gripperSwitch.get();
  }


  //switch the vacRight solenoid and vacLeft solenoid state 
  public void switchVac(){
    this.vacSol.set(!(getVacSol()));
  }


  //gets if the 
  public boolean getPanelArm()
  {
    return this.panelArm.get();
    //return this.pushPanelRight.get();
  }

  //option one to set the solenoid
  public  void switchPanelArm()
  {
    this.panelArm.set(!(getPanelArm()));
  }

  //option two to set the solenoid
  public void setPanelPush(boolean isPushed)
  {
    this.panelArm.set(isPushed);
  }
  /////////////////////////////////////////////
  //stop motors
  public void stopGripperMotor(){
    this.moveGripper.set(0);
  }




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
