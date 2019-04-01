/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
//import com.ctre.phoenix.motorControl.can.WPI_TalonSRX;

public class RobotMap {

  // Driver Motors:
  public static WPI_TalonSRX driverFrontRight;
  public static WPI_TalonSRX driverFrontLeft;
  public static WPI_VictorSPX driverFollowerRight;
  public static WPI_VictorSPX driverFollowerLeft;
  public static AnalogGyro gyro;
  // Climb Motors:
  public static Solenoid solenoidFrontRight;
  public static Solenoid solenoidFrontLeft;
  public static Solenoid solenoidBackRight;
  public static DoubleSolenoid solenoidBackLeft;
  public static WPI_VictorSPX moveClimbWheel;
  public static DigitalInput checkIfNeedToCloseLeft;
  public static DigitalInput checkIfNeedToCloseRight;
  public static DigitalInput checkIfNeedToCloseBackWheel;

  // Elevator Motors:
  public static WPI_TalonSRX elevatorTalonR;
  public static WPI_VictorSPX elevatorFollowerVictorR;
  public static WPI_TalonSRX elevatorTalonL;
  public static WPI_VictorSPX elevatorFollowerVictorL;
  public static Compressor compressor;
  public static DigitalInput elevatorMicFloor;
  public static DigitalInput elevatorMag;

  // Gripper Motors:
  public static Spark moveGripper;
  public static Spark rollerGripper;
  public static Counter opticalEncoder;
  public static Solenoid pushPanel;
  public static Solenoid vacSol;
  public static DigitalInput gripperSwitch;
  public static DigitalInput opticalDigital;
  public static DigitalInput vacumSwitch;

  //LEDs network protocol handler
  public static I2C i2c;

public static void Init() {
  i2c = new I2C(I2C.Port.kOnboard, 4);

// Driver Ports:
driverFollowerLeft = new WPI_VictorSPX(1);
driverFollowerRight = new WPI_VictorSPX(0);
driverFrontLeft = new WPI_TalonSRX(1);
driverFrontRight = new WPI_TalonSRX(0);
gyro = new AnalogGyro(0);
driverFollowerLeft.follow(driverFrontLeft);
driverFollowerRight.follow(driverFrontRight);

// Elevator Ports:
elevatorTalonR = new WPI_TalonSRX(2);
elevatorMag = new DigitalInput(7);
elevatorFollowerVictorR = new WPI_VictorSPX(2);
elevatorTalonL = new WPI_TalonSRX(3);
elevatorFollowerVictorL = new WPI_VictorSPX(3);
compressor = new Compressor(0);
elevatorFollowerVictorR.follow(elevatorTalonR);
elevatorFollowerVictorL.follow(elevatorTalonL);
elevatorMicFloor = new DigitalInput(8);

// Gripper Ports

moveGripper = new Spark(1); //Could be 2, need to check
rollerGripper = new Spark(0);
opticalDigital = new DigitalInput(0);
gripperSwitch = new DigitalInput(2); 
opticalEncoder = new Counter(opticalDigital);
pushPanel = new Solenoid(3);
vacSol = new Solenoid(2); 
vacumSwitch = new DigitalInput(3);
compressor = new Compressor(0);


// Climb Ports:
solenoidFrontRight = new Solenoid(6);
solenoidFrontLeft = new Solenoid(4);
solenoidBackRight = new Solenoid(7);
solenoidBackLeft = new DoubleSolenoid(0,1);
moveClimbWheel = new WPI_VictorSPX(4);
checkIfNeedToCloseLeft = new DigitalInput(7);
checkIfNeedToCloseRight = new DigitalInput(4);
checkIfNeedToCloseBackWheel = new DigitalInput(5);
  }
}
