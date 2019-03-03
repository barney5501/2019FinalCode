/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
 
import java.util.ArrayList;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.HandleCameras;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LiftByJoystick;
import frc.robot.commands.LightControl;
import frc.robot.commands.MoveGripperByJoystick;
import frc.robot.commands.VacuumByMicro;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Driver;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;


public class Robot extends TimedRobot {
  //this table handles vision values
  public static NetworkTable table;
  public static OI m_oi;
  public static DriverStation ds;
  //our Subsystems ->
  public static Gripper gripper;
  public static Driver driver;
  public static Compressor compressor;
  public static Elevator elevator;
  public static Climb climb;
  //these are command flags, that help us stop commands under the scheduler
  public static boolean commandFlag;
  public static boolean vacuumFlag;
  public static boolean visionFlag;
  public static boolean disableVacumSwitch;
  public static boolean climbingFlag;
  public static HandleCameras handleCameras;
  public static I2C i2c;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    RobotMap.Init();
    i2c = RobotMap.i2c;
    ds = m_ds;
    //Connects the subsystems to the RobotMap motors and sensors.
    table = NetworkTableInstance.getDefault().getTable("imgProc");
    gripper = new Gripper(RobotMap.moveGripper, RobotMap.rollerGripper, RobotMap.opticalEncoder, RobotMap.pushPanel,
        RobotMap.vacSol, RobotMap.gripperSwitch);
    driver = new Driver(RobotMap.driverFrontRight, RobotMap.driverFrontLeft, RobotMap.driverFollowerRight,
        RobotMap.driverFollowerLeft, RobotMap.gyro);
    climb = new Climb(RobotMap.solenoidFrontRight, RobotMap.solenoidFrontLeft, RobotMap.solenoidBackRight,
        RobotMap.solenoidBackLeft, RobotMap.moveClimbWheel, RobotMap.checkIfNeedToCloseLeft,
        RobotMap.checkIfNeedToCloseRight, RobotMap.checkIfNeedToCloseBackWheel);
    elevator = new Elevator(RobotMap.elevatorTalonR, RobotMap.elevatorFollowerVictorR, RobotMap.elevatorTalonL,
        RobotMap.elevatorFollowerVictorL, RobotMap.compressor, RobotMap.elevatorMicFloor);

    //Sets all the flags to false so we could use the schedulers
    gripper.setPanelPush(false);
    disableVacumSwitch = false;
    commandFlag = false;
    visionFlag = false;
    vacuumFlag = false;
    climbingFlag = false;
    initCameras();
    elevator.resetEncoder();
    m_oi = new OI();
    //UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
    //cam0.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    //UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
    //cam1.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    //Adds our Axis commands to the scheduler 
    Scheduler.getInstance().add(new ArcadeDrive());
    Scheduler.getInstance().add(new LiftByJoystick());
    Scheduler.getInstance().add(new MoveGripperByJoystick());
    RobotMap.vacSol.set(true);
    Scheduler.getInstance().add(new VacuumByMicro());
    //Scheduler.getInstance().add(new VacuumByMicro());
  }

  @Override
  public void autonomousPeriodic() {
    //Starts our scheduler so we could use the joysticks
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    elevator.resetEncoder();
    RobotMap.compressor.stop();
    climbingFlag = false;
    disableVacumSwitch = false;
    commandFlag = false;
    visionFlag = false;
    vacuumFlag = false;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboardInit();
    //same as the auto function
    RobotMap.elevatorTalonL.setSelectedSensorPosition(0, 0, 10);
    Scheduler.getInstance().add(new ArcadeDrive());
    Scheduler.getInstance().add(new LiftByJoystick());
    Scheduler.getInstance().add(new MoveGripperByJoystick());
    Scheduler.getInstance().add(new VacuumByMicro());

  }
  public static double time = 0;
  @Override
  public void teleopPeriodic() {
    //same as the auto function
    Scheduler.getInstance().run();
    if (disableVacumSwitch)
    {
    }
    SmartDashboardPerodic();
  }

  @Override
  public void testPeriodic() {
  }

  //These are values we want to define during our practice
  public void SmartDashboardInit() {
    SmartDashboard.putNumber("MaxDrivingSpeed", 0.8);
    SmartDashboard.putNumber("MaxRotationSpeed", 0.85);
    SmartDashboard.putNumber("MaxElevatorSpeed", 0.85);
    SmartDashboard.putNumber("MaxGripperSpeed", 1);
    SmartDashboard.putNumber("Climb Wheel Speed", -0.5);
    SmartDashboard.putNumber("P", 0.006); 
    SmartDashboard.putNumber("I", 0.0);
    SmartDashboard.putNumber("D", 0.01);
    SmartDashboard.putNumber("Igyro", 0.1);
    SmartDashboard.putNumber("Pgyro", 0.0);
    SmartDashboard.putNumber("Dgyro", 0.15);
  }

  //These are values we want to see post-match 
  public void SmartDashboardPerodic() {
    SmartDashboard.putBoolean("Sol State", RobotMap.solenoidFrontLeft.get());
    SmartDashboard.putNumber("Analog Gyro", RobotMap.gyro.getAngle());
    SmartDashboard.putNumber("Vision Angle", table.getEntry("angle").getDouble(0));
    SmartDashboard.putBoolean("Compressor", RobotMap.compressor.enabled());
    SmartDashboard.putNumber("Elevator current", RobotMap.elevatorTalonL.getOutputCurrent());
    SmartDashboard.putNumber("Driving Current", RobotMap.driverFrontRight.getOutputCurrent());
    SmartDashboard.putBoolean("Vacuum Status", !RobotMap.vacSol.get());
    SmartDashboard.putBoolean("Elevator Floor Switch", RobotMap.elevatorMicFloor.get());
    SmartDashboard.putString("Gripper Status", Gripper.gripperStatus.toString());
    //Those are values we want to print during testing ->

    //SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorTalonR.getSelectedSensorPosition(0));
    //SmartDashboard.putNumber("Elevator Joystick", m_oi.joystickOperator.getRawAxis(1));
    //SmartDashboard.putBoolean("Vaccum Switch", RobotMap.vacumSwitch.get());
    //SmartDashboard.putBoolean("Gripper Switch", RobotMap.gripperSwitch.get());
  }

  public void initCameras() {
    //This command connects our cameras to the MJPEG server
    ArrayList<VideoCamera> cams = new ArrayList<>();
    for (int i = 0; i < 2; i++) {
      UsbCamera cam = new UsbCamera("cam" + i, i);
      cam.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
      cams.add(cam);
    }
    handleCameras = new HandleCameras(cams);
  }
}