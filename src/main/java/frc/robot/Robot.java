/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
 
import java.io.IOException;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.Constants;
import frc.robot.Utils.HandleCameras;
import frc.robot.Utils.Motion;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LiftByJoystick;
import frc.robot.commands.MoveGripperByJoystick;
import frc.robot.commands.VacuumByMicro;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Driver;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


public class Robot extends TimedRobot {
  // Motion.m_follower_notifier.stop();
  //this table handles vision values
  public static NetworkTable table;
  public static OI m_oi;
  public static DriverStation ds;
  //our Subsystems ->
  public static Gripper gripper;
  public static Driver driver;
  public static Elevator elevator;
  public static Climb climb;
  //these are command flags, that help us stop commands under the scheduler
  public static boolean commandFlag;
  public static boolean vacuumFlag;
  public static boolean visionFlag;
  public static boolean disableVacumSwitch;
  public static boolean climbingFlag;
  public static boolean climbSolFlag;
  public static HandleCameras handleCameras;
  public static I2C i2c;
  private Trajectory trajectory;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static AHRS Navx;
  @Override
  public void robotInit() {
    RobotMap.Init();
    i2c = RobotMap.i2c;
    Navx = new AHRS(Port.kMXP);
    ds = m_ds;
    this.climbSolFlag = false;
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
        RobotMap.elevatorFollowerVictorL, RobotMap.compressor, RobotMap.elevatorMicFloor, RobotMap.elevatorMag);

    //Sets all the flags to false so we could use the schedulers
    gripper.setPanelPush(false);
    disableVacumSwitch = false;
    commandFlag = false;
    visionFlag = false;
    vacuumFlag = false;
    climbingFlag = false;
    initCameras();
    elevator.resetEncoder();
    //UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
    //cam0.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    //UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
    //cam1.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    m_oi = new OI();
    m_oi.joystickDriver.setRumble(RumbleType.kRightRumble, 0);
    m_oi.joystickDriver.setRumble(RumbleType.kLeftRumble, 0);
    RobotMap.gyro.calibrate();
    climb.setRevers();
    climb.closeAllSolenoinds();

    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0,0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(2, 0, 0)
      // Waypoint @ x=-2, y=-2, exit angle=0 radians
      // new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
  };
  


  Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.05, 2.33, 2.0, 60.0);
   trajectory = Pathfinder.generate(points, config);
  }

  @Override
  public void robotPeriodic() {
    if(Navx.isCalibrating()){
      System.out.println("calibrating");
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  public EncoderFollower m_left_follower;
  public EncoderFollower m_right_follower;
  
  @Override
  public void autonomousInit() {
    driver.setSpeedLeft(0);
    driver.setSpeedRight(0);

    Navx.reset();
    elevator.resetEncoder();
    RobotMap.driverFrontLeft.setSelectedSensorPosition(0, 0, 10);
    RobotMap.driverFrontRight.setSelectedSensorPosition(0, 0, 10);
    //RobotMap.compressor.enabled();
    RobotMap.compressor.close();
    TankModifier modifier = new TankModifier(trajectory).modify(0.575);

    Trajectory left_trajectory = modifier.getLeftTrajectory();
    Trajectory right_trajectory = modifier.getRightTrajectory();
    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);
    
    m_left_follower.configureEncoder(0, Constants.k_ticks_per_rev, Constants.k_wheel_diameter);
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.k_max_velocity, 0);

    m_right_follower.configureEncoder(0, Constants.k_ticks_per_rev, Constants.k_wheel_diameter);
    m_right_follower.configurePIDVA(1.2, 0.0, 0.0, 1 / Constants.k_max_velocity, 0);

    // m_autonomousCommand = m_chooser.getSelected();
    // //Adds our Axis commands to the scheduler 
    // Scheduler.getInstance().add(new ArcadeDrive());
    // Scheduler.getInstance().add(new LiftByJoystick());
    // Scheduler.getInstance().add(new MoveGripperByJoystick());
    // Scheduler.getInstance().add(new VacuumByMicro());
    // RobotMap.vacSol.set(false);
    // SmartDashboardInit();
    // //Scheduler.getInstance().add(new VacuumByMicro());
    
  }

  @Override
  public void autonomousPeriodic() {
   
    SmartDashboardPerodic();
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      Driver.getInstance().setSpeedLeft(0);
      Driver.getInstance().setSpeedRight(0);
    } else {
      double left_speed = m_left_follower.calculate(RobotMap.driverFrontLeft.getSelectedSensorPosition());
      double right_speed = m_right_follower.calculate(-RobotMap.driverFrontRight.getSelectedSensorPosition());
      double heading = Robot.Navx.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      Driver.getInstance().setSpeedLeft(left_speed + turn);
      Driver.getInstance().setSpeedRight(right_speed - turn);
    }
  }

  @Override
  public void teleopInit() {

    //motion.m_follower_notifier.stop();
    driver.setSpeedLeft(0);
    driver.setSpeedRight(0);

    Navx.reset();
    elevator.resetEncoder();
    RobotMap.driverFrontLeft.setSelectedSensorPosition(0, 0, 10);
    RobotMap.driverFrontRight.setSelectedSensorPosition(0, 0, 10);
    //RobotMap.compressor.enabled();
    climbingFlag = false;
    disableVacumSwitch = false;
    commandFlag = false;
    visionFlag = false;
    vacuumFlag = false;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotMap.solenoidBackLeft.set(Value.kReverse);

    SmartDashboardInit();
    //same as the auto function
    RobotMap.elevatorTalonL.setSelectedSensorPosition(0, 0, 10);
    Scheduler.getInstance().add(new ArcadeDrive());
    Scheduler.getInstance().add(new LiftByJoystick());
    Scheduler.getInstance().add(new MoveGripperByJoystick());
    Scheduler.getInstance().add(new VacuumByMicro());
    //Scheduler.getInstance().add(new ClimbingOptic());

  }
  public static double time = 0;
  @Override
  public void teleopPeriodic() {
     //RobotMap.compressor.stop();
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
    value = 0;
    lvalue = RobotMap.driverFrontLeft.getSelectedSensorPosition();
  }
private int value;
private int lvalue;
  //These are values we want to see post-match 
  public void SmartDashboardPerodic() {
    if(!Navx.isCalibrating()){
      SmartDashboard.putNumber("Navx angle", Navx.getAngle());
    }
    SmartDashboard.putNumber("right velocity", RobotMap.driverFrontRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("left velocity",RobotMap.driverFrontLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("left encoder pos", RobotMap.driverFrontLeft.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("right encoder pos", RobotMap.driverFrontRight.getSelectedSensorPosition(0));
    SmartDashboard.putBoolean("Sol State", RobotMap.solenoidFrontLeft.get());
    SmartDashboard.putNumber("Analog Gyro", RobotMap.gyro.getAngle());
    SmartDashboard.putNumber("Vision Angle", table.getEntry("angle").getDouble(0));
    SmartDashboard.putBoolean("Compressor", RobotMap.compressor.enabled());
    SmartDashboard.putNumber("Elevator current", RobotMap.elevatorTalonL.getOutputCurrent());
    SmartDashboard.putNumber("Driving Current", RobotMap.driverFrontRight.getOutputCurrent());
    SmartDashboard.putBoolean("Vacuum Status", !RobotMap.vacSol.get());
    SmartDashboard.putBoolean("Elevator Floor Switch", RobotMap.elevatorMicFloor.get());
    SmartDashboard.putString("Gripper Status", Gripper.gripperStatus.toString());
    SmartDashboard.putBoolean("Climbing Optic", RobotMap.checkIfNeedToCloseLeft.get());
    SmartDashboard.putBoolean("Gripper Switch", RobotMap.gripperSwitch.get());
    SmartDashboard.putNumber("Gripper Count", RobotMap.opticalEncoder.get());
    SmartDashboard.putBoolean("Gripper Digital", RobotMap.opticalDigital.get());
    //Those are values we want to print during testing ->

    SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorTalonR.getSelectedSensorPosition(0));
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