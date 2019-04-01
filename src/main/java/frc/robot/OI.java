/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ActivateClimbWheel;
import frc.robot.commands.AutoElevatorToCatchCargo;
import frc.robot.commands.AutoElevatorToCatchPanel;
import frc.robot.commands.CargoCollector;
import frc.robot.commands.DriveByVision;
import frc.robot.commands.MoveElevatorToHeight;
import frc.robot.commands.MoveGripperInside;
import frc.robot.commands.MoveGripperToFloor;
import frc.robot.commands.MoveGripperToMid;
import frc.robot.commands.PanelArm;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SwitchAllSolenoids;
import frc.robot.commands.SwitchBackSolenoids;
import frc.robot.commands.SwitchCameras;
import frc.robot.commands.SwitchCompressor;
import frc.robot.commands.SwitchFrontSolenoids;
import frc.robot.commands.VacuumMaker;

public class OI {

    //Driver Joystick & Buttons
    public Joystick joystickDriver;
    public JoystickButton a; //Switch Cameras
    public JoystickButton b; //Push Panel Arm
    public JoystickButton x; //Activate Vacum
    public JoystickButton y; //Vision Align
    public JoystickButton rb; //Catch the cargo
    public JoystickButton lb; //Shoot the cargo
    public JoystickButton start; //FuckGoBack;
     
    //Console Joystick & Buttons
    public Joystick consoleOperator; //joystickOperator
    public JoystickButton consoleA; //Elevator move to floor
    public JoystickButton consoleD; //Elevator move to low panel
    public JoystickButton consoleE; //Elavator move to middle panel
    public JoystickButton consoleB; //Elevator move to low cargo
    public JoystickButton consoleF; //Gripper inside

    //Operator Joystick & Buttons
    public Joystick joystickOperator;
    public JoystickButton btn6; //Compressor
    public JoystickButton btn5; //All solenoids are opening
    public JoystickButton btn3; //Back solenoids are closing
    public JoystickButton btn8;
    public JoystickButton btn7;

    public OI(){
    //This sets the buttons to their location on the joystick
    joystickDriver = new Joystick(0);
    a = new JoystickButton(joystickDriver,1); 
    b = new JoystickButton(joystickDriver,2);
    x = new JoystickButton(joystickDriver,3);
    y = new JoystickButton(joystickDriver,4);
    rb = new JoystickButton(joystickDriver,5);
    lb = new JoystickButton(joystickDriver,6);
    start = new JoystickButton(joystickDriver, 7);
     
    joystickOperator = new Joystick(1);
    btn6 = new JoystickButton(joystickOperator,8);
    btn5 = new JoystickButton(joystickOperator,4);
    btn3 = new JoystickButton(joystickOperator,6);
    btn8 = new JoystickButton(joystickOperator, 5);
    btn7 = new JoystickButton(joystickOperator, 7);
    
    consoleOperator = new Joystick(2);
    consoleA = new JoystickButton(consoleOperator,3); 
    consoleD = new JoystickButton(consoleOperator,2);
    consoleE = new JoystickButton(consoleOperator,1);
    consoleB = new JoystickButton(consoleOperator,5);
    consoleF = new JoystickButton(consoleOperator,4);

    
    //This connects our buttons to the commands we want
    y.whileHeld(new DriveByVision());
    a.whenPressed(new SwitchCameras());
    b.whenPressed(new PanelArm());
    x.whenPressed(new VacuumMaker());
    rb.whileHeld(new ShootCargo());
    lb.whileHeld(new CargoCollector());

    btn6.whenPressed(new SwitchCompressor());
    btn5.whenPressed(new SwitchAllSolenoids()); 
    btn3.whenPressed(new SwitchFrontSolenoids());
    btn8.whenPressed(new SwitchBackSolenoids()); //SwitchBackSolenoids
    btn7.whileHeld(new ActivateClimbWheel());

    consoleA.whenPressed(new MoveGripperInside());
    consoleD.whenPressed(new MoveGripperToMid());
    consoleE.whenPressed(new MoveGripperToFloor());
    consoleB.whenPressed(new AutoElevatorToCatchPanel());
    consoleF.whenPressed(new AutoElevatorToCatchCargo()); 
    }



  
}
