/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utils;


import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.Driver;
/**
 * Add your docs here.
 */
public class Motion {
    public Notifier m_follower_notifier;
    public EncoderFollower m_left_follower;
    public EncoderFollower m_right_follower;
    
    

    public void Init() throws IOException
    {
        
        Trajectory left_trajectory = PathfinderFRC.getTrajectory(Constants.k_path_nameL);
        Trajectory right_trajectory = PathfinderFRC.getTrajectory(Constants.k_path_nameR);
        m_left_follower = new EncoderFollower(left_trajectory);
        m_right_follower = new EncoderFollower(right_trajectory);
        
        m_left_follower.configureEncoder(RobotMap.driverFrontLeft.getSelectedSensorPosition(), Constants.k_ticks_per_rev, Constants.k_wheel_diameter);
        m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.k_max_velocity, 0);

        m_right_follower.configureEncoder(-RobotMap.driverFrontRight.getSelectedSensorPosition(), Constants.k_ticks_per_rev, Constants.k_wheel_diameter);
        m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.k_max_velocity, 0);

        m_follower_notifier = new Notifier(this::followPath);
        m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);



    }
    
    public Runnable followPath()
    {
        return new Runnable(){
            @Override
            public void run() {
                if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
                    m_follower_notifier.stop();
                  } else {
                    double left_speed = m_left_follower.calculate(RobotMap.driverFrontLeft.getSelectedSensorPosition());
                    double right_speed = m_right_follower.calculate(RobotMap.driverFrontRight.getSelectedSensorPosition());
                    double heading = Robot.Navx.getAngle();
                    double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
                    double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
                    double turn =  0.8 * (-1.0/80.0) * heading_difference;
                    Driver.getInstance().setSpeedLeft(left_speed + turn+0.5);
                    Driver.getInstance().setSpeedRight(right_speed + turn+0.5);
                  }
            }
        };
    }

    
}
