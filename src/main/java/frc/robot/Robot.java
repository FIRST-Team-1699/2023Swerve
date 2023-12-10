// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Drive.DriveState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private XboxController controller = new XboxController(ControllerConstants.kDriverPort);
  private Drive swerve = new Drive(controller);
  private ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
  private Trajectory testTraj; 
  private FollowTrajectory action;

  @Override
  public void robotInit() {
    interiorWaypoints.add(new Translation2d(1, 1));
    testTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(), 
    interiorWaypoints,
    new Pose2d(new Translation2d(1, 2), Rotation2d.fromDegrees(90)),
    new TrajectoryConfig(SwerveConstants.kMaxSpeed, SwerveConstants.kMaxSpeed/2));
    action = new FollowTrajectory(testTraj, swerve);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    action.start();
  }

  @Override
  public void autonomousPeriodic() {
    if(action.isFinished()) {
      action.finish();
    } else {
      action.run();
    }
  }

  @Override
  public void teleopInit() {
    swerve.setState(DriveState.TELEOP_DRIVE);
  }

  @Override
  public void teleopPeriodic() {
    if(controller.getXButton()) {
      swerve.setState(DriveState.LOCK);
    } else {
      swerve.setState(DriveState.TELEOP_DRIVE);
    }
    swerve.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
