// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.team1699.Constants.ControllerConstants;
import frc.robot.team1699.lib.auto.modes.AutoMode;
import frc.robot.team1699.lib.auto.modes.DriveForward;
import frc.robot.team1699.lib.auto.trajectory.ProcessedTrajectory;
import frc.robot.team1699.subsystems.Drive;
import frc.robot.team1699.subsystems.Drive.DriveState;

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

  private AutoMode auto;
  private Trajectory trajectory;

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    trajectory = new ProcessedTrajectory(Filesystem.getDeployDirectory().toPath().resolve("output/DriveFiveFeet.wpilib.json")).getTrajectory();
    auto = new DriveForward(trajectory, swerve);
    auto.initialize();
  }

  @Override
  public void autonomousPeriodic() {
    if(auto.isFinished()) {
      auto.finish();
    } else {
      auto.run();
    }
    swerve.update();
  }

  @Override
  public void teleopInit() {
    swerve.setWantedState(DriveState.TELEOP_DRIVE);
  }

  @Override
  public void teleopPeriodic() {
    if(controller.getXButton()) {
      swerve.setWantedState(DriveState.LOCK);
    } else {
      swerve.setWantedState(DriveState.TELEOP_DRIVE);
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
