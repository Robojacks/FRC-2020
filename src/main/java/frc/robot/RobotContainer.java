/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.turret.Turret;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Limelight;
import frc.robot.wheel.SenseColor;
import frc.robot.wheel.Spinner;
import frc.robot.climber.Arm;
import frc.robot.drive.RevDrivetrain;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.*;

import java.util.Arrays;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive Controller
  XboxController xbox = new XboxController(Constants.kControllerPort);
  
  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Limelight Subsystem
  private final Limelight limelight = new Limelight();

  private final SenseColor colorSense = new SenseColor();

  private final Spinner spinner = new Spinner();

  private final Turret goalMover = new Turret();

  private final Arm arm = new Arm();

  private final Shooter shooter = new Shooter();

  // Drive with Controller 
  Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().tankDrive(xbox.getRawAxis(5), xbox.getRawAxis(1)), rDrive);
   Command shootAndGo = new ProxyScheduleCommand(new FollowTarget()) 
   .andThen(new WaitCommand(2)) 
   .andThen(()-> shooter.setVelocity(500, 50))
   .andThen(()-> rDrive.getDifferentialDrive().tankDrive(0.2, 0.2), rDrive) 
   .andThen(new WaitCommand(2))
   .andThen(()-> rDrive.getDifferentialDrive().tankDrive(0, 0), rDrive);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    manualDrive.schedule();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(xbox, Button.kA.value)
    .whenPressed(() -> goalMover.swapHeight(), goalMover);

    new JoystickButton(xbox, Button.kB.value)
    .whenPressed(() -> shooter.setVelocity(shooterRPM, shooterError))
    .whenReleased(() -> shooter.setVelocity(0, 0));

    new JoystickButton(xbox, Button.kY.value)
    .whenPressed(() -> arm.switchState(), arm);

   // new JoystickButton(xbox, Button.kX.value)
    //.whileHeld(() -> limelight.getTargetDistanceMeasured(cameraToTargetHeight, cameraAngle));
   
    new JoystickButton(xbox, Button.kX.value)
    .whenPressed(() -> spinner.toSelectedColor(), spinner);
  }

  public void periodic() {
    SmartDashboard.putNumber("Raw Color Value", colorSense.getRawColor());
    SmartDashboard.putNumber("Proximity", colorSense.getProximity());
    SmartDashboard.putString("Detected Color", colorSense.getColorString());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final TrajectoryConfig config = new TrajectoryConfig(
      MaxSafeVelocityMeters, MaxSafeAccelerationMeters);
    
    final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(Update.getStartingPose(), new Pose2d(1.0, 0, new Rotation2d()),
        new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))), config);
      
      final RamseteCommand rbase = new RamseteCommand(
      trajectory, 
      rDrive::getPose, 
      new RamseteController(Ramsete.kb, Ramsete.kzeta), 
      rDrive.getFeedforward(), 
      rDrive.getKinematics(), 
      rDrive::getSpeeds, 
      rDrive.getLeftDrivePID(), 
      rDrive.getRightDrivePID(), 
      rDrive::setOutputVolts, 
      rDrive);
    
    return rbase.andThen(() -> rDrive.setOutputVolts(0, 0));
  }
}