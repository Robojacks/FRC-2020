/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.shooter.ChangePosition;
import frc.robot.shooter.Conveyor;
import frc.robot.vision.FollowTarget;
import frc.robot.vision.Limelight;
import frc.robot.wheel.SenseColor;
import frc.robot.wheel.Spinner;
import frc.robot.climber.Lift;
import frc.robot.drive.Gears;
import frc.robot.drive.RevDrivetrain;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Arrays;

import static frc.robot.Constants.*;
import static frc.robot.Gains.Ramsete;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive Controller
  XboxController xbox = new XboxController(Constants.kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Limelight Subsystem
  private final Limelight limelight = new Limelight();

  private final SenseColor colorSense = new SenseColor();

  private final Spinner spinner = new Spinner(colorSense);

  private final ChangePosition goalMover = new ChangePosition();

  private final Lift lift = new Lift();

  private final Shooter shooter = new Shooter(goalMover);

  private final Conveyor conveyor = new Conveyor(goalMover, shooter);

  private final Gears gears = new Gears();

  // Update PID values
  private final Update update = new Update(colorSense, shooter);

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().tankDrive(
      drivePercentLimit * xbox.getRawAxis(1), drivePercentLimit * xbox.getRawAxis(5), false), rDrive);
  
  private Command moveArmOneAxis = new RunCommand(
    () -> lift.moveOneAxis(xbox.getRawAxis(Axis.kLeftTrigger.value)), lift);
  
  private Command moveArm = new RunCommand(
    () -> lift.move(xbox.getRawAxis(Axis.kRightTrigger.value) - xbox.getRawAxis(Axis.kLeftTrigger.value)), lift);
  
  private Command moveSpinner = new RunCommand(() -> 
    spinner.move(xbox.getRawAxis(Axis.kRightTrigger.value)), spinner);
  
  // Autonomous 
  
  private Command shootThenGo = new InstantCommand(
    () -> shooter.setSpeedVolts(intakeVolts, shooterVolts), shooter)
    .andThen(() -> goalMover.collectPose(), goalMover)
    .andThen(new WaitCommand(.25)) 
    .andThen(() -> goalMover.shootPose(), goalMover)
    .andThen(new WaitCommand(shooterRampUpTime))
    .andThen(() -> conveyor.setSpeed(conveyorVolts))
    .andThen(new WaitCommand(4))
    .andThen(() -> shooter.setSpeedVolts(0, 0), shooter)
    .andThen(() -> conveyor.setSpeed(0))
    .andThen(() -> rDrive.getDifferentialDrive().tankDrive(-0.2, -0.2), rDrive) 
    .andThen(new WaitCommand(1.5))
    .andThen(()-> rDrive.getDifferentialDrive().tankDrive(0, 0), rDrive);
  
  private RamseteCommand rbase = new RamseteCommand(
    getMovingTrajectory(), 
    rDrive::getPose, 
    new RamseteController(Ramsete.kb, Ramsete.kzeta), 
    rDrive.getFeedforward(), 
    rDrive.getKinematics(), 
    rDrive::getSpeeds, 
    rDrive.getLeftDrivePID(), 
    rDrive.getRightDrivePID(), 
    rDrive::setOutputVolts, 
    rDrive);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    rDrive.setDefaultCommand(manualDrive);
    lift.setDefaultCommand(moveArmOneAxis);
    spinner.setDefaultCommand(moveSpinner);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Switch position between shooting and intake
    new JoystickButton(xbox, Button.kA.value)
    .whenPressed(() -> goalMover.swapHeight(), goalMover);

    // Shoot or intake with voltage
    new JoystickButton(xbox, Button.kBumperLeft.value)
    .whenPressed(() -> shooter.toggleSpeedVolts(intakeVolts, shooterVolts), shooter)
    .whenPressed(() -> conveyor.toggleSpeed(conveyorVolts));
    
    
    // Shoot or intake with set velocity
    new JoystickButton(xbox, Button.kB.value)
    .whenPressed(() -> shooter.setSpeedTalon(intakeRPM, shooterRPM), shooter)
    .whenReleased(() -> shooter.setSpeedTalon(0, 0), shooter);
    

    // Switches arm modes from up to down
    new JoystickButton(xbox, Button.kY.value)
    .whenPressed(() -> lift.switchMovement(), lift);

    // Vision correction
    new JoystickButton(xbox, Button.kX.value)
    .whileHeld(new FollowTarget(limelight, rDrive));
  
    // Spins to selected color
    new JoystickButton(xbox, Button.kStart.value)
    .whileHeld(() -> spinner.toSelectedColor
    (DriverStation.getInstance().getGameSpecificMessage()), spinner);

    // Spin number of rotations
    new JoystickButton(xbox, Button.kBack.value)
    .whileHeld(() -> spinner.toSelectedColorSwitches(), spinner);
    
    // Switch Gears
    new JoystickButton(xbox, Button.kBumperRight.value)
    .whenPressed(() -> gears.switchGears(), gears);
    
  }

  public void periodic() {
    update.periodic();
  }

  private Trajectory getMovingTrajectory() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(Update.getStartingPose(), new Pose2d(1.0, 0, new Rotation2d()),
        new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))), 
      new TrajectoryConfig(MaxSafeVelocityMeters, MaxSafeAccelerationMeters)
    );
    
    return trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shootThenGo;
  }
}
