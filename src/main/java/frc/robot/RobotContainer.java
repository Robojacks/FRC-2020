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
import frc.robot.shooter.Plucker;
import frc.robot.vision.AimTarget;
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
  private XboxController xbox = new XboxController(Constants.kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Limelight Subsystem
  private final Limelight limelight = new Limelight();

  private final SenseColor colorSense = new SenseColor();

  private final Spinner spinner = new Spinner(colorSense);

  private ChangePosition goalMover = new ChangePosition();

  private final Lift lift = new Lift();

  private final Shooter shooter = new Shooter(goalMover, limelight);

  private final Conveyor conveyor = new Conveyor(goalMover, shooter);

  private final Plucker plucker = new Plucker(goalMover, shooter);

  private final Gears gears = new Gears();

  // Update PID values
  private final Update update = new Update(colorSense, shooter, spinner);

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().tankDrive(
      drivePercentLimit * xbox.getRawAxis(Axis.kLeftY.value), 
      drivePercentLimit * xbox.getRawAxis(Axis.kRightY.value)
      ), 
    rDrive
  );
  
  private Command moveArmOneAxis = new RunCommand(
    () -> lift.moveOneAxis(xbox.getRawAxis(Axis.kLeftTrigger.value)), lift);
  
  private Command moveArm = new RunCommand(
    () -> lift.move(xbox.getRawAxis(Axis.kRightTrigger.value) - xbox.getRawAxis(Axis.kLeftTrigger.value)), lift);
  
  private Command moveSpinner = new RunCommand(() -> 
    spinner.move(xbox.getRawAxis(Axis.kRightTrigger.value)), spinner);
  
  // Autonomous 
  private Command shootThenGo = new InstantCommand(() -> goalMover.collectPose(), goalMover)
    .andThen(new WaitCommand(.25)) 
    .andThen(() -> goalMover.shootPose(), goalMover)
    .andThen(() -> shooter.setSpeedVolts(intakeVolts, shooterVolts), shooter)
    .andThen(() -> conveyor.setSpeedHighGoal(conveyorVolts))
    .andThen(() -> plucker.setSpeedHighGoal(inPluckerVolts, outPluckerVolts))
    .andThen(new WaitCommand(4 + shooterRampUpTime))
    .andThen(() -> shooter.setSpeedVolts(0, 0), shooter)
    .andThen(() -> conveyor.setSpeedHighGoal(0))
    .andThen(() -> plucker.setSpeedHighGoal(0, 0))
    .andThen(() -> rDrive.getDifferentialDrive().tankDrive(-0.1, -0.1), rDrive) 
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

    // Shoot or intake with voltage, aiming for low goal
    new JoystickButton(xbox, Button.kBumperLeft.value)
    .whenPressed(() -> shooter.toggleSpeedVolts(intakeVolts, shooterVolts), shooter)
    .whenPressed(() -> conveyor.toggleSpeedLowGoal(conveyorVolts), shooter)
    .whenPressed(() -> plucker.toggleSpeedLowGoal(inPluckerVolts, outPluckerVolts), plucker);
    
    // Shoot or intake with set velocity, specifically for high goal
    new JoystickButton(xbox, Button.kB.value)
    .whileHeld(() -> plucker.setSpeedLowGoal(inPluckerVolts, outPluckerVolts), plucker)
    .whenReleased(() -> plucker.setSpeedLowGoal(0, 0), plucker);
    
    // Switches arm modes from up to down
    new JoystickButton(xbox, Button.kY.value)
    .whenPressed(() -> lift.switchMovement(), lift);

    // Vision correction
    new JoystickButton(xbox, Button.kX.value)
    .whenPressed(new AimTarget(limelight, rDrive))
    .whenReleased(() -> shooter.toggleRelativeSpeedSpark(intakeRPM, shooterRPM), shooter )
    .whenReleased(() -> conveyor.toggleSpeedHighGoal(conveyorVolts),conveyor)
    .whenReleased(() -> plucker.toggleSpeedHighGoal(inPluckerVolts, outPluckerVolts),plucker);
  

    // Spins to selected color
    new JoystickButton(xbox, Button.kStart.value)
    .whileHeld(() -> spinner.toSelectedColor
    (DriverStation.getInstance().getGameSpecificMessage()), spinner);

    // Spin number of rotations
    new JoystickButton(xbox, Button.kBack.value)
    .whenPressed(() -> spinner.setCountColor(), spinner)
    .whileHeld(() -> spinner.toSelectedColorSwitches(), spinner)
    .whenReleased(() -> spinner.changeMaxSwitches(4), spinner)
    .whenReleased(() -> spinner.move(0), spinner);

    // Switch Gears
    new JoystickButton(xbox, Button.kBumperRight.value)
    .whenPressed(() -> gears.switchGears(), gears);
    
  }

  public void init(){
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();

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
