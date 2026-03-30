// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.Alinhamento;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private Command currentDriveCommand;
 
  public static double varX=0;
  public static double varY=0;
  public boolean zeroTag;

  public static double tempoDeAjuste=0.0;

  private final Limelight LimelightFront = new Limelight();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final  static       CommandXboxController driverXbox = new CommandXboxController(0);
  public final  static       CommandXboxController codriverXbox = new CommandXboxController(1);
  public final static XboxController m_Xbox = new XboxController(0);
  public final static XboxController m_coXbox = new XboxController(1);

  SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  //private BooleanSupplier isAligning = () -> driverXbox.getAButton();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
  //                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
  //                                                                driverXbox.getHID()::getYButtonPressed,
  //                                                                driverXbox.getHID()::getAButtonPressed,
  //                                                                driverXbox.getHID()::getXButtonPressed,
  //                                                                driverXbox.getHID()::getBButtonPressed,
  //                                                                driverXbox.getHID()::getLeftBumperPressed,
  //                                                                driverXbox.getHID()::getBackButtonPressed,
  //                                                                driverXbox.getHID()::getRightBumperPressed);

  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND)*driverXbox.getRawAxis(3),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND)*driverXbox.getRawAxis(3),
      () -> driverXbox.getRightX() * -1);


  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  Command Alinhamento = new AbsoluteDriveAdv(drivebase,
    () -> TyGain(),
    () -> TxGain(),
    () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    codriverXbox.getHID()::getYButtonPressed,
    codriverXbox.getHID()::getAButtonPressed,
    codriverXbox.getHID()::getXButtonPressed,
    codriverXbox.getHID()::getBButtonPressed);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("AUTO1", drivebase.getAutonomousCommand("PROCESSOR_1"));

    //autoChooser.addOption("P1", drivebase.getAutonomousCommand("P_P1_1"));
    // autoChooser.addOption("Path 2", path2Command);
    
    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // if (DriverStation.isTest())
    // {
    //   driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
    //   driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    //   driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    //   driverXbox.leftBumper().onTrue(Commands.none());
    //   driverXbox.rightBumper().onTrue(Commands.none());

      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());


      BooleanSupplier isRightTriggerPressed = () -> driverXbox.getRightTriggerAxis() > 0.2;
      BooleanSupplier isRightTriggerRelease = () ->  driverXbox.getRightTriggerAxis() > 0.01 && driverXbox.getRightTriggerAxis() <= 0.2;
      activateCommandOnCondition(isRightTriggerPressed, driveFieldOrientedAnglularVelocity);
      activateCommandOnCondition(isRightTriggerRelease, Alinhamento);

      codriverXbox.a().whileTrue(drivebase.driveToPose(new Pose2d(5.36, 1.2, new Rotation2d(0))));
      codriverXbox.b().onTrue(Commands.runOnce(()-> drivebase.resetOdometry(new Pose2d(varX, varY, new Rotation2d(0)))));


      /* ERROOOOO */
      // Timer timer = new Timer();
      // timer.start();
      // new Thread(() -> {
      //     while (true) {
      //         if (timer.advanceIfElapsed(0.5)) { // 500ms = 0.5s
      //             if(varX>0 && varY>0 && driverXbox.povUp().getAsBoolean()==false && zeroTag==false) {
      //               drivebase.resetOdometry(new Pose2d(varX, varY, new Rotation2d()));
      //               // toggle=!toggle;
      //               // SmartDashboard.putBoolean("Toggle ", toggle);
      //             }
      //         }
      //     }
      // }).start();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    // return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  Double TxGain(){
    double kP = 0.04;

    if(LimelightFront.getTx()>20) kP=0.043;
    if(LimelightFront.getTx()>10 && LimelightFront.getTx()<=20) kP=0.035;
    if(LimelightFront.getTx()<=10) kP = 0.03;

    if(LimelightFront.getTx()==0) kP=0;

    return m_coXbox.getAButton() ? -kP * LimelightFront.getTx(): 0;
  }
  Double TyGain(){
    double kP = 0.02;
    if(LimelightFront.getTy()>12) kP=0.04;
    if(LimelightFront.getTy()>7 && LimelightFront.getTx()<=12) kP=0.03;
    if(LimelightFront.getTy()<=7) kP = 0.02;

    if(LimelightFront.getTy()==0) kP=0;

    return m_coXbox.getAButton() ? -kP * LimelightFront.getTy(): 0;
  }

  // public BooleanSupplier rightTrigger(double range) {
  //   BooleanSupplier Value;
  //   if(m_Xbox.getRightTriggerAxis() > range) Value = () -> true;
  //   else Value = () -> false;
  //   return Value;
  // }

  private void activateCommandOnCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(Commands.run(() -> {
        if (drivebase.getDefaultCommand() != null) {
            drivebase.getDefaultCommand().cancel(); // Interrompe o comando padrão atual
        }
        drivebase.setDefaultCommand(command); // Define o novo comando padrão
        drivebase.getDefaultCommand().schedule(); // Força a execução do novo comando
    }));
}
}
