// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTestSubsystem;
import frc.robot.subsystems.Swerve.DriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final DriveTrain drivetrain = new DriveTrain();
  private final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain);
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivetrain);
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private  final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 

  private final ShooterTestSubsystem shooter = new ShooterTestSubsystem(drivetrain);

  public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

  //private static final double slowFactor = 0.5;


  public RobotContainer() {
    NamedCommands.registerCommand("Open Intake", new InstantCommand(() -> intakeSubsystem.openIntake(), intakeSubsystem));
    NamedCommands.registerCommand("Close Intake", new InstantCommand(() -> intakeSubsystem.closeIntake(), intakeSubsystem));
    NamedCommands.registerCommand("Open Climb", new InstantCommand(() -> climbSubsystem.climbForward(), climbSubsystem));
    NamedCommands.registerCommand("Close Climb", new InstantCommand(() -> climbSubsystem.climbReverse(), climbSubsystem));
    NamedCommands.registerCommand("Feed Shooter", new InstantCommand(() -> feederSubsystem.feedShooter(), feederSubsystem));
    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(
                  new RunCommand(() -> {shooter.shoot(); shooter.enablePID();}, shooter),
                  new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
                    
                  ));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
        // Varsayılan Sürüş Komutu: Robot her zaman joystick verilerini dinler
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
                // Joystick verilerini al (WPILib standartları için Y ters çevrilir)
                double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.driveDeadband);
                double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.driveDeadband);
                double rot = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);

                // B butonuna basılıyorsa hızı %50'ye düşür
                /*if (primary.b().getAsBoolean()) {
                    ySpeed *= slowFactor;
                    xSpeed *= slowFactor;
                    rot *= slowFactor;
                }*/

                // Sürüşü başlat (fieldRelative: true -> Saha odaklı sürüş)
                drivetrain.drive(ySpeed, xSpeed, rot, true);
            },
            drivetrain));

  }


  private void configureBindings() {
    // Start butonuna basıldığında Gyro'yu (ön yönü) o anki bakış yönüne göre sıfırla
    primary.start().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading()));
    primary.start().onTrue(Commands.runOnce(() -> questNav.zeroPose()));
    primary.start().onTrue(Commands.runOnce(() -> drivetrain.resetPoseToSelected()));

    // X butonuna basılı tutarken tekerlekleri X şeklinde kilitle (Defans modu)
    primary.x().whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

    primary.rightBumper().whileTrue(new RunCommand(() -> drivetrain.driveAtTarget(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    //primary.rightBumper().whileTrue(new RunCommand(() -> drivetrain.lockFront(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    primary.leftBumper().onTrue(new InstantCommand(() -> intakeSubsystem.intakeToggle(), intakeSubsystem));

    // Sadece onTrue kullanıyoruz. Bas-çek yapınca toggle çalışır.
    primary.povUp().onTrue(new InstantCommand(() -> climbSubsystem.climbToggle(), climbSubsystem));
    primary.povDown().onTrue(new InstantCommand(() -> intakeSubsystem.closeIntake(), intakeSubsystem)).onFalse(new InstantCommand(() -> intakeSubsystem.openIntake(), intakeSubsystem));

    primary.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                  new RunCommand(() -> {shooter.shoot(); shooter.enablePID();}, shooter),
                  new InstantCommand(() -> climbSubsystem.compressorDisable(), climbSubsystem),
                  new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
                    
                  )
                )
                .onFalse(new InstantCommand(() -> {
                  shooter.stopShooter();
                  shooter.disablePID();
                  feederSubsystem.stopMotors();
                }));

    primary.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.openIntake(), intakeSubsystem));
    primary.leftTrigger().whileTrue(new InstantCommand(() -> intakeSubsystem.runIntake(), intakeSubsystem));

    primary.b().onTrue(new InstantCommand(() -> climbSubsystem.compressorToggle(), climbSubsystem));

    primary.a().onTrue(new InstantCommand(() -> shooter.toggleShooter(), shooter));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
  



