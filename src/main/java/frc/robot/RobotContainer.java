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

  private static final double slowFactor = 0.5;

  public RobotContainer() {
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
                    if (primary.b().getAsBoolean()) {
                        ySpeed *= slowFactor;
                        xSpeed *= slowFactor;
                        rot *= slowFactor;
                    }
    
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

    primary.rightTrigger().whileTrue(new RunCommand(() -> drivetrain.driveAtTarget(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    primary.rightBumper().whileTrue(new RunCommand(() -> drivetrain.lockFront(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    primary.leftBumper().whileTrue(new RunCommand(() -> drivetrain.lockBack(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));

    //primary.a().toggleOnTrue(new RunCommand(() -> shooter.setHoodAngle(50), shooter));

    //primary.leftTrigger().whileTrue(new RunCommand(() -> shooterSubsystem.shoot(), shooterSubsystem));

    //primary.y().toggleOnTrue(new RunCommand(() -> climbSubsystem.climbForward(), climbSubsystem)).onFalse(new RunCommand(() -> climbSubsystem.climbReverse(), climbSubsystem));

    // primary.y().onTrue(new InstantCommand(() -> climbSubsystem.climbForward(), climbSubsystem));

    // primary.a().onTrue(new InstantCommand(() -> climbSubsystem.climbReverse(), climbSubsystem));

// Sadece onTrue kullanıyoruz. Bas-çek yapınca toggle çalışır.
primary.povUp().onTrue(new InstantCommand(() -> climbSubsystem.climbToggle(), climbSubsystem));
primary.povDown().onTrue(new InstantCommand(() -> intakeSubsystem.closeIntake(), intakeSubsystem)).onFalse(new InstantCommand(() -> intakeSubsystem.openIntake(), intakeSubsystem));

    primary.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                  new RunCommand(() -> {shooter.shoot(); shooter.enablePID();}, shooter),
                  new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
                    
                  )
                )
                .onFalse(new InstantCommand(() -> {
                  shooter.stopShooter();
                  shooter.disablePID();
                  feederSubsystem.stopMotors();
                })); 

    /*primary.leftTrigger().whileTrue(
        new ParallelCommandGroup(
            // Sadece kontrolü aktif et, gerisini subsystem'in periodic'i halleder
            new RunCommand(() -> shooter.enableAutoControl(), shooter), 
            
            // Hazır olana kadar bekle ve besleyiciyi çalıştır
            new WaitUntilCommand(() -> shooter.isReady())
                .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
        )
    ).onFalse(new InstantCommand(() -> {
        shooter.disableAutoControl(); // Her şeyi subsystem içinden durdurur
        feederSubsystem.stopMotors();
    }));*/

    //primary.leftTrigger().whileTrue(new InstantCommand(() -> shooter.enablePID(), shooter)).onFalse(new InstantCommand(() -> shooter.disablePID(), shooter));
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
  



