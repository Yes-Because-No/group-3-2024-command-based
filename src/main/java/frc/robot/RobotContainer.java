// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//command based imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import subsystems from the subdirectory subsystems
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.Constants.ArmConstants.PID.setpoints;

public class RobotContainer {
  //get controller up and running
  private CommandXboxController controller = new CommandXboxController(0);

  //instantiate all of your subsystems
  private Arm arm = new Arm();
  private Drivetrain drive = new Drivetrain();
  private Intake intake = new Intake();

  //create a trigger (will call bound commands when arm.getPidOn() returns true)
  private Trigger PIDEnabled = new Trigger(arm::getPidOn);
  
  //constructor will add all bindings below
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.rightBumper().whileTrue(intake.intake());//when right bumper pressed, intake
    controller.leftBumper().whileTrue(intake.outtake());//when left bumper pressed, outtake
    //when neither pressed, stop moving intake motor
    controller.rightBumper().or(controller.leftBumper()).onFalse(intake.stopIntake());
    
    //when you have PID, make the arm rotate to goal when not explicitly in use
    PIDEnabled
      .whileTrue(new InstantCommand(()->{
        arm.setDefaultCommand(arm.rotateArmToGoalCommand());
     })) // otherwise, stop that, so you can actually control it manually
      .whileFalse(new InstantCommand(()->{
        arm.removeDefaultCommand();
      }));

    //if PID is on and you hit A, move arm to ground
    controller.a().and(PIDEnabled).whileTrue(arm.rotateArmToNewGoalCommand(setpoints.GROUND));

    //if PID is on and you hit B, move arm to the high scoring zone
    controller.b().and(PIDEnabled).whileTrue(arm.rotateArmToNewGoalCommand(setpoints.HIGH));

    //this toggles between manual and PID arm control
    controller.x().whileTrue(arm.togglePid());

    //if PID is off and you are pressing the right trigger, move arm up
    controller.rightTrigger(0.1).and(PIDEnabled.negate()).whileTrue(arm.armManualControl(
      ()->controller.getRightTriggerAxis()*4.0
    ));

    //if PID is off and you are pressing the left trigger, move arm down
    controller.leftTrigger(0.1).and(PIDEnabled.negate()).whileTrue(arm.armManualControl(
      ()->controller.getLeftTriggerAxis()*-4.0
    ));

    //if neither is being pressed and PID is off, stop moving
    controller.rightTrigger(0.1).negate()
    .and(controller.leftTrigger(0.1).negate())
    .and(PIDEnabled.negate())
    .onTrue(arm.armManualControl(
      ()->0.0
    ));

    //arcade drive using controller inputs. The ()-> functions must return a double
    drive.setDefaultCommand(
      drive.arcadeDriveCommand(
        ()->controller.getLeftY()*-0.75, 
        ()->controller.getRightX()*0.75
      )
    );


  }

  /**
   * Move the arm up and then drive forwards slowly for 5 seconds
   * @return Command that moves the {@link #arm} up and {@link #drive drives} for 5 seconds at 30% speed
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      arm.rotateArmToNewGoalCommand(setpoints.HIGH),
      drive.arcadeDriveCommand(()->0.3, ()->0.0).repeatedly().withTimeout(5)
      );
  }
}
