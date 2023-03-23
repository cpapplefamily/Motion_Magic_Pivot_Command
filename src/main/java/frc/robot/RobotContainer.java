// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Pivot_To_Setpoint;
import frc.robot.commands.Pivot_Percent;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivot_MM;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Pivot_MM m_pivot_MM = new Pivot_MM();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(m_pivot_MM);
    // Configure the trigger bindings
    SmartDashboard.putData("Arm to 90", new Pivot_To_Setpoint(90, m_pivot_MM));
    SmartDashboard.putData("Arm to 0", new Pivot_To_Setpoint(0, m_pivot_MM));
    SmartDashboard.putData("Arm percent", new Pivot_Percent(.2, m_pivot_MM));
    SmartDashboard.putData("Arm Reset", new InstantCommand(()-> m_pivot_MM.my_resetEncoder()));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

        //These do not hard OT the motor but set the current position equal to the soft stops
        //Option 1 monitor input dirctly
        new Trigger(m_pivot_MM::my_Forward_Limit).onTrue(new InstantCommand(()-> m_pivot_MM.my_SetEncoder()).ignoringDisable(true));
        //Option 2 use a hardware counter *needs extra code to reset
        new Trigger(m_pivot_MM::my_Revese_Limit_Counter).onTrue(new InstantCommand(()-> m_pivot_MM.my_resetEncoder()).ignoringDisable(true));

        //Any time a Forward command is iniated reset the reverse counter
        m_driverController.b().onTrue(Commands.parallel(new Pivot_To_Setpoint(90, m_pivot_MM),
                                                        new InstantCommand(()-> m_pivot_MM.my_Reset_RevCounter())));
        m_driverController.a().onTrue(new Pivot_To_Setpoint(1, m_pivot_MM));
        m_driverController.start().onTrue(new InstantCommand(()-> m_pivot_MM.my_resetEncoder()));
        
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  /**
   * This will remove any active Closed loop control on the Pivot_MM subsystem
   */
  public Command my_Disable_Pivot_MM(){
    return new InstantCommand(()-> m_pivot_MM.my_PercentOutput_Run(0),m_pivot_MM).ignoringDisable(true);
  }

  public Command my_Disable_All_MotionMagic(){
    return Commands.parallel(new InstantCommand(()-> m_pivot_MM.my_PercentOutput_Run(0),m_pivot_MM).ignoringDisable(true),
                              new WaitCommand(0) // Add each Subsystem here
                              );
  }
}
