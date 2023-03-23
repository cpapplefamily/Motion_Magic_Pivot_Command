// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Piviot_MM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SampleSequentialCommandGroup extends SequentialCommandGroup {
  /** Creates a new SampleSequentialCommandGroup. */
  public SampleSequentialCommandGroup(Piviot_MM m_pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Piviot_To_Setpoint(90, m_pivot).withTimeout(2),   //These will all run for either 2 seconds or untill the isFinished returns true.
      new WaitCommand(2),                                   //This will pause for 2 seconds
      new Piviot_To_Setpoint(0, m_pivot).withTimeout(2),
      new WaitCommand(2),
      new Piviot_To_Setpoint(90, m_pivot).withTimeout(2),
      new WaitCommand(2),
      new Piviot_To_Setpoint(0, m_pivot).withTimeout(2),
      new WaitCommand(2),
      new Piviot_To_Setpoint(90, m_pivot).withTimeout(2)
    );
  }
}
