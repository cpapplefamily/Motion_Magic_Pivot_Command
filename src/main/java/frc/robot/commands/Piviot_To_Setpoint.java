// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Piviot_MM;

public class Piviot_To_Setpoint extends CommandBase {
  private final Piviot_MM m_Piviot;
  private double m_setpoint;
 
  /** Creates a new Piviot_To_Setpoint. */
  public Piviot_To_Setpoint(double deg, Piviot_MM subsystem) {
    m_setpoint = deg;
    m_Piviot = subsystem;
    addRequirements(m_Piviot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Piviot.my_motionMagic_Run(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
      
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Piviot.my_get_PositionLock(m_setpoint);
  }


 

}
