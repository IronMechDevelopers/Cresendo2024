// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionModule extends SubsystemBase {
  /** Creates a new PowerDistributionModule. */

  private PowerDistribution m_pdp = new PowerDistribution(1, ModuleType.kRev);

  public PowerDistributionModule() {
  }

  @Override
  public void periodic() {
    // // // Get the voltage going into the PDP, in Volts.
    // // // The PDP returns the voltage in increments of 0.05 Volts.
    // double voltage = m_pdp.getVoltage();

    // // // Get the total current of all channels.
    // double totalCurrent = m_pdp.getTotalCurrent();

    // // // Get the total energy of all channels.
    // // // Energy is the power summed over time with units Joules.
    // double totalEnergy = m_pdp.getTotalEnergy();

    // // // Get the total power of all channels.
    // // // Power is the bus voltage multiplied by the current with the units Watts.
    // double totalPower = m_pdp.getTotalPower();

    // SmartDashboard.putNumber("Total Current", totalCurrent);
    // SmartDashboard.putNumber("Total Power", totalPower);
    // SmartDashboard.putNumber("Total Energy", totalEnergy);
    // SmartDashboard.putNumber("Voltage", voltage);
  }
}
