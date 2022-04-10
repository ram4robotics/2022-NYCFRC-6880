// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainTeleOp extends CommandBase {
  private final Supplier<Double> m_leftY_Supplier;
  private final Supplier<Double> m_rightXSupplier;
  private final DriveTrain m_drive;
  /** Creates a new DriveTrainTeleOp. */
  public DriveTrainTeleOp(Supplier<Double> leftY, Supplier<Double> rightX, DriveTrain drvSys) {
    m_drive = drvSys;
    m_leftY_Supplier = leftY;
    m_rightXSupplier = rightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drvSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = -m_leftY_Supplier.get();
    double rightX = m_rightXSupplier.get();

    // double newLeftY = Math.pow(leftY, 2) * Math.signum(leftY);
    double newLeftY = leftY;
    double newRightX = rightX / 2;
    m_drive.arcadeDrive(newLeftY, newRightX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
