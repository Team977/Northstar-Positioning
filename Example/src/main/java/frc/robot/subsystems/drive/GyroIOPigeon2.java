// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Libary.Odymetery;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(20, Constants.CANbusName);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Double> XAccleration = pigeon.getAccelerationX();
  private final StatusSignal<Double> YAccleration = pigeon.getAccelerationY();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(100.0);
    yawVelocity.setUpdateFrequency(100.0);
    XAccleration.setUpdateFrequency(100);
    YAccleration.setUpdateFrequency(100);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  public Odymetery.gyroTimeStep getTimeStamp() {
    Odymetery.gyroTimeStep timeStep = new Odymetery.gyroTimeStep();

    yaw.refresh();
    timeStep.rotation = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    timeStep.xAccl = XAccleration.getValueAsDouble();
    timeStep.yAccl = YAccleration.getValueAsDouble();
    timeStep.Time =
        (XAccleration.getTimestamp().getTime() + YAccleration.getTimestamp().getTime()) / 2;

    return timeStep;
  }
}
