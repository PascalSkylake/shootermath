// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  static Shooter shooter = new Shooter();
  static double height = 0.5;
  static double v0 = 20;

  public static void main(String... args) {
    calcAllIntersects();
  }

  public static void calcAllIntersects() {
    // Vector<N4> in = VecBuilder.fill(0, 0.5, 20 * Math.cos(Math.PI/6), 20 *
    // Math.sin(Math.PI/6));
    // shooter.propagateState(in, 4, 100);
    // double theta = 1;

    // Translation2d shooterExit = shooter.shooterExitRobotRelative(theta);
    // Vector<N4> in = VecBuilder.fill(shooterExit.getX(), shooterExit.getY(), 20 * Math.cos(theta), 20 * Math.sin(theta));
    // shooter.propagateState(in, 3, 50);

    for (double theta = 0; theta < Math.PI / 2; theta += 0.002) {
      Translation2d shooterExit = shooter.shooterExitRobotRelative(theta);
      Vector<N4> in = VecBuilder.fill(shooterExit.getX(), shooterExit.getY(), 20 * Math.cos(theta), 20 * Math.sin(theta));
      Optional<Double> a = shooter.calcTrajectoryIntersectWithSpeakerHeightPlane(in, 0.0001);

      if (a.isPresent()) {
        //System.out.println("" + theta + "," + a.get());
      }
    }
  }



}
