// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.MultivariateVectorFunction;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.OptimizationData;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.MultivariateOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.apache.commons.math3.optim.nonlinear.vector.MultivariateVectorOptimizer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  static Shooter shooter = new Shooter();
  static double height = 0.5;
  static double v0 = 20;
  static double x = 0;
  static double y = 0;
  static double z = 0;
  static double vx = 0;
  static double vy = 0;

  public static void main(String... args) {
    try {
    Thread.sleep(500);
    } catch (Exception exception) {

    }
    long start = System.nanoTime();
    //double[] optimal = optimizeShooterOrientation(0.25, Math.PI / 2, 0.05, 0, 3, 2);
    //System.out.println(optimal[0] + "," + optimal[1] + "," + optimal[2]);

    //Translation2d shooterExit = shooter.shooterExitRobotRelative(optimal[0]);
    // Vector<N6> in = VecBuilder.fill(Main.x, Main.y, Main.z,
    //         Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.cos(optimal[1])),
    //         Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.sin(optimal[1])), Main.v0 * Math.sin(Math.PI / 2 - optimal[0]));
    // ArrayList<Vector<N8>> trajectory = shooter.propagateWholeTrajectory3d(in, optimal[2], 100);

    Vector<N6> in = VecBuilder.fill(Main.x, Main.y, Main.z,
            Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - 1) * Math.cos(1)),
            Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - 1) * Math.sin(1)), Main.v0 * Math.sin(Math.PI / 2 - 1));
        //ArrayList<Vector<N8>> trajectory = shooter.propagateWholeTrajectory3d(in, 1, 20);
        shooter.propagateWholeTrajectory3d(in, 1, 20);
    long end = System.nanoTime();

    System.out.println((end - start) / 1000000.0);

    //Vector<N8> v = trajectory.get(trajectory.size() - 1);

    // for (Vector<N8> v : trajectory) {
    //   System.out.println(v.get(0, 0) + "," + v.get(1,0) + "," + v.get(2, 0) + "," + v.get(5, 0) + "," + v.get(6, 0));
    // }
    //System.out.println(v.get(0, 0) + "," + v.get(1,0) + "," + v.get(2, 0) + "," + v.get(5, 0) + "," + v.get(6, 0));
  }



  private static double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime, double targetX, double targetY, double targetZ) {

    MultivariateFunction f = new MultivariateFunction() {
      @Override
      public double value(double[] point) {
        // calculate trajectory given theta phi and t

        Vector<N6> in = VecBuilder.fill(Main.x, Main.y, Main.z,
            Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - point[0]) * Math.cos(point[1])),
            Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - point[0]) * Math.sin(point[1])), Main.v0 * Math.sin(Math.PI / 2 - point[0]));
        ArrayList<Vector<N8>> trajectory = shooter.propagateWholeTrajectory3d(in, point[2], 20);
        Vector<N8> finalPosition = trajectory.get(trajectory.size() - 1);

        Translation3d target = new Translation3d(targetX, targetY, targetZ);
        Translation3d finalPose = new Translation3d(finalPosition.get(0, 0), finalPosition.get(1, 0), finalPosition.get(2, 0));


        //System.out.println(finalPose);
        return target.getDistance(finalPose);
      }

    };

    ObjectiveFunction objective = new ObjectiveFunction(f);



    PowellOptimizer optimizer = new PowellOptimizer(0.000001, 0.000001);
    double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
    InitialGuess guess = new InitialGuess(initialGuess);
    MaxIter maxIter = new MaxIter(500);
    // look at my lawyer dawg I'm goin to jail!!!
    MaxEval maxEval = new MaxEval(500);
    GoalType goalType = GoalType.MINIMIZE;

    double[] optimalParams = optimizer.optimize(new OptimizationData[] {objective, guess, maxIter, maxEval, goalType}).getPoint();

    return optimalParams;
  }

}
