package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N8;

public class Shooter {
  Function<Vector<N6>, Vector<N6>> projectileEquation;
  Function<Vector<N8>, Vector<N8>> projectileEquation3d;

  private final double SPEAKER_HEIGHT = 1.9812;
  private final double DRAG_COEFFICIENT = 0.5;
  private final double AIR_DENSITY = 1.225;
  private final double CROSS_SECTIONAL_AREA = 0.018;
  private final double NOTE_MASS = 0.2353;
  private final double MU = (DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA) / (2 * NOTE_MASS);

  private final double SHOOTER_PIVOT_TO_END = 0.37516;
  private final Translation2d SHOOTER_PIVOT_ROBOT_REL = new Translation2d(-0.2757, 0.5972);

  public Shooter() {

    projectileEquation = (Vector<N6> x) -> {
      double vx = x.get(2, 0);
      double vy = x.get(3, 0);
      double v = Math.sqrt((vx * vx) + (vy * vy));
      double ax = -MU * vx * v;
      double ay = -9.8 - (MU * vy * v);

      Vector<N6> out = VecBuilder.fill(vx, vy, ax, ay, 0, 0);

      return out;
    };

    projectileEquation3d = (Vector<N8> x) -> {
      double vx = x.get(3, 0);
      double vy = x.get(4, 0);
      double vz = x.get(5, 0);
      double v = Math.sqrt((vx * vx) + (vy * vy) + (vz * vz));
      double ax = -MU * vx * v;
      double ay = -MU * vy * v;
      double az = -9.8 - (MU * vz * v);

      Vector<N8> out = VecBuilder.fill(vx, vy, vz, ax, ay, az, 0, 0);

      return out;
    };
  }

  public <R extends Num> Vector<R> rkFour(Vector<R> x, Function<Vector<R>, Vector<R>> f) {
    double h = x.get(x.getNumRows() - 1, 0);

    Vector<R> k_1 = f.apply(x);
    Vector<R> k_2 = f.apply(x.plus(k_1.times(h / 2.0)));
    Vector<R> k_3 = f.apply(x.plus(k_2.times(h / 2.0)));
    Vector<R> k_4 = f.apply(x.plus(k_3.times(h)));

    Vector<R> out = x.plus((k_1.plus(k_2.times(2)).plus(k_3.times(2)).plus(k_4)).times(h / 6.0));
    out.set(x.getNumRows() - 2, 0, x.get(x.getNumRows() - 2, 0) + h);
    return out;
  }

  public Vector<N6> propagateState(Vector<N4> x, double t, int intervals) {
    double dt = t / intervals;
    Vector<N6> state = VecBuilder.fill(x.get(0, 0), x.get(1, 0), x.get(2, 0), x.get(3, 0), dt, 0);
    for (int i = 0; i < intervals; i++) {
      state = rkFour(state, projectileEquation);
      System.out.println("" + (i * dt) + "," + state.get(0, 0) + "," + state.get(1, 0) + "," + state.get(2, 0) + ","
          + state.get(3, 0));

    }

    return state;
  }

  public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane(Vector<N4> k, double dt) {
    double x = k.get(0, 0);
    double y = k.get(1, 0);
    double vx = k.get(3, 0);
    double vy = k.get(4, 0);
    Vector<N6> state = VecBuilder.fill(x, y, vx, vy, 0, dt);

    while (state.get(3, 0) > 0) {

      state = rkFour(state, projectileEquation);

      if (state.get(1, 0) > SPEAKER_HEIGHT) {
        double timeAgo = (state.get(1, 0) - SPEAKER_HEIGHT) / state.get(3, 0);
        double intersectX = state.get(0, 0) - (timeAgo * state.get(2, 0));
        //System.out.println(state.get(4, 0) + "," + intersectX);
        return Optional.of(intersectX);
      }

    }

    return Optional.empty();
  }

  public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane3d(Vector<N6> k, double dt) {
    double x = k.get(0, 0);
    double y = k.get(1, 0);
    double z = k.get(2, 0);
    double vx = k.get(3, 0);
    double vy = k.get(4, 0);
    double vz = k.get(5, 0);

    Vector<N8> state = VecBuilder.fill(x, y, z, vx, vy, vz, 0, dt);

    while (state.get(5, 0) > 0) {

      state = rkFour(state, projectileEquation3d);
      // System.out.println(state.get(2, 0) + "," + state.get(5, 0));

      if (state.get(2, 0) > SPEAKER_HEIGHT) {
        double timeAgo = (state.get(2, 0) - SPEAKER_HEIGHT) / state.get(5, 0);
        double intersectX = state.get(0, 0) - (timeAgo * state.get(3, 0));
        double intersectY = state.get(1, 0) - (timeAgo * state.get(4, 0));
        System.out.println(state.get(6, 0) + "," + intersectX + "," + intersectY);
        return Optional.of(intersectX);
      }

    }

    return Optional.empty();
  }

  public ArrayList<Vector<N8>> propagateWholeTrajectory3d(Vector<N6> k, double t, int intervals) {
    double x = k.get(0, 0);
    double y = k.get(1, 0);
    double z = k.get(2, 0);
    double vx = k.get(3, 0);
    double vy = k.get(4, 0);
    double vz = k.get(5, 0);

    ArrayList<Vector<N8>> out = new ArrayList<>(intervals);
    double dt = t / intervals;

    Vector<N8> state = VecBuilder.fill(x, y, z, vx, vy, vz, 0, dt);

    for (int i = 0; i < intervals; i++) {
      state = rkFour(state, projectileEquation3d);
      out.add(state);
    }
    
    return out;
  }

  public Translation2d shooterExitRobotRelative(double theta) {
    double x = SHOOTER_PIVOT_TO_END * Math.cos(theta);
    double y = SHOOTER_PIVOT_TO_END * Math.sin(theta);

    return SHOOTER_PIVOT_ROBOT_REL.plus(new Translation2d(x, y));
  }

}
