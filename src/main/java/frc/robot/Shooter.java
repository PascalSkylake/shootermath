package frc.robot;

import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;



public class Shooter {
  Function<Vector<N5>, Vector<N5>> projectileEquation;

  private final double SPEAKER_HEIGHT = 1.9812;
  private final double DRAG_COEFFICIENT = 0.5;
  private final double AIR_DENSITY = 1.225;
  private final double CROSS_SECTIONAL_AREA = 0.018;
  private final double NOTE_MASS = 0.2353;
  private final double MU = (DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA) / (2 * NOTE_MASS);

  private final double SHOOTER_PIVOT_TO_END = 0.37516;
  private final Translation2d SHOOTER_PIVOT_ROBOT_REL = new Translation2d(-0.2757, 0.5972);


  public Shooter() {


    projectileEquation = (Vector<N5> x) -> {
      double[] in = x.getData();
      double vx = in[2];
      double vy = in[3];
      double v = Math.sqrt((vx * vx) + (vy * vy));
      double ax = -MU * vx * v;
      double ay = -9.8 - (MU * vy * v);

      Vector<N5> out = VecBuilder.fill(vx, vy, ax, ay, 0);

      return out;
    };
  }

  public Vector<N5> rkFour(Vector<N5> x) {
    double h = x.get(4, 0);

    Vector<N5> k_1 = projectileEquation.apply(x);
    Vector<N5> k_2 = projectileEquation.apply(x.plus(k_1.times(h / 2.0)));
    Vector<N5> k_3 = projectileEquation.apply(x.plus(k_2.times(h / 2.0)));
    Vector<N5> k_4 = projectileEquation.apply(x.plus(k_3.times(h)));
  
    Vector<N5> out = x.plus((k_1.plus(k_2.times(2)).plus(k_3.times(2)).plus(k_4)).times(h / 6.0));
    return out; 
  }

  public Vector<N5> propagateState(Vector<N4> x, double t, int intervals) {
    double dt = t / intervals;
    Vector<N5> state = VecBuilder.fill(x.get(0, 0), x.get(1, 0), x.get(2, 0), x.get(3, 0), dt);
    for (int i = 0; i < intervals; i++) {
      state = rkFour(state);
      System.out.println("" + (i * dt) + "," + state.get(0, 0) + "," + state.get(1,0) + "," + state.get(2, 0) + "," + state.get(3, 0));
      
    }

    return state;
  }

  public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane(Vector<N4> x, double dt) {
    Vector<N5> state = VecBuilder.fill(x.get(0, 0), x.get(1, 0), x.get(2, 0), x.get(3, 0), dt);
    double counter = 0;

    while (state.get(3, 0) > 0) {

      counter += dt;
      state = rkFour(state);


      if (state.get(1, 0) > SPEAKER_HEIGHT) {
        double timeAgo = (state.get(1, 0) - SPEAKER_HEIGHT) / state.get(3, 0);
        double intersectX = state.get(0, 0) - (timeAgo * state.get(2, 0));
        System.out.println(counter + "," + intersectX);
        return Optional.of(intersectX);
      }
      
    }

    return Optional.empty();
  }

  public Translation2d shooterExitRobotRelative(double theta) {
    double x = SHOOTER_PIVOT_TO_END * Math.cos(theta);
    double y = SHOOTER_PIVOT_TO_END * Math.sin(theta);

    return SHOOTER_PIVOT_ROBOT_REL.plus(new Translation2d(x, y));
  }

  
}
