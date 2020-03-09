package frc.robot.oi;

/**
 * Add your docs here.
 */
public class Binder {

  public static void bindAll(Bindable ...bindables) {
    for (var b : bindables) {
      b.bind();
    }
  }

}
