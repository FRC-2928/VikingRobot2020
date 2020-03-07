package org.ballardrobotics.types;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Add your docs here.
 */
public class InterpolatingTreeMapTest {

  @Test
  public void testGet() {
    final double kEpsilon = 0.0001;

    var map = new InterpolatingTreeMap();
    map.put(1.0, 2.0);
    map.put(2.0, 4.0);
    map.put(3.0, 6.0);
    map.put(4.0, 8.0);
    map.put(5.0, 10.0);

    assertEquals(2.0, map.get(0.0), kEpsilon);
    assertEquals(3.0, map.get(1.5), kEpsilon);
    assertEquals(10.0, map.get(8.0), kEpsilon);
  }

}
