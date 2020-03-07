package org.ballardrobotics.types;

import java.util.TreeMap;

/**
 * 
 */
public class InterpolatingTreeMap extends TreeMap<Double, Double> {
  private static final long serialVersionUID = 4572880438798754547L;

  @Override
  public Double get(Object key) {
    if (key instanceof Double) {
      return this.getInterpolated((Double)key);
    }
    return super.get(key);
  }

  public Double getInterpolated(Double key) {
    Double value = super.get(key);
    if (value != null) {
      return value;
    }

    Double ceilingKey = super.ceilingKey(key);
    Double floorKey = super.floorKey(key);
    if (ceilingKey == null && floorKey == null) {
      return null;
    }
    if (ceilingKey == null) {
      return super.get(floorKey);
    }
    if (floorKey == null) {
      return super.get(ceilingKey);
    }

    double ceilingValue = super.get(ceilingKey);
    double floorValue = super.get(floorKey);

    double keyValue = key.doubleValue();
    double keyDifference = ceilingKey.doubleValue() - floorKey.doubleValue();
    double valueDifference = ceilingValue - floorValue;

    return floorValue + (keyValue-floorKey)*(valueDifference/keyDifference);
  }
}
