package frc2713.robot.subsystems.fuelDetector;

import java.util.ArrayList;

public class FuelCluster {
  public ArrayList<FuelSquare> fuelSquares = new ArrayList<>(0);
  public int fuelCount = 0;
  public double clusterDepth;

  public FuelCluster() {}

  public FuelCluster(ArrayList<FuelSquare> squares) {
    fuelSquares = squares;
    updateFuelCount();
  }

  public FuelCluster(FuelSquare square) {
    addFuelSquare(square);
  }

  public void addFuelSquare(FuelSquare square) {
    fuelSquares.add(square);
    square.addToCluster(this);
    updateFuelCount(square);
  }

  public void updateFuelCount() {
    int tempCount = 0;
    for (int i = 0; i < fuelSquares.size(); i++) {
      tempCount += fuelSquares.get(i).getFuelCount();
    }
    fuelCount = tempCount;
  }

  private void updateFuelCount(FuelSquare square) {
    fuelCount += square.getFuelCount();
  }

  public double averageSquareDepth() {
    int size = fuelSquares.size();
    double sum = 0;
    for (int i = 0; i < size; i++) {
      sum += fuelSquares.get(i).averageDepth;
    }
    sum /= size;
    return sum;
  }

  public String toString() {
    return "fuelSquare count: "
        + fuelSquares.size()
        + " Depth: "
        + clusterDepth
        + " Fuel count: "
        + fuelCount;
  }
}
