package frc2713.robot.subsystems.fuelDetector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

public class FuelCluster {
  public ArrayList<FuelSquare> fuelSquares = new ArrayList<>(0);
  public int fuelCount = 0;
  public double clusterDepth;

  public
  FuelCluster() {} // Used for an edge case in FuelDetector where an instance of this class had to
  // be initialized because Java was being dumb. Do not delete.

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

  public Rotation2d findAngleTranslation(
      double FOV, int xResolution, int pixelsPerHorizontalGrid, boolean isLimelightData) {
    double avgX = 0;
    for (int i = 0; i < fuelSquares.size(); i++) {
      avgX += ((fuelSquares.get(i).getGridX()) * pixelsPerHorizontalGrid);
    }
    avgX /= fuelSquares.size();

    avgX -= xResolution / 2;
    System.out.println(avgX);
    double degreesPerPixel = FOV / xResolution;
    return new Rotation2d(Units.degreesToRadians(-(avgX * degreesPerPixel)));
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
