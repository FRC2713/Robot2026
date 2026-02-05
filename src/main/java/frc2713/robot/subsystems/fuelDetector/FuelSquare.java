package frc2713.robot.subsystems.fuelDetector;

import java.util.ArrayList;

public class FuelSquare {
  public ArrayList<FuelCoordinates> fuelList = new ArrayList<>(0);

  public int gridX;
  public int gridY;
  public double squareWidth;
  public double squareHeight;

  public boolean isInFuelCluster = false;
  public FuelCluster cluster;

  public double averageDepth;

  private int fuelCount = 0;

  public FuelSquare(int x, int y, double width, double height) {
    gridX = x;
    gridY = y;
    squareWidth = width;
    squareHeight = height;
  }

  public FuelSquare(int x, int y) {
    gridX = x;
    gridY = y;
  }

  public FuelSquare() {
    gridX = 0;
    gridY = 0;
  }

  public void addFuel(FuelCoordinates fuel) {
    fuelList.add(fuel);
    setFuelCount();
  }

  public int getFuelCount() {
    return fuelCount;
  }

  private void setFuelCount() {
    fuelCount = fuelList.size();
  }

  public void addToCluster(FuelCluster c) {
    isInFuelCluster = true;
    cluster = c;
  }

  public double averageFuelWidth() {
    int size = fuelList.size();
    double sum = 0;
    for (int i = 0; i < size; i++) {
      sum += fuelList.get(i).width;
    }
    sum /= size;
    return sum;
  }

  public int getGridX() {
    return gridX;
  }

  public int getGridY() {
    return gridY;
  }

  public String toString() {
    return "Fuel square: " + " Fuel count: " + getFuelCount();
  }
}
