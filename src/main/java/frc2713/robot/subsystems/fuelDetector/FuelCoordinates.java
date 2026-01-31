package frc2713.robot.subsystems.fuelDetector;

public class FuelCoordinates {
  public double chance;
  public double centerX;
  public double centerY;
  public double boxX;
  public double boxY;
  public double boxX2;
  public double boxY2;
  public double width;
  public double height;
  public double depth;

  public FuelCoordinates(double x, double y, double boxWidth, double boxHeight, double c) {
    centerX = x;
    centerY = y;
    width = boxWidth;
    height = boxHeight;
    computeDepth(45); // TODO: replace 45 with constant for camera FOV
    chance = c;
  }

  public FuelCoordinates(String args) {
    if (args.length() > 0) {
      String[] values = args.split(",");
      centerX = Double.parseDouble(values[0]);
      centerY = Double.parseDouble(values[1]);
      width = Double.parseDouble(values[2]);
      height = Double.parseDouble(values[3]);
      chance = Double.parseDouble(values[4].substring(0, values[4].length() - 1));
    }
  }

  private static double pointFromDistance(double point, double length) {
    // Get a second point from one point and a distance
    return point + length;
  }

  public void computeCenterPoint(double x, double y, double x2, double y2) {
    centerX = (x2 - x) / 2;
    centerY = (y2 - y) / 2;
  }

  public void assignSelfToFuelSquare(
      int gridWidth, int gridHeight, int imageWidth, int imageHeight, FuelSquare[][] squareArray) {
    int squareX = (int) Math.round(centerX / (imageWidth / gridWidth));
    int squareY = (int) Math.round(centerY / (imageHeight / gridHeight));
    squareArray[squareX][squareY].addFuel(this);
  }
  // TODO: implement depth function
  public double computeDepth(double cameraFOV) {
    return 0.0;
  }
}
