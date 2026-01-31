package frc2713.robot.subsystems.fuelDetector;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class FuelDetector extends SubsystemBase {
  public final double fuelChanceThreshold = 0.8; // Percentage in decimal format
  public final int fuelDensityThreshold = 1; // fuels per grid square
  public final int kGridWidth = 3; // number of horizontal grid cells - 1
  public final int kGridHeight = 3; // number of vertical grid cells - 1
  public final int kImageWidth = 640;
  public final int kImageHeight = 480;

  private final StringSubscriber fuelSub =
      NetworkTableInstance.getDefault().getStringTopic("/fuelDetector/fuelData").subscribe("");

  public void periodic() {
    // get fuel information, call algorithm
    String fuelData = fuelSub.get("");
    // System.out.println("fuelData: " + fuelData);
    FuelCoordinates[] fuels = FuelDetector.dataToFuelCoordinates(fuelData);
    ArrayList<FuelCluster> fuelClusters = findFuelClusters(fuels, kGridWidth, kGridHeight);
    if (fuelClusters.size() > 0) {
      double vector =
          fuelClusters.get(0).findAngleTranslation(60.0, kImageWidth, (kImageWidth / kGridWidth));
      Logger.recordOutput("First Fuel Cluster", vector);
    }
    // System.out.println(
    //    findFuelClusters(fuels, kGridWidth, kGridHeight).toString() + " fuel clusters");
  }

  public ArrayList<FuelCoordinates> filterByHighChance(FuelCoordinates[] inputs) {
    // There's probably an easier and shorter way of doing this, but this is simple. Feel free to
    // change it as long as the output doesn't change.
    ArrayList<FuelCoordinates> output = new ArrayList<>(0);
    for (int i = 0; i < inputs.length; i++) {
      if (inputs[i].chance >= fuelChanceThreshold) {
        output.add(inputs[i]);
      }
    }
    return output;
  }

  public FuelSquare[][] divideIntoSquares(
      ArrayList<FuelCoordinates> fuelCoords, int gridWidth, int gridHeight) {
    FuelSquare[][] output =
        new FuelSquare[gridWidth + 1]
            [gridHeight
                + 1]; // Do not change this to not add 1 to grid height and width, the code will
    // crash.
    for (int w = 0; w < output.length; w++) {
      for (int h = 0; h < output[w].length; h++) {
        output[w][h] = new FuelSquare(w, h);
      }
    }
    for (int i = 0; i < fuelCoords.size(); i++) {
      fuelCoords
          .get(i)
          .assignSelfToFuelSquare(gridWidth, gridHeight, kImageWidth, kImageHeight, output);
    }
    return output;
  }

  public ArrayList<FuelCluster> getFuelClusters(FuelSquare[][] fuelSquares) {
    ArrayList<FuelCluster> clusters = new ArrayList<>(0);
    int width = fuelSquares.length;
    int height = fuelSquares[0].length;
    for (int w = 0; w < width; w++) {
      for (int h = 0; h < height; h++) {
        FuelSquare fuelSquare = fuelSquares[w][h];
        if (fuelSquare.getFuelCount() >= fuelDensityThreshold) {
          if (w + 1 < width && fuelSquares[w + 1][h].isInFuelCluster) {
            fuelSquares[w + 1][h].cluster.addFuelSquare(fuelSquare);
          } else if (w - 1 >= 0 && fuelSquares[w - 1][h].isInFuelCluster) {
            fuelSquares[w - 1][h].cluster.addFuelSquare(fuelSquare);
          } else if (h + 1 < height && fuelSquares[w][h + 1].isInFuelCluster) {
            fuelSquares[w][h + 1].cluster.addFuelSquare(fuelSquare);
          } else if (h - 1 >= 0 && fuelSquares[w][h - 1].isInFuelCluster) {
            fuelSquares[w][h - 1].cluster.addFuelSquare(fuelSquare);
          } else {
            FuelCluster c = new FuelCluster(fuelSquare);
            clusters.add(c);
          }
        }
      }
    }
    return clusters;
  }

  public ArrayList<FuelCluster> findFuelClusters(
      FuelCoordinates[] inputs, int gridWidth, int gridHeight) {
    ArrayList<FuelCoordinates> highChanceFuel = filterByHighChance(inputs);
    FuelSquare[][] fuelSquares = divideIntoSquares(highChanceFuel, gridWidth, gridHeight);
    ArrayList<FuelCluster> clusters = getFuelClusters(fuelSquares);
    return clusters;
  }

  public static FuelCoordinates[] dataToFuelCoordinates(String data) {
    // data is essentially a special type of .csv file
    // a ; seperates fuels, a , seperates fuel properties
    // In order of properties: x, y, width, height, chance

    String[] fuels = data.split(";");
    FuelCoordinates[] output = new FuelCoordinates[fuels.length];
    for (int i = 0; i < fuels.length; i++) {
      output[i] = new FuelCoordinates(fuels[i]);
    }
    return output;
  }
}
