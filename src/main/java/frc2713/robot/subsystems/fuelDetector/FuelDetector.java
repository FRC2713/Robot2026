package frc2713.robot.subsystems.fuelDetector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class FuelDetector extends SubsystemBase {
  public final double fuelChanceThreshold = 0.8; // Percentage in decimal format
  public final int fuelDensityThreshold = 1; // fuels per grid square
  public final int kGridWidth = 9; // number of horizontal grid cells - 1
  public final int kGridHeight = 3; // number of vertical grid cells - 1
  public final int kImageWidth = 640;
  public final int kImageHeight = 480;

  public boolean isLimelights;

  private StringSubscriber simFuelSub;
  private DoubleArraySubscriber realFuelSub;

  public FuelDetector() {
    simFuelSub =
        NetworkTableInstance.getDefault().getStringTopic("/fuelDetector/fuelData").subscribe("");
    realFuelSub =
        NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/limelight-d/tcornxy")
            .subscribe(new double[0]);
  }

  public void periodic() {
    // get fuel information, call algorithm
    FuelCoordinates[] fuels = getDataFromNT();
    if (fuels.length > 0) {
      System.out.println(fuels[0].toString());
    } else {
      System.out.println("no fuels");
    }
    // System.out.println("fuelData: " + fuelData);
    Logger.recordOutput("First Fuel Cluster", getRotation2D(fuels, !isLimelights).getDegrees());
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
    for (int i = 0; i < fuelCoords.size() - 1; i++) {
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
      FuelCoordinates[] inputs, int gridWidth, int gridHeight, boolean filter) {

    ArrayList<FuelCoordinates> highChanceFuel;
    if (filter) {
      highChanceFuel = filterByHighChance(inputs);
    } else {
      highChanceFuel = new ArrayList<FuelCoordinates>(Arrays.asList(inputs));
    }

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

  public static FuelCoordinates[] dataToFuelCoordinates(double[] data) {
    // data is essentially a special type of .csv file
    // a ; seperates fuels, a , seperates fuel properties
    // In order of properties: x, y, width, height, chance

    double[] fuels = data;
    FuelCoordinates[] output = new FuelCoordinates[fuels.length / 2];
    for (int i = 0; i < fuels.length; i += 2) {
      output[i / 2] = new FuelCoordinates(fuels[i], fuels[i + 1]);
    }
    return output;
  }

  public FuelCoordinates[] getDataFromNT() {
    FuelCoordinates[] fuels;
    if (realFuelSub.exists()) {
      isLimelights = true;
      fuels = FuelDetector.dataToFuelCoordinates(realFuelSub.get());
    } else if (simFuelSub.exists()) {
      isLimelights = false;
      fuels = FuelDetector.dataToFuelCoordinates(simFuelSub.get());
    } else {
      isLimelights = false;
      fuels = new FuelCoordinates[0];
    }

    Logger.recordOutput("FuelDetector/is_limelights", isLimelights);
    Logger.recordOutput("FuelDetector/n_fuels", isLimelights);

    // System.out.println("Fuel Length: " + fuels.length);

    return fuels;
  }

  public Rotation2d getRotation2D(FuelCoordinates[] fuels, boolean filter) {
    ArrayList<FuelCluster> fuelClusters = findFuelClusters(fuels, kGridWidth, kGridHeight, filter);
    // System.out.println(fuelClusters.size());
    // ArrayList<FuelCluster> fuelClusters = fuels;
    if (fuelClusters.size() > 0) {
      FuelCluster largestCluster =
          new FuelCluster(); // Note: this is the largest in terms of fuel count, not visiual size.
      int maxFuels = 0;
      for (int i = 0; i < fuelClusters.size(); i++) {
        int size = fuelClusters.get(i).fuelCount;
        if (size > maxFuels) {
          maxFuels = size;
          largestCluster = fuelClusters.get(i);
        }
      }
      Rotation2d vector =
          largestCluster.findAngleTranslation(
              60.0, kImageWidth, (kImageWidth / kGridWidth), isLimelights);
      return vector;
    } else {
      return new Rotation2d(0); // Default value
    }
  }
}
