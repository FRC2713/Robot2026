package frc2713.lib.io;

public class AdvantageScopePathBuilder {
  private String basePath = "";

  public AdvantageScopePathBuilder(String basePath) {
    this.basePath = basePath;
  }

  /**
   * Creates a path string by concatenating the subsystem name with the provided path segments.
   *
   * @param pathSegments The path segments to append to the subsystem name.
   * @return A concatenated path string.
   */
  public String makePath(String... pathSegments) {
    StringBuilder path = new StringBuilder();
    path.append(this.basePath);
    for (String segment : pathSegments) {
      path.append("/").append(segment);
    }

    return path.toString();
  }

  /**
   * Creates a name string by concatenating the subsystem name with the provided name.
   *
   * @param name
   * @return
   */
  public String makeName(String name) {
    return this.basePath + " " + name;
  }
}
