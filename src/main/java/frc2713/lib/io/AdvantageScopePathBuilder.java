package frc2713.lib.io;

import java.util.HashMap;
import java.util.Map;

public class AdvantageScopePathBuilder {
  private final String basePath;
  // Tree cache: each level is keyed by path segment. Leaf values are the full path string.
  // Uses segment strings as keys (no allocation) so cache hits avoid building the path.
  private final Map<String, Object> pathCache = new HashMap<>();

  public AdvantageScopePathBuilder(String basePath) {
    this.basePath = basePath;
  }

  /**
   * Creates a path string by concatenating the subsystem name with the provided path segments.
   * Results are cached so repeated calls with the same arguments return the same String instance,
   * avoiding per-cycle allocations in periodic().
   *
   * @param pathSegments The path segments to append to the subsystem name.
   * @return A concatenated path string (cached).
   */
  public String makePath(String... pathSegments) {
    if (pathSegments.length == 0) {
      return basePath;
    }
    Map<String, Object> current = pathCache;
    for (int i = 0; i < pathSegments.length; i++) {
      String segment = pathSegments[i];
      if (i == pathSegments.length - 1) {
        return (String) current.computeIfAbsent(segment, k -> buildPathString(pathSegments));
      }
      current =
          (Map<String, Object>)
              current.computeIfAbsent(segment, k -> new HashMap<String, Object>());
    }
    return basePath; // unreachable for pathSegments.length > 0
  }

  private String buildPathString(String... pathSegments) {
    StringBuilder path = new StringBuilder(basePath);
    for (String segment : pathSegments) {
      if (path.length() > 0) {
        path.append("/");
      }
      path.append(segment);
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
