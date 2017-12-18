package gov.dot.fhwa.saxton.carma.rosutils;

/**
 * Singleton class which holds the path of the log file to be used by the CARMA platform
 */
public class FilePathHolder {
  private static FilePathHolder instance = null;

  private String filePath_;

  /**
   * Private constructor makes a new FilePathHolder object with the specified path
   * @param filePath The file path which will be stored in this object
   */
  private FilePathHolder(String filePath) {
    filePath_ = filePath;
  }

  /**
   * Returns the singleton instance of the class
   * @param defaultFilePath A default file path which will be used only if no instance of this object has been instantiated
   * @return The singleton instance
   */
  public static FilePathHolder getInstance(String defaultFilePath) {
   if (instance == null) {
     instance = new FilePathHolder(defaultFilePath);
   }

   return instance;
  }

  /**
   * Gets the file path
   * @return the path
   */
  public String getFilePath() {
    return filePath_;
  }
}
