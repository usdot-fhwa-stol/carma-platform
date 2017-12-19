package gov.dot.fhwa.saxton.carma.rosutils;

/**
 * Singleton class which holds the path of the log file to be used by the CARMA platform
 * This class should be thread-safe
 */
public class FilePathHolder {
  private static volatile FilePathHolder instance = null;
  private static final Object mutex = new Object();
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
    FilePathHolder result = instance; // Copy volatile variable before check
   if (result == null) { // See if instance has been created
     // Synchronize access.
     // Inside if-statement for efficiency. Once the instance is created there is no need to synchronize
     synchronized (mutex) {
      result = instance; // Ensure no other thread has modified the instance yet
      if (result == null) { // Check if instance has been created
        result = new FilePathHolder(defaultFilePath);
        instance = result; // Assign instance
      }
     }
   }

   return result;
  }

  /**
   * Gets the file path
   * @return the path
   */
  public String getFilePath() {
    return filePath_;
  }
}
