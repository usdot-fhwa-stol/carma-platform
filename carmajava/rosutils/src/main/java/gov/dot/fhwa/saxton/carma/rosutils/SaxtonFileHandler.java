package gov.dot.fhwa.saxton.carma.rosutils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.logging.FileHandler;

public class SaxtonFileHandler extends FileHandler {
  public SaxtonFileHandler () throws IOException {
    super();
    try {
      //Initial setup requires performing 2 commands on the terminal.
      //1) sudo mkdir -p /opt/carma/logs and
      //2) sudo chmod -R ugo+rw /opt/carma
      //log file name is generated and stored in /opt/carma/logname.txt
      BufferedReader br = new BufferedReader(new FileReader("/opt/carma/logname.txt")); 
      String fileName = br.readLine() + ".txt";
      this.pattern = "/opt/carma/logs/" + fileName; //TODO: Will see later if needed to be stored in param.
      br.close();
    } catch (Exception e) {
      //Ignore but do log it.
    }
  }
}