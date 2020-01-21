/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.rosutils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.logging.FileHandler;

public class SaxtonFileHandler extends FileHandler {
  private static String filePath = "";
  // Static code for generating the logs in a timestamped folder
  static {
    try {
      //Initial setup requires performing 2 commands on the terminal.
      //1) sudo mkdir -p /opt/carma/logs and
      //2) sudo chmod -R ugo+rw /opt/carma
      //log file name is generated and stored in /opt/carma/logname.txt
      BufferedReader br = new BufferedReader(new FileReader("/opt/carma/logname.txt")); 
      String folderName = br.readLine();
      // One liner for getting the path of the jar file running this code
      String jarPath = new File(SaxtonBaseNode.class.getProtectionDomain().getCodeSource().getLocation().toURI()).getPath();
      // Extract the node name from the jar file
      String[] jarPathParts = jarPath.split("/");
      String nodeName = jarPathParts[jarPathParts.length - 3];
      String fileName = nodeName + ".txt";
      // Create folder and log
      File file = new File("/opt/carma/logs/" + folderName + "/" + fileName); 
      file.getParentFile().mkdirs();
      SaxtonFileHandler.filePath = file.getAbsolutePath();
      br.close();
    } catch (Exception e) {
      //Ignore but do log it.
      System.out.println("Exception from SaxtonFileHandler: " + e);
      // If failed to make log folder dump logs with unique id
      SaxtonFileHandler.filePath = "/opt/carma/logs/carma_%u.log"; // %u will be a unique integer value for each log
    }
  }
  public SaxtonFileHandler() throws IOException {
    super(SaxtonFileHandler.filePath);
  }
}