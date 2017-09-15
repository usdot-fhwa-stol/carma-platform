/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.route;

import com.esotericsoftware.yamlbeans.YamlException;
import com.esotericsoftware.yamlbeans.YamlReader;
import java.io.FileNotFoundException;
import java.io.FileReader;

/**
 * Loads a route based on the provided file path.
 */
public class FileStrategy implements IRouteLoadStrategy{
  protected String filePath;

  /**
   * Constructor initializes a FileStrategy by providing the file path
   * @param path the file path
   */
  public FileStrategy(String path){
    this.filePath = path;
  }

  @Override public Route load() {
    try {
      FileReader fr = new FileReader(filePath);
      YamlReader reader = new YamlReader(fr);
      return reader.read(gov.dot.fhwa.saxton.carma.route.Route.class);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (YamlException e) {
      e.printStackTrace();
    }
    return null;
  }
}
