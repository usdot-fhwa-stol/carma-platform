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

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Static class for assisting in the parsing of Mobility Messages
 */
public final class MobilityHelper {
  /**
   * Function to extract strategy params from a mobility message strategy string
   * Params are expected to be of the form TYPE|KEY1:value1,KEY2:value2 etc.
   * This will throw an exception if the provided paramsString does not match the expected type and keys
   * 
   * @param paramsString The strategy params string to process
   * @param expectedType The expected type value in the strategy params. If this value is null the type will be assumed not to exist.
   * @param keys A list of expected keys in the order they appear. Example ["KEY1", "KEY2"]
   * 
   * @return A list of string values associated with the provided keys. Example ["value1", "value2"]
   */
  public static List<String> extractStrategyParams(String paramsString, String expectedType, List<String> keys) 
    throws IllegalArgumentException {

    String dataString = paramsString;

    // If we expect a data type extract and validate it
    if (expectedType != null) {
      String[] paramsParts  = paramsString.split("\\|");

      // Check the correct type string was provided
      if (paramsParts.length != 2
        || !paramsParts[0].equals(expectedType)) {
        throw new IllegalArgumentException("Invalid type. Expected: " + expectedType + " String: " + paramsString);
      }

      dataString = paramsParts[1]; // Get data string
    }

    Pattern pattern = Pattern.compile("(?<=(^|,))(.*?)(?=(,|$))"); // Reg ex grabs everything between two commas or the start and end of a string
    Matcher matcher = pattern.matcher(dataString);

    List<String> dataList = new ArrayList<>(keys.size());

    // Iterate expected number of times and extract values
    for (int i = 0; i < keys.size(); i++) {
      // If we can't extract a value the input string is badly formatted
      if (!matcher.find()) {
        throw new IllegalArgumentException("Failed to find pattern match between commas. String: " + paramsString);
      }

      String[] dataParts = matcher.group().split(":");

      // Check if the key is correct
      if (dataParts.length != 2
       || !dataParts[0].equals(keys.get(i))) {
        throw new IllegalArgumentException("Invalid key. Expected: " + keys.get(i) + " String: " + paramsString);
      }

      dataList.add(dataParts[1]);
    }

    return dataList;
  }
}