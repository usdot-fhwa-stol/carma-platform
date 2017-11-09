/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.util;

/**
 * Statically available class to allow logging from any class in Guidance.
 * <p>
 * Will construct new Logger instances for classes that call getLogger() and determine
 * their classes automatically. Returns ILogger instances which should normally be backed
 * by SaxtonLogger concrete classes.
 */
public class LoggerManager {
  private static ILoggerFactory loggerFactory;

  /**
   * Set the ILoggerFactory instance to be used to create new ILogger instances
   */
  public static void setLoggerFactory(ILoggerFactory factory) {
    loggerFactory = factory;
  }

  /**
   * Get a logger instance for the class calling this method, determined by reflection
   */
  public static ILogger getLogger() {
    try {
      // Get the callee class and make a new logger
      Class<?> callee = Class.forName(Thread.currentThread().getStackTrace()[2].getClassName());
      return loggerFactory.createLoggerForClass(callee);
    } catch (ClassNotFoundException e) {
      // Should never hit this branch... means we got called from a class that doesn't exist
      throw new RuntimeException("Unable to create logger for class!");
    }
  }

  /**
   * Stringly typed variant of getLogger() for uses in cases where getLogger() may return the wrong class name
   */
  public static ILogger getLogger(String className) {
    try {
      // Get the callee class and make a new logger
      Class<?> callee = Class.forName(className);
      return loggerFactory.createLoggerForClass(callee);
    } catch (ClassNotFoundException e) {
      // Should never hit this branch... means we got called from a class that doesn't exist
      throw new RuntimeException("Unable to create logger for class!");
    }
  }
}