/*
 * Copyright (C) 2018-2019 LEIDOS.
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

public interface ILogger {
  public void debug(String message);

  public void debug(String message, Throwable t);

  public void debug(String tag, String message);

  public void debugf(String message, Object... args);

  public void debugf(String tag, String message, Object... args);

  public void debug(String tag, String message, Throwable t);

  public void info(String message);

  public void info(String message, Throwable t);

  public void info(String tag, String message);

  public void info(String tag, String message, Throwable t);

  public void infof(String message, Object... args);

  public void infof(String tag, String message, Object... args);

  public void warn(String message);

  public void warn(String message, Throwable t);

  public void warn(String tag, String message);

  public void warn(String tag, String message, Throwable t);

  public void warnf(String message, Object... args);

  public void warnf(String tag, String message, Object... args);

  public void error(String message);

  public void error(String message, Throwable t);

  public void error(String tag, String message);

  public void error(String tag, String message, Throwable t);

  public void errorf(String message, Object... args);

  public void errorf(String tag, String message, Object... args);

  public void fatal(String message);

  public void fatal(String message, Throwable t);

  public void fatal(String tag, String message);

  public void fatal(String tag, String message, Throwable t);

  public void fatalf(String message, Object... args);

  public void fatalf(String tag, String message, Object... args);

  public void trace(String message);

  public void trace(String message, Throwable t);

  public void trace(String tag, String message);

  public void trace(String tag, String message, Throwable t);

  public void tracef(String message, Object... args);

  public void tracef(String tag, String message, Object... args);
}