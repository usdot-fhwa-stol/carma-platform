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

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;

/**
 * SaxtonLogger backed implementation of ILogger
 * <p>
 * Done so that rosutils doesn't have to depend on GuidancePluginAPI and vice-versa
 */
public class SaxtonLoggerProxy implements ILogger {

  protected SaxtonLogger log;

  SaxtonLoggerProxy(Log baseLog, String className) {
    log = new SaxtonLogger(className, baseLog);
  }

	@Override
	public void debug(String message) {
    log.debug(message);
	}

	@Override
	public void debug(String message, Throwable t) {
		log.debug(message, t);
	}

	@Override
	public void debug(String tag, String message) {
		log.debug(tag, message);
	}

	@Override
	public void debug(String tag, String message, Throwable t) {
		log.debug(tag, message, t);
	}

	@Override
	public void info(String message) {
		log.info(message);
	}

	@Override
	public void info(String message, Throwable t) {
		log.info(message, t);
	}

	@Override
	public void info(String tag, String message) {
		log.info(tag, message);
	}

	@Override
	public void info(String tag, String message, Throwable t) {
		log.info(tag, message, t);
	}

	@Override
	public void warn(String message) {
		log.warn(message);
	}

	@Override
	public void warn(String message, Throwable t) {
		log.warn(message, t);
	}

  @Override
	public void warn(String tag, String message) {
    log.warn(tag, message);
	}

	@Override
	public void warn(String tag, String message, Throwable t) {
		log.warn(tag, message, t);
	}

	@Override
	public void error(String message) {
		log.error(message);
	}

	@Override
	public void error(String message, Throwable t) {
		log.error(message, t);
	}

	@Override
	public void error(String tag, String message) {
		log.error(tag, message);
	}

	@Override
	public void error(String tag, String message, Throwable t) {
    log.error(tag, message, t);
	}

	@Override
	public void fatal(String message) {
		log.fatal(message);
	}

	@Override
	public void fatal(String message, Throwable t) {
		log.fatal(message, t);
	}

	@Override
	public void fatal(String tag, String message) {
		log.fatal(tag, message);
	}

	@Override
	public void fatal(String tag, String message, Throwable t) {
    log.fatal(tag, message, t);
	}

	@Override
	public void trace(String message) {
		log.trace(message);
	}

	@Override
	public void trace(String message, Throwable t) {
		log.trace(message, t);
	}

	@Override
	public void trace(String tag, String message) {
		log.trace(tag, message);
	}

	@Override
	public void trace(String tag, String message, Throwable t) {
		log.trace(tag, message, t);
	}
}