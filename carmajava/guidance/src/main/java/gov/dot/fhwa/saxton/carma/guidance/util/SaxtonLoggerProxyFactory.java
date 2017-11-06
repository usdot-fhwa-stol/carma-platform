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

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.logging.Log;

/**
 * Factory class for SaxtongLoggerProxy instances
 * <p>
 * Maintains an internal map of proxies created for each class so they can be reused between
 * multiple instances of the same class or recreated instances.
 */
public class SaxtonLoggerProxyFactory implements ILoggerFactory {
  protected Map<Class<?>, SaxtonLoggerProxy> proxies = new HashMap<>();
  protected Log baseLog;

  public SaxtonLoggerProxyFactory(Log baseLog) {
    this.baseLog = baseLog;
  }

	@Override
	public ILogger createLoggerForClass(Class<?> clazz) {
    // Use a hashmap to memoize and return previously constructed loggers if need be, avoids too many writers to the same logfile
    if (proxies.containsKey(clazz)) {
      return proxies.get(clazz);
    } else {
      SaxtonLoggerProxy proxy = new SaxtonLoggerProxy(baseLog, clazz.getSimpleName());
      proxies.put(clazz, proxy);
      return proxy;
    }
	}
}