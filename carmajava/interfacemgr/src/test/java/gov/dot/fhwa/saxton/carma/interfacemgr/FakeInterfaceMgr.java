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

package gov.dot.fhwa.saxton.carma.interfacemgr;

import gov.dot.fhwa.saxton.carma.rosutils.AlertSeverity;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.ArrayList;
import java.util.List;
import static org.junit.Assert.*;

public class FakeInterfaceMgr implements IInterfaceMgr {
    private SaxtonLogger log = new SaxtonLogger(FakeInterfaceMgr.class.getSimpleName(), LogFactory.getLog(FakeInterfaceMgr.class));
    private boolean shutdownCalled_ = false;

    public void errorShutdown(String msg) {
        log.debug("errorShutdown received the following message: " + msg);
        shutdownCalled_ = true;
    }

    public boolean isShutdownUnderway() {
        return shutdownCalled_;
    }
}