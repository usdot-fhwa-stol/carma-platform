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

import static org.junit.Assert.*;

import java.util.concurrent.TimeUnit;

import org.junit.Test;
import org.junit.Ignore;
import org.junit.After;

public class RosServiceResultTest {

  protected Thread thread;

  @After
  public void tearDown() {
    thread.interrupt();
    thread = null;
  }

  @Test
  public void testBasicSynchronization() {
    final boolean[] testVal = { true };
    final RosServiceResult<String> result = new RosServiceResult<String>();
    thread = new Thread(new Runnable() {
      public void run() {
        try {
          Thread.sleep(1000);
          testVal[0] = false;
          result.complete(new RosServiceResponse<String>("Hello world!"));
        } catch (InterruptedException e) {
          fail();
        }
      }
    });
    thread.start();

    assertTrue("testVal not yet modified", testVal[0]);
    try {
      RosServiceResponse<String> o = result.get();
      assertTrue("testVal now modified", testVal[0] == false);
      assertEquals("Hello world!", o.get());
    } catch (InterruptedException e) {
      fail();
    }
  }

  @Test
  public void testTimeout() {
    final boolean[] testVal = { true };
    final RosServiceResult<Object> result = new RosServiceResult<Object>();
    thread = new Thread(new Runnable() {
      public void run() {
        try {
          Thread.sleep(1000);
          testVal[0] = false;
          result.complete(null);
        } catch (InterruptedException e) {
          fail();
        }
      }
    });
    thread.start();

    assertTrue("testVal not yet modified", testVal[0]);
    try {
      Object o = result.get(500, TimeUnit.MILLISECONDS);
      assertTrue("testVal not yet modified, after sleep", testVal[0]);
      assertEquals(null, o);
    } catch (InterruptedException e) {
      fail();
    }
  }
}