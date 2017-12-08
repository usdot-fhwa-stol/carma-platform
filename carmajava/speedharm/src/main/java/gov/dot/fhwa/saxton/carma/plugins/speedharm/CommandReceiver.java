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

package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.speedharm.api.objects.VehicleCommand;

import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;
import static gov.dot.fhwa.saxton.carma.plugins.speedharm.UrlConstants.*;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Handles the communications necessary to receive speed commands from the 
 * STOL Infrastructure server
 */
public class CommandReceiver implements Runnable {
	protected ILogger log = LoggerManager.getLogger();
	protected RestTemplate restClient;
	protected String serverUrl;
	protected int vehicleSessionId;
	protected AtomicReference<VehicleCommand> lastCommand = new AtomicReference<>(null);

	CommandReceiver(String serverUrl, int vehicleSessionId, RestTemplate restClient) {
		this.serverUrl = serverUrl;
		this.vehicleSessionId = vehicleSessionId;
		this.restClient = restClient;
	}

	@Override
	public void run() {
		while (!Thread.currentThread().isInterrupted()) {
			try {
				LocalDateTime timestepStart = LocalDateTime.now();
				VehicleCommand cmd = restClient.getForObject(serverUrl + COMMANDS_LIST + "/" + vehicleSessionId,
						VehicleCommand.class);
				if (cmd != null) {
					lastCommand.set(cmd);
					LocalDateTime cmdRecvd = LocalDateTime.now();

					log.info(String.format("Received speed command %s after %dms from server!", cmd.toString(),
							Duration.between(timestepStart, cmdRecvd).toMillis()));
				} else {
					log.warn("Null command received");
				}
			} catch (RestClientException rce) {
				log.warn("Unable to wait for server speed command, received exception.", rce);
				try {
					Thread.sleep(100);
				} catch (InterruptedException ie) {
					Thread.currentThread().interrupt();
				}
			}
		}
	}

	public VehicleCommand getLastCommand() {
		return lastCommand.get();
	}
}
