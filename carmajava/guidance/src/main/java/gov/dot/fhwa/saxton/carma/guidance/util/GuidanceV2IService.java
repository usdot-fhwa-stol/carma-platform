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

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Iterator;
import java.util.TimeZone;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.time.LocalDateTime;
import java.time.temporal.ChronoUnit;
import cav_msgs.IntersectionGeometry;
import cav_msgs.IntersectionState;
import cav_msgs.MapData;
import cav_msgs.SPAT;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;

public class GuidanceV2IService implements V2IService {
    private ISubscriber<MapData> mapSub;
    private ISubscriber<SPAT> spatSub;
    private final int mapCommsReliabilityCheckThreshold;
    private final int spatCommsReliabilityCheckThreshold;
    private final double minMapMsgsPerSec;
    private final double minSpatMsgsPerSec;
    private final long expiryTimeoutMs;
    private Thread expiryCheckThread;
    private List<V2IDataCallback> callbacks = Collections.synchronizedList(new ArrayList<>());

    private final ConcurrentHashMap<Integer, I2VData> trackedI2Vs = new ConcurrentHashMap<>(10); // Initialize with capacity of 10. Unlikely we will see more intersections than this at one time.

    private IPubSubService pubSub;
    private ILogger log = LoggerManager.getLogger();

    public GuidanceV2IService(IPubSubService pubSub, int mapCommsReliabilityCheckThreshold,
    int spatCommsReliabilityCheckThreshold, 
    double minMapMsgsPerSec, double minSpatMsgsPerSec, long expiryTimeoutMs) {

        this.pubSub = pubSub;
        this.mapCommsReliabilityCheckThreshold = mapCommsReliabilityCheckThreshold;
        this.spatCommsReliabilityCheckThreshold = spatCommsReliabilityCheckThreshold;
        this.minMapMsgsPerSec = minMapMsgsPerSec;
        this.minSpatMsgsPerSec = minSpatMsgsPerSec;
        this.expiryTimeoutMs = expiryTimeoutMs;
    }

    public void init() {
        log.infof("GuidanceV2IService init'd with mapCommsReliabilityCheckThreshold=%d, spatCommsReliabilityCheckThreshold=%d, " + 
        "minMapMsgsPerSec=%.02f, minSpatMsgsPerSec=%.02f, expiryTimeoutMs=%d",
        mapCommsReliabilityCheckThreshold, spatCommsReliabilityCheckThreshold, minMapMsgsPerSec, minSpatMsgsPerSec, expiryTimeoutMs);

        mapSub = pubSub.getSubscriberForTopic("incoming_map", MapData._TYPE);

        mapSub.registerOnMessageCallback((map) -> {
            for (IntersectionGeometry geometry : map.getIntersections()) {
                LocalDateTime msgTs = LocalDateTime.ofInstant(Instant.ofEpochMilli((long) (map.getHeader().getStamp().toSeconds() * 1000.0)), TimeZone.getDefault().toZoneId());
                int id = geometry.getId().getId();
                
                reportNewMapComms(id, msgTs, geometry);

                fireCallbacks();
            }
        });

        spatSub = pubSub.getSubscriberForTopic("incoming_spat", SPAT._TYPE);
        spatSub.registerOnMessageCallback((spat) -> {
            for (IntersectionState state : spat.getIntersectionStateList()) {
                LocalDateTime msgTs = LocalDateTime.now(); // TODO: Improve methodology for determining data age
                int id = state.getId().getId();
                reportNewSPATComms(id, msgTs, state);
                fireCallbacks();
            }
        });

        expiryCheckThread = new Thread(() -> {
            while (!Thread.interrupted())  {

                synchronized (trackedI2Vs) {
                    Iterator<Map.Entry<Integer, I2VData>> iterator = trackedI2Vs.entrySet().iterator(); // Iteration on entry set is not thread safe and must be synchronized
                    while(iterator.hasNext()) { // Check all tracked intersections for expiration
                        Map.Entry<Integer, I2VData> entry = iterator.next();
                        Integer id = entry.getKey();
                        I2VData data = entry.getValue();
                        LocalDateTime now = LocalDateTime.now();
                        boolean oldEnoughCreationTime = now.minus(expiryTimeoutMs, ChronoUnit.MILLIS).isAfter(data.creationTime);
                        boolean mapIsNull = data.mapComms == null;
                        boolean spatIsNull = data.spatComms == null;
                        if(oldEnoughCreationTime // If this intersection was detected long enough ago
                            && ((mapIsNull || spatIsNull) // AND (It still has not seen both map and spat OR map or spat is unreliable)
                            || (!data.mapComms.isReliable() ||  !data.spatComms.isReliable())))
                        {
                            log.info("Removing unreliable intersection with id: " + id
                                + " causes: mapIsNull: " + mapIsNull + " spatIsNull: " + spatIsNull
                                + " mapIsNotReliable: " + (!mapIsNull ? !data.mapComms.isReliable() : false)
                                + " spatIsNotReliable: " + (!spatIsNull ? !data.spatComms.isReliable() : false)
                            );
                            trackedI2Vs.remove(id);
                        }
                    }
                }
                try {
					Thread.sleep(expiryTimeoutMs);
				} catch (InterruptedException e) {
                    log.error("Exception in V2I expiryCheckThread: ", e);
				}
            }
        });
        expiryCheckThread.start();
    }

    private void fireCallbacks() {
        List<IntersectionData> recentData = getV2IData();
        callbacks.forEach((V2IDataCallback cb) -> {
            cb.onV2IDataChanged(recentData);
        });
    }

    private void reportNewSPATComms(int id, LocalDateTime ts, IntersectionState state) {

        I2VData newI2VData = new I2VData(LocalDateTime.now());

        DsrcCommsCheck newSpatCheck = new DsrcCommsCheck(spatCommsReliabilityCheckThreshold, minSpatMsgsPerSec);

        newI2VData.spatComms = newSpatCheck;

        trackedI2Vs.merge(id, newI2VData, (I2VData existingData, I2VData newValue) -> {
            if (existingData.spatComms == null) {
                existingData.spatComms = newSpatCheck;
            }
            existingData.spatComms.recordNewCommsRx(ts);
            if (existingData.intersection != null) {
                IntersectionGeometry curGeometry = existingData.intersection.getIntersectionGeometry();
                LocalDateTime curGeometryStamp = existingData.intersection.getIntersectionGeometryRxTimestamp();
                IntersectionData newIntData = new IntersectionData(curGeometry, curGeometryStamp, state, ts);
                existingData.intersection = newIntData;
            }
            log.debug("Updated spat comms for id: " + id);
            return existingData;
        });
    }

    private void reportNewMapComms(int id, LocalDateTime ts, IntersectionGeometry geometry) {

        I2VData newI2VData = new I2VData(LocalDateTime.now());

        DsrcCommsCheck newMapCheck = new DsrcCommsCheck(mapCommsReliabilityCheckThreshold, minMapMsgsPerSec);

        newI2VData.mapComms = newMapCheck;

        IntersectionData newIntersectionData = new IntersectionData(geometry, ts); 
        newI2VData.intersection = newIntersectionData;

        trackedI2Vs.merge(id, newI2VData, (I2VData existingData, I2VData newValue) -> {
            if (existingData.mapComms == null) {
                existingData.mapComms = newMapCheck;
            }
            existingData.mapComms.recordNewCommsRx(ts);
            if (existingData.intersection == null) {
                existingData.intersection = newIntersectionData;
            } else {
                IntersectionState curState = existingData.intersection.getIntersectionState();
                LocalDateTime curStateStamp = existingData.intersection.getIntersectionStateRxTimestamp();
                IntersectionData newIntData = new IntersectionData(geometry, ts, curState, curStateStamp);
                existingData.intersection = newIntData;
            }
            log.debug("Updated map comms for id: " + id);
            return existingData;
        });
    }

    private class DsrcCommsCheck {
        protected List<LocalDateTime> stamps = new ArrayList<>();
        protected int idx = 0;
        protected final int commsReliabilityCheckThreshold;
        protected final double minMsgsPerSec;

        public DsrcCommsCheck(int commsReliabilityCheckThreshold, double minMsgsPerSec) {
            this.commsReliabilityCheckThreshold = commsReliabilityCheckThreshold;
            this.minMsgsPerSec = minMsgsPerSec;
        }

        protected void recordNewCommsRx(LocalDateTime rxTs) {
            stamps.add(rxTs);
            if (stamps.size() > commsReliabilityCheckThreshold) {
                stamps.remove(0);
            }
        }

        protected boolean isReliable() {
            if (stamps.size() < commsReliabilityCheckThreshold) {
                return false;
            }

            double messagesPerSec = stamps.size() / ((double)Duration.between(stamps.get(0), LocalDateTime.now()).toMillis() / 1000.0);

            if (messagesPerSec < minMsgsPerSec) {
                return false;
            } else {
                return true;
            }
       }
    }

    private class I2VData {
        public final LocalDateTime creationTime;

        public I2VData(LocalDateTime creationTime) {
            this.creationTime = creationTime;
        }

        private IntersectionData intersection;
        private DsrcCommsCheck mapComms;
        private DsrcCommsCheck spatComms;
    }

	@Override
	public void registerV2IDataCallback(V2IDataCallback callback) {
        callbacks.add(callback);
	}

	@Override
	public List<IntersectionData> getV2IData() {
        // Ensure we only get intersections which are reliable when the intersection was grabbed. This guarantees map and spat are present.
        List<I2VData> intersections = trackedI2Vs.values().stream()
        .filter(e -> e.intersection != null 
            && e.mapComms != null && e.spatComms != null
            && e.mapComms.isReliable() && e.spatComms.isReliable())
        .collect(Collectors.toList());
        
        // Extract IntersectionData objects
        List<IntersectionData> intData = new ArrayList<>();
        for (I2VData intersection: intersections) {
            intData.add(intersection.intersection);
        }

        return intData;
	}
}