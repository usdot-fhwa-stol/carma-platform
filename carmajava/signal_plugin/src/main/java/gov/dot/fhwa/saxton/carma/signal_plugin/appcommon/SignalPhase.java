/*
 * Copyright (C) 2018 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon;

public enum SignalPhase {
	GREEN(0),
	YELLOW(1),
	RED(2),
    NONE(3);            // this value is used to indicate missing data

	SignalPhase(int val) {
		this.val = val;
	}
	
	public int value() {
		return this.val;
	}
	
	public SignalPhase next() {
		return values()[(ordinal()+1) % values().length];
	}
	
	private int val;
}
