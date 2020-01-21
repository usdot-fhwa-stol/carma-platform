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

package gov.dot.fhwa.saxton.carma.message.helper;

import java.util.Arrays;

import j2735_msgs.*;

/**
 * This is the helper class for encoding BSM.
 * All fields' unit in this class match the units in J2735 message.
 */
public class BSMMessageHelper {
	
	protected static final int UINT8_MAX = 255;
	protected static final int MSG_COUNT_MAX = 127;
	protected static final int MSG_COUNT_MIN = 0;
	protected static final int ID_MAX = 255;
	protected static final int ID_MIN = 0;
	protected static final int DSECOND_MIN = 0;
	protected static final int DSECOND_MAX = 65535;
	protected static final int LATITUDE_UNAVAILABLE = 900000001;
	protected static final int LATITUDE_MAX = 900000000;
	protected static final int LATITUDE_MIN = -900000000;
	protected static final int LONGITUDE_UNAVAILABLE = 1800000001;
	protected static final int LONGITUDE_MAX = 1800000000;
	protected static final int LONGITUDE_MIN = -1799999999;
	protected static final int ELEVATION_UNAVAILABLE = -4096;
	protected static final int ELEVATION_MAX = 61439;
	protected static final int ELEVATION_MIN = -4095;
	protected static final int ACCURACY_UNAVAILABLE = 255;
	protected static final int ACCURACY_MAX = 254;
	protected static final int ACCURACY_MIN = 0;
	protected static final int ACCURACY_ORIENTATION_UNAVAILABLE = 65535;
	protected static final int ACCURACY_ORIENTATION_MAX = 65534;
	protected static final int ACCURACY_ORIENTATION_MIN = 0;
	protected static final int TRANSMISSION_UNAVAILABLE = 7;
	protected static final int SPEED_UNAVAILABLE = 8191;
	protected static final int SPEED_MAX = 8190;
	protected static final int SPEED_MIN = 0;
	protected static final int HEADING_UNAVAILABLE = 28800;
	protected static final int HEADING_MAX = 28799;
	protected static final int HEADING_MIN = 0;
	protected static final int STEER_WHEEL_ANGLE_UNAVAILABLE = 127;
	protected static final int STEER_WHEEL_ANGLE_MAX = 126;
	protected static final int STEER_WHEEL_ANGLE_MIN = -126;
	protected static final int ACCELERATION_UNAVAILABLE = 2001;
	protected static final int ACCELERATION_MAX = 2000;
	protected static final int ACCELERATION_MIN = -2000;
	protected static final int ACCELERATION_VERTICAL_UNAVAILABLE = -127;
	protected static final int ACCELERATION_VERTICAL_MAX = 127;
	protected static final int ACCELERATION_VERTICAL_MIN = -126;
	protected static final int YAWRATE_UNAVAILABLE = 0;
	protected static final int YAWRATE_MAX = 32767;
	protected static final int YAWRATE_MIN = -32767;
	protected static final int BRAKES_STATUS_UNAVAILABLE = 0x10;
	protected static final int BRAKES_NOT_APPLIED = 0x0;
	protected static final int BRAKES_APPLIED = 0x78;
	protected static final int VEHICLE_WIDTH_MAX = 1023;
	protected static final int VEHICLE_LENGTH_MAX = 4095;
	protected static final int VEHICLE_SIZE_UNAVAILABLE = 0;
	protected static final int VEHICLE_SIZE_MIN = 1;
	
	private int msgCnt = MSG_COUNT_MIN;
	private int[] id = {ID_MIN, ID_MIN, ID_MIN, ID_MIN};
	private int secMark = DSECOND_MIN;
	private int lat = LATITUDE_UNAVAILABLE;
	private int lon = LONGITUDE_UNAVAILABLE;
	private int elev = ELEVATION_UNAVAILABLE;
	private int[] accuracy = {ACCURACY_UNAVAILABLE, ACCURACY_UNAVAILABLE, ACCURACY_ORIENTATION_UNAVAILABLE};
	private int transmission = TransmissionState.UNAVAILABLE;
	private int speed = SPEED_UNAVAILABLE;
	private int heading = HEADING_UNAVAILABLE;
	private int angle = STEER_WHEEL_ANGLE_UNAVAILABLE;
	private int[] acceleration = {ACCELERATION_UNAVAILABLE, ACCELERATION_UNAVAILABLE, ACCELERATION_VERTICAL_UNAVAILABLE, YAWRATE_UNAVAILABLE};
	private int wheel_brakes = BRAKES_STATUS_UNAVAILABLE;
	private int traction = TractionControlStatus.UNAVAILABLE;
	private int abs = AntiLockBrakeStatus.UNAVAILABLE;
	private int scs = StabilityControlStatus.UNAVAILABLE;
	private int bba = BrakeBoostApplied.UNAVAILABLE;
	private int aux = AuxiliaryBrakeStatus.UNAVAILABLE;
	private int[] vehicle_size = {VEHICLE_SIZE_UNAVAILABLE, VEHICLE_SIZE_UNAVAILABLE};
	
	/**
	 * This is the constructor for HelperBSM.
	 * @param bsm_core Take ros message as the input and set all fields in HelperBSM after necessary validations
	 */
	public BSMMessageHelper(BSMCoreData bsm_core) {
		this.setMsgCnt(bsm_core.getMsgCount());
		byte[] temp_ID = new byte[4];
		for(int i = 0; i < bsm_core.getId().capacity(); i++) {
			temp_ID[i] = bsm_core.getId().getByte(i);
		}
		this.setId(temp_ID);
		this.setSecMark(bsm_core.getSecMark());
		this.setLat(bsm_core.getLatitude());
		this.setLon(bsm_core.getLongitude());
		this.setElev(bsm_core.getElev());
		this.setAccuracy(bsm_core.getAccuracy().getSemiMajor(), bsm_core.getAccuracy().getSemiMinor(), bsm_core.getAccuracy().getOrientation());
		this.setTransmission(bsm_core.getTransmission().getTransmissionState());
		this.setSpeed(bsm_core.getSpeed());
		this.setHeading(bsm_core.getHeading());
		this.setAngle(bsm_core.getAngle());
		this.setAcceleration(bsm_core.getAccelSet().getLateral(), bsm_core.getAccelSet().getLongitudinal(),
				bsm_core.getAccelSet().getVert(), bsm_core.getAccelSet().getYawRate());
		this.setWheel_brakes(bsm_core.getBrakes().getWheelBrakes().getBrakeAppliedStatus());
		this.setTraction(bsm_core.getBrakes().getTraction().getTractionControlStatus());
		this.setAbs(bsm_core.getBrakes().getAbs().getAntiLockBrakeStatus());
		this.setScs(bsm_core.getBrakes().getScs().getStabilityControlStatus());
		this.setBba(bsm_core.getBrakes().getBrakeBoost().getBrakeBoostApplied());
		this.setAux(bsm_core.getBrakes().getAuxBrakes().getAuxiliaryBrakeStatus());
		this.setVehicle_size(bsm_core.getSize().getVehicleWidth(), bsm_core.getSize().getVehicleLength());
	}

	public void setMsgCnt(byte msgCnt_input) {
		if(msgCnt_input >= MSG_COUNT_MIN && msgCnt_input <= MSG_COUNT_MAX) {
			this.msgCnt = msgCnt_input;
		}
	}
	// TODO handle signed bytes in j2735_convertor node
	public void setId(byte[] id_input) {
		for(int i = 0; i < this.id.length; i++) {
			int temp_id;
			if(id_input[i] < 0) {
				temp_id = 1 + UINT8_MAX + id_input[i]; 
			} else {
				temp_id = id_input[i];
			}
			if(temp_id >= ID_MIN && temp_id <= ID_MAX) {
				this.id[i] = temp_id;
			}
		}
	}

	public void setSecMark(int secMark_input) {
		if(secMark_input >= DSECOND_MIN && secMark_input <= DSECOND_MAX) {
			this.secMark = secMark_input;
		}
	}

	public void setLat(int lat_input) {
		if(lat_input == BSMCoreData.LATITUDE_UNAVAILABLE) {
			return;
		}
		int integer_lat_input = lat_input;
		if(integer_lat_input >= LATITUDE_MIN && integer_lat_input <= LATITUDE_MAX) {
			this.lat = integer_lat_input;
		}
	}

	public void setLon(int lon_input) {
		if(lon_input == BSMCoreData.LONGITUDE_UNAVAILABLE) {
			return;
		}
		int integer_lon_input = lon_input;
		if(integer_lon_input >= LONGITUDE_MIN && integer_lon_input <= LONGITUDE_MAX) {
			this.lon = integer_lon_input;
		}
		
	}

	public void setElev(int elev_input) {
		if(elev_input == BSMCoreData.ELEVATION_UNAVAILABLE) {
			return;
		}
		int integer_elev_input = elev_input;
		if(integer_elev_input >= ELEVATION_MIN && integer_elev_input <= ELEVATION_MAX) {
			this.elev = integer_elev_input;
		}
	}

	public void setAccuracy(int semimajor_input, int semiminor_input, int orientation_input) {
		if(semimajor_input != PositionalAccuracy.ACCURACY_UNAVAILABLE) {
			int integer_semimajor_input = semimajor_input;
			if(integer_semimajor_input >= ACCURACY_MAX) {
				this.accuracy[0] = ACCURACY_MAX;
			} else if(integer_semimajor_input >= ACCURACY_MIN) {
				this.accuracy[0] = integer_semimajor_input;
			}
		}
		if(semiminor_input != PositionalAccuracy.ACCURACY_UNAVAILABLE) {
			int integer_semiminor_input = semiminor_input;
			if(integer_semiminor_input >= ACCURACY_MAX) {
				this.accuracy[1] = ACCURACY_MAX;
			} else if(integer_semiminor_input >= ACCURACY_MIN) {
				this.accuracy[1] = integer_semiminor_input;
			}
		}
		if(orientation_input != PositionalAccuracy.ACCURACY_ORIENTATION_UNAVAILABLE) {
			int integer_orientation_input = orientation_input;
			if(integer_orientation_input >= ACCURACY_ORIENTATION_MIN && integer_orientation_input <= ACCURACY_ORIENTATION_MAX) {
				this.accuracy[2] = integer_orientation_input;
			}
		}
	}

	public void setTransmission(byte transmission_input) {
		this.transmission = transmission_input;
	}

	public void setSpeed(int speed_input) {
		if(speed_input == BSMCoreData.SPEED_UNAVAILABLE) {
			return;
		}
		int integer_speed_input = speed_input;
		if(integer_speed_input >= SPEED_MIN && integer_speed_input <= SPEED_MAX) {
			this.speed = integer_speed_input;
		}
	}

	public void setHeading(int heading_input) {
		if(heading_input == BSMCoreData.HEADING_UNAVAILABLE) {
			return;
		}
		int integer_heading_input = heading_input;
		if(integer_heading_input >= HEADING_MIN && integer_heading_input <= HEADING_MAX) {
			this.heading = integer_heading_input;
		}
	}

	public void setAngle(int angle_input) {
		if(angle_input == BSMCoreData.STEER_WHEEL_ANGLE_UNAVAILABLE) {
			return;
		}
		int integer_angle_input = angle_input;
		if(integer_angle_input >= STEER_WHEEL_ANGLE_MAX) {
			this.angle = STEER_WHEEL_ANGLE_MAX; 
		} else if(integer_angle_input <= STEER_WHEEL_ANGLE_MIN) {
			this.angle = STEER_WHEEL_ANGLE_MIN;
		} else {
			this.angle = integer_angle_input;
		}
	}

	public void setAcceleration(int acceleration_lat_input, int acceleration_lon_input, int acceleration_vert_input, int yaw_rate_input) {
		if(acceleration_lat_input != AccelerationSet4Way.ACCELERATION_UNAVAILABLE) {
			int integer_acceleration_lat_input = acceleration_lat_input;
			if(integer_acceleration_lat_input >= ACCELERATION_MAX) {
				this.acceleration[0] = ACCELERATION_MAX;
			} else if(integer_acceleration_lat_input <= ACCELERATION_MIN) {
				this.acceleration[0] = ACCELERATION_MIN;
			} else {
				this.acceleration[0] = integer_acceleration_lat_input;
			}
		}
		if(acceleration_lon_input != AccelerationSet4Way.ACCELERATION_UNAVAILABLE) {
			int integer_acceleration_lon_input = acceleration_lon_input;
			if(integer_acceleration_lon_input >= ACCELERATION_MAX) {
				this.acceleration[1] = ACCELERATION_MAX;
			} else if(integer_acceleration_lon_input <= ACCELERATION_MIN) {
				this.acceleration[1] = ACCELERATION_MIN;
			} else {
				this.acceleration[1] = integer_acceleration_lon_input;
			}
		}
		if(acceleration_vert_input != AccelerationSet4Way.ACCELERATION_VERTICAL_UNAVAILABLE) {
			int integer_acceleration_vert_input = acceleration_vert_input;
			if(integer_acceleration_vert_input >= ACCELERATION_VERTICAL_MAX) {
				this.acceleration[2] = ACCELERATION_VERTICAL_MAX;
			} else if(integer_acceleration_vert_input <= ACCELERATION_VERTICAL_MIN) {
				this.acceleration[2] = ACCELERATION_VERTICAL_MIN;
			} else {
				this.acceleration[2] = integer_acceleration_vert_input;
			}
		}
		if(yaw_rate_input != AccelerationSet4Way.YAWRATE_UNAVAILABLE) {
			int integer_yaw_rate_input = yaw_rate_input;
			if(integer_yaw_rate_input >= YAWRATE_MIN && integer_yaw_rate_input <= YAWRATE_MAX) {
				this.acceleration[3] = integer_yaw_rate_input; 
			}
		}
	}

	public void setWheel_brakes(byte wheel_brakes_input) {
		if(wheel_brakes_input == (BRAKES_APPLIED >> 3)) {
			this.wheel_brakes = BRAKES_APPLIED;
		} else if(wheel_brakes_input == BRAKES_NOT_APPLIED) {
			this.wheel_brakes = BRAKES_NOT_APPLIED;
		}
	}

	public void setTraction(byte traction_input) {
		this.traction = traction_input;
	}

	public void setAbs(byte abs_input) {
		this.abs = abs_input;
	}

	public void setScs(byte scs_input) {
		this.scs = scs_input;
	}

	public void setBba(byte bba_input) {
		this.bba = bba_input;
	}

	public void setAux(byte aux_input) {
		this.aux = aux_input;
	}

	public void setVehicle_size(int vehicle_width_input, int vehicle_length_input) {
		if(vehicle_width_input != VehicleSize.VEHICLE_WIDTH_UNAVAILABLE) {
			int integer_vehicle_width_input = vehicle_width_input;
			if(integer_vehicle_width_input >= VEHICLE_SIZE_MIN && integer_vehicle_width_input <= VEHICLE_WIDTH_MAX) {
				this.vehicle_size[0] = integer_vehicle_width_input;
			}
		}
		if(vehicle_length_input != VehicleSize.VEHICLE_WIDTH_UNAVAILABLE) {
			int integer_vehicle_length_input = vehicle_length_input;
			if(integer_vehicle_length_input >= VEHICLE_SIZE_MIN && integer_vehicle_length_input <= VEHICLE_LENGTH_MAX) {
				this.vehicle_size[1] = integer_vehicle_length_input;
			}
		}
	}
	
	public int getMsgCnt() {
		return msgCnt;
	}

	public int[] getId() {
		return id;
	}

	public int getSecMark() {
		return secMark;
	}

	public int getLat() {
		return lat;
	}

	public int getLon() {
		return lon;
	}

	public int getElev() {
		return elev;
	}

	public int[] getAccuracy() {
		return accuracy;
	}

	public int getTransmission() {
		return transmission;
	}

	public int getSpeed() {
		return speed;
	}

	public int getHeading() {
		return heading;
	}

	public int getAngle() {
		return angle;
	}

	public int[] getAcceleration() {
		return acceleration;
	}

	public int getWheel_brakes() {
		return wheel_brakes;
	}

	public int getTraction() {
		return traction;
	}

	public int getAbs() {
		return abs;
	}

	public int getScs() {
		return scs;
	}

	public int getBba() {
		return bba;
	}

	public int getAux() {
		return aux;
	}

	public int[] getVehicle_size() {
		return vehicle_size;
	}

	@Override
	public String toString() {
		return "HelperBSM [msgCnt=" + msgCnt + ", id=" + Arrays.toString(id) + ", secMark=" + secMark + ", lat=" + lat
				+ ", Lon=" + lon + ", elev=" + elev + ", accuracy=" + Arrays.toString(accuracy) + ", transmission="
				+ transmission + ", speed=" + speed + ", heading=" + heading + ", angle=" + angle + ", acceleration="
				+ Arrays.toString(acceleration) + ", wheel_brakes=" + wheel_brakes + ", traction="
				+ traction + ", abs=" + abs + ", scs=" + scs + ", bba=" + bba + ", aux=" + aux + ", vehicle_size="
				+ Arrays.toString(vehicle_size) + "]";
	}
	
}
