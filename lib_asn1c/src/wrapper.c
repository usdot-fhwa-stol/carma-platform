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

#include <stdio.h>
#include <sys/types.h>
#include "gov_dot_fhwa_saxton_carma_message_factory_BSMMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityIntroductionMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage.h"
#include "MessageFrame.h"

/**
 * BSM Encoder:
 * This function can encode an BSM object from Java to a byte array in J2735 standards.
 * When an error happened, this function will return NULL.
 * Note: In this function, we pass parameters instead of passing a single object,
 * because making the natives to reach for many individual fields
 * from objects passed to them leads to poor performance.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_BSMMessage_encode_1BSM
		(JNIEnv *env, jobject cls, jint msgCount, jintArray bsm_id, jint secMark,
		  jint lat, jint lon, jint elev, jintArray accuracy_set, jint transmission,
		  jint speed, jint heading, jint angle, jintArray accel_set, jintArray brakes_set, jintArray size_set) {

	uint8_t buffer[128];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if(!message) {
		return NULL;
	}

	//set default fields of BSM
	message -> messageId = 20;
	message -> value.present = MessageFrame__value_PR_BasicSafetyMessage;

	//Set fields
	message -> value.choice.BasicSafetyMessage.coreData.msgCnt = msgCount;

	jint *bsm_msg_id = (*env) -> GetIntArrayElements(env, bsm_id, 0);
	if(bsm_msg_id == NULL) {
		return NULL;
	}
	uint8_t content[4] = {0, 0, 0, 0};
	for(int i = 0; i < 4; i++) {
		content[i] = (char) bsm_msg_id[i];
	}
	message -> value.choice.BasicSafetyMessage.coreData.id.buf = content;
	message -> value.choice.BasicSafetyMessage.coreData.id.size = 4;
	(*env) -> ReleaseIntArrayElements(env, bsm_id, bsm_msg_id, 0);


	message -> value.choice.BasicSafetyMessage.coreData.secMark = secMark;

	message -> value.choice.BasicSafetyMessage.coreData.lat = lat;
	message -> value.choice.BasicSafetyMessage.coreData.Long = lon;
	message -> value.choice.BasicSafetyMessage.coreData.elev = elev;

	jint *accuracy = (*env) -> GetIntArrayElements(env, accuracy_set, 0);
	if(accuracy == NULL) {
		return NULL;
	}
	message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor = accuracy[0];
	message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor = accuracy[1];
	message -> value.choice.BasicSafetyMessage.coreData.accuracy.orientation = accuracy[2];
	(*env) -> ReleaseIntArrayElements(env, accuracy_set, accuracy, 0);

	message -> value.choice.BasicSafetyMessage.coreData.transmission = transmission;
	message -> value.choice.BasicSafetyMessage.coreData.speed = speed;
	message -> value.choice.BasicSafetyMessage.coreData.heading = heading;
	message -> value.choice.BasicSafetyMessage.coreData.angle = angle;

	jint *accel = (*env) -> GetIntArrayElements(env, accel_set, 0);
	if(accel == NULL) {
		return NULL;
	}
	message -> value.choice.BasicSafetyMessage.coreData.accelSet.lat = accel[0];
	message -> value.choice.BasicSafetyMessage.coreData.accelSet.Long = accel[1];
	message -> value.choice.BasicSafetyMessage.coreData.accelSet.vert = accel[2];
	message -> value.choice.BasicSafetyMessage.coreData.accelSet.yaw = accel[3];
	(*env) -> ReleaseIntArrayElements(env, accel_set, accel, 0);

	jint *brakes = (*env) -> GetIntArrayElements(env, brakes_set, 0);
	if(brakes == NULL) {
		return NULL;
	}
	uint8_t break_content[1] = {16};
	break_content[0] = brakes[0];
	message -> value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.bits_unused = 3;
	message -> value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf = break_content;
	message -> value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.size = 1;
	message -> value.choice.BasicSafetyMessage.coreData.brakes.traction = brakes[1];
	message -> value.choice.BasicSafetyMessage.coreData.brakes.abs = brakes[2];
	message -> value.choice.BasicSafetyMessage.coreData.brakes.scs = brakes[3];
	message -> value.choice.BasicSafetyMessage.coreData.brakes.brakeBoost = brakes[4];
	message -> value.choice.BasicSafetyMessage.coreData.brakes.auxBrakes = brakes[5];
	(*env) -> ReleaseIntArrayElements(env, brakes_set, brakes, 0);

	jint *size = (*env) -> GetIntArrayElements(env, size_set, 0);
	if(size == NULL) {
		return NULL;
	}
	message -> value.choice.BasicSafetyMessage.coreData.size.width = size[0];
	message -> value.choice.BasicSafetyMessage.coreData.size.length = size[1];
	(*env) -> ReleaseIntArrayElements(env, size_set, size, 0);

	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		return NULL;
	}

	jsize length = ec.encoded / 8;
	jbyteArray outJNIArray = (*env) -> NewByteArray(env, length);
	if(outJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outJNIArray, 0, length, buffer);
	return outJNIArray;
}

/**
 * BSM Decoder:
 * This function can decode a byte array in J2735 standards to a messageFrame structure and map to a Java BSMCore object.
 * When an error happened, this function will return without any mapping.
 * Return -1 means an error has happened; return 0 means decoding succeed.
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_BSMMessage_decode_1BSM
  (JNIEnv *env, jobject cls, jbyteArray encoded_bsm,
   jobject plain_bsm, jbyteArray bsm_id, jobject accuracy,
   jobject transmission, jobject accelset, jbyteArray brakeStatus, jobject size) {

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Type to decode */

	int len = (*env) -> GetArrayLength(env, encoded_bsm); /* Number of bytes in encoded_bsm */
	jbyte *inCArray = (*env) -> GetByteArrayElements(env, encoded_bsm, 0); /* Get Java byte array content */
	char buf[len]; /* Buffer for decoder function */
	for(int i = 0; i < len; i++) {
		buf[i] = inCArray[i];
	} /* Copy into buffer */

	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

	if(rval.code == RC_OK) {
		jclass bsm_core_class = (*env) -> GetObjectClass(env, plain_bsm);
		jclass accuracy_class = (*env) -> GetObjectClass(env, accuracy);
		jclass transmission_class = (*env) -> GetObjectClass(env, transmission);
		jclass accelset_class = (*env) -> GetObjectClass(env, accelset);
		jclass size_class = (*env) -> GetObjectClass(env, size);

		jmethodID mid_setMsgCount = (*env) -> GetMethodID(env, bsm_core_class, "setMsgCount", "(B)V");
		jbyte msgCount = message -> value.choice.BasicSafetyMessage.coreData.msgCnt;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setMsgCount, msgCount);

		uint8_t *content = message -> value.choice.BasicSafetyMessage.coreData.id.buf;
		(*env) -> SetByteArrayRegion(env, bsm_id, 0, 4, content);

		jmethodID mid_setSecMark = (*env) -> GetMethodID(env, bsm_core_class, "setSecMark", "(I)V");
		jint secMark = message -> value.choice.BasicSafetyMessage.coreData.secMark;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setSecMark, secMark);

		jmethodID mid_setLatitude = (*env) -> GetMethodID(env, bsm_core_class, "setLatitude", "(D)V");
		jdouble lat = message -> value.choice.BasicSafetyMessage.coreData.lat / 10000000.0;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setLatitude, lat);

		jmethodID mid_setLongitude = (*env) -> GetMethodID(env, bsm_core_class, "setLongitude", "(D)V");
		jdouble lon = message -> value.choice.BasicSafetyMessage.coreData.Long / 10000000.0;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setLongitude, lon);

		jmethodID mid_setElev = (*env) -> GetMethodID(env, bsm_core_class, "setElev", "(F)V");
		jfloat elev = message -> value.choice.BasicSafetyMessage.coreData.elev / 10.0;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setElev, elev);

		jmethodID mid_setSemiMajor = (*env) -> GetMethodID(env, accuracy_class, "setSemiMajor", "(F)V");
		jmethodID mid_setSemiMinor = (*env) -> GetMethodID(env, accuracy_class, "setSemiMinor", "(F)V");
		jmethodID mid_setOrientation = (*env) -> GetMethodID(env, accuracy_class, "setOrientation", "(D)V");
		jfloat major = message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor * 0.05;
		jfloat minor = message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor * 0.05;
		jdouble orientation = message -> value.choice.BasicSafetyMessage.coreData.accuracy.orientation * 0.0054932479;
		(*env) -> CallVoidMethod(env, accuracy, mid_setSemiMajor, major);
		(*env) -> CallVoidMethod(env, accuracy, mid_setSemiMinor, minor);
		(*env) -> CallVoidMethod(env, accuracy, mid_setOrientation, orientation);

		jmethodID mid_setTransmissionState = (*env) -> GetMethodID(env, transmission_class, "setTransmissionState", "(B)V");
		jbyte transmissionstate = message -> value.choice.BasicSafetyMessage.coreData.transmission;
		(*env) -> CallVoidMethod(env, transmission, mid_setTransmissionState, transmissionstate);

		jmethodID mid_setSpeed = (*env) -> GetMethodID(env, bsm_core_class, "setSpeed", "(F)V");
		jfloat speed = message -> value.choice.BasicSafetyMessage.coreData.speed * 0.02;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setSpeed, speed);

		jmethodID mid_setHeading = (*env) -> GetMethodID(env, bsm_core_class, "setHeading", "(F)V");
		jfloat heading = message -> value.choice.BasicSafetyMessage.coreData.heading * 0.0125;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setHeading, heading);

		jmethodID mid_setAngle = (*env) -> GetMethodID(env, bsm_core_class, "setAngle", "(F)V");
		jfloat angle = message -> value.choice.BasicSafetyMessage.coreData.angle * 1.5;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setAngle, angle);

		jmethodID mid_accelset_setLongitude = (*env) -> GetMethodID(env, accelset_class, "setLongitudinal", "(F)V");
		jmethodID mid_accelset_setLatitude = (*env) -> GetMethodID(env, accelset_class, "setLateral", "(F)V");
		jmethodID mid_accelset_setVert = (*env) -> GetMethodID(env, accelset_class, "setVert", "(F)V");
		jmethodID mid_accelset_setYaw = (*env) -> GetMethodID(env, accelset_class, "setYawRate", "(F)V");
		jfloat accel_long = message -> value.choice.BasicSafetyMessage.coreData.accelSet.Long * 0.01;
		jfloat accel_lat = message -> value.choice.BasicSafetyMessage.coreData.accelSet.lat * 0.01;
		jfloat accel_vert = message -> value.choice.BasicSafetyMessage.coreData.accelSet.vert * 0.02 * 9.8;
		jfloat accel_yaw = message -> value.choice.BasicSafetyMessage.coreData.accelSet.yaw * 0.01;
		(*env) -> CallVoidMethod(env, accelset, mid_accelset_setLongitude, accel_long);
		(*env) -> CallVoidMethod(env, accelset, mid_accelset_setLatitude, accel_lat);
		(*env) -> CallVoidMethod(env, accelset, mid_accelset_setVert, accel_vert);
		(*env) -> CallVoidMethod(env, accelset, mid_accelset_setYaw, accel_yaw);

		uint8_t brake_content[6];
		uint8_t *brake_buf = message -> value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf;
		brake_content[0] = brake_buf[0];
		brake_content[1] = message -> value.choice.BasicSafetyMessage.coreData.brakes.traction;
		brake_content[2] = message -> value.choice.BasicSafetyMessage.coreData.brakes.abs;
		brake_content[3] = message -> value.choice.BasicSafetyMessage.coreData.brakes.scs;
		brake_content[4] = message->value.choice.BasicSafetyMessage.coreData.brakes.brakeBoost;
		brake_content[5] = message->value.choice.BasicSafetyMessage.coreData.brakes.auxBrakes;
		(*env) -> SetByteArrayRegion(env, brakeStatus, 0, 6, brake_content);

		jmethodID mid_setVehicleWidth = (*env) -> GetMethodID(env, size_class, "setVehicleWidth", "(F)V");
		jmethodID mid_setVehicleLength = (*env) -> GetMethodID(env, size_class, "setVehicleLength", "(F)V");
		jfloat v_width = message->value.choice.BasicSafetyMessage.coreData.size.width / 100.0;
		jfloat v_length = message->value.choice.BasicSafetyMessage.coreData.size.length / 100.0;
		(*env) -> CallVoidMethod(env, size, mid_setVehicleWidth, v_width);
		(*env) -> CallVoidMethod(env, size, mid_setVehicleLength, v_length);
	}
	else {
		return -1;
	}
	return 0;
}

/**
 * Mobility Introduction Encoder:
 * This function can encode an Mobility Introduction message object from Java to
 * a byte array in J2735 standards. When an error happened, this function will return NULL.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityIntroductionMessage_encode_1MobilityIntro
  (JNIEnv *env, jobject cls, jintArray senderId, jintArray targetId,
   jintArray planId, jintArray timestamp, jint vehicleType, jintArray roadwayId,
   jint position, jint laneId, jint speed, jint planType, jint planParam,
   jintArray publicKey, jintArray expiration, jintArray capabilities) {

	uint8_t buffer[256];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if (!message) {
		return NULL;
	}

	//set default value of testmessage00
	message -> messageId = 240;
	message -> value.present = MessageFrame__value_PR_TestMessage00;

	//set senderId in header
	jint *sender_id = (*env) -> GetIntArrayElements(env, senderId, 0);
	if(sender_id == NULL) {
		return NULL;
	}
	uint8_t sender_id_content[16] = {0};
	for(int i = 0; i < 16; i++) {
		sender_id_content[i] = (char) sender_id[i];
	}
	message -> value.choice.TestMessage00.header.hostStaticId.buf = sender_id_content;
	message -> value.choice.TestMessage00.header.hostStaticId.size = 16;
	(*env) -> ReleaseIntArrayElements(env, senderId, sender_id, 0);

	//set targetId in header
	jint *target_id = (*env) -> GetIntArrayElements(env, targetId, 0);
	if (target_id == NULL) {
		return NULL;
	}
	uint8_t target_id_content[16] = { 0 };
	for (int i = 0; i < 16; i++) {
		target_id_content[i] = (char) target_id[i];
	}
	message -> value.choice.TestMessage00.header.targetStaticId.buf = target_id_content;
	message -> value.choice.TestMessage00.header.targetStaticId.size = 16;
	(*env) -> ReleaseIntArrayElements(env, targetId, target_id, 0);

	//set planId in header
	jint *plan_id = (*env) -> GetIntArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[16] = { 0 };
	for (int i = 0; i < 16; i++) {
		plan_id_content[i] = (char) plan_id[i];
	}
	message -> value.choice.TestMessage00.header.planId.buf = plan_id_content;
	message -> value.choice.TestMessage00.header.planId.size = 16;
	(*env) -> ReleaseIntArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jint *time_stamp = (*env) -> GetIntArrayElements(env, timestamp, 0);
	if(time_stamp == NULL) {
		return NULL;
	}
	DYear_t year = time_stamp[0];
	DMonth_t month = time_stamp[1];
	DDay_t day = time_stamp[2];
	DHour_t	hour = time_stamp[3];
	DMinute_t minute = time_stamp[4];
	DSecond_t second = time_stamp[5];
	DOffset_t offset = time_stamp[6];
	message -> value.choice.TestMessage00.header.timestamp.year = &year;
	message -> value.choice.TestMessage00.header.timestamp.month = &month;
	message -> value.choice.TestMessage00.header.timestamp.day = &day;
	message -> value.choice.TestMessage00.header.timestamp.hour = &hour;
	message -> value.choice.TestMessage00.header.timestamp.minute = &minute;
	message -> value.choice.TestMessage00.header.timestamp.second = &second;
	message -> value.choice.TestMessage00.header.timestamp.offset = &offset;

	//set entity type
	message -> value.choice.TestMessage00.body.vehicleType = vehicleType;

	//set roadway link ID
	int roadway_id_size = (*env) -> GetArrayLength(env, roadwayId);
	jint *roadway_id = (*env) -> GetIntArrayElements(env, roadwayId, 0);
	if (roadway_id == NULL) {
		return NULL;
	}
	uint8_t roadway_id_content[roadway_id_size];
	for (int i = 0; i < roadway_id_size; i++) {
		roadway_id_content[i] = (char) roadway_id[i];
	}
	message -> value.choice.TestMessage00.body.roadwayId.buf = roadway_id_content;
	message -> value.choice.TestMessage00.body.roadwayId.size = roadway_id_size;
	(*env) -> ReleaseIntArrayElements(env, roadwayId, roadway_id, 0);

	//set roadway position
	message -> value.choice.TestMessage00.body.roadwayPosition = position;

	//set lane id
	message -> value.choice.TestMessage00.body.lane = laneId;

	//set forward speed
	message -> value.choice.TestMessage00.body.speed = speed;

	//set plan type and param
	message -> value.choice.TestMessage00.body.planType = planType;
	message -> value.choice.TestMessage00.body.planParam = planParam;

	//set public key
	jint *public_key = (*env)->GetIntArrayElements(env, publicKey, 0);
	if (public_key == NULL) {
		return NULL;
	}
	uint8_t public_key_content[64] = { 0 };
	for (int i = 0; i < 64; i++) {
		public_key_content[i] = (char) public_key[i];
	}
	message->value.choice.TestMessage00.body.publicKey.buf = public_key_content;
	message->value.choice.TestMessage00.body.publicKey.size = 64;
	(*env)->ReleaseIntArrayElements(env, publicKey, public_key, 0);

	//set expiration date & time
	jint *expiration_time = (*env) -> GetIntArrayElements(env, expiration, 0);
	if(expiration_time == NULL) {
		return NULL;
	}
	DYear_t expiration_year = expiration_time[0];
	DMonth_t expiration_month = expiration_time[1];
	DDay_t expiration_day = expiration_time[2];
	DHour_t	expiration_hour = expiration_time[3];
	DMinute_t expiration_minute = expiration_time[4];
	DSecond_t expiration_second = expiration_time[5];
	DOffset_t expiration_offset = expiration_time[6];
	message -> value.choice.TestMessage00.body.expiration.year = &expiration_year;
	message -> value.choice.TestMessage00.body.expiration.month = &expiration_month;
	message -> value.choice.TestMessage00.body.expiration.day = &expiration_day;
	message -> value.choice.TestMessage00.body.expiration.hour = &expiration_hour;
	message -> value.choice.TestMessage00.body.expiration.minute = &expiration_minute;
	message -> value.choice.TestMessage00.body.expiration.second = &expiration_second;
	message -> value.choice.TestMessage00.body.expiration.offset = &expiration_offset;

	//set capabilities string
	int capabilities_string_size = (*env) -> GetArrayLength(env, capabilities);
	jint *capabilities_string = (*env)->GetIntArrayElements(env, capabilities, 0);
	if (capabilities_string == NULL) {
		return NULL;
	}
	uint8_t capabilities_string_content[capabilities_string_size];
	for (int i = 0; i < capabilities_string_size; i++) {
		capabilities_string_content[i] = (char) capabilities_string[i];
	}
	message -> value.choice.TestMessage00.body.capabilities.buf = capabilities_string_content;
	message -> value.choice.TestMessage00.body.capabilities.size = capabilities_string_size;
	(*env) -> ReleaseIntArrayElements(env, capabilities, capabilities_string, 0);

	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		return NULL;
	}

	//copy back to output
	jsize length = ec.encoded / 8;
	jbyteArray outJNIArray = (*env) -> NewByteArray(env, length);
	if(outJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outJNIArray, 0, length, buffer);
	return outJNIArray;
}

/**
 * Mobility Intro Decoder:
 * This function can decode a byte array in J2735 standards to
 * a messageFrame structure and map to a ROS MobilityIntro object.
 * Return -1 means an error has happened; return 0 means decoding succeed.
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityIntroductionMessage_decode_1MobilityIntro
  (JNIEnv *env, jobject cls, jbyteArray encodedIntro, jobject introMsg,
   jbyteArray senderId, jbyteArray targetId, jbyteArray planId,
   jintArray dateTime, jobject vehicleType, jbyteArray roadwayId,
   jobject planType, jbyteArray publicKey, jintArray expiration, jbyteArray capabilities) {

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Type to decode */

	int len = (*env) -> GetArrayLength(env, encodedIntro); /* Number of bytes in encoded_bsm */
	jbyte *inCArray = (*env) -> GetByteArrayElements(env, encodedIntro, 0); /* Get Java byte array content */
	char buf[len]; /* Buffer for decoder function */
	for (int i = 0; i < len; i++) {
		buf[i] = inCArray[i];
	} /* Copy into buffer */

	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {
		//get jclass of each jobject
		jclass mobility_class = (*env) -> GetObjectClass(env, introMsg);
		jclass vehicle_type_class = (*env) -> GetObjectClass(env, vehicleType);
		jclass plan_type_class = (*env) -> GetObjectClass(env, planType);

		//set senderId, targetId and planId array
		uint8_t *sender_id_content = message -> value.choice.TestMessage00.header.hostStaticId.buf;
		(*env) -> SetByteArrayRegion(env, senderId, 0, 16, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage00.header.targetStaticId.buf;
		(*env) -> SetByteArrayRegion(env, targetId, 0, 16, target_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage00.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 16, plan_id_content);

		//set message creation dateTime
		int time_content[7] = {0};
		time_content[0] = *(message -> value.choice.TestMessage00.header.timestamp.year);
		time_content[1] = *(message -> value.choice.TestMessage00.header.timestamp.month);
		time_content[2] = *(message -> value.choice.TestMessage00.header.timestamp.day);
		time_content[3] = *(message -> value.choice.TestMessage00.header.timestamp.hour);
		time_content[4] = *(message -> value.choice.TestMessage00.header.timestamp.minute);
		time_content[5] = *(message -> value.choice.TestMessage00.header.timestamp.second);
		time_content[6] = *(message -> value.choice.TestMessage00.header.timestamp.offset);
		(*env) -> SetIntArrayRegion(env, dateTime, 0, 7, time_content);

		//set vehicle type
		jmethodID mid_setType = (*env) -> GetMethodID(env, vehicle_type_class, "setType", "(B)V");
		jbyte vehicle_type = message -> value.choice.TestMessage00.body.vehicleType;
		(*env) -> CallVoidMethod(env, vehicleType, mid_setType, vehicle_type);

		//set roadway link id
		uint8_t *roadway_id_content = message -> value.choice.TestMessage00.body.roadwayId.buf;
		int roadway_id_size = message -> value.choice.TestMessage00.body.roadwayId.size;
		(*env) -> SetByteArrayRegion(env, roadwayId, 0, roadway_id_size, roadway_id_content);

		//set roadway link position
		jmethodID mid_setPosition = (*env) -> GetMethodID(env, mobility_class, "setMyRoadwayLinkPosition", "(S)V");
		jshort vehicle_position = message -> value.choice.TestMessage00.body.roadwayPosition;
		(*env) -> CallVoidMethod(env, introMsg, mid_setPosition, vehicle_position);

		//set lane id
		jmethodID mid_setLane = (*env) -> GetMethodID(env, mobility_class, "setMyLaneId", "(B)V");
		jbyte lane_id = message -> value.choice.TestMessage00.body.lane;
		(*env) -> CallVoidMethod(env, introMsg, mid_setLane, lane_id);

		//set forward speed
		jmethodID mid_setSpeed = (*env) -> GetMethodID(env, mobility_class, "setForwardSpeed", "(F)V");
		jfloat forward_speed = message -> value.choice.TestMessage00.body.speed * 0.02;
		(*env) -> CallVoidMethod(env, introMsg, mid_setSpeed, forward_speed);

		//set plan type
		jmethodID mid_setPlanType = (*env) -> GetMethodID(env, plan_type_class, "setType", "(B)V");
		jbyte plan_type = message -> value.choice.TestMessage00.body.planType;
		(*env) -> CallVoidMethod(env, planType, mid_setPlanType, plan_type);

		//set plan parameter
		jmethodID mid_setParam = (*env) -> GetMethodID(env, mobility_class, "setProposalParam", "(S)V");
		jshort proposal_param = message -> value.choice.TestMessage00.body.planParam;
		(*env) -> CallVoidMethod(env, introMsg, mid_setParam, proposal_param);

		//set public key
		uint8_t *public_key_content = message -> value.choice.TestMessage00.body.publicKey.buf;
		(*env) -> SetByteArrayRegion(env, publicKey, 0, 64, public_key_content);

		//set expiration
		int expiration_content[7] = {0};
		expiration_content[0] = *(message -> value.choice.TestMessage00.body.expiration.year);
		expiration_content[1] = *(message -> value.choice.TestMessage00.body.expiration.month);
		expiration_content[2] = *(message -> value.choice.TestMessage00.body.expiration.day);
		expiration_content[3] = *(message -> value.choice.TestMessage00.body.expiration.hour);
		expiration_content[4] = *(message -> value.choice.TestMessage00.body.expiration.minute);
		expiration_content[5] = *(message -> value.choice.TestMessage00.body.expiration.second);
		expiration_content[6] = *(message -> value.choice.TestMessage00.body.expiration.offset);
		(*env) -> SetIntArrayRegion(env, expiration, 0, 7, expiration_content);

		//set capabilities string
		uint8_t *capabilities_content = message -> value.choice.TestMessage00.body.capabilities.buf;
		int capabilities_size = message -> value.choice.TestMessage00.body.capabilities.size;
		(*env) -> SetByteArrayRegion(env, capabilities, 0, capabilities_size, capabilities_content);
	} else {
		return -1; /* decoding fails */
	}
	return 0;
}

/**
 * Mobility Ack Encoder:
 * This function can encode an Mobility Ack message object from Java to
 * a byte array in J2735 standards. When an error happened, this function will return NULL.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage_encode_1MobilityAck
  (JNIEnv *env, jobject cls, jintArray senderId, jintArray targetId,
   jintArray planId, jintArray timestamp, jint ackType, jbyteArray verification) {

	uint8_t buffer[256];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if (!message) {
		return NULL;
	}

	//set default value of testmessage02
	message->messageId = 242;
	message->value.present = MessageFrame__value_PR_TestMessage02;

	//set senderId in header
	jint *sender_id = (*env)->GetIntArrayElements(env, senderId, 0);
	if (sender_id == NULL) {
		return NULL;
	}
	uint8_t sender_id_content[16] = { 0 };
	for (int i = 0; i < 16; i++) {
		sender_id_content[i] = (char) sender_id[i];
	}
	message->value.choice.TestMessage02.header.hostStaticId.buf = sender_id_content;
	message->value.choice.TestMessage02.header.hostStaticId.size = 16;
	(*env)->ReleaseIntArrayElements(env, senderId, sender_id, 0);

	//set targetId in header
	jint *target_id = (*env)->GetIntArrayElements(env, targetId, 0);
	if (target_id == NULL) {
		return NULL;
	}
	uint8_t target_id_content[16] = { 0 };
	for (int i = 0; i < 16; i++) {
		target_id_content[i] = (char) target_id[i];
	}
	message->value.choice.TestMessage02.header.targetStaticId.buf = target_id_content;
	message->value.choice.TestMessage02.header.targetStaticId.size = 16;
	(*env)->ReleaseIntArrayElements(env, targetId, target_id, 0);

	//set planId in header
	jint *plan_id = (*env)->GetIntArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[16] = { 0 };
	for (int i = 0; i < 16; i++) {
		plan_id_content[i] = (char) plan_id[i];
	}
	message->value.choice.TestMessage02.header.planId.buf = plan_id_content;
	message->value.choice.TestMessage02.header.planId.size = 16;
	(*env)->ReleaseIntArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jint *time_stamp = (*env)->GetIntArrayElements(env, timestamp, 0);
	if (time_stamp == NULL) {
		return NULL;
	}
	DYear_t year = time_stamp[0];
	DMonth_t month = time_stamp[1];
	DDay_t day = time_stamp[2];
	DHour_t hour = time_stamp[3];
	DMinute_t minute = time_stamp[4];
	DSecond_t second = time_stamp[5];
	DOffset_t offset = time_stamp[6];
	message->value.choice.TestMessage02.header.timestamp.year = &year;
	message->value.choice.TestMessage02.header.timestamp.month = &month;
	message->value.choice.TestMessage02.header.timestamp.day = &day;
	message->value.choice.TestMessage02.header.timestamp.hour = &hour;
	message->value.choice.TestMessage02.header.timestamp.minute = &minute;
	message->value.choice.TestMessage02.header.timestamp.second = &second;
	message->value.choice.TestMessage02.header.timestamp.offset = &offset;

	//set ack type
	message->value.choice.TestMessage02.body.ackType = ackType;

	//set verification string if we have it
	int verification_string_size = (*env) -> GetArrayLength(env, verification);
	if(verification_string_size != 0) {
		jbyte *verification_string = (*env)->GetByteArrayElements(env, verification, 0);
		if (verification_string == NULL) {
			return NULL;
		}
		uint8_t verification_string_content[8];
		for (int i = 0; i < 8; i++) {
			verification_string_content[i] = (char) verification_string[i];
		}
		message -> value.choice.TestMessage02.body.verification -> buf = verification_string_content;
		message -> value.choice.TestMessage02.body.verification -> size = 8;
	}


	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		return NULL;
	}

	//copy back to output
	jsize length = ec.encoded / 8;
	jbyteArray outJNIArray = (*env) -> NewByteArray(env, length);
	if(outJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outJNIArray, 0, length, buffer);
	return outJNIArray;
}

/**
 * Mobility Ack Decoder:
 * This function can decode a byte array in J2735 standards to
 * a messageFrame structure and map to a ROS MobilityIntro object.
 * Return -1 means an error has happened; return 0 means decoding succeed.
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityAckMessage_decode_1MobilityAck
  (JNIEnv *env, jobject cls, jbyteArray encodedAck, jbyteArray senderId,
   jbyteArray targetId, jbyteArray planId, jintArray dateTime, jobject ackType, jbyteArray verification) {

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Type to decode */

	int len = (*env)->GetArrayLength(env, encodedAck); /* Number of bytes in encoded_bsm */
	jbyte *inCArray = (*env)->GetByteArrayElements(env, encodedAck, 0); /* Get Java byte array content */
	char buf[len]; /* Buffer for decoder function */
	for (int i = 0; i < len; i++) {
		buf[i] = inCArray[i];
	} /* Copy into buffer */

	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

	if(rval.code == RC_OK) {
		jclass ackType_class = (*env) -> GetObjectClass(env, ackType);

		//set senderId, targetId and planId array
		uint8_t *sender_id_content = message -> value.choice.TestMessage02.header.hostStaticId.buf;
		(*env) -> SetByteArrayRegion(env, senderId, 0, 16, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage02.header.targetStaticId.buf;
		(*env) -> SetByteArrayRegion(env, targetId, 0, 16, target_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage02.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 16, plan_id_content);

		//set message creation dateTime
		int time_content[7] = {0};
		time_content[0] = *(message -> value.choice.TestMessage02.header.timestamp.year);
		time_content[1] = *(message -> value.choice.TestMessage02.header.timestamp.month);
		time_content[2] = *(message -> value.choice.TestMessage02.header.timestamp.day);
		time_content[3] = *(message -> value.choice.TestMessage02.header.timestamp.hour);
		time_content[4] = *(message -> value.choice.TestMessage02.header.timestamp.minute);
		time_content[5] = *(message -> value.choice.TestMessage02.header.timestamp.second);
		time_content[6] = *(message -> value.choice.TestMessage02.header.timestamp.offset);
		(*env) -> SetIntArrayRegion(env, dateTime, 0, 7, time_content);

		jmethodID mid_setAckType = (*env) -> GetMethodID(env, ackType_class, "setType", "(B)V");
		jbyte type = message -> value.choice.TestMessage02.body.ackType;
		(*env) -> CallVoidMethod(env, ackType, mid_setAckType, type);

		//TODO add verification string later
	} else {
		return -1;
	}
	return 0;
}

