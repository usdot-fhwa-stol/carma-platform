/*
 * Copyright (C) 2017-2020 LEIDOS.
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
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityRequestMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityPathMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityResponseMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MobilityOperationMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_MapMessage.h"
#include "gov_dot_fhwa_saxton_carma_message_factory_SPATMessage.h"
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

		jmethodID mid_setSecMark = (*env) -> GetMethodID(env, bsm_core_class, "setSecMark", "(S)V");
		jshort secMark = message -> value.choice.BasicSafetyMessage.coreData.secMark;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setSecMark, secMark);

		jmethodID mid_setLatitude = (*env) -> GetMethodID(env, bsm_core_class, "setLatitude", "(I)V");
		jint lat = message -> value.choice.BasicSafetyMessage.coreData.lat;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setLatitude, lat);

		jmethodID mid_setLongitude = (*env) -> GetMethodID(env, bsm_core_class, "setLongitude", "(I)V");
		jint lon = message -> value.choice.BasicSafetyMessage.coreData.Long;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setLongitude, lon);

		jmethodID mid_setElev = (*env) -> GetMethodID(env, bsm_core_class, "setElev", "(I)V");
		jint elev = message -> value.choice.BasicSafetyMessage.coreData.elev;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setElev, elev);

		jmethodID mid_setSemiMajor = (*env) -> GetMethodID(env, accuracy_class, "setSemiMajor", "(B)V");
		jmethodID mid_setSemiMinor = (*env) -> GetMethodID(env, accuracy_class, "setSemiMinor", "(B)V");
		jmethodID mid_setOrientation = (*env) -> GetMethodID(env, accuracy_class, "setOrientation", "(S)V");
		jbyte major = message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor;
		jbyte minor = message -> value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor;
		jshort orientation = message -> value.choice.BasicSafetyMessage.coreData.accuracy.orientation;
		(*env) -> CallVoidMethod(env, accuracy, mid_setSemiMajor, major);
		(*env) -> CallVoidMethod(env, accuracy, mid_setSemiMinor, minor);
		(*env) -> CallVoidMethod(env, accuracy, mid_setOrientation, orientation);

		jmethodID mid_setTransmissionState = (*env) -> GetMethodID(env, transmission_class, "setTransmissionState", "(B)V");
		jbyte transmissionstate = message -> value.choice.BasicSafetyMessage.coreData.transmission;
		(*env) -> CallVoidMethod(env, transmission, mid_setTransmissionState, transmissionstate);

		jmethodID mid_setSpeed = (*env) -> GetMethodID(env, bsm_core_class, "setSpeed", "(S)V");
		jshort speed = message -> value.choice.BasicSafetyMessage.coreData.speed;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setSpeed, speed);

		jmethodID mid_setHeading = (*env) -> GetMethodID(env, bsm_core_class, "setHeading", "(S)V");
		jshort heading = message -> value.choice.BasicSafetyMessage.coreData.heading;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setHeading, heading);

		jmethodID mid_setAngle = (*env) -> GetMethodID(env, bsm_core_class, "setAngle", "(B)V");
		jbyte angle = message -> value.choice.BasicSafetyMessage.coreData.angle;
		(*env) -> CallVoidMethod(env, plain_bsm, mid_setAngle, angle);

		jmethodID mid_accelset_setLongitude = (*env) -> GetMethodID(env, accelset_class, "setLongitudinal", "(S)V");
		jmethodID mid_accelset_setLatitude = (*env) -> GetMethodID(env, accelset_class, "setLateral", "(S)V");
		jmethodID mid_accelset_setVert = (*env) -> GetMethodID(env, accelset_class, "setVert", "(B)V");
		jmethodID mid_accelset_setYaw = (*env) -> GetMethodID(env, accelset_class, "setYawRate", "(S)V");
		jshort accel_long = message -> value.choice.BasicSafetyMessage.coreData.accelSet.Long;
		jshort accel_lat = message -> value.choice.BasicSafetyMessage.coreData.accelSet.lat;
		jbyte accel_vert = message -> value.choice.BasicSafetyMessage.coreData.accelSet.vert;
		jshort accel_yaw = message -> value.choice.BasicSafetyMessage.coreData.accelSet.yaw;
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

		jmethodID mid_setVehicleWidth = (*env) -> GetMethodID(env, size_class, "setVehicleWidth", "(S)V");
		jmethodID mid_setVehicleLength = (*env) -> GetMethodID(env, size_class, "setVehicleLength", "(S)V");
		jshort v_width = message->value.choice.BasicSafetyMessage.coreData.size.width;
		jshort v_length = message->value.choice.BasicSafetyMessage.coreData.size.length;
		(*env) -> CallVoidMethod(env, size, mid_setVehicleWidth, v_width);
		(*env) -> CallVoidMethod(env, size, mid_setVehicleLength, v_length);
	}
	else {
		return -1;
	}
	return 0;
}

/**
 * MobilityRequest Encoder:
 * This function can encode an MobilityRequest message object from Java to
 * a byte array in J2735 standards. When an error happened, this function will return NULL.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityRequestMessage_encodeMobilityRequest
  (JNIEnv *env, jobject cls, jbyteArray senderId, jbyteArray targetId, jbyteArray senderBSMId,
   jbyteArray planId, jbyteArray timestamp, jbyteArray strategy, jint planType, jint urgency,
   jint currentX, jint currentY, jint currentZ, jbyteArray currentT, jbyteArray strategyParams,
   jint startX, jint startY, jint startZ, jbyteArray startT, jobjectArray offsets, jbyteArray expiration) {

	//Build a log in test file to debug if necessary
	//FILE *fp;
	//fp = fopen("/home/qsw/carma/log_C.txt", "w");
	//fprintf(fp, "encodeMobilityRequest function is called\n");

	uint8_t buffer[512];
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
	jsize sender_string_size = (*env) -> GetArrayLength(env, senderId);
	jbyte *sender_string = (*env) -> GetByteArrayElements(env, senderId, 0);
	if (sender_string == NULL) {
		return NULL;
	}
	uint8_t sender_string_content[sender_string_size];
	for (int i = 0; i < sender_string_size; i++) {
		sender_string_content[i] = sender_string[i];
	}
	message -> value.choice.TestMessage00.header.hostStaticId.buf = sender_string_content;
	message -> value.choice.TestMessage00.header.hostStaticId.size = (size_t) sender_string_size;
	(*env) -> ReleaseByteArrayElements(env, senderId, sender_string, 0);

	//set targetId in header
	jsize target_string_size = (*env) -> GetArrayLength(env, targetId);
	jbyte *target_string = (*env) -> GetByteArrayElements(env, targetId, 0);
	if (target_string == NULL) {
		return NULL;
	}
	uint8_t target_string_content[target_string_size];
	for (int i = 0; i < target_string_size; i++) {
		target_string_content[i] = target_string[i];
	}
	message -> value.choice.TestMessage00.header.targetStaticId.buf = target_string_content;
	message -> value.choice.TestMessage00.header.targetStaticId.size = (size_t) target_string_size;
	(*env) -> ReleaseByteArrayElements(env, targetId, target_string, 0);

	//set hostBSMId in header
	jbyte *bsm_string = (*env) -> GetByteArrayElements(env, senderBSMId, 0);
	if(bsm_string == NULL) {
	    return NULL;
	}
	uint8_t host_bsm_id_content[8] = {0};
	for(int i = 0; i < 8; i++) {
		host_bsm_id_content[i] = bsm_string[i];
	}
	message -> value.choice.TestMessage00.header.hostBSMId.buf = host_bsm_id_content;
	message -> value.choice.TestMessage00.header.hostBSMId.size = 8;
	(*env) -> ReleaseByteArrayElements(env, senderBSMId, bsm_string, 0);

	//set planId in header
	jbyte *plan_id = (*env) -> GetByteArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[36] = {0};
	for (int i = 0; i < 36; i++) {
		plan_id_content[i] = plan_id[i];
	}
	message -> value.choice.TestMessage00.header.planId.buf = plan_id_content;
	message -> value.choice.TestMessage00.header.planId.size = 36;
	(*env) -> ReleaseByteArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jbyte *time = (*env) -> GetByteArrayElements(env, timestamp, 0);
	if (time == NULL) {
		return NULL;
	}
	uint8_t time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		time_content[i] = time[i];
	}
	message -> value.choice.TestMessage00.header.timestamp.buf = time_content;
	message -> value.choice.TestMessage00.header.timestamp.size = 19;
	(*env) -> ReleaseByteArrayElements(env, timestamp, time, 0);

	//set strategy string
	jsize strategy_string_size = (*env) -> GetArrayLength(env, strategy);
	jbyte *strategy_string = (*env) -> GetByteArrayElements(env, strategy, 0);
	if (strategy_string == NULL) {
		return NULL;
	}
	uint8_t strategy_string_content[strategy_string_size];
	for (int i = 0; i < strategy_string_size; i++) {
		strategy_string_content[i] = strategy_string[i];
	}
	message -> value.choice.TestMessage00.body.strategy.buf = strategy_string_content;
	message -> value.choice.TestMessage00.body.strategy.size = (size_t) strategy_string_size;
	(*env) -> ReleaseByteArrayElements(env, strategy, strategy_string, 0);

	//set plan type
	message -> value.choice.TestMessage00.body.planType = planType;

	//set plan urgency
	message -> value.choice.TestMessage00.body.urgency = urgency;

	//set current location
	message -> value.choice.TestMessage00.body.location.ecefX = currentX;
	message -> value.choice.TestMessage00.body.location.ecefY = currentY;
	message -> value.choice.TestMessage00.body.location.ecefZ = currentZ;
	jbyte *current_time = (*env) -> GetByteArrayElements(env, currentT, 0);
	if(current_time == NULL) {
		return NULL;
	}
	uint8_t current_time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		current_time_content[i] = current_time[i];
	}
	message -> value.choice.TestMessage00.body.location.timestamp.buf = current_time_content;
	message -> value.choice.TestMessage00.body.location.timestamp.size = 19;
	(*env) -> ReleaseByteArrayElements(env, currentT, current_time, 0);

	//set strategy parameters
	jsize params_string_size = (*env) -> GetArrayLength(env, strategyParams);
	jbyte *params_string = (*env) -> GetByteArrayElements(env, strategyParams, 0);
	if (params_string == NULL) {
		return NULL;
	}
	uint8_t params_string_content[params_string_size];
	for (int i = 0; i < params_string_size; i++) {
		params_string_content[i] = params_string[i];
	}
	message -> value.choice.TestMessage00.body.strategyParams.buf = params_string_content;
	message -> value.choice.TestMessage00.body.strategyParams.size = (size_t) params_string_size;
	(*env) -> ReleaseByteArrayElements(env, strategyParams, params_string, 0);

	//The following fields are optional
	if(startX != 0 || startY != 0 || startZ != 0) {
		MobilityLocation_t *location;
		location = calloc(1, sizeof(MobilityLocation_t));
		location -> ecefX = startX;
		location -> ecefY = startY;
		location -> ecefZ = startZ;
		jbyte *start_time = (*env) -> GetByteArrayElements(env, startT, 0);
		uint8_t start_time_content[19] = {0};
		for (int i = 0; i < 19; i++) {
			start_time_content[i] = start_time[i];
		}
		location -> timestamp.buf = start_time_content;
		location -> timestamp.size = 19;
		message -> value.choice.TestMessage00.body.trajectoryStart = location;
		(*env) -> ReleaseByteArrayElements(env, startT, start_time, 0);

		// TODO handle ObjectArray
		jsize dim = (*env) -> GetArrayLength(env, offsets);
		if(dim == 3) {
			jintArray offsets_X =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 0);
			jintArray offsets_Y =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 1);
			jintArray offsets_Z =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 2);
			jsize count = (*env) -> GetArrayLength(env, offsets_X);
			jint *java_offsets_X = (*env) -> GetIntArrayElements(env, offsets_X, 0);
			jint *java_offsets_Y = (*env) -> GetIntArrayElements(env, offsets_Y, 0);
			jint *java_offsets_Z = (*env) -> GetIntArrayElements(env, offsets_Z, 0);
			if(count > 0) {
				int *localArray[3];
				int offsets_X_content[count];
				int offsets_Y_content[count];
				int offsets_Z_content[count];
				for(int i = 0; i < count; i++) {
					offsets_X_content[i] = java_offsets_X[i];
					offsets_Y_content[i] = java_offsets_Y[i];
					offsets_Z_content[i] = java_offsets_Z[i];
				}
				localArray[0] = offsets_X_content;
				localArray[1] = offsets_Y_content;
				localArray[2] = offsets_Z_content;
				MobilityLocationOffsets_t *trajectory_offsets;
				trajectory_offsets = calloc(1, sizeof(MobilityLocationOffsets_t));
				for(int i = 0; i < count; i++) {
					MobilityECEFOffset_t *offset_point;
					offset_point = calloc(1, sizeof(MobilityECEFOffset_t));
					offset_point -> offsetX = localArray[0][i];
					offset_point -> offsetY = localArray[1][i];
					offset_point -> offsetZ = localArray[2][i];
					asn_sequence_add(&trajectory_offsets -> list, offset_point);
				}
				message -> value.choice.TestMessage00.body.trajectory = trajectory_offsets;
			}
			(*env) -> ReleaseIntArrayElements(env, offsets_X, java_offsets_X, 0);
			(*env) -> ReleaseIntArrayElements(env, offsets_Y, java_offsets_Y, 0);
			(*env) -> ReleaseIntArrayElements(env, offsets_Z, java_offsets_Z, 0);
			(*env) -> DeleteLocalRef(env, offsets_X);
			(*env) -> DeleteLocalRef(env, offsets_Y);
			(*env) -> DeleteLocalRef(env, offsets_Z);
		}
	}

	// set expiration if we need
	int hasExpiration = 0;
	jbyte *expiration_time = (*env) -> GetByteArrayElements(env, expiration, 0);
	if(expiration_time == NULL) {
		return NULL;
	}
	uint8_t expiration_time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		expiration_time_content[i] = expiration_time[i];
		// check if there is a non-zero value
		if(expiration_time_content[i] != 48) {
			hasExpiration = 1;
		}
	}
	if(hasExpiration != 0) {
		MobilityTimestamp_t *expiration_time_in_C;
		expiration_time_in_C = calloc(1, sizeof(MobilityTimestamp_t));
		expiration_time_in_C -> buf = expiration_time_content;
		expiration_time_in_C -> size = 19;
		message -> value.choice.TestMessage00.body.expiration = expiration_time_in_C;
	}
	(*env) -> ReleaseByteArrayElements(env, expiration, expiration_time, 0);

	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		//fprintf(fp, "!!!%s", ec.failed_type->name);
		return NULL;
	}

	//copy back to java output
	jsize length = ec.encoded / 8;
	jbyteArray outputJNIArray = (*env) -> NewByteArray(env, length);
	if(outputJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outputJNIArray, 0, length, buffer);
	return outputJNIArray;
}

/**
 * Mobility Reuqest Decoder:
 * This function can decode a byte array in J2735 standards to
 * a messageFrame structure and map to a ROS MobilityRequest object.
 * Return -1 means an error has happened; return 0 means decoding succeed.
 */

JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityRequestMessage_decodeMobilityRequest
  (JNIEnv *env, jobject cls, jbyteArray encodedReq, jobject request, jbyteArray senderId, jbyteArray targetId,
   jbyteArray bsmId, jbyteArray planId, jbyteArray timestamp, jbyteArray strategy, jobject planType,
   jobject location, jbyteArray locationTimestamp, jbyteArray strategyParams, jobject startLocation,
   jbyteArray startTimestamp, jobjectArray trajectoryOffsets, jbyteArray expiration) {

	//Build a log in test file to debug if necessary
	//FILE *fp;
	//fp = fopen("/home/qsw/carma/log_C.txt", "w");
	//fprintf(fp, "decodeMobilityRequest function is called\n");

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedReq); /* Number of bytes in encoded mobility request */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedReq, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
	    buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

	if(rval.code == RC_OK) {
		//get jclass of each jobject
		jclass mobility_class = (*env) -> GetObjectClass(env, request);
		jclass plan_type_class = (*env) -> GetObjectClass(env, planType);
		jclass current_location_class = (*env) -> GetObjectClass(env, location);
		jclass start_location_class = (*env) -> GetObjectClass(env, startLocation);

		//set senderId, targetId, bsmId, planId and creation timestamp
		uint8_t *sender_id_content = message -> value.choice.TestMessage00.header.hostStaticId.buf;
		size_t sender_id_size = message -> value.choice.TestMessage00.header.hostStaticId.size;
		(*env) -> SetByteArrayRegion(env, senderId, 0, sender_id_size, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage00.header.targetStaticId.buf;
		size_t target_id_size = message -> value.choice.TestMessage00.header.targetStaticId.size;
		(*env) -> SetByteArrayRegion(env, targetId, 0, target_id_size, target_id_content);
		uint8_t *bsm_id_content = message -> value.choice.TestMessage00.header.hostBSMId.buf;
		(*env) -> SetByteArrayRegion(env, bsmId, 0, 8, bsm_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage00.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 36, plan_id_content);
		uint8_t *creation_time_content = message -> value.choice.TestMessage00.header.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, timestamp, 0, 19, creation_time_content);

		//set strategy string
		uint8_t *strategy_content = message -> value.choice.TestMessage00.body.strategy.buf;
		size_t strategy_size = message -> value.choice.TestMessage00.body.strategy.size;
		(*env) -> SetByteArrayRegion(env, strategy, 0, strategy_size, strategy_content);

		//set plan type
		jmethodID mid_setPlanType = (*env) -> GetMethodID(env, plan_type_class, "setType", "(B)V");
		jbyte plan_type = message -> value.choice.TestMessage00.body.planType;
		(*env) -> CallVoidMethod(env, planType, mid_setPlanType, plan_type);

		//set urgency
		jmethodID mid_setUrgency = (*env) -> GetMethodID(env, mobility_class, "setUrgency", "(S)V");
		jshort urgency_value = message -> value.choice.TestMessage00.body.urgency;
		(*env) -> CallVoidMethod(env, request, mid_setUrgency, urgency_value);

		//set current location in ECEF frame
		jmethodID mid_setEcefX = (*env) -> GetMethodID(env, current_location_class, "setEcefX", "(I)V");
		jmethodID mid_setEcefY = (*env) -> GetMethodID(env, current_location_class, "setEcefY", "(I)V");
		jmethodID mid_setEcefZ = (*env) -> GetMethodID(env, current_location_class, "setEcefZ", "(I)V");
		jint ecef_x = message -> value.choice.TestMessage00.body.location.ecefX;
		jint ecef_y = message -> value.choice.TestMessage00.body.location.ecefY;
		jint ecef_z = message -> value.choice.TestMessage00.body.location.ecefZ;
		(*env) -> CallVoidMethod(env, location, mid_setEcefX, ecef_x);
		(*env) -> CallVoidMethod(env, location, mid_setEcefY, ecef_y);
		(*env) -> CallVoidMethod(env, location, mid_setEcefZ, ecef_z);
		uint8_t *location_time_content = message -> value.choice.TestMessage00.body.location.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, locationTimestamp, 0, 19, location_time_content);

		//set strategy parameters
		uint8_t *strategy_params_content = message -> value.choice.TestMessage00.body.strategyParams.buf;
		size_t strategy_params_size = message -> value.choice.TestMessage00.body.strategyParams.size;
		(*env) -> SetByteArrayRegion(env, strategyParams, 0, strategy_params_size, strategy_params_content);

		//set trajectory start location if necessary
		if(message -> value.choice.TestMessage00.body.trajectoryStart) {
			jint start_ecef_x = message -> value.choice.TestMessage00.body.trajectoryStart -> ecefX;
			jint start_ecef_y = message -> value.choice.TestMessage00.body.trajectoryStart -> ecefY;
			jint start_ecef_z = message -> value.choice.TestMessage00.body.trajectoryStart -> ecefZ;
			(*env) -> CallVoidMethod(env, startLocation, mid_setEcefX, start_ecef_x);
			(*env) -> CallVoidMethod(env, startLocation, mid_setEcefY, start_ecef_y);
			(*env) -> CallVoidMethod(env, startLocation, mid_setEcefZ, start_ecef_z);
			uint8_t *start_location_time_content = message -> value.choice.TestMessage00.body.trajectoryStart -> timestamp.buf;
			(*env) -> SetByteArrayRegion(env, startTimestamp, 0, 19, start_location_time_content);
		}

		// set trajectory offset data if necessary
		if(message -> value.choice.TestMessage00.body.trajectory) {
			jintArray offsets_X =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 0);
			jintArray offsets_Y =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 1);
			jintArray offsets_Z =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 2);
			int count = message -> value.choice.TestMessage00.body.trajectory -> list.count;
			int temp_offsets_X[60] = {0};
			int temp_offsets_Y[60] = {0};
			int temp_offsets_Z[60] = {0};
			for(int i = 0; i < count; i++) {
				temp_offsets_X[i] = message -> value.choice.TestMessage00.body.trajectory -> list.array[i] -> offsetX;
				temp_offsets_Y[i] = message -> value.choice.TestMessage00.body.trajectory -> list.array[i] -> offsetY;
				temp_offsets_Z[i] = message -> value.choice.TestMessage00.body.trajectory -> list.array[i] -> offsetZ;
			}
			(*env) -> SetIntArrayRegion(env, offsets_X, 0, 60, temp_offsets_X);
			(*env) -> SetIntArrayRegion(env, offsets_Y, 0, 60, temp_offsets_Y);
			(*env) -> SetIntArrayRegion(env, offsets_Z, 0, 60, temp_offsets_Z);
			(*env) -> DeleteLocalRef(env, offsets_X);
			(*env) -> DeleteLocalRef(env, offsets_Y);
			(*env) -> DeleteLocalRef(env, offsets_Z);
		}

		if(message -> value.choice.TestMessage00.body.expiration) {
			uint8_t *expiration_time_content = message -> value.choice.TestMessage00.body.expiration -> buf;
			(*env) -> SetByteArrayRegion(env, expiration, 0, 19, expiration_time_content);
		}
	} else {
		return -1;
	}
	return 0;
}


/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityPathMessage
 * Method:    encodeMobilityPath
 * Signature: ([B[B[B[B[B[BIII[[I)[B
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityPathMessage_encodeMobilityPath
  (JNIEnv *env, jobject obj, jbyteArray senderId, jbyteArray targetId, jbyteArray senderBsmId, jbyteArray planId, jbyteArray timestamp, jint startX, jint startY, jint startZ, jbyteArray locationTimestamp, jobjectArray offsets) {
	uint8_t buffer[512];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if (!message) {
		return NULL;
	}

	//set default value of testmessage02
	message -> messageId = 242;
	message -> value.present = MessageFrame__value_PR_TestMessage02;

	//set senderId in header
	jsize sender_string_size = (*env) -> GetArrayLength(env, senderId);
	jbyte *sender_string = (*env) -> GetByteArrayElements(env, senderId, 0);
	if (sender_string == NULL) {
		return NULL;
	}
	uint8_t sender_string_content[sender_string_size];
	for (int i = 0; i < sender_string_size; i++) {
		sender_string_content[i] = sender_string[i];
	}
	message -> value.choice.TestMessage02.header.hostStaticId.buf = sender_string_content;
	message -> value.choice.TestMessage02.header.hostStaticId.size = (size_t) sender_string_size;
	(*env) -> ReleaseByteArrayElements(env, senderId, sender_string, 0);

	//set targetId in header
	jsize target_string_size = (*env) -> GetArrayLength(env, targetId);
	jbyte *target_string = (*env) -> GetByteArrayElements(env, targetId, 0);
	if (target_string == NULL) {
		return NULL;
	}
	uint8_t target_string_content[target_string_size];
	for (int i = 0; i < target_string_size; i++) {
		target_string_content[i] = target_string[i];
	}
	message -> value.choice.TestMessage02.header.targetStaticId.buf = target_string_content;
	message -> value.choice.TestMessage02.header.targetStaticId.size = (size_t) target_string_size;
	(*env) -> ReleaseByteArrayElements(env, targetId, target_string, 0);

	//set hostBSMId in header
	jbyte *bsm_string = (*env) -> GetByteArrayElements(env, senderBsmId, 0);
	if(bsm_string == NULL) {
	    return NULL;
	}
	uint8_t host_bsm_id_content[8] = {0};
	for(int i = 0; i < 8; i++) {
		host_bsm_id_content[i] = bsm_string[i];
	}
	message -> value.choice.TestMessage02.header.hostBSMId.buf = host_bsm_id_content;
	message -> value.choice.TestMessage02.header.hostBSMId.size = 8;
	(*env) -> ReleaseByteArrayElements(env, senderBsmId, bsm_string, 0);

	//set planId in header
	jbyte *plan_id = (*env) -> GetByteArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[36] = {0};
	for (int i = 0; i < 36; i++) {
		plan_id_content[i] = plan_id[i];
	}
	message -> value.choice.TestMessage02.header.planId.buf = plan_id_content;
	message -> value.choice.TestMessage02.header.planId.size = 36;
	(*env) -> ReleaseByteArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jbyte *time = (*env) -> GetByteArrayElements(env, timestamp, 0);
	if (time == NULL) {
		return NULL;
	}
	uint8_t time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		time_content[i] = time[i];
	}
	message -> value.choice.TestMessage02.header.timestamp.buf = time_content;
	message -> value.choice.TestMessage02.header.timestamp.size = 19;
	(*env) -> ReleaseByteArrayElements(env, timestamp, time, 0);

	MobilityLocation_t location;
	location.ecefX = startX;
	location.ecefY = startY;
	location.ecefZ = startZ;
	jbyte *start_time = (*env) -> GetByteArrayElements(env, locationTimestamp, 0);

	uint8_t start_time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		start_time_content[i] = start_time[i];
	}
	location.timestamp.buf = start_time_content;
	location.timestamp.size = 19;

	message -> value.choice.TestMessage02.body.location = location;
	(*env) -> ReleaseByteArrayElements(env, locationTimestamp, start_time, 0);

	jsize dim = (*env) -> GetArrayLength(env, offsets);
	if(dim == 3) {
		jintArray offsets_X =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 0);
		jintArray offsets_Y =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 1);
		jintArray offsets_Z =  (jintArray) (*env) -> GetObjectArrayElement(env, offsets, 2);
		jsize count = (*env) -> GetArrayLength(env, offsets_X);
		jint *java_offsets_X = (*env) -> GetIntArrayElements(env, offsets_X, 0);
		jint *java_offsets_Y = (*env) -> GetIntArrayElements(env, offsets_Y, 0);
		jint *java_offsets_Z = (*env) -> GetIntArrayElements(env, offsets_Z, 0);
		if(count > 0) {
			int *localArray[3];
			int offsets_X_content[count];
			int offsets_Y_content[count];
			int offsets_Z_content[count];
			for(int i = 0; i < count; i++) {
				offsets_X_content[i] = java_offsets_X[i];
				offsets_Y_content[i] = java_offsets_Y[i];
				offsets_Z_content[i] = java_offsets_Z[i];
			}
			localArray[0] = offsets_X_content;
			localArray[1] = offsets_Y_content;
			localArray[2] = offsets_Z_content;
			MobilityLocationOffsets_t *trajectory_offsets;
			trajectory_offsets = calloc(1, sizeof(MobilityLocationOffsets_t));
			for(int i = 0; i < count; i++) {
				MobilityECEFOffset_t *offset_point;
				offset_point = calloc(1, sizeof(MobilityECEFOffset_t));
				offset_point -> offsetX = localArray[0][i];
				offset_point -> offsetY = localArray[1][i];
				offset_point -> offsetZ = localArray[2][i];
				asn_sequence_add(&trajectory_offsets->list, offset_point);
			}
			message -> value.choice.TestMessage02.body.trajectory.list = trajectory_offsets->list;
		}
		(*env) -> ReleaseIntArrayElements(env, offsets_X, java_offsets_X, 0);
		(*env) -> ReleaseIntArrayElements(env,offsets_Y, java_offsets_Y, 0);
		(*env) -> ReleaseIntArrayElements(env, offsets_Z, java_offsets_Z, 0);
		(*env) -> DeleteLocalRef(env, offsets_X);
		(*env) -> DeleteLocalRef(env, offsets_Y);
		(*env) -> DeleteLocalRef(env, offsets_Z);
	}

	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		//fprintf(fp, "!!!%s", ec.failed_type->name);
		return NULL;
	}

	//copy back to java output
	jsize length = ec.encoded / 8;
	jbyteArray outputJNIArray = (*env) -> NewByteArray(env, length);
	if(outputJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outputJNIArray, 0, length, buffer);
	return outputJNIArray;
  }

/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityPathMessage
 * Method:    decodeMobilityPath
 * Signature: ([BLjava/lang/Object;[B[B[B[B[B[BLjava/lang/Object;[[I)I
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityPathMessage_decodeMobilityPath
  (JNIEnv *env, jobject this, jbyteArray encodedArray, jobject pathObj, jbyteArray senderId, jbyteArray targetId, jbyteArray bsmId, jbyteArray planId, jbyteArray timestamp, jobject location, jbyteArray locationTimestamp, jobjectArray trajectoryOffsets) {
	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedArray); /* Number of bytes in encoded mobility path */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedArray, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
	    buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {
		jclass start_location_class = (*env) -> GetObjectClass(env, location);

		//set senderId, targetId, bsmId, planId and creation timestamp
		uint8_t *sender_id_content = message -> value.choice.TestMessage02.header.hostStaticId.buf;
		size_t sender_id_size = message -> value.choice.TestMessage02.header.hostStaticId.size;
		(*env) -> SetByteArrayRegion(env, senderId, 0, sender_id_size, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage02.header.targetStaticId.buf;
		size_t target_id_size = message -> value.choice.TestMessage02.header.targetStaticId.size;
		(*env) -> SetByteArrayRegion(env, targetId, 0, target_id_size, target_id_content);
		uint8_t *bsm_id_content = message -> value.choice.TestMessage02.header.hostBSMId.buf;
		(*env) -> SetByteArrayRegion(env, bsmId, 0, 8, bsm_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage02.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 36, plan_id_content);
		uint8_t *creation_time_content = message -> value.choice.TestMessage02.header.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, timestamp, 0, 19, creation_time_content);

		//set current location in ECEF frame
		jmethodID mid_setEcefX = (*env) -> GetMethodID(env, start_location_class, "setEcefX", "(I)V");
		jmethodID mid_setEcefY = (*env) -> GetMethodID(env, start_location_class, "setEcefY", "(I)V");
		jmethodID mid_setEcefZ = (*env) -> GetMethodID(env, start_location_class, "setEcefZ", "(I)V");
		jint ecef_x = message -> value.choice.TestMessage02.body.location.ecefX;
		jint ecef_y = message -> value.choice.TestMessage02.body.location.ecefY;
		jint ecef_z = message -> value.choice.TestMessage02.body.location.ecefZ;
		(*env) -> CallVoidMethod(env, location, mid_setEcefX, ecef_x);
		(*env) -> CallVoidMethod(env, location, mid_setEcefY, ecef_y);
		(*env) -> CallVoidMethod(env, location, mid_setEcefZ, ecef_z);

		uint8_t *location_time_content = message -> value.choice.TestMessage02.body.location.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, locationTimestamp, 0, 19, location_time_content);

		// set trajectory offset
		jintArray offsets_X =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 0);
		jintArray offsets_Y =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 1);
		jintArray offsets_Z =  (jintArray) (*env) -> GetObjectArrayElement(env, trajectoryOffsets, 2);
		int count = message -> value.choice.TestMessage02.body.trajectory.list.count;
		int temp_offsets_X[60] = {0};
		int temp_offsets_Y[60] = {0};
		int temp_offsets_Z[60] = {0};
		for(int i = 0; i < count; i++) {
			temp_offsets_X[i] = message -> value.choice.TestMessage02.body.trajectory.list.array[i] -> offsetX;
			temp_offsets_Y[i] = message -> value.choice.TestMessage02.body.trajectory.list.array[i] -> offsetY;
			temp_offsets_Z[i] = message -> value.choice.TestMessage02.body.trajectory.list.array[i] -> offsetZ;
		}
		(*env) -> SetIntArrayRegion(env, offsets_X, 0, count, temp_offsets_X);
		(*env) -> SetIntArrayRegion(env, offsets_Y, 0, count, temp_offsets_Y);
		(*env) -> SetIntArrayRegion(env, offsets_Z, 0, count, temp_offsets_Z);
		(*env) -> DeleteLocalRef(env, offsets_X);
		(*env) -> DeleteLocalRef(env, offsets_Y);
		(*env) -> DeleteLocalRef(env, offsets_Z);

		return 0;
	} else {
		return -1;
	}
  }

/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityResponseMessage
 * Method:    encodeMobilityResponse
 * Signature: ([B[B[B[B[BIZ)[B
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityResponseMessage_encodeMobilityResponse
  (JNIEnv *env, jobject obj, jbyteArray senderId, jbyteArray targetId, jbyteArray senderBsmId,
   jbyteArray planId, jbyteArray timestamp, jint urgency, jboolean isAccepted) {

	uint8_t buffer[512];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if (!message) {
		return NULL;
	}

	//set default value of testmessage01
	message -> messageId = 241;
	message -> value.present = MessageFrame__value_PR_TestMessage01;

	//set senderId in header
	jsize sender_string_size = (*env) -> GetArrayLength(env, senderId);
	jbyte *sender_string = (*env) -> GetByteArrayElements(env, senderId, 0);
	if (sender_string == NULL) {
		return NULL;
	}
	uint8_t sender_string_content[sender_string_size];
	for (int i = 0; i < sender_string_size; i++) {
		sender_string_content[i] = sender_string[i];
	}
	message -> value.choice.TestMessage01.header.hostStaticId.buf = sender_string_content;
	message -> value.choice.TestMessage01.header.hostStaticId.size = (size_t) sender_string_size;
	(*env) -> ReleaseByteArrayElements(env, senderId, sender_string, 0);

	//set targetId in header
	jsize target_string_size = (*env) -> GetArrayLength(env, targetId);
	jbyte *target_string = (*env) -> GetByteArrayElements(env, targetId, 0);
	if (target_string == NULL) {
		return NULL;
	}
	uint8_t target_string_content[target_string_size];
	for (int i = 0; i < target_string_size; i++) {
		target_string_content[i] = target_string[i];
	}
	message -> value.choice.TestMessage01.header.targetStaticId.buf = target_string_content;
	message -> value.choice.TestMessage01.header.targetStaticId.size = (size_t) target_string_size;
	(*env) -> ReleaseByteArrayElements(env, targetId, target_string, 0);

	//set hostBSMId in header
	jbyte *bsm_string = (*env) -> GetByteArrayElements(env, senderBsmId, 0);
	if(bsm_string == NULL) {
		return NULL;
	}
	uint8_t host_bsm_id_content[8] = {0};
	for(int i = 0; i < 8; i++) {
		host_bsm_id_content[i] = bsm_string[i];
	}
	message -> value.choice.TestMessage01.header.hostBSMId.buf = host_bsm_id_content;
	message -> value.choice.TestMessage01.header.hostBSMId.size = 8;
	(*env) -> ReleaseByteArrayElements(env, senderBsmId, bsm_string, 0);

	//set planId in header
	jbyte *plan_id = (*env) -> GetByteArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[36] = {0};
	for (int i = 0; i < 36; i++) {
		plan_id_content[i] = plan_id[i];
	}
	message -> value.choice.TestMessage01.header.planId.buf = plan_id_content;
	message -> value.choice.TestMessage01.header.planId.size = 36;
	(*env) -> ReleaseByteArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jbyte *time = (*env) -> GetByteArrayElements(env, timestamp, 0);
	if (time == NULL) {
		return NULL;
	}
	uint8_t time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		time_content[i] = time[i];
	}
	message -> value.choice.TestMessage01.header.timestamp.buf = time_content;
	message -> value.choice.TestMessage01.header.timestamp.size = 19;
	(*env) -> ReleaseByteArrayElements(env, timestamp, time, 0);

	// set urgency and isAccepted flag
	message -> value.choice.TestMessage01.body.urgency = urgency;
	message -> value.choice.TestMessage01.body.isAccepted = (int) isAccepted;

	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		//fprintf(fp, "!!!%s", ec.failed_type->name);
		return NULL;
	}

	//copy back to java output
	jsize length = ec.encoded / 8;
	jbyteArray outputJNIArray = (*env) -> NewByteArray(env, length);
	if(outputJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outputJNIArray, 0, length, buffer);
	return outputJNIArray;
}

/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityResponseMessage
 * Method:    decodeMobilityResponse
 * Signature: ([BLjava/lang/Object;[B[B[B[B[B)I
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityResponseMessage_decodeMobilityResponse
  (JNIEnv *env, jobject obj, jbyteArray encodedArray, jobject mobilityResponse, jbyteArray senderId,
   jbyteArray targetId, jbyteArray bsmId, jbyteArray planId, jbyteArray timestamp) {

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedArray); /* Number of bytes in encoded mobility path */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedArray, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
	    buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {
		jclass response_class = (*env) -> GetObjectClass(env, mobilityResponse);

		//set senderId, targetId, bsmId, planId and creation timestamp
		uint8_t *sender_id_content = message -> value.choice.TestMessage01.header.hostStaticId.buf;
		size_t sender_id_size = message -> value.choice.TestMessage01.header.hostStaticId.size;
		(*env) -> SetByteArrayRegion(env, senderId, 0, sender_id_size, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage01.header.targetStaticId.buf;
		size_t target_id_size = message -> value.choice.TestMessage01.header.targetStaticId.size;
		(*env) -> SetByteArrayRegion(env, targetId, 0, target_id_size, target_id_content);
		uint8_t *bsm_id_content = message -> value.choice.TestMessage01.header.hostBSMId.buf;
		(*env) -> SetByteArrayRegion(env, bsmId, 0, 8, bsm_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage01.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 36, plan_id_content);
		uint8_t *creation_time_content = message -> value.choice.TestMessage01.header.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, timestamp, 0, 19, creation_time_content);

		//set urgency and isAccepted flag
		jmethodID mid_setUrgency = (*env) -> GetMethodID(env, response_class, "setUrgency", "(S)V");
		jmethodID mid_setFlag = (*env) -> GetMethodID(env, response_class, "setIsAccepted", "(Z)V");
		jshort urgency = message -> value.choice.TestMessage01.body.urgency;
		jboolean flag = message -> value.choice.TestMessage01.body.isAccepted;
		(*env) -> CallVoidMethod(env, mobilityResponse, mid_setUrgency, urgency);
		(*env) -> CallVoidMethod(env, mobilityResponse, mid_setFlag, flag);

		return 0;
	} else {
		return -1;
	}
}

	/*
	 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityOperationMessage
	 * Method:    encodeMobilityOperation
	 * Signature: ([B[B[B[B[B[B[B)[B
	 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityOperationMessage_encodeMobilityOperation
  (JNIEnv *env, jobject obj, jbyteArray senderId, jbyteArray targetId, jbyteArray senderBsmId,
   jbyteArray planId, jbyteArray timestamp, jbyteArray strategy, jbyteArray params) {

	uint8_t buffer[512];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if (!message) {
		return NULL;
	}

	//set default value of testmessage01
	message -> messageId = 243;
	message -> value.present = MessageFrame__value_PR_TestMessage03;

	//set senderId in header
	jsize sender_string_size = (*env) -> GetArrayLength(env, senderId);
	jbyte *sender_string = (*env) -> GetByteArrayElements(env, senderId, 0);
	if (sender_string == NULL) {
		return NULL;
	}
	uint8_t sender_string_content[sender_string_size];
	for (int i = 0; i < sender_string_size; i++) {
		sender_string_content[i] = sender_string[i];
	}
	message -> value.choice.TestMessage03.header.hostStaticId.buf = sender_string_content;
	message -> value.choice.TestMessage03.header.hostStaticId.size = (size_t) sender_string_size;
	(*env) -> ReleaseByteArrayElements(env, senderId, sender_string, 0);

	//set targetId in header
	jsize target_string_size = (*env) -> GetArrayLength(env, targetId);
	jbyte *target_string = (*env) -> GetByteArrayElements(env, targetId, 0);
	if (target_string == NULL) {
		return NULL;
	}
	uint8_t target_string_content[target_string_size];
	for (int i = 0; i < target_string_size; i++) {
		target_string_content[i] = target_string[i];
	}
	message -> value.choice.TestMessage03.header.targetStaticId.buf = target_string_content;
	message -> value.choice.TestMessage03.header.targetStaticId.size = (size_t) target_string_size;
	(*env) -> ReleaseByteArrayElements(env, targetId, target_string, 0);

	//set hostBSMId in header
	jbyte *bsm_string = (*env) -> GetByteArrayElements(env, senderBsmId, 0);
	if(bsm_string == NULL) {
		return NULL;
	}
	uint8_t host_bsm_id_content[8] = {0};
	for(int i = 0; i < 8; i++) {
		host_bsm_id_content[i] = bsm_string[i];
	}
	message -> value.choice.TestMessage03.header.hostBSMId.buf = host_bsm_id_content;
	message -> value.choice.TestMessage03.header.hostBSMId.size = 8;
	(*env) -> ReleaseByteArrayElements(env, senderBsmId, bsm_string, 0);

	//set planId in header
	jbyte *plan_id = (*env) -> GetByteArrayElements(env, planId, 0);
	if (plan_id == NULL) {
		return NULL;
	}
	uint8_t plan_id_content[36] = {0};
	for (int i = 0; i < 36; i++) {
		plan_id_content[i] = plan_id[i];
	}
	message -> value.choice.TestMessage03.header.planId.buf = plan_id_content;
	message -> value.choice.TestMessage03.header.planId.size = 36;
	(*env) -> ReleaseByteArrayElements(env, planId, plan_id, 0);

	//set timestamp
	jbyte *time = (*env) -> GetByteArrayElements(env, timestamp, 0);
	if (time == NULL) {
		return NULL;
	}
	uint8_t time_content[19] = {0};
	for (int i = 0; i < 19; i++) {
		time_content[i] = time[i];
	}
	message -> value.choice.TestMessage03.header.timestamp.buf = time_content;
	message -> value.choice.TestMessage03.header.timestamp.size = 19;
	(*env) -> ReleaseByteArrayElements(env, timestamp, time, 0);

	// set strategy
	jsize strategy_string_size = (*env) -> GetArrayLength(env, strategy);
	jbyte *strategy_string = (*env) -> GetByteArrayElements(env, strategy, 0);
	if (strategy_string == NULL) {
		return NULL;
	}
	uint8_t strategy_string_content[strategy_string_size];
	for (int i = 0; i < strategy_string_size; i++) {
		strategy_string_content[i] = strategy_string[i];
	}
	message -> value.choice.TestMessage03.body.strategy.buf = strategy_string_content;
	message -> value.choice.TestMessage03.body.strategy.size = (size_t) strategy_string_size;
	(*env) -> ReleaseByteArrayElements(env, strategy, strategy_string, 0);

	//set params
	jsize params_string_size = (*env) -> GetArrayLength(env, params);
	jbyte *params_string = (*env) -> GetByteArrayElements(env, params, 0);
	if (params_string == NULL) {
		return NULL;
	}
	uint8_t params_string_content[params_string_size];
	for (int i = 0; i < params_string_size; i++) {
		params_string_content[i] = params_string[i];
	}
	message -> value.choice.TestMessage03.body.operationParams.buf = params_string_content;
	message -> value.choice.TestMessage03.body.operationParams.size = (size_t) params_string_size;
	(*env) -> ReleaseByteArrayElements(env, params, params_string, 0);

	//encode message
	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
	if(ec.encoded == -1) {
		//fprintf(fp, "!!!%s", ec.failed_type->name);
		return NULL;
	}

	//copy back to java output
	jsize length = ec.encoded / 8;
	jbyteArray outputJNIArray = (*env) -> NewByteArray(env, length);
	if(outputJNIArray == NULL) {
		return NULL;
	}
	(*env) -> SetByteArrayRegion(env, outputJNIArray, 0, length, buffer);
	return outputJNIArray;
}

/*
 * Class:     gov_dot_fhwa_saxton_carma_message_factory_MobilityOperationMessage
 * Method:    decodeMobilityOperation
 * Signature: ([B[B[B[B[B[B[B[B)I
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MobilityOperationMessage_decodeMobilityOperation
  (JNIEnv *env, jobject obj, jbyteArray encodedArray, jbyteArray senderId, jbyteArray targetId,
   jbyteArray bsmId, jbyteArray planId, jbyteArray timestamp, jbyteArray strategy, jbyteArray params) {

	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedArray); /* Number of bytes in encoded mobility path */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedArray, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
	    buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {
		//set senderId, targetId, bsmId, planId and creation timestamp
		uint8_t *sender_id_content = message -> value.choice.TestMessage03.header.hostStaticId.buf;
		size_t sender_id_size = message -> value.choice.TestMessage03.header.hostStaticId.size;
		(*env) -> SetByteArrayRegion(env, senderId, 0, sender_id_size, sender_id_content);
		uint8_t *target_id_content = message -> value.choice.TestMessage03.header.targetStaticId.buf;
		size_t target_id_size = message -> value.choice.TestMessage03.header.targetStaticId.size;
		(*env) -> SetByteArrayRegion(env, targetId, 0, target_id_size, target_id_content);
		uint8_t *bsm_id_content = message -> value.choice.TestMessage03.header.hostBSMId.buf;
		(*env) -> SetByteArrayRegion(env, bsmId, 0, 8, bsm_id_content);
		uint8_t *plan_id_content = message -> value.choice.TestMessage03.header.planId.buf;
		(*env) -> SetByteArrayRegion(env, planId, 0, 36, plan_id_content);
		uint8_t *creation_time_content = message -> value.choice.TestMessage03.header.timestamp.buf;
		(*env) -> SetByteArrayRegion(env, timestamp, 0, 19, creation_time_content);

		//set strategy and params
		uint8_t *strategy_content = message -> value.choice.TestMessage03.body.strategy.buf;
		size_t strategy_size = message -> value.choice.TestMessage03.body.strategy.size;
		(*env) -> SetByteArrayRegion(env, strategy, 0, strategy_size, strategy_content);
		uint8_t *strategy_params_content = message -> value.choice.TestMessage03.body.operationParams.buf;
		size_t strategy_params_size = message -> value.choice.TestMessage03.body.operationParams.size;
		(*env) -> SetByteArrayRegion(env, params, 0, strategy_params_size, strategy_params_content);

		return 0;
	} else {
		return -1;
	}
}

/**
 * Decode Map
 * 
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_MapMessage_decodeMap
  (JNIEnv *env, jobject obj, jbyteArray encodedArray, jobject plain_map, jintArray intersectionGeometry,
   jintArray laneId, jintArray ingressApproach, jintArray egressApproach, jintArray laneDirection, jintArray laneType, jobjectArray nodeXY, jobjectArray connectsTo) {
	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedArray); /* Number of bytes in encoded mobility path */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedArray, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
	    buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {

		jclass map_data_class = (*env) -> GetObjectClass(env, plain_map);
		jmethodID mid_setMsgCount = (*env) -> GetMethodID(env, map_data_class, "setMsgIssueRevision", "(B)V");
		jbyte msgCount = message -> value.choice.MapData.msgIssueRevision;
		(*env) -> CallVoidMethod(env, plain_map, mid_setMsgCount, msgCount);

		// In current decoder, we assume we have only one intersection in this list
		if(message -> value.choice.MapData.intersections && message -> value.choice.MapData.intersections -> list.count >= 1) {
			int intersectionData[9] = {0};
			intersectionData[8] = 1;
			intersectionData[0] = message -> value.choice.MapData.intersections -> list.array[0] -> id.id;
			intersectionData[1] = message -> value.choice.MapData.intersections -> list.array[0] -> revision;
			intersectionData[2] = message -> value.choice.MapData.intersections -> list.array[0] -> refPoint.lat;
			intersectionData[3] = message -> value.choice.MapData.intersections -> list.array[0] -> refPoint.Long;
			if(message -> value.choice.MapData.intersections -> list.array[0] -> refPoint.elevation) {
				intersectionData[4] = *message -> value.choice.MapData.intersections -> list.array[0] -> refPoint.elevation;
				intersectionData[5] = 1;
			}
			if(message -> value.choice.MapData.intersections -> list.array[0] -> laneWidth) {
				intersectionData[6] = *message -> value.choice.MapData.intersections -> list.array[0] -> laneWidth;
				intersectionData[7] = 1;
			}
			(*env) -> SetIntArrayRegion(env, intersectionGeometry, 0, 9, intersectionData);

			// the MAP message allowed up to 255 lanes in each intersection
			int laneIDData[255];
			// Initialize lane ID with invalid value to indicate the end
			for(int i = 0; i < 255; i++) {
				laneIDData[i] = -1;
			}
			int ingressApproachData[255] = {0};
			int egressApproachData[255] = {0};
			int laneDirectionData[255] = {0};
			int laneTypeData[255] = {0};
			// each node needs three data fields for X, Y and type
			// each lane has up to 63 nodes, so 63 * 3 = 189
			int nodeOffsetData[255][189] = {0};
			int connectionsData[255][32] = {0};
			for(int i = 0; i < 255; i++) {
				for(int j = 0; j < 32; j++) {
					connectionsData[i][j] = -1;
				}
			}
			int numOfLanes = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.count;
			for(int i = 0; i < numOfLanes; i++) {
				laneIDData[i] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> laneID;
				if(message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> ingressApproach) {
					ingressApproachData[i] = *message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> ingressApproach;
				} else {
					ingressApproachData[i] = -1;
				}
				if(message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> egressApproach) {
					egressApproachData[i] = *message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> egressApproach;
				} else {
					egressApproachData[i] = -1;
				}
				laneDirectionData[i] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i]->laneAttributes.directionalUse.buf[0];
				laneTypeData[i] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i]->laneAttributes.laneType.present;
				int numOfNodes = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.count;
				for(int j = 0; j < numOfNodes; j++) {
					int offsetType = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.present;
					switch(offsetType) {
					case NodeOffsetPointXY_PR_node_XY1:
						nodeOffsetData[i][j * 3] = 1;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY1.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY1.y;
						break;
					case NodeOffsetPointXY_PR_node_XY2:
						nodeOffsetData[i][j * 3] = 2;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY2.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY2.y;
						break;
					case NodeOffsetPointXY_PR_node_XY3:
						nodeOffsetData[i][j * 3] = 3;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY3.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY3.y;
						break;
					case NodeOffsetPointXY_PR_node_XY4:
						nodeOffsetData[i][j * 3] = 4;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY4.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY4.y;
						break;
					case NodeOffsetPointXY_PR_node_XY5:
						nodeOffsetData[i][j * 3] = 5;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY5.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY5.y;
						break;
					case NodeOffsetPointXY_PR_node_XY6:
						nodeOffsetData[i][j * 3] = 6;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY6.x;
						nodeOffsetData[i][j * 3 + 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_XY6.y;
						break;
					case NodeOffsetPointXY_PR_node_LatLon:
						nodeOffsetData[i][j * 3] = 7;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_LatLon.lat;
						nodeOffsetData[i][j * 3 + 1] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> nodeList.choice.nodes.list.array[j]->delta.choice.node_LatLon.lon;
						break;
					default:
						break;
					}
				}
				if(message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> connectsTo) {
					int numOfConnection = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> connectsTo -> list.count;
					for(int j = 0; j < numOfConnection; j++) {
						connectionsData[i][j * 2] = message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> connectsTo -> list.array[j] -> connectingLane.lane;
						if(message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> connectsTo -> list.array[j] -> signalGroup) {
							connectionsData[i][j * 2 + 1] = *message -> value.choice.MapData.intersections -> list.array[0] -> laneSet.list.array[i] -> connectsTo -> list.array[j] -> signalGroup;
						} else {
							connectionsData[i][j * 2 + 1] = -1;
						}
					}
				} else {
					connectionsData[i][0] = -1;
				}
			}
			(*env) -> SetIntArrayRegion(env, laneId, 0, 255, laneIDData);
			(*env) -> SetIntArrayRegion(env, ingressApproach, 0, 255, ingressApproachData);
			(*env) -> SetIntArrayRegion(env, egressApproach, 0, 255, egressApproachData);
			(*env) -> SetIntArrayRegion(env, laneDirection, 0, 255, laneDirectionData);
			(*env) -> SetIntArrayRegion(env, laneType, 0, 255, laneTypeData);
			for(int i = 0; i < numOfLanes; i++) {
				jintArray laneNodes = (*env) -> GetObjectArrayElement(env, nodeXY, i);
				jintArray connections = (*env) -> GetObjectArrayElement(env, connectsTo, i);
				(*env) -> SetIntArrayRegion(env, laneNodes, 0, 189, nodeOffsetData[i]);
				(*env) -> SetIntArrayRegion(env, connections, 0, 32, connectionsData[i]);
				(*env) -> DeleteLocalRef(env, laneNodes);
				(*env) -> DeleteLocalRef(env, connections);
			}
		}
		return 0;
	} else {
		return -1;
	}
}

/**
 * Decode Spat
 * 
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_factory_SPATMessage_decodeSPAT
    (JNIEnv *env, jobject obj, jbyteArray encodedArray, jintArray intersection, jobjectArray movementState) {
	asn_dec_rval_t rval; /* Decoder return value */
	MessageFrame_t *message = 0; /* Construct MessageFrame */

	int len = (*env) -> GetArrayLength(env, encodedArray); /* Number of bytes in encoded mobility path */
	jbyte *encodedMsg = (*env) -> GetByteArrayElements(env, encodedArray, 0); /* Get Java byte array content */
	char buf[len]; /* Input buffer for decoder function */
	for(int i = 0; i < len; i++) {
		buf[i] = encodedMsg[i];
	} /* Copy into buffer */
	rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
	if(rval.code == RC_OK) {
		int intersectionData[8] = {0};
		if(message->value.choice.SPAT.timeStamp) {
			intersectionData[0] = *message->value.choice.SPAT.timeStamp;
			intersectionData[1] = 1;
		}
		// In current decoder, we assume we have only one intersection in this list
		intersectionData[2] = message->value.choice.SPAT.intersections.list.array[0]->id.id;
		intersectionData[3] = message->value.choice.SPAT.intersections.list.array[0]->revision;
		if(message->value.choice.SPAT.intersections.list.array[0]->moy) {
			intersectionData[4] = *message->value.choice.SPAT.intersections.list.array[0]->moy;
			intersectionData[5] = 1;
		}
		if(message->value.choice.SPAT.intersections.list.array[0]->timeStamp) {
			intersectionData[6] = *message->value.choice.SPAT.intersections.list.array[0]->timeStamp;
			intersectionData[7] = 1;
		}
		(*env) -> SetIntArrayRegion(env, intersection, 0, 8, intersectionData);

		// each intersection allows up to 255 state
		// each state may contain up to 16 movement events plus one phase state enum
		// each movement event contains 9 integer data fields
		// each state needs 9 * 16 + 1 = 145 space
		int statesData[255][145];
		for(int i = 0; i < 255; i++) {
			for(int j = 0; j < 145; j++) {
				statesData[i][j] = -1;
			}
		}
		int numOfStates = message->value.choice.SPAT.intersections.list.array[0]->states.list.count;
		for(int i = 0; i < numOfStates; i++) {
			statesData[i][0] = message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->signalGroup;
			int numOfEvent = message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.count;
			for(int j = 0; j < numOfEvent; j++) {
				statesData[i][j * 9 + 1] = message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->eventState;
				if(message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing) {
					statesData[i][j * 9 + 9] = 1;
					if(message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->startTime) {
						statesData[i][j * 9 + 2] = *message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->startTime;
						statesData[i][j * 9 + 3] = 1;
					} else {
						statesData[i][j * 9 + 3] = 0;
					}
					statesData[i][j * 9 + 4] = message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->minEndTime;
					if(message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->maxEndTime) {
						statesData[i][j * 9 + 5] = *message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->maxEndTime;
						statesData[i][j * 9 + 6] = 1;
					} else {
						statesData[i][j * 9 + 6] = 0;
					}
					if(message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->nextTime) {
						statesData[i][j * 9 + 7] = *message->value.choice.SPAT.intersections.list.array[0]->states.list.array[i]->state_time_speed.list.array[j]->timing->nextTime;
						statesData[i][j * 9 + 8] = 1;
					} else {
						statesData[i][j * 9 + 8] = 0;
					}
				} else {
					statesData[i][j * 9 + 9] = 0;
				}
			}
		}
		for(int i = 0; i < 255; i++) {
			jintArray events = (*env) -> GetObjectArrayElement(env, movementState, i);
			(*env) -> SetIntArrayRegion(env, events, 0, 145, statesData[i]);
			(*env) -> DeleteLocalRef(env, events);
		}
		return 0;
	} else {
		return -1;
	}
}
