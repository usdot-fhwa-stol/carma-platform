#include <stdio.h>
#include <sys/types.h>
#include "gov_dot_fhwa_saxton_carma_factory_BSMFactory.h"
#include "MessageFrame.h"

/**
 * BSM Encoder:
 * This function can encode an BSM object from Java to a byte array in J2735 standards.
 * When an error happened, this function will return NULL.
 * Note: In this function, we pass parameters instead of passing a single object,
 * because making the natives to reach for many individual fields
 * from objects passed to them leads to poor performance.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_BSMFactory_encode_1BSM
		(JNIEnv *env, jclass cls, jint msgCount, jintArray bsm_id, jint secMark,
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
	message -> value.present = value_PR_BasicSafetyMessage;

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

	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, message, buffer, buffer_size);
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
 * Return -1 mains an error is happened; return 0 mains decoding succeed.
 */
JNIEXPORT jint JNICALL Java_gov_dot_fhwa_saxton_carma_message_BSMFactory_decode_1BSM
  (JNIEnv *env, jclass cls, jbyteArray encoded_bsm,
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
