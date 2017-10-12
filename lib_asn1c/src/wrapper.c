#include <stdio.h>
#include <sys/types.h>
#include "gov_dot_fhwa_saxton_carma_message_BSMFactory.h"
#include "MessageFrame.h"

/**
 * BSM Encoder:
 * This function can decode an BSM object from Java to a byte array in J2735 standards.
 * When an error happened, this function will return NULL.
 */
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_BSMFactory_encode_1BSM
  (JNIEnv *env, jclass cls, jobject bsm, jbyteArray bsm_id, jobject accuracy, jobject transmission, jobject accelset, jbyteArray brakestatus, jobject size) {

	uint8_t buffer[128];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if(!message) {
		return NULL;
	}

	//get a reference to classes
	jclass bsm_core_class = (*env) -> GetObjectClass(env, bsm);
	jclass accuracy_class = (*env) -> GetObjectClass(env, accuracy);
	jclass transmission_class = (*env) -> GetObjectClass(env, transmission);
	jclass accelset_class = (*env) -> GetObjectClass(env, accelset);
	jclass size_class = (*env) -> GetObjectClass(env, size);

	//set default fields of BSM
	message->messageId = 20;
	message->value.present = value_PR_BasicSafetyMessage;

	//get the method ID of "getMsgCount"
	jmethodID mid_getMsgCount = (*env) -> GetMethodID(env, bsm_core_class, "getMsgCount", "()B");

	//Get and set msgCnt
	jbyte msgCount = (*env) -> CallByteMethod(env, bsm, mid_getMsgCount);
	message->value.choice.BasicSafetyMessage.coreData.msgCnt = msgCount;

	uint8_t content[4] = {0x00, 0x00, 0x00, 0x00};
	jbyte *bsm_msg_id = (*env) -> GetByteArrayElements(env, bsm_id, 0);
	if(bsm_msg_id == NULL) return NULL;
	for(int i = 0; i < 4; i++) {
		content[i] = bsm_msg_id[i];
	}
	(*env) -> ReleaseByteArrayElements(env, bsm_id, bsm_msg_id, 0);
	message->value.choice.BasicSafetyMessage.coreData.id.buf = content;
	message->value.choice.BasicSafetyMessage.coreData.id.size = 4;

	//set other fields
	jmethodID mid_getSecMark = (*env) -> GetMethodID(env, bsm_core_class, "getSecMark", "()S");
	jshort secMark = (*env) -> CallShortMethod(env, bsm, mid_getSecMark);
	message->value.choice.BasicSafetyMessage.coreData.secMark = secMark;

	jmethodID mid_getLatitude = (*env) -> GetMethodID(env, bsm_core_class, "getLatitude", "()D");
	jdouble lat = (*env) -> CallDoubleMethod(env, bsm, mid_getLatitude);
	message->value.choice.BasicSafetyMessage.coreData.lat = lat * 10000000;

	jmethodID mid_getLongitude = (*env) -> GetMethodID(env, bsm_core_class, "getLongitude", "()D");
	jdouble lon = (*env) -> CallDoubleMethod(env, bsm, mid_getLongitude);
	message->value.choice.BasicSafetyMessage.coreData.Long = lon * 10000000;

	jmethodID mid_getElev = (*env) -> GetMethodID(env, bsm_core_class, "getElev", "()F");
	jfloat elev = (*env) -> CallFloatMethod(env, bsm, mid_getElev);
	message->value.choice.BasicSafetyMessage.coreData.elev = elev * 10;

	jmethodID mid_getSemiMajor = (*env) -> GetMethodID(env, accuracy_class, "getSemiMajor", "()F");
	jmethodID mid_getSemiMinor = (*env) -> GetMethodID(env, accuracy_class, "getSemiMinor", "()F");
	jmethodID mid_getOrientation = (*env) -> GetMethodID(env, accuracy_class, "getOrientation", "()D");
	jfloat major = (*env) -> CallFloatMethod(env, accuracy, mid_getSemiMajor);
	jfloat minor = (*env) -> CallFloatMethod(env, accuracy, mid_getSemiMinor);
	jdouble orientation = (*env) -> CallDoubleMethod(env, accuracy, mid_getOrientation);
	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor = major / 0.05;
	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor = minor / 0.05;
	message->value.choice.BasicSafetyMessage.coreData.accuracy.orientation = orientation / 0.054932479;

	jmethodID mid_getTransmissionState = (*env) -> GetMethodID(env, transmission_class, "getTransmissionState", "()B");
	jbyte transmissionstate = (*env) -> CallByteMethod(env, transmission, mid_getTransmissionState);
	message->value.choice.BasicSafetyMessage.coreData.transmission = transmissionstate;

	jmethodID mid_getSpeed = (*env) -> GetMethodID(env, bsm_core_class, "getSpeed", "()F");
	jfloat speed = (*env) -> CallFloatMethod(env, bsm, mid_getSpeed);
	message->value.choice.BasicSafetyMessage.coreData.speed = speed / 0.02;

	jmethodID mid_getHeading = (*env) -> GetMethodID(env, bsm_core_class, "getHeading", "()F");
	jfloat heading = (*env) -> CallFloatMethod(env, bsm, mid_getHeading);
	message->value.choice.BasicSafetyMessage.coreData.heading = heading / 0.0125;

	jmethodID mid_getAngle = (*env) -> GetMethodID(env, bsm_core_class, "getAngle", "()F");
	jfloat angle = (*env) -> CallFloatMethod(env, bsm, mid_getAngle);
	message->value.choice.BasicSafetyMessage.coreData.angle = angle / 1.5;

	jmethodID mid_accelset_getLongitude = (*env) -> GetMethodID(env, accelset_class, "getLongitude", "()F");
	jmethodID mid_accelset_getLatitude = (*env) -> GetMethodID(env, accelset_class, "getLatitude", "()F");
	jmethodID mid_accelset_getVert = (*env) -> GetMethodID(env, accelset_class, "getVert", "()F");
	jmethodID mid_accelset_getYaw = (*env) -> GetMethodID(env, accelset_class, "getYaw", "()F");
	jfloat accel_long = (*env) -> CallFloatMethod(env, accelset, mid_accelset_getLongitude);
	jfloat accel_lat = (*env) -> CallFloatMethod(env, accelset, mid_accelset_getLatitude);
	jfloat accel_vert = (*env) -> CallFloatMethod(env, accelset, mid_accelset_getVert);
	jfloat accel_yaw = (*env) -> CallFloatMethod(env, accelset, mid_accelset_getYaw);
	message->value.choice.BasicSafetyMessage.coreData.accelSet.Long = accel_long / 0.01;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.lat = accel_lat / 0.01;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.vert = accel_vert / 0.02;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.yaw = accel_yaw / 0.01;

	jbyte *inCArray = (*env) -> GetByteArrayElements(env, brakestatus, 0);
	uint8_t break_content[1] = {0};
	break_content[0] = inCArray[0];
	message->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.bits_unused = 3;
	message->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf = break_content;
	message->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.size = 1;
	message->value.choice.BasicSafetyMessage.coreData.brakes.traction = inCArray[1];
	message->value.choice.BasicSafetyMessage.coreData.brakes.abs = inCArray[2];
	message->value.choice.BasicSafetyMessage.coreData.brakes.scs = inCArray[3];
	message->value.choice.BasicSafetyMessage.coreData.brakes.brakeBoost = inCArray[4];
	message->value.choice.BasicSafetyMessage.coreData.brakes.auxBrakes = inCArray[5];
	(*env) -> ReleaseByteArrayElements(env, brakestatus, inCArray, 0);



	jmethodID mid_getVehicleWidth = (*env) -> GetMethodID(env, size_class, "getVehicleWidth", "()F");
	jmethodID mid_getVehicleLength = (*env) -> GetMethodID(env, size_class, "getVehicleLength", "()F");
	jfloat v_width = (*env) -> CallFloatMethod(env, size, mid_getVehicleWidth);
	jfloat v_length = (*env) -> CallFloatMethod(env, size, mid_getVehicleLength);
	message->value.choice.BasicSafetyMessage.coreData.size.width = v_width * 100;
	message->value.choice.BasicSafetyMessage.coreData.size.length = v_length * 100;

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
