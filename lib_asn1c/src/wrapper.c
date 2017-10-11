#include <stdio.h>
#include <sys/types.h>
#include "gov_dot_fhwa_saxton_carma_message_BSMFactory.h"
#include "MessageFrame.h"

//This is just a test for C wrapper of asn1c library
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_BSMFactory_encode_1BSM
  (JNIEnv *env, jclass cls, jobject bsm, jbyteArray bsm_id, jobject accuracy, jobject transmission, jobject accelset, jbyteArray brakestatus, jobject size) {

	uint8_t buffer[128];
	size_t buffer_size = sizeof(buffer);
	asn_enc_rval_t ec;
	MessageFrame_t *message;

	message = calloc(1, sizeof(MessageFrame_t));
	if(!message) {
		perror("calloc() failed");
		exit(1);
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
	//message->value.choice.BasicSafetyMessage.coreData.msgCnt = 101;

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
	//message->value.choice.BasicSafetyMessage.coreData.secMark = 31202;

	jmethodID mid_getLatitude = (*env) -> GetMethodID(env, bsm_core_class, "getLatitude", "()D");
	jdouble lat = (*env) -> CallDoubleMethod(env, bsm, mid_getLatitude);
	message->value.choice.BasicSafetyMessage.coreData.lat = lat * 10000000;
	//message->value.choice.BasicSafetyMessage.coreData.lat = 41252;

	jmethodID mid_getLongitude = (*env) -> GetMethodID(env, bsm_core_class, "getLongitude", "()D");
	jdouble lon = (*env) -> CallDoubleMethod(env, bsm, mid_getLongitude);
	message->value.choice.BasicSafetyMessage.coreData.Long = lon * 10000000;
	//message->value.choice.BasicSafetyMessage.coreData.Long = -21000001;

	jmethodID mid_getElev = (*env) -> GetMethodID(env, bsm_core_class, "getElev", "()D");
	jdouble elev = (*env) -> CallDoubleMethod(env, bsm, mid_getElev);
	message->value.choice.BasicSafetyMessage.coreData.elev = elev * 10;
	//message->value.choice.BasicSafetyMessage.coreData.elev = 312;

	jmethodID mid_getSemiMajor = (*env) -> GetMethodID(env, accuracy_class, "getSemiMajor", "()D");
	jmethodID mid_getSemiMinor = (*env) -> GetMethodID(env, accuracy_class, "getSemiMinor", "()D");
	jmethodID mid_getOrientation = (*env) -> GetMethodID(env, accuracy_class, "getOrientation", "()D");
	jdouble major = (*env) -> CallDoubleMethod(env, accuracy, mid_getSemiMajor);
	jdouble minor = (*env) -> CallDoubleMethod(env, accuracy, mid_getSemiMinor);
	jdouble orientation = (*env) -> CallDoubleMethod(env, accuracy, mid_getOrientation);
	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor = major / 0.05;
	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor = minor / 0.05;
	message->value.choice.BasicSafetyMessage.coreData.accuracy.orientation = orientation / 0.054932479;
//	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMajor = 145;
//	message->value.choice.BasicSafetyMessage.coreData.accuracy.semiMinor = 125;
//	message->value.choice.BasicSafetyMessage.coreData.accuracy.orientation = 30252;

	jmethodID mid_getTransmissionState = (*env) -> GetMethodID(env, transmission_class, "getTransmissionState", "()B");
	jbyte transmissionstate = (*env) -> CallByteMethod(env, transmission, mid_getTransmissionState);
	message->value.choice.BasicSafetyMessage.coreData.transmission = transmissionstate;
	//message->value.choice.BasicSafetyMessage.coreData.transmission = 2;

	jmethodID mid_getSpeed = (*env) -> GetMethodID(env, bsm_core_class, "getSpeed", "()D");
	jdouble speed = (*env) -> CallDoubleMethod(env, bsm, mid_getSpeed);
	message->value.choice.BasicSafetyMessage.coreData.speed = speed / 0.02;
	//message->value.choice.BasicSafetyMessage.coreData.speed = 2100;

	jmethodID mid_getHeading = (*env) -> GetMethodID(env, bsm_core_class, "getHeading", "()D");
	jdouble heading = (*env) -> CallDoubleMethod(env, bsm, mid_getHeading);
	message->value.choice.BasicSafetyMessage.coreData.heading = heading / 0.0125;
	//message->value.choice.BasicSafetyMessage.coreData.heading = 22049;

	jmethodID mid_getAngle = (*env) -> GetMethodID(env, bsm_core_class, "getAngle", "()D");
	jdouble angle = (*env) -> CallDoubleMethod(env, bsm, mid_getAngle);
	message->value.choice.BasicSafetyMessage.coreData.angle = angle / 1.5;
	//message->value.choice.BasicSafetyMessage.coreData.angle = 13;

	jmethodID mid_accelset_getLongitude = (*env) -> GetMethodID(env, accelset_class, "getLongitude", "()D");
	jmethodID mid_accelset_getLatitude = (*env) -> GetMethodID(env, accelset_class, "getLatitude", "()D");
	jmethodID mid_accelset_getVert = (*env) -> GetMethodID(env, accelset_class, "getVert", "()D");
	jmethodID mid_accelset_getYaw = (*env) -> GetMethodID(env, accelset_class, "getYaw", "()D");
	jdouble accel_long = (*env) -> CallDoubleMethod(env, accelset, mid_accelset_getLongitude);
	jdouble accel_lat = (*env) -> CallDoubleMethod(env, accelset, mid_accelset_getLatitude);
	jdouble accel_vert = (*env) -> CallDoubleMethod(env, accelset, mid_accelset_getVert);
	jdouble accel_yaw = (*env) -> CallDoubleMethod(env, accelset, mid_accelset_getYaw);
	message->value.choice.BasicSafetyMessage.coreData.accelSet.Long = accel_long / 0.01;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.lat = accel_lat / 0.01;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.vert = accel_vert / 0.02;
	message->value.choice.BasicSafetyMessage.coreData.accelSet.yaw = accel_yaw / 0.01; //TODO This line cause an one-bit error
//	message->value.choice.BasicSafetyMessage.coreData.accelSet.Long = 12;
//	message->value.choice.BasicSafetyMessage.coreData.accelSet.lat = -180;
//	message->value.choice.BasicSafetyMessage.coreData.accelSet.vert = 55;
//	message->value.choice.BasicSafetyMessage.coreData.accelSet.yaw = -16001;

	jbyte *inCArray = (*env) -> GetByteArrayElements(env, brakestatus, 0);
//	uint8_t break_content[1] = {0x48};
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
//	message->value.choice.BasicSafetyMessage.coreData.brakes.traction = 2;
//	message->value.choice.BasicSafetyMessage.coreData.brakes.abs = 3;
//	message->value.choice.BasicSafetyMessage.coreData.brakes.scs = 1;
//	message->value.choice.BasicSafetyMessage.coreData.brakes.brakeBoost = 0;
//	message->value.choice.BasicSafetyMessage.coreData.brakes.auxBrakes = 1;



	jmethodID mid_getVehicleWidth = (*env) -> GetMethodID(env, size_class, "getVehicleWidth", "()D");
	jmethodID mid_getVehicleLength = (*env) -> GetMethodID(env, size_class, "getVehicleLength", "()D");
	jdouble v_width = (*env) -> CallDoubleMethod(env, size, mid_getVehicleWidth);
	jdouble v_length = (*env) -> CallDoubleMethod(env, size, mid_getVehicleLength);
	message->value.choice.BasicSafetyMessage.coreData.size.width = v_width * 100;
	message->value.choice.BasicSafetyMessage.coreData.size.length = v_length * 100;
//	message->value.choice.BasicSafetyMessage.coreData.size.width = 199;
//	message->value.choice.BasicSafetyMessage.coreData.size.length = 3069;

	ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, message, buffer, buffer_size);

	jsize length = ec.encoded / 8;
	jbyteArray outJNIArray = (*env) -> NewByteArray(env, length);
	if(outJNIArray == NULL) return NULL;
	(*env) -> SetByteArrayRegion(env, outJNIArray, 0, length, buffer);
	return outJNIArray;
}
