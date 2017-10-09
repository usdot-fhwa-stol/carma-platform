#include "gov_dot_fhwa_saxton_carma_message_MessageConsumer.h"
#include <string.h>

//This is just a test for C wrapper of asn1c library
JNIEXPORT jbyteArray JNICALL Java_gov_dot_fhwa_saxton_carma_message_MessageConsumer_encode_1BSM (JNIEnv *env, jclass cls, jobject bsm) {
	jbyte outCArray[] = {5, 2, 8};
	jbyteArray outJNIArray = (*env) -> NewByteArray(env, 3);
	if(outJNIArray == NULL) return NULL;
	(*env) -> SetByteArrayRegion(env, outJNIArray, 0, 3, outCArray);
	return outJNIArray;
}
