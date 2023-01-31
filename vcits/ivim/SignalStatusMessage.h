/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_SignalStatusMessage_H_
#define	_SignalStatusMessage_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MinuteOfTheYear.h"
#include "DSecond.h"
#include "MsgCount.h"
#include "SignalStatusList.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Reg_SignalStatusMessage;

/* SignalStatusMessage */
typedef struct SignalStatusMessage {
	MinuteOfTheYear_t	*timeStamp;	/* OPTIONAL */
	DSecond_t	 second;
	MsgCount_t	*sequenceNumber;	/* OPTIONAL */
	SignalStatusList_t	 status;
	struct SignalStatusMessage__regional {
		A_SEQUENCE_OF(struct Reg_SignalStatusMessage) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SignalStatusMessage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SignalStatusMessage;

#ifdef __cplusplus
}
#endif

#endif	/* _SignalStatusMessage_H_ */
#include <asn_internal.h>
