/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/srem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SREM`
 */

#ifndef	_RequestorPositionVector_H_
#define	_RequestorPositionVector_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Position3D.h"
#include "Angle.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct TransmissionAndSpeed;

/* RequestorPositionVector */
typedef struct RequestorPositionVector {
	Position3D_t	 position;
	Angle_t	*heading;	/* OPTIONAL */
	struct TransmissionAndSpeed	*speed;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RequestorPositionVector_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RequestorPositionVector;
extern asn_SEQUENCE_specifics_t asn_SPC_RequestorPositionVector_specs_1;
extern asn_TYPE_member_t asn_MBR_RequestorPositionVector_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _RequestorPositionVector_H_ */
#include <asn_internal.h>
