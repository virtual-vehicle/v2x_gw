/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/spatem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SPATEM`
 */

#ifndef	_Node_XY_20b_H_
#define	_Node_XY_20b_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Offset-B10.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Node-XY-20b */
typedef struct Node_XY_20b {
	Offset_B10_t	 x;
	Offset_B10_t	 y;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Node_XY_20b_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Node_XY_20b;
extern asn_SEQUENCE_specifics_t asn_SPC_Node_XY_20b_specs_1;
extern asn_TYPE_member_t asn_MBR_Node_XY_20b_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Node_XY_20b_H_ */
#include <asn_internal.h>
