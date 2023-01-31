/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ssem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SSEM`
 */

#ifndef	_NodeAttributeSetXY_H_
#define	_NodeAttributeSetXY_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Offset-B10.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NodeAttributeXYList;
struct SegmentAttributeXYList;
struct LaneDataAttributeList;
struct Reg_NodeAttributeSetXY;

/* NodeAttributeSetXY */
typedef struct NodeAttributeSetXY {
	struct NodeAttributeXYList	*localNode;	/* OPTIONAL */
	struct SegmentAttributeXYList	*disabled;	/* OPTIONAL */
	struct SegmentAttributeXYList	*enabled;	/* OPTIONAL */
	struct LaneDataAttributeList	*data;	/* OPTIONAL */
	Offset_B10_t	*dWidth;	/* OPTIONAL */
	Offset_B10_t	*dElevation;	/* OPTIONAL */
	struct NodeAttributeSetXY__regional {
		A_SEQUENCE_OF(struct Reg_NodeAttributeSetXY) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NodeAttributeSetXY_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeAttributeSetXY;
extern asn_SEQUENCE_specifics_t asn_SPC_NodeAttributeSetXY_specs_1;
extern asn_TYPE_member_t asn_MBR_NodeAttributeSetXY_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _NodeAttributeSetXY_H_ */
#include <asn_internal.h>
