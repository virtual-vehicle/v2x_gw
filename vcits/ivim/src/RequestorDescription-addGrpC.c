/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#include "RequestorDescription-addGrpC.h"

asn_TYPE_member_t asn_MBR_RequestorDescription_addGrpC_1[] = {
	{ ATF_POINTER, 2, offsetof(struct RequestorDescription_addGrpC, fuel),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FuelType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"fuel"
		},
	{ ATF_POINTER, 1, offsetof(struct RequestorDescription_addGrpC, batteryStatus),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BatteryStatus,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"batteryStatus"
		},
};
static const int asn_MAP_RequestorDescription_addGrpC_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_RequestorDescription_addGrpC_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RequestorDescription_addGrpC_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* fuel */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* batteryStatus */
};
asn_SEQUENCE_specifics_t asn_SPC_RequestorDescription_addGrpC_specs_1 = {
	sizeof(struct RequestorDescription_addGrpC),
	offsetof(struct RequestorDescription_addGrpC, _asn_ctx),
	asn_MAP_RequestorDescription_addGrpC_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_RequestorDescription_addGrpC_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RequestorDescription_addGrpC = {
	"RequestorDescription-addGrpC",
	"RequestorDescription-addGrpC",
	&asn_OP_SEQUENCE,
	asn_DEF_RequestorDescription_addGrpC_tags_1,
	sizeof(asn_DEF_RequestorDescription_addGrpC_tags_1)
		/sizeof(asn_DEF_RequestorDescription_addGrpC_tags_1[0]), /* 1 */
	asn_DEF_RequestorDescription_addGrpC_tags_1,	/* Same as above */
	sizeof(asn_DEF_RequestorDescription_addGrpC_tags_1)
		/sizeof(asn_DEF_RequestorDescription_addGrpC_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RequestorDescription_addGrpC_1,
	2,	/* Elements count */
	&asn_SPC_RequestorDescription_addGrpC_specs_1	/* Additional specs */
};

