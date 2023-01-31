/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/spatem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SPATEM`
 */

#include "IntersectionReferenceID.h"

asn_TYPE_member_t asn_MBR_IntersectionReferenceID_1[] = {
	{ ATF_POINTER, 1, offsetof(struct IntersectionReferenceID, region),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RoadRegulatorID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"region"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct IntersectionReferenceID, id),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IntersectionID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
};
static const int asn_MAP_IntersectionReferenceID_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_IntersectionReferenceID_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IntersectionReferenceID_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* region */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* id */
};
asn_SEQUENCE_specifics_t asn_SPC_IntersectionReferenceID_specs_1 = {
	sizeof(struct IntersectionReferenceID),
	offsetof(struct IntersectionReferenceID, _asn_ctx),
	asn_MAP_IntersectionReferenceID_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_IntersectionReferenceID_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IntersectionReferenceID = {
	"IntersectionReferenceID",
	"IntersectionReferenceID",
	&asn_OP_SEQUENCE,
	asn_DEF_IntersectionReferenceID_tags_1,
	sizeof(asn_DEF_IntersectionReferenceID_tags_1)
		/sizeof(asn_DEF_IntersectionReferenceID_tags_1[0]), /* 1 */
	asn_DEF_IntersectionReferenceID_tags_1,	/* Same as above */
	sizeof(asn_DEF_IntersectionReferenceID_tags_1)
		/sizeof(asn_DEF_IntersectionReferenceID_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_IntersectionReferenceID_1,
	2,	/* Elements count */
	&asn_SPC_IntersectionReferenceID_specs_1	/* Additional specs */
};

