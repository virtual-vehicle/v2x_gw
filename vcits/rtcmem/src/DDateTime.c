/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/rtcmem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=RTCMEM`
 */

#include "DDateTime.h"

asn_TYPE_member_t asn_MBR_DDateTime_1[] = {
	{ ATF_POINTER, 7, offsetof(struct DDateTime, year),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DYear,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"year"
		},
	{ ATF_POINTER, 6, offsetof(struct DDateTime, month),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DMonth,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"month"
		},
	{ ATF_POINTER, 5, offsetof(struct DDateTime, day),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DDay,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"day"
		},
	{ ATF_POINTER, 4, offsetof(struct DDateTime, hour),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DHour,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"hour"
		},
	{ ATF_POINTER, 3, offsetof(struct DDateTime, minute),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DMinute,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"minute"
		},
	{ ATF_POINTER, 2, offsetof(struct DDateTime, second),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSecond,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"second"
		},
	{ ATF_POINTER, 1, offsetof(struct DDateTime, offset),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DOffset,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"offset"
		},
};
static const int asn_MAP_DDateTime_oms_1[] = { 0, 1, 2, 3, 4, 5, 6 };
static const ber_tlv_tag_t asn_DEF_DDateTime_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DDateTime_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* year */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* month */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* day */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* hour */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* minute */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* second */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 } /* offset */
};
asn_SEQUENCE_specifics_t asn_SPC_DDateTime_specs_1 = {
	sizeof(struct DDateTime),
	offsetof(struct DDateTime, _asn_ctx),
	asn_MAP_DDateTime_tag2el_1,
	7,	/* Count of tags in the map */
	asn_MAP_DDateTime_oms_1,	/* Optional members */
	7, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DDateTime = {
	"DDateTime",
	"DDateTime",
	&asn_OP_SEQUENCE,
	asn_DEF_DDateTime_tags_1,
	sizeof(asn_DEF_DDateTime_tags_1)
		/sizeof(asn_DEF_DDateTime_tags_1[0]), /* 1 */
	asn_DEF_DDateTime_tags_1,	/* Same as above */
	sizeof(asn_DEF_DDateTime_tags_1)
		/sizeof(asn_DEF_DDateTime_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_DDateTime_1,
	7,	/* Elements count */
	&asn_SPC_DDateTime_specs_1	/* Additional specs */
};

