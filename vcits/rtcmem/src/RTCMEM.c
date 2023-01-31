/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "RTCMEM-PDU-Descriptions"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/RTCMEM-PDU-Descriptions.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/rtcmem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=RTCMEM`
 */

#include "RTCMEM.h"

static asn_TYPE_member_t asn_MBR_RTCMEM_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RTCMEM, header),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ItsPduHeader,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"header"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RTCMEM, rtcmc),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RTCMcorrections,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"rtcmc"
		},
};
static const ber_tlv_tag_t asn_DEF_RTCMEM_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RTCMEM_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* header */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* rtcmc */
};
static asn_SEQUENCE_specifics_t asn_SPC_RTCMEM_specs_1 = {
	sizeof(struct RTCMEM),
	offsetof(struct RTCMEM, _asn_ctx),
	asn_MAP_RTCMEM_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RTCMEM = {
	"RTCMEM",
	"RTCMEM",
	&asn_OP_SEQUENCE,
	asn_DEF_RTCMEM_tags_1,
	sizeof(asn_DEF_RTCMEM_tags_1)
		/sizeof(asn_DEF_RTCMEM_tags_1[0]), /* 1 */
	asn_DEF_RTCMEM_tags_1,	/* Same as above */
	sizeof(asn_DEF_RTCMEM_tags_1)
		/sizeof(asn_DEF_RTCMEM_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RTCMEM_1,
	2,	/* Elements count */
	&asn_SPC_RTCMEM_specs_1	/* Additional specs */
};

