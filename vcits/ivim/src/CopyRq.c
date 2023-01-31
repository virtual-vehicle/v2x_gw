/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#include "CopyRq.h"

static int
memb_destinationEID_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 127)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_destinationEID_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_memb_destinationEID_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  7,  7,  0,  127 }	/* (0..127,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_CopyRq_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CopyRq, destinationEID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_destinationEID_constr_2, &asn_PER_memb_destinationEID_constr_2,  memb_destinationEID_constraint_1 },
		0, 0, /* No default value */
		"destinationEID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CopyRq, attributeIdList),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_AttributeIdList,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"attributeIdList"
		},
};
static const ber_tlv_tag_t asn_DEF_CopyRq_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CopyRq_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* destinationEID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* attributeIdList */
};
static asn_SEQUENCE_specifics_t asn_SPC_CopyRq_specs_1 = {
	sizeof(struct CopyRq),
	offsetof(struct CopyRq, _asn_ctx),
	asn_MAP_CopyRq_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CopyRq = {
	"CopyRq",
	"CopyRq",
	&asn_OP_SEQUENCE,
	asn_DEF_CopyRq_tags_1,
	sizeof(asn_DEF_CopyRq_tags_1)
		/sizeof(asn_DEF_CopyRq_tags_1[0]), /* 1 */
	asn_DEF_CopyRq_tags_1,	/* Same as above */
	sizeof(asn_DEF_CopyRq_tags_1)
		/sizeof(asn_DEF_CopyRq_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_CopyRq_1,
	2,	/* Elements count */
	&asn_SPC_CopyRq_specs_1	/* Additional specs */
};

