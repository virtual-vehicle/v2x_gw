/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/tmp/gen_env/build/asn1/ISO19321IVIv2.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_LaneStatus_H_
#define	_LaneStatus_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum LaneStatus {
	LaneStatus_open	= 0,
	LaneStatus_closed	= 1,
	LaneStatus_mergeR	= 2,
	LaneStatus_mergeL	= 3,
	LaneStatus_mergeLR	= 4,
	LaneStatus_provisionallyOpen	= 5,
	LaneStatus_diverging	= 6
} e_LaneStatus;

/* LaneStatus */
typedef long	 LaneStatus_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_LaneStatus_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_LaneStatus;
asn_struct_free_f LaneStatus_free;
asn_struct_print_f LaneStatus_print;
asn_constr_check_f LaneStatus_constraint;
ber_type_decoder_f LaneStatus_decode_ber;
der_type_encoder_f LaneStatus_encode_der;
xer_type_decoder_f LaneStatus_decode_xer;
xer_type_encoder_f LaneStatus_encode_xer;
oer_type_decoder_f LaneStatus_decode_oer;
oer_type_encoder_f LaneStatus_encode_oer;
per_type_decoder_f LaneStatus_decode_uper;
per_type_encoder_f LaneStatus_encode_uper;
per_type_decoder_f LaneStatus_decode_aper;
per_type_encoder_f LaneStatus_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _LaneStatus_H_ */
#include <asn_internal.h>
