/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/experimental/CPM-PDU-Descriptions.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/cpm -R -no-gen-example -fcompound-names -fno-include-deps -pdu=CPM`
 */

#ifndef	_RearOverhang_H_
#define	_RearOverhang_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RearOverhang {
	RearOverhang_zeroPointOneMeter	= 1,
	RearOverhang_oneMeter	= 10
} e_RearOverhang;

/* RearOverhang */
typedef long	 RearOverhang_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_RearOverhang_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_RearOverhang;
asn_struct_free_f RearOverhang_free;
asn_struct_print_f RearOverhang_print;
asn_constr_check_f RearOverhang_constraint;
ber_type_decoder_f RearOverhang_decode_ber;
der_type_encoder_f RearOverhang_encode_der;
xer_type_decoder_f RearOverhang_decode_xer;
xer_type_encoder_f RearOverhang_encode_xer;
oer_type_decoder_f RearOverhang_decode_oer;
oer_type_encoder_f RearOverhang_encode_oer;
per_type_decoder_f RearOverhang_decode_uper;
per_type_encoder_f RearOverhang_encode_uper;
per_type_decoder_f RearOverhang_decode_aper;
per_type_encoder_f RearOverhang_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _RearOverhang_H_ */
#include <asn_internal.h>
